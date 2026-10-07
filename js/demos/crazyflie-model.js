/* Crazyflie 2.1 Brushless as simulated in Crazyflie-IsaacLab, flown by its gain cascade: at every layer a small trained
 * network writes the 9 PID gains (kp, ki, kd of three axes) and the PID of that layer computes the command.
 *
 *   position  50 Hz   [target, yaw, target velocity] -> wanted velocity          pid_control/position.py
 *   velocity 100 Hz   wanted velocity -> roll, pitch, thrust                     pid_control/velocity.py
 *   attitude 250 Hz   roll, pitch, yaw -> wanted body rates                      pid_control/attitude.py
 *   rate     500 Hz   body rates -> torque -> mixer inverse -> PWM               pid_control/rate.py, mdp/actions/mixer.py
 *   motors  1000 Hz   PWM -> thrust (Folk thrust-stand curve) -> wrench           mdp/actions/propulsion.py, uav_cfg.py
 *
 * Gains: gain = nominal * 3^a where the tuned PID has the term, else max * max(a, 0) (mdp/gains.py); a = 0 is the tuned
 * PID. Rigid body with the mass and inertia of uav_cfg.py, 1 ms steps. Pure logic, no DOM: also runs under Node. */
(function (root) {
  const h = 0.05 / Math.SQRT2;
  const P = {
    mass: 0.045, inertia: [2.3951e-5, 2.3951e-5, 3.2347e-5], g: 9.81,
    motorXY: [[h, -h], [-h, -h], [-h, h], [h, h]], spin: [-1, 1, -1, 1], km: 7.73e-11 / 3.72e-8,
    thrustCoef: [2.0247478161264422e-8, 0.0017784193532727984, -12.46265354570185], fMax: 0.46430910662037017, pwmMax: 65535,
    hoverThrottle: 0.3833738125541706, physicsDt: 0.001,
  };
  P.weight = P.mass * P.g;

  const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
  const wrap = (a) => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

  // (kind, axis) tuned gains of each layer, integral limits, output limits
  const LAYERS = {
    rate: { period: 2, kp: [73, 73, 9.7], ki: [0, 0, 0], kd: [0.27, 0.27, 0.14], intLimit: [0.5, 0.5, 0.5], outLimit: null },
    attitude: { period: 4, kp: [8.6, 8.6, 4], ki: [0, 0, 0], kd: [0, 0, 0], intLimit: [0.3, 0.3, 0.3], outLimit: [6, 6, 3] },
    velocity: { period: 10, kp: [0.95, 0.95, 0.9], ki: [0.092, 0.092, 0.086], kd: [0, 0, 0], intLimit: [2, 2, 1], outLimit: [0.6, 0.6, 0.27] },
    position: { period: 20, kp: [3.9, 3.9, 6.8], ki: [0, 0, 0], kd: [0, 0, 0], intLimit: [0.5, 0.5, 0.5], outLimit: [1.6, 1.6, 1.6] },
  };

  /** Network output (9) in [-1, 1] -> kp, ki, kd (3 each). */
  function gains(layer, a) {
    const L = LAYERS[layer], nominal = [L.kp, L.ki, L.kd], maximum = [[0, 0, 0], L.kp.map((k) => 0.3 * k), L.kp.map((k) => 0.05 * k)];
    return [0, 1, 2].map((kind) => [0, 1, 2].map((j) => {
      const x = clamp(a[kind * 3 + j], -1, 1);
      return nominal[kind][j] > 0 ? nominal[kind][j] * 3 ** x : maximum[kind][j] * Math.max(x, 0);
    }));
  }

  class PID {
    constructor(L) { this.L = L; this.reset(); }
    reset() { this.integral = null; this.prev = null; }
    update(error, dt, [kp, ki, kd]) {
      if (!this.integral) { this.integral = [0, 0, 0]; this.prev = error.slice(); }
      return error.map((e, j) => {
        this.integral[j] = clamp(this.integral[j] + e * dt, -this.L.intLimit[j], this.L.intLimit[j]);
        const d = (e - this.prev[j]) / dt;
        this.prev[j] = e;
        const out = kp[j] * e + ki[j] * this.integral[j] + kd[j] * d;
        return this.L.outLimit ? clamp(out, -this.L.outLimit[j], this.L.outLimit[j]) : out;
      });
    }
  }

  function invert4(m) {
    const n = 4, a = m.map((row, i) => [...row, ...Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))]);
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
      [a[c], a[p]] = [a[p], a[c]];
      const d = a[c][c];
      for (let j = 0; j < 2 * n; j++) a[c][j] /= d;
      for (let r = 0; r < n; r++) if (r !== c) { const f = a[r][c]; for (let j = 0; j < 2 * n; j++) a[r][j] -= f * a[c][j]; }
    }
    return a.map((row) => row.slice(n));
  }
  const ALLOCATION_INVERSE = invert4([[1, 1, 1, 1], P.motorXY.map((m) => m[1]), P.motorXY.map((m) => -m[0]), P.spin.map((s) => P.km * s)]);

  const thrustToPwm = (f) => {
    const [a, b, c] = P.thrustCoef, grams = (f * 4 * 1000) / P.g;
    return clamp((-b + Math.sqrt(Math.max(b * b - 4 * a * (c - grams), 0))) / (2 * a), 0, P.pwmMax);
  };
  const pwmToThrust = (p) => {
    const [a, b, c] = P.thrustCoef;
    return clamp((((a * p * p + b * p + c) / 4) * P.g) / 1000, 0, P.fMax);
  };

  class CrazyflieSim {
    constructor(policies) {
      this.policies = policies; // { rate, attitude, velocity, position } Policy objects
      this.reset();
    }

    reset(position = [0, 0, 1.1]) {
      this.p = position.slice();
      this.v = [0, 0, 0];
      this.q = [1, 0, 0, 0]; // w, x, y, z
      this.w = [0, 0, 0]; // body rates
      this.command = [...position, 0, 0, 0, 0]; // target (3), yaw, target velocity (3)
      this.layer = Object.fromEntries(Object.keys(LAYERS).map((n) => [n, { pid: new PID(LAYERS[n]), history: new Array(18).fill(0), command: [0, 0, 0, 0], output: [0, 0, 0, 0], gains: null }]));
      this.motor = [1, 1, 1, 1].map(() => P.hoverThrottle);
      this.force = [0, 0, 0, 0];
      this.tick = 0;
      this.t = 0;
    }

    get R() { // body -> world
      const [w, x, y, z] = this.q;
      return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
      ];
    }

    euler() { // roll, pitch, yaw of Rz Ry Rx (Isaac Lab euler_xyz_from_quat)
      const [w, x, y, z] = this.q;
      return [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), Math.asin(clamp(2 * (w * y - z * x), -1, 1)), Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))];
    }

    observe(name) {
      const L = this.layer[name], c = L.command, R = this.R;
      if (name === "rate") return [c[0] - this.w[0], c[1] - this.w[1], c[2] - this.w[2], ...this.w, c[3] / P.weight, ...L.history];
      if (name === "attitude") {
        const e = this.euler();
        return [wrap(c[0] - e[0]), wrap(c[1] - e[1]), wrap(c[2] - e[2]), c[3] / P.weight, -R[2][0], -R[2][1], -R[2][2], ...this.w, ...L.history];
      }
      if (name === "velocity") return [c[0] - this.v[0], c[1] - this.v[1], c[2] - this.v[2], ...this.v, R[0][2], R[1][2], R[2][2], ...L.history];
      return [c[0] - this.p[0], c[1] - this.p[1], c[2] - this.p[2], ...this.v, c[4], c[5], c[6], ...L.history];
    }

    /** Run one layer: network -> gains -> PID -> command of the layer below. */
    run(name, command) {
      const L = this.layer[name];
      L.command = command;
      const a = this.policies[name].act(this.observe(name)).map((x) => clamp(x, -1, 1));
      L.history = [...a, ...L.history.slice(0, 9)];
      L.gains = gains(name, a);
      const dt = LAYERS[name].period * P.physicsDt, R = this.R;
      if (name === "position") {
        const vel = L.pid.update([0, 1, 2].map((j) => command[j] - this.p[j]), dt, L.gains);
        L.output = [...vel, command[3]];
      } else if (name === "velocity") {
        const e = [0, 1, 2].map((j) => command[j] - this.v[j]), cy = Math.cos(command[3]), sy = Math.sin(command[3]);
        const out = L.pid.update([cy * e[0] + sy * e[1], -sy * e[0] + cy * e[1], e[2]], dt, L.gains);
        L.output = [-out[1], out[0], command[3], P.weight + out[2]];
      } else if (name === "attitude") {
        const [roll, pitch, yaw] = command;
        const ux = Math.sin(pitch) * Math.cos(roll), uy = -Math.sin(roll);
        const wanted = [Math.cos(yaw) * ux - Math.sin(yaw) * uy, Math.sin(yaw) * ux + Math.cos(yaw) * uy, Math.cos(pitch) * Math.cos(roll)];
        const up = [R[0][2], R[1][2], R[2][2]], tw = cross(up, wanted);
        const tb = [0, 1, 2].map((i) => R[0][i] * tw[0] + R[1][i] * tw[1] + R[2][i] * tw[2]);
        const yawNow = Math.atan2(R[1][0], R[0][0]);
        const rates = L.pid.update([tb[0], tb[1], wrap(yaw - yawNow)], dt, L.gains);
        L.output = [...rates, command[3]];
      } else {
        const acc = L.pid.update([0, 1, 2].map((j) => command[j] - this.w[j]), dt, L.gains);
        const wrench = [command[3], ...acc.map((v, j) => P.inertia[j] * v)];
        L.output = ALLOCATION_INVERSE.map((row) => thrustToPwm(row.reduce((s, m, k) => s + m * wrench[k], 0)) / P.pwmMax);
      }
    }

    /** One position-layer period (20 ms): the position network at its step, then 20 physics steps. */
    step() {
      this.run("position", this.command);
      for (let k = 0; k < LAYERS.position.period; k++) {
        let cmd = this.layer.position.output;
        for (const name of ["velocity", "attitude", "rate"]) {
          if (this.tick % LAYERS[name].period === 0) this.run(name, cmd);
          cmd = this.layer[name].output;
        }
        this.motor = cmd;
        this.physicsStep();
      }
    }

    physicsStep() {
      const dt = P.physicsDt;
      this.force = this.motor.map((m) => pwmToThrust(m * P.pwmMax));
      const T = this.force.reduce((s, f) => s + f, 0);
      const tau = [
        this.force.reduce((s, f, i) => s + f * P.motorXY[i][1], 0),
        -this.force.reduce((s, f, i) => s + f * P.motorXY[i][0], 0),
        P.km * this.force.reduce((s, f, i) => s + f * P.spin[i], 0),
      ];
      const R = this.R;
      for (let j = 0; j < 3; j++) this.v[j] += ((R[j][2] * T) / P.mass - (j === 2 ? P.g : 0)) * dt;
      for (let j = 0; j < 3; j++) this.p[j] += this.v[j] * dt;
      const Iw = this.w.map((v, j) => P.inertia[j] * v), gyro = cross(this.w, Iw);
      this.w = this.w.map((v, j) => v + ((tau[j] - gyro[j]) / P.inertia[j]) * dt);
      const [w, x, y, z] = this.q, [p, q, r] = this.w;
      const dq = [-x * p - y * q - z * r, w * p + y * r - z * q, w * q - x * r + z * p, w * r + x * q - y * p];
      const nq = this.q.map((v, i) => v + 0.5 * dq[i] * dt), n = Math.hypot(...nq);
      this.q = nq.map((v) => v / n);
      if (this.p[2] < 0.02) { this.p[2] = 0.02; this.v = this.v.map((v, j) => (j === 2 ? Math.max(v, 0) : v * 0.9)); }
      this.tick++;
      this.t += dt;
    }

    gust(dv) { this.v = this.v.map((v, j) => v + dv[j]); }

    get state() {
      return { position: this.p, velocity: this.v, rotation: this.R, euler: this.euler(), motor: this.motor, force: this.force,
        target: this.command.slice(0, 3), gains: Object.fromEntries(Object.entries(this.layer).map(([n, L]) => [n, L.gains])), t: this.t };
    }
  }

  root.CrazyflieSim = CrazyflieSim;
  root.CrazyflieParams = P;
})(typeof window !== "undefined" ? window : globalThis);
