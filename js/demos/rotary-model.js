/* Rotary inverted pendulum (Quanser QUBE-Servo 2) as simulated in RotaryPendulum-IsaacLab, plus its trained policy.
 *
 *   policy, 100 Hz:     12 observations -> 6 gains (Kp, Ki, Kd of both loops)            mdp/actions.py, CascadePidAction
 *   encoders, 500 Hz:   2048 counts/rev, velocity through the 50 s / (s + 50) filter      mdp/sensors.py
 *   outer PID, 250 Hz:  arm error -> pendulum setpoint (lean cut at +-10 deg)
 *   inner PID, 500 Hz:  pendulum error -> voltage, cut at +-12 V -> 16-bit PWM
 *   motor, 500 Hz:      PWM -> voltage -> current (exact over a step, back-EMF) -> torque   mdp/motor.py
 *
 * Every constant is the repository's (rotary_cfg.py, mdp/actions.py). The equations of motion come from the inertial
 * data of its URDF (qube_servo2.urdf, built from the CAD), derived with full inertia tensors. q1 = arm angle,
 * q2 = pendulum angle (0 hanging, +-pi upright). Pure logic, no DOM: also runs under Node for testing. */
(function (root) {
  const P = {
    physicsDt: 0.002, decimation: 5, outerEvery: 2, substeps: 4,
    R: 8.4, L: 1.16e-3, kt: 0.042, ke: 0.042, Jm: 4e-6, vMax: 12, pwmMax: 65535,
    cpr: 2048, velocityCutoff: 50,
    armDamping: 2.75e-4, pendulumDamping: 5.05e-5,
    armLength: 0.0859, pendulumLength: 0.129, pivotHeight: 0.0165,
    tiltMax: (10 * Math.PI) / 180,
    gainMax: [1.0, 1.0, 0.1, 100.0, 100.0, 10.0],
  };

  const wrap = (a) => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));
  const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
  const quantize = (a) => Math.round(a / ((2 * Math.PI) / P.cpr)) * ((2 * Math.PI) / P.cpr);

  // M(q) qdd + h(q, qd) = [tau - b1 qd1, -b2 qd2]; coefficients from the URDF (armature on the arm joint).
  function accel(q2, d1, d2, tau) {
    const s = Math.sin(q2), c = Math.cos(q2);
    const m11 = 2.26612e-4 + P.Jm + 1.13104e-4 * s * s, m12 = 1.2025e-4 * c, m22 = 1.13428e-4;
    const h1 = 2.26208e-4 * d1 * d2 * s * c - 1.2025e-4 * d2 * d2 * s;
    const h2 = -1.13104e-4 * d1 * d1 * s * c + 0.013733 * s;
    const r1 = tau - P.armDamping * d1 - h1, r2 = -P.pendulumDamping * d2 - h2;
    const det = m11 * m22 - m12 * m12;
    return [(m22 * r1 - m12 * r2) / det, (m11 * r2 - m12 * r1) / det];
  }

  function rk4([q1, q2, d1, d2], tau, h) {
    const f = (x) => [x[2], x[3], ...accel(x[1], x[2], x[3], tau)];
    const add = (x, k, s) => x.map((v, i) => v + s * k[i]);
    const x = [q1, q2, d1, d2];
    const k1 = f(x), k2 = f(add(x, k1, h / 2)), k3 = f(add(x, k2, h / 2)), k4 = f(add(x, k3, h));
    return x.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }

  class RotarySim {
    constructor(policy) {
      this.policy = policy;
      this.tick = 0; // physics steps since the start, never reset (the firmware's timer)
      this.reset();
    }

    reset(q2 = 0) {
      this.x = [0, q2, 0, 0];
      this.command = 0;
      this.enc = { angle: [quantize(0), quantize(q2)], velocity: [0, 0] };
      this.raw = [0, 0, 0, 0, 0, 0];
      this.gains = [0, 0, 0, 0, 0, 0];
      this.armIntegral = 0;
      this.pendulumIntegral = 0;
      this.setpoint = Math.PI;
      this.voltage = 0;
      this.pwm = 0;
      this.current = 0;
      this.t = 0;
    }

    observation() {
      const e = this.enc;
      return [wrap(e.angle[0] - this.command), this.command, Math.sin(e.angle[1]), Math.cos(e.angle[1]),
        e.velocity[0], e.velocity[1], ...this.raw];
    }

    /** One policy period (10 ms): new gains, then 5 physics steps of encoders, PIDs and motor. */
    step() {
      this.raw = this.policy.act(this.observation());
      this.gains = this.raw.map((a, i) => clamp(a, -1, 1) * P.gainMax[i]);
      for (let k = 0; k < P.decimation; k++) this.physicsStep();
    }

    physicsStep() {
      const dt = P.physicsDt;
      this.readEncoders(dt);
      if (this.tick % P.outerEvery === 0) this.armPid(dt * P.outerEvery);
      this.pendulumPid(dt);
      // motor: exact current over the step with the PWM and the arm speed held
      const v = (this.pwm / P.pwmMax) * P.vMax;
      const steady = (v - P.ke * this.x[2]) / P.R;
      this.current = steady + (this.current - steady) * Math.exp((-dt * P.R) / P.L);
      const tau = P.kt * this.current;
      for (let i = 0; i < P.substeps; i++) this.x = rk4(this.x, tau, dt / P.substeps);
      this.tick++;
      this.t += dt;
    }

    readEncoders(dt) {
      const alpha = dt / (dt + 1 / P.velocityCutoff);
      for (let j = 0; j < 2; j++) {
        const a = quantize(this.x[j]);
        this.enc.velocity[j] += alpha * ((a - this.enc.angle[j]) / dt - this.enc.velocity[j]);
        this.enc.angle[j] = a;
      }
    }

    armPid(dt) {
      const [kp, ki, kd] = this.gains;
      const error = wrap(this.enc.angle[0] - this.command);
      const limit = P.tiltMax / P.gainMax[1];
      this.armIntegral = clamp(this.armIntegral + error * dt, -limit, limit);
      const lean = kp * error + ki * this.armIntegral + kd * this.enc.velocity[0];
      this.setpoint = Math.PI + clamp(lean, -P.tiltMax, P.tiltMax);
    }

    pendulumPid(dt) {
      const [, , , kp, ki, kd] = this.gains;
      const error = wrap(this.enc.angle[1] - this.setpoint);
      const limit = P.vMax / P.gainMax[4];
      this.pendulumIntegral = clamp(this.pendulumIntegral + error * dt, -limit, limit);
      const v = -(kp * error + ki * this.pendulumIntegral + kd * this.enc.velocity[1]);
      this.voltage = clamp(v, -P.vMax, P.vMax);
      this.pwm = Math.round(clamp(this.voltage / P.vMax, -1, 1) * P.pwmMax);
    }

    /** Motor off, both joints damped: brings the robot to rest between two runs of the demo. */
    coast(dt = 0.01, damping = 0.9) {
      this.voltage = 0;
      this.current = 0;
      for (let i = 0; i < 5; i++) this.x = rk4(this.x, 0, dt / 5);
      this.x[2] *= damping;
      this.x[3] *= Math.sqrt(damping);
      this.t += dt;
    }

    kick(pendulumSpeed) {
      this.x[3] += pendulumSpeed;
    }

    /** Arm tip and pendulum end in the robot frame (x, y horizontal, z up; the arm pivot at the origin). */
    geometry(armOffset = 0) {
      const q1 = this.x[0] + armOffset, q2 = this.x[1];
      const ax = -Math.cos(q1), ay = -Math.sin(q1); // the arm points along -x of its frame
      const tip = [P.armLength * ax, P.armLength * ay, P.pivotHeight];
      // the pendulum turns about the arm axis: hanging along -z at q2 = 0
      const nx = -ay, ny = ax; // horizontal, perpendicular to the arm
      const side = -Math.sin(q2) * P.pendulumLength, down = -Math.cos(q2) * P.pendulumLength;
      const bob = [tip[0] + side * nx, tip[1] + side * ny, tip[2] + down];
      return { tip, bob };
    }

    get state() {
      const up = Math.abs(wrap(this.x[1] - Math.PI));
      return { arm: this.x[0], pendulum: this.x[1], fromUpright: up, armSpeed: this.x[2], voltage: this.voltage,
        current: this.current, command: this.command, gains: this.gains, t: this.t };
    }
  }

  root.RotarySim = RotarySim;
  root.RotaryParams = P;
})(typeof window !== "undefined" ? window : globalThis);
