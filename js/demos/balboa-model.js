/* Pololu Balboa 32U4 self-balancing robot as simulated in BalancingRobot-IsaacLab, driven by its frozen policies.
 *
 *   position policy, 50 Hz:  9 observations -> (v_x, w_z) command              rl_control/position_env_cfg.py
 *   velocity policy, 50 Hz:  8 observations -> common and turning wheel torque  rl_control/velocity_env_cfg.py
 *   wheels, 200 Hz:          torque clipped by the motor's torque-speed line     car_cfg.py (DCMotorCfg)
 *   sensing:                 raw IMU (LSM6DS33 noise) -> complementary filter -> pitch; encoders + gyro -> speed
 *
 * Constants from car_cfg.py and balboa.urdf; the equations of motion (rolling without slip, full body inertia) were
 * derived from the URDF with Kane's method. Generalised speeds u = [pitch rate, left and right joint speeds relative to
 * the body]. Pure logic, no DOM: also runs under Node for testing. */
(function (root) {
  const P = {
    physicsDt: 1 / 200, decimation: 4, substeps: 2,
    r: 0.04, halfTrack: 0.0535,
    stallTorque: 0.74 * 0.0980665 * (49 / 17), noLoadSpeed: (650 * 2 * Math.PI) / 60 / (49 / 17),
    armature: 7e-9 * ((3344 / 65) * (49 / 17)) ** 2,
    speedMax: 0.4, yawRateMax: 2.0, turnShare: 0.2, goalObsMax: 2.0,
    filterTau: 1.0, accelTau: 0.05,
    accelNoise: 90e-6 * 9.80665 * Math.sqrt(52), gyroNoise: ((7e-3 * Math.PI) / 180) * Math.sqrt(52),
    imuPos: [-0.010207, 0.000746, 0.025776],
    R: [[0, 0.172939702, -0.984932414], [-1, 0, 0], [0, 0.984932414, 0.172939702]], // v_car = R v_imu
    fallTilt: 0.8,
  };

  const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
  const wrap = (a) => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));
  const gauss = () => Math.sqrt(-2 * Math.log(1 - Math.random())) * Math.cos(2 * Math.PI * Math.random());
  const mulRT = (v) => [0, 1, 2].map((j) => P.R[0][j] * v[0] + P.R[1][j] * v[1] + P.R[2][j] * v[2]); // car -> imu
  const mulR = (v) => P.R.map((row) => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]); // imu -> car
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

  // M(th) u' = f(th, u, torques); u = [pitch rate, left joint speed, right joint speed]
  function accel(th, u1, u2, u3, tL, tR) {
    const S = Math.sin(th), C = Math.cos(th), S2 = Math.sin(2 * th), C2 = Math.cos(2 * th);
    const m00 = 4.29743e-4 * C + 8.51259e-4;
    const m01 = -1.59467e-8 * S + 1.0822e-4 * C + 2.63867e-4;
    const m02 = 1.59467e-8 * S + 1.06652e-4 * C + 2.63184e-4;
    const m11 = 4.28308e-6 * S2 - 1.7701e-5 * C2 + 2.01878e-4 + P.armature;
    const m12 = -4.28308e-6 * S2 + 1.7701e-5 * C2 + 6.19893e-5;
    const m22 = 4.28308e-6 * S2 - 1.7701e-5 * C2 + 2.01195e-4 + P.armature;
    const a = 3.00283e-5 * S, b = 1.7701e-5 * S2 + 4.28308e-6 * C2, b2 = 3.5402e-5 * S2 + 8.56616e-6 * C2;
    const f0 = 2.14871e-4 * u1 * u1 * S + (a + b) * (u2 * u2 + u3 * u3) - (2 * a + b2) * u2 * u3 + 0.0526972 * S;
    const f1 = tL + 1.0822e-4 * u1 * u1 * S + 1.59467e-8 * u1 * u1 * C - (a + b2) * u1 * u2 + (a + b2) * u1 * u3 - a * u2 * u3 + a * u3 * u3;
    const f2 = tR + 1.06652e-4 * u1 * u1 * S - 1.59467e-8 * u1 * u1 * C + (a + b2) * u1 * u2 - (a + b2) * u1 * u3 + a * u2 * u2 - a * u2 * u3;
    return solve3([[m00, m01, m02], [m01, m11, m12], [m02, m12, m22]], [f0, f1, f2]);
  }

  function solve3(m, f) {
    const det = (a) => a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    const d = det(m);
    return [0, 1, 2].map((k) => det(m.map((row, i) => row.map((v, j) => (j === k ? f[i] : v)))) / d);
  }

  class BalboaSim {
    constructor(velocityPolicy, positionPolicy) {
      this.velocityPolicy = velocityPolicy;
      this.positionPolicy = positionPolicy;
      this.noise = true;
      this.reset();
    }

    reset(pitch = 0) {
      // state: x, y, heading, pitch, pitch rate, left and right joint speeds (relative to the body)
      this.s = { x: 0, y: 0, psi: 0, th: pitch, u1: 0, u2: 0, u3: 0 };
      this.g = mulRT([Math.sin(pitch), 0, -Math.cos(pitch)]); // filter seeded from the true gravity, IMU frame
      this.imu = { pitch, pitchRate: 0, yawRate: 0 };
      this.speedPrev = 0;
      this.accelLp = 0;
      this.imuVelPrev = this.imuVelocity();
      this.sample = { accel: mulRT([0, 0, 9.81]), gyro: [0, 0, 0] };
      this.velocityLast = [0, 0];
      this.positionLast = [0, 0];
      this.command = [0, 0]; // (v_x, w_z) given to the velocity policy
      this.goal = null; // { x, y, heading } in the world: the position policy drives there; null = velocity mode
      this.drive = [0, 0]; // velocity command when there is no goal
      this.torque = [0, 0];
      this.t = 0;
      this.fallen = false;
    }

    /** One policy step (20 ms): IMU filter, policies, then 4 physics steps. */
    step() {
      this.filterImu();
      if (this.goal) {
        const a = this.positionPolicy.act([this.imu.pitch, this.imu.pitchRate, this.speed(), this.imu.yawRate, ...this.goalInCarFrame(), ...this.positionLast]);
        this.positionLast = a.map((v) => clamp(v, -1, 1));
        this.command = [this.positionLast[0] * P.speedMax, this.positionLast[1] * P.yawRateMax];
      } else {
        this.command = [clamp(this.drive[0], -P.speedMax, P.speedMax), clamp(this.drive[1], -P.yawRateMax, P.yawRateMax)];
      }
      const a = this.velocityPolicy.act([this.imu.pitch, this.imu.pitchRate, this.speed(), this.imu.yawRate, ...this.command, ...this.velocityLast]);
      this.velocityLast = a.map((v) => clamp(v, -1, 1));
      const [common, turn] = [this.velocityLast[0], P.turnShare * this.velocityLast[1]];
      this.torque = [clamp(common - turn, -1, 1) * P.stallTorque, clamp(common + turn, -1, 1) * P.stallTorque];
      for (let k = 0; k < P.decimation; k++) this.physicsStep();
      if (Math.abs(this.s.th) > P.fallTilt) this.fallen = true;
    }

    speed() {
      return P.r * ((this.s.u2 + this.s.u3) / 2 + this.imu.pitchRate);
    }

    goalInCarFrame() {
      const dx = this.goal.x - this.s.x, dy = this.goal.y - this.s.y, c = Math.cos(this.s.psi), s = Math.sin(this.s.psi);
      let gx = c * dx + s * dy, gy = -s * dx + c * dy;
      const scale = Math.min(1, P.goalObsMax / Math.max(Math.hypot(gx, gy), 1e-6));
      return [gx * scale, gy * scale, wrap(this.goal.heading - this.s.psi)];
    }

    // DCMotor: the commanded torque clipped by the torque-speed line at the joint speed
    clipTorque(tau, w) {
      const hi = clamp(P.stallTorque * (1 - w / P.noLoadSpeed), 0, P.stallTorque);
      const lo = clamp(P.stallTorque * (-1 - w / P.noLoadSpeed), -P.stallTorque, 0);
      return clamp(tau, lo, hi);
    }

    physicsStep() {
      const dt = P.physicsDt, h = dt / P.substeps;
      const tL = this.clipTorque(this.torque[0], this.s.u2), tR = this.clipTorque(this.torque[1], this.s.u3);
      for (let i = 0; i < P.substeps; i++) this.integrate(h, tL, tR);
      this.t += dt;
      // IMU sample: specific force from the finite difference of the sensor velocity, like Isaac Lab's IMU
      const v = this.imuVelocity();
      const aWorld = [0, 1, 2].map((k) => (v[k] - this.imuVelPrev[k]) / dt + (k === 2 ? 9.81 : 0));
      this.imuVelPrev = v;
      this.sample = { accel: mulRT(this.worldToCar(aWorld)), gyro: mulRT(this.bodyRates()) };
    }

    integrate(h, tL, tR) {
      const f = (s) => {
        const [a1, a2, a3] = accel(s.th, s.u1, s.u2, s.u3, tL, tR);
        const v = P.r * (s.u1 + (s.u2 + s.u3) / 2), wz = (P.r * (s.u3 - s.u2)) / (2 * P.halfTrack);
        return { x: v * Math.cos(s.psi), y: v * Math.sin(s.psi), psi: wz, th: s.u1, u1: a1, u2: a2, u3: a3 };
      };
      const add = (s, k, c) => Object.fromEntries(Object.keys(s).map((n) => [n, s[n] + c * k[n]]));
      const k1 = f(this.s), k2 = f(add(this.s, k1, h / 2)), k3 = f(add(this.s, k2, h / 2)), k4 = f(add(this.s, k3, h));
      this.s = Object.fromEntries(Object.keys(this.s).map((n) => [n, this.s[n] + (h / 6) * (k1[n] + 2 * k2[n] + 2 * k3[n] + k4[n])]));
    }

    // car (body) frame: x forward, y left, z up, pitched by th about y, yawed by psi
    bodyRates() {
      const wz = (P.r * (this.s.u3 - this.s.u2)) / (2 * P.halfTrack), th = this.s.th;
      return [-Math.sin(th) * wz, this.s.u1, Math.cos(th) * wz];
    }

    worldToCar([X, Y, Z]) {
      const c = Math.cos(this.s.psi), s = Math.sin(this.s.psi), ct = Math.cos(this.s.th), st = Math.sin(this.s.th);
      const ax = c * X + s * Y, ay = -s * X + c * Y; // yaw frame
      return [ct * ax - st * Z, ay, st * ax + ct * Z];
    }

    carToWorld([bx, by, bz]) {
      const c = Math.cos(this.s.psi), s = Math.sin(this.s.psi), ct = Math.cos(this.s.th), st = Math.sin(this.s.th);
      const ax = ct * bx + st * bz, az = -st * bx + ct * bz;
      return [c * ax - s * by, s * ax + c * by, az];
    }

    imuVelocity() {
      const v = P.r * (this.s.u1 + (this.s.u2 + this.s.u3) / 2), wz = (P.r * (this.s.u3 - this.s.u2)) / (2 * P.halfTrack);
      const w = this.carToWorld(this.bodyRates()), rp = this.carToWorld(P.imuPos);
      const wxr = cross(w, rp);
      return [v * Math.cos(this.s.psi) + wxr[0], v * Math.sin(this.s.psi) + wxr[1], wxr[2]];
    }

    /** ImuPitchAndRate: noisy samples -> acceleration-compensated complementary filter -> pitch, rates (car frame). */
    filterImu() {
      const dt = P.physicsDt * P.decimation, n = this.noise ? 1 : 0;
      const accel = this.sample.accel.map((v) => v + n * gauss() * P.accelNoise);
      const gyro = this.sample.gyro.map((v) => v + n * gauss() * P.gyroNoise);
      const gyroCar = mulR(gyro);
      const speed = P.r * ((this.s.u2 + this.s.u3) / 2 + gyroCar[1]);
      const ka = P.accelTau / (P.accelTau + dt);
      this.accelLp = ka * this.accelLp + (1 - ka) * ((speed - this.speedPrev) / dt);
      this.speedPrev = speed;
      const f = accel.map((v, i) => v - this.accelLp * P.R[0][i]);
      const wxg = cross(gyro, this.g);
      const pred = this.g.map((v, i) => v - wxg[i] * dt);
      const norm = Math.hypot(...f), alpha = P.filterTau / (P.filterTau + dt);
      const gNew = pred.map((v, i) => alpha * v + (1 - alpha) * (-f[i] / norm));
      const len = Math.hypot(...gNew);
      this.g = gNew.map((v) => v / len);
      const gCar = mulR(this.g);
      this.imu = { pitch: Math.atan2(gCar[0], -gCar[2]), pitchRate: gyroCar[1], yawRate: gyroCar[2] };
    }

    push(dv) {
      // a shove on the body: changes the pitch rate (and the wheels roll with it)
      this.s.u1 += dv / 0.03;
      this.s.u2 -= dv / 0.03;
      this.s.u3 -= dv / 0.03;
    }

    get state() {
      const v = P.r * (this.s.u1 + (this.s.u2 + this.s.u3) / 2), wz = (P.r * (this.s.u3 - this.s.u2)) / (2 * P.halfTrack);
      return { x: this.s.x, y: this.s.y, heading: this.s.psi, pitch: this.s.th, pitchEstimate: this.imu.pitch, speed: v, yawRate: wz,
        command: this.command, torque: this.torque, t: this.t, fallen: this.fallen };
    }
  }

  root.BalboaSim = BalboaSim;
  root.BalboaParams = P;
})(typeof window !== "undefined" ? window : globalThis);
