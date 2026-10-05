/* Rotary inverted pendulum: plant model + trained SAC policies.
 * Plant: Furuta pendulum driven by a DC motor (params from matlab/config/params_system.m
 * and the Simscape CAD data). Policies: deterministic SAC actors, action = 12 V * tanh(mean).
 * alpha = 0 hanging down, alpha = ±pi upright. Pure logic, no DOM. */
(function (root) {
  const P = {
    R: 8.4, kt: 0.042, km: 0.042, // DC motor
    Jr: 6e-5, Lr: 0.11, // rotary arm (incl. motor + hub)
    mp: 0.0625, l: 0.0583, Jpc: 11.67e-6, // pendulum: mass, pivot->CoM, inertia about CoM
    b: 1e-7, g: 9.81,
    vMax: 12, dt: 1e-3, ctrlEvery: 10, // physics 1 kHz, policy 100 Hz (Ts = 0.01 s)
    switchAngle: 2.6, // |alpha| above this -> balance agent
  };

  const wrap = (a) => ((((a + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) - Math.PI;

  function forward(layers, x) {
    for (let i = 0; i < layers.length; i++) {
      const { W, b } = layers[i];
      const y = new Array(b.length);
      for (let r = 0; r < b.length; r++) {
        let s = b[r];
        for (let c = 0; c < x.length; c++) s += W[r][c] * x[c];
        y[r] = i < layers.length - 1 ? Math.max(0, s) : s; // ReLU on hidden layers
      }
      x = y;
    }
    return x[0];
  }

  function deriv([, al, thd, ald], V) {
    const { R, kt, km, Jr, Lr, mp, l, Jpc, b, g } = P;
    const S = Math.sin(al), C = Math.cos(al), Jp = Jpc + mp * l * l;
    const tau = (kt * (V - km * thd)) / R;
    const m11 = Jr + mp * Lr * Lr + Jp * S * S, m12 = mp * Lr * l * C, m22 = Jp;
    const r1 = tau - (2 * Jp * S * C * thd * ald - mp * Lr * l * S * ald * ald + b * thd);
    const r2 = -(-Jp * S * C * thd * thd + mp * g * l * S + b * ald);
    const det = m11 * m22 - m12 * m12;
    return [thd, ald, (m22 * r1 - m12 * r2) / det, (m11 * r2 - m12 * r1) / det];
  }

  function rk4(x, V, h) {
    const add = (a, k, s) => a.map((v, i) => v + s * k[i]);
    const k1 = deriv(x, V), k2 = deriv(add(x, k1, h / 2), V);
    const k3 = deriv(add(x, k2, h / 2), V), k4 = deriv(add(x, k3, h), V);
    return x.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }

  class PendulumSim {
    constructor(policy) {
      this.policy = policy;
      this.reset();
    }

    reset(alpha0 = 0.01) {
      this.x = [0, alpha0, 0, 0]; // theta, alpha, theta_dot, alpha_dot
      this.t = 0;
      this.V = 0;
      this.mode = "swingup";
      this.steps = 0;
    }

    control() {
      const [th, al, thd, ald] = this.x;
      this.mode = Math.abs(wrap(al)) > P.switchAngle ? "balance" : "swingup";
      const obs = [wrap(th), thd, wrap(al), ald, 0, Math.PI];
      this.V = P.vMax * Math.tanh(forward(this.policy[this.mode], obs));
    }

    step() {
      if (this.steps++ % P.ctrlEvery === 0) this.control();
      this.x = rk4(this.x, this.V, P.dt);
      this.t += P.dt;
    }

    // Motor off with heavy damping: brings the pendulum back to rest before a new episode.
    settle() {
      this.V = 0;
      this.mode = "reset";
      this.x = rk4(this.x, 0, P.dt);
      this.x[2] *= 0.995;
      this.x[3] *= 0.996;
      this.t += P.dt;
    }

    kick(alphaDot) {
      this.x[3] += alphaDot;
    }

    get state() {
      const [th, al, thd, ald] = this.x;
      return { theta: th, alpha: wrap(al), thetaDot: thd, alphaDot: ald, V: this.V, mode: this.mode, t: this.t };
    }
  }

  root.PendulumSim = PendulumSim;
  root.PendulumParams = P;
})(typeof window !== "undefined" ? window : globalThis);
