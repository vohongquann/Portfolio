/* Live demo: planar (x-z) Crazyflie flown by the same four-layer PID cascade as Drone_RL
 * (position P -> velocity PI -> attitude P -> rate PD, gains from guide/04_pid_cascade.md). */
class PID {
  constructor({ kp, ki = 0, kd = 0, intLimit = Infinity, outLimit = Infinity }) {
    Object.assign(this, { kp, ki, kd, intLimit, outLimit });
    this.reset();
  }

  reset() {
    this.integral = 0;
    this.prev = null;
  }

  update(error, dt) {
    this.integral = clamp(this.integral + error * dt, -this.intLimit, this.intLimit);
    const deriv = this.prev === null ? 0 : (error - this.prev) / dt; // no derivative kick on first call
    this.prev = error;
    return clamp(this.kp * error + this.ki * this.integral + this.kd * deriv, -this.outLimit, this.outLimit);
  }
}

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

class DroneSim {
  static P = {
    m: 0.034, J: 2.5e-5, arm: 0.035, g: 9.81, // 34 g, 100 mm X-frame (moment arm d·cos45°)
    fMax: 0.4, motorLag: 0.03, // per side (2 motors x 0.2 N), first-order lag (s)
    dt: 1 / 500, // physics and every PID layer at 500 Hz
  };

  constructor() {
    this.pos = { x: new PID({ kp: 3.9, outLimit: 1.6 }), z: new PID({ kp: 6.8, outLimit: 1.6 }) };
    this.vel = {
      x: new PID({ kp: 9.3, ki: 0.9, intLimit: 2, outLimit: 7 }),
      z: new PID({ kp: 20, ki: 1.9, intLimit: 2, outLimit: 6 }),
    };
    this.att = new PID({ kp: 17.3, outLimit: 6 });
    this.rate = new PID({ kp: 28.4, kd: 0.3 });
    this.reset();
  }

  reset(x = -1, z = 0.05) {
    Object.assign(this, { x, z, vx: 0, vz: 0, phi: 0, w: 0, fL: 0, fR: 0, T: 0 });
    this.target = { x, z: 0.6 };
    [this.pos.x, this.pos.z, this.vel.x, this.vel.z, this.att, this.rate].forEach((p) => p.reset());
  }

  control() {
    const { m, J, arm, g, fMax, dt } = DroneSim.P;
    // position -> wanted velocity -> wanted acceleration
    const vx = this.pos.x.update(this.target.x - this.x, dt);
    const vz = this.pos.z.update(this.target.z - this.z, dt);
    const ax = this.vel.x.update(vx - this.vx, dt);
    const az = this.vel.z.update(vz - this.vz, dt);
    // attitude: thrust direction that produces (ax, az + g), thrust projected on body axis
    const fx = ax, fz = az + g;
    const phiWanted = Math.atan2(fx, fz);
    this.T = m * (fx * Math.sin(this.phi) + fz * Math.cos(this.phi));
    const wWanted = this.att.update(phiWanted - this.phi, dt);
    // rate -> torque, then mixer inverse to the two motor pairs
    const tau = J * this.rate.update(wWanted - this.w, dt);
    return [clamp(this.T / 2 - tau / (2 * arm), 0, fMax), clamp(this.T / 2 + tau / (2 * arm), 0, fMax)];
  }

  step() {
    const { m, J, arm, g, motorLag, dt } = DroneSim.P;
    const [cL, cR] = this.control();
    this.fL += ((cL - this.fL) * dt) / motorLag;
    this.fR += ((cR - this.fR) * dt) / motorLag;
    const thrust = this.fL + this.fR;
    this.vx += ((thrust * Math.sin(this.phi)) / m) * dt;
    this.vz += ((thrust * Math.cos(this.phi)) / m - g) * dt;
    this.w += (((this.fR - this.fL) * arm) / J) * dt;
    this.x += this.vx * dt;
    this.z += this.vz * dt;
    this.phi += this.w * dt;
    if (this.z < 0) Object.assign(this, { z: 0, vz: Math.max(0, this.vz), vx: this.vx * 0.5 }); // ground
  }

  gust(strength) {
    this.vx += strength;
    this.w += strength * 6;
  }
}

Demos.drone = (root) => {
  const sim = new DroneSim();
  const WORLD = { x: 1.6, zMax: 1.5 }; // visible half-width and height (m)
  const SIZE = 2.5; // drone drawn larger than life so it reads at small sizes
  const trail = [];
  let mode = "point";
  let clock = 0;
  let acc = 0;
  let view = null;

  const routes = {
    point: () => {},
    square: (t) => {
      const corners = [[-0.8, 0.4], [0.8, 0.4], [0.8, 1.1], [-0.8, 1.1]];
      const [x, z] = corners[Math.floor(t / 2.5) % 4];
      sim.target = { x, z };
    },
    eight: (t) => {
      const w = (2 * Math.PI) / 8;
      sim.target = { x: 1.0 * Math.sin(w * t), z: 0.75 + 0.3 * Math.sin(2 * w * t) };
    },
  };

  function update(dt) {
    acc += dt;
    while (acc >= DroneSim.P.dt) {
      acc -= DroneSim.P.dt;
      clock += DroneSim.P.dt;
      routes[mode](clock);
      sim.step();
    }
    trail.push([sim.x, sim.z]);
    if (trail.length > 180) trail.shift();
  }

  function draw(ctx, w, h, c) {
    const s = Math.min(w / (2 * WORLD.x + 0.2), (h - 40) / (WORLD.zMax + 0.15));
    const gy = h - 36;
    const P = (x, z) => [w / 2 + x * s, gy - z * s];
    view = { s, gy, w };

    // grid + ground
    ctx.strokeStyle = c.grid;
    ctx.lineWidth = 1;
    for (let x = -1.5; x <= 1.5001; x += 0.5) line(ctx, P(x, 0), P(x, WORLD.zMax));
    for (let z = 0.5; z <= WORLD.zMax; z += 0.5) line(ctx, P(-WORLD.x, z), P(WORLD.x, z));
    ctx.strokeStyle = c.muted;
    ctx.lineWidth = 2;
    line(ctx, P(-WORLD.x - 1, 0), P(WORLD.x + 1, 0));

    // target
    const [tx, tz] = P(sim.target.x, sim.target.z);
    ctx.strokeStyle = c.c2;
    ctx.lineWidth = 1.5;
    line(ctx, [tx - 9, tz], [tx + 9, tz]);
    line(ctx, [tx, tz - 9], [tx, tz + 9]);
    ctx.beginPath();
    ctx.arc(tx, tz, 14, 0, Math.PI * 2);
    ctx.stroke();

    // trail
    ctx.strokeStyle = c.accent;
    ctx.lineWidth = 2;
    ctx.globalAlpha = 0.5;
    ctx.beginPath();
    trail.forEach(([x, z], i) => (i ? ctx.lineTo(...P(x, z)) : ctx.moveTo(...P(x, z))));
    ctx.stroke();
    ctx.globalAlpha = 1;

    drawDrone(ctx, P(sim.x, sim.z), s * SIZE, c);
    drawHud(ctx, w, c);
  }

  function drawDrone(ctx, [x, y], s, c) {
    const half = 0.05 * s;
    const { fMax } = DroneSim.P;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(sim.phi);
    // thrust plumes, length proportional to each motor pair's force
    [[-half, sim.fL], [half, sim.fR]].forEach(([px, f]) => {
      const len = (f / fMax) * 0.16 * s;
      const g = ctx.createLinearGradient(0, -4, 0, len);
      g.addColorStop(0, c.accent);
      g.addColorStop(1, "transparent");
      ctx.fillStyle = g;
      ctx.globalAlpha = 0.35;
      ctx.beginPath();
      ctx.moveTo(px - 0.02 * s, -6);
      ctx.lineTo(px + 0.02 * s, -6);
      ctx.lineTo(px, len);
      ctx.fill();
      ctx.globalAlpha = 1;
    });
    ctx.fillStyle = c.text;
    roundRect(ctx, -half, -3, 2 * half, 6, 3);
    ctx.fill();
    roundRect(ctx, -0.014 * s, -7, 0.028 * s, 10, 4);
    ctx.fill();
    ctx.strokeStyle = c.muted;
    ctx.lineWidth = 3;
    [-half, half].forEach((px) => line(ctx, [px - 0.022 * s, -7], [px + 0.022 * s, -7]));
    ctx.restore();
  }

  function drawHud(ctx, w, c) {
    const label = { point: "TAP TO SET TARGET", square: "SQUARE ROUTE", eight: "FIGURE-8 ROUTE" }[mode];
    badge(ctx, u(14), u(14), label, c.c2, c);
    ctx.font = mono(12);
    ctx.fillStyle = c.muted;
    ctx.textAlign = "right";
    const speed = Math.hypot(sim.vx, sim.vz);
    ctx.fillText(`z ${sim.z.toFixed(2)} m`, w - u(14), u(26));
    ctx.fillText(`|v| ${speed.toFixed(2)} m/s`, w - u(14), u(44));
    ctx.fillText(`pitch ${((sim.phi * 180) / Math.PI).toFixed(1).padStart(5)}°`, w - u(14), u(62));
    ctx.fillText(`T ${(sim.T * 1000).toFixed(0)} mN`, w - u(14), u(80));
    ctx.textAlign = "left";
  }

  /* ---- controls ---- */
  const canvas = root.querySelector("canvas");
  const setMode = (m) => {
    mode = m;
    clock = 0;
    root.querySelectorAll("[data-route]").forEach((b) => b.setAttribute("aria-pressed", b.dataset.route === m));
  };

  canvas.addEventListener("pointerdown", (e) => {
    if (!view) return;
    const r = canvas.getBoundingClientRect();
    setMode("point");
    sim.target = {
      x: clamp((e.clientX - r.left - view.w / 2) / view.s, -WORLD.x + 0.1, WORLD.x - 0.1),
      z: clamp((view.gy - (e.clientY - r.top)) / view.s, 0.15, WORLD.zMax - 0.1),
    };
  });
  root.querySelectorAll("[data-route]").forEach((b) => b.addEventListener("click", () => setMode(b.dataset.route)));
  root.querySelector("[data-action=gust]").addEventListener("click", () => sim.gust(sim.x > 0 ? -1.5 : 1.5)); // push towards the centre

  setMode("square");
  new Stage(canvas, { update, draw });
};
