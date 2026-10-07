/* Live demo: the Crazyflie-IsaacLab gain cascade (crazyflie-model.js), seen from an oblique camera. The program is the
 * one of the repository's demo video: hover, a 0.8 m circle at 0.7 m/s, a figure 8. */
Demos.crazyflie = (root) => {
  const policies = Object.fromEntries(Object.entries(window.CRAZYFLIE_GAIN_POLICIES).map(([k, v]) => [k, new Policy(v)]));
  const sim = new CrazyflieSim(policies);
  const canvas = root.querySelector("canvas");
  const HEIGHT = 1.1, DRAW_SCALE = 4; // flight height (m); the drone is drawn 4x its size
  const NOMINAL_KP = { position: 3.9, velocity: 0.95, attitude: 8.6, rate: 73 }; // x-axis kp of the tuned PID
  const TRAIL = 90;
  const trail = [];
  let route = "circle";
  let routeTime = 0;
  let hoverAt = [0, 0, HEIGHT];
  let yaw = 0;
  let speed = 1;
  let acc = 0;

  const ROUTES = {
    hover: () => [hoverAt, [0, 0, 0]],
    circle: (t) => {
      const R = 0.8, V = 0.7, ph = (V / R) * t;
      return [[-R + R * Math.cos(ph), R * Math.sin(ph), HEIGHT], [-V * Math.sin(ph), V * Math.cos(ph), 0]];
    },
    eight: (t) => {
      const S = 1.0, W = 0.8, ph = W * t;
      return [[S * Math.sin(ph), 0.5 * S * Math.sin(2 * ph), HEIGHT + 0.1 * Math.sin(ph)],
        [S * W * Math.cos(ph), S * W * Math.cos(2 * ph), 0.1 * W * Math.cos(ph)]];
    },
  };

  const setRoute = (name) => {
    route = name;
    routeTime = 0;
    root.querySelectorAll("[data-route]").forEach((b) => b.setAttribute("aria-pressed", b.dataset.route === name));
  };

  sim.reset([0, 0, HEIGHT]);

  function update(dt) {
    acc += dt * speed;
    let n = 0;
    while (acc >= 0.02 && n++ < 4) {
      acc -= 0.02;
      routeTime += 0.02;
      const [p, v] = ROUTES[route](routeTime);
      if (Math.hypot(v[0], v[1]) > 0.15) { // turn the nose along the flight, at most 1.2 rad/s (as the repo's demo)
        const d = Math.atan2(v[1], v[0]) - yaw;
        yaw += Math.max(-0.024, Math.min(0.024, d - 2 * Math.PI * Math.floor((d + Math.PI) / (2 * Math.PI))));
      }
      sim.command = [...p, yaw, ...v];
      sim.step();
      if (Math.round(sim.t / 0.02) % 2 === 0) {
        trail.push(sim.p.slice());
        if (trail.length > TRAIL) trail.shift();
      }
    }
  }

  /* ---- drawing: orthographic camera from the side and above ---- */
  const YAW = -0.75, ELEV = 0.5;
  let cam;
  const project = ([x, y, z]) => {
    const x1 = x * Math.cos(YAW) - y * Math.sin(YAW);
    const y1 = x * Math.sin(YAW) + y * Math.cos(YAW);
    return [cam.cx + x1 * cam.s, cam.cy - ((z - 0.6) * Math.cos(ELEV) - y1 * Math.sin(ELEV)) * cam.s];
  };

  function draw(ctx, w, h, c) {
    cam = { s: Math.min(w / 3.4, h / 2.3), cx: w * 0.5, cy: h * 0.55 };
    drawGround(ctx, c);
    drawPath(ctx, c);
    drawDrone(ctx, c);
    drawHud(ctx, w, c);
  }

  function drawGround(ctx, c) {
    ctx.strokeStyle = c.grid;
    ctx.lineWidth = 1;
    for (let i = -1.5; i <= 1.501; i += 0.25) {
      line(ctx, project([i, -1.5, 0]), project([i, 1.5, 0]));
      line(ctx, project([-1.5, i, 0]), project([1.5, i, 0]));
    }
  }

  function drawPath(ctx, c) {
    ctx.fillStyle = c.c1; // the target's path for the next 1.5 s
    for (let k = 1; k <= 12; k++) {
      ctx.globalAlpha = 0.9 - k * 0.06;
      circle(ctx, ...project(ROUTES[route](routeTime + k * 0.12)[0]), u(2.2));
    }
    ctx.fillStyle = c.c3; // where the drone flew
    trail.forEach((p, i) => {
      ctx.globalAlpha = 0.1 + (0.6 * i) / TRAIL;
      circle(ctx, ...project(p), u(1.8));
    });
    ctx.globalAlpha = 1;
    ctx.fillStyle = c.c2;
    circle(ctx, ...project(sim.command.slice(0, 3)), u(4));
  }

  function drawDrone(ctx, c) {
    const R = sim.R, p = sim.p, hx = (0.05 / Math.SQRT2) * DRAW_SCALE;
    const world = (b) => [0, 1, 2].map((j) => p[j] + R[j][0] * b[0] + R[j][1] * b[1] + R[j][2] * b[2]);
    // shadow
    const [sx, sy] = project([p[0], p[1], 0]);
    ctx.fillStyle = c.muted;
    ctx.globalAlpha = 0.18;
    ctx.beginPath();
    ctx.ellipse(sx, sy, hx * cam.s * 1.2, hx * cam.s * 1.2 * Math.sin(ELEV), 0, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = 0.35;
    ctx.strokeStyle = c.muted;
    ctx.setLineDash([3, 4]);
    line(ctx, [sx, sy], project(p));
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    // arms, rotors (front motors in the accent colour), nose
    const motors = [[hx, -hx], [-hx, -hx], [-hx, hx], [hx, hx]].map(([x, y]) => world([x, y, 0]));
    const centre = project(p);
    ctx.strokeStyle = c.text;
    ctx.lineWidth = 2.5;
    motors.forEach((m) => line(ctx, centre, project(m)));
    motors.forEach((m, i) => {
      const [mx, my] = project(m), r = 0.022 * DRAW_SCALE * cam.s;
      const load = sim.force[i] / CrazyflieParams.fMax;
      ctx.fillStyle = i === 0 || i === 3 ? c.accent : c.muted;
      ctx.globalAlpha = 0.25 + 0.6 * load;
      ctx.beginPath();
      ctx.ellipse(mx, my, r, r * Math.sin(ELEV), 0, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.globalAlpha = 1;
    ctx.fillStyle = c.text;
    circle(ctx, ...centre, u(3.5));
  }

  function drawHud(ctx, w, c) {
    const s = sim.state;
    badge(ctx, u(14), u(14), { hover: "HOVER", circle: "CIRCLE 0.7 m/s", eight: "FIGURE 8" }[route], c.c1, c);
    const err = Math.hypot(...[0, 1, 2].map((j) => s.position[j] - s.target[j]));
    ctx.font = mono(12);
    ctx.fillStyle = c.muted;
    ctx.textAlign = "right";
    ctx.fillText(`error ${(err * 100).toFixed(1).padStart(5)} cm`, w - u(14), u(26));
    ctx.font = mono(11);
    ctx.fillText("kp written by each network", w - u(14), u(46));
    ["position", "velocity", "attitude", "rate"].forEach((name, i) => {
      const g = s.gains[name];
      if (!g) return;
      const ratio = g[0][0] / NOMINAL_KP[name];
      ctx.fillText(`${name.padEnd(8)} ×${ratio.toFixed(2)} tuned`, w - u(14), u(64 + i * 16));
    });
    ctx.textAlign = "left";
  }

  /* ---- controls ---- */
  root.querySelectorAll("[data-route]").forEach((b) => b.addEventListener("click", () => {
    if (b.dataset.route === "hover") hoverAt = sim.p.slice();
    setRoute(b.dataset.route);
  }));
  root.querySelector("[data-action=gust]").addEventListener("click", () => {
    const a = Math.random() * 2 * Math.PI;
    sim.gust([0.8 * Math.cos(a), 0.8 * Math.sin(a), 0.2]);
  });

  new Stage(canvas, { update, draw });
};
