/* Live demo: the BalancingRobot-IsaacLab position network on top of its frozen velocity network, driving a simulated
 * Pololu Balboa (balboa-model.js). Top view of the floor, with a side view of the robot to show it balancing. */
Demos.balboa = (root) => {
  const sim = new BalboaSim(new Policy(window.BALBOA_VELOCITY_POLICY), new Policy(window.BALBOA_POSITION_POLICY));
  const canvas = root.querySelector("canvas");
  const AREA = 0.8; // auto goals stay within +-AREA m of the centre
  const TRAIL = 160;
  const trail = [];
  let wheelAngle = 0; // for the spokes of the side view
  let idle = 0; // s since the last goal
  let fallenFor = 0;
  let speed = 1;
  let acc = 0;
  let view = { x: 0, y: 0, s: 1 }; // camera centre (m) and scale (px/m)

  const newGoal = (x, y) => {
    const heading = Math.atan2(y - sim.s.y, x - sim.s.x);
    sim.goal = { x, y, heading: Math.hypot(x - sim.s.x, y - sim.s.y) > 0.05 ? heading : sim.s.psi };
    idle = 0;
  };
  const randomGoal = () => {
    const r = 0.3 + Math.random() * 0.35, a = Math.random() * 2 * Math.PI;
    newGoal(Math.max(-AREA, Math.min(AREA, sim.s.x + r * Math.cos(a))), Math.max(-AREA, Math.min(AREA, sim.s.y + r * Math.sin(a))));
  };
  const restart = () => {
    const { x, y, psi } = sim.s;
    const goal = sim.goal;
    sim.reset((Math.random() - 0.5) * 0.1);
    Object.assign(sim.s, { x, y, psi });
    sim.goal = goal;
    fallenFor = 0;
  };

  sim.reset(0.05);
  newGoal(0.6, 0.3);

  function update(dt) {
    acc += dt * speed;
    let n = 0;
    while (acc >= 0.02 && n++ < 4) {
      acc -= 0.02;
      if (sim.fallen) {
        if ((fallenFor += 0.02) > 0.8) restart();
        continue;
      }
      sim.step();
      wheelAngle += ((sim.s.u2 + sim.s.u3) / 2 + sim.s.u1) * 0.02;
      if ((idle += 0.02) > 7) randomGoal();
      if (Math.round(sim.t / 0.02) % 3 === 0) {
        trail.push([sim.s.x, sim.s.y]);
        if (trail.length > TRAIL) trail.shift();
      }
    }
  }

  /* ---- drawing ---- */
  const toScreen = ([x, y]) => [view.cx + (y - view.y) * -view.s, view.cy - (x - view.x) * view.s]; // x up the screen
  const toWorld = (px, py) => [view.x + (view.cy - py) / view.s, view.y - (px - view.cx) / view.s];

  function draw(ctx, w, h, c) {
    view.s = Math.min(w, h) / 1.4;
    view.cx = w / 2;
    view.cy = h / 2;
    // follow the robot loosely
    view.x += (sim.s.x - view.x) * 0.03;
    view.y += (sim.s.y - view.y) * 0.03;
    drawFloor(ctx, w, h, c);
    drawTrail(ctx, c);
    if (sim.goal) drawGoal(ctx, c);
    drawRobotTop(ctx, c);
    drawSideView(ctx, w, h, c);
    drawHud(ctx, w, c);
  }

  function drawFloor(ctx, w, h, c) {
    ctx.strokeStyle = c.grid;
    ctx.lineWidth = 1;
    const [x0, y1] = toWorld(0, h), [x1, y0] = toWorld(w, 0);
    for (let x = Math.floor(Math.min(x0, x1) * 4) / 4; x <= Math.max(x0, x1); x += 0.25) line(ctx, toScreen([x, -9]), toScreen([x, 9]));
    for (let y = Math.floor(Math.min(y0, y1) * 4) / 4; y <= Math.max(y0, y1); y += 0.25) line(ctx, toScreen([-9, y]), toScreen([9, y]));
  }

  function drawTrail(ctx, c) {
    ctx.fillStyle = c.c3;
    trail.forEach((p, i) => {
      ctx.globalAlpha = 0.15 + (0.6 * i) / TRAIL;
      circle(ctx, ...toScreen(p), u(2));
    });
    ctx.globalAlpha = 1;
  }

  function drawGoal(ctx, c) {
    const g = sim.goal, [gx, gy] = toScreen([g.x, g.y]);
    ctx.strokeStyle = c.c2;
    ctx.fillStyle = c.c2;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(gx, gy, 0.05 * view.s, 0, Math.PI * 2);
    ctx.stroke();
    circle(ctx, gx, gy, u(3));
    line(ctx, [gx, gy], toScreen([g.x + 0.12 * Math.cos(g.heading), g.y + 0.12 * Math.sin(g.heading)]));
  }

  function drawRobotTop(ctx, c) {
    const { x, y, psi } = sim.s;
    const [px, py] = toScreen([x, y]);
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-psi); // screen y is down, world x is up: heading 0 points up the screen
    const s = view.s;
    ctx.fillStyle = c.text; // wheels
    ctx.fillRect(-0.0595 * s, -0.04 * s, 0.012 * s, 0.08 * s);
    ctx.fillRect(0.0475 * s, -0.04 * s, 0.012 * s, 0.08 * s);
    ctx.fillStyle = c.bg; // body
    ctx.strokeStyle = c.muted;
    ctx.lineWidth = 1.5;
    roundRect(ctx, -0.045 * s, -0.022 * s, 0.09 * s, 0.044 * s, 0.006 * s);
    ctx.fill();
    ctx.stroke();
    ctx.strokeStyle = c.accent; // heading
    ctx.lineWidth = 2;
    line(ctx, [0, 0], [0, -0.07 * s]);
    ctx.restore();
  }

  function drawSideView(ctx, w, h, c) {
    const bw = Math.min(u(150), w * 0.32), bh = bw * 0.9, bx = u(12), by = h - bh - u(12);
    ctx.fillStyle = c.bg;
    ctx.strokeStyle = c.grid;
    roundRect(ctx, bx, by, bw, bh, u(10));
    ctx.fill();
    ctx.stroke();
    const s = bh / 0.22, gx = bx + bw / 2, gy = by + bh - u(16);
    ctx.strokeStyle = c.muted;
    ctx.lineWidth = 1;
    line(ctx, [bx + u(8), gy], [bx + bw - u(8), gy]);
    const ax = gx, ay = gy - 0.04 * s, th = sim.s.th; // axle; pitch leans the body forward (to the right)
    ctx.save();
    ctx.translate(ax, ay);
    ctx.rotate(th);
    ctx.fillStyle = c.bg;
    ctx.strokeStyle = c.text;
    ctx.lineWidth = 1.5;
    roundRect(ctx, -0.016 * s, -0.12 * s, 0.032 * s, 0.13 * s, 0.004 * s);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
    ctx.fillStyle = c.text; // wheel
    circle(ctx, ax, ay, 0.04 * s);
    ctx.strokeStyle = c.bg;
    ctx.lineWidth = 1.5;
    for (let k = 0; k < 3; k++) {
      const a = wheelAngle + (k * 2 * Math.PI) / 3;
      line(ctx, [ax, ay], [ax + Math.sin(a) * 0.034 * s, ay - Math.cos(a) * 0.034 * s]);
    }
    ctx.font = mono(10);
    ctx.fillStyle = c.muted;
    ctx.fillText(`pitch ${((th * 180) / Math.PI).toFixed(1)}°`, bx + u(8), by + u(16));
  }

  function drawHud(ctx, w, c) {
    const st = sim.state;
    const near = sim.goal && Math.hypot(sim.goal.x - st.x, sim.goal.y - st.y) < 0.03;
    const label = sim.fallen ? "FELL, RESTARTING" : near ? "HOLDING ON GOAL" : "DRIVING TO GOAL";
    badge(ctx, u(14), u(14), label, sim.fallen ? c.warn : near ? c.c1 : c.c2, c);
    ctx.font = mono(12);
    ctx.fillStyle = c.muted;
    ctx.textAlign = "right";
    ctx.fillText(`v ${st.speed.toFixed(2).padStart(5)} m/s`, w - u(14), u(26));
    ctx.fillText(`ω ${st.yawRate.toFixed(2).padStart(5)} rad/s`, w - u(14), u(44));
    ctx.font = mono(11);
    ctx.fillText(`command → velocity net  ${st.command[0].toFixed(2)} m/s  ${st.command[1].toFixed(2)} rad/s`, w - u(14), u(64));
    ctx.textAlign = "left";
  }

  /* ---- controls ---- */
  canvas.addEventListener("pointerdown", (e) => {
    const r = canvas.getBoundingClientRect();
    const [x, y] = toWorld(e.clientX - r.left, e.clientY - r.top);
    newGoal(x, y);
  });
  root.querySelector("[data-action=push]").addEventListener("click", () => sim.push((Math.random() < 0.5 ? -1 : 1) * (0.05 + Math.random() * 0.04)));
  root.querySelector("[data-action=goal]").addEventListener("click", randomGoal);
  root.querySelector("[data-action=slow]").addEventListener("click", (e) => {
    speed = speed === 1 ? 0.25 : 1;
    e.currentTarget.setAttribute("aria-pressed", speed !== 1);
  });

  new Stage(canvas, { update, draw });
};
