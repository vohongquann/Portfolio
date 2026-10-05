/* Live demo: the trained SAC swing-up and balance agents driving a simulated rotary pendulum. */
Demos.pendulum = (root) => {
  const sim = new PendulumSim(window.PENDULUM_POLICY);
  const LR = PendulumParams.Lr, LP = 0.125; // drawn lengths (m)
  const HISTORY = 600; // alpha trace samples (6 s at 100 Hz)
  const history = [];
  let phase = "run"; // run -> settle -> run ...
  let caught = false; // has the balance agent taken over this episode?
  let armOffset = 0; // encoder is re-zeroed on reset, the arm itself stays where it is
  let speed = 1;
  let acc = 0;
  let tick = 0;

  const newEpisode = () => {
    armOffset += sim.x[0];
    sim.reset(0.01 + Math.random() * 0.1);
    caught = false;
    phase = "run";
    history.length = 0;
  };

  const supervise = () => {
    const s = sim.state;
    if (phase === "settle") {
      const still = Math.abs(s.alpha) < 0.05 && Math.abs(s.alphaDot) < 0.2 && Math.abs(s.thetaDot) < 0.2;
      if (still || sim.t > 6) newEpisode();
      return;
    }
    if (s.mode === "balance") caught = true;
    const fell = caught && Math.abs(s.alpha) < 1.6;
    const gaveUp = !caught && sim.t > 6;
    if (fell || gaveUp || Math.abs(s.thetaDot) > 80) {
      phase = "settle";
      sim.t = 0;
    }
  };

  function update(dt) {
    acc += dt * speed;
    let n = 0;
    while (acc >= PendulumParams.dt && n++ < 60) {
      acc -= PendulumParams.dt;
      phase === "settle" ? sim.settle() : sim.step();
      if (sim.steps % 10 === 0 || phase === "settle") supervise();
      if (++tick % 10 === 0) {
        history.push(sim.state.alpha);
        if (history.length > HISTORY) history.shift();
      }
    }
  }

  /* ---- drawing: orthographic camera, yawed and looking slightly down ---- */
  const YAW = -0.6, ELEV = 0.45;
  const BOX = 0.038, BOX_TOP = -0.012, BOX_BOTTOM = -0.11; // QUBE-style housing (m)
  let cam;

  const project = ([x, y, z]) => {
    const x1 = x * Math.cos(YAW) - y * Math.sin(YAW);
    const y1 = x * Math.sin(YAW) + y * Math.cos(YAW); // towards the viewer
    return [cam.cx + x1 * cam.s, cam.cy - (z * Math.cos(ELEV) - y1 * Math.sin(ELEV)) * cam.s];
  };
  const depth = ([x, y, z]) => (x * Math.sin(YAW) + y * Math.cos(YAW)) * Math.cos(ELEV) + z * Math.sin(ELEV);

  function draw(ctx, w, h, c) {
    const sceneH = h - u(64); // bottom strip is the alpha trace
    cam = { s: Math.min((w * 0.8) / 0.5, sceneH / 0.42), cx: w / 2, cy: sceneH * 0.5 };

    const s = sim.state;
    const th = s.theta + armOffset, al = s.alpha;
    const tip = [LR * Math.sin(th), LR * Math.cos(th), 0];
    const tan = [Math.cos(th), -Math.sin(th)];
    const bob = [tip[0] + LP * tan[0] * Math.sin(al), tip[1] + LP * tan[1] * Math.sin(al), -LP * Math.cos(al)];

    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, w, sceneH - u(14)); // keep the scene out of the trace strip
    ctx.clip();
    drawFloor(ctx, c);
    const behind = depth(tip) < depth([0, 0, -0.06]);
    if (behind) drawLinkage(ctx, tip, bob, c);
    drawHousing(ctx, c);
    if (!behind) drawLinkage(ctx, tip, bob, c);
    ctx.restore();

    drawHud(ctx, w, h, s, c);
    drawTrace(ctx, w, h, c);
  }

  function drawFloor(ctx, c) {
    ctx.strokeStyle = c.grid;
    ctx.lineWidth = 1;
    const R = 0.18, z = BOX_BOTTOM;
    for (let i = -R; i <= R + 1e-9; i += 0.04) {
      line(ctx, project([i, -R, z]), project([i, R, z]));
      line(ctx, project([-R, i, z]), project([R, i, z]));
    }
  }

  function face(ctx, pts, fill, stroke) {
    ctx.beginPath();
    pts.map(project).forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  function drawHousing(ctx, c) {
    const b = BOX, t = BOX_TOP, d = BOX_BOTTOM;
    // the two side faces that face the camera, then the top
    face(ctx, [[-b, b, t], [b, b, t], [b, b, d], [-b, b, d]], c.bg, c.muted);
    face(ctx, [[-b, -b, t], [-b, b, t], [-b, b, d], [-b, -b, d]], c.bg, c.muted);
    face(ctx, [[-b, -b, t], [b, -b, t], [b, b, t], [-b, b, t]], c.bg, c.muted);
    ctx.globalAlpha = 0.16; // shading on top of the opaque faces
    face(ctx, [[-b, b, t], [b, b, t], [b, b, d], [-b, b, d]], c.muted, c.muted);
    ctx.globalAlpha = 0.3;
    face(ctx, [[-b, -b, t], [-b, b, t], [-b, b, d], [-b, -b, d]], c.muted, c.muted);
    ctx.globalAlpha = 1;
    // hub
    const [hx, hy] = project([0, 0, 0]);
    ctx.fillStyle = c.text;
    ctx.beginPath();
    ctx.ellipse(hx, hy, 0.016 * cam.s, 0.016 * cam.s * Math.sin(ELEV), 0, 0, Math.PI * 2);
    ctx.fill();
  }

  function drawLinkage(ctx, tip, bob, c) {
    ctx.lineCap = "round";
    ctx.strokeStyle = c.muted;
    ctx.lineWidth = Math.max(4, 0.011 * cam.s);
    line(ctx, project([0, 0, 0]), project(tip));
    ctx.strokeStyle = c.accent;
    ctx.lineWidth = Math.max(4, 0.009 * cam.s);
    line(ctx, project(tip), project(bob));
    ctx.fillStyle = c.accent;
    circle(ctx, ...project(bob), Math.max(3, 0.007 * cam.s));
    ctx.fillStyle = c.text;
    circle(ctx, ...project(tip), Math.max(3.5, 0.008 * cam.s));
    ctx.lineCap = "butt";
  }

  function drawHud(ctx, w, h, s, c) {
    const label = { swingup: "SWING-UP AGENT", balance: "BALANCE AGENT", reset: "RESETTING" }[s.mode];
    const color = { swingup: c.c2, balance: c.c1, reset: c.muted }[s.mode];
    badge(ctx, u(14), u(14), label, color, c);

    ctx.font = mono(12);
    ctx.fillStyle = c.muted;
    ctx.textAlign = "right";
    const deg = (r) => ((r * 180) / Math.PI).toFixed(0).padStart(4) + "°";
    ctx.fillText(`α ${deg(s.alpha)}`, w - u(14), u(26));
    ctx.fillText(`arm ω ${s.thetaDot.toFixed(1).padStart(5)} rad/s`, w - u(14), u(44));
    ctx.fillText(`u ${s.V.toFixed(1).padStart(5)} V`, w - u(14), u(62));
    ctx.textAlign = "left";

    // voltage bar
    const bw = Math.min(u(160), w * 0.3), bx = w - u(14) - bw, by = u(72);
    ctx.fillStyle = c.grid;
    ctx.fillRect(bx, by, bw, 4);
    ctx.fillStyle = color;
    const v = s.V / 12;
    ctx.fillRect(bx + bw / 2, by, (v * bw) / 2, 4);
  }

  function drawTrace(ctx, w, h, c) {
    const top = h - u(46), hh = u(34), left = u(14), width = w - u(28);
    ctx.font = mono(11);
    ctx.fillStyle = c.muted;
    ctx.fillText("|α| last 6 s (dashed = upright)", left, top - u(8));
    ctx.strokeStyle = c.grid;
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    line(ctx, [left, top], [left + width, top]);
    ctx.setLineDash([]);
    ctx.strokeStyle = c.accent;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    history.forEach((a, i) => {
      const x = left + (i / HISTORY) * width;
      const y = top + hh - (Math.abs(a) / Math.PI) * hh;
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
  }

  /* ---- controls ---- */
  root.querySelector("[data-action=restart]").addEventListener("click", () => {
    phase = "settle";
    sim.t = 0;
  });
  root.querySelector("[data-action=push]").addEventListener("click", () => {
    sim.kick((Math.random() < 0.5 ? -1 : 1) * (1.5 + Math.random() * 1.5));
  });
  root.querySelector("[data-action=slow]").addEventListener("click", (e) => {
    speed = speed === 1 ? 0.25 : 1;
    e.currentTarget.setAttribute("aria-pressed", speed !== 1);
  });

  new Stage(root.querySelector("canvas"), { update, draw });
};
