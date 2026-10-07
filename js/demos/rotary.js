/* Live demo: the RotaryPendulum-IsaacLab policy retuning the two PIDs of a simulated QUBE-Servo 2 (rotary-model.js). */
Demos.rotary = (root) => {
  const sim = new RotarySim(new Policy(window.ROTARY_POLICY));
  const canvas = root.querySelector("canvas");
  const view3d = Pendulum3D.create(); // null without Three.js / WebGL: fall back to the flat drawing
  view3d?.orbit.attach(canvas);
  const deg = (r) => (r * 180) / Math.PI;
  const PROGRAM = [0, -45, -90, -45, 0, 45, 90, 45].map((d) => (d * Math.PI) / 180); // arm commands, as in the repo's demo
  const HOLD = 3; // s per command
  const HISTORY = 600; // trace samples (6 s at 100 Hz)
  const history = [];
  let phase = "run"; // run -> settle -> run ...
  let armOffset = 0; // the encoder is zeroed at every start, the arm itself stays where it is
  let balancedFor = 0;
  let programTime = 0;
  let speed = 1;
  let acc = 0;

  const start = () => {
    armOffset += sim.x[0];
    sim.reset(0.02 + Math.random() * 0.05);
    phase = "run";
    balancedFor = 0;
    programTime = 0;
    history.length = 0;
  };

  // Motor off, joints damped: the pendulum comes to rest hanging before the next swing-up.
  const settle = () => {
    sim.coast();
    const hanging = Math.cos(sim.x[1]) > 0.998 && Math.abs(sim.x[3]) < 0.2 && Math.abs(sim.x[2]) < 0.2;
    if (hanging || sim.t > 8) start();
  };

  const stepOnce = () => {
    if (phase === "settle") return settle();
    const s = sim.state;
    if (s.fromUpright < 0.35) balancedFor += 0.01;
    if (balancedFor > 1.5) {
      programTime += 0.01;
      sim.command = PROGRAM[Math.floor(programTime / HOLD) % PROGRAM.length];
    }
    sim.step();
    // the training episode ends when the arm turns more than half a turn; a push it can recover from is left to the policy
    if (Math.abs(sim.x[0]) > Math.PI || (sim.t > 6 && balancedFor === 0)) {
      phase = "settle";
      sim.t = 0;
    }
    history.push([sim.x[0], sim.command]);
    if (history.length > HISTORY) history.shift();
  };

  function update(dt) {
    acc += dt * speed;
    let n = 0;
    while (acc >= 0.01 && n++ < 8) {
      acc -= 0.01;
      stepOnce();
    }
  }

  /* ---- drawing: 3D view when available, otherwise an orthographic 2D projection ---- */
  const YAW = -0.6, ELEV = 0.45;
  const BOX = 0.038, BOX_TOP = -0.012, BOX_BOTTOM = -0.11; // QUBE housing (m)
  let cam;

  const project = ([x, y, z]) => {
    const x1 = x * Math.cos(YAW) - y * Math.sin(YAW);
    const y1 = x * Math.sin(YAW) + y * Math.cos(YAW);
    return [cam.cx + x1 * cam.s, cam.cy - (z * Math.cos(ELEV) - y1 * Math.sin(ELEV)) * cam.s];
  };
  const depth = ([x, y, z]) => (x * Math.sin(YAW) + y * Math.cos(YAW)) * Math.cos(ELEV) + z * Math.sin(ELEV);

  function draw(ctx, w, h, c) {
    const sceneH = h - u(64); // bottom strip is the arm trace
    cam = { s: Math.min((w * 0.8) / 0.45, sceneH / 0.4), cx: w / 2, cy: sceneH * 0.45 };
    const { tip, bob } = sim.geometry(armOffset);
    const frame = view3d?.render(w, sceneH, ctx.getTransform().a, c, tip, bob);
    if (frame) ctx.drawImage(frame, 0, 0, w, sceneH);
    else drawScene2D(ctx, w, sceneH, c, tip, bob);
    drawHud(ctx, w, c);
    drawTrace(ctx, w, h, c);
  }

  function drawScene2D(ctx, w, sceneH, c, tip, bob) {
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, w, sceneH - u(14));
    ctx.clip();
    ctx.strokeStyle = c.grid;
    ctx.lineWidth = 1;
    for (let i = -0.16; i <= 0.161; i += 0.04) {
      line(ctx, project([i, -0.16, BOX_BOTTOM]), project([i, 0.16, BOX_BOTTOM]));
      line(ctx, project([-0.16, i, BOX_BOTTOM]), project([0.16, i, BOX_BOTTOM]));
    }
    const behind = depth(tip) < depth([0, 0, -0.06]);
    if (behind) drawLinkage(ctx, tip, bob, c);
    drawHousing(ctx, c);
    if (!behind) drawLinkage(ctx, tip, bob, c);
    ctx.restore();
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
    face(ctx, [[-b, b, t], [b, b, t], [b, b, d], [-b, b, d]], c.bg, c.muted);
    face(ctx, [[-b, -b, t], [-b, b, t], [-b, b, d], [-b, -b, d]], c.bg, c.muted);
    face(ctx, [[-b, -b, t], [b, -b, t], [b, b, t], [-b, b, t]], c.bg, c.muted);
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
    ctx.lineCap = "butt";
  }

  function drawHud(ctx, w, c) {
    const s = sim.state;
    const label = phase === "settle" ? "RESETTING" : s.fromUpright < 0.35 ? "BALANCING" : "SWING-UP";
    const color = phase === "settle" ? c.muted : s.fromUpright < 0.35 ? c.c1 : c.c2;
    badge(ctx, u(14), u(14), label, color, c);

    ctx.font = mono(12);
    ctx.fillStyle = c.muted;
    ctx.textAlign = "right";
    const a = (r) => deg(r).toFixed(1).padStart(6) + "°";
    const q2 = ((deg(s.pendulum) % 360) + 360) % 360;
    ctx.fillText(`θ1* ${a(s.command)}`, w - u(14), u(26));
    ctx.fillText(`θ1 ${a(s.arm)}`, w - u(14), u(44));
    ctx.fillText(`θ2 ${q2.toFixed(1).padStart(6)}°`, w - u(14), u(62));
    ctx.fillText(`V ${s.voltage.toFixed(1).padStart(6)} `, w - u(14), u(80));
    const [, , , kp2, , kd2] = s.gains;
    ctx.font = mono(11);
    ctx.fillText(`pendulum PID  Kp ${kp2.toFixed(1)}  Kd ${kd2.toFixed(2)}`, w - u(14), u(100));
    ctx.textAlign = "left";
  }

  function drawTrace(ctx, w, h, c) {
    const top = h - u(50), hh = u(36), left = u(14), width = w - u(28);
    ctx.font = mono(11);
    ctx.fillStyle = c.muted;
    ctx.fillText("arm angle (solid) and command (dashed), last 6 s", left, top - u(6));
    const y = (r) => top + hh / 2 - (r / Math.PI) * hh;
    const path = (k, color, dash) => {
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.setLineDash(dash);
      ctx.beginPath();
      history.forEach((p, i) => {
        const x = left + (i / HISTORY) * width;
        i ? ctx.lineTo(x, y(p[k])) : ctx.moveTo(x, y(p[k]));
      });
      ctx.stroke();
      ctx.setLineDash([]);
    };
    path(1, c.muted, [4, 4]);
    path(0, c.accent, []);
  }

  /* ---- controls ---- */
  root.querySelector("[data-action=restart]").addEventListener("click", () => {
    phase = "settle";
    sim.t = 0;
  });
  root.querySelector("[data-action=push]").addEventListener("click", () => {
    sim.kick((Math.random() < 0.5 ? -1 : 1) * (2 + Math.random() * 2));
  });
  root.querySelector("[data-action=slow]").addEventListener("click", (e) => {
    speed = speed === 1 ? 0.25 : 1;
    e.currentTarget.setAttribute("aria-pressed", speed !== 1);
  });

  new Stage(canvas, { update, draw });
};
