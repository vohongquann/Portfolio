/* Live demo: a tiny JS port of NeuroKit's building blocks (Linear, ReLU/Tanh, CrossEntropy,
 * SGD/Adam/RMSprop) training on NeuroKit's own synthetic datasets, with the decision boundary drawn live. */
const nk = (() => {
  const randn = () => Math.sqrt(-2 * Math.log(1 - Math.random())) * Math.cos(2 * Math.PI * Math.random());

  // Matrices are { r, c, d: Float64Array } in row-major order.
  const mat = (r, c, fill = () => 0) => ({ r, c, d: Float64Array.from({ length: r * c }, fill) });

  function matmul(A, B, transA = false, transB = false) {
    const n = transA ? A.c : A.r, k = transA ? A.r : A.c, m = transB ? B.r : B.c;
    const C = mat(n, m);
    for (let i = 0; i < n; i++)
      for (let p = 0; p < k; p++) {
        const a = transA ? A.d[p * A.c + i] : A.d[i * A.c + p];
        if (a === 0) continue;
        for (let j = 0; j < m; j++) C.d[i * m + j] += a * (transB ? B.d[j * B.c + p] : B.d[p * B.c + j]);
      }
    return C;
  }

  class Linear {
    constructor(inF, outF) {
      const std = Math.sqrt(2 / inF); // He init
      this.W = mat(inF, outF, () => randn() * std);
      this.b = mat(1, outF);
      this.gW = mat(inF, outF);
      this.gb = mat(1, outF);
    }
    forward(X) {
      this.X = X;
      const Y = matmul(X, this.W);
      for (let i = 0; i < Y.r; i++) for (let j = 0; j < Y.c; j++) Y.d[i * Y.c + j] += this.b.d[j];
      return Y;
    }
    backward(dY) {
      this.gW = matmul(this.X, dY, true);
      this.gb = mat(1, dY.c);
      for (let i = 0; i < dY.r; i++) for (let j = 0; j < dY.c; j++) this.gb.d[j] += dY.d[i * dY.c + j];
      return matmul(dY, this.W, false, true);
    }
    params() {
      return [[this.W, this.gW], [this.b, this.gb]];
    }
  }

  class Activation {
    constructor(fn, grad) {
      Object.assign(this, { fn, grad });
    }
    forward(X) {
      this.Y = { ...X, d: X.d.map(this.fn) };
      return this.Y;
    }
    backward(dY) {
      return { ...dY, d: dY.d.map((g, i) => g * this.grad(this.Y.d[i])) };
    }
    params() {
      return [];
    }
  }
  const ReLU = () => new Activation((x) => Math.max(0, x), (y) => (y > 0 ? 1 : 0));
  const Tanh = () => new Activation(Math.tanh, (y) => 1 - y * y);

  class Sequential {
    constructor(...layers) {
      this.layers = layers;
    }
    forward(X) {
      return this.layers.reduce((x, l) => l.forward(x), X);
    }
    backward(dY) {
      this.layers.reduceRight((g, l) => l.backward(g), dY);
    }
    params() {
      return this.layers.flatMap((l) => l.params());
    }
  }

  // Softmax + negative log-likelihood, returns loss, accuracy and dLogits.
  function crossEntropy(logits, labels) {
    const { r, c } = logits, grad = mat(r, c);
    let loss = 0, correct = 0;
    for (let i = 0; i < r; i++) {
      const row = logits.d.subarray(i * c, i * c + c);
      const max = Math.max(...row);
      const exps = row.map((v) => Math.exp(v - max));
      const sum = exps.reduce((a, b) => a + b, 0);
      loss -= Math.log(exps[labels[i]] / sum + 1e-12);
      if (row.indexOf(max) === labels[i]) correct++;
      for (let j = 0; j < c; j++) grad.d[i * c + j] = (exps[j] / sum - (j === labels[i])) / r;
    }
    return { loss: loss / r, acc: correct / r, grad };
  }

  // Optimizers keep per-parameter state keyed by the parameter matrix.
  const optimizers = {
    SGD: (lr = 0.1, momentum = 0.9) => {
      const v = new Map();
      return (params) =>
        params.forEach(([p, g]) => {
          const vel = v.get(p) || v.set(p, new Float64Array(p.d.length)).get(p);
          p.d.forEach((_, i) => {
            vel[i] = momentum * vel[i] - lr * g.d[i];
            p.d[i] += vel[i];
          });
        });
    },
    Adam: (lr = 0.02, b1 = 0.9, b2 = 0.999) => {
      const s = new Map();
      let t = 0;
      return (params) => {
        t++;
        params.forEach(([p, g]) => {
          if (!s.has(p)) s.set(p, { m: new Float64Array(p.d.length), v: new Float64Array(p.d.length) });
          const { m, v } = s.get(p);
          p.d.forEach((_, i) => {
            m[i] = b1 * m[i] + (1 - b1) * g.d[i];
            v[i] = b2 * v[i] + (1 - b2) * g.d[i] ** 2;
            p.d[i] -= (lr * (m[i] / (1 - b1 ** t))) / (Math.sqrt(v[i] / (1 - b2 ** t)) + 1e-8);
          });
        });
      };
    },
    RMSprop: (lr = 0.01, rho = 0.9) => {
      const s = new Map();
      return (params) =>
        params.forEach(([p, g]) => {
          const sq = s.get(p) || s.set(p, new Float64Array(p.d.length)).get(p);
          p.d.forEach((_, i) => {
            sq[i] = rho * sq[i] + (1 - rho) * g.d[i] ** 2;
            p.d[i] -= (lr * g.d[i]) / (Math.sqrt(sq[i]) + 1e-8);
          });
        });
    },
  };

  // Same generators as neurokit/data_generator.py (N points per class, K classes, 2-D).
  const datasets = {
    Spiral: (N, K) =>
      gen(N, K, (j, i) => {
        const r = i / (N - 1), t = j * 4 + (4 * i) / (N - 1) + randn() * 0.2;
        return [r * Math.sin(t), r * Math.cos(t)];
      }),
    Circle: (N, K) =>
      gen(N, K, (j, i) => {
        const r = (j + 1) * 2 + randn() * 0.5, t = (2 * Math.PI * i) / (N - 1) + randn() * 0.2;
        return [r * Math.sin(t), r * Math.cos(t)];
      }),
    Zone: (N, K) =>
      gen(N, K, (j, i) => {
        const th = (j * 2 * Math.PI) / K, r = randn() * 0.5, t = (2 * Math.PI * i) / (N - 1) + randn() * 0.2;
        return [Math.cos(th) + r * Math.sin(t), Math.sin(th) + r * Math.cos(t)];
      }),
  };

  function gen(N, K, point) {
    const pts = [], labels = [];
    for (let j = 0; j < K; j++) for (let i = 0; i < N; i++) pts.push(point(j, i)), labels.push(j);
    const s = Math.max(...pts.flat().map(Math.abs)) * 1.05; // fit into [-1, 1]
    return { X: { r: pts.length, c: 2, d: Float64Array.from(pts.flat().map((v) => v / s)) }, labels };
  }

  return { Linear, ReLU, Tanh, Sequential, crossEntropy, optimizers, datasets, mat };
})();

Demos.neurokit = (root) => {
  const GRID = 56; // decision-boundary resolution
  const K = 3;
  const opts = { dataset: "Spiral", optimizer: "Adam", activation: "ReLU" };
  let data, model, step, epoch, stats, boundary, frame = 0, running = true;

  const mapCanvas = document.createElement("canvas");
  mapCanvas.width = mapCanvas.height = GRID;
  const mapCtx = mapCanvas.getContext("2d");
  const hexToRgb = (hex) => [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16));

  const gridX = nk.mat(GRID * GRID, 2);
  for (let i = 0; i < GRID; i++)
    for (let j = 0; j < GRID; j++) {
      gridX.d[(i * GRID + j) * 2] = (j + 0.5) / GRID * 2 - 1;
      gridX.d[(i * GRID + j) * 2 + 1] = 1 - (i + 0.5) / GRID * 2;
    }

  function reset() {
    data = nk.datasets[opts.dataset](100, opts.dataset === "Circle" ? 2 : K);
    const act = nk[opts.activation];
    model = new nk.Sequential(new nk.Linear(2, 24), act(), new nk.Linear(24, 24), act(), new nk.Linear(24, K));
    step = nk.optimizers[opts.optimizer]();
    epoch = 0;
    stats = { loss: NaN, acc: 0, history: [] };
    boundary = null;
  }

  function train(iters) {
    for (let k = 0; k < iters; k++) {
      const out = nk.crossEntropy(model.forward(data.X), data.labels);
      model.backward(out.grad);
      step(model.params());
      epoch++;
      stats.loss = out.loss;
      stats.acc = out.acc;
    }
    stats.history.push(stats.loss);
    if (stats.history.length > 200) stats.history.shift();
    if (boundary && frame % 3) return; // the boundary is the expensive part, refresh it every 3rd frame
    const logits = model.forward(gridX);
    boundary = Array.from({ length: GRID * GRID }, (_, i) => {
      const row = logits.d.subarray(i * K, i * K + K);
      return row.indexOf(Math.max(...row));
    });
  }

  function update() {
    frame++;
    if (running && epoch < 3000) train(3);
    else if (!boundary) train(0);
  }

  function draw(ctx, w, h, c) {
    const size = Math.min(w, h) - 28;
    const ox = (w - size) / 2, oy = (h - size) / 2;
    const palette = [c.c1, c.c2, c.c3];

    if (boundary) {
      // paint one pixel per grid cell, then let the browser upscale it smoothly
      const rgb = palette.map(hexToRgb);
      const img = mapCtx.createImageData(GRID, GRID);
      boundary.forEach((k, i) => img.data.set([...rgb[k], 255], i * 4));
      mapCtx.putImageData(img, 0, 0);
      ctx.globalAlpha = 0.24;
      ctx.imageSmoothingEnabled = true;
      ctx.drawImage(mapCanvas, ox, oy, size, size);
      ctx.globalAlpha = 1;
    }
    ctx.strokeStyle = c.grid;
    ctx.strokeRect(ox, oy, size, size);

    const { X, labels } = data;
    labels.forEach((k, i) => {
      const x = ox + ((X.d[2 * i] + 1) / 2) * size, y = oy + ((1 - X.d[2 * i + 1]) / 2) * size;
      ctx.fillStyle = palette[k];
      ctx.strokeStyle = c.bg;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(x, y, Math.max(2.5, size / 130), 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    });

    // stats card
    ctx.font = mono(12);
    const lines = [`epoch ${epoch}`, `loss  ${isNaN(stats.loss) ? "–" : stats.loss.toFixed(3)}`, `acc   ${(stats.acc * 100).toFixed(1)}%`];
    ctx.fillStyle = c.bg;
    ctx.globalAlpha = 0.85;
    roundRect(ctx, ox + u(10), oy + u(10), u(150), u(92), u(10));
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.fillStyle = c.text;
    lines.forEach((t, i) => ctx.fillText(t, ox + u(22), oy + u(32 + i * 18)));
    // loss sparkline
    const hist = stats.history, max = Math.max(...hist, 1e-6);
    ctx.strokeStyle = c.accent;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    hist.forEach((v, i) => {
      const x = ox + u(22 + (i / 200) * 126), y = oy + u(92 - (v / max) * 14);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
  }

  /* ---- controls ---- */
  root.querySelectorAll("select[data-opt]").forEach((sel) =>
    sel.addEventListener("change", () => {
      opts[sel.dataset.opt] = sel.value;
      reset();
    })
  );
  const playBtn = root.querySelector("[data-action=play]");
  playBtn.addEventListener("click", () => {
    running = !running;
    playBtn.setAttribute("aria-pressed", !running);
    playBtn.innerHTML = running ? '<i class="bx bx-pause"></i> Pause' : '<i class="bx bx-play"></i> Train';
  });
  root.querySelector("[data-action=reset]").addEventListener("click", reset);

  reset();
  new Stage(root.querySelector("canvas"), { update, draw });
};
