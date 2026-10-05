/* Shared canvas stage for the live demos:
 * HiDPI sizing, theme colours from CSS variables, and an animation loop
 * that only runs while the demo is visible on screen. */
window.Demos = window.Demos || {};

// HUD sizes follow the page's root font size, so overlays scale with the rest of the layout.
let UI = 1;
const u = (n) => n * UI;
const mono = (size, weight = 500) => `${weight} ${u(size)}px 'JetBrains Mono', monospace`;

class Stage {
  constructor(canvas, { update, draw }) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.update = update;
    this.draw = draw;
    this.visible = false;
    this.last = 0;
    this.readColors();

    new ResizeObserver(() => this.resize()).observe(canvas);
    new IntersectionObserver(([e]) => {
      this.visible = e.isIntersecting;
      if (this.visible) this.start();
    }).observe(canvas);
    document.addEventListener("themechange", () => this.readColors());
    document.addEventListener("visibilitychange", () => !document.hidden && this.start());
  }

  readColors() {
    const css = getComputedStyle(document.documentElement);
    const v = (name) => css.getPropertyValue(name).trim();
    this.colors = {
      bg: v("--demo-bg"), grid: v("--border"), text: v("--text"), muted: v("--muted"),
      accent: v("--accent"), c1: v("--c1"), c2: v("--c2"), c3: v("--c3"), warn: v("--warn"),
    };
  }

  resize() {
    UI = parseFloat(getComputedStyle(document.documentElement).fontSize) / 16;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const { width, height } = this.canvas.getBoundingClientRect();
    this.w = width;
    this.h = height;
    this.canvas.width = Math.round(width * dpr);
    this.canvas.height = Math.round(height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.render();
  }

  start() {
    if (this.running) return;
    this.running = true;
    this.last = performance.now();
    requestAnimationFrame((t) => this.frame(t));
  }

  frame(now) {
    if (!this.visible || document.hidden) {
      this.running = false;
      return;
    }
    const dt = Math.min((now - this.last) / 1000, 1 / 20); // clamp after tab switches
    this.last = now;
    this.update(dt);
    this.render();
    requestAnimationFrame((t) => this.frame(t));
  }

  render() {
    if (!this.w) return;
    const { ctx, w, h, colors } = this;
    ctx.fillStyle = colors.bg;
    ctx.fillRect(0, 0, w, h);
    this.draw(ctx, w, h, colors);
  }
}

window.Stage = Stage;

/* ---- small canvas helpers shared by the demos ---- */
function line(ctx, [x1, y1], [x2, y2]) {
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
}

function circle(ctx, x, y, r) {
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fill();
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.roundRect(x, y, w, h, r);
}

function badge(ctx, x, y, text, color, c) {
  ctx.font = mono(11, 600);
  const w = ctx.measureText(text).width + u(26);
  ctx.fillStyle = c.bg;
  ctx.strokeStyle = c.grid;
  roundRect(ctx, x, y, w, u(24), u(12));
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = color;
  circle(ctx, x + u(12), y + u(12), u(3.5));
  ctx.fillStyle = c.text;
  ctx.fillText(text, x + u(20), y + u(16));
}
