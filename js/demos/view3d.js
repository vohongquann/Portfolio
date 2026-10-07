/* Shared Three.js plumbing for the 3D demos: renderer creation, lights and an orbit camera.
 * Each demo renders into an offscreen WebGL canvas that it copies into its 2D canvas, so the HUD
 * and layout stay in 2D. Everything here degrades to null when Three.js or WebGL is unavailable. */
window.View3D = (() => {
  function createRenderer() {
    const THREE = window.THREE;
    if (!THREE) return null;
    try {
      const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
      renderer.shadowMap.enabled = true;
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      return renderer;
    } catch {
      return null; // no WebGL
    }
  }

  function addLights(scene, extent) {
    const T = window.THREE;
    const hemi = new T.HemisphereLight(0xffffff, 0x888888, 0.75);
    const sun = new T.DirectionalLight(0xffffff, 0.55);
    sun.position.set(extent * 0.8, extent * 2, extent);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1024, 1024);
    Object.assign(sun.shadow.camera, { left: -extent, right: extent, top: extent, bottom: -extent, near: 0.1, far: extent * 8 });
    scene.add(hemi, sun);
    return { hemi, sun };
  }

  /* Camera orbiting `target`. Drag rotates (touch only yaws so the page can still scroll);
   * a press that barely moves counts as a tap and goes to `onTap`. */
  class Orbit {
    constructor({ fov, target, reach, az, el }) {
      this.camera = new window.THREE.PerspectiveCamera(fov, 1, 0.05, 50);
      Object.assign(this, { fov, target, reach, az, el, dist: 1 });
    }

    attach(canvas, onTap) {
      canvas.classList.add("orbit");
      let start = null, last = null;
      canvas.addEventListener("pointerdown", (e) => {
        start = last = [e.clientX, e.clientY];
        canvas.setPointerCapture(e.pointerId);
      });
      canvas.addEventListener("pointermove", (e) => {
        if (!last) return;
        this.az -= (e.clientX - last[0]) * 0.008;
        this.el = Math.min(1.4, Math.max(0.05, this.el + (e.clientY - last[1]) * 0.006));
        last = [e.clientX, e.clientY];
      });
      canvas.addEventListener("pointerup", (e) => {
        if (start && onTap && Math.hypot(e.clientX - start[0], e.clientY - start[1]) < 5) onTap(e);
        start = last = null;
      });
      canvas.addEventListener("pointercancel", () => (start = last = null));
    }

    resize(w, h) {
      this.camera.aspect = w / h;
      // pull back until the swept volume fits both vertically and horizontally
      const t = Math.tan((this.fov * Math.PI) / 360);
      this.dist = Math.max(this.reach.y / t, this.reach.x / (t * this.camera.aspect));
      this.camera.updateProjectionMatrix();
    }

    update() {
      const [tx, ty, tz] = this.target, ce = Math.cos(this.el);
      this.camera.position.set(
        tx + this.dist * ce * Math.sin(this.az),
        ty + this.dist * Math.sin(this.el),
        tz + this.dist * ce * Math.cos(this.az)
      );
      this.camera.lookAt(tx, ty, tz);
    }
  }

  return { createRenderer, addLights, Orbit };
})();
