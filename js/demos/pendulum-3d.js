/* Real 3D view of the rotary pendulum (Three.js, WebGL).
 * It renders into an offscreen canvas that pendulum.js copies into the demo's 2D canvas,
 * so the HUD, trace and layout stay the same. `Pendulum3D.create()` returns null when
 * Three.js or WebGL is unavailable and the demo falls back to its flat 2D drawing.
 *
 * Geometry is built in the simulation's frame (x, y horizontal, z up) inside a group
 * rotated into Three.js's y-up world, so positions from the model can be used as they are. */
window.Pendulum3D = (() => {
  const BOX = 0.038, BOX_TOP = -0.012, BOX_BOTTOM = -0.11; // QUBE-style housing (m)

  class Scene {
    constructor(THREE, renderer) {
      this.T = THREE;
      this.renderer = renderer;
      // reach: half extents (m) of everything the pendulum sweeps, horizontally and vertically
      this.orbit = new View3D.Orbit({ fov: 32, target: [0, -0.01, 0], reach: { x: 0.23, y: 0.17 }, az: 0.75, el: 0.42 });
      this.key = "";
      this.size = "";
      this.build();
    }

    build() {
      const T = this.T;
      this.scene = new T.Scene();
      this.hemi = View3D.addLights(this.scene, 0.3).hemi;

      // simulation frame -> Three.js: z up becomes y up
      const world = new T.Group();
      world.rotation.x = -Math.PI / 2;
      this.scene.add(world);

      const mat = () => new T.MeshStandardMaterial({ roughness: 0.55, metalness: 0.1 });
      this.mats = { housing: mat(), hub: mat(), arm: mat(), rod: mat(), bob: mat() };
      const shadowed = (mesh) => ((mesh.castShadow = true), mesh);

      this.housing = shadowed(new T.Mesh(new T.BoxGeometry(BOX * 2, BOX * 2, BOX_TOP - BOX_BOTTOM), this.mats.housing));
      this.housing.position.z = (BOX_TOP + BOX_BOTTOM) / 2;
      this.edges = new T.LineSegments(new T.EdgesGeometry(this.housing.geometry), new T.LineBasicMaterial());
      this.housing.add(this.edges);

      this.hub = shadowed(new T.Mesh(new T.CylinderGeometry(0.016, 0.016, 0.012, 32), this.mats.hub));
      this.hub.rotation.x = Math.PI / 2; // cylinder axis along z

      const unitBar = (material, radius) => shadowed(new T.Mesh(new T.CylinderGeometry(radius, radius, 1, 16), material));
      this.arm = unitBar(this.mats.arm, 0.0045);
      this.rod = unitBar(this.mats.rod, 0.0035);
      this.tip = shadowed(new T.Mesh(new T.SphereGeometry(0.0085, 24, 16), this.mats.hub));
      this.bob = shadowed(new T.Mesh(new T.SphereGeometry(0.0095, 24, 16), this.mats.bob));

      this.grid = new T.GridHelper(0.32, 8);
      this.grid.rotation.x = Math.PI / 2; // lie flat in the x-y plane
      this.grid.position.z = BOX_BOTTOM;
      const ground = new T.Mesh(new T.PlaneGeometry(0.32, 0.32), new T.ShadowMaterial({ opacity: 0.22 }));
      ground.position.z = BOX_BOTTOM + 0.0005;
      ground.receiveShadow = true;

      world.add(this.housing, this.hub, this.arm, this.rod, this.tip, this.bob, this.grid, ground);
    }

    setColors(c) {
      const key = [c.bg, c.muted, c.text, c.accent].join();
      if (key === this.key) return;
      this.key = key;
      const T = this.T;
      const housing = new T.Color(c.bg).lerp(new T.Color(c.muted), 0.3);
      this.mats.housing.color.copy(housing);
      this.mats.hub.color.set(c.text);
      this.mats.arm.color.set(c.muted);
      this.mats.rod.color.set(c.accent);
      this.mats.bob.color.set(c.accent);
      this.edges.material.color.set(c.muted);
      this.grid.material.color.copy(new T.Color(c.bg).lerp(new T.Color(c.muted), 0.4));
      this.hemi.groundColor.set(c.muted);
    }

    resize(w, h, dpr) {
      const key = `${w}x${h}@${dpr}`;
      if (key === this.size) return;
      this.size = key;
      this.renderer.setPixelRatio(dpr);
      this.renderer.setSize(w, h, false);
      this.orbit.resize(w, h);
    }

    placeBar(mesh, a, b) {
      const dir = new this.T.Vector3(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
      const len = dir.length();
      mesh.position.set((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2);
      mesh.quaternion.setFromUnitVectors(new this.T.Vector3(0, 1, 0), dir.divideScalar(len || 1));
      mesh.scale.set(1, len, 1);
    }

    /* tip and bob are in the simulation frame. Returns the canvas holding the frame, or null if the GL context is gone. */
    render(w, h, dpr, c, tip, bob) {
      if (this.renderer.getContext().isContextLost()) return null;
      this.resize(w, h, dpr);
      this.setColors(c);
      this.placeBar(this.arm, [0, 0, 0], tip);
      this.placeBar(this.rod, tip, bob);
      this.tip.position.set(...tip);
      this.bob.position.set(...bob);

      this.orbit.update();
      this.renderer.render(this.scene, this.orbit.camera);
      return this.renderer.domElement;
    }
  }

  function create() {
    const renderer = View3D.createRenderer();
    return renderer && new Scene(window.THREE, renderer);
  }

  return { create };
})();
