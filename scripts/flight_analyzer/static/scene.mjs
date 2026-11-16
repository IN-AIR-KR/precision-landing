import * as THREE from "./vendor/three/three.module.min.js";
import { OrbitControls } from "./vendor/three/OrbitControls.js";
import { LineMaterial } from "./vendor/three/LineMaterial.js";
import { LineSegments2 } from "./vendor/three/LineSegments2.js";
import { LineSegmentsGeometry } from "./vendor/three/LineSegmentsGeometry.js";
import { sample, sceneData } from "./scene-data.mjs";

const lineWidth = 2;
const colors = { drone: "#67b7ff", estimate: "#ffb45e", truth: "#53e6af" };
const gaps = { drone: 0.3, estimate: 0.5, truth: 0.3 };
const nedToEnu = new THREE.Quaternion(Math.SQRT1_2, Math.SQRT1_2, 0, 0);

function screenLine(geometry, options) {
  const material = new LineMaterial({ linewidth: lineWidth, ...options });
  const line = new LineSegments2(geometry, material);
  line.onBeforeRender = (renderer) => renderer.getSize(material.resolution);
  return line;
}

function droneAxes(size) {
  const geometry = new LineSegmentsGeometry();
  geometry.setPositions([
    0, 0, 0, size, 0, 0,
    0, 0, 0, 0, size, 0,
    0, 0, 0, 0, 0, size,
  ]);
  geometry.setColors([
    1, 0, 0, 1, 0.6, 0,
    0, 1, 0, 0.6, 1, 0,
    0, 0, 1, 0, 0.6, 1,
  ]);
  return screenLine(geometry, { vertexColors: true });
}

export class LandingScene {
  constructor(host) {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color("#101722");
    this.camera = new THREE.PerspectiveCamera(48, 1, 0.02, 10000);
    this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.domElement.setAttribute("aria-label", "비행 궤적 3D 뷰어");
    host.replaceChildren(this.renderer.domElement);
    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.enablePan = false;
    this.orbit.addEventListener("change", () => {
      if (!this.updatingTime) this.render();
    });
    this.items = new THREE.Group();
    this.scene.add(this.items, new THREE.AxesHelper(1));
    this.resize = new ResizeObserver(() => {
      const { clientWidth: w, clientHeight: h } = host;
      if (!w || !h) return;
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(w, h, false);
      this.render();
    });
    this.resize.observe(host);
  }

  load(topics) {
    this.items.traverse((item) => {
      item.geometry?.dispose();
      item.material?.dispose();
    });
    this.items.clear();
    this.data = sceneData(topics);
    this.objects = {};
    let radius = 3;
    for (const [key, color] of Object.entries(colors)) {
      const rows = this.data[key];
      const vertices = [];
      for (let i = 0; i < rows.length; i++) {
        const row = rows[i], prev = rows[i - 1];
        if (row.length < 4) continue;
        radius = Math.max(radius, Math.hypot(...row.slice(1)));
        if (prev?.length === 4 && row[0] - prev[0] <= gaps[key]) {
          vertices.push(...prev.slice(1), ...row.slice(1));
        }
      }
      if (vertices.length) {
        const geometry = new LineSegmentsGeometry();
        geometry.setPositions(vertices);
        this.items.add(screenLine(geometry, { color, transparent: true, opacity: 0.65 }));
      }
      const shape = key === "drone" ? new THREE.ConeGeometry(0.21, 0.58, 4)
        : key === "estimate" ? new THREE.SphereGeometry(0.16, 16, 12)
        : new THREE.BoxGeometry(0.36, 0.36, 0.06);
      if (key === "drone") shape.rotateZ(-Math.PI / 2);
      const object = new THREE.Mesh(shape, new THREE.MeshBasicMaterial({
        color, wireframe: key === "truth",
      }));
      if (key === "drone") object.add(droneAxes(0.65));
      this.items.add(object);
      this.objects[key] = object;
    }
    const size = Math.max(6, 2 * Math.ceil(radius));
    const grid = new THREE.GridHelper(size, Math.min(size, 60), 0x53647b, 0x28374a);
    grid.rotation.x = Math.PI / 2;
    this.items.add(grid);
    this.orbit.target.set(0, 0, 0);
    this.orbit.cursor.copy(this.orbit.target);
    this.camera.position.set(1, -1.15, 0.8).multiplyScalar(radius);
    this.orbit.update();
    return this.data;
  }

  setTime(t, center) {
    if (!this.data) return;
    for (const [key, object] of Object.entries(this.objects)) {
      const position = sample(this.data[key], t, gaps[key], key !== "estimate");
      object.visible = !!position;
      if (position) object.position.set(...position);
    }
    const drone = this.objects.drone;
    const attitude = sample(this.data.attitude, t, 0.3, false);
    drone.quaternion.identity();
    if (attitude && Math.hypot(...attitude) > 0) {
      drone.quaternion.set(...attitude).normalize().premultiply(nedToEnu);
    }
    const target = center === "drone" && drone.visible ? drone.position : this.scene.position;
    this.camera.position.sub(this.orbit.target).add(target);
    this.orbit.target.copy(target);
    this.orbit.cursor.copy(target);
    this.updatingTime = true;
    this.orbit.update();
    this.updatingTime = false;
    this.render();
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }
}
