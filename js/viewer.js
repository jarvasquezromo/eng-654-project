// three.js viewer for GoFa: STL meshes on the URDF chain plus helpers for frames, axes and arrows.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { parseURDF, urdfFK } from './gofa.js';

const ROOT = new URL('../', import.meta.url);
export const assetURL = (p) => new URL(p, ROOT).href;

let modelPromise;
export function loadModel() {
  modelPromise ??= fetch(assetURL('assets/abb_gofa/crb15000_5_95.urdf')).then((r) => r.text()).then((t) => ({ text: t, model: parseURDF(t) }));
  return modelPromise;
}

function parseSTL(buffer) {
  const view = new DataView(buffer), n = view.getUint32(80, true);
  const pos = new Float32Array(n * 9), nor = new Float32Array(n * 9);
  for (let i = 0; i < n; i++) {
    const o = 84 + i * 50;
    const nx = view.getFloat32(o, true), ny = view.getFloat32(o + 4, true), nz = view.getFloat32(o + 8, true);
    for (let v = 0; v < 3; v++) for (let k = 0; k < 3; k++) {
      pos[i * 9 + v * 3 + k] = view.getFloat32(o + 12 + v * 12 + k * 4, true);
      nor[i * 9 + v * 3 + k] = [nx, ny, nz][k];
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  g.computeVertexNormals();
  return g;
}

export const toMatrix4 = (T) => new THREE.Matrix4().set(...T[0], ...T[1], ...T[2], ...T[3]);

function css(name, fallback) {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return v || fallback;
}

export class RobotViewer {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(2.1, -1.9, 1.45);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.25, 0, 0.55);
    this.controls.enableDamping = true;
    this.controls.addEventListener('change', () => this.render());

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8a8a, 2.2));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(2, -3, 4); this.scene.add(sun);
    const grid = new THREE.GridHelper(2, 20, 0xc8c8c8, 0xe6e6e6); grid.rotation.x = Math.PI / 2; this.scene.add(grid);
    this.grid = grid;

    this.robot = new THREE.Group(); this.scene.add(this.robot);
    this.overlay = new THREE.Group(); this.scene.add(this.overlay);
    this.meshes = {};
    this.q = [0, 0, 0, 0, 0, 0];

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    const loop = () => { this.controls.update(); requestAnimationFrame(loop); };
    loop();
  }

  async load() {
    const { model } = await loadModel();
    this.model = model;
    await Promise.all(model.links.filter((l) => l.mesh).map(async (l) => {
      const file = l.mesh.split('/').pop();
      const buf = await fetch(assetURL(`assets/abb_gofa/${file}`)).then((r) => r.arrayBuffer());
      const rgba = (l.color ?? '0.6 0.6 0.6 1').split(/\s+/).map(Number);
      const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(rgba[0], rgba[1], rgba[2]), roughness: 0.55, metalness: 0.1, transparent: true, opacity: 1 });
      const mesh = new THREE.Mesh(parseSTL(buf), mat);
      mesh.matrixAutoUpdate = false;
      this.robot.add(mesh);
      this.meshes[l.name] = mesh;
    }));
    this.setQ(this.q);
    return this;
  }

  setOpacity(a) {
    Object.values(this.meshes).forEach((m) => { m.material.opacity = a; m.material.depthWrite = a > 0.95; });
    this.render();
  }
  setRobotVisible(v) { this.robot.visible = v; this.render(); }

  setQ(q) {
    this.q = q.slice();
    if (!this.model) return;
    const { links } = urdfFK(this.model, q);
    for (const [name, mesh] of Object.entries(this.meshes)) mesh.matrix.copy(toMatrix4(links[name]));
    this.render();
  }

  clearOverlay() {
    this.overlay.children.slice().forEach((c) => { this.overlay.remove(c); c.traverse?.((o) => { o.geometry?.dispose(); o.material?.dispose?.(); }); });
  }

  // Coordinate frame triad at a 4×4 transform
  addFrame(T, { size = 0.08, label = null, lineWidth = 0.006, opacity = 1 } = {}) {
    const g = new THREE.Group();
    const cols = [0xe53935, 0x2e9d4f, 0x2f6fe4];
    [[1, 0, 0], [0, 1, 0], [0, 0, 1]].forEach((d, i) => {
      g.add(makeArrow(new THREE.Vector3(0, 0, 0), new THREE.Vector3(...d), size, cols[i], lineWidth, opacity));
    });
    g.matrixAutoUpdate = false; g.matrix.copy(toMatrix4(T));
    if (label) g.add(makeLabel(label, [size * 0.25, size * 0.25, -size * 0.15]));
    this.overlay.add(g);
    return g;
  }

  // Infinite-looking line through point r with direction w
  addAxisLine(r, w, { color = 0xd4a017, length = 0.9, radius = 0.004, opacity = 0.9 } = {}) {
    const dir = new THREE.Vector3(...w).normalize();
    const geo = new THREE.CylinderGeometry(radius, radius, length, 10);
    const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color, transparent: true, opacity }));
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    mesh.position.set(...r);
    this.overlay.add(mesh);
    return mesh;
  }

  addArrow(origin, vec, { color = 0xff0000, scale = 1, radius = 0.006 } = {}) {
    const v = new THREE.Vector3(...vec).multiplyScalar(scale);
    const len = v.length();
    if (len < 1e-6) return null;
    const a = makeArrow(new THREE.Vector3(...origin), v.clone().normalize(), len, color, radius, 1);
    this.overlay.add(a);
    return a;
  }

  addPoint(p, { color = 0xff0000, radius = 0.012, label = null } = {}) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(radius, 16, 12), new THREE.MeshBasicMaterial({ color }));
    m.position.set(...p);
    if (label) m.add(makeLabel(label, [radius * 1.6, 0, radius * 1.6]));
    this.overlay.add(m);
    return m;
  }

  addPolyline(points, { color = 0xff0000, closed = false } = {}) {
    const pts = points.map((p) => new THREE.Vector3(...p));
    if (closed) pts.push(pts[0].clone());
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color }));
    this.overlay.add(line);
    return line;
  }

  view(name) {
    const t = this.controls.target;
    const views = { iso: [2.1, -1.9, 1.45], front: [0.25, -2.9, 0.6], top: [0.25, -0.001, 3.0], side: [2.9, 0, 0.6] };
    this.camera.position.set(...views[name]);
    t.set(0.25, 0, name === 'top' ? 0 : 0.55);
    this.controls.update(); this.render();
  }

  resize() {
    const w = this.container.clientWidth || 1, h = this.container.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
    this.render();
  }
  render() {
    if (this._pending) return;
    this._pending = true;
    requestAnimationFrame(() => {
      this._pending = false;
      this.renderer.render(this.scene, this.camera);
      this.labels?.();
    });
  }
}

function makeArrow(origin, dir, length, color, radius, opacity) {
  const g = new THREE.Group();
  const mat = new THREE.MeshBasicMaterial({ color, transparent: opacity < 1, opacity });
  const head = Math.min(length * 0.3, radius * 5);
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length - head, 8), mat);
  shaft.position.y = (length - head) / 2;
  const tip = new THREE.Mesh(new THREE.ConeGeometry(radius * 2.4, head, 12), mat);
  tip.position.y = length - head / 2;
  g.add(shaft, tip);
  g.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
  g.position.copy(origin);
  return g;
}

// Text sprite label
function makeLabel(text, offset) {
  const c = document.createElement('canvas'), ctx = c.getContext('2d');
  const font = 44; ctx.font = `600 ${font}px Inter, Helvetica, Arial, sans-serif`;
  c.width = Math.ceil(ctx.measureText(text).width) + 24; c.height = font + 20;
  ctx.font = `600 ${font}px Inter, Helvetica, Arial, sans-serif`;
  ctx.fillStyle = 'rgba(255,255,255,0.85)'; ctx.fillRect(0, 0, c.width, c.height);
  ctx.fillStyle = '#141414'; ctx.textBaseline = 'middle'; ctx.fillText(text, 12, c.height / 2);
  const tex = new THREE.CanvasTexture(c); tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  const h = 0.032; s.scale.set((h * c.width) / c.height, h, 1);
  s.position.set(...offset); s.renderOrder = 10;
  return s;
}
export { THREE, css };
