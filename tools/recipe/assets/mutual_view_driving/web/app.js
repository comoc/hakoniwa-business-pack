// Mutual-view driving client: renders the shared Hakoniwa world from my driver eye (main view)
// and from the partner's driver eye (picture-in-picture), and sends my driving inputs.
// World frame (x east, y north, z up) maps to Three.js as (x, z, -y); yaw maps to rotation.y.
import * as THREE from "three";
import { RIDER_LAYER, buildVehicle, occluders, targetPoints } from "./vehicle_models.js";

const ROLES = ["bike", "car"];
const LABEL = { bike: "自転車", car: "自動車", spectator: "観戦" };
const params = new URLSearchParams(location.search);
const wsUrl = params.get("ws") ?? `ws://${location.hostname || "127.0.0.1"}:8771`;
let myRole = params.get("role");
let swapped = false;

const $ = (id) => document.getElementById(id);
const statusEl = $("status");

// ---------------------------------------------------------------- renderer and scene
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
document.body.prepend(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xa9cbe8);
scene.fog = new THREE.Fog(0xa9cbe8, 80, 260);
scene.add(new THREE.HemisphereLight(0xffffff, 0x667755, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(-40, 80, 30);
scene.add(sun);

const w2t = (x, y, z = 0) => new THREE.Vector3(x, z, -y);

function windowTexture() {
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = 64;
  const g = canvas.getContext("2d");
  g.fillStyle = "#ffffff";
  g.fillRect(0, 0, 64, 64);
  g.fillStyle = "#5d6d7e";
  for (const x of [8, 36]) for (const y of [10, 40]) g.fillRect(x, y, 20, 16);
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function flatRect(rect, color, height) {
  const width = rect.x1 - rect.x0, depth = rect.y1 - rect.y0;
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, depth), new THREE.MeshLambertMaterial({ color }));
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.copy(w2t((rect.x0 + rect.x1) / 2, (rect.y0 + rect.y1) / 2, height));
  return mesh;
}

const staticOccluders = [];

// Road markings from the world definition: plain rectangles, dashed lines, and crosswalk stripes.
function markingRects(marking) {
  if (marking.type === "rect") return [marking];
  const rects = [];
  if (marking.type === "dashed") {
    const half = marking.width / 2;
    for (let p = marking.from; p < marking.to; p += marking.dash + marking.gap) {
      const q = Math.min(p + marking.dash, marking.to);
      rects.push(marking.axis === "x"
        ? { x0: p, y0: marking.at - half, x1: q, y1: marking.at + half }
        : { x0: marking.at - half, y0: p, x1: marking.at + half, y1: q });
    }
  } else if (marking.type === "stripes") {
    const [start, end] = marking.along === "y" ? [marking.y0, marking.y1] : [marking.x0, marking.x1];
    for (let p = start; p + marking.stripe <= end + 1e-6; p += marking.stripe + marking.gap) {
      rects.push(marking.along === "y"
        ? { x0: marking.x0, y0: p, x1: marking.x1, y1: p + marking.stripe }
        : { x0: p, y0: marking.y0, x1: p + marking.stripe, y1: marking.y1 });
    }
  }
  return rects;
}

function buildWorld(world, profiles) {
  const b = world.bounds;
  const [fogNear, fogFar] = world.fog ?? [80, 260];
  scene.fog.near = fogNear;
  scene.fog.far = fogFar;
  scene.add(flatRect({ x0: b.x0 - 300, y0: b.y0 - 300, x1: b.x1 + 300, y1: b.y1 + 300 }, 0x7d9a5c, -0.02));
  scene.add(flatRect(b, 0xbfbfb8, 0));
  for (const road of world.roads) scene.add(flatRect(road, 0x4a4d52, 0.01));
  for (const marking of world.markings ?? []) {
    for (const rect of markingRects(marking)) scene.add(flatRect(rect, 0xf2f2f2, 0.02));
  }

  const texture = windowTexture();
  for (const bld of world.buildings) {
    const width = bld.x1 - bld.x0, depth = bld.y1 - bld.y0;
    const tex = texture.clone();
    tex.repeat.set(Math.max(1, Math.round(width / 4)), Math.max(1, Math.round(bld.height / 3.5)));
    tex.needsUpdate = true;
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, bld.height, depth),
      new THREE.MeshLambertMaterial({ color: bld.color, map: tex }));
    mesh.position.copy(w2t((bld.x0 + bld.x1) / 2, (bld.y0 + bld.y1) / 2, bld.height / 2));
    scene.add(mesh);
    staticOccluders.push(mesh);
  }
  for (const parked of world.parked ?? []) {
    const model = buildVehicle(profiles[parked.profile]);
    model.position.copy(w2t(parked.x, parked.y));
    model.rotation.y = parked.yaw;
    scene.add(model);
    staticOccluders.push(...occluders(model));
  }
}

// ---------------------------------------------------------------- road users
const FOV = 70;
const VISION_ASPECT = 16 / 9;
// A cyclist looks slightly down over the handlebar and basket with a wider view than a seated driver.
const EYE_VIEW = { bicycle: { fov: 75, pitch: -0.25 } };
const DEFAULT_VIEW = { fov: FOV, pitch: 0 };

class RoadUser {
  constructor(role) {
    this.role = role;
    this.root = new THREE.Group();
    this.eye = new THREE.Object3D();
    this.camera = new THREE.PerspectiveCamera(FOV, 16 / 9, 0.05, 600);
    this.camera.rotation.order = "YXZ"; // turn to face the vehicle's front, then pitch
    this.camera.rotation.y = -Math.PI / 2; // cameras look down -Z; the vehicle faces +X
    this.camera.layers.enable(RIDER_LAYER);
    this.eye.add(this.camera);
    this.root.add(this.eye);
    this.model = null;
    this.profile = null;
    this.state = null;
    this.shown = null;
    scene.add(this.root);
  }

  setProfile(profile) {
    if (this.profile?.id === profile.id) return;
    if (this.model) this.root.remove(this.model);
    this.profile = profile;
    this.model = buildVehicle(profile);
    this.root.add(this.model);
    const [ex, ey, ez] = profile.eye;
    this.eye.position.set(ex, ez, -ey);
    const view = EYE_VIEW[profile.kind] ?? DEFAULT_VIEW;
    this.camera.fov = view.fov;
    this.camera.rotation.x = view.pitch;
    this.camera.updateProjectionMatrix();
    // Only a cyclist's own eye skips the rider's body; every other camera sees it.
    if (profile.kind === "bicycle") this.camera.layers.disable(RIDER_LAYER);
    else this.camera.layers.enable(RIDER_LAYER);
  }

  update(dt) {
    if (!this.state) return;
    const s = this.state;
    if (!this.shown || Math.hypot(s.x - this.shown.x, s.y - this.shown.y) > 5) {
      this.shown = { x: s.x, y: s.y, yaw: s.yaw, look: s.look ?? 0 };
    }
    const k = Math.min(1, dt * 15);
    const dyaw = Math.atan2(Math.sin(s.yaw - this.shown.yaw), Math.cos(s.yaw - this.shown.yaw));
    this.shown.x += (s.x - this.shown.x) * k;
    this.shown.y += (s.y - this.shown.y) * k;
    this.shown.yaw += dyaw * k;
    this.shown.look += ((s.look ?? 0) - this.shown.look) * k;
    this.root.position.copy(w2t(this.shown.x, this.shown.y));
    this.root.rotation.y = this.shown.yaw;
    this.eye.rotation.y = this.shown.look;
  }
}

const users = { bike: new RoadUser("bike"), car: new RoadUser("car") };
let oncoming = null; // scenario vehicle (bus), created when the state carries it
const partnerOf = (role) => (role === "bike" ? "car" : "bike");
// The cyclist can turn to look behind (shoulder check); a seated driver turns less.
const lookLimit = () => (myRole === "bike" ? 2.8 : 1.6);

// Can `viewer` see any target point of `target`? Field of view and line of sight both count.
const raycaster = new THREE.Raycaster();
raycaster.layers.enableAll(); // the rider's body occludes for others even on RIDER_LAYER
const visionCamera = new THREE.PerspectiveCamera(FOV, VISION_ASPECT, 0.05, 600);
const frustum = new THREE.Frustum();
const matrix = new THREE.Matrix4();

function canSee(viewer, target) {
  if (!viewer.model || !target.model) return false;
  scene.updateMatrixWorld();
  viewer.camera.getWorldPosition(visionCamera.position);
  viewer.camera.getWorldQuaternion(visionCamera.quaternion);
  visionCamera.fov = viewer.camera.fov;
  visionCamera.updateProjectionMatrix();
  visionCamera.updateMatrixWorld();
  matrix.multiplyMatrices(visionCamera.projectionMatrix, visionCamera.matrixWorldInverse);
  frustum.setFromProjectionMatrix(matrix);
  const blockers = [...staticOccluders, ...occluders(viewer.model, { forOwnEye: true })];
  if (oncoming?.model) blockers.push(...occluders(oncoming.model));
  const origin = visionCamera.position;
  for (const local of targetPoints(target.profile)) {
    const point = target.root.localToWorld(new THREE.Vector3(...local));
    if (!frustum.containsPoint(point)) continue;
    const direction = point.clone().sub(origin);
    const distance = direction.length();
    raycaster.set(origin, direction.normalize());
    raycaster.far = distance - 0.05;
    if (raycaster.intersectObjects(blockers, false).length === 0) return true;
  }
  return false;
}

// ---------------------------------------------------------------- HUD, PiP, minimap
const mapCamera = new THREE.OrthographicCamera(-30, 30, 30, -30, 1, 200);
mapCamera.up.set(0, 0, -1); // north (world +y, three -z) at the top
mapCamera.layers.enable(RIDER_LAYER);
let profiles = {};
let latest = null;
let lastVisibilityCheck = 0;

function badge(el, ok, yes, no) {
  el.className = `badge ${ok === null ? "mute" : ok ? "ok" : "ng"}`;
  el.textContent = ok === null ? "-" : ok ? yes : no;
}

function updateHud(now) {
  if (!latest) return;
  const viewRole = myRole in users ? myRole : "bike";
  const me = users[viewRole], partner = users[partnerOf(viewRole)];
  const speed = Math.abs(me.state?.speed ?? 0) * 3.6;
  $("speed").textContent = speed.toFixed(0);
  $("me-label").textContent = me.profile ? `${LABEL[viewRole]}（${me.profile.label}）` : LABEL[viewRole];
  const occupied = latest.occupied?.[partner.role];
  const autopilot = partner.role === "bike" && latest.scenario?.bike_autopilot;
  $("partner-label").textContent = `${partner.profile?.label ?? "-"}${occupied ? "" : autopilot ? "（自動走行）" : "（未参加）"}`;
  if (me.state && partner.state) {
    $("distance").textContent = Math.hypot(me.state.x - partner.state.x, me.state.y - partner.state.y).toFixed(1);
  }
  if (now - lastVisibilityCheck > 150) {
    lastVisibilityCheck = now;
    badge($("seen-by-partner"), me.model && partner.model ? canSee(partner, me) : null, "見えている", "見えていない（死角）");
    badge($("i-see-partner"), me.model && partner.model ? canSee(me, partner) : null, "見えている", "見えていない");
  }
  const contact = latest.contact?.[viewRole];
  $("contact").style.display = contact ? "block" : "none";
  $("contact").textContent = contact === "vehicle" ? "接触！" : "障害物に衝突";
  updateScenarioPanel();
}

const PHASE_LABEL = { waiting: "待機中（車が走り出すと接近）", approaching: "接近中", leaving: "通過", finished: "終了（リセットで再開）" };

function updateScenarioPanel() {
  const scenario = latest.scenario ?? {};
  $("scenario-name").textContent = scenario.name ?? "-";
  const toggle = $("oncoming-toggle");
  toggle.hidden = !scenario.has_oncoming;
  toggle.textContent = `対向バス: ${scenario.oncoming_enabled ? "あり" : "なし"}`;
  toggle.classList.toggle("off", !scenario.oncoming_enabled);
  $("oncoming-phase").textContent = scenario.has_oncoming && scenario.oncoming_enabled
    ? `バス: ${PHASE_LABEL[scenario.phase] ?? scenario.phase}` : "";
  const passing = latest.passing ?? {};
  const fmt = (gap) => (gap === null || gap === undefined ? "-" : `${gap.toFixed(2)} m`);
  $("passing-now").textContent = passing.active ? `追い抜き中: 側方間隔 ${fmt(passing.gap)}（最小 ${fmt(passing.min_gap)}）` : "";
  const limit = passing.min_clearance_m ?? 1.0;
  const warning = $("passing-warning");
  warning.hidden = !passing.warning;
  warning.textContent = passing.warning ? `⚠ 側方間隔 ${limit} m 未満` : "";
  const last = passing.last;
  const violation = last ? last.min_gap < limit : false;
  $("passing-last").classList.toggle("violation", violation);
  $("passing-last").textContent = last
    ? `前回の追い抜き: 最小側方間隔 ${fmt(last.min_gap)}${violation ? `（${limit} m 未満：違反）` : ""} / ${last.speed_kmh.toFixed(0)} km/h / 対向バス ${last.oncoming_near ? "あり" : "なし"}`
    : "";
}

$("oncoming-toggle").addEventListener("click", (e) => {
  send({ type: "scenario", oncoming: !(latest?.scenario?.oncoming_enabled) });
  e.currentTarget.blur();
});
$("reset-scenario").addEventListener("click", (e) => {
  send({ type: "reset" });
  e.currentTarget.blur();
});

function layout() {
  const W = innerWidth, H = innerHeight;
  const pipW = Math.round(Math.min(W * 0.36, 560));
  const pipH = Math.round(pipW * 9 / 16);
  const pip = { x: W - pipW - 16, y: 16, w: pipW, h: pipH };
  const mapSize = Math.round(Math.min(220, W * 0.22));
  const map = { x: 16, y: H - mapSize - 16, w: mapSize, h: mapSize };
  for (const [id, r] of [["pip-frame", pip], ["map-frame", map]]) {
    Object.assign($(id).style, { left: `${r.x - 2}px`, top: `${r.y - 2}px`, width: `${r.w}px`, height: `${r.h}px` });
  }
  Object.assign($("pip-label").style, { left: `${pip.x + 6}px`, top: `${pip.y + pip.h - 28}px` });
  return { W, H, pip, map };
}

function renderView(camera, rect, H) {
  renderer.setViewport(rect.x, H - rect.y - rect.h, rect.w, rect.h);
  renderer.setScissor(rect.x, H - rect.y - rect.h, rect.w, rect.h);
  if (camera.isPerspectiveCamera) {
    camera.aspect = rect.w / rect.h;
    camera.updateProjectionMatrix();
  }
  renderer.render(scene, camera);
}

let previous = performance.now();
function frame(now) {
  const dt = Math.min(0.1, (now - previous) / 1000);
  previous = now;
  for (const user of Object.values(users)) user.update(dt);
  oncoming?.update(dt);
  const { W, H, pip, map } = layout();
  const mainRole = myRole in users ? myRole : "bike";
  const [mainUser, pipUser] = swapped ? [users[partnerOf(mainRole)], users[mainRole]]
    : [users[mainRole], users[partnerOf(mainRole)]];
  $("pip-label").textContent = !(myRole in users) ? `${LABEL[pipUser.role]}の視点`
    : pipUser.role === mainRole ? "自分の視点" : `相手（${LABEL[pipUser.role]}）の視点`;
  renderer.setScissorTest(true);
  renderView(mainUser.camera, { x: 0, y: 0, w: W, h: H }, H);
  renderView(pipUser.camera, pip, H);
  const center = users[mainRole].root.position;
  mapCamera.position.set(center.x, 100, center.z);
  mapCamera.lookAt(center.x, 0, center.z);
  renderView(mapCamera, map, H);
  updateHud(now);
  requestAnimationFrame(frame);
}

addEventListener("resize", () => renderer.setSize(innerWidth, innerHeight));

// ---------------------------------------------------------------- input
const keys = new Set();
addEventListener("keydown", (e) => {
  if (e.target.tagName === "SELECT") return;
  keys.add(e.code);
  if (e.code === "KeyV") swapped = !swapped;
  if (e.code === "KeyR") send({ type: "respawn" });
  if (e.code === "KeyC") control.look = 0;
  if (e.code.startsWith("Arrow")) e.preventDefault();
});
addEventListener("keyup", (e) => keys.delete(e.code));
addEventListener("blur", () => keys.clear());

const control = { steer: 0, look: 0 };
const pressed = (...codes) => codes.some((c) => keys.has(c));

function readInput(dt) {
  let throttle = pressed("KeyW", "ArrowUp") ? 1 : 0;
  let brake = pressed("KeyS", "ArrowDown") ? 1 : 0;
  let steerTarget = (pressed("KeyA", "ArrowLeft") ? 1 : 0) - (pressed("KeyD", "ArrowRight") ? 1 : 0);
  let lookAxis = (pressed("KeyQ") ? 1 : 0) - (pressed("KeyE") ? 1 : 0);
  let lookDirect = null;
  const pad = [...(navigator.getGamepads?.() ?? [])].find((p) => p);
  if (pad) {
    const dead = (v) => (Math.abs(v) < 0.12 ? 0 : v);
    if (dead(pad.axes[0])) steerTarget = -pad.axes[0];
    throttle = Math.max(throttle, pad.buttons[7]?.value ?? 0);
    brake = Math.max(brake, pad.buttons[6]?.value ?? 0);
    if (dead(pad.axes[2] ?? 0)) lookDirect = -pad.axes[2] * lookLimit();
  }
  const rate = steerTarget === 0 ? 3 : 2;
  control.steer += Math.max(-rate * dt, Math.min(rate * dt, steerTarget - control.steer));
  if (pad && Math.abs(pad.axes[0]) >= 0.12) control.steer = steerTarget;
  const limit = lookLimit();
  control.look = lookDirect ?? Math.max(-limit, Math.min(limit, control.look + lookAxis * 1.5 * dt));
  // Holding the brake at a standstill backs up slowly.
  const speed = users[myRole]?.state?.speed ?? 0;
  if (brake > 0.5 && throttle === 0 && speed < 0.2) {
    throttle = -0.6;
    brake = 0;
  }
  return { type: "input", throttle, brake, steer: control.steer, look: control.look };
}

// ---------------------------------------------------------------- networking
let socket = null;
const send = (message) => socket?.readyState === WebSocket.OPEN && socket.send(JSON.stringify(message));
let worldBuilt = false;

function connect() {
  socket = new WebSocket(wsUrl);
  socket.onopen = () => {
    statusEl.textContent = `接続済み: ${wsUrl}`;
    if (myRole) send({ type: "join", role: myRole });
    else send({ type: "join", role: "spectator" });
  };
  socket.onclose = () => {
    statusEl.textContent = "切断されました。再接続中…";
    setTimeout(connect, 1500);
  };
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "welcome") onWelcome(message);
    else if (message.type === "state") onState(message);
  };
}

function onWelcome(message) {
  profiles = Object.fromEntries(message.profiles.map((p) => [p.id, p]));
  if (!worldBuilt) {
    buildWorld(message.world, profiles);
    worldBuilt = true;
  }
  myRole = message.role;
  statusEl.textContent = `${LABEL[myRole] ?? myRole} として参加中 — ${wsUrl}`;
  const select = $("profile-select");
  select.hidden = myRole !== "car";
  if (myRole === "car" && select.options.length === 0) {
    // Only passenger cars and the truck are selectable; the bus is a scenario vehicle.
    for (const p of message.profiles.filter((p) => p.kind === "car" || p.kind === "truck")) {
      select.add(new Option(p.label, p.id));
    }
  }
}

function onState(message) {
  latest = message;
  for (const role of ROLES) {
    const s = message[role];
    if (!s) continue;
    users[role].state = s;
    if (profiles[s.profile]) users[role].setProfile(profiles[s.profile]);
  }
  if (message.oncoming && profiles[message.oncoming.profile]) {
    oncoming ??= new RoadUser("oncoming");
    oncoming.state = message.oncoming;
    oncoming.setProfile(profiles[message.oncoming.profile]);
  }
  const select = $("profile-select");
  if (myRole === "car" && document.activeElement !== select) select.value = message.car_profile_request;
}

$("profile-select").addEventListener("change", (e) => {
  send({ type: "profile", id: e.target.value });
  statusEl.textContent = "車種は停止中に切り替わります";
  e.target.blur();
});

let lastSend = performance.now();
setInterval(() => {
  const now = performance.now();
  const dt = (now - lastSend) / 1000;
  lastSend = now;
  if (myRole in users) send(readInput(dt));
}, 33);

if (!myRole) {
  $("lobby").hidden = false;
  for (const button of document.querySelectorAll("#lobby button")) {
    button.addEventListener("click", () => {
      myRole = button.dataset.role;
      params.set("role", myRole);
      history.replaceState(null, "", `?${params}`);
      $("lobby").hidden = true;
      send({ type: "join", role: myRole });
    });
  }
}

connect();
requestAnimationFrame(frame);
