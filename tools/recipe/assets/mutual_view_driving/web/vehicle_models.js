// Simplified road-user bodies built from the gateway's vehicle profiles.
// Local frame: +X forward, +Y up, -Z left (world +y). The origin is the footprint centre on the ground.
// Every solid part is an occluder for visibility checks; glass is not.
import * as THREE from "three";

// Cabin geometry per profile: hood length from the front bumper to the windshield base,
// windshield horizontal run, beltline height, and the length behind the cabin (trunk).
const CABIN = {
  sedan: { hood: 1.3, rake: 0.85, belt: 0.95, trunk: 0.95 },
  suv: { hood: 1.15, rake: 0.6, belt: 1.15, trunk: 0.25 },
  minivan: { hood: 0.75, rake: 0.95, belt: 1.05, trunk: 0.15 },
};

function box(w, h, d, material, x, y, z) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), material);
  mesh.position.set(x, y, z);
  return mesh;
}

// A thin beam between two local points (used for pillars and frames).
function beam(a, b, thickness, material) {
  const start = new THREE.Vector3(...a);
  const end = new THREE.Vector3(...b);
  const length = start.distanceTo(end);
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(thickness, length, thickness), material);
  mesh.position.copy(start).add(end).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), end.clone().sub(start).normalize());
  return mesh;
}

function wheel(radius, width, x, z, material) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, width, 20), material);
  mesh.rotation.x = Math.PI / 2;
  mesh.position.set(x, radius, z);
  return mesh;
}

const tire = new THREE.MeshLambertMaterial({ color: 0x1d1d1d });
const glass = new THREE.MeshLambertMaterial({ color: 0x9ec9e8, transparent: true, opacity: 0.18, depthWrite: false });
const trim = new THREE.MeshLambertMaterial({ color: 0x222222 });
const dash = new THREE.MeshLambertMaterial({ color: 0x2b2b2b });

// Rider parts the rider's own eye camera does not render (it sits inside the head).
export const RIDER_LAYER = 1;

function basketTexture() {
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = 64;
  const g = canvas.getContext("2d");
  g.strokeStyle = "#cfd3d6";
  g.lineWidth = 3;
  for (let p = 2; p < 64; p += 10) {
    g.beginPath(); g.moveTo(p, 0); g.lineTo(p, 64); g.stroke();
    g.beginPath(); g.moveTo(0, p); g.lineTo(64, p); g.stroke();
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

// City bicycle (mamachari): swept-back handlebar, front wire basket, lamp, and fenders.
function buildBicycle(profile) {
  const group = new THREE.Group();
  const frame = new THREE.MeshLambertMaterial({ color: profile.color });
  const chrome = new THREE.MeshLambertMaterial({ color: 0xb8bcc0 });
  const half = profile.wheelbase / 2;
  for (const x of [half, -half]) {
    const rim = new THREE.Mesh(new THREE.TorusGeometry(0.33, 0.035, 8, 24), tire);
    rim.position.set(x, 0.34, 0);
    group.add(rim);
    // Fender over the upper half of the wheel.
    const fender = new THREE.Mesh(new THREE.TorusGeometry(0.37, 0.025, 4, 16, Math.PI * 0.8), chrome);
    fender.position.set(x, 0.34, 0);
    fender.rotation.z = Math.PI * 0.1;
    group.add(fender);
  }
  const headTop = [half - 0.1, 0.95, 0];
  group.add(beam([-half, 0.34, 0], [-0.05, 0.42, 0], 0.04, frame));           // chain stay
  group.add(beam([-0.05, 0.42, 0], headTop, 0.045, frame));                     // low step-through tube
  group.add(beam([-0.05, 0.42, 0], [-0.22, 0.88, 0], 0.04, frame));            // seat tube
  group.add(beam([half, 0.34, 0], headTop, 0.035, chrome));                     // fork
  group.add(box(0.26, 0.06, 0.15, trim, -0.24, 0.92, 0));                       // saddle
  // Stem and swept-back handlebar with grips and a bell.
  const bar = [0.44, 1.07];
  group.add(beam(headTop, [bar[0], bar[1], 0], 0.035, chrome));
  group.add(beam([bar[0], bar[1], -0.2], [bar[0], bar[1], 0.2], 0.025, chrome));
  for (const side of [-1, 1]) {
    group.add(beam([bar[0], bar[1], side * 0.2], [0.34, bar[1] + 0.02, side * 0.29], 0.025, chrome));
    group.add(beam([0.34, bar[1] + 0.02, side * 0.29], [0.24, bar[1] + 0.02, side * 0.31], 0.04, trim));
    group.add(beam([0.4, bar[1] + 0.03, side * 0.24], [0.33, bar[1] + 0.05, side * 0.34], 0.012, trim)); // brake lever
  }
  const bell = new THREE.Mesh(new THREE.SphereGeometry(0.022, 10, 6, 0, Math.PI * 2, 0, Math.PI / 2), chrome);
  bell.position.set(bar[0] - 0.02, bar[1] + 0.02, -0.14);
  group.add(bell);
  // Front wire basket above the front wheel, open at the top.
  const wire = new THREE.MeshLambertMaterial({ map: basketTexture(), transparent: true, alphaTest: 0.3,
    side: THREE.DoubleSide, color: 0xffffff });
  const bx0 = 0.5, bx1 = 0.84, bz = 0.2, by0 = 0.8, by1 = 1.04;
  const panels = [
    [bx1 - bx0, by1 - by0, 0.005, (bx0 + bx1) / 2, (by0 + by1) / 2, bz],
    [bx1 - bx0, by1 - by0, 0.005, (bx0 + bx1) / 2, (by0 + by1) / 2, -bz],
    [0.005, by1 - by0, 2 * bz, bx0, (by0 + by1) / 2, 0],
    [0.005, by1 - by0, 2 * bz, bx1, (by0 + by1) / 2, 0],
    [bx1 - bx0, 0.005, 2 * bz, (bx0 + bx1) / 2, by0, 0],
  ];
  for (const [w, h, d, x, y, z] of panels) group.add(box(w, h, d, wire, x, y, z));
  for (const [x, z] of [[bx0, bz], [bx0, -bz], [bx1, bz], [bx1, -bz]]) group.add(beam([x, by1, z], [x, by0, z], 0.012, chrome));
  group.add(beam([bx0, by1, -bz], [bx0, by1, bz], 0.012, chrome));
  group.add(beam([bx1, by1, -bz], [bx1, by1, bz], 0.012, chrome));
  group.add(beam([bx0, by1, bz], [bx1, by1, bz], 0.012, chrome));
  group.add(beam([bx0, by1, -bz], [bx1, by1, -bz], 0.012, chrome));
  const lamp = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.05, 0.08, 12),
    new THREE.MeshLambertMaterial({ color: 0xfff3c4, emissive: 0x665a20 }));
  lamp.rotation.z = Math.PI / 2;
  lamp.position.set(bx1 + 0.05, by0 - 0.04, 0);
  group.add(lamp);

  // Rider. Torso, upper arms, head, and helmet are hidden from the rider's own eye (RIDER_LAYER);
  // forearms and hands stay visible on the grips, as a cyclist sees them.
  const clothes = new THREE.MeshLambertMaterial({ color: 0x3f51b5 });
  const skin = new THREE.MeshLambertMaterial({ color: 0xf0c8a0 });
  const [ex, , ez] = profile.eye;
  const shoulder = (side) => [ex + 0.02, ez - 0.25, side * 0.2];
  const elbow = (side) => [ex + 0.2, ez - 0.4, side * 0.26];
  const torso = beam([-0.22, 0.95, 0], [ex + 0.02, ez - 0.2, 0], 0.32, clothes);
  const head = new THREE.Mesh(new THREE.SphereGeometry(0.12, 16, 12), skin);
  head.position.set(ex, ez + 0.02, -profile.eye[1]);
  const helmet = new THREE.Mesh(new THREE.SphereGeometry(0.135, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2),
    new THREE.MeshLambertMaterial({ color: 0xffffff }));
  helmet.position.copy(head.position).add(new THREE.Vector3(0, 0.03, 0));
  const hidden = [torso, head, helmet];
  for (const side of [-1, 1]) {
    hidden.push(beam(shoulder(side), elbow(side), 0.09, clothes));
    group.add(beam(elbow(side), [0.26, bar[1] + 0.04, side * 0.3], 0.055, clothes));
    const hand = new THREE.Mesh(new THREE.SphereGeometry(1, 12, 8), skin);
    hand.scale.set(0.05, 0.03, 0.045);
    hand.position.set(0.29, bar[1] + 0.045, side * 0.3);
    group.add(hand);
  }
  for (const part of hidden) {
    part.userData.rider = true;
    part.layers.set(RIDER_LAYER);
    group.add(part);
  }
  // Thighs, so the bicycle has a recognisable silhouette from a car.
  group.add(beam([-0.15, 0.95, -0.12], [0.2, 0.6, -0.12], 0.13, clothes));
  group.add(beam([-0.15, 0.95, 0.12], [0.2, 0.6, 0.12], 0.13, clothes));
  return group;
}

function buildCar(profile) {
  const group = new THREE.Group();
  const paint = new THREE.MeshLambertMaterial({ color: profile.color });
  const c = CABIN[profile.id] ?? CABIN.sedan;
  const L = profile.length, W = profile.width, H = profile.height;
  const front = L / 2, rear = -L / 2, half = W / 2;
  const clearance = 0.25;
  const wsBase = front - c.hood;
  const roofFront = wsBase - c.rake;
  const roofRear = rear + c.trunk + 0.35;
  const cabinRear = rear + c.trunk;

  // Lower body up to the beltline: hood, cabin section (dark interior on top), and trunk.
  const lowerH = c.belt - clearance, lowerY = (c.belt + clearance) / 2;
  group.add(box(front - wsBase, lowerH, W, paint, (front + wsBase) / 2, lowerY, 0));
  const interior = [paint, paint, dash, paint, paint, paint]; // BoxGeometry face order: +x -x +y -y +z -z
  group.add(box(wsBase - cabinRear, lowerH, W, interior, (wsBase + cabinRear) / 2, lowerY, 0));
  group.add(box(cabinRear - rear, lowerH, W, paint, (cabinRear + rear) / 2, lowerY, 0));
  // Roof.
  group.add(box(roofFront - roofRear, 0.07, W - 0.12, paint, (roofFront + roofRear) / 2, H - 0.035, 0));
  // Pillars: A (raked), B, C (raked to the rear).
  const inset = half - 0.06;
  for (const side of [-1, 1]) {
    const z = side * inset;
    group.add(beam([wsBase, c.belt, z], [roofFront, H, z], 0.11, paint));
    group.add(beam([(roofFront + roofRear) / 2 - 0.1, c.belt, z], [(roofFront + roofRear) / 2 - 0.1, H, z], 0.1, paint));
    group.add(beam([cabinRear, c.belt, z], [roofRear, H, z], 0.14, paint));
    // Side mirror just outside the A-pillar base.
    group.add(box(0.12, 0.13, 0.2, trim, wsBase - 0.1, c.belt + 0.08, side * (half + 0.08)));
  }
  // Dashboard below the windshield.
  group.add(box(0.55, 0.12, W - 0.2, dash, wsBase - 0.3, c.belt + 0.04, 0));
  // Glass (not an occluder).
  const windshield = beam([wsBase, c.belt, 0], [roofFront, H, 0], 0.01, glass);
  windshield.geometry = new THREE.BoxGeometry(0.01, windshield.geometry.parameters.height, W - 0.2);
  windshield.userData.glass = true;
  group.add(windshield);
  // Wheels.
  const r = 0.34;
  for (const x of [profile.wheelbase / 2, -profile.wheelbase / 2]) {
    for (const side of [-1, 1]) group.add(wheel(r, 0.24, x, side * (half - 0.15), tire));
  }
  return group;
}

function buildTruck(profile) {
  const group = new THREE.Group();
  const paint = new THREE.MeshLambertMaterial({ color: profile.color });
  const cargo = new THREE.MeshLambertMaterial({ color: 0xd9dde3 });
  const L = profile.length, W = profile.width, H = profile.height;
  const front = L / 2, half = W / 2;
  const cabLength = 2.3, cabTop = 3.2, belt = 2.05, floor = 1.0;
  const cabRear = front - cabLength;
  // Cab-over: the front panel reaches the windshield, so the driver cannot see close to the bumper.
  const interior = [paint, paint, dash, paint, paint, paint]; // dark cab floor seen from the driver seat
  group.add(box(cabLength, belt - 0.45, W, interior, front - cabLength / 2, (belt + 0.45) / 2, 0));
  group.add(box(cabLength - 0.15, 0.1, W - 0.1, paint, front - cabLength / 2 - 0.05, cabTop - 0.05, 0));
  group.add(box(0.12, cabTop - belt, W, paint, cabRear + 0.06, (cabTop + belt) / 2, 0));
  for (const side of [-1, 1]) {
    const z = side * (half - 0.06);
    group.add(beam([front - 0.05, belt, z], [front - 0.15, cabTop, z], 0.14, paint));
    group.add(beam([cabRear + 0.9, belt, z], [cabRear + 0.9, cabTop, z], 0.1, paint));
    // Large mirrors on stays.
    group.add(box(0.1, 0.45, 0.25, trim, front - 0.1, belt + 0.25, side * (half + 0.25)));
  }
  group.add(box(0.5, 0.12, W - 0.3, dash, front - 0.35, belt + 0.04, 0));
  // Cargo box and chassis.
  const cargoLength = L - cabLength - 0.2;
  group.add(box(cargoLength, H - floor, W, cargo, cabRear - 0.2 - cargoLength / 2, (H + floor) / 2, 0));
  group.add(box(L - 0.6, 0.3, 1.0, trim, -0.3, 0.75, 0));
  const r = 0.5;
  const axles = [front - 1.2, front - 1.2 - profile.wheelbase, front - 1.2 - profile.wheelbase - 1.3];
  for (const x of axles) {
    for (const side of [-1, 1]) group.add(wheel(r, 0.3, x, side * (half - 0.25), tire));
  }
  return group;
}

function buildBus(profile) {
  const group = new THREE.Group();
  const paint = new THREE.MeshLambertMaterial({ color: profile.color });
  const white = new THREE.MeshLambertMaterial({ color: 0xf4f6f7 });
  const dark = new THREE.MeshLambertMaterial({ color: 0x1c2833 });
  const L = profile.length, W = profile.width, H = profile.height, half = W / 2;
  const floor = 0.35, windowBottom = 1.25, windowTop = H - 0.35;
  // Lower body, dark window band (tinted side and front glass), and roof.
  group.add(box(L, windowBottom - floor, W, paint, 0, (windowBottom + floor) / 2, 0));
  group.add(box(L - 0.1, windowTop - windowBottom, W - 0.06, dark, 0, (windowTop + windowBottom) / 2, 0));
  group.add(box(L, H - windowTop, W, white, 0, (H + windowTop) / 2, 0));
  // Window pillars along both sides.
  for (let x = -L / 2 + 0.8; x < L / 2 - 0.5; x += 1.5) {
    for (const side of [-1, 1]) group.add(box(0.12, windowTop - windowBottom, 0.04, white, x, (windowTop + windowBottom) / 2, side * half));
  }
  // Destination sign and mirrors at the front.
  group.add(box(0.05, 0.25, W - 0.5, new THREE.MeshLambertMaterial({ color: 0xf39c12 }), L / 2 + 0.01, windowTop - 0.15, 0));
  for (const side of [-1, 1]) group.add(box(0.1, 0.4, 0.2, trim, L / 2 + 0.2, 2.0, side * (half + 0.2)));
  const r = 0.5;
  for (const x of [profile.wheelbase / 2, -profile.wheelbase / 2]) {
    for (const side of [-1, 1]) group.add(wheel(r, 0.3, x, side * (half - 0.25), tire));
  }
  return group;
}

export function buildVehicle(profile) {
  const group = profile.kind === "bicycle" ? buildBicycle(profile)
    : profile.kind === "truck" ? buildTruck(profile)
      : profile.kind === "bus" ? buildBus(profile) : buildCar(profile);
  group.userData.profileId = profile.id;
  return group;
}

// Points on the body used to decide whether this road user is visible to someone.
export function targetPoints(profile) {
  if (profile.kind === "bicycle") {
    return [[profile.eye[0], profile.eye[2], 0], [-0.1, 1.15, 0], [0.3, 0.7, 0],
      [profile.wheelbase / 2, 0.35, 0], [-profile.wheelbase / 2, 0.35, 0]];
  }
  const xs = [profile.length / 2 - 0.1, 0, -profile.length / 2 + 0.1];
  const zs = [-(profile.width / 2 - 0.1), profile.width / 2 - 0.1];
  const ys = [0.6, profile.height - 0.1];
  const points = [];
  for (const x of xs) for (const z of zs) for (const y of ys) points.push([x, y, z]);
  return points;
}

// Occluding meshes of a vehicle (solid parts). The rider is excluded for the rider's own eye.
export function occluders(group, { forOwnEye = false } = {}) {
  const list = [];
  group.traverse((obj) => {
    if (!obj.isMesh || obj.userData.glass) return;
    if (forOwnEye && obj.userData.rider) return;
    list.push(obj);
  });
  return list;
}
