import * as THREE from 'three';
import { HANGANG_COURSE, bridgeArchSegments, bridgeSolids, hangangHeight } from '../config/hangangCourse.js';

// Conservative part envelopes in the C172 model frame. Covers fixed landing gear.
// These deliberately have a small safety margin; not triangle-exact contact.
export const C172_PARTS = [
  ['fuselage', [-0.65, -0.55, -2.9], [0.65, 1.1, 4.6]],
  ['wing', [-5.8, 0.70, -1.3], [5.8, 1.15, 1]],
  ['tail', [-2.1, 0.3, 3.1], [2.1, 0.85, 4.6]],
  ['fin', [-0.3, 0.5, 3], [0.3, 2.4, 4.6]],
  ['gear', [-1.3, -1.65, -1.8], [1.3, -0.3, 1]],
].map(([name, min, max]) => ({ name, box: new THREE.Box3(new THREE.Vector3(...min), new THREE.Vector3(...max)) }));
const parts = C172_PARTS;
const PAD = 0.15;

// Oriented box: centre, three orthonormal axes and the half extent along each.
const makeObb = () => ({ center: new THREE.Vector3(), axes: [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)], half: [0, 0, 0] });
export function obbFromBox(box, obb = makeObb()) {
  box.getCenter(obb.center);
  obb.half[0] = (box.max.x - box.min.x) / 2; obb.half[1] = (box.max.y - box.min.y) / 2; obb.half[2] = (box.max.z - box.min.z) / 2;
  return obb;
}
// Fills `box` with the world AABB of `obb`.
function obbBounds(obb, box) {
  const { axes: [a, b, c], half: [x, y, z] } = obb;
  const ex = Math.abs(a.x) * x + Math.abs(b.x) * y + Math.abs(c.x) * z;
  const ey = Math.abs(a.y) * x + Math.abs(b.y) * y + Math.abs(c.y) * z;
  const ez = Math.abs(a.z) * x + Math.abs(b.z) * y + Math.abs(c.z) * z;
  box.min.set(obb.center.x - ex, obb.center.y - ey, obb.center.z - ez);
  box.max.set(obb.center.x + ex, obb.center.y + ey, obb.center.z + ez);
  return box;
}
// Separating-axis test between two oriented boxes (face axes, then edge cross products).
const R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], AR = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], T = [0, 0, 0], d = new THREE.Vector3();
export function obbOverlap(a, b) {
  const A = a.axes, B = b.axes, ea = a.half, eb = b.half;
  d.subVectors(b.center, a.center);
  for (let i = 0; i < 3; i++) {
    T[i] = d.dot(A[i]);
    for (let j = 0; j < 3; j++) { R[i][j] = A[i].dot(B[j]); AR[i][j] = Math.abs(R[i][j]) + 1e-9; }
  }
  for (let i = 0; i < 3; i++) if (Math.abs(T[i]) > ea[i] + eb[0] * AR[i][0] + eb[1] * AR[i][1] + eb[2] * AR[i][2]) return false;
  for (let j = 0; j < 3; j++) {
    if (Math.abs(T[0] * R[0][j] + T[1] * R[1][j] + T[2] * R[2][j]) > ea[0] * AR[0][j] + ea[1] * AR[1][j] + ea[2] * AR[2][j] + eb[j]) return false;
  }
  for (let i = 0; i < 3; i++) {
    const i1 = (i + 1) % 3, i2 = (i + 2) % 3;
    for (let j = 0; j < 3; j++) {
      const j1 = (j + 1) % 3, j2 = (j + 2) % 3;
      if (Math.abs(T[i2] * R[i1][j] - T[i1] * R[i2][j]) >
        ea[i1] * AR[i2][j] + ea[i2] * AR[i1][j] + eb[j1] * AR[i][j2] + eb[j2] * AR[i][j1]) return false;
    }
  }
  return true;
}

// Solid contact with the Han River bridges, Nodeulseom, its pavilion, banks and water.
// Free flight: no course, gates or flight envelope are judged here (#391).
export class HangangCollision {
  constructor(course = HANGANG_COURSE) {
    if (course.bridges.some(b => ![b.x, b.z, b.yaw].every(Number.isFinite))) throw new Error('Invalid Hangang course');
    this.course = course;
    this.bridges = course.bridges.map(b => {
      const transform = new THREE.Matrix4().compose(new THREE.Vector3(b.x, 0, b.z),
        new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), b.yaw), new THREE.Vector3(1, 1, 1));
      const boxes = bridgeSolids(b).map(s => {
        const box = new THREE.Box3().setFromCenterAndSize(new THREE.Vector3(...s.center), new THREE.Vector3(...s.size));
        return { ...s, box, obb: obbFromBox(box) };
      });
      const arches = bridgeArchSegments(b).map(s => {
        const obb = { center: new THREE.Vector3(...s.center), axes: s.axes.map(v => new THREE.Vector3(...v)), half: [...s.half] };
        return { ...s, obb, box: obbBounds(obb, new THREE.Box3()) };
      });
      return { ...b, inverse: transform.invert(), solids: [...boxes, ...arches] };
    });
    this.matrix = new THREE.Matrix4(); this.localMatrix = new THREE.Matrix4();
    this.p = new THREE.Vector3(); this.q = new THREE.Quaternion(); this.unit = new THREE.Vector3(1, 1, 1);
    this.box = new THREE.Box3(); this.local = new THREE.Vector3();
    this.obb = makeObb(); this.cell = makeObb();
    this.reset();
  }
  reset() { this.contact = null; this.previous = null; }
  // Sets this.obb to an aircraft part in `matrix`'s frame, padded, and this.box to its AABB.
  setPart(part, matrix) {
    const obb = this.obb;
    part.box.getCenter(obb.center).applyMatrix4(matrix);
    matrix.extractBasis(...obb.axes);
    obb.half[0] = (part.box.max.x - part.box.min.x) / 2 + PAD;
    obb.half[1] = (part.box.max.y - part.box.min.y) / 2 + PAD;
    obb.half[2] = (part.box.max.z - part.box.min.z) / 2 + PAD;
    return obbBounds(obb, this.box);
  }
  // Pavilion cells store the highest surface in each square, so each is a column up to
  // that height. `box` (the AABB) picks the candidate cells; the oriented part box must
  // reach into a column, so a banked wing's raised side no longer counts (#395).
  pavilionHit(box, obb = obbFromBox(box)) {
    const island = this.course.nodeulseom, field = island.pavilion.heights, column = this.cell;
    if (box.min.y > field.top) return false;
    const cell = v => Math.floor(v / field.cell);
    const i0 = Math.max(0, cell(box.min.x - island.x - field.x0)), i1 = Math.min(field.cols - 1, cell(box.max.x - island.x - field.x0));
    const j0 = Math.max(0, cell(box.min.z - island.z - field.z0)), j1 = Math.min(field.rows - 1, cell(box.max.z - island.z - field.z0));
    const bottom = box.min.y - 1;
    column.half[0] = column.half[2] = field.cell / 2;
    for (let j = j0; j <= j1; j++) {
      for (let i = i0; i <= i1; i++) {
        const top = field.heights[j * field.cols + i];
        if (top < box.min.y) continue;
        column.center.set(island.x + field.x0 + (i + 0.5) * field.cell, (top + bottom) / 2, island.z + field.z0 + (j + 0.5) * field.cell);
        column.half[1] = (top - bottom) / 2;
        if (obbOverlap(obb, column)) return true;
      }
    }
    return false;
  }
  // The island tapers upward, so test its footprint at the box's lowest point, where
  // it is widest. Its 24-sided mesh sits just inside this ellipse.
  nodeulseomHit(box, obb = obbFromBox(box)) {
    const island = this.course.nodeulseom;
    if (!island) return null;
    if (this.pavilionHit(box, obb)) return 'pavilion';
    if (box.min.y > island.height) return null;
    const r = island.bottomRadius + (island.topRadius - island.bottomRadius) * Math.max(0, box.min.y) / island.height;
    const dx = Math.max(box.min.x - island.x, 0, island.x - box.max.x) / r;
    const dz = Math.max(box.min.z - island.z, 0, island.z - box.max.z) / (r * island.zScale);
    return dx * dx + dz * dz <= 1 ? 'island' : null;
  }
  // Returns the first contact, then keeps it until reset().
  update(position, quaternion) {
    if (this.contact) return this.contact;
    // Malformed samples are skipped; the next valid one is tested on its own.
    if (![...position.toArray(), ...quaternion.toArray()].every(Number.isFinite) ||
        Math.abs(quaternion.lengthSq() - 1) > 0.01) { this.previous = null; return null; }
    const c = this.course;
    let previous = this.previous;
    // Subdivide both translation and rotation; padding covers between-sample motion.
    let count = previous ? Math.ceil(Math.max(1, previous.p.distanceTo(position) / 0.2, previous.q.angleTo(quaternion) / 0.015)) : 1;
    // A jump this large is a respawn or teleport, not flight; test only the new pose.
    if (count > 512) { previous = null; count = 1; }
    for (let sample = 1; sample <= count && !this.contact; sample++) {
      if (previous) { this.p.copy(previous.p).lerp(position, sample / count); this.q.copy(previous.q).slerp(quaternion, sample / count); }
      else { this.p.copy(position); this.q.copy(quaternion); }
      this.matrix.compose(this.p, this.q, this.unit);
      for (const part of parts) {
        this.setPart(part, this.matrix);
        const structure = this.box.min.y <= c.waterLevel || this.box.min.y <= hangangHeight(this.p.x, this.p.z)
          ? 'water' : this.nodeulseomHit(this.box, this.obb);
        if (structure) { this.contact = { aircraftPart: part.name, structure, position: this.p.toArray() }; break; }
      }
      for (let i = 0; i < this.bridges.length && !this.contact; i++) {
        const bridge = this.bridges[i];
        this.local.copy(this.p).applyMatrix4(bridge.inverse);
        // Conservative broadphase includes the aircraft's furthest extremity.
        if (Math.abs(this.local.z) > 28) continue;
        this.localMatrix.multiplyMatrices(bridge.inverse, this.matrix);
        for (const part of parts) {
          this.setPart(part, this.localMatrix);
          const hit = bridge.solids.find(s => s.box.intersectsBox(this.box) && obbOverlap(this.obb, s.obb));
          if (hit) { this.contact = { bridge: bridge.id, aircraftPart: part.name, structure: hit.part, position: this.p.toArray() }; break; }
        }
      }
    }
    this.previous ??= { p: new THREE.Vector3(), q: new THREE.Quaternion() };
    this.previous.p.copy(position); this.previous.q.copy(quaternion);
    return this.contact;
  }
}
