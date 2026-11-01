import test from 'node:test';
import assert from 'node:assert/strict';
import { Vector3, Quaternion, Scene, Box3, Matrix4, Euler } from 'three';
import { HangangCollision, C172_PARTS } from '../src/director/HangangCollision.js';
import { HangangScenario } from '../src/director/HangangScenario.js';
import { HANGANG_COURSE as course, riverCenter } from '../src/config/hangangCourse.js';
import { lobbyBriefing } from '../src/ui/lobbyBriefing.js';
import { setLanguage, t } from '../src/i18n/index.js';

const q = new Quaternion();
function step(collision, x, y, z, attitude = q) {
  return collision.update(new Vector3(x, y, z), attitude);
}
test('flying the river under all three bridges makes no contact', () => {
  const collision = new HangangCollision();
  for (let z = 850; z >= -3800; z -= 2.3) assert.equal(step(collision, riverCenter(z), 19, z), null, `z ${z}`);
});
test('route, direction, altitude band and duration are not judged (#391)', () => {
  // Over the deck, against the course direction, starting past a bridge.
  for (const path of [[[0, 80, 30], [0, 80, -30]], [[0, 19, -30], [0, 19, 30]], [[-120, 19, -3000], [-120, 19, -3800]]]) {
    const collision = new HangangCollision();
    for (const p of path) assert.equal(step(collision, ...p), null, JSON.stringify(path));
  }
  // Ten minutes of slow, high circling far beyond any former time or envelope limit.
  const collision = new HangangCollision();
  for (let i = 0; i < 12000; i++) {
    const a = i * 0.002;
    assert.equal(step(collision, 150 * Math.sin(a), 60, 600 + 150 * Math.cos(a)), null);
  }
});
test('thin deck, wing and fin contacts are detected between frames', () => {
  for (const [x, y] of [[0, 33], [49, 19]]) {
    const collision = new HangangCollision(); step(collision, x, y, 30);
    const contact = step(collision, x, y, -30);
    assert.equal(contact.bridge, 'west'); assert.ok(contact.aircraftPart);
  }
});
// Flies the central opening along its own axis, `offset` metres across the river.
function centralPass(height, offset = 0) {
  const collision = new HangangCollision();
  const b = course.bridges[1];
  const yaw = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), b.yaw);
  const forward = new Vector3(0, 0, -1).applyQuaternion(yaw), across = new Vector3(1, 0, 0).applyQuaternion(yaw);
  for (let s = -200; s <= 200 && !collision.contact; s += 2.3) {
    collision.update(new Vector3(b.x, height, b.z).addScaledVector(across, offset).addScaledVector(forward, s), yaw);
  }
  return collision;
}
test('Nodeulseom island and pavilion under the central bridge are solid', () => {
  for (const height of [11, 14]) assert.equal(centralPass(height).contact?.structure, 'pavilion');
  assert.equal(centralPass(5, 38).contact?.structure, 'island');
  for (const collision of [centralPass(18), centralPass(11, 38)]) assert.equal(collision.contact, null);
});
test('pavilion collides where its mesh stands, not across its bounding box', () => {
  const collision = new HangangCollision(), island = course.nodeulseom, field = island.pavilion.heights;
  const probe = (x, z, y) => collision.nodeulseomHit(new Box3().setFromCenterAndSize(
    new Vector3(island.x + x, y, island.z + z), new Vector3(0.4, 0.4, 0.4)));
  let roofs = 0, open = 0;
  field.heights.forEach((top, k) => {
    const x = field.x0 + (k % field.cols + 0.5) * field.cell, z = field.z0 + (Math.floor(k / field.cols) + 0.5) * field.cell;
    if (top > island.height + 1) {
      roofs++; assert.equal(probe(x, z, top - 0.3), 'pavilion'); assert.equal(probe(x, z, top + 0.3), null);
    } else if (Math.abs(x) < 22.5 && Math.abs(z) < 40) {
      // Inside the former 45 x 80 m box, but only plaza or bare island here.
      open++; assert.equal(probe(x, z, 8), null);
    }
  });
  assert.ok(roofs > 800, `roof cells ${roofs}`); assert.ok(open > 800, `open cells ${open}`);
  assert.ok(field.top > 15 && field.top < 15.5, `roof top ${field.top}`);
  assert.equal(probe(0, 0, field.top + 0.3), null);
});
// Flies bridge-local `from` -> `to` in 1 m steps, nose along the path.
function bridgePass(bridgeIndex, from, to) {
  const b = course.bridges[bridgeIndex], collision = new HangangCollision();
  const toWorld = new Matrix4().makeRotationY(b.yaw).setPosition(b.x, 0, b.z);
  const heading = Math.atan2(-(to[0] - from[0]), -(to[2] - from[2]));
  const attitude = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), b.yaw + heading);
  const a = new Vector3(...from), span = new Vector3(...to).sub(a), steps = Math.ceil(span.length());
  for (let s = 0; s <= steps && !collision.contact; s++) {
    collision.update(a.clone().addScaledVector(span, s / steps).applyMatrix4(toWorld), attitude);
  }
  return collision.contact;
}
test('deck streetlights and arches above the deck are solid (#395)', () => {
  // Along the deck through a row of streetlights, clear of the rails.
  for (const i of [0, 1, 2]) assert.match(bridgePass(i, [-300, 42.5, 9], [300, 42.5, 9])?.structure ?? '', /^lamp/);
  // Same run between the rows touches nothing (the east bridge's towers stand there).
  for (const i of [0, 1]) assert.equal(bridgePass(i, [-300, 42.5, 0], [300, 42.5, 0]), null);
  // Across the central bridge through the arch crowns, and under them.
  assert.equal(bridgePass(1, [0, 75, 40], [0, 75, -40])?.structure, 'arch');
  assert.equal(bridgePass(1, [-100, 58, 40], [-100, 58, -40])?.structure, 'arch');
  assert.equal(bridgePass(1, [0, 70, 40], [0, 70, -40]), null);
  assert.equal(bridgePass(1, [-100, 52, 40], [-100, 52, -40]), null);
  // Along the deck between the two arches; the other bridges have none.
  assert.equal(bridgePass(1, [-200, 60, 0], [200, 60, 0]), null);
  assert.equal(bridgePass(0, [0, 75, 40], [0, 75, -40]), null);
});

// Surface points of an oriented box, about `step` apart.
function obbSurface({ center, axes, half }, step) {
  const points = [];
  for (let f = 0; f < 3; f++) {
    const [u, v] = [(f + 1) % 3, (f + 2) % 3], nu = Math.ceil(2 * half[u] / step), nv = Math.ceil(2 * half[v] / step);
    for (const sign of [-1, 1]) for (let a = 0; a <= nu; a++) for (let b = 0; b <= nv; b++) {
      points.push(center.clone().addScaledVector(axes[f], sign * half[f])
        .addScaledVector(axes[u], (2 * a / nu - 1) * half[u]).addScaledVector(axes[v], (2 * b / nv - 1) * half[v]));
    }
  }
  return points;
}
test('banked parts touch the pavilion only where their surface reaches it (#395)', () => {
  const collision = new HangangCollision(), island = course.nodeulseom, field = island.pavilion.heights;
  const height = (x, z) => {
    const i = Math.floor((x - island.x - field.x0) / field.cell), j = Math.floor((z - island.z - field.z0) / field.cell);
    return i < 0 || j < 0 || i >= field.cols || j >= field.rows ? -Infinity : field.heights[j * field.cols + i];
  };
  // Highest cell within `e` of a point, for the tolerant reference.
  const near = (x, z, e) => Math.max(height(x - e, z - e), height(x + e, z - e), height(x - e, z + e), height(x + e, z + e));
  const step = 0.2, e = 0.2, matrix = new Matrix4(), q = new Quaternion();
  let checked = 0, contacts = 0, aabbOnly = 0;
  for (const bank of [12, 20, 29]) for (const yaw of [0, 0.7, Math.PI / 2]) {
    q.setFromEuler(new Euler(0, yaw, bank * Math.PI / 180, 'YXZ'));
    for (let x = -24; x <= 24; x += 2) for (let z = -24; z <= 24; z += 2) {
      const ground = height(island.x + x, island.z + z);
      for (const lift of [1, 2.5, 4]) {
        matrix.compose(new Vector3(island.x + x, ground + lift, island.z + z), q, new Vector3(1, 1, 1));
        for (const part of C172_PARTS) {
          const box = collision.setPart(part, matrix), hit = collision.pavilionHit(box, collision.obb);
          checked++;
          // The former test: any overlapped cell above the AABB's lowest point.
          const aabbHit = box.min.y <= field.top && [...Array(Math.ceil(box.max.x - box.min.x) + 2)].some((_, a) =>
            [...Array(Math.ceil(box.max.z - box.min.z) + 2)].some((_, b) =>
              height(Math.min(box.min.x + a, box.max.x), Math.min(box.min.z + b, box.max.z)) >= box.min.y));
          if (!aabbHit) { assert.equal(hit, false); continue; }
          const surface = obbSurface(collision.obb, step);
          const strict = surface.some(p => height(p.x, p.z) >= p.y), loose = surface.some(p => near(p.x, p.z, e) >= p.y - e);
          const where = `bank ${bank} yaw ${yaw} at ${x},${z},+${lift} ${part.name}`;
          if (strict) assert.ok(hit, `missed: ${where}`);
          if (hit) assert.ok(loose, `false contact: ${where}`);
          contacts += hit; aabbOnly += !strict;
        }
      }
    }
  }
  assert.ok(contacts > 1000, `contacts ${contacts} of ${checked}`);
  // The grid includes poses the former AABB test failed but the surface never reaches.
  assert.ok(aabbOnly > 100, `AABB-only contacts ${aabbOnly}`);
});
test('malformed samples and teleports never switch collision off', () => {
  const collision = new HangangCollision();
  assert.equal(step(collision, NaN, 19, 100), null);
  assert.equal(step(collision, 0, 19, 100, new Quaternion(0, 0, 0, 2)), null);
  // An 820 m jump tests only the new pose; the following deck strike still counts.
  step(collision, 0, 33, 850); step(collision, 0, 33, 30);
  assert.equal(step(collision, 0, 33, -30)?.structure, 'girder');
});
test('first contact is kept until reset', () => {
  const collision = new HangangCollision();
  const contact = step(collision, 0, 0.5, 600);
  assert.equal(contact.structure, 'water');
  assert.equal(step(collision, 0, 19, 500), contact);
  collision.reset(); assert.equal(collision.contact, null); assert.equal(collision.previous, null);
  assert.equal(step(collision, 0, 19, 500), null);
  assert.throws(() => new HangangCollision({ ...course, bridges: [{ ...course.bridges[0], x: NaN }] }));
});
test('briefing describes free flight and stays out of unrelated airports', () => {
  for (const lang of ['en', 'ko']) {
    setLanguage(lang);
    const briefing = lobbyBriefing('hangang', 'hangang').join();
    assert.ok(briefing.includes('C172'));
    assert.ok(!briefing.includes('01 → 02'));
    assert.ok(!lobbyBriefing('coastal', 'hangang').join().includes(t('hangang.briefing')));
  }
  setLanguage('en');
});

function withSim(run) {
  const prevDoc = globalThis.document;
  globalThis.document = {
    getElementById: () => ({ addEventListener: () => {}, hidden: false }),
    createElement: () => ({ getContext: () => new Proxy({}, { get: () => () => {} }) })
  };
  try {
    run({
      scene: new Scene(),
      activeAirport: 'incheon',
      gameFlow: { state: 'flight' },
      position: new Vector3(),
      quaternion: new Quaternion(),
      linearVelocity: new Vector3(),
      terrain: { mesh: { visible: false } },
      airport: { group: { visible: false } },
      water: { mesh: { visible: true } },
      ships: { group: { visible: false } },
      foliage: { group: { visible: false } },
      neonCity: { group: { visible: false } },
      incheonAirport: {
        group: { visible: false },
        terrain: { mesh: { visible: false } }
      },
      collisionDetector: {
        setEnvironment: () => {},
        terrain: null, airport: null, foliage: null, ships: null, water: null
      },
      sky: { timeOfDay: 12 },
      aero: { windWorld: new Vector3() },
      cameraSystem: { towerPos: new Vector3() },
      setTimeOfDay: () => {}
    });
  } finally {
    if (prevDoc === undefined) delete globalThis.document;
    else globalThis.document = prevDoc;
  }
}

test('HangangScenario stop reconciles active airport terrain visibility', () => {
  withSim(sim => {
    const hs = new HangangScenario(sim);
    hs.start();
    hs.stop();
    assert.equal(sim.incheonAirport.terrain.mesh.visible, true);
    assert.equal(sim.incheonAirport.group.visible, true);
    assert.equal(sim.currentTerrain, sim.incheonAirport.terrain);
    assert.equal(sim.currentAirport, sim.incheonAirport);
  });
});

test('HangangScenario crashes only on contact and restarts collision on start', () => {
  withSim(sim => {
    const crashes = [];
    sim.isCrashed = false;
    sim.triggerCrash = result => { crashes.push(result); sim.isCrashed = true; };
    const hs = new HangangScenario(sim);
    hs.start();
    // Well above the former 28 m ceiling for longer than the former 8 s excursion.
    for (let i = 0; i < 400; i++) { sim.position.set(0, 60, 800 - i * 2.3); hs.update(0.05); }
    assert.equal(crashes.length, 0);
    sim.position.set(0, 0.5, -200); hs.update(0.05);
    assert.equal(crashes.length, 1); assert.equal(crashes[0].type, 'WATER');
    hs.update(0.05); assert.equal(crashes.length, 1);
    sim.isCrashed = false; hs.start(); assert.equal(hs.collision.contact, null);
  });
});
