import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

// Pilot-local metres. Each section runs across the sleeve or glove, with its
// plane perpendicular to the local centreline. Shared vertices smooth the bends.
function loft(sections, sides) {
  const centers = sections.map(([x,y,z]) => new THREE.Vector3(x,y,z));
  const vertices = [], faces = [];
  for (let i = 0; i < sections.length; i++) {
    const along = centers[Math.min(i+1, centers.length-1)].clone()
      .sub(centers[Math.max(i-1, 0)]).normalize();
    const axis = Math.abs(along.x) > 0.85 ? new THREE.Vector3(0,1,0) : new THREE.Vector3(1,0,0);
    const up = along.clone().cross(axis).normalize();
    const across = up.clone().cross(along).normalize();
    const [, , , width, depth, tilt = 0] = sections[i];
    for (let j = 0; j < sides; j++) {
      const a = 2 * Math.PI * j / sides;
      const pt = centers[i].clone().addScaledVector(across, width * Math.cos(a))
        .addScaledVector(up, depth * Math.sin(a)).addScaledVector(along, tilt * Math.cos(a));
      vertices.push(...pt.toArray());
      if (i === sections.length - 1) continue;
      const p = i * sides + j, q = i * sides + (j+1)%sides;
      faces.push(p,q,p+sides, q,q+sides,p+sides);
    }
  }
  const end = (sections.length-1)*sides;
  for (let j=1; j<sides-1; j++) faces.push(0,j+1,j, end,end+j,end+j+1);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices,3));
  geometry.setIndex(faces);
  geometry.computeVertexNormals();
  return geometry;
}

function ellipsoid(center, radii, tilt=0) {
  const geometry = new THREE.SphereGeometry(1,12,8);
  geometry.scale(...radii);
  geometry.rotateZ(tilt);
  geometry.translate(...center);
  return geometry;
}

export function fitRightArm(root) {
  const body = root.getObjectByName('Pilot_Body');
  const part = tag => root.getObjectByName('Pilot_Body_' + tag);
  const cloth = part('ArmUpper_R').material;
  const leather = part('Glove_Palm_R').material;
  const gloveBack = part('Glove_R').material;
  for (const tag of ['ShoulderSocket_R','Deltoid_R']) part(tag).removeFromParent();
  function add(name, geometry, material) {
    const mesh = new THREE.Mesh(geometry, material);
    mesh.name = name;
    body.add(mesh);
  }
  // Shoulder crown, biceps, compressed elbow, tapered forearm and gathered cuff.
  add('Right sleeve', loft([
    [.202,.476,.220,.033,.038], [.207,.461,.214,.065,.069],
    [.214,.428,.200,.081,.083], [.219,.393,.170,.084,.078],
    [.224,.354,.127,.078,.068], [.231,.316,.080,.068,.061],
    [.236,.283,.040,.061,.061,.006], [.235,.274,.023,.066,.065,.009],
    [.235,.268,.011,.058,.053,.010], [.234,.259,-.003,.064,.059,.010],
    [.230,.255,-.019,.061,.053,.010], [.228,.254,-.039,.062,.052,.009],
    [.224,.253,-.071,.057,.052], [.218,.247,-.123,.050,.045],
    [.215,.243,-.156,.046,.041], [.214,.242,-.169,.047,.042,.006],
    [.214,.241,-.177,.041,.036,.005], [.214,.240,-.185,.043,.039,.003],
    [.214,.240,-.195,.040,.036], [.214,.240,-.198,.035,.032]
  ],16), cloth);
  // Knuckles sit outside the grip. The thumb crosses its inboard side.
  const palm = loft([
    [.215,.240,-.187,.031,.028], [.218,.240,-.202,.032,.030],
    [.224,.239,-.224,.035,.033], [.230,.236,-.248,.035,.032],
    [.230,.233,-.265,.031,.027], [.226,.231,-.277,.020,.018],
    [.222,.231,-.282,.009,.010]
  ],14);
  const back = ellipsoid([.240,.247,-.233],[.021,.027,.043],-.33);
  add('Right glove back', back, gloveBack);
  const fingers = [palm,loft([
    [.217,.240,-.202,.032,.029], [.217,.240,-.203,.034,.031],
    [.219,.240,-.211,.035,.032], [.219,.240,-.213,.033,.030]
  ],14)];
  for (let i=0; i<4; i++) {
    const y=.260-i*.013, z=-.272+i*.003;
    fingers.push(loft([[.246,y,z+.016,.008,.008],[.245,y,z,.009,.009],
      [.237,y,z-.011,.009,.009],[.223,y,z-.014,.008,.008],
      [.209,y-.002,z-.009,.007,.007],[.202,y-.003,z-.003,.004,.005]],8));
  }
  fingers.push(ellipsoid([.202,.259,-.246],[.012,.014,.029],-.34));
  add('Right glove leather', mergeGeometries(fingers.map(g=>{
    if(g.hasAttribute('uv')) g.deleteAttribute('uv');
    return g;
  })),leather);
}
