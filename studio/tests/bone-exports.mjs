// Actual three.js exports and the UI's shared selection rules; no WebGL/browser.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import * as THREE from 'three';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { STLExporter } from 'three/addons/exporters/STLExporter.js';
import { partVisible, partMatches, exportGroup } from '../src/part_roles.js';

globalThis.FileReader=class {
  readAsArrayBuffer(blob){blob.arrayBuffer().then(value=>{this.result=value;this.onloadend?.();});}
  readAsDataURL(blob){blob.arrayBuffer().then(value=>{this.result='data:'+blob.type+';base64,'+Buffer.from(value).toString('base64');this.onloadend?.();});}
};
const model=JSON.parse(await fs.readFile(new URL('../dist/f28_refined.json',import.meta.url),'utf8'));
const bones=model.parts.filter(p=>p.role==='bone_proxy');
assert.equal(bones.length,206);
assert.deepEqual(model.parts.filter(p=>partVisible(p,{boneOnly:true},{frame:false,joints:false,envelope:true})),bones);
assert.equal(partVisible(model.parts.find(p=>p.id==='head_arch_0'),{boneOnly:true,isolated:true,selected:'head_arch_0'},{frame:true,joints:true,envelope:true}),false);
assert.equal(partVisible(model.parts.find(p=>p.id==='clavicle_R'),{boneOnly:false},{frame:true,joints:false,envelope:false}),true,'a brass bone is not hardware');
assert.equal(partVisible(model.parts.find(p=>p.id==='knee_joint_R'),{boneOnly:false},{frame:true,joints:false,envelope:false}),false);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_RADIUS_R',true)).map(p=>p.id),['radius_R']);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_PHAL_1_2_R',true)).map(p=>p.id),['phal_1_2_R']);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_T_PHAL_1_2_R',true)).map(p=>p.id),['t_phal_1_2_R']);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_CALCANEUS_L',true)).map(p=>p.id),['calcaneus_L']);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_MAXILLA_R',false)).map(p=>p.id),['maxilla_R']);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_MAXILLA_R',true)).map(p=>p.id),['maxilla_R']);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_MANDIBLE',true)).map(p=>p.id),['mandible']);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_PAR_L',true)).map(p=>p.id),['par_L']);
for(const [boneId,partId] of [['BONE_MALLEUS_R','malleus_R'],['BONE_INCUS_L','incus_L'],
  ['BONE_STAPES_L','stapes_L'],['BONE_HYOID','hyoid']]){
  assert.deepEqual(model.parts.filter(p=>partMatches(p,boneId,true)).map(p=>p.id),[partId]);
}
assert.equal(model.bone_equivalence.bone_distribution_complete,true);
assert.equal(model.bone_equivalence.dimensional_fidelity_verified,false);
assert.equal(model.fabrication_released,false);

const root=new THREE.Group(),meshes=[];
for(const p of model.parts){
  const geometry=new THREE.BufferGeometry();
  geometry.setAttribute('position',new THREE.Float32BufferAttribute(p.vertices.flatMap(v=>[v[0],v[2],-v[1]]),3));
  geometry.setIndex(p.faces.flat());geometry.computeVertexNormals();
  const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial());
  mesh.name=p.id;mesh.userData.part=p;root.add(mesh);meshes.push(mesh);
}
root.scale.setScalar(1700/model.height_mm);
const all=exportGroup(root,meshes,false),core=exportGroup(root,meshes,true);
assert.equal(all.children.length,228);assert.equal(core.children.length,206);
assert.equal(new Set(core.children.map(p=>p.userData.bone_id)).size,206);
assert.ok(core.children.every(p=>p.userData.role==='bone_proxy'&&p.userData.dimensional_fidelity==='unverified'));
assert.ok(!core.children.some(p=>['knee_joint_R','head_arch_0','foot_R','shoulder_bridge'].includes(p.name)));
// Even hidden display objects belong to a complete-frame export unless bone-only.
meshes.forEach(mesh=>mesh.visible=false);
assert.equal(exportGroup(root,meshes,false).children.length,228);

all.rotateX(Math.PI/2);all.updateMatrixWorld(true);
const stl=new STLExporter().parse(all,{binary:true}),count=stl.getUint32(80,true);
assert.equal(stl.byteLength,84+count*50);
let maxZ=-Infinity;
for(let i=0;i<count;i++)for(let j=0;j<3;j++)maxZ=Math.max(maxZ,stl.getFloat32(84+i*50+12+j*12+8,true));
assert.ok(Math.abs(maxZ-1700)<.01,'full-frame STL must preserve explored height and canonical Z-up millimeters');
core.scale.setScalar(.001);core.updateMatrixWorld(true);
const glb=Buffer.from(await new GLTFExporter().parseAsync(core,{binary:true}));
assert.equal(glb.readUInt32LE(0),0x46546c67);assert.equal(glb.readUInt32LE(8),glb.length);
const contents=JSON.parse(glb.subarray(20,20+glb.readUInt32LE(12)).toString());
assert.equal(contents.meshes.length,206);
const extras=contents.nodes.filter(n=>n.mesh!==undefined).map(n=>n.extras);
assert.equal(new Set(extras.map(n=>n.bone_id)).size,206);
assert.ok(extras.every(n=>n.role==='bone_proxy'&&n.status==='reference/unreleased'));
const handIds=new Set(model.parts.filter(p=>p.region==='hands').map(p=>p.bone_id));
const hands=extras.filter(n=>handIds.has(n.bone_id));
assert.equal(handIds.size,54);assert.equal(hands.length,54);
assert.ok(hands.every(n=>n.hand_layout_sha256===model.hand_proxy_layout_sha256&&n.physical_evidence==='unmeasured'));
assert.ok(hands.every(n=>n.geometry_inputs.some(p=>p.includes('#/hand_proxy_layout/'))));
assert.equal(hands.find(n=>n.bone_id==='BONE_META1_L').topology_parent,'BONE_TRAPEZIUM_L');
const footIds=new Set(model.parts.filter(p=>p.region==='feet').map(p=>p.bone_id));
const feet=extras.filter(n=>footIds.has(n.bone_id));
assert.equal(footIds.size,52);assert.equal(feet.length,52);
assert.ok(feet.every(n=>n.foot_layout_sha256===model.foot_proxy_layout_sha256&&n.physical_evidence==='unmeasured'));
assert.ok(feet.every(n=>n.geometry_inputs.some(p=>p.includes('#/foot_proxy_layout/'))));
assert.equal(feet.find(n=>n.bone_id==='BONE_MT1_L').topology_parent,'BONE_MEDIAL_CUNEIFORM_L');
assert.equal(feet.find(n=>n.bone_id==='BONE_T_PHAL_1_2_L').topology_parent,'BONE_T_PHAL_1_1_L');
const skullIds=new Set(model.parts.filter(p=>p.skull_layout_sha256).map(p=>p.bone_id));
const skull=extras.filter(n=>skullIds.has(n.bone_id));
assert.equal(skullIds.size,22);assert.equal(skull.length,22);
assert.ok(skull.every(n=>n.skull_layout_sha256===model.skull_proxy_layout_sha256&&n.physical_evidence==='unmeasured'));
assert.ok(skull.every(n=>n.geometry_inputs.some(p=>p.includes('#/skull_proxy_layout/'))));
assert.equal(skull.filter(n=>n.source_record_status==='missing'&&n.source_record===null).length,13);
assert.equal(skull.filter(n=>n.source_record_status==='present'&&n.source_record).length,9);
assert.ok(skull.every(n=>n.topology_neighbors.length>0&&n.topology_neighbors.every(id=>skullIds.has(id))));
assert.deepEqual(skull.find(n=>n.bone_id==='BONE_MANDIBLE').articulates_with,['BONE_TEMP_R','BONE_TEMP_L']);
assert.equal(skull.find(n=>n.bone_id==='BONE_MANDIBLE').motion_implemented,false);
const headSeven=extras.filter(n=>n.ear_hyoid_layout_sha256);
assert.equal(headSeven.length,7);
assert.ok(headSeven.every(n=>n.ear_hyoid_layout_sha256===model.ear_hyoid_proxy_layout_sha256&&n.physical_evidence==='unmeasured'));
assert.ok(headSeven.every(n=>n.geometry_inputs.some(p=>p.includes('#/ear_hyoid_proxy_layout/'))));
assert.ok(headSeven.every(n=>n.source_record_status==='present'&&n.source_record&&n.motion_implemented===false));
for(const side of ['R','L']){
  const malleus=headSeven.find(n=>n.bone_id==='BONE_MALLEUS_'+side);
  const incus=headSeven.find(n=>n.bone_id==='BONE_INCUS_'+side);
  const stapes=headSeven.find(n=>n.bone_id==='BONE_STAPES_'+side);
  assert.deepEqual(malleus.articulates_with,['BONE_INCUS_'+side]);
  assert.deepEqual(incus.articulates_with,['BONE_MALLEUS_'+side,'BONE_STAPES_'+side]);
  assert.equal(stapes.topology_parent,'BONE_INCUS_'+side);
  assert.equal(stapes.housing_bone_id,'BONE_TEMP_'+side);
  assert.equal(stapes.proxy_shape,'stirrup_loop');
  assert.equal(malleus.non_bone_connection,'tympanic_membrane');
  assert.equal(stapes.non_bone_connection,'oval_window');
}
const hyoid=headSeven.find(n=>n.bone_id==='BONE_HYOID');
assert.deepEqual(hyoid.articulates_with,[]);
assert.equal(hyoid.topology_parent,null);assert.equal(hyoid.housing_bone_id,null);
assert.equal(hyoid.non_bone_support,'muscle_and_ligament_suspension_unmodeled');
console.log('Bone export logic passed: 206 IDs including six ear ossicles and one hyoid, role filtering, search, provenance/topology GLB extras, and 1700 mm STL. Visual browser interaction is not tested here.');
