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
assert.equal(bones.length,125);
assert.deepEqual(model.parts.filter(p=>partVisible(p,{boneOnly:true},{frame:false,joints:false,envelope:true})),bones);
assert.equal(partVisible(model.parts.find(p=>p.id==='foot_R'),{boneOnly:true,isolated:true,selected:'foot_R'},{frame:true,joints:true,envelope:true}),false);
assert.equal(partVisible(model.parts.find(p=>p.id==='clavicle_R'),{boneOnly:false},{frame:true,joints:false,envelope:false}),true,'a brass bone is not hardware');
assert.equal(partVisible(model.parts.find(p=>p.id==='knee_joint_R'),{boneOnly:false},{frame:true,joints:false,envelope:false}),false);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_RADIUS_R',true)).map(p=>p.id),['radius_R']);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_PHAL_1_2_R',true)).map(p=>p.id),['phal_1_2_R']);
assert.deepEqual(model.parts.filter(p=>partMatches(p,'BONE_MAXILLA_R',false)).map(p=>p.id),['head_arch_0']);
assert.equal(model.parts.filter(p=>partMatches(p,'BONE_MAXILLA_R',true)).length,0);

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
assert.equal(all.children.length,149);assert.equal(core.children.length,125);
assert.equal(new Set(core.children.map(p=>p.userData.bone_id)).size,125);
assert.ok(core.children.every(p=>p.userData.role==='bone_proxy'&&p.userData.dimensional_fidelity==='unverified'));
assert.ok(!core.children.some(p=>['knee_joint_R','head_arch_0','foot_R','shoulder_bridge'].includes(p.name)));
// Even hidden display objects belong to a complete-frame export unless bone-only.
meshes.forEach(mesh=>mesh.visible=false);
assert.equal(exportGroup(root,meshes,false).children.length,149);

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
assert.equal(contents.meshes.length,125);
const extras=contents.nodes.filter(n=>n.mesh!==undefined).map(n=>n.extras);
assert.equal(new Set(extras.map(n=>n.bone_id)).size,125);
assert.ok(extras.every(n=>n.role==='bone_proxy'&&n.status==='reference/unreleased'));
const handIds=new Set(model.parts.filter(p=>p.region==='hands').map(p=>p.bone_id));
const hands=extras.filter(n=>handIds.has(n.bone_id));
assert.equal(handIds.size,54);assert.equal(hands.length,54);
assert.ok(hands.every(n=>n.hand_layout_sha256===model.hand_proxy_layout_sha256&&n.physical_evidence==='unmeasured'));
assert.ok(hands.every(n=>n.geometry_inputs.some(p=>p.includes('#/hand_proxy_layout/'))));
assert.equal(hands.find(n=>n.bone_id==='BONE_META1_L').topology_parent,'BONE_TRAPEZIUM_L');
console.log('Bone export logic passed: 125 IDs, role filtering, search, GLB extras, and 1700 mm STL. Visual browser interaction is not tested here.');
