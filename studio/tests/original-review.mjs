// Software fixtures only. The chosen faces do not identify anatomical landmarks.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import * as THREE from 'three';
import { OriginalSurface, strictJson, canonicalJson, originalContext, emptyOriginal,
  originalAnnotation, validateOriginal, compareOriginal, originalPacket, validateOriginalPacket } from '../src/original_review.js';

const html=await fs.readFile(new URL('../dist/PHY-Studio.html',import.meta.url),'utf8');
const data=JSON.parse(html.match(/window\.PHY_DATA=(.*?);<\/script>/s)[1]);
assert.throws(()=>strictJson('{"x":1,"\\u0078":2}'),/Duplicate/);
assert.throws(()=>strictJson('['.repeat(65)+'0'+']'.repeat(65)),/nesting/);
assert.throws(()=>strictJson('"12345"',3),/byte budget/);
assert.deepEqual(strictJson('{"x":[{"x":1}]}'),{x:[{x:1}]});
assert.equal(canonicalJson({z:1,a:{z:2,a:3}}),canonicalJson({a:{a:3,z:2},z:1}));

function transformedFixture(){
  const positions=new Float32Array([0,0,0,.1,0,0,0,.1,0]),indices=new Uint32Array([0,1,2]);
  const doc={asset:{version:'2.0'},scene:0,scenes:[{nodes:[0]}],nodes:[{translation:[.02,.03,.04],rotation:[0,0,Math.SQRT1_2,Math.SQRT1_2],children:[1]},{name:'fixture',mesh:0,scale:[2,1,1]}],meshes:[{primitives:[{attributes:{POSITION:0},indices:1}]}],buffers:[{byteLength:48}],bufferViews:[{buffer:0,byteLength:36},{buffer:0,byteOffset:36,byteLength:12}],accessors:[{bufferView:0,componentType:5126,count:3,type:'VEC3'},{bufferView:1,componentType:5125,count:3,type:'SCALAR'}]};
  let json=JSON.stringify(doc);json+=' '.repeat((4-json.length%4)%4);const bytes=Buffer.alloc(28+json.length+48);bytes.writeUInt32LE(0x46546c67);bytes.writeUInt32LE(2,4);bytes.writeUInt32LE(bytes.length,8);bytes.writeUInt32LE(json.length,12);bytes.writeUInt32LE(0x4e4f534a,16);bytes.write(json,20);bytes.writeUInt32LE(48,20+json.length);bytes.writeUInt32LE(0x004e4942,24+json.length);Buffer.from(positions.buffer).copy(bytes,28+json.length);Buffer.from(indices.buffer).copy(bytes,64+json.length);return bytes;
}
const surface=new OriginalSurface(transformedFixture()),points=surface.triangle(1,0,0).points;
for(const [i,expected] of [[0,[.02,.03,.04]],[1,[.02,.23,.04]],[2,[-.08,.03,.04]]])assert.ok(points[i].distanceTo(new THREE.Vector3(...expected))<1e-8);
assert.throws(()=>surface.triangle(1,0,true),/Invalid index/);
assert.throws(()=>surface.triangle(1,0,0,[1,0,2]),/lineage/);

if(data.native_references?.original_review_contract){
  const evidence=structuredClone(data.native_references),contract=structuredClone(evidence.original_review_contract);contract.features.L_ASIS={...contract.features.L_ASIS,source_node:1,mesh_name:'fixture'};
  const ctx={surface,contract,evidence},scene=surface.scene(),root=new THREE.Group();root.scale.setScalar(1000);root.add(scene);root.updateMatrixWorld(true);
  const mesh=scene.children[0],point=mesh.localToWorld(new THREE.Vector3(.03,.05,0));
  const annotation=originalAnnotation(ctx,{object:mesh,faceIndex:0,point},'L_ASIS');
  assert.ok(annotation.native_point_m.every((v,i)=>Math.abs(v-[-.03,.09,.04][i])<1e-8));
  const a=emptyOriginal(ctx);a.reviewer.evidence_kind='software_fixture';a.annotations.push(annotation);
  assert.equal(validateOriginal(ctx,a).annotations,1);
  for(const change of [d=>d.accepted_landmarks={},d=>d.canonical_transform=[],d=>d.fabrication_released=true,d=>d.annotations[0].anatomically_accepted=true,d=>d.annotations[0].native_point_m[0]+=.01,d=>d.annotations[0].barycentric=[-1,1,1],d=>d.annotations[0].face_index=true,d=>d.annotations[0].vertex_indices=[2,1,0],d=>d.annotations[0].patch_faces=[{primitive_index:0,face_index:1,vertex_indices:[0,1,2]}],d=>d.annotations.push(d.annotations[0]),d=>d.reference_revision='0'.repeat(40),d=>d.source_sha256='0'.repeat(64),d=>d.kind='PHY_PELVIS_PREVIEW_DRAFT_V1',d=>d.views=[{label:'bad',camera_position_mm:[0,NaN,1],target_mm:[0,0,0]}]]){
    const bad=structuredClone(a);change(bad);assert.throws(()=>validateOriginal(ctx,bad));
  }
  const before=canonicalJson(a),same=compareOriginal(ctx,a,a);assert.equal(same.rows.find(r=>r.landmark_id==='L_ASIS').distance_source_mm,0);assert.equal(same.reviewer_relation,'same_declared_reviewer');
  const b=structuredClone(a);b.reviewer.id='fixture-B';b.annotations[0].barycentric=[.3,.2,.5];b.annotations[0].native_point_m=points.reduce((p,v,i)=>p.addScaledVector(v,b.annotations[0].barycentric[i]),new THREE.Vector3()).toArray();
  assert.ok(compareOriginal(ctx,a,b).rows.find(r=>r.landmark_id==='L_ASIS').distance_source_mm>0);
  const incomplete=structuredClone(a);incomplete.annotations=[];assert.equal(compareOriginal(ctx,a,incomplete).rows.find(r=>r.landmark_id==='L_ASIS').status,'missing_b');assert.equal(canonicalJson(a),before);
  const packet=originalPacket(ctx,[a,b],data.build.package_input_sha256);validateOriginalPacket(ctx,packet);assert.deepEqual(validateOriginalPacket(ctx,strictJson(canonicalJson(packet))),[a,b]);
  for(const change of [p=>p.comparison.rows[0].distance_source_mm=10,p=>p.unresolved_gates=[],p=>p.fabrication_released=true,p=>p.extra='accepted']){const bad=structuredClone(packet);change(bad);assert.throws(()=>validateOriginalPacket(ctx,bad));}
  assert.equal(packet.accepted_landmarks,null);assert.equal(packet.canonical_transform,null);
  if(process.env.PHY_PELVIS_ORIGINAL){
    const raw=await fs.readFile(path.resolve(process.env.PHY_PELVIS_ORIGINAL)),real=await originalContext(raw,data.native_references);
    assert.equal(real.surface.parts.length,14);assert.equal(real.surface.parts.reduce((n,p)=>n+p.triangle_count,0),87448);
    const draft=emptyOriginal(real);draft.reviewer={id:'geometry-fixture',session_id:'logic-fixture',evidence_kind:'software_fixture'};
    for(const [id,f] of Object.entries(real.contract.features)){
      const scene=real.surface.scene(),root=new THREE.Group();root.scale.setScalar(1000);root.add(scene);root.updateMatrixWorld(true);const mesh=scene.children.find(m=>m.name===f.mesh_name),t=real.surface.triangle(f.source_node,0,0);
      const point=t.points.reduce((v,p)=>v.addScaledVector(p,1/3),new THREE.Vector3()).multiplyScalar(1000),a=originalAnnotation(real,{object:mesh,faceIndex:0,point},id);a.selection_rule='Software fixture: triangle centroid, not an anatomical observation.';a.patch_faces=[{primitive_index:0,face_index:0,vertex_indices:t.vertices}];draft.annotations.push(a);
    }
    validateOriginal(real,draft);assert.deepEqual(strictJson(canonicalJson(draft)),draft);
    const corrupted=Buffer.from(raw);corrupted[corrupted.length-1]^=1;await assert.rejects(()=>originalContext(corrupted,data.native_references),/checksum/);await assert.rejects(()=>originalContext(raw.subarray(0,128),data.native_references),/size mismatch/);
    console.log('Original logic: exact 14-mesh / 87,448-triangle source, all five surfaces reconstructed; tampered and wrong-size originals rejected.');
  }else console.log('SKIP exact original custody fixture: PHY_PELVIS_ORIGINAL not supplied; no original is fetched.');
  console.log('Original logic passed: transformed parent reconstruction, format separation, patch lineage, qualification rejection, round trip and non-destructive comparisons.');
}else console.log('SKIP original contracts in explicit no-reference build.');
