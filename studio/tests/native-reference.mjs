// Geometric fixture checks only: no anatomical feature is accepted here.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import * as THREE from 'three';
import { annotationFromHit, emptyDraft, previewBytes } from '../src/native_reference.js';

const geometry=new THREE.BufferGeometry();
geometry.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0, .1,0,0, 0,.1,0],3));
geometry.setIndex([0,1,2]);
const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial());
mesh.userData.part={id:'cortical_fixture',source_node:13,preview_node:7};
mesh.position.set(.02,.03,.04);
const root=new THREE.Group();root.scale.setScalar(1000);root.add(mesh);root.updateMatrixWorld(true);
const hit={object:mesh,faceIndex:0,point:mesh.localToWorld(new THREE.Vector3(.03,.05,0))};
const feature={id:'L_ASIS',preview_mesh_names:['cortical_fixture']};
const a=annotationFromHit(hit,feature);
assert.deepEqual(a.vertex_indices,[0,1,2]);
assert.ok(a.barycentric.every((v,i)=>Math.abs(v-[.2,.3,.5][i])<1e-7));
assert.ok(a.native_point_m.every((v,i)=>Math.abs(v-[.05,.08,.04][i])<1e-10));
assert.equal(a.source_node,13);assert.equal(a.preview_node,7);
assert.equal(a.original_face_index,null);assert.equal(a.anatomically_accepted,false);
assert.throws(()=>annotationFromHit(hit,{...feature,preview_mesh_names:['other_surface']}),/specified cortical/);
assert.throws(()=>annotationFromHit({...hit,faceIndex:null},feature),/triangle lineage/);

const html=await fs.readFile(new URL('../dist/PHY-Studio.html',import.meta.url),'utf8');
const data=JSON.parse(html.match(/window\.PHY_DATA=(.*?);<\/script>/s)[1]);
if(data.native_references){
  const draft=emptyDraft(data.native_references);
  assert.equal(draft.annotations.length,0);assert.equal(draft.canonical_transform,null);
  assert.equal(draft.accepted_landmarks,null);assert.equal(draft.fabrication_released,false);
  for(const model of data.models.filter(m=>m.kind==='nih_native_preview')){
    const raw=await previewBytes(model);
    assert.deepEqual(Buffer.from(raw),await fs.readFile(new URL('../dist/'+model.derivative.preview_filename,import.meta.url)));
    const changed=Buffer.from(raw);changed[changed.length-1]^=1;
    await assert.rejects(()=>previewBytes({...model,glb_base64:changed.toString('base64')}),/checksum/);
    await assert.rejects(()=>previewBytes({...model,glb_base64:Buffer.from(raw).subarray(0,-1).toString('base64')}),/length/);
  }
  console.log('Native reference logic passed: two exact pinned GLBs, checksum/length rejection, source-frame triangle reconstruction, wrong-surface rejection and unaccepted draft.');
}else console.log('Native geometric fixture passed; NIH custody checks omitted in explicit no-reference build.');
