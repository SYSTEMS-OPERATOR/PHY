import * as THREE from 'three';

// Shared by the UI and non-WebGL regressions. CAD models without role metadata
// retain the previous separate-model fallback; reference bones do not use it.
export function partVisible(part, state, layers) {
  const hardware=part.role?part.role==='hardware':part.name.toLowerCase().includes('coupling')||['brass','steel'].includes(part.material);
  const visible=state.boneOnly?part.role==='bone_proxy':part.region==='envelope'?layers.envelope:hardware?layers.joints:layers.frame;
  return visible&&(!state.isolated||!state.selected||part.id===state.selected);
}

export function partMatches(part, query, boneOnly) {
  const text=[part.name,part.id,part.region,part.bone_id??'',...(part.grouped_bone_ids??[])].join(' ').toLowerCase();
  return (!boneOnly||part.role==='bone_proxy')&&text.includes(query.toLowerCase());
}

export function exportGroup(root, meshes, boneOnly) {
  root.updateMatrixWorld(true);
  const group=new THREE.Group();
  for(const mesh of meshes){
    const p=mesh.userData.part;
    if(p.region==='envelope'||(boneOnly&&p.role!=='bone_proxy'))continue;
    const clone=new THREE.Mesh(mesh.geometry,mesh.material);clone.name=mesh.name;
    clone.applyMatrix4(mesh.matrixWorld);
    clone.userData={part_id:mesh.name,role:p.role??'separate_CAD',bone_id:p.bone_id??null,
      grouped_bone_ids:p.grouped_bone_ids??[],dimensional_fidelity:p.dimensional_fidelity??'unverified',status:'reference/unreleased'};
    group.add(clone);
  }
  return group;
}
