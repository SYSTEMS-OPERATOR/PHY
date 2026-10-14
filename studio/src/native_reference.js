import * as THREE from 'three';
import { escape, htmlTable } from './evidence.js';

export const isNative = model => model?.kind === 'nih_native_preview';

export async function previewBytes(model) {
  const raw = Uint8Array.from(atob(model.glb_base64), c => c.charCodeAt(0));
  const view = new DataView(raw.buffer);
  if (raw.length !== model.derivative.preview_bytes || view.getUint32(0,true) !== 0x46546c67 ||
      view.getUint32(4,true) !== 2 || view.getUint32(8,true) !== raw.length) throw new Error('Pinned preview length/header mismatch');
  const hash = [...new Uint8Array(await crypto.subtle.digest('SHA-256',raw))].map(x=>x.toString(16).padStart(2,'0')).join('');
  if (hash !== model.derivative.preview_sha256) throw new Error('Pinned preview checksum mismatch');
  return raw;
}

export function emptyDraft(evidence) {
  const packet=evidence.landmark_packet;
  return {kind:'PHY_PELVIS_PREVIEW_DRAFT_V1',packet_id:packet.packet_id,
    packet_sha256:evidence.landmark_packet_sha256,reference_revision:evidence.revision,
    source_sha256:packet.source.sha256,preview_sha256:packet.preview.sha256,
    annotations:[],accepted_landmarks:null,canonical_transform:null,fabrication_released:false};
}

export function annotationFromHit(hit, landmark, model) {
  const mesh=hit.object,part=mesh.userData.part;
  if (!landmark.preview_mesh_names.includes(part.id)) throw new Error('Choose the specified cortical surface: '+landmark.preview_mesh_names.join(', '));
  const geometry=mesh.geometry,index=geometry.index,position=geometry.attributes.position,face=hit.faceIndex;
  if (!Number.isInteger(face)||!index) throw new Error('Preview triangle lineage unavailable');
  const vertices=[0,1,2].map(i=>index.getX(face*3+i));
  const triangle=vertices.map(i=>new THREE.Vector3().fromBufferAttribute(position,i));
  const local=mesh.worldToLocal(hit.point.clone()),weights=new THREE.Vector3();
  if (!THREE.Triangle.getBarycoord(local,...triangle,weights)) throw new Error('Degenerate preview triangle');
  const native=hit.point.clone().multiplyScalar(.001).toArray();
  return {landmark_id:landmark.id,mesh_name:part.id,preview_node:part.preview_node,
    source_node:part.source_node,face_index:face,vertex_indices:vertices,
    barycentric:weights.toArray(),native_point_m:native,original_face_index:null,anatomically_accepted:false};
}

export function nativeReferenceHtml(data) {
  const evidence=data.native_references;
  if (!evidence) return '<p>NIH previews were omitted from this build. Rebuild with the explicit pinned PHY-REF checkout using --reference-root.</p>';
  const packet=evidence.landmark_packet;
  const cards=data.models.filter(isNative).map(model=>{
    const s=model.source,d=model.derivative;
    return '<h3>'+escape(s.title)+'</h3><p>'+escape(s.attribution)+'</p><p><a href="'+escape(s.entry_url)+'" target="_blank" rel="noopener">NIH source entry</a> · <a href="'+escape(s.license_url)+'" target="_blank" rel="noopener">CC BY 4.0</a> · entry '+s.entry_version+' / HRA '+escape(s.reference_object_version)+'</p><p class="small-note">Original '+escape(s.original_filename)+' · '+s.bytes+' bytes · SHA-256 '+escape(s.sha256)+'<br>Preview '+d.preview_bytes+' bytes · '+d.preview_triangles+' triangles · SHA-256 '+escape(d.preview_sha256)+'</p><p>'+escape(d.recipe)+'. '+escape(d.qualification)+'. '+escape(d.textures_and_vertex_colors)+'.</p>';
  }).join('');
  return '<p><b>Separate native reference views.</b> Source world Y-up / meters. Display conversion is 1000 mm/m; no centering, mirroring, anatomical axis mapping, stature normalization or registration to PHY is applied. Switch between NIH and SOPHY to compare their separate frames. Camera changes do not move geometry.</p><p class="warning">The skin cannot supply bone dimensions. Pelvic tissue segments represent four adult bones. Bounds, draft points and decimation samples are not qualified osteometry, joint datums or fabrication evidence.</p><p class="small-note">PHY-REF revision '+escape(evidence.revision)+' · '+escape(evidence.review_status)+'</p>'+cards+'<h3>Sacrum–left hemipelvis review</h3><p>Status: awaiting independent review. Accepted points, original-face correspondence, metric calibration, tolerances and canonical transform remain unknown.</p>'+htmlTable(['CANDIDATE','FEATURE / POINT RULE'],packet.landmarks.map(p=>[p.id,p.definition+' '+p.review_rule]))+'<p>Choose a candidate in the pelvis inspector, arm point capture and click its specified cortical surface. Download the draft JSON, then run PHY-REF tools/validate_landmark_review.py --draft /path/to/draft.json. Geometric reconstruction checks the preview only. Two independent original-surface annotations and SYSOP-approved error budgets remain required.</p><p>NASA positioning/range references remain available in the NASA tab as comparison evidence; they do not set these bone points or mechanical limits.</p><p><a href="https://github.com/SYSTEMS-OPERATOR/PHY-REF/blob/'+escape(evidence.revision)+'/reviews/PELVIS_LEFT_REVIEW.md" target="_blank" rel="noopener">Original-surface review gates and stop criteria</a></p>';
}
