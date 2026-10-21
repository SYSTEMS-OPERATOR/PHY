import * as THREE from 'three';

const fail=message=>{throw new Error(message);};
const clone=value=>structuredClone(value);
export const canonicalJson=value=>JSON.stringify(value,(_,v)=>v&&typeof v==='object'&&!Array.isArray(v)?Object.fromEntries(Object.keys(v).sort().map(k=>[k,v[k]])):v,2)+'\n';
export const sha256=async bytes=>[...new Uint8Array(await crypto.subtle.digest('SHA-256',bytes))].map(v=>v.toString(16).padStart(2,'0')).join('');
const bytes=value=>new TextEncoder().encode(canonicalJson(value)).length;
const keys=(v,names,label)=>{if(!v||Array.isArray(v)||typeof v!=='object'||Object.keys(v).sort().join(' ')!==names.split(' ').sort().join(' '))fail('Unsupported fields: '+label);};
const vector=(v,label)=>{if(!Array.isArray(v)||v.length!==3||v.some(x=>typeof x!=='number'||!Number.isFinite(x)))fail('Invalid finite vector: '+label);};
const text=(v,label,max=8192)=>{if(typeof v!=='string'||v.length>max||v.includes('\0'))fail('Invalid text: '+label);};
const index=(v,n,label)=>{if(!Number.isInteger(v)||v<0||v>=n)fail('Invalid index: '+label);return v;};

// JSON.parse alone silently discards duplicate keys. Scan object-key tokens first.
export function strictJson(raw,max=2097152){
  if(new TextEncoder().encode(raw).length>max)fail('JSON exceeds byte budget');
  const stack=[];
  for(let i=0;i<raw.length;i++){
    const ch=raw[i];
    if(ch==='"'){
      const start=i;let escaped=false;
      while(++i<raw.length){if(!escaped&&raw[i]==='"')break;if(!escaped&&raw[i]==='\\')escaped=true;else escaped=false;}
      const token=JSON.parse(raw.slice(start,i+1));let next=i+1;while(/\s/.test(raw[next]??'')&&next<raw.length)next++;
      if(raw[next]===':'&&stack.at(-1) instanceof Set){if(stack.at(-1).has(token))fail('Duplicate JSON field: '+token);stack.at(-1).add(token);}
    }else if(ch==='{')stack.push(new Set());else if(ch==='[')stack.push(null);else if(ch==='}'||ch===']')stack.pop();
    if(stack.length>64)fail('JSON nesting budget exceeded');
  }
  return JSON.parse(raw);
}

export class OriginalSurface{
  constructor(raw){
    this.raw=raw instanceof Uint8Array?raw:new Uint8Array(raw);const view=new DataView(this.raw.buffer,this.raw.byteOffset,this.raw.byteLength);
    if(this.raw.length<28||view.getUint32(0,true)!==0x46546c67||view.getUint32(4,true)!==2||view.getUint32(8,true)!==this.raw.length)fail('Invalid original GLB header/length');
    const size=view.getUint32(12,true),bin=20+size;
    if(view.getUint32(16,true)!==0x4e4f534a||bin+8>this.raw.length||view.getUint32(bin+4,true)!==0x004e4942||bin+8+view.getUint32(bin,true)!==this.raw.length)fail('Unsupported original GLB chunks');
    this.doc=strictJson(new TextDecoder().decode(this.raw.subarray(20,bin)),this.raw.length);this.binStart=bin+8;this.view=view;
    const d=this.doc;
    if(d.asset?.version!=='2.0'||d.extensionsRequired?.length||d.skins?.length||d.animations?.length||d.buffers?.length!==1||d.buffers[0].uri)fail('Only static embedded original geometry is supported');
    this.parents=new Map();this.worlds=new Map();this.parts=[];this.geometry=new Map();
    d.nodes.forEach((n,i)=>(n.children??[]).forEach(child=>{index(child,d.nodes.length,'child');if(this.parents.has(child))fail('Multiple-parent original node');this.parents.set(child,i);}));
    const world=(i,active=new Set())=>{
      if(this.worlds.has(i))return this.worlds.get(i);if(active.has(i))fail('Original node cycle');active=new Set([...active,i]);const n=d.nodes[index(i,d.nodes.length,'node')];let local=new THREE.Matrix4();
      if(n.matrix){if(n.translation||n.rotation||n.scale||n.matrix.length!==16||n.matrix.some(v=>!Number.isFinite(v)))fail('Unsupported node transform');local.fromArray(n.matrix);}
      else{const t=n.translation??[0,0,0],q=n.rotation??[0,0,0,1],s=n.scale??[1,1,1];vector(t,'translation');vector(s,'scale');if(q.length!==4||q.some(v=>!Number.isFinite(v))||Math.abs(q.reduce((a,b)=>a+b*b,0)-1)>1e-6||s.some(v=>v===0))fail('Invalid original TRS');local.compose(new THREE.Vector3(...t),new THREE.Quaternion(...q),new THREE.Vector3(...s));}
      const m=this.parents.has(i)?world(this.parents.get(i),active).clone().multiply(local):local;this.worlds.set(i,m);return m;
    };
    d.nodes.forEach((_,i)=>world(i));const selected=new Set();
    const select=i=>{index(i,d.nodes.length,'scene node');if(selected.has(i))fail('Repeated scene node');selected.add(i);(d.nodes[i].children??[]).forEach(select);};
    const scene=d.scenes[index(d.scene??0,d.scenes.length,'scene')];scene.nodes.forEach(select);
    for(const node of selected){const n=d.nodes[node];if(n.mesh===undefined)continue;const mesh=d.meshes[index(n.mesh,d.meshes.length,'mesh')];
      // The exact pinned original has one indexed primitive per named surface.
      if(mesh.primitives.length!==1||this.parts.some(p=>p.id===n.name))fail('Unsupported original surface grouping');
      mesh.primitives.forEach((p,primitive)=>{
        if((p.mode??4)!==4||p.extensions||p.targets||p.indices===undefined)fail('Indexed static triangles required');
        const positions=this.accessor(p.attributes.POSITION,3,5126),indices=this.accessor(p.indices,1);
        if(indices.length%3||indices.some(v=>v>=positions.length/3))fail('Invalid original index buffer');
        const box=new THREE.Box3();for(let i=0;i<positions.length;i+=3)box.expandByPoint(new THREE.Vector3(positions[i],positions[i+1],positions[i+2]).applyMatrix4(world(node)));
        this.geometry.set(node+':'+primitive,{positions,indices,normal:p.attributes.NORMAL===undefined?null:this.accessor(p.attributes.NORMAL,3,5126),material:p.material});
        this.parts.push({id:n.name,name:n.name,source_node:node,primitive_index:primitive,region:'native_reference',triangle_count:indices.length/3,bounds_m:{min:box.min.toArray(),max:box.max.toArray()},bounds_mm:{min:box.min.toArray().map(v=>v*1000),max:box.max.toArray().map(v=>v*1000)},authority:'Pinned original surface; anatomical and metric qualification unresolved',source:'NIH/HRA original pelvis'});
      });
    }
  }
  accessor(id,width,expectedType){
    const d=this.doc,a=d.accessors[index(id,d.accessors.length,'accessor')],v=d.bufferViews[index(a.bufferView,d.bufferViews.length,'buffer view')];
    const specs={5121:[1,'getUint8'],5123:[2,'getUint16'],5125:[4,'getUint32'],5126:[4,'getFloat32']},spec=specs[a.componentType];
    if(!spec||a.sparse||a.normalized||a.extensions||v.extensions||v.buffer!==0||a.type!==(width===3?'VEC3':'SCALAR')||(expectedType&&a.componentType!==expectedType)||(!expectedType&&a.componentType===5126))fail('Unsupported original accessor');
    index(a.count-1,1000000,'accessor count');const [size,get]=spec,stride=v.byteStride??size*width,offset=(v.byteOffset??0)+(a.byteOffset??0),end=(a.byteOffset??0)+(a.count-1)*stride+size*width;
    if(!Number.isInteger(stride)||stride<size*width||stride%size||end>v.byteLength||offset<0||offset%size||(v.byteOffset??0)+v.byteLength>d.buffers[0].byteLength||this.binStart+offset+(a.count-1)*stride+size*width>this.raw.length)fail('Original accessor outside embedded buffer');
    const values=[];for(let i=0;i<a.count;i++)for(let j=0;j<width;j++){const value=this.view[get](this.binStart+offset+i*stride+j*size,true);if(!Number.isFinite(value))fail('Nonfinite original position');values.push(value);}return values;
  }
  triangle(node,primitive,face,vertices){
    index(node,this.doc.nodes.length,'source node');index(primitive,this.doc.meshes[this.doc.nodes[node].mesh]?.primitives.length??0,'primitive');const g=this.geometry.get(node+':'+primitive);if(!g)fail('Original node outside selected scene');index(face,g.indices.length/3,'face');
    const ids=g.indices.slice(face*3,face*3+3);if(vertices&&(!Array.isArray(vertices)||vertices.length!==3||vertices.some((v,i)=>!Number.isInteger(v)||v!==ids[i])))fail('Original triangle vertex lineage mismatch');
    const points=ids.map(i=>new THREE.Vector3(...g.positions.slice(i*3,i*3+3)).applyMatrix4(this.worlds.get(node)));
    if(new THREE.Vector3().subVectors(points[1],points[0]).cross(new THREE.Vector3().subVectors(points[2],points[0])).lengthSq()===0)fail('Degenerate original triangle');return {vertices:ids,points};
  }
  scene(){
    const group=new THREE.Group();
    for(const part of this.parts){const g=this.geometry.get(part.source_node+':'+part.primitive_index),geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(g.positions,3));geometry.setIndex(g.indices);if(g.normal)geometry.setAttribute('normal',new THREE.Float32BufferAttribute(g.normal,3));else geometry.computeVertexNormals();
      const factor=this.doc.materials?.[g.material]?.pbrMetallicRoughness?.baseColorFactor??[.8,.8,.8,1];const material=new THREE.MeshStandardMaterial({color:new THREE.Color().setRGB(...factor.slice(0,3)),roughness:.8,metalness:0});const mesh=new THREE.Mesh(geometry,material);mesh.name=part.id;mesh.matrixAutoUpdate=false;mesh.matrix.copy(this.worlds.get(part.source_node));mesh.userData.part=part;group.add(mesh);
    }return group;
  }
}

export async function originalContext(raw,evidence){
  const c=evidence.original_review_contract;if(!c)fail('Original review omitted from this build');raw=new Uint8Array(raw);
  if(raw.length!==c.source.bytes)fail('Original missing, LFS pointer or size mismatch');if(await sha256(raw)!==c.source.sha256)fail('Original checksum mismatch');
  const surface=new OriginalSurface(raw);
  for(const f of Object.values(c.features))if(!surface.parts.some(p=>p.source_node===f.source_node&&p.id===f.mesh_name))fail('Original source node/label mismatch');
  return {contract:c,evidence,surface};
}
export function originalHeader(ctx,kind){const c=ctx.contract,e=ctx.evidence;return {kind,contract_sha256:e.original_review_contract_sha256,packet_id:e.landmark_packet.packet_id,packet_sha256:e.landmark_packet_sha256,reference_revision:e.revision,source_sha256:c.source.sha256,units:'m',coordinate_frame:c.coordinate_frame,accepted_landmarks:null,canonical_transform:null,fabrication_released:false};}
export function emptyOriginal(ctx){return {...originalHeader(ctx,ctx.contract.draft_kind),reviewer:{id:'unassigned',session_id:'session-1',evidence_kind:'review_submission'},annotations:[],views:[],review_notes:''};}
export function validateOriginal(ctx,d){
  const c=ctx.contract,base=originalHeader(ctx,c.draft_kind);keys(d,Object.keys(base).join(' ')+' reviewer annotations views review_notes','original draft');if(Object.keys(base).some(k=>d[k]!==base[k]))fail('Original draft custody, frame or qualification mismatch');if(bytes(d)>c.max_draft_bytes)fail('Original draft exceeds byte budget');
  keys(d.reviewer,'id session_id evidence_kind','reviewer');for(const k of ['id','session_id'])if(typeof d.reviewer[k]!=='string'||!/^[A-Za-z0-9_.:-]{1,80}$/.test(d.reviewer[k]))fail('Invalid reviewer/session identifier');if(!['review_submission','software_fixture'].includes(d.reviewer.evidence_kind))fail('Invalid declared evidence kind');text(d.review_notes,'review notes');
  if(!Array.isArray(d.views)||d.views.length>c.max_views)fail('Invalid view list');for(const v of d.views){keys(v,'label camera_position_mm target_mm','view');text(v.label,'view label',80);vector(v.camera_position_mm,'camera');vector(v.target_mm,'target');}
  if(!Array.isArray(d.annotations)||d.annotations.length>Object.keys(c.features).length)fail('Invalid original annotation list');const seen=new Set(),pending=[];
  for(const a of d.annotations){keys(a,'landmark_id mesh_name source_node primitive_index face_index vertex_indices barycentric native_point_m selection_rule patch_faces patch_rule construction_note anatomically_accepted','annotation');const f=c.features[a.landmark_id];if(!f||seen.has(a.landmark_id))fail('Unknown or duplicate original landmark');seen.add(a.landmark_id);if(a.mesh_name!==f.mesh_name||a.source_node!==f.source_node||a.anatomically_accepted!==false)fail('Wrong original surface or anatomical promotion');
    const t=ctx.surface.triangle(a.source_node,a.primitive_index,a.face_index,a.vertex_indices);vector(a.barycentric,'barycentric');vector(a.native_point_m,'native point');const w=a.barycentric;if(Math.min(...w)<-1e-7||Math.max(...w)>1+1e-7||Math.abs(w.reduce((x,y)=>x+y,0)-1)>1e-7)fail('Barycentric point outside original triangle');const p=new THREE.Vector3();t.points.forEach((v,i)=>p.addScaledVector(v,w[i]));if(p.distanceTo(new THREE.Vector3(...a.native_point_m))>c.reconstruction_epsilon_m)fail('Original point does not reconstruct');
    for(const k of ['selection_rule','patch_rule','construction_note'])text(a[k],k,2000);if(!Array.isArray(a.patch_faces)||a.patch_faces.length>c.max_patch_faces)fail('Invalid patch face budget');const faces=new Set();for(const face of a.patch_faces){keys(face,'primitive_index face_index vertex_indices','patch face');ctx.surface.triangle(a.source_node,face.primitive_index,face.face_index,face.vertex_indices);const id=face.primitive_index+':'+face.face_index;if(faces.has(id))fail('Duplicate patch face');faces.add(id);}
    if(!a.selection_rule.trim())pending.push(a.landmark_id+': point-selection rule');if(f.patch_required&&(!a.patch_faces.length||!a.patch_rule.trim()))pending.push(a.landmark_id+': patch and representative-point rule');if(f.construction_required&&!a.construction_note.trim())pending.push(a.landmark_id+': median-plane construction');
  }return {status:'original_geometry_verified_only',annotations:d.annotations.length,missing_features:Object.keys(c.features).filter(k=>!seen.has(k)).sort(),unresolved_requirements:pending.sort(),anatomically_accepted:false,fabrication_released:false};
}
export function compareOriginal(ctx,a,b){
  validateOriginal(ctx,a);validateOriginal(ctx,b);const aa=Object.fromEntries(a.annotations.map(p=>[p.landmark_id,p])),bb=Object.fromEntries(b.annotations.map(p=>[p.landmark_id,p]));const patch=p=>p.patch_faces.map(f=>f.primitive_index+':'+f.face_index).sort().join(',');
  return {kind:ctx.contract.comparison_kind,status:'unaccepted_coordinate_comparison',reviewer_relation:a.reviewer.id===b.reviewer.id?'same_declared_reviewer':'distinct_declared_ids_independence_unverified',contains_software_fixtures:[a,b].some(d=>d.reviewer.evidence_kind==='software_fixture'),distance_meaning:'Source-coordinate distances converted to mm; metric calibration unresolved; no tolerance verdict.',rows:Object.keys(ctx.contract.features).sort().map(id=>{const x=aa[id],y=bb[id];return {landmark_id:id,status:x&&y?'paired':!x&&!y?'missing_both':!x?'missing_a':'missing_b',distance_source_mm:x&&y?Math.hypot(...x.native_point_m.map((v,i)=>v-y.native_point_m[i]))*1000:null,selection_rules_differ:x&&y?['selection_rule','patch_rule','construction_note'].some(k=>x[k]!==y[k]):null,patch_faces_differ:x&&y?patch(x)!==patch(y):null};}),accepted_landmarks:null,canonical_transform:null,fabrication_released:false};
}
export function originalPacket(ctx,reviews,build=null){if(reviews.length<1||reviews.length>2)fail('One or two reviews required');if(build!==null&&!/^[0-9a-f]{64}$/.test(build))fail('Invalid viewer build fingerprint');return {...originalHeader(ctx,ctx.contract.packet_kind),reviews:clone(reviews),validation:reviews.map(d=>validateOriginal(ctx,d)),comparison:reviews.length===2?compareOriginal(ctx,...reviews):null,viewer_build_sha256:build,unresolved_gates:clone(ctx.contract.unresolved_gates),attribution:ctx.evidence.landmark_packet.source.attribution,license:'CC-BY-4.0'};}
export function validateOriginalPacket(ctx,packet){
  if(!packet||!Array.isArray(packet.reviews)||bytes(packet)>ctx.contract.max_packet_bytes)fail('Invalid review packet or byte budget');
  const expected=originalPacket(ctx,packet.reviews,packet.viewer_build_sha256);
  const equal=(a,b)=>{
    if(typeof a==='number'&&typeof b==='number')return Number.isFinite(a)&&Math.abs(a-b)<=Math.max(1e-9,Math.abs(b)*1e-12);
    if(Array.isArray(a)&&Array.isArray(b))return a.length===b.length&&a.every((v,i)=>equal(v,b[i]));
    if(a&&b&&typeof a==='object'&&typeof b==='object')return Object.keys(a).sort().join(' ')===Object.keys(b).sort().join(' ')&&Object.keys(a).every(k=>equal(a[k],b[k]));
    return a===b;
  };
  if(!equal(packet,expected))fail('Packet reports, custody or qualification differ from reconstruction');
  return packet.reviews;
}
export function originalAnnotation(ctx,hit,ident,old){
  const part=hit.object.userData.part,f=ctx.contract.features[ident];if(!f||part.source_node!==f.source_node||part.id!==f.mesh_name)fail('Choose the specified original cortical surface');
  const g=hit.object.geometry,face=hit.faceIndex;const ids=[0,1,2].map(i=>g.index.getX(face*3+i));ctx.surface.triangle(part.source_node,part.primitive_index,face,ids);
  const triangle=ids.map(i=>new THREE.Vector3().fromBufferAttribute(g.attributes.position,i)),local=hit.object.worldToLocal(hit.point.clone()),w=new THREE.Vector3();if(!THREE.Triangle.getBarycoord(local,...triangle,w))fail('Degenerate original triangle');
  return {landmark_id:ident,mesh_name:part.id,source_node:part.source_node,primitive_index:part.primitive_index,face_index:face,vertex_indices:ids,barycentric:w.toArray(),native_point_m:hit.point.clone().multiplyScalar(.001).toArray(),selection_rule:old?.selection_rule??'',patch_faces:clone(old?.patch_faces??[]),patch_rule:old?.patch_rule??'',construction_note:old?.construction_note??'',anatomically_accepted:false};
}
