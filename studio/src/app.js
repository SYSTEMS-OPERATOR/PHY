import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFExporter } from 'three/addons/exporters/GLTFExporter.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { STLExporter } from 'three/addons/exporters/STLExporter.js';
import { partVisible, partMatches, exportGroup as makeExportGroup } from './part_roles.js';
import { escape, htmlTable, boneEvidenceHtml, engineeringHtml, nasaHtml, nasaMotionHtml, conditionLabels } from './evidence.js';
import { relationshipsHtml } from './relationships.js';
import { isNative, previewBytes, emptyDraft, annotationFromHit, nativeReferenceHtml } from './native_reference.js';
import { createOriginalReview } from './original_review_ui.js';
import { canonicalJson } from './original_review.js';

const data = window.PHY_DATA;
const $ = id => document.getElementById(id);
const canvas = $('viewport'), stage = $('stage');
const round = (v,n=1) => Number(v).toFixed(n);
const coord = v => new THREE.Vector3(v[0],v[2],-v[1]);
const sourceMean = data.reference.statistics.stature.mean_mm;
const state = { model:'F28_REFINED', height:sourceMean, pose:75, palette:'wood', exploded:0,
  selected:null, isolated:false, boneOnly:false, ruler:false, points:[], view:'iso', loading:false, referenceCapture:false };
let model, renderer, scene, camera, controls, root, meshes=[], guides, ground, rulerGroup, annotationGroup, environment;
let selectionSequence=0, comparisonModel='NIH_PELVIS';
let referenceDraft=data.native_references?emptyDraft(data.native_references):null;
let originalReview;
let toastTimer, lastFocus, focusTarget, redraw=true;
const palette = {
  wood: {redwood:0xb27850, brass:0xb99b5e, copper:0xb57055, ivory:0xaabbab, steel:0x6b767b, aluminum:0xb2b9b5, acetal:0x383f3d},
  carbon: {redwood:0x424d50, brass:0xc6b58c, copper:0xa1aeb0, ivory:0xd5dacf, steel:0x687277, aluminum:0xc4cac6, acetal:0x363f3d}
};

function toast(message) { $('toast').textContent=message; $('toast').classList.add('show'); clearTimeout(toastTimer); toastTimer=setTimeout(()=>$('toast').classList.remove('show'),3200); }
function download(content,name,type='application/octet-stream') {
  const blob=content instanceof Blob ? content : new Blob([content],{type});
  const url=URL.createObjectURL(blob), a=document.createElement('a'); a.href=url;a.download=name;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),5000);
}
function init() {
  try {
    renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true,preserveDrawingBuffer:true});
  } catch(e) {
    $('loader').hidden=true; $('fallback').style.display='block';
    $('fallback').innerHTML='<h2>3D rendering is unavailable here.</h2><p>Open PHY Studio in a browser with WebGL enabled. Proportions, readiness, and the maquette download still work.</p>';
    $('render-status').textContent='WebGL unavailable';
    populateModels(); wireResources(); selectModel('F28_REFINED',false); $('export').disabled=true; return;
  }
  renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));
  renderer.setClearColor(0xf2f1eb,0); renderer.outputColorSpace=THREE.SRGBColorSpace;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.5;
  renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
  scene=new THREE.Scene();
  camera=new THREE.PerspectiveCamera(31,1,1,25000);camera.up.set(0,1,0);
  controls=new OrbitControls(camera,canvas);controls.enableDamping=true;controls.dampingFactor=.09;controls.maxPolarAngle=Math.PI*.94;controls.minDistance=150;controls.maxDistance=18000;
  controls.addEventListener('change',()=>redraw=true);
  scene.add(new THREE.HemisphereLight(0xffffff,0xd8d4c3,2.4));
  const key=new THREE.DirectionalLight(0xffffff,4);key.position.set(-1400,2100,-1900);key.castShadow=true;
  key.shadow.mapSize.set(2048,2048);Object.assign(key.shadow.camera,{left:-1700,right:1700,top:2200,bottom:-1000,near:1,far:7000});key.shadow.bias=-.0003;scene.add(key);
  const fill=new THREE.DirectionalLight(0xffd2a3,2.2);fill.position.set(1500,650,1100);scene.add(fill);
  const rim=new THREE.DirectionalLight(0xc8e5dd,3);rim.position.set(1000,2000,1400);scene.add(rim);
  root=new THREE.Group();scene.add(root);guides=new THREE.Group();scene.add(guides);rulerGroup=new THREE.Group();scene.add(rulerGroup);annotationGroup=new THREE.Group();scene.add(annotationGroup);
  environment=new THREE.Group();scene.add(environment);
  ground=new THREE.Mesh(new THREE.PlaneGeometry(10000,10000),new THREE.ShadowMaterial({opacity:.12}));ground.rotation.x=-Math.PI/2;ground.position.y=-3;ground.receiveShadow=true;environment.add(ground);
  const grid=new THREE.GridHelper(6000,60,0xc7ccbf,0xdfe1d7);grid.position.y=-2;grid.material.transparent=true;grid.material.opacity=.3;environment.add(grid);
  const circle=new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(Array.from({length:120},(_,i)=>new THREE.Vector3(Math.cos(i*Math.PI/60)*430,0,Math.sin(i*Math.PI/60)*430))),new THREE.LineBasicMaterial({color:0xc6cfc0,transparent:true,opacity:.7}));circle.position.y=-1;environment.add(circle);
  populateModels(); wireControls(); wireResources(); selectModel('F28_REFINED');
  new ResizeObserver(resize).observe(stage); resize();
  let pointerStart=null;
  canvas.addEventListener('pointerdown',e=>pointerStart=[e.clientX,e.clientY]);
  canvas.addEventListener('pointerup',e=>{if(pointerStart&&Math.hypot(e.clientX-pointerStart[0],e.clientY-pointerStart[1])<5)pick(e);pointerStart=null;});
  const animate=()=>{requestAnimationFrame(animate);controls.update();if(redraw){renderer.render(scene,camera);redraw=false;}};animate();
  $('loader').hidden=true; $('render-status').textContent='3D ready · models stored locally';
}
function resize() { if(!renderer)return;const r=stage.getBoundingClientRect();renderer.setSize(r.width,r.height,false);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();redraw=true; }
function populateModels() {
  const labels={F28_MEAN:['01','Female 28','Arithmetic mean'],F28_REFINED:['02','Female 28 · refined','Slight form adjustments'],SOPHY_SCALE:['03','SOPHY scale','1676.4 mm · canon reference'],A0_R2:['04','Shoulder · A0-R2','Located CAD / shop review'],REFERENCE_KIT:['05','Source library','Legacy component envelopes'],NIH_PELVIS:['06','NIH · female pelvis','Native frame / reduced preview'],NIH_SKIN:['07','NIH · female skin','Exterior / reduced preview'],NIH_PELVIS_ORIGINAL:['08','NIH · pelvis original','Verified local file / original faces']};
  $('models').replaceChildren();
  for(const m of data.models){const [glyph,label,note]=labels[m.id];const b=document.createElement('button');b.className='model-button';b.dataset.model=m.id;b.innerHTML='<span class="model-glyph">'+glyph+'</span><span><span class="model-label">'+label+'</span><div class="model-note">'+note+'</div></span>';b.onclick=()=>selectModel(m.id);$('models').append(b);}
}
function disposeGroup(group) { while(group.children.length){const c=group.children.pop();c.traverse(o=>{o.geometry?.dispose();if(o.material){for(const m of (Array.isArray(o.material)?o.material:[o.material])){m.map?.dispose();m.dispose();}}});c.parent=null;} }
function material(part) {
  const isWood=part.material==='redwood', shell=part.region==='envelope';
  return new THREE.MeshStandardMaterial({color:palette[state.palette][part.material]??0xb0b5b0,
    roughness:isWood?.58:.34,metalness:isWood?.04:.62,transparent:shell,opacity:shell?.105:1,
    depthWrite:!shell,side:shell?THREE.DoubleSide:THREE.FrontSide});
}
async function selectModel(id,render=true) {
  model=data.models.find(x=>x.id===id);if(!model)return;
  const chosen=model,sequence=++selectionSequence,native=isNative(model);
  state.loading=false;state.referenceCapture=false;$('capture-landmark').classList.remove('active');
  originalReview?.syncModel(model);
  if(native)comparisonModel=id;
  state.model=id;state.height=model.height_mm??0;state.pose=model.arm_drop_deg??0;state.selected=null;state.isolated=false;state.boneOnly=false;state.exploded=0;
  $('layer-bones-only').checked=false;$('layer-bones-only').disabled=!model.bone_equivalence;
  state.points=[];state.ruler=false;$('measure').classList.remove('active');$('explode').value=0;
  $('part-search').value='';
  document.querySelectorAll('[data-model]').forEach(b=>{b.classList.toggle('active',b.dataset.model===id);b.setAttribute('aria-pressed',b.dataset.model===id);});
  const body=!!model.height_mm, isA0=id==='A0_R2';
  $('model-title').textContent=body?'The female armature':isA0?'The shoulder article':'A library of parts';
  $('model-eyebrow').textContent=body?'FEMALE / 28 / REFERENCE ASSEMBLY':isA0?'A0-R2 / SINGLE-SIDE BENCH ARTICLE':'PHY / SOURCE COMPONENT LIBRARY';
  $('model-subtitle').textContent=body?'Measured proportions. Considered form.':isA0?'35 located instances. Exact current CAD geometry.':'Reference envelopes, arranged for inspection.';
  $('model-badge').textContent=id==='F28_MEAN'?'ARITHMETIC MEAN · AGE 28':id==='F28_REFINED'?'MEAN-BASED · GENTLY REFINED':id==='SOPHY_SCALE'?'CANON SCALE / PROPOSED FORM OVERLAY':isA0?'SHOP REVIEW · PHYSICAL QUALIFICATION OPEN':'UNASSEMBLED / DISPLAY LAYOUT ONLY';
  $('profile-note').textContent=body?(id==='F28_MEAN'?'Measured reference dimensions; mechanical station mapping is a design proposal.':id==='SOPHY_SCALE'?'H = span = 1676.4 mm. This form overlay does not adopt new canon landmarks.':'92 women aged exactly 28. Shoulder −1%, waist −3%, hip +2%.'):isA0?'A0-R2 stays in its original bench datum and dimensions.':'Gallery positions are display transforms, not anatomical placement.';
  $('source-note').textContent=body?'ANSUR II, female US Army cohort (2010–2012), age exactly 28; not a civilian/worldwide mean. 92 samples. External dimensions differ from mechanical centers. Arm station closure factor '+round(model.design_datums.arm_chain_closure_scale,4)+'. See Reference for every mean and limitation.':model.limitations.join(' ');
  $('pose-note').textContent=body?'Visual arm motion around proposed centers.':isA0?'Neutral A0 assembly. Its physical motion screen remains discrete.':'Unassembled component gallery.';
  if(native){
    $('model-title').textContent=id==='NIH_SKIN'?'The female exterior':'The female pelvis';
    $('model-eyebrow').textContent='NIH / HRA / NATIVE REFERENCE';
    $('model-subtitle').textContent='Source geometry in a separate native frame.';
    $('model-badge').textContent=id==='NIH_PELVIS_ORIGINAL'?'VERIFIED ORIGINAL / ANATOMICAL QUALIFICATION OPEN':'DECIMATED PREVIEW / ANATOMICAL QUALIFICATION OPEN';
    $('profile-note').textContent='Source world coordinates preserved. 1000 mm/m display conversion only; no stature normalization.';
    $('pose-note').textContent='Native static source pose. No PHY joint axes or anatomical registration.';
  }
  if(environment)environment.visible=!native;
  $('axis-note').textContent=native?'SOURCE X · Y UP · Z / m → mm ×1000':'mm / X RIGHT · Y FRONT · Z UP';
  $('canvas-note').textContent=native?'SEPARATE NATIVE VIEW · CAMERA ORBIT ONLY · NO REGISTRATION':'DRAG TO ORBIT · SCROLL / PINCH TO ZOOM · CLICK A PART';
  $('reference-review').hidden=id!=='NIH_PELVIS';
  $('original-review').hidden=!data.native_references?.original_review_contract;
  $('compare-reference').hidden=!data.native_references || (!native&&id!=='SOPHY_SCALE');
  $('compare-reference').textContent=native?'View SOPHY separately ↗':'Return to NIH native view ↗';
  for(const control of ['explode','mat-wood','mat-carbon','layer-frame','layer-joints','layer-dimensions'])$(control).disabled=native;
  for(const option of $('export-format').options)option.disabled=native&&option.value==='stl';
  if(native&&$('export-format').value==='stl')$('export-format').value='glb';
  $('export-note').textContent=id==='NIH_PELVIS_ORIGINAL'?'GLB saves the verified original bytes. JSON exports the unaccepted original review packet.':native?'GLB downloads the identical pinned preview in meters. JSON retains draft/provenance. No transformed STL or fabrication mesh export.':'Current scale and pose. Reference geometry; full-scale structural release is open.';
  $('export').disabled=!renderer&&!native;
  updateDraft();
  for(const input of ['height','height-number','pose','pose-a','pose-t'])$(input).disabled=!body;
  $('layer-envelope').disabled=!body;
  $('height').value=state.height;$('height-number').value=body?round(state.height):'';$('pose').value=state.pose;
  $('part-count').textContent=model.parts.length+' parts';
  updatePartList();inspectPart(null);updateStats();updatePoseLabel();
  if(!renderer||!render)return;
  disposeGroup(root);disposeGroup(guides);disposeGroup(rulerGroup);disposeGroup(annotationGroup);meshes=[];
  root.position.set(0,0,0);root.quaternion.identity();root.scale.setScalar(1);
  if(native){
    state.loading=true;$('render-status').textContent='Checking pinned native geometry…';
    try{
      const gltf=chosen._originalSurface?{scene:chosen._originalSurface.scene()}:await new GLTFLoader().parseAsync((await previewBytes(chosen)).buffer,'');
      if(sequence!==selectionSequence){disposeGroup(gltf.scene);return;}
      gltf.scene.traverse(mesh=>{if(!mesh.isMesh)return;const part=chosen.parts.find(p=>p.id===mesh.name);if(!part)throw new Error('Native mesh label mismatch');mesh.userData.part=part;mesh.castShadow=true;mesh.receiveShadow=true;meshes.push(mesh);});
      if(meshes.length!==chosen.parts.length)throw new Error('Native mesh count mismatch');
      root.add(gltf.scene);root.scale.setScalar(1000);
      root.userData={profile:id,units:'m',display_units:'mm',display_scale:1000,canonical_transform:null};
      state.loading=false;$('render-status').textContent=(chosen._originalSurface?'Verified local original':'Pinned native preview')+' · anatomy/metric fidelity unqualified';
      applyTransforms();cameraView('iso');updateDraft();return;
    }catch(e){if(sequence===selectionSequence){state.loading=false;meshes=[];disposeGroup(root);$('render-status').textContent='Reference load blocked: '+e.message;toast('Reference load blocked: '+e.message);}return;}
  }
  for(const p of model.parts){
    const geometry=new THREE.BufferGeometry();const positions=[];
    for(const v of p.vertices)positions.push(v[0],v[2],-v[1]);
    geometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));geometry.setIndex(p.faces.flat());geometry.computeVertexNormals();geometry.computeBoundingSphere();
    const mesh=new THREE.Mesh(geometry,material(p));mesh.name=p.id;mesh.userData={part:p};mesh.castShadow=p.region!=='envelope';mesh.receiveShadow=true;root.add(mesh);meshes.push(mesh);
  }
  root.userData={profile:id,geometry_status:'reference / unreleased',units:'mm'};
  $('render-status').textContent='3D ready · models stored locally';
  applyTransforms();cameraView('iso');redraw=true;
}
function updateStats() {
  const body=!!model.height_mm;
  if(isNative(model)){
    const vals=[model.parts.length+' <span>source meshes</span>',model.derivative.preview_triangles.toLocaleString()+' <span>triangles</span>','m <span>native / Y up</span>'];
    vals.forEach((v,i)=>{$('stat'+(i+1)).innerHTML=v;$('stat'+(i+1)+'-label').textContent=['Segmentation',model._originalSurface?'Original triangles':'Reduced preview','Source frame'][i];});return;
  }
  const vals=body?[round(state.height)+' <span>mm</span>',round(model.t_pose_span_mm*state.height/model.height_mm)+' <span>mm</span>','92 <span>women / age 28</span>']:model.id==='A0_R2'?[model.part_definition_count+' <span>CAD parts</span>',model.parts.length+' <span>located instances</span>','317 <span>mm / S–E station</span>']:['18 <span>source solids</span>','mm <span>canonical units</span>','0 <span>anatomical transforms</span>'];
  const labels=body?['Standing height','T-pose span','Source cohort']:model.id==='A0_R2'?['Unique geometry','Assembly','Dummy member']:['Library','Dimensions','Placement'];
  vals.forEach((v,i)=>{$('stat'+(i+1)).innerHTML=v;$('stat'+(i+1)+'-label').textContent=labels[i];});
}
function updatePoseLabel() { $('pose-label').textContent=model.height_mm?(state.pose===0?'T / 0°':'A / '+state.pose+'°'):'Neutral';$('pose-a').classList.toggle('active',state.pose!==0);$('pose-t').classList.toggle('active',state.pose===0); }
function applyTransforms() {
  if(!root)return;
  if(isNative(model)){
    for(const mesh of meshes){mesh.visible=!state.isolated||mesh.name===state.selected;for(const mat of Array.isArray(mesh.material)?mesh.material:[mesh.material]){mat.emissive?.setHex(mesh.name===state.selected?0x503a16:0);mat.emissiveIntensity=mesh.name===state.selected?.4:0;}}
    disposeGroup(guides);guides.visible=false;redraw=true;return;
  }
  const scale=model.height_mm?state.height/model.height_mm:1;root.scale.setScalar(scale);
  const origin=model.height_mm?new THREE.Vector3(0,model.height_mm*.56,0):coord([ -240,0,40]);
  for(const mesh of meshes){
    const p=mesh.userData.part;
    mesh.position.set(0,0,0);mesh.quaternion.identity();
    const match=p.id.match(/_(R|L)(?:_|$)/), movable=match&&['arms','hands'].includes(p.region)&&!p.id.startsWith('shoulder');
    if(movable&&model.height_mm){
      const side=match[1],pivot=coord(model.landmarks['shoulder_'+side]);
      const angle=(model.arm_drop_deg-state.pose)*Math.PI/180*(side==='R'?1:-1);
      mesh.quaternion.setFromAxisAngle(new THREE.Vector3(0,0,1),angle);
      mesh.position.copy(pivot).sub(pivot.clone().applyQuaternion(mesh.quaternion));
    }
    if(state.exploded){const c=coord(p.bounds_mm.min).add(coord(p.bounds_mm.max)).multiplyScalar(.5);const direction=c.sub(origin);if(direction.length()>1)mesh.position.add(direction.normalize().multiplyScalar(state.exploded*(model.height_mm?2.5:1.5)));}
    mesh.visible=partVisible(p,state,{envelope:$('layer-envelope').checked,joints:$('layer-joints').checked,frame:$('layer-frame').checked});
    mesh.material.wireframe=false;
    mesh.material.emissive.setHex(p.id===state.selected?0x503a16:0);
    mesh.material.emissiveIntensity=p.id===state.selected?.4:0;
  }
  $('explode-label').textContent=state.exploded+'%';
  makeDimensions();redraw=true;
}
function line(a,b,color=0x899c8a,group=guides) { const mesh=new THREE.Line(new THREE.BufferGeometry().setFromPoints([a,b]),new THREE.LineBasicMaterial({color,transparent:true,opacity:.8}));group.add(mesh);return mesh; }
function label(text,position,size=65,group=guides) {
  const c=document.createElement('canvas');c.width=400;c.height=88;const ctx=c.getContext('2d');ctx.fillStyle='#f2f1ebe8';ctx.fillRect(0,0,c.width,c.height);ctx.font='40px system-ui';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillStyle='#506552';ctx.fillText(text,200,44);
  const texture=new THREE.CanvasTexture(c);texture.colorSpace=THREE.SRGBColorSpace;
  const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,depthTest:false,transparent:true,toneMapped:false}));sprite.position.copy(position);sprite.scale.set(size*4.5,size,1);group.add(sprite);return sprite;
}
function makeDimensions() {
  disposeGroup(guides);guides.visible=$('layer-dimensions').checked;
  if(!model.height_mm)return;
  const h=state.height,x=-280*h/sourceMean,z=45;
  line(new THREE.Vector3(x,0,z),new THREE.Vector3(x,h,z));
  for(const y of [0,h])line(new THREE.Vector3(x-13,y,z),new THREE.Vector3(x+13,y,z));
  label(round(h)+' mm',new THREE.Vector3(x-85,h*.51,z),h*.048);
  if(state.pose===0){const span=model.t_pose_span_mm*h/model.height_mm,y=model.design_datums.shoulder_z_mm*h/model.height_mm+90;
    line(new THREE.Vector3(-span/2,y,0),new THREE.Vector3(span/2,y,0));label(round(span)+' mm span',new THREE.Vector3(0,y+38,0),h*.048);}
}
function cameraView(view='iso',target=null,size=null) {
  if(!camera)return;state.view=view;
  // Mesh-only bounds queries do not update the parent's native display scale.
  root.updateMatrixWorld(true);
  const box=new THREE.Box3();for(const m of meshes)if(m.visible)box.expandByObject(m);if(box.isEmpty())box.setFromObject(root);
  const extent=box.getSize(new THREE.Vector3()),center=target??box.getCenter(new THREE.Vector3());
  const d=size??Math.max(extent.y,extent.x,extent.z,180)*(innerWidth<760?2.5:2.18);
  controls.target.copy(center);const directions={iso:new THREE.Vector3(-.42,.14,-1),front:new THREE.Vector3(0,.035,-1),side:new THREE.Vector3(-1,.035,0),back:new THREE.Vector3(0,.035,1)};
  camera.position.copy(center).add(directions[view].clone().normalize().multiplyScalar(d));camera.near=Math.max(.1,d/1000);camera.far=Math.max(25000,d*10);camera.updateProjectionMatrix();controls.update();
  document.querySelectorAll('[data-view]').forEach(b=>{b.classList.toggle('active',b.dataset.view===view);if(b.dataset.view!=='iso')b.textContent=isNative(model)?{front:'−Z view',side:'−X view',back:'+Z view'}[b.dataset.view]:{front:'Front',side:'Side',back:'Back'}[b.dataset.view];});redraw=true;
}
function updatePartList() {
  const query=$('part-search').value.toLowerCase();const select=$('part-select');select.replaceChildren(new Option('Select in the viewport',''));
  for(const p of model.parts)if(partMatches(p,query,state.boneOnly))select.add(new Option(p.name+(p.bone_id?' · '+p.bone_id:''),p.id));
  select.value=state.selected??'';
}
function inspectPart(id) {
  state.selected=id;const p=model.parts.find(x=>x.id===id);$('part-select').value=id??'';
  if(!p){$('part-details').innerHTML='<p class="quiet">Click the model to inspect a part, its dimensions, and its source.</p>';if(renderer)applyTransforms();return;}
  const dims=p.bounds_mm.max.map((v,i)=>(v-p.bounds_mm.min[i])*(model.height_mm?state.height/model.height_mm:1));
  $('part-details').innerHTML='<p><strong>'+escape(p.name)+'</strong><br>'+escape(p.id)+'</p>'+(p.role?'<p>Role: '+escape(p.role)+(p.bone_id?'<br>'+escape(p.bone_id)+' · dimensional fidelity unverified':p.grouped_bone_ids?'<br>'+p.grouped_bone_ids.length+' grouped identities · not individual bones':'')+'</p>':'')+(p.source_record_status?'<p class="quiet">Canonical source record '+escape(p.source_record_status)+'</p>':'')+'<p>'+dims.map(x=>round(x)).join(' × ')+' mm <span class="quiet">/ axis-aligned envelope</span></p><p class="quiet">'+escape(p.authority)+'<br>'+escape(p.source)+'</p>'+(p.center_distance_mm?'<p>Member center distance: '+round(p.center_distance_mm*(model.height_mm?state.height/model.height_mm:1))+' mm</p>':'');
  if(p.bone_id)$('part-details').innerHTML+=canonicalDetails(p.bone_id);
  if(isNative(model))$('part-details').innerHTML+='<p class="quiet">Native X/Y/Z display envelope only; not a bone measurement.<br>Original source node '+p.source_node+(model._originalSurface?' · original primitive '+p.primitive_index:' · preview node '+p.preview_node)+' · '+p.triangle_count+' triangles<br>Source '+escape(model.source.sha256)+(model._originalSurface?'':'<br>Preview '+escape(model.derivative.preview_sha256))+'</p>';
  applyTransforms();
}
function canonicalDetails(uid) {
  const available=new Set(model.parts.filter(p=>p.bone_id).map(p=>p.bone_id));
  return boneEvidenceHtml(data.engineering,uid)+relationshipsHtml(data.engineering.relationships,uid,available);
}
function inspectRelatedBone(uid) {
  const part=model.parts.find(p=>p.bone_id===uid);
  if(!part){toast('No proxy for this identity in the current model.');return false;}
  $('part-search').value='';updatePartList();inspectPart(part.id);
  const selector=$('evidence-bone');
  if(selector){selector.value=uid;$('evidence-details').innerHTML=canonicalDetails(uid);}
  return true;
}
function pick(event) {
  if(state.loading)return;
  const rect=canvas.getBoundingClientRect(),point=new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1),ray=new THREE.Raycaster();ray.setFromCamera(point,camera);
  const hit=ray.intersectObjects(meshes.filter(m=>m.visible&&m.userData.part.region!=='envelope'),false)[0];if(!hit)return;
  if(originalReview?.pick(hit))return;
  if(state.referenceCapture&&model.id==='NIH_PELVIS'){
    try{const feature=data.native_references.landmark_packet.landmarks.find(p=>p.id===$('landmark-candidate').value),annotation=annotationFromHit(hit,feature,model);
      referenceDraft.annotations=referenceDraft.annotations.filter(p=>p.landmark_id!==annotation.landmark_id);referenceDraft.annotations.push(annotation);
      state.referenceCapture=false;$('capture-landmark').classList.remove('active');updateDraft();toast('Saved an unaccepted preview candidate. Original-surface review remains required.');
    }catch(e){toast(e.message);}return;
  }
  if(state.ruler){
    if(state.points.length===2){state.points=[];disposeGroup(rulerGroup);}
    state.points.push(hit.point.clone());const ball=new THREE.Mesh(new THREE.SphereGeometry(Math.max(state.height*.004,3),12,8),new THREE.MeshBasicMaterial({color:0xb66a48,depthTest:false}));ball.position.copy(hit.point);rulerGroup.add(ball);
    if(state.points.length===2){const [a,b]=state.points;line(a,b,0xb66a48,rulerGroup);const distance=a.distanceTo(b);label(round(distance)+' mm',a.clone().add(b).multiplyScalar(.5),Math.max(state.height*.028,12),rulerGroup);toast((isNative(model)?'Unqualified preview distance: ':'Surface-to-surface distance: ')+round(distance)+' mm');}
    else toast('Choose the second surface point.');redraw=true;
  } else inspectPart(hit.object.name);
}
function updateDraft(){
  if(!referenceDraft)return;
  $('draft-count').textContent=referenceDraft.annotations.length+'/5 preview candidates';
  $('draft-points').textContent=referenceDraft.annotations.map(p=>p.landmark_id+': ['+p.native_point_m.map(v=>round(v,6)).join(', ')+'] m · preview face '+p.face_index).join('\n');
  if(!annotationGroup)return;disposeGroup(annotationGroup);
  if(model?.id==='NIH_PELVIS'&&!state.loading)for(const p of referenceDraft.annotations){const ball=new THREE.Mesh(new THREE.SphereGeometry(1.5,12,8),new THREE.MeshBasicMaterial({color:0xb66a48,depthTest:false}));ball.position.fromArray(p.native_point_m).multiplyScalar(1000);annotationGroup.add(ball);}
  if(!state.loading)originalReview?.overlays(annotationGroup);
  redraw=true;
}
function wireControls() {
  $('height').oninput=()=>setHeight(Number($('height').value));$('height-number').onchange=()=>setHeight(Number($('height-number').value));
  $('pose').oninput=()=>setPose(Number($('pose').value));$('pose-a').onclick=()=>setPose(75);$('pose-t').onclick=()=>setPose(0);
  for(const id of ['layer-frame','layer-joints','layer-envelope','layer-dimensions'])$(id).onchange=applyTransforms;
  $('layer-bones-only').onchange=()=>{state.boneOnly=$('layer-bones-only').checked;state.isolated=false;
    if(state.boneOnly&&model.parts.find(p=>p.id===state.selected)?.role!=='bone_proxy')state.selected=null;
    state.points=[];disposeGroup(rulerGroup);updatePartList();inspectPart(state.selected);cameraView(state.view);};
  $('explode').oninput=()=>{state.exploded=Number($('explode').value);applyTransforms();};
  $('part-search').oninput=updatePartList;$('part-select').onchange=()=>inspectPart($('part-select').value||null);
  $('isolate').onclick=()=>{if(!state.selected){toast('Select a component first.');return;}state.isolated=true;applyTransforms();cameraView(state.view);};
  $('show-all').onclick=()=>{state.isolated=false;inspectPart(null);cameraView(state.view);};
  $('focus-part').onclick=()=>{const m=meshes.find(x=>x.name===state.selected);if(!m){toast('Select a component first.');return;}const b=new THREE.Box3().setFromObject(m);cameraView(state.view,b.getCenter(new THREE.Vector3()),Math.max(...b.getSize(new THREE.Vector3()).toArray(),45)*2.5);};
  $('mat-wood').onclick=()=>setPalette('wood');$('mat-carbon').onclick=()=>setPalette('carbon');
  document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>cameraView(b.dataset.view));
  $('measure').onclick=()=>{originalReview?.stop();state.referenceCapture=false;$('capture-landmark').classList.remove('active');state.ruler=!state.ruler;$('measure').classList.toggle('active',state.ruler);state.points=[];disposeGroup(rulerGroup);if(state.ruler)toast(isNative(model)?'Click two preview points. Distances are not qualified osteometry.':'Click two points on the frame to measure.');redraw=true;};
  $('compare-reference').onclick=()=>selectModel(isNative(model)?'SOPHY_SCALE':comparisonModel);
  $('capture-landmark').onclick=()=>{if(state.loading||!renderer)return;state.ruler=false;$('measure').classList.remove('active');state.points=[];disposeGroup(rulerGroup);state.referenceCapture=!state.referenceCapture;$('capture-landmark').classList.toggle('active',state.referenceCapture);if(state.referenceCapture)toast('Click the specified cortical surface for a draft point.');};
  $('landmark-candidate').onchange=()=>{const p=data.native_references.landmark_packet.landmarks.find(p=>p.id===$('landmark-candidate').value);$('landmark-definition').textContent=p.definition+' '+p.review_rule;inspectPart(p.preview_mesh_names[0]);};
  $('download-draft').onclick=()=>download(JSON.stringify(referenceDraft,null,2),'PHY-pelvis-preview-draft.json','application/json');
  $('clear-draft').onclick=()=>{referenceDraft=emptyDraft(data.native_references);updateDraft();};
  $('export').onclick=exportModel;
}
function setHeight(value) { if(!model.height_mm)return;if(!Number.isFinite(value)||value<1400||value>1900){toast('Use a height from 1400 to 1900 mm.');$('height-number').value=round(state.height);return;}state.height=value;$('height').value=value;$('height-number').value=round(value);state.points=[];disposeGroup(rulerGroup);updateStats();applyTransforms();if(state.selected)inspectPart(state.selected); }
function setPose(value) { if(!model.height_mm)return;state.pose=Math.max(0,Math.min(80,value));$('pose').value=state.pose;updatePoseLabel();state.points=[];disposeGroup(rulerGroup);applyTransforms(); }
function setPalette(value) { if(isNative(model))return;state.palette=value;$('mat-wood').classList.toggle('active',value==='wood');$('mat-carbon').classList.toggle('active',value==='carbon');for(const mesh of meshes){mesh.material.color.setHex(palette[value][mesh.userData.part.material]??0xb0b5b0);}redraw=true; }
function exportGroup() {
  return makeExportGroup(root,meshes,state.boneOnly);
}
async function exportModel() {
  if(isNative(model)){
    const selectedModel=model,format=$('export-format').value;
    try{
      if(format==='stl'){toast('Native preview STL export is unavailable. Use the pinned GLB or draft JSON.');return;}
      if(selectedModel._originalSurface){
        if(format==='glb')download(selectedModel._originalSurface.raw,'VH_F_Pelvis.glb','model/gltf-binary');
        else download(canonicalJson(originalReview.packet()),'PHY-pelvis-original-review-packet.json','application/json');
        return;
      }
      if(format==='glb')download(await previewBytes(selectedModel),selectedModel.derivative.preview_filename,'model/gltf-binary');
      else download(JSON.stringify({reference_model:{...selectedModel,glb_base64:undefined},
        review_state:{units:'m',frame:selectedModel.frame,display_conversion_mm_per_m:1000,canonical_transform:null,fabrication_released:false,build:data.build},
        landmark_draft:selectedModel.id==='NIH_PELVIS'?referenceDraft:null,native_reference_evidence:data.native_references,
        relationship_graph:data.engineering.relationships},null,2),'PHY-'+selectedModel.id+'-native-review.json','application/json');
      toast('Exported pinned native reference / unaccepted review data.');
    }catch(e){toast('Reference export blocked: '+e.message);}return;
  }
  if(!renderer){toast('3D exports need WebGL. The maquette package remains available.');return;}
  const format=$('export-format').value,name='PHY-'+state.model+'-'+round(state.height??0,0)+'mm';
  const review={...state,points:[],units:'mm',fabrication_released:false,source_snapshot_sha256:data.reference.snapshot_sha256,build:data.build,notes:model.limitations};
  try{
    if(format==='json'){download(JSON.stringify({model,review_state:review,engineering_evidence:data.engineering},null,2),name+'.json','application/json');}
    else{
      const group=exportGroup();group.userData=review;
      if(format==='stl'){group.rotateX(Math.PI/2);group.updateMatrixWorld(true);download(new STLExporter().parse(group,{binary:true}),name+'.stl','model/stl');}
      else {group.scale.setScalar(.001);group.userData.units='meters / glTF standard';group.updateMatrixWorld(true);const result=await new GLTFExporter().parseAsync(group,{binary:true});download(result,name+'.glb','model/gltf-binary');}
    }
    toast('Exported current scale, pose and separation. Reference geometry.');
  }catch(e){console.error(e);toast('Export failed: '+e.message);}
}
// Small ZIP writer with stored entries: maquette SVGs, BOM and instructions are embedded.
function zipFiles(files) {
  const encoder=new TextEncoder(), chunks=[],central=[],crcTable=Array.from({length:256},(_,n)=>{let c=n;for(let k=0;k<8;k++)c=(c&1)?0xedb88320^(c>>>1):c>>>1;return c>>>0;});let offset=0;
  const header=(size)=>new Uint8Array(size),write=(buffer,at,value,bytes)=>{for(let i=0;i<bytes;i++)buffer[at+i]=(value>>>(i*8))&255;};
  for(const [name,text] of Object.entries(files)){
    const filename=encoder.encode(name),body=encoder.encode(text);let crc=0xffffffff;for(const b of body)crc=crcTable[(crc^b)&255]^(crc>>>8);crc=(crc^0xffffffff)>>>0;
    const h=header(30);write(h,0,0x04034b50,4);write(h,4,20,2);write(h,6,0x800,2);write(h,12,33,2);write(h,14,crc,4);write(h,18,body.length,4);write(h,22,body.length,4);write(h,26,filename.length,2);chunks.push(h,filename,body);
    const c=header(46);write(c,0,0x02014b50,4);write(c,4,20,2);write(c,6,20,2);write(c,8,0x800,2);write(c,14,33,2);write(c,16,crc,4);write(c,20,body.length,4);write(c,24,body.length,4);write(c,28,filename.length,2);write(c,42,offset,4);central.push(c,filename);offset+=h.length+filename.length+body.length;
  }
  const end=header(22),count=Object.keys(files).length,size=central.reduce((s,c)=>s+c.length,0);write(end,0,0x06054b50,4);write(end,8,count,2);write(end,10,count,2);write(end,12,size,4);write(end,16,offset,4);return new Blob([...chunks,...central,end],{type:'application/zip'});
}
function maquetteDownload() { if(!data.maquette_files){toast('Rebuild PHY Studio to embed the maquette cut files.');return;}download(zipFiles(data.maquette_files),'PHY-F28-quarter-scale-maquette.zip');toast('Cut sheets, assembly stencil, BOM and instructions downloaded.'); }
function openModal(tab) {
  if(!$('modal-backdrop').classList.contains('open'))lastFocus=document.activeElement;
  $('modal-backdrop').classList.add('open');document.querySelector('.modal').focus();
  document.querySelectorAll('[data-tab]').forEach(b=>b.classList.toggle('active',b.dataset.tab===tab));
  const ref=data.reference,stats=ref.statistics;
  if(tab==='reference'){
    $('modal-title').textContent='Proportion reference';
    $('modal-content').innerHTML='<p>Arithmetic means of <b>92 women aged exactly 28</b> from ANSUR II. '+escape(ref.population)+'</p><p>The baseline retains the measured mean stature and span. The refinement uses shoulder −1%, waist −3%, and hip +2%. Marginal means are a design reference, not one real individual or an objective beauty standard.</p><p>Joint centers, head height, curved formers and the interpolated form envelope are proposed mechanical datums. External segment ratios are mapped to the measured span; they are not osteometric bone lengths.</p>'+htmlTable(['MEASUREMENT','MEAN / mm','SAMPLE SD / mm','n'],Object.entries(stats).map(([k,v])=>[k,round(v.mean_mm,3),round(v.sd_mm,3),v.n]))+'<p><a href="'+escape(ref.report_url)+'" target="_blank" rel="noopener">ANSUR II report</a> · <a href="'+escape(ref.data_url)+'" target="_blank" rel="noopener">Public data source</a></p><p class="small-note">Projected snapshot SHA-256: '+escape(ref.snapshot_sha256)+'</p>';
  } else if(tab==='nih'){
    $('modal-title').textContent='NIH native references and landmark review';
    $('modal-content').innerHTML=nativeReferenceHtml(data);
  } else if(tab==='engineering'){
    $('modal-title').textContent='Engineering evidence';
    $('modal-content').innerHTML=engineeringHtml(data.engineering,data.build);
    const selector=$('evidence-bone');
    const selected=model.parts.find(p=>p.id===state.selected)?.bone_id;
    if(selected)selector.value=selected;
    selector.onchange=()=>{const uid=selector.value;$('evidence-details').innerHTML=canonicalDetails(uid);const part=model.parts.find(p=>p.bone_id===uid);if(part)inspectPart(part.id);};
    $('evidence-details').innerHTML=canonicalDetails(selector.value);
  } else if(tab==='nasa'){
    $('modal-title').textContent='NASA positioning and ranges';
    $('modal-content').innerHTML=nasaHtml(data.engineering);
    const selector=$('nasa-condition');
    selector.onchange=()=>{
      const condition=selector.value,count=data.engineering.nasa_counts.conditions[condition];
      $('nasa-condition-count').textContent=conditionLabels[condition]+' · '+count+' motion pairs';
      $('nasa-motion').innerHTML=nasaMotionHtml(data.engineering.nasa,condition);
    };
    selector.onchange();
  } else if(tab==='bones'){
    const body=model.bone_equivalence?model:data.models.find(m=>m.bone_equivalence),audit=body.bone_equivalence;
    $('modal-title').textContent='Bone equivalence / distribution';
    $('modal-content').innerHTML='<p>Reference-body scope only; A0 and the source gallery are separate. '+audit.expected_bones+' adult identities: <b>'+audit.individual_bone_proxies+' individual project proxies</b>, '+audit.grouped_bones+' grouped-form identities, '+audit.unrepresented_bones+' unrepresented. Hardware/supports do not count.</p><p class="warning">Dimensions and morphology are unverified project proposals, not osteometry or canon adoption. Coverage is not fabrication readiness.</p>'+htmlTable(['REGION','EXPECTED','INDIVIDUAL','GROUPED','UNREPRESENTED'],audit.regions.map(r=>[r.region,r.expected,r.individual,r.grouped,r.unrepresented]))+'<p>'+audit.source_records_present+' canonical source records; '+audit.missing_source_records.length+' records absent. Source presence does not qualify dimensions, geometry or mechanics.</p><details><summary>Missing source records</summary><ul>'+audit.missing_source_records.map(id=>'<li>'+escape(id)+'</li>').join('')+'</ul></details><details><summary>All 206 identities / current disposition</summary>'+htmlTable(['BONE ID','DISPOSITION','MESH','SOURCE'],audit.bones.map(r=>[r.bone_id,r.representation,r.mesh_ids.join(', ')||'—',r.source_record||'MISSING']))+'</details>';
  } else if(tab==='bom'){
    $('modal-title').textContent='Quarter-scale maquette';
    $('modal-content').innerHTML='<p>Supported, passive form study · '+round(data.maquette.height_mm)+' mm tall · '+data.maquette.cut_parts+' cut parts · '+data.maquette.cut_sheets+' A3 sheets. Nominal 3 mm birch plywood. Verify stock, hole coupons and cutter kerf before cutting.</p>'+htmlTable(['ID','QTY','DESCRIPTION','STOCK / SIZE','ASSEMBLY NOTE'],data.maquette.bill_of_materials.map(r=>[r.id,r.qty,r.description,r.stock+' / '+round(r.width_mm)+' × '+round(r.length_mm)+' mm',r.note]))+'<button id="modal-download" class="primary">Download the cut and assembly package ↗</button>';
    $('modal-download').onclick=maquetteDownload;
  } else if(tab==='build'){
    $('modal-title').textContent='From reference to physical form';
    $('modal-content').innerHTML='<div class="build-steps"><div class="build-step"><span class="num">01 / SEE</span><h3>Complete reference assembly</h3><span class="tag">DEMONSTRABLE</span><p>Whole-body bilateral armature, source means, explicit mechanical datums, materials, poses and transferable meshes.</p></div><div class="build-step"><span class="num">02 / MAKE</span><h3>Supported 1:4 maquette</h3><span class="tag">CUT PACKAGE AVAILABLE</span><p>Actual-size SVG cut paths, plywood parts, copper pins, rear support post, assembly stencil and bill of materials.</p></div><div class="build-step"><span class="num">03 / PROVE</span><h3>A0-R2 shoulder article</h3><span class="tag">SHOP REVIEW</span><p>Revised cartridge geometry and manufacturing drawings. Stop compliance, purchased fits, remaining article interfaces and physical measurements remain open.</p></div><div class="build-step"><span class="num">04 / INTEGRATE</span><h3>Full-scale functional armature</h3><span class="tag">ENGINEERING OPEN</span><p>Resolve whole-body interfaces, joint mechanisms, retained motion, loads, support, balance and fabrication inspection. A form study does not close these gates.</p></div></div><p>Start with the quarter-scale physical form. Use its measurements to review the proportions and record actual assembly observations.</p><button id="modal-download" class="primary">Get the maquette package ↗</button>';
    $('modal-download').onclick=maquetteDownload;
  } else {
    $('modal-title').textContent='Fabrication readiness';
    const a0=data.models.find(m=>m.id==='A0_R2');
    $('modal-content').innerHTML='<p>The Studio and whole-body form-study model are demonstrable. The maquette has reproducible cut geometry and assembly instructions. The full-scale armature has <b>no structural fabrication release or physical qualification</b>.</p>'+htmlTable(['DELIVERABLE','CURRENT STATE'],[['Female-28 reference assembly','Complete visual reference / explicit design proposals'],['Quarter-scale supported maquette','Cut/assembly package; physical build unmeasured'],['SOPHY canon 1.0.0','Kernel unchanged; scale comparison overlay only'],['A0-R2 shoulder','Shop-review candidate; release and bench evidence open'],['Whole-body functional armature','Mechanism integration and load qualification open']])+(a0?'<h3>A0-R2 release blockers</h3><p class="warning">The revised cartridge remains blocked for release: the assumed 4 mm stop compression is unverified and conflicts with the hard-stop angle limits. Bearing fits and remaining article interfaces also need review.</p><ul>'+a0.limitations.slice(2).map(n=>'<li>'+escape(n)+'</li>').join('')+'</ul><p>Six load cases and five evidence packages remain unapproved. The 35-pose centerline screen is not a continuous swept-solid clearance proof.</p>':'<p>A0 geometry was omitted in this standard-library build.</p>')+'<h3>Current model assumptions</h3><ul>'+model.limitations.map(n=>'<li>'+escape(n)+'</li>').join('')+'</ul>';
    $('modal-content').innerHTML+='<p>Canonical evidence: '+data.engineering.dimension_expansion.current_complete+'/206 complete declared maps; '+data.engineering.metrics.record_validation_pass.count+'/206 records pass the current validator. See Engineering for sources and unresolved fields.</p>';
  }
}
function closeModal(){$('modal-backdrop').classList.remove('open');lastFocus?.focus();}
function wireResources(){
  if(data.native_references?.original_review_contract){
    originalReview=createOriginalReview({data,download,selectModel,changed:updateDraft,renderAvailable:!!renderer,
      prepareCapture(){state.ruler=false;state.points=[];$('measure').classList.remove('active');disposeGroup(rulerGroup);},
      activate(ctx){
        const base=data.models.find(m=>m.id==='NIH_PELVIS'),original={...base,id:'NIH_PELVIS_ORIGINAL',kind:'nih_native_original',parts:ctx.surface.parts,
          derivative:{...base.derivative,preview_triangles:ctx.surface.parts.reduce((n,p)=>n+p.triangle_count,0)},glb_base64:undefined};
        Object.defineProperty(original,'_originalSurface',{value:ctx.surface});
        const at=data.models.findIndex(m=>m.id===original.id);if(at<0)data.models.push(original);else data.models[at]=original;populateModels();
      },
      focus(id){inspectPart(id);state.isolated=true;applyTransforms();cameraView(state.view);},
      getView:()=>({camera_position_mm:camera.position.toArray(),target_mm:controls.target.toArray()}),
      restoreView(view){camera.position.fromArray(view.camera_position_mm);controls.target.fromArray(view.target_mm);controls.update();redraw=true;},
      snapshot(){renderer.render(scene,camera);canvas.toBlob(blob=>download(blob,'PHY-original-annotated-view.png','image/png'));
        download(canonicalJson({viewer_build_sha256:data.build.package_input_sha256,user_agent:navigator.userAgent,viewport_px:[canvas.width,canvas.height],view:{camera_position_mm:camera.position.toArray(),target_mm:controls.target.toArray()},note:'Review context only; image bytes are not geometric proof and vary across renderers.'}),'PHY-original-annotated-view-context.json','application/json');}
    });
  }
  if(data.native_references){
    for(const p of data.native_references.landmark_packet.landmarks)$('landmark-candidate').add(new Option(p.label,p.id));
    const p=data.native_references.landmark_packet.landmarks[0];$('landmark-definition').textContent=p.definition+' '+p.review_rule;
  }
  $('download-maquette').onclick=maquetteDownload;$('open-reference').onclick=()=>openModal('reference');$('open-build').onclick=()=>openModal('build');$('open-readiness').onclick=()=>openModal('readiness');$('open-bones').onclick=()=>openModal('bones');$('close-modal').onclick=closeModal;
  $('open-engineering').onclick=()=>openModal('engineering');
  for(const container of [$('part-details'),$('modal-content')])container.addEventListener('click',event=>{
    const button=event.target.closest('button[data-related-bone]');
    if(button&&!button.disabled)inspectRelatedBone(button.dataset.relatedBone);
  });
  $('build-stamp').textContent='Engineering baseline '+data.build.engineering_revision.slice(0,8)+' · package '+data.build.package_input_sha256.slice(0,12);
  document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>openModal(b.dataset.tab));$('modal-backdrop').onclick=e=>{if(e.target===$('modal-backdrop'))closeModal();};
  document.addEventListener('keydown',e=>{if(!$('modal-backdrop').classList.contains('open'))return;if(e.key==='Escape')closeModal();if(e.key==='Tab'){const focusable=[...document.querySelector('.modal').querySelectorAll('button,a,input,select,[tabindex="0"]')].filter(x=>!x.disabled);const first=focusable[0],last=focusable.at(-1);if(e.shiftKey&&document.activeElement===first){e.preventDefault();last.focus();}else if(!e.shiftKey&&document.activeElement===last){e.preventDefault();first.focus();}}});
}
window.PHY_STUDIO={getState:()=>({...state,points:state.points.map(p=>p.toArray()),modelParts:model?.parts.length,webgl:!!renderer}),selectModel,setHeight,setPose,setPalette,inspectPart,
  referenceDraft:()=>referenceDraft?structuredClone(referenceDraft):null,
  originalReview:()=>originalReview,
  boneAudit:()=>model?.bone_equivalence??null,visiblePartIds:()=>meshes.filter(m=>m.visible).map(m=>m.name),
  modelBounds:()=>root?new THREE.Box3().setFromObject(root).getSize(new THREE.Vector3()).toArray():null,
  projectPart:id=>{const mesh=meshes.find(x=>x.name===id);if(!mesh)return null;const c=new THREE.Box3().setFromObject(mesh).getCenter(new THREE.Vector3()).project(camera);const r=canvas.getBoundingClientRect();return [r.left+(c.x+1)*r.width/2,r.top+(1-c.y)*r.height/2];}};
init();
