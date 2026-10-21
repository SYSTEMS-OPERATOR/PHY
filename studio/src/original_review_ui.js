import * as THREE from 'three';
import { escape, htmlTable } from './evidence.js';
import { canonicalJson, strictJson, originalContext, emptyOriginal, validateOriginal,
  compareOriginal, originalPacket, validateOriginalPacket, originalAnnotation } from './original_review.js';

// Session state stays outside PHY_DATA. Original bytes are never bundled or packaged.
export function createOriginalReview({data,download,activate,selectModel,changed,focus,getView,restoreView,snapshot,prepareCapture,renderAvailable}){
  const panel=document.getElementById('original-review'),$=id=>document.getElementById('original-'+id);
  let ctx=null,draft=null,second=null,mode=null,active=false,importSequence=0;
  panel.innerHTML=`<h3>Original-surface review</h3>
    <p class="small-note">Choose the pinned <b>VH_F_Pelvis.glb</b> from your computer. It is checked locally. Reopen this file before resuming a saved review after closing Studio.</p>
    <label class="file-control">Load original pelvis<input id="original-file" type="file" accept=".glb"></label>
    <p id="original-status" class="small-note" role="status">Original not loaded. Preview drafts remain separate.</p>
    <div class="button-row"><button id="original-preview">Reduced preview</button><button id="original-mode" disabled>Verified original</button></div>
    <fieldset id="original-controls" disabled>
      <p class="warning">Proposed points only. Anatomical acceptance, metric calibration and an approved uncertainty budget remain open.</p>
      <label>Reviewer ID<input id="original-reviewer" maxlength="80" value="unassigned"></label>
      <label>Session ID<input id="original-session" maxlength="80" value="session-1"></label>
      <label>Declared evidence<select id="original-evidence"><option value="review_submission">Review submission</option><option value="software_fixture">Software fixture</option></select></label>
      <label>Candidate<select id="original-candidate"></select></label>
      <p id="original-definition" class="small-note"></p>
      <div class="button-row"><button id="original-focus">Focus surface</button><button id="original-point">Capture / replace point</button></div>
      <div class="button-row"><button id="original-patch">Toggle patch faces</button><button id="original-clear-patch">Clear patch</button><button id="original-delete">Delete candidate</button></div>
      <p id="original-selection" class="small-note"></p>
      <label>Point-selection rule<textarea id="original-rule" maxlength="2000" rows="2"></textarea></label>
      <label>Patch selection / representative-point rule<textarea id="original-patch-rule" maxlength="2000" rows="2"></textarea></label>
      <label>Construction / median-plane note<textarea id="original-construction" maxlength="2000" rows="2"></textarea></label>
      <label>Review notes<textarea id="original-notes" maxlength="8192" rows="2"></textarea></label>
      <p id="original-summary" class="small-note"></p>
      <div class="button-row"><button id="original-save">Save draft</button><button id="original-clear">Clear review</button></div>
      <label class="file-control">Resume draft or packet<input id="original-resume" type="file" accept=".json"></label>
      <label class="file-control">Load comparison draft B<input id="original-second" type="file" accept=".json"></label>
      <button id="original-remove-second">Remove comparison B</button>
      <div id="original-comparison" class="small-note"></div>
      <label>View label<input id="original-view-label" maxlength="80" value="Review view"></label>
      <div class="button-row"><button id="original-record-view">Record view</button><button id="original-image">Save annotated view</button></div>
      <label>Recorded context<select id="original-views"><option value="">No recorded views</option></select></label>
      <div class="button-row"><button id="original-restore-view">Restore view</button><button id="original-remove-view">Remove view</button></div>
      <button id="original-export" class="primary">Export review packet ↗</button>
      <p class="small-note">A = orange, B = blue. Both stay unaccepted. Packet JSON carries original triangle lineage, source/build fingerprints and unresolved gates. No source binary is included.</p>
    </fieldset>`;
  for(const f of data.native_references.landmark_packet.landmarks)$('candidate').add(new Option(f.label,f.id));
  const message=(s,error=false)=>{$('status').textContent=s;$('status').classList.toggle('warning',error);};
  const required=()=>{if(!ctx)throw new Error('Load the matching original first');};
  const annotation=()=>draft?.annotations.find(p=>p.landmark_id===$('candidate').value);
  const fieldValues=()=>({selection_rule:$('rule').value,patch_rule:$('patch-rule').value,construction_note:$('construction').value});
  function collect(){
    required();const next=structuredClone(draft);
    next.reviewer={id:$('reviewer').value,session_id:$('session').value,evidence_kind:$('evidence').value};next.review_notes=$('notes').value;
    const a=next.annotations.find(p=>p.landmark_id===$('candidate').value);if(a)Object.assign(a,fieldValues());
    validateOriginal(ctx,next);return next;
  }
  function stop(){mode=null;$('point').classList.remove('active');$('patch').classList.remove('active');}
  function refresh(){
    $('controls').disabled=!ctx;$('mode').disabled=!ctx;$('mode').classList.toggle('active',active);$('preview').classList.toggle('active',!active);
    const feature=data.native_references.landmark_packet.landmarks.find(f=>f.id===$('candidate').value);
    $('definition').textContent=feature.definition+' '+feature.review_rule;
    for(const id of ['focus','point','patch','record-view','image','restore-view'])$(id).disabled=!active||!renderAvailable;
    if(!draft)return;
    $('reviewer').value=draft.reviewer.id;$('session').value=draft.reviewer.session_id;$('evidence').value=draft.reviewer.evidence_kind;$('notes').value=draft.review_notes;
    const a=annotation();$('rule').value=a?.selection_rule??'';$('patch-rule').value=a?.patch_rule??'';$('construction').value=a?.construction_note??'';
    $('selection').textContent=a?'Original node '+a.source_node+' / primitive '+a.primitive_index+' / face '+a.face_index+' · '+a.patch_faces.length+' patch faces · ['+a.native_point_m.map(v=>v.toFixed(7)).join(', ')+'] m':'No point for this candidate. Capture a point before adding its patch.';
    const report=validateOriginal(ctx,draft);$('summary').textContent=report.annotations+'/5 proposed points. Missing: '+(report.missing_features.join(', ')||'none')+'. Unresolved rules: '+(report.unresolved_requirements.join('; ')||'none recorded')+'. Anatomical adoption remains pending.';
    const previous=$('views').value;$('views').replaceChildren(new Option('Choose recorded view',''));draft.views.forEach((v,i)=>$('views').add(new Option(v.label,String(i))));$('views').value=previous;
    if(second){const report=compareOriginal(ctx,draft,second);
      $('comparison').innerHTML='<p>A: '+escape(draft.reviewer.id)+' / '+escape(draft.reviewer.session_id)+'; B: '+escape(second.reviewer.id)+' / '+escape(second.reviewer.session_id)+'. '+escape(report.reviewer_relation)+'. '+(report.contains_software_fixtures?'Contains software fixtures. ':'')+escape(report.distance_meaning)+'</p>'+htmlTable(['FEATURE','SOURCE Δ / mm','RULES / PATCH'],report.rows.map(r=>[r.landmark_id,r.status==='paired'?r.distance_source_mm.toFixed(6):r.status,r.status==='paired'?(r.selection_rules_differ?'different rules':'same rules')+' / '+(r.patch_faces_differ?'different faces':'same faces'):'—']));
    }else $('comparison').textContent='No comparison draft B loaded. Declared identities do not establish reviewer independence.';
    changed();
  }
  function commit(next){validateOriginal(ctx,next);draft=next;refresh();}
  const run=fn=>async()=>{try{await fn();}catch(e){message('Review blocked: '+e.message+'. Current valid work is retained.',true);}};
  async function readJson(file){required();if(!file||file.size>ctx.contract.max_packet_bytes)throw new Error('Missing JSON or byte budget exceeded');message('Checking supplied review…');return strictJson(await file.text(),ctx.contract.max_packet_bytes);}
  async function loadOriginal(file){
    if(!file||file.size!==data.native_references.original_review_contract.source.bytes)throw new Error('Original missing, LFS pointer or size mismatch');
    message('Checking local original…');const sequence=++importSequence,next=await originalContext(await file.arrayBuffer(),data.native_references);if(sequence!==importSequence)return;
    if(draft)validateOriginal(next,draft);if(second)validateOriginal(next,second);
    ctx=next;draft??=emptyOriginal(ctx);stop();activate(ctx);await selectModel('NIH_PELVIS_ORIGINAL');refresh();
    message('Verified original · '+ctx.surface.parts.length+' source meshes · '+ctx.surface.parts.reduce((n,p)=>n+p.triangle_count,0)+' original triangles. No anatomical acceptance.');
  }
  function loadReview(value){
    required();let next,other=null;
    if(value?.kind===ctx.contract.packet_kind){const reviews=validateOriginalPacket(ctx,value);[next,other=null]=reviews;}
    else{validateOriginal(ctx,value);next=value;}
    // All supplied reviews and reports are checked before either active set changes.
    draft=structuredClone(next);second=structuredClone(other);stop();refresh();message('Restored original review at unchanged native coordinates.');
  }
  function loadSecond(value){required();validateOriginal(ctx,value);const current=collect();compareOriginal(ctx,current,value);draft=current;second=structuredClone(value);refresh();message('Comparison B loaded; reviewer independence remains unverified.');}
  const packet=()=>{const current=collect();const result=originalPacket(ctx,second?[current,second]:[current],data.build.package_input_sha256);draft=current;return result;};
  $('file').onchange=run(async()=>{const f=$('file').files[0];try{await loadOriginal(f);}finally{$('file').value='';}});
  $('preview').onclick=()=>selectModel('NIH_PELVIS');$('mode').onclick=()=>selectModel('NIH_PELVIS_ORIGINAL');
  $('candidate').onchange=()=>{stop();refresh();};
  $('focus').onclick=run(()=>{focus(ctx.contract.features[$('candidate').value].mesh_name);});
  for(const [id,value] of [['point','point'],['patch','patch']])$(id).onclick=run(()=>{commit(collect());if(value==='patch'&&!annotation())throw new Error('Capture this candidate point before selecting patch faces');prepareCapture();const next=mode===value?null:value;stop();mode=next;if(mode)$(id).classList.add('active');message(mode==='patch'?'Click original faces to add/remove them from this candidate patch.':mode==='point'?'Click the specified original cortical surface.':'Capture paused.');});
  $('delete').onclick=run(()=>{const next=collect();next.annotations=next.annotations.filter(a=>a.landmark_id!==$('candidate').value);stop();commit(next);});
  $('clear-patch').onclick=run(()=>{const next=collect(),a=next.annotations.find(a=>a.landmark_id===$('candidate').value);if(a)a.patch_faces=[];commit(next);});
  for(const id of ['reviewer','session','evidence'])$(id).onchange=run(()=>commit(collect()));
  for(const id of ['rule','patch-rule','construction'])$(id).oninput=run(()=>{if(annotation())commit(collect());});
  $('notes').oninput=run(()=>commit(collect()));
  $('save').onclick=run(()=>{commit(collect());download(canonicalJson(draft),'PHY-pelvis-original-draft.json','application/json');});
  $('clear').onclick=run(()=>{stop();commit(emptyOriginal(ctx));message('Review A cleared; source remains verified.');});
  $('resume').onchange=run(async()=>{try{loadReview(await readJson($('resume').files[0]));}finally{$('resume').value='';}});
  $('second').onchange=run(async()=>{try{loadSecond(await readJson($('second').files[0]));}finally{$('second').value='';}});
  $('remove-second').onclick=()=>{second=null;refresh();};
  $('export').onclick=run(()=>{download(canonicalJson(packet()),'PHY-pelvis-original-review-packet.json','application/json');refresh();});
  $('record-view').onclick=run(()=>{const next=collect();next.views.push({label:$('view-label').value,...getView()});commit(next);$('views').value=String(draft.views.length-1);});
  $('restore-view').onclick=run(()=>{commit(collect());if($('views').value==='')throw new Error('Choose a recorded view');restoreView(draft.views[Number($('views').value)]);});
  $('remove-view').onclick=run(()=>{const next=collect();if($('views').value==='')return;next.views.splice(Number($('views').value),1);commit(next);});
  $('image').onclick=run(()=>{commit(collect());snapshot();});
  refresh();
  return {
    currentView:getView,
    syncModel(model){active=model.id==='NIH_PELVIS_ORIGINAL';stop();refresh();},stop,
    draft:()=>draft?structuredClone(draft):null,second:()=>second?structuredClone(second):null,packet,
    loadReview,loadSecond,
    pick(hit){if(!active||!mode)return false;
      try{const next=collect(),ident=$('candidate').value,old=next.annotations.find(a=>a.landmark_id===ident),a=originalAnnotation(ctx,hit,ident,old);
        if(mode==='point'){Object.assign(a,fieldValues());next.annotations=next.annotations.filter(p=>p.landmark_id!==ident);next.annotations.push(a);stop();}
        else{const face={primitive_index:a.primitive_index,face_index:a.face_index,vertex_indices:a.vertex_indices},at=old.patch_faces.findIndex(f=>f.primitive_index===face.primitive_index&&f.face_index===face.face_index);if(at<0)old.patch_faces.push(face);else old.patch_faces.splice(at,1);}
        commit(next);message('Original lineage recorded. Proposed review only.');
      }catch(e){message(e.message,true);}return true;
    },
    overlays(group){if(!active||!ctx)return;
      for(const [review,color] of [[draft,0xb85a29],[second,0x166cbd]])for(const a of review?.annotations??[]){
        const dot=new THREE.Mesh(new THREE.SphereGeometry(1.6,12,8),new THREE.MeshBasicMaterial({color,depthTest:false}));dot.position.fromArray(a.native_point_m).multiplyScalar(1000);dot.renderOrder=2;group.add(dot);
        const points=[];for(const f of a.patch_faces){const t=ctx.surface.triangle(a.source_node,f.primitive_index,f.face_index,f.vertex_indices).points.map(v=>v.multiplyScalar(1000));for(let i=0;i<3;i++)points.push(t[i],t[(i+1)%3]);}
        if(points.length){const edges=new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color,depthTest:false}));edges.renderOrder=1;group.add(edges);}
      }
    }
  };
}
