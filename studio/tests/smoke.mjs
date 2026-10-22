import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { chromium } from 'playwright';
import { createHash } from 'node:crypto';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const output=process.env.PHY_TEST_OUTPUT||path.join(os.tmpdir(),'phy-studio-qa');
await fs.mkdir(output,{recursive:true});
const browser=await chromium.launch({headless:true,executablePath:process.env.PHY_CHROMIUM_EXECUTABLE||undefined,
  args:['--no-sandbox','--enable-webgl','--ignore-gpu-blocklist','--use-angle=swiftshader']});
const errors=[];
try {
  const page=await browser.newPage({viewport:{width:1440,height:1000},deviceScaleFactor:1});
  page.on('pageerror',e=>errors.push(e.message));
  page.on('console',m=>{if(m.type()==='error')errors.push(m.text());});
  const assertStableViewport=async label=>{
    const samples=await page.evaluate(async()=>{
      await document.fonts.ready;
      const rows=[];
      for(let i=0;i<12;i++){
        await new Promise(requestAnimationFrame);
        const stage=document.querySelector('#stage').getBoundingClientRect();
        const canvas=document.querySelector('#viewport').getBoundingClientRect();
        rows.push({width:stage.width,height:stage.height,canvasWidth:canvas.width,canvasHeight:canvas.height,viewportHeight:innerHeight});
      }
      return rows;
    });
    for(const sample of samples){
      assert.ok(sample.width>0&&sample.height>0&&sample.height<sample.viewportHeight*2,label+': canvas stage stays bounded');
      assert.ok(Math.abs(sample.width-sample.canvasWidth)<1&&Math.abs(sample.height-sample.canvasHeight)<1,label+': canvas fits its stage');
    }
    assert.ok(Math.abs(samples.at(-1).height-samples[0].height)<1,label+': drawing-buffer resizing must not grow the layout');
  };
  const assertModelFramed=async label=>{
    const {points,rect}=await page.evaluate(async()=>{
      await new Promise(requestAnimationFrame);
      const ids=PHY_STUDIO.visiblePartIds();
      const r=document.querySelector('#viewport').getBoundingClientRect();
      return {points:ids.map(id=>PHY_STUDIO.projectPart(id)),rect:{x:r.x,y:r.y,width:r.width,height:r.height}};
    });
    assert.ok(points.every(([x,y])=>x>=rect.x&&x<=rect.x+rect.width&&y>=rect.y&&y<=rect.y+rect.height),label+': visible part centers stay in the viewport');
    const ys=points.map(p=>p[1]);
    assert.ok(Math.max(...ys)-Math.min(...ys)>rect.height*.15,label+': model remains large enough to inspect');
  };
  await page.goto(pathToFileURL(path.join(root,'studio/dist/PHY-Studio.html')).href);
  await page.waitForFunction(()=>window.PHY_STUDIO?.getState().webgl);
  for(const viewport of [{width:1800,height:1000},{width:1024,height:768},{width:1440,height:1000}]){
    await page.setViewportSize(viewport);
    await assertStableViewport('desktop '+viewport.width);
  }
  await assertModelFramed('initial armature');
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().model),'F28_REFINED');
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().modelParts),230);
  assert.deepEqual(await page.evaluate(()=>{const a=PHY_STUDIO.boneAudit();return [a.expected_bones,a.individual_bone_proxies,a.grouped_bones,a.unrepresented_bones];}),[206,206,0,0]);
  await page.click('#open-bones');
  assert.match(await page.locator('#modal-content').innerText(),/dimensions and morphology are unverified/i);
  assert.match(await page.locator('#modal-content').innerText(),/206 canonical source records; 0 records absent/);
  await page.keyboard.press('Escape');
  const networkRequests=[];
  page.on('request',request=>{if(/^https?:/.test(request.url()))networkRequests.push(request.url());});
  await page.click('#open-engineering');
  assert.match(await page.locator('#modal-content').innerText(),/40\/206 complete declared dimension maps/);
  assert.match(await page.locator('#modal-content').innerText(),/28 improved partial maps and 128 traced additions/);
  await page.selectOption('#evidence-bone','BONE_T6');
  assert.match(await page.locator('#evidence-details').innerText(),/Declared map: partial/);
  assert.ok(await page.locator('#evidence-details').getByText('width_mm',{exact:true}).isVisible());
  assert.equal(await page.locator('#evidence-details .measurement-evidence').count(),4);
  assert.equal(await page.evaluate(()=>PHY_DATA.engineering.bones.BONE_T6.dimensions.values_mm.width_mm),null);
  const method=page.locator('#evidence-details .measurement-evidence').first();
  await method.locator('summary').click();
  assert.match(await method.innerText(),/Landmark definition/);
  assert.match(await method.innerText(),/cohort variation/);
  assert.match(await method.innerText(),/published_mean_identity/);
  await page.screenshot({path:path.join(output,'engineering.png'),fullPage:true});
  await assertStableViewport('after full-page engineering capture');
  await page.click('[data-tab="nasa"]');
  assert.match(await page.locator('#modal-content').innerText(),/52 motion pairs, 11 external anthropometric comparisons, 17 source issues retained/);
  assert.equal(await page.locator('.nasa-motion').count(),20);
  const neck=page.locator('.nasa-motion').filter({hasText:'neck 1979 · flexion extension'});
  await neck.locator('summary').click();
  assert.match(await neck.innerText(),/Requires source clarification/);
  const poseBefore=await page.evaluate(()=>PHY_STUDIO.getState().pose);
  await page.selectOption('#nasa-condition','suited_unpressurized');
  assert.equal(await page.locator('.nasa-motion').count(),16);
  assert.match(await page.locator('#nasa-condition-count').innerText(),/unpressurized/);
  await page.selectOption('#nasa-condition','suited_pressurized');
  assert.equal(await page.locator('.nasa-motion').count(),16);
  assert.match(await page.locator('#nasa-condition-count').innerText(),/Suited \/ pressurized/);
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().pose),poseBefore);
  await page.screenshot({path:path.join(output,'nasa.png'),fullPage:true});
  await page.keyboard.press('Escape');
  await page.fill('#part-search','BONE_RADIUS_R');
  await page.selectOption('#part-select','radius_R');
  assert.match(await page.locator('#part-details').innerText(),/BONE_RADIUS_R/);
  await page.fill('#part-search','BONE_PHAL_1_2_R');
  await page.selectOption('#part-select','phal_1_2_R');
  assert.match(await page.locator('#part-details').innerText(),/BONE_PHAL_1_2_R/);
  await page.fill('#part-search','BONE_T_PHAL_1_2_R');
  await page.selectOption('#part-select','t_phal_1_2_R');
  assert.match(await page.locator('#part-details').innerText(),/BONE_T_PHAL_1_2_R/);
  await page.fill('#part-search','BONE_MAXILLA_R');
  await page.selectOption('#part-select','maxilla_R');
  assert.match(await page.locator('#part-details').innerText(),/canonical source record present/i);
  assert.match(await page.locator('#part-details').innerText(),/Declared map: unknown/);
  await page.fill('#part-search','BONE_MANDIBLE');
  await page.selectOption('#part-select','mandible');
  assert.match(await page.locator('#part-details').innerText(),/BONE_MANDIBLE/);
  for(const [boneId,partId] of [['BONE_MALLEUS_R','malleus_R'],['BONE_STAPES_L','stapes_L'],['BONE_HYOID','hyoid']]){
    await page.fill('#part-search',boneId);
    await page.selectOption('#part-select',partId);
    assert.match(await page.locator('#part-details').innerText(),new RegExp(boneId));
    assert.match(await page.locator('#part-details').innerText(),/canonical source record present/i);
  }
  await page.fill('#part-search','');
  await page.check('#layer-bones-only');
  const boneIds=await page.evaluate(()=>PHY_STUDIO.visiblePartIds());
  assert.equal(boneIds.length,206);
  assert.ok(boneIds.includes('patella_R')&&boneIds.includes('radius_L')&&boneIds.includes('rib_12_R'));
  assert.ok(boneIds.includes('pisiform_R')&&boneIds.includes('meta1_L')&&boneIds.includes('phal_5_3_L'));
  assert.ok(boneIds.includes('mandible')&&boneIds.includes('maxilla_R')&&boneIds.includes('par_L'));
  assert.ok(boneIds.includes('malleus_R')&&boneIds.includes('stapes_L')&&boneIds.includes('hyoid'));
  assert.ok(!boneIds.includes('head_arch_0'));
  assert.ok(boneIds.includes('calcaneus_R')&&boneIds.includes('mt1_L')&&boneIds.includes('t_phal_5_3_L'));
  assert.ok(!boneIds.includes('knee_joint_R')&&!boneIds.includes('foot_R')&&!boneIds.includes('shoulder_bridge'));
  await page.click('#show-all');
  assert.equal(await page.evaluate(()=>PHY_STUDIO.visiblePartIds().length),206,'show-all must not override bone-only filtering');
  await page.screenshot({path:path.join(output,'core-bones.png')});
  await page.uncheck('#layer-bones-only');
  await page.screenshot({path:path.join(output,'desktop.png'),fullPage:true});
  await page.click('#pose-t');await page.click('[data-view="front"]');
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().pose),0);
  await page.screenshot({path:path.join(output,'t-pose.png')});
  await page.click('[data-model="A0_R2"]');
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().modelParts),64);
  assert.ok(await page.locator('#layer-bones-only').isDisabled());
  assert.ok(await page.locator('#height-number').isDisabled());
  await page.screenshot({path:path.join(output,'a0.png')});
  await page.click('[data-model="REFERENCE_KIT"]');
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().modelParts),18);
  await page.click('[data-model="F28_REFINED"]');
  await page.fill('#height-number','1700');await page.locator('#height-number').press('Tab');
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().height),1700);
  await page.selectOption('#part-select','femur_R');await page.click('#isolate');
  assert.ok(await page.evaluate(()=>PHY_STUDIO.getState().isolated));await page.click('#show-all');
  await page.click('#mat-carbon');assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().palette),'carbon');
  // Surface picking must work independently of the inspector dropdown.
  await page.evaluate(()=>PHY_STUDIO.inspectPart(null));
  const position=await page.evaluate(()=>PHY_STUDIO.projectPart('femur_R'));
  await page.mouse.click(...position);
  assert.ok(await page.evaluate(()=>PHY_STUDIO.getState().selected));
  await page.click('#measure');
  for(const id of ['femur_R','tibia_R']){const p=await page.evaluate(id=>PHY_STUDIO.projectPart(id),id);await page.mouse.click(...p);}
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().points.length),2);
  await page.click('#measure');
  await page.click('#open-build');assert.ok(await page.locator('#modal-backdrop').evaluate(e=>e.classList.contains('open')));
  await page.click('[data-tab="readiness"]');assert.match(await page.locator('#modal-content').innerText(),/4 mm stop compression/);
  await page.keyboard.press('Escape');
  const getDownload=async(format,filename)=>{
    await page.selectOption('#export-format',format);
    const [download]=await Promise.all([page.waitForEvent('download'),page.click('#export')]);
    const file=path.join(output,filename);await download.saveAs(file);return fs.readFile(file);
  };
  const stl=await getDownload('stl','armature.stl'),count=stl.readUInt32LE(80);
  assert.equal(stl.length,84+count*50);assert.ok(count>10000);
  let maxZ=-Infinity;
  for(let i=0;i<count;i++)for(let v=0;v<3;v++)maxZ=Math.max(maxZ,stl.readFloatLE(84+i*50+12+v*12+8));
  assert.ok(Math.abs(maxZ-1700)<.01,'STL must preserve current height in canonical millimeter coordinates');
  const glb=await getDownload('glb','armature.glb');
  assert.equal(glb.readUInt32LE(0),0x46546c67);assert.equal(glb.readUInt32LE(4),2);assert.equal(glb.readUInt32LE(8),glb.length);
  const json=JSON.parse(glb.subarray(20,20+glb.readUInt32LE(12)).toString());assert.ok(json.meshes.length>80);
  const review=JSON.parse(await getDownload('json','review.json'));
  assert.equal(review.review_state.height,1700);assert.equal(review.review_state.fabrication_released,false);
  assert.equal(review.model.bone_equivalence.individual_bone_proxies,206);
  assert.equal(review.model.bone_equivalence.dimensional_fidelity_verified,false);
  assert.equal(review.engineering_evidence.dimension_expansion.current_complete,40);
  assert.equal(review.review_state.build.evidence_input_sha256,review.engineering_evidence.input_sha256);
  assert.equal(review.engineering_evidence.bones.BONE_T6.dimensions.values_mm.width_mm,null);
  assert.equal(review.engineering_evidence.bones.BONE_TALUS_R.dimensions.values_mm.length_mm,
    await page.evaluate(()=>PHY_DATA.engineering.bones.BONE_TALUS_R.dimensions.values_mm.length_mm));
  await page.check('#layer-bones-only');
  const boneGlb=await getDownload('glb','core-bones.glb');
  const boneJson=JSON.parse(boneGlb.subarray(20,20+boneGlb.readUInt32LE(12)).toString());
  assert.equal(boneJson.meshes.length,206);
  const extras=boneJson.nodes.filter(n=>n.mesh!==undefined).map(n=>n.extras);
  assert.equal(new Set(extras.map(x=>x.bone_id)).size,206);
  assert.ok(extras.every(x=>x.role==='bone_proxy'&&x.dimensional_fidelity==='unverified'));
  const handExtras=extras.filter(x=>/^(scaphoid|lunate|triquetrum|pisiform|trapezium|trapezoid|capitate|hamate|meta[1-5]|phal_)/.test(x.part_id));
  assert.equal(handExtras.length,54);
  assert.ok(handExtras.every(x=>x.physical_evidence==='unmeasured'&&x.hand_layout_sha256===review.model.hand_proxy_layout_sha256));
  const footIds=new Set(review.model.parts.filter(x=>x.region==='feet').map(x=>x.bone_id));
  const footExtras=extras.filter(x=>footIds.has(x.bone_id));
  assert.equal(footExtras.length,52);
  assert.ok(footExtras.every(x=>x.physical_evidence==='unmeasured'&&x.foot_layout_sha256===review.model.foot_proxy_layout_sha256));
  const skullExtras=extras.filter(x=>x.skull_layout_sha256);
  assert.equal(skullExtras.length,22);
  assert.ok(skullExtras.every(x=>x.skull_layout_sha256===review.model.skull_proxy_layout_sha256));
  assert.equal(skullExtras.filter(x=>x.source_record_status==='missing'&&x.source_record===null).length,0);
  assert.deepEqual(skullExtras.find(x=>x.bone_id==='BONE_MANDIBLE').articulates_with,['BONE_TEMP_R','BONE_TEMP_L']);
  assert.equal(skullExtras.find(x=>x.bone_id==='BONE_MANDIBLE').motion_implemented,false);
  const headSeven=extras.filter(x=>x.ear_hyoid_layout_sha256);
  assert.equal(headSeven.length,7);
  assert.ok(headSeven.every(x=>x.ear_hyoid_layout_sha256===review.model.ear_hyoid_proxy_layout_sha256));
  assert.ok(headSeven.every(x=>x.source_record_status==='present'&&x.source_record&&x.motion_implemented===false));
  assert.equal(headSeven.find(x=>x.bone_id==='BONE_STAPES_L').housing_bone_id,'BONE_TEMP_L');
  assert.equal(headSeven.find(x=>x.bone_id==='BONE_STAPES_L').non_bone_connection,'oval_window');
  assert.deepEqual(headSeven.find(x=>x.bone_id==='BONE_HYOID').articulates_with,[]);
  assert.equal(headSeven.find(x=>x.bone_id==='BONE_HYOID').non_bone_support,'muscle_and_ligament_suspension_unmodeled');
  assert.equal(footExtras.find(x=>x.bone_id==='BONE_MT1_L').topology_parent,'BONE_MEDIAL_CUNEIFORM_L');
  const boneReview=JSON.parse(await getDownload('json','core-review.json'));
  assert.equal(boneReview.review_state.boneOnly,true);
  assert.equal(boneReview.model.bone_equivalence.expected_bones,206,'JSON retains the entire source model and disposition audit');
  await page.uncheck('#layer-bones-only');
  const [maquette]=await Promise.all([page.waitForEvent('download'),page.click('#download-maquette')]);
  await maquette.saveAs(path.join(output,'maquette.zip'));
  const zip=await fs.readFile(path.join(output,'maquette.zip'));
  assert.equal(zip.readUInt32LE(0),0x04034b50);assert.equal(zip.readUInt32LE(zip.length-22),0x06054b50);
  assert.ok(zip.includes(Buffer.from('cut-sheet-1.svg')));assert.ok(zip.includes(Buffer.from('BUILD.md')));
  const hasNative=await page.evaluate(()=>!!PHY_DATA.native_references);
  if(hasNative){
    for(const [id,parts] of [['NIH_PELVIS',14],['NIH_SKIN',1]]){
      await page.evaluate(id=>PHY_STUDIO.selectModel(id),id);
      assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().modelParts),parts);
      assert.equal(await page.evaluate(()=>PHY_STUDIO.visiblePartIds().length),parts);
      assert.match(await page.locator('#axis-note').innerText(),/Y UP/);
      for(const selector of ['#height-number','#pose','#explode','#mat-carbon','#layer-bones-only'])assert.ok(await page.locator(selector).isDisabled());
      assert.ok(await page.locator('#export-format option[value="stl"]').isDisabled());
      const expected=await page.evaluate(()=>PHY_DATA.models.find(m=>m.id===PHY_STUDIO.getState().model).bounds_m);
      const actual=await page.evaluate(()=>PHY_STUDIO.modelBounds());
      assert.ok(actual.every((v,i)=>Math.abs(v-(expected.max[i]-expected.min[i])*1000)<1e-4),'native axes and units remain unchanged');
      if(id==='NIH_PELVIS')await assertModelFramed('native pelvis');
      await page.evaluate(()=>{PHY_STUDIO.setHeight(1800);PHY_STUDIO.setPose(0);PHY_STUDIO.setPalette('carbon');});
      assert.deepEqual(await page.evaluate(()=>PHY_STUDIO.modelBounds()),actual);
      const raw=await getDownload('glb',id+'.glb');
      const digest=createHash('sha256').update(raw).digest('hex');
      assert.equal(digest,await page.evaluate(()=>PHY_DATA.models.find(m=>m.id===PHY_STUDIO.getState().model).derivative.preview_sha256));
      await page.screenshot({path:path.join(output,id.toLowerCase()+'.png'),fullPage:true});
    }
    // Finish on a normal model even if earlier asynchronous parses complete late.
    await page.evaluate(()=>Promise.all([PHY_STUDIO.selectModel('NIH_SKIN'),PHY_STUDIO.selectModel('NIH_PELVIS'),PHY_STUDIO.selectModel('SOPHY_SCALE')]));
    assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().model),'SOPHY_SCALE');
    assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().modelParts),230);
    assert.equal(await page.locator('#layer-envelope').isChecked(),false);
    assert.equal(await page.evaluate(()=>PHY_STUDIO.visiblePartIds().length),228,'two form envelopes stay hidden by default');
    await page.check('#layer-envelope');
    assert.equal(await page.evaluate(()=>PHY_STUDIO.visiblePartIds().length),230);
    await page.uncheck('#layer-envelope');
    assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().height),1676.4);
    await assertModelFramed('armature after native views');
    await page.click('#compare-reference');
    await page.waitForFunction(()=>PHY_STUDIO.getState().model==='NIH_PELVIS'&&!PHY_STUDIO.getState().loading);
    await page.selectOption('#landmark-candidate','L_ASIS');await page.click('#isolate');
    const nativeBefore=await page.evaluate(()=>PHY_STUDIO.modelBounds());
    await page.click('#capture-landmark');
    const p=await page.evaluate(()=>PHY_STUDIO.projectPart('VH_F_ilium_compact_bone_L'));
    // This tests a draft point on the requested mesh, not anatomical ASIS location.
    for(const [dx,dy] of [[0,0],...[8,-8,20,-20,35,-35].flatMap(x=>[0,8,-8,20,-20].map(y=>[x,y]))]){
      if(await page.evaluate(()=>PHY_STUDIO.referenceDraft().annotations.length))break;
      await page.mouse.click(p[0]+dx,p[1]+dy);
    }
    assert.equal(await page.evaluate(()=>PHY_STUDIO.referenceDraft().annotations.length),1);
    const [draftDownload]=await Promise.all([page.waitForEvent('download'),page.click('#download-draft')]);
    await draftDownload.saveAs(path.join(output,'pelvis-preview-draft.json'));
    const draft=JSON.parse(await fs.readFile(path.join(output,'pelvis-preview-draft.json'),'utf8'));
    assert.equal(draft.accepted_landmarks,null);assert.equal(draft.canonical_transform,null);
    assert.equal(draft.annotations[0].original_face_index,null);assert.equal(draft.annotations[0].anatomically_accepted,false);
    assert.equal(draft.annotations[0].source_node,13);assert.equal(draft.annotations[0].mesh_name,'VH_F_ilium_compact_bone_L');
    assert.equal(draft.annotations[0].barycentric.length,3);assert.equal(draft.annotations[0].native_point_m.length,3);
    assert.deepEqual(await page.evaluate(()=>PHY_STUDIO.modelBounds()),nativeBefore);
    const nativeReview=JSON.parse(await getDownload('json','pelvis-native-review.json'));
    assert.equal(nativeReview.review_state.units,'m');assert.equal(nativeReview.landmark_draft.annotations.length,1);
    await page.click('#show-all');await page.click('#compare-reference');
    assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().model),'SOPHY_SCALE');
    await page.click('#compare-reference');await page.waitForFunction(()=>!PHY_STUDIO.getState().loading);
    assert.equal(await page.evaluate(()=>PHY_STUDIO.referenceDraft().annotations.length),1,'draft persists across frame switching');
    await page.click('#open-engineering');await page.click('[data-tab="nih"]');
    assert.ok((await page.locator('#modal-content').innerText()).includes(JSON.parse(await fs.readFile(path.join(root,'studio/reference_lock.json'),'utf8')).revision));
    assert.match(await page.locator('#modal-content').innerText(),/CC BY 4.0/);
    await page.keyboard.press('Escape');
  }
  // Both source provenance and all primary controls remain reachable on a phone.
  await page.setViewportSize({width:390,height:844});
  await assertStableViewport('390 px phone');
  await page.click('[data-model="F28_REFINED"]');await page.click('#mat-wood');
  await assertModelFramed('390 px armature after native views');
  const widths=await page.evaluate(()=>[document.documentElement.scrollWidth,innerWidth]);assert.equal(widths[0],widths[1]);
  await page.locator('#export').scrollIntoViewIfNeeded();assert.ok(await page.locator('#export').isVisible());
  await page.click('#open-engineering');
  await page.selectOption('#evidence-bone','BONE_TALUS_R');
  assert.match(await page.locator('#evidence-details').innerText(),/Declared map: complete/);
  const modalWidths=await page.locator('.modal').evaluate(e=>[e.scrollWidth,e.clientWidth]);
  assert.equal(modalWidths[0],modalWidths[1],'evidence tables stay inside the mobile dialog');
  await page.waitForFunction(()=>getComputedStyle(document.getElementById('toast')).opacity==='0');
  await page.locator('#evidence-details').scrollIntoViewIfNeeded();
  await page.screenshot({path:path.join(output,'mobile-engineering.png'),fullPage:false});
  await page.click('[data-tab="nasa"]');
  assert.ok(await page.locator('#nasa-condition').isVisible());
  if(hasNative){
    await page.click('[data-tab="nih"]');
    const nativeWidths=await page.locator('.modal').evaluate(e=>[e.scrollWidth,e.clientWidth]);
    assert.equal(nativeWidths[0],nativeWidths[1],'native provenance fits the phone dialog');
    await page.screenshot({path:path.join(output,'mobile-nih-evidence.png')});
  }
  await page.keyboard.press('Escape');
  await page.evaluate(()=>scrollTo(0,0));await page.waitForTimeout(400);
  await page.screenshot({path:path.join(output,'mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);
  assert.deepEqual(networkRequests,[],'evidence, NASA and native NIH views stay offline');
  await fs.writeFile(path.join(output,'results.json'),JSON.stringify({status:'passed',browserErrors:errors,stlTriangles:count,stlHeightMm:maxZ,glbMeshes:json.meshes.length,boneOnlyMeshes:boneJson.meshes.length,mobileWidth:widths[0]},null,2));
  console.log('PHY Studio browser smoke passed: offline WebGL, '+(hasNative?'seven':'five')+' models, stable canvas layout and camera framing, 206-bone filtering/identity/GLB audit including six ear ossicles and one hyoid, surface selection/ruler, height/pose/isolation, valid GLB/STL/JSON, embedded maquette ZIP, and 390px mobile layout.');
} finally {
  await browser.close();
}
