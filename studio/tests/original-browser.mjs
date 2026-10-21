// Actual browser clicks on arbitrary surfaces: software fixtures, never anatomy.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { chromium } from 'playwright';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
if(!process.env.PHY_PELVIS_ORIGINAL){console.log('SKIP original browser fixture: explicit PHY_PELVIS_ORIGINAL not supplied; no original fetched.');process.exit(0);}
const source=path.resolve(root,process.env.PHY_PELVIS_ORIGINAL),output=path.join(os.tmpdir(),'phy-original-review-qa');
await fs.mkdir(output,{recursive:true});
const browser=await chromium.launch({headless:true,executablePath:process.env.PHY_CHROMIUM_EXECUTABLE||undefined,args:['--no-sandbox','--enable-webgl','--ignore-gpu-blocklist','--use-angle=swiftshader']});
const errors=[],network=[];let page,passed=false;
try{
  page=await browser.newPage({viewport:{width:1440,height:1000},deviceScaleFactor:1});
  page.on('pageerror',e=>errors.push(e.message));page.on('console',m=>{if(m.type()==='error')errors.push(m.text());});
  page.on('request',r=>{if(/^https?:/.test(r.url()))network.push(r.url());});
  const open=async()=>{await page.goto(pathToFileURL(path.join(root,'studio/dist/PHY-Studio.html')).href);await page.waitForFunction(()=>window.PHY_STUDIO?.getState().webgl);};
  const draft=()=>page.evaluate(()=>PHY_STUDIO.originalReview().draft());
  const uploadJson=(selector,value)=>page.setInputFiles(selector,{name:'review.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify(value))});
  const download=async(selector,name)=>{const event=page.waitForEvent('download');await page.click(selector);const item=await event;await item.saveAs(path.join(output,name));return fs.readFile(path.join(output,name));};
  const stable=async()=>{const samples=await page.evaluate(async()=>{const a=[];for(let i=0;i<8;i++){await new Promise(requestAnimationFrame);const s=document.getElementById('stage').getBoundingClientRect(),c=document.getElementById('viewport').getBoundingClientRect();a.push([s.width,s.height,c.width,c.height]);}return a;});for(const [w,h,cw,ch] of samples){assert.ok(w>0&&h>0&&h<2000);assert.ok(Math.abs(w-cw)<1&&Math.abs(h-ch)<1);}assert.ok(Math.abs(samples.at(-1)[1]-samples[0][1])<1);};
  async function clickSurface(id,changedFrom=null){
    await page.selectOption('#original-candidate',id);await page.click('#original-focus');await page.click('#original-point');await page.locator('#viewport').scrollIntoViewIfNeeded();
    const candidates=await page.evaluate(id=>{const f=PHY_DATA.native_references.original_review_contract.features[id],c=PHY_STUDIO.projectPart(f.mesh_name),r=document.getElementById('viewport').getBoundingClientRect();const points=[];for(const y of [0,-.06,.06,-.12,.12,-.2,.2])for(const x of [0,-.04,.04,-.08,.08,-.15,.15]){const p=[c[0]+x*r.width,c[1]+y*r.height];if(p[0]>r.x&&p[0]<r.right&&p[1]>r.y&&p[1]<r.bottom-100)points.push(p);}return points;},id);
    for(const [x,y] of candidates){
      if(changedFrom&&Math.hypot(x-changedFrom[0],y-changedFrom[1])<2)continue;
      await page.mouse.click(x,y);const a=(await draft()).annotations.find(a=>a.landmark_id===id);
      if(a&&(!changedFrom||a.face_index!==changedFrom[2]))return [x,y,a.face_index];
    }throw new Error('No original click found for '+id+': '+await page.locator('#original-status').innerText());
  }
  await open();assert.equal(await page.locator('#original-mode').isDisabled(),true);
  await page.setInputFiles('#original-file',{name:'VH_F_Pelvis.glb',mimeType:'model/gltf-binary',buffer:Buffer.from('version https://git-lfs.github.com/spec/v1\n')});
  await page.waitForFunction(()=>document.getElementById('original-status').textContent.includes('size mismatch'));
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().model),'F28_REFINED');
  await page.setInputFiles('#original-file',source);
  await page.waitForFunction(()=>PHY_STUDIO.getState().model==='NIH_PELVIS_ORIGINAL'&&!PHY_STUDIO.getState().loading);
  assert.match(await page.locator('#model-badge').innerText(),/VERIFIED ORIGINAL/);
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().modelParts),14);assert.equal(await page.locator('#height').isDisabled(),true);
  await page.fill('#original-reviewer','fixture-A');await page.fill('#original-session','browser-session-A');await page.selectOption('#original-evidence','software_fixture');
  const clicks={};
  for(const id of ['L_ASIS','L_PSIS','L_PUBIC_TUBERCLE','SACRAL_PROMONTORY','L_ISCHIAL_TUBEROSITY']){
    clicks[id]=await clickSurface(id);
    await page.fill('#original-rule','Software fixture: arbitrary visible original triangle; not an anatomical observation.');
    if(id==='SACRAL_PROMONTORY')await page.fill('#original-construction','Software fixture only. No anatomical median plane has been constructed.');
  }
  await page.fill('#original-patch-rule','Software fixture: explicit clicked faces, chosen to exercise lineage. No anatomical representative rule is approved.');
  await page.click('#original-patch');await page.locator('#viewport').scrollIntoViewIfNeeded();
  const [px,py]=clicks.L_ISCHIAL_TUBEROSITY;await page.mouse.click(px,py);
  assert.equal((await draft()).annotations.find(a=>a.landmark_id==='L_ISCHIAL_TUBEROSITY').patch_faces.length,1);
  let secondClick=null;
  for(const [dx,dy] of [[3,0],[-3,0],[0,3],[0,-3],[8,0],[-8,0],[0,8],[0,-8]]){
    await page.mouse.click(px+dx,py+dy);const a=(await draft()).annotations.find(a=>a.landmark_id==='L_ISCHIAL_TUBEROSITY');
    if(a.patch_faces.length>=2){secondClick=[px+dx,py+dy];break;}
    if(!a.patch_faces.length)await page.mouse.click(px,py);
  }
  assert.ok(secondClick,'Two actual original faces selected');await page.mouse.click(...secondClick);
  assert.equal((await draft()).annotations.find(a=>a.landmark_id==='L_ISCHIAL_TUBEROSITY').patch_faces.length,1);await page.mouse.click(...secondClick);
  await page.click('#original-patch');
  await page.fill('#original-notes','SOFTWARE FIXTURE ONLY. Arbitrary clicked points exercise review software; none identifies an accepted anatomical landmark.');
  await page.fill('#original-view-label','Fixture ischium inspection');await page.click('#original-record-view');
  await page.selectOption('#original-candidate','L_ASIS');await page.click('#original-delete');assert.equal((await draft()).annotations.length,4);clicks.L_ASIS=await clickSurface('L_ASIS');
  await page.fill('#original-rule','Software fixture: recaptured arbitrary triangle, not an anatomical observation.');
  const a=JSON.parse(await download('#original-save','review-A.json'));assert.equal(a.annotations.length,5);assert.equal(a.reviewer.evidence_kind,'software_fixture');assert.equal(a.views.length,1);
  for(const mutate of [d=>d.source_sha256='0'.repeat(64),d=>d.reference_revision='0'.repeat(40),d=>d.kind='PHY_PELVIS_PREVIEW_DRAFT_V1',d=>d.fabrication_released=true,d=>d.annotations[0].anatomically_accepted=true,d=>d.annotations[0].patch_faces=[{primitive_index:0,face_index:-1,vertex_indices:[0,1,2]}]]){
    const bad=structuredClone(a);mutate(bad);await uploadJson('#original-resume',bad);await page.waitForFunction(()=>document.getElementById('original-status').textContent.includes('Review blocked:'));assert.deepEqual(await draft(),a);
  }
  const originalRaw=await fs.readFile(source),corrupt=Buffer.from(originalRaw);corrupt[corrupt.length-1]^=1;
  await page.setInputFiles('#original-file',{name:'altered.glb',mimeType:'model/gltf-binary',buffer:corrupt});await page.waitForFunction(()=>document.getElementById('original-status').textContent.includes('checksum mismatch'));assert.deepEqual(await draft(),a);
  await page.click('#original-clear');assert.equal((await draft()).annotations.length,0);await page.setInputFiles('#original-resume',path.join(output,'review-A.json'));await page.waitForFunction(()=>PHY_STUDIO.originalReview().draft().annotations.length===5);assert.deepEqual(await draft(),a);
  const bClick=await clickSurface('L_ASIS',clicks.L_ASIS);assert.notEqual(bClick[2],clicks.L_ASIS[2]);
  await page.fill('#original-reviewer','fixture-B');await page.fill('#original-session','browser-session-B');
  const b=JSON.parse(await download('#original-save','review-B.json'));assert.notDeepEqual(b.annotations.find(a=>a.landmark_id==='L_ASIS').native_point_m,a.annotations.find(a=>a.landmark_id==='L_ASIS').native_point_m);
  await page.setInputFiles('#original-resume',path.join(output,'review-A.json'));await page.waitForFunction(()=>PHY_STUDIO.originalReview().draft().reviewer.id==='fixture-A');
  await page.setInputFiles('#original-second',path.join(output,'review-B.json'));await page.waitForFunction(()=>PHY_STUDIO.originalReview().second()?.reviewer.id==='fixture-B');
  assert.match(await page.locator('#original-comparison').innerText(),/independence_unverified/);assert.match(await page.locator('#original-comparison').innerText(),/metric calibration unresolved/);
  const packetBytes=await download('#original-export','packet.json'),packet=JSON.parse(packetBytes);
  assert.deepEqual(packet.reviews,[a,b]);assert.ok(packet.comparison.rows.find(r=>r.landmark_id==='L_ASIS').distance_source_mm>0);
  assert.deepEqual(await download('#original-export','packet-repeat.json'),packetBytes);
  for(const id of ['NIH_PELVIS','SOPHY_SCALE','NIH_SKIN','NIH_PELVIS_ORIGINAL','NIH_PELVIS','NIH_PELVIS_ORIGINAL']){
    await page.evaluate(id=>PHY_STUDIO.selectModel(id),id);await page.waitForFunction(()=>!PHY_STUDIO.getState().loading);await stable();assert.deepEqual(await draft(),a);
  }
  await page.selectOption('#original-views','0');await page.click('#original-restore-view');
  const restored=await page.evaluate(()=>PHY_STUDIO.originalReview().currentView());
  for(const key of ['camera_position_mm','target_mm'])for(let i=0;i<3;i++)assert.ok(Math.abs(restored[key][i]-a.views[0][key][i])<1e-6);
  await page.click('#show-all');await page.click('[data-view="iso"]');
  await page.locator('#original-review').evaluate(e=>{const aside=e.closest('aside');aside.scrollTop=e.offsetTop-aside.offsetTop-20;});
  await page.screenshot({path:path.join(output,'original-desktop.png'),fullPage:true});await stable();
  await page.locator('#original-file').scrollIntoViewIfNeeded();await page.screenshot({path:path.join(output,'original-controls-desktop.png')});
  await page.locator('#original-comparison').scrollIntoViewIfNeeded();await page.screenshot({path:path.join(output,'original-comparison.png')});
  const pngEvent=page.waitForEvent('download',{predicate:d=>d.suggestedFilename().endsWith('.png')}),contextEvent=page.waitForEvent('download',{predicate:d=>d.suggestedFilename().endsWith('-context.json')});
  await page.click('#original-image');const [png,context]=await Promise.all([pngEvent,contextEvent]);await png.saveAs(path.join(output,'annotated.png'));await context.saveAs(path.join(output,'annotated-context.json'));
  assert.deepEqual((await fs.readFile(path.join(output,'annotated.png'))).subarray(0,8),Buffer.from([137,80,78,71,13,10,26,10]));
  assert.equal(JSON.parse(await fs.readFile(path.join(output,'annotated-context.json'))).viewer_build_sha256,packet.viewer_build_sha256);
  await page.selectOption('#export-format','glb');
  assert.deepEqual(await download('#export','original-copy.glb'),originalRaw);
  await page.setViewportSize({width:390,height:844});await stable();await page.locator('#original-export').scrollIntoViewIfNeeded();
  const overflow=await page.evaluate(()=>({width:document.documentElement.scrollWidth,viewport:innerWidth,bad:[...document.querySelectorAll('#original-review input,#original-review select,#original-review textarea,#original-review button')].filter(e=>{const r=e.getBoundingClientRect();return r.left<0||r.right>innerWidth+1;}).map(e=>e.id)}));assert.ok(overflow.width<=overflow.viewport+1,JSON.stringify(overflow));assert.deepEqual(overflow.bad,[]);
  await page.screenshot({path:path.join(output,'original-mobile.png'),fullPage:true});await stable();
  await page.setInputFiles('#original-resume',path.join(output,'packet.json'));await page.waitForFunction(()=>PHY_STUDIO.originalReview().second()?.reviewer.id==='fixture-B');assert.deepEqual(await draft(),a);
  // A fresh page has neither original bytes nor reviews; explicitly supply both again.
  await open();assert.equal(await page.locator('#original-mode').isDisabled(),true);await page.setInputFiles('#original-file',source);await page.waitForFunction(()=>PHY_STUDIO.getState().model==='NIH_PELVIS_ORIGINAL'&&!PHY_STUDIO.getState().loading);
  await page.setInputFiles('#original-resume',path.join(output,'packet.json'));await page.waitForFunction(()=>PHY_STUDIO.originalReview().draft().annotations.length===5);assert.deepEqual(await draft(),a);assert.deepEqual(await page.evaluate(()=>PHY_STUDIO.originalReview().second()),b);
  assert.deepEqual(await download('#original-export','packet-reopened.json'),packetBytes);
  assert.deepEqual(network,[],'Review remains offline');assert.deepEqual(errors,[],'No browser errors');
  await fs.writeFile(path.join(output,'environment.json'),JSON.stringify({browser:await browser.version(),executable_override:!!process.env.PHY_CHROMIUM_EXECUTABLE,fixture_only:true,network_requests:network,browser_errors:errors},null,2)+'\n');
  passed=true;console.log('Original browser passed: five real original-surface clicks, patch editing, strict reload rollback, A/B comparison, fresh-page packet reopen, deterministic JSON, exact GLB export, mobile controls and zero HTTP requests.');
}finally{if(!passed&&page)await page.screenshot({path:path.join(output,'failure.png'),fullPage:true}).catch(()=>{});await browser.close();}
