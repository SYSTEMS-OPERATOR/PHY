import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { chromium } from 'playwright';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const output=process.env.PHY_REL_TEST_OUTPUT||path.join(os.tmpdir(),'phy-relationships-qa');
await fs.mkdir(output,{recursive:true});
const graphBytes=await fs.readFile(path.join(root,'studio/dist/relationship_graph.json'));
const graph=JSON.parse(graphBytes),manifest=JSON.parse(await fs.readFile(path.join(root,'studio/dist/manifest.json'),'utf8'));
const digest=createHash('sha256').update(graphBytes).digest('hex');
assert.equal(manifest.files['relationship_graph.json'],digest);
assert.equal(manifest.relationship_graph_sha256,digest);
const browser=await chromium.launch({headless:true,executablePath:process.env.PHY_CHROMIUM_EXECUTABLE||undefined,
  args:['--no-sandbox','--enable-webgl','--ignore-gpu-blocklist','--use-angle=swiftshader']});
const requests=[],errors=[];
try {
  const page=await browser.newPage({viewport:{width:1440,height:1000},deviceScaleFactor:1});
  page.on('request',r=>{if(/^https?:/.test(r.url()))requests.push(r.url());});
  page.on('pageerror',e=>errors.push(e.message));
  await page.goto(pathToFileURL(path.join(root,'studio/dist/PHY-Studio.html')).href);
  await page.waitForFunction(()=>window.PHY_STUDIO?.getState().webgl);
  assert.deepEqual(await page.evaluate(()=>PHY_DATA.engineering.relationships),graph);
  const geometryHash=()=>page.evaluate(async()=>{
    const bytes=new TextEncoder().encode(JSON.stringify(PHY_DATA.models));
    return Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),n=>n.toString(16).padStart(2,'0')).join('');
  });
  const geometryBefore=await geometryHash();
  const stateBefore=await page.evaluate(()=>{const s=PHY_STUDIO.getState();return [s.model,s.height,s.pose,s.exploded];});
  await page.click('#open-engineering');
  await page.selectOption('#evidence-bone','BONE_MANDIBLE');
  const details=page.locator('#evidence-details');
  const joints=details.locator('[data-relationship-kind="qualitative_articulation"]');
  assert.equal(await joints.count(),2);
  for(let i=0;i<2;i++){
    const joint=joints.nth(i);await joint.locator('summary').first().click();
    assert.match(await joint.innerText(),/rotation, translation/);
    assert.match(await joint.innerText(),/Engineering definition.*unresolved \(null\)/s);
    assert.equal(await joint.locator('[data-related-bone="BONE_MANDIBLE"]').count(),1);
  }
  const legacy=details.locator('[data-relationship-kind="legacy_articulation"]');
  await legacy.locator('summary').first().click();
  assert.match(await legacy.innerText(),/Runtime legacy matching: ambiguous/);
  assert.match(await legacy.innerText(),/BONE_TEMP_L, BONE_TEMP_R/);
  assert.match(await legacy.innerText(),/Legacy joint label: hinge/);
  await page.screenshot({path:path.join(output,'relationships-desktop.png'),fullPage:true});
  await details.locator('[data-related-bone="BONE_TEMP_L"]').first().click();
  assert.equal(await page.inputValue('#evidence-bone'),'BONE_TEMP_L');
  assert.equal(await page.evaluate(()=>PHY_STUDIO.getState().selected),await page.evaluate(()=>
    PHY_DATA.models.find(m=>m.id===PHY_STUDIO.getState().model).parts.find(p=>p.bone_id==='BONE_TEMP_L').id));
  assert.match(await page.locator('#part-details').innerText(),/BONE_TEMP_L/);
  await page.selectOption('#evidence-bone','BONE_FRONTAL');
  assert.ok(await details.locator('[data-relationship-kind="structural_membership"]').count()>0);
  assert.ok(await details.locator('[data-relationship-kind="operational_signal"]').count()>0);
  const membership=details.locator('[data-relationship-kind="structural_membership"]').first();
  await membership.locator('summary').first().click();
  assert.match(await membership.innerText(),/does not declare pairwise contact/);
  await page.selectOption('#evidence-bone','BONE_HYOID');
  assert.equal(await details.locator('[data-relationship-kind="qualitative_articulation"]').count(),0);
  assert.equal(await details.locator('[data-relationship-kind="operational_signal"]').count(),0);
  assert.match(await details.innerText(),/unspecified/);
  await page.selectOption('#evidence-bone','BONE_MANDIBLE');
  await page.setViewportSize({width:390,height:844});
  await details.locator('[data-relationship-kind="qualitative_articulation"]').first().locator('summary').first().click();
  await details.scrollIntoViewIfNeeded();
  assert.ok(await page.evaluate(()=>document.querySelector('.modal').scrollWidth<=document.querySelector('.modal').clientWidth+1));
  await page.screenshot({path:path.join(output,'relationships-mobile.png')});
  assert.equal(await geometryHash(),geometryBefore,'navigation cannot change any model geometry or limits');
  assert.deepEqual(await page.evaluate(()=>{const s=PHY_STUDIO.getState();return [s.model,s.height,s.pose,s.exploded];}),stateBefore);
  await page.keyboard.press('Escape');
  await page.setViewportSize({width:1440,height:1000});
  await page.selectOption('#export-format','json');
  const downloadPromise=page.waitForEvent('download');
  await page.click('#export');
  const download=await downloadPromise;
  const downloaded=path.join(output,'relationship-review.json');await download.saveAs(downloaded);
  const review=JSON.parse(await fs.readFile(downloaded,'utf8'));
  assert.deepEqual(review.engineering_evidence.relationships,graph);
  assert.equal(review.engineering_evidence.inputs.relationship_graph_sha256,digest);
  assert.equal(review.review_state.fabrication_released,false);

  // Exercise malformed Python declarations through evidence, packaging, UI and export.
  const fixtureRoot=await fs.mkdtemp(path.join(output,'malformed-'));
  execFileSync(process.env.PHY_PYTHON_EXECUTABLE||'python3',
    [path.join(root,'studio/tests/malformed-relationships.py'),fixtureRoot],{cwd:root,timeout:60000});
  const fixtureBytes=await fs.readFile(path.join(fixtureRoot,'relationship_graph.json'));
  const fixtureGraph=JSON.parse(fixtureBytes);
  const fixtureDigest=createHash('sha256').update(fixtureBytes).digest('hex');
  const fixtureManifest=JSON.parse(await fs.readFile(path.join(fixtureRoot,'manifest.json'),'utf8'));
  assert.equal(fixtureManifest.files['relationship_graph.json'],fixtureDigest);
  assert.equal(fixtureManifest.relationship_graph_sha256,fixtureDigest);
  for(const name of ['f28_mean.json','f28_refined.json','sophy_scale.json'])
    assert.equal(fixtureManifest.files[name],manifest.files[name],'malformed declarations cannot change body geometry');
  await page.goto(pathToFileURL(path.join(fixtureRoot,'PHY-Studio.html')).href);
  await page.waitForFunction(()=>window.PHY_STUDIO?.getState().webgl);
  assert.deepEqual(await page.evaluate(()=>PHY_DATA.engineering.relationships),fixtureGraph);
  const fixtureGeometryBefore=await geometryHash();
  await page.click('#open-engineering');
  for(const [uid,rid,original] of [
    ['BONE_HYOID','REL:BONE_HYOID:articulations[0]',null],
    ['BONE_INCUS_L','REL:BONE_INCUS_L:articulations',{invalid_type:'builtins.tuple'}]
  ]){
    const row=fixtureGraph.relationships.find(r=>r.relationship_id===rid);
    assert.equal(row.resolution_status,'invalid');
    assert.equal(row.participants[1].bone_id,null);
    assert.deepEqual(row.original_declaration,original);
    await page.selectOption('#evidence-bone',uid);
    const card=details.locator('[data-relationship-id="'+rid+'"]');
    assert.match(await card.locator('summary').first().innerText(),/invalid/);
    await card.locator('summary').first().click();
    assert.match(await card.innerText(),/unresolved target/);
    assert.equal(await card.locator('[data-related-bone]').count(),1,'invalid target has no navigation button');
    await card.locator('.relationship-original summary').click();
    assert.deepEqual(JSON.parse(await card.locator('.relationship-original pre').innerText()),original);
  }
  const valid=fixtureGraph.relationships.find(r=>r.relationship_id==='REL:BONE_HYOID:articulations[1]');
  assert.equal(valid.participants[1].bone_id,'BONE_MALLEUS_R');
  assert.equal(fixtureGraph.relationships.filter(r=>r.kind==='operational_signal'&&
    r.participants[0].bone_id==='BONE_INCUS_L').length,0,'tuple contents must never become runtime links');
  await page.screenshot({path:path.join(output,'malformed-relationships.png'),fullPage:true});
  assert.equal(await geometryHash(),fixtureGeometryBefore);
  assert.deepEqual(await page.evaluate(()=>PHY_DATA.engineering.relationships),fixtureGraph,'inspection is read-only');
  await page.keyboard.press('Escape');
  await page.selectOption('#export-format','json');
  const fixtureDownloadPromise=page.waitForEvent('download');
  await page.click('#export');
  const fixtureDownload=await fixtureDownloadPromise;
  const fixtureReviewPath=path.join(output,'malformed-relationship-review.json');
  await fixtureDownload.saveAs(fixtureReviewPath);
  const fixtureReview=JSON.parse(await fs.readFile(fixtureReviewPath,'utf8'));
  assert.deepEqual(fixtureReview.engineering_evidence.relationships,fixtureGraph);
  assert.equal(fixtureReview.engineering_evidence.inputs.relationship_graph_sha256,fixtureDigest);
  assert.equal(fixtureReview.review_state.fabrication_released,false);
  assert.deepEqual(errors,[]);assert.deepEqual(requests,[],'relationship inspection must stay offline');
  await fs.writeFile(path.join(output,'verification.json'),JSON.stringify({passed:true,relationship_sha256:digest,
    malformed_relationship_sha256:fixtureDigest,malformed_shapes:2,adult_identities:206,tmjs:2,regions:7,
    geometry_unchanged:true,network_requests:requests,browser_errors:errors},null,2)+'\n');
  console.log('Offline relationship inspection, malformed declarations, unchanged geometry and review export pass.');
}finally{await browser.close();}
