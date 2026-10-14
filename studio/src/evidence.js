// Evidence is an immutable review projection. Display scale and pose never
// transform anatomical measurements or NASA's printed reference endpoints.
export const escape = s => String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
export function htmlTable(headers,rows,className='') {
  return '<div class="table-scroll"><table class="'+escape(className)+'"><thead><tr>'+headers.map(x=>'<th>'+escape(x)+'</th>').join('')+'</tr></thead><tbody>'+rows.map(row=>'<tr>'+row.map(x=>'<td>'+escape(x)+'</td>').join('')+'</tr>').join('')+'</tbody></table></div>';
}
function link(uri,label) {
  return typeof uri==='string'&&uri.startsWith('https://')?'<a href="'+escape(uri)+'" target="_blank" rel="noopener">'+escape(label)+'</a>':escape(label);
}
const list = rows => '<ul>'+rows.map(x=>'<li>'+escape(x)+'</li>').join('')+'</ul>';
const text = value => value===null||value===undefined?'unknown':typeof value==='object'?JSON.stringify(value):String(value);
const number = value => value===null?'unknown':String(value);

export function boneEvidenceHtml(evidence,uid) {
  const bone=evidence.bones[uid];
  if(!bone)return '<p>Canonical engineering evidence unavailable for this component.</p>';
  const dims=bone.dimensions,values=dims.values_mm??{};
  const rows=Object.entries(values).sort(([a],[b])=>a.localeCompare(b)).map(([key,value])=>[
    key,number(value),bone.measurements[key]?'Source-backed tabular field':value===null?'Unresolved':'Declared value; evidence review open'
  ]);
  let html='<div class="bone-evidence"><h3>Canonical dimensions</h3><p>Declared map: <b>'+escape(dims.status)+'</b>. Declared values in mm; independent of this display’s height and pose.</p>'+htmlTable(['FIELD','mm','EVIDENCE'],rows,'dimension-table');
  html+='<p>Geometry: '+escape(bone.canonical_geometry?.status??'unresolved')+'. Record validation: '+escape(bone.record_validation.status)+'. Physical evidence unmeasured; fabrication release false.</p>';
  html+='<p class="small-note">Source record: '+escape(bone.source.path??'missing')+'<br>SHA-256: '+escape(bone.source.sha256??'unknown')+'</p>';
  for(const [key,trace] of Object.entries(bone.measurements)){
    const source=evidence.sources[trace.source_id],native=trace.native_measurement,mapping=trace.derivation;
    html+='<details class="measurement-evidence"><summary>'+escape(key)+' · measurement evidence</summary><p>'+link(source.locator,source.title)+'</p>'+htmlTable(['PROVENANCE','VALUE'],[
      ['Source revision',source.revision],['DOI',source.doi],['Native parameter',native.parameter],
      ['Native mean / mm',native.mean],['Published SD / mm',native.sd],['Study cohort',source.cohort],['Population subset',native.population_subset??'not separately recorded'],
      ['Native side',native.side],['Landmark definition',trace.landmark_definition],
      ['Method locator',trace.method_locator],['Derivation',mapping.method],['Dimensional meaning',mapping.meaning],
      ['Unit scale',mapping.unit_scale],['Canonical registration',text(mapping.canonical_registration)],
      ['Side application',mapping.side_application??'native side'],['Side assumption',mapping.side_assumption??'none added'],
      ['Evidence maturity',trace.evidence_maturity],['Validation scope',trace.validation_result],
      ['Native snapshot',source.native_snapshot.path],['Native snapshot SHA-256',source.native_snapshot.sha256],
      ['Measurement protocol SHA-256',evidence.inputs.measurement_protocols_sha256],
      ['Publisher license',source.publisher_license],['Rights review',source.rights_review]
    ])+'<p>'+escape(trace.uncertainty)+'</p><p>'+escape(native.limitation??'')+'</p>'+list(source.limitations)+'</details>';
  }
  const declarations=dims.metric_source_declarations??{};
  if(Object.keys(values).some(k=>values[k]!==null&&!bone.measurements[k])){
    html+='<details><summary>Other declared values / provenance</summary><p>Field-level measurement protocols are unavailable in this reviewed DIM-072 projection. Declaration completeness does not qualify dimensional fidelity.</p>'+htmlTable(['FIELD','DECLARED SOURCE'],Object.entries(declarations).map(([k,v])=>[k,text(v)]))+'<p>Reference declarations:</p>'+list(bone.reference_declarations.declarations)+'</details>';
  }
  if(bone.embodiment_fit)html+='<p>'+escape(bone.embodiment_fit)+'</p>';
  if(bone.candidate_notes.length)html+=list(bone.candidate_notes);
  html+='<details><summary>Remaining engineering evidence</summary>'+list(bone.evidence_needed)+'<p>Record blockers:</p>'+list(bone.record_validation.blocker_categories)+'</details></div>';
  return html;
}

export function engineeringHtml(evidence,build) {
  const expansion=evidence.dimension_expansion,canon=evidence.canonical_reference;
  return '<p><b>'+expansion.current_complete+'/'+expansion.inventory_count+' complete declared dimension maps</b>; '+expansion.improved_partial_ids.length+' improved partial maps and '+expansion.new_source_backed_field_count+' traced additions from PHY-DIM-072. '+expansion.remaining_to_target+' maps remain to the 72-map target.</p>'+
    '<p class="warning">Declared completeness counts supplied fields. Dimensional fidelity is unverified, anatomical geometry and articulation are unqualified, and physical evidence is unmeasured. Fabrication release remains false.</p>'+
    htmlTable(['ENGINEERING OBSERVATION','COUNT','MEANING'],Object.entries(evidence.metrics).map(([key,row])=>[key.replaceAll('_',' '),row.count+'/'+row.total,row.meaning]),'metric-table')+
    '<p>SOPHY canon '+escape(canon.version)+' · height '+canon.height_mm+' mm · span '+canon.span_mm+' mm. Source measurements have no adopted spatial registration.</p>'+
    '<label for="evidence-bone">Inspect a canonical identity</label><select id="evidence-bone" class="select">'+Object.values(evidence.bones).map(b=>'<option value="'+escape(b.bone_id)+'">'+escape(b.bone_id+' · '+b.name)+'</option>').join('')+'</select><div id="evidence-details"></div>'+
    '<details><summary>Recovery additions</summary><p>New complete declared maps:</p>'+list(expansion.new_complete_ids)+'<p>Improved partial maps:</p>'+list(expansion.improved_partial_ids)+'</details>'+
    '<details><summary>Package provenance</summary><p>Engineering baseline: '+link('https://github.com/SYSTEMS-OPERATOR/PHY/commit/'+evidence.engineering_revision,evidence.engineering_revision)+' · '+escape(evidence.engineering_revision_time)+'</p>'+htmlTable(['INPUT','SHA-256'],[
      ['Evidence inputs',evidence.input_sha256],['Package inputs',build.package_input_sha256],
      ['Canonical records',evidence.inputs.canonical_records_sha256],['NASA catalog',evidence.inputs.nasa_catalog_sha256],
      ['SOPHY canon',evidence.inputs.canon_sha256]
    ])+'<p>The revision names the reviewed engineering baseline. Content hashes identify the evidence and viewer shipped in this offline package.</p></details>';
}

export const conditionLabels={unsuited:'Unsuited',suited_unpressurized:'Suited / unpressurized',suited_pressurized:'Suited / pressurized'};
export function nasaMotionHtml(nasa,condition) {
  return '<div id="nasa-motion-rows">'+nasa.ranges.filter(r=>r.condition===condition).map(r=>
    '<details class="nasa-motion"><summary>'+escape(r.joint_label.replaceAll('_',' ')+' · '+r.motion_label.replaceAll('_',' '))+(r.source_issue?' · SOURCE ISSUE':'')+'</summary>'+
    htmlTable(['PRINTED ENDPOINT','deg'],Object.entries(r.endpoints_as_printed_deg))+
    '<p>'+escape(r.sign_convention)+'. '+escape(r.sex_scope)+'.</p><p>Source '+escape(r.source_id)+' · table '+r.table+' · PDF pages '+escape(r.pdf_pages.join(', '))+' · printed pages '+escape(r.printed_pages.join(', '))+'. Mechanical axis, mechanical limit and combined pose feasibility are unresolved.</p>'+
    (r.source_issue?'<p class="warning">'+escape(r.source_issue)+'</p>':'')+'</details>').join('')+'</div>';
}
export function nasaHtml(evidence) {
  const nasa=evidence.nasa,counts=evidence.nasa_counts;
  return '<p>Human positioning and range references · '+nasa.ranges.length+' motion pairs, '+counts.anthropometry+' external anthropometric comparisons, '+counts.source_issues+' source issues retained.</p>'+
    '<p class="warning">Human reference ranges require their stated conditions and measurement conventions. PHY mechanical limits, joint centers, coupled pose feasibility and flight compliance remain unqualified.</p>'+
    '<h3>Motion conditions</h3><label for="nasa-condition">Reference condition</label><select id="nasa-condition" class="select">'+Object.entries(conditionLabels).map(([key,label])=>'<option value="'+key+'">'+label+' · '+counts.conditions[key]+' pairs</option>').join('')+'</select><p id="nasa-condition-count" aria-live="polite"></p><div id="nasa-motion"></div>'+
    '<h3>Positioning and measurement conventions</h3>'+list(nasa.interpretation_rules)+
    '<p>Studio pose controls remain visual exploration around proposed centers. An external measurement endpoint requires an explicit anatomical definition before it can inform a mechanical datum.</p>'+
    '<h3>External anthropometric comparisons</h3>'+htmlTable(['SOURCE MEASUREMENT','PRINTED cm','CONVERTED mm','POPULATION / MAPPING'],nasa.anthropometry.map(r=>[r.source_label,r.values_cm.join(' to '),r.values_mm.join(' to '),r.population_scope+'; '+r.mapping_status]))+
    '<h3>Sources and applicability</h3>'+nasa.sources.map(s=>'<details><summary>'+escape(s.title)+'</summary><p>'+link(s.uri,'Official NASA source')+' · revision '+escape(s.revision)+(s.approved_on?' · approved '+escape(s.approved_on):'')+'</p><p>Reference scope: '+escape(s.supports.join(', '))+'.</p><p>Excluded qualification claims: '+escape(s.does_not_qualify.join(', '))+'.</p><p class="small-note">Recorded source hash: '+escape(s.sha256??'not recorded for the web page')+'. PDFs are not embedded or automatically acquired.</p></details>').join('');
}
