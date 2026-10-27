// Relationship inspection observes evidence; it never changes geometry or routing.
import { escape, htmlTable } from './evidence.js';

const groups = [
  ['Legacy anatomical declarations', ['legacy_parent','legacy_adjacency','legacy_articulation']],
  ['Source-backed qualitative articulations', ['qualitative_articulation']],
  ['Structural region membership', ['structural_membership']],
  ['Operational signal links · runtime behavior', ['operational_signal']]
];

export function relationshipsForBone(graph,uid) {
  return (graph?.relationships??[]).filter(row=>row.participants.some(p=>
    p.bone_id===uid||p.resolution.candidates?.includes(uid)||p.runtime_resolution?.candidates?.includes(uid)));
}

function resolutionText(result) {
  return result.status+(result.candidates?.length?' · candidates: '+result.candidates.join(', '):'');
}

export function relationshipsHtml(graph,uid,availableBoneIds=null) {
  if(!graph)return '<p>Relationship snapshot unavailable.</p>';
  const rows=relationshipsForBone(graph,uid),sources=new Map(graph.sources.map(s=>[s.source_id,s]));
  let html='<section class="relationships" data-relationship-bone="'+escape(uid)+'"><h3>Relationships</h3>'+
    '<p class="small-note">Declarations, qualitative anatomy, regional participation and runtime links have different meanings. None supplies a mechanical joint or a fabrication release.</p>';
  for(const [title,kinds] of groups){
    const subset=rows.filter(row=>kinds.includes(row.kind));
    html+='<div class="relationship-group"><h4>'+escape(title)+' <span class="tag">'+subset.length+'</span></h4>';
    if(!subset.length)html+='<p class="quiet">None declared in this snapshot.</p>';
    for(const row of subset){
      html+='<details class="relationship-card" data-relationship-id="'+escape(row.relationship_id)+'" data-relationship-kind="'+escape(row.kind)+'"><summary>'+escape(row.name??row.semantic_role.replaceAll('_',' '))+' · '+escape(row.resolution_status)+'</summary>';
      html+='<p class="small-note">'+escape(row.relationship_id)+'<br>Role: '+escape(row.semantic_role)+'</p><ul class="relationship-participants">';
      for(const p of row.participants){
        const navigable=p.bone_id!==null&&(availableBoneIds===null||availableBoneIds.has(p.bone_id));
        html+='<li><span>'+escape(p.role.replaceAll('_',' '))+': </span>'+
          (p.bone_id===null?'<span>unresolved target</span>':'<button type="button" class="related-bone" data-related-bone="'+escape(p.bone_id)+'"'+(navigable?'':' disabled title="No proxy for this identity in the current model"')+'>'+escape(p.bone_id)+'</button>')+
          '<p class="small-note">'+escape(p.matching_policy)+': '+escape(resolutionText(p.resolution))+
          (p.runtime_resolution?'<br>Runtime legacy matching: '+escape(resolutionText(p.runtime_resolution)):'')+'</p></li>';
      }
      html+='</ul>';
      if(row.kind==='qualitative_articulation')html+=htmlTable(['EVIDENCE','DECLARATION'],[
        ['Biological classification',row.biological_classification],['Qualitative motion',row.qualitative_motion.join(', ')],
        ['Non-bone participation',(row.non_bone_components??[]).join(', ')],['Engineering definition','unresolved (null)']
      ]);
      if(row.kind==='structural_membership')html+='<p class="small-note">Membership describes contributions to a region. It does not declare pairwise contact. Spatial definition: unresolved (null).</p>';
      if(row.kind==='operational_signal')html+='<p class="small-note">Observed runtime link only; no biological or mechanical articulation is inferred.</p>';
      if(row.kind==='legacy_articulation'&&row.original_declaration?.joint_type)html+='<p>Legacy joint label: <b>'+escape(row.original_declaration.joint_type)+'</b>. Engineering definition: unresolved (null).</p>';
      html+='<details class="relationship-original"><summary>Original declaration</summary><pre>'+escape(JSON.stringify(row.original_declaration,null,2))+'</pre></details>';
      const provenance=row.provenance;
      html+='<details class="relationship-evidence"><summary>Source and evidence</summary><p class="small-note">'+escape(provenance.source_path??'Supplied snapshot')+' · '+escape(provenance.field??provenance.catalog_locator??'')+
        (provenance.reference_locator?'<br>'+escape(provenance.reference_locator):'')+'<br>Value SHA-256: '+escape(provenance.value_sha256)+'<br>File SHA-256: '+escape(provenance.source_file_sha256??'not asserted')+'</p>';
      if(provenance.source_correspondence_only)html+='<p class="small-note">Source-file correspondence; the field value is fingerprinted independently.</p>';
      for(const id of Array.isArray(provenance.source_ids)?provenance.source_ids:[]){
        const source=sources.get(id);
        html+='<p class="small-note">'+escape(id)+(source?'<br>'+escape(source.title)+'<br>'+escape(source.review_scope):'')+'</p>';
        if(source?.url?.startsWith('https://'))html+='<a href="'+escape(source.url)+'" target="_blank" rel="noopener">Reference declaration ↗</a>';
      }
      if(provenance.references!==undefined)html+='<pre>'+escape(JSON.stringify(provenance.references,null,2))+'</pre>';
      html+='</details></details>';
    }
    html+='</div>';
  }
  const cycles=graph.diagnostics.parent_cycles.filter(cycle=>cycle.includes(uid));
  if(cycles.length)html+='<p class="warning">Declared parent cycle: '+escape(JSON.stringify(cycles))+'. Anatomical adjacency loops are checked separately.</p>';
  return html+'</section>';
}
