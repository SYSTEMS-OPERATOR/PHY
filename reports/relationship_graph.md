# PHY relationship inspection / 1.0.0

Adult records present: 206 / 206.
Missing records, regions and supplemental records do not increase adult coverage.
Declarations, qualitative anatomy, regional membership and runtime links are separate. No mechanical definition or fabrication release is granted.

| Kind | Declarations |
| --- | ---: |
| legacy_parent | 206 |
| legacy_adjacency | 11 |
| legacy_articulation | 11 |
| qualitative_articulation | 2 |
| structural_membership | 7 |
| operational_signal | 16 |

## Selected curated relationships

| ID | Role | Participants | Engineering definition |
| --- | --- | --- | --- |
| REL:catalog:articulations/tmj_l | qualitative_articulation | BONE_MANDIBLE, BONE_TEMP_L | null |
| REL:catalog:articulations/tmj_r | qualitative_articulation | BONE_MANDIBLE, BONE_TEMP_R | null |
| REL:catalog:regions/hard_palate | structural_membership | BONE_MAXILLA_R, BONE_MAXILLA_L, BONE_PALATINE_R, BONE_PALATINE_L | null |
| REL:catalog:regions/mandibular_unit | structural_membership | BONE_MANDIBLE | null |
| REL:catalog:regions/nasal_bridge | structural_membership | BONE_NASAL_R, BONE_NASAL_L | null |
| REL:catalog:regions/nasal_septum | structural_membership | BONE_ETHMOID, BONE_VOMER | null |
| REL:catalog:regions/orbit_l | structural_membership | BONE_FRONTAL, BONE_ZYGOMATIC_L, BONE_MAXILLA_L, BONE_PALATINE_L, BONE_ETHMOID, BONE_LACRIMAL_L, BONE_SPHENOID | null |
| REL:catalog:regions/orbit_r | structural_membership | BONE_FRONTAL, BONE_ZYGOMATIC_R, BONE_MAXILLA_R, BONE_PALATINE_R, BONE_ETHMOID, BONE_LACRIMAL_R, BONE_SPHENOID | null |
| REL:catalog:regions/zygomatic_arches | structural_membership | BONE_ZYGOMATIC_R, BONE_TEMP_R, BONE_ZYGOMATIC_L, BONE_TEMP_L | null |

## Unresolved declarations and compatibility projections

Exact export/render matching and case-insensitive legacy runtime matching remain distinct.

| Declaration | Projection | Reference | Status | Candidates |
| --- | --- | --- | --- | --- |
| REL:BONE_MANDIBLE:articulations[0] | declared | temporal | missing | [] |
| REL:BONE_MANDIBLE:articulations[0] | runtime_compatibility | temporal | ambiguous | ['BONE_TEMP_L', 'BONE_TEMP_R'] |
| REL:BONE_MANDIBLE:connections.children[0] | declared | temporal | missing | [] |
| REL:BONE_MANDIBLE:connections.parent | declared | temporal | missing | [] |
| REL:BONE_C1:connections.parent | declared | skull | missing | [] |
| REL:BONE_CALCANEUS_L:connections.parent | declared | tibia/fibula | missing | [] |
| REL:BONE_CALCANEUS_R:connections.parent | declared | tibia/fibula | missing | [] |
| REL:BONE_CAPITATE_L:connections.parent | declared | radius/ulna | missing | [] |
| REL:BONE_CAPITATE_R:connections.parent | declared | radius/ulna | missing | [] |
| REL:BONE_CLAVICLE_L:connections.parent | declared | sternum | missing | [] |
| REL:BONE_CLAVICLE_R:connections.parent | declared | sternum | missing | [] |
| REL:BONE_COCCYX:connections.parent | declared | sacrum | missing | [] |
| REL:BONE_CUBOID_L:connections.parent | declared | tibia/fibula | missing | [] |
| REL:BONE_CUBOID_R:connections.parent | declared | tibia/fibula | missing | [] |
| REL:BONE_ETHMOID:articulations[0] | declared | vomer | missing | [] |
| REL:BONE_ETHMOID:connections.children[0] | declared | vomer | missing | [] |
| REL:BONE_ETHMOID:connections.parent | declared | frontal | missing | [] |
| REL:BONE_FEMUR_L:connections.parent | declared | hip | missing | [] |
| REL:BONE_FEMUR_R:connections.parent | declared | hip | missing | [] |
| REL:BONE_FIBULA_L:connections.parent | declared | tibia | missing | [] |
| REL:BONE_FIBULA_R:connections.parent | declared | tibia | missing | [] |
| REL:BONE_FRONTAL:articulations[0] | declared | parietal | missing | [] |
| REL:BONE_FRONTAL:articulations[0] | runtime_compatibility | parietal | ambiguous | ['BONE_PAR_L', 'BONE_PAR_R'] |
| REL:BONE_FRONTAL:connections.children[0] | declared | parietal | missing | [] |
| REL:BONE_HAMATE_L:connections.parent | declared | radius/ulna | missing | [] |
| REL:BONE_HAMATE_R:connections.parent | declared | radius/ulna | missing | [] |
| REL:BONE_HIP_L:connections.parent | declared | sacrum | missing | [] |
| REL:BONE_HIP_R:connections.parent | declared | sacrum | missing | [] |
| REL:BONE_HUMERUS_L:connections.parent | declared | scapula | missing | [] |
| REL:BONE_HUMERUS_R:connections.parent | declared | scapula | missing | [] |

Showing 30 of 186 unresolved resolution diagnostics; all remain in JSON.

Parent cycles (declared parent projection only): `[]`.

Complete original values, source locators, content hashes and null definitions are retained in relationship_graph.json.
Source-file correspondence alone does not prove that a supplied snapshot was loaded from that file.
