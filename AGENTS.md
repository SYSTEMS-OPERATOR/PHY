# AGENTS.md

**SYSTEMS-OPERATOR/PHY: Digital Twin Armature Agent Specification**
*Last Revision: 2025-06-22*

---

## ::AGENT HIERARCHY::

### 1. **BONE_AGENT**

**Purpose:** Represents a single bone as a modular, autonomous data/process unit.
**Location:** `/bones/` (`/axial/`, `/appendicular/`, etc.)
**Count:** ~206 (full adult human, canonical)

#### **Responsibilities:**

* Stores all metrics & metadata for its bone (dimensions, geometry, connections, material properties, references).
* Exposes interface for geometry queries, physics recalculation, reference lookup.
* Supports on-the-fly update of physical material parameters (density, modulus, etc).
* Publishes spatial node/grid mapping for RoS/TF compatible assembly.
* Self-contained: runs independently or as part of aggregate skeleton.

#### **Schema:**

```python
class BoneAgent:
    def __init__(self, ...):
        self.name = ...
        self.latin_name = ...
        self.type = ...
        self.region = ...
        self.dimensions = {...}
        self.geometry = {...}      # low-poly, grid/node based, code-native
        self.material = {...}      # density, E, strength, editable
        self.physics = {...}       # mass, inertia, calc. by material
        self.connections = {...}   # joint points, type, parent/child
        self.references = [...]    # TA, diagrams, DOIs, source-urls
    def set_material(self, material_props): ...
    def export_geometry(self): ...
    def get_reference(self): ...
    def as_ros_node(self): ...
```

---

### 2. **ASSEMBLER_AGENT**

**Purpose:** Dynamically discovers, imports, and spatially assembles all `BoneAgent`s into a coherent digital skeleton.
**Location:** `/assemble_skeleton.py` (or `system/`)

#### **Responsibilities:**

* Scans `/bones/` for all valid agents, verifies integrity (all expected bones present, no orphans).
* Constructs spatial/joint hierarchy per canonical human form (TA/Gray’s Anatomy standard).
* Handles batch updates (e.g., set all material properties skeleton-wide).
* Broadcasts skeleton structure as ROS/TF tree or URDF-style output.
* Enables step-through inspection, comparison, or dynamic simulation.

#### **Schema:**

```python
class AssemblerAgent:
    def __init__(self):
        self.bones = {}
        self.hierarchy = {}
    def discover_bones(self, path): ...
    def assemble(self): ...
    def set_global_material(self, material_props): ...
    def export_ros_tf(self): ...
    def validate(self): ...
```

---

### 3. **VISUALIZER_AGENT**

**Purpose:** Renders the digital skeleton using grid-native data.
**Location:** `/visualization/` or `tools/`
**(optional, but recommended)**

#### **Responsibilities:**

* Parses bone geometry/coordinate sets.
* Generates ASCII/point-cloud/URDF/TF visualizations (no binaries).
* Supports selectable overlays (dimensions, density, stress points).
* Interfaced for use with ROS RViz, TF Tree Viewer, or CLI preview.

#### **Schema:**

```python
class VisualizerAgent:
    def __init__(self, assembler):
        self.skeleton = assembler
    def render_ascii(self): ...
    def export_urdf(self): ...
    def show_tf_tree(self): ...
```

---

### 4. **REFERENCE_AGENT**

**Purpose:** Maintains, audits, and serves all supporting references (TA, anatomy texts, journal DOIs, etc)
**Location:** `/references/`, `reference_manager.py`

#### **Responsibilities:**

* Compiles/updates canonical source links for each bone.
* Validates that every `BoneAgent` has at least one high-quality reference.
* Exports reference sets (Markdown, BibTeX, CSV).
* Supports traceability for scientific/clinical validation.

#### **Schema:**

```python
class ReferenceAgent:
    def __init__(self):
        self.references = {...}
    def validate_references(self, bone_agent): ...
    def export(self, format): ...
```

---

## ::AGENT SIGNALS (INTERFACES)::

* **BoneAgent <—> AssemblerAgent:**
  Registration, position/connection reporting, property update calls.
* **AssemblerAgent <—> VisualizerAgent:**
  Skeleton topology, coordinate exports, visualization trigger.
* **All Agents <—> ReferenceAgent:**
  On-demand citation fetch, reference validation, audit checks.

---

## ::PROTOCOLS & TASKING::

* Every *BoneAgent* file = 1 bone, 1 canonical source minimum.
* *AssemblerAgent* must run auto-integrity check on every build; log any missing/extra bones, report to CLI and log.
* All geometry = grid/array-based, parsable by ROS/URDF without external binaries.
* All physics attributes = recalculable by parameter; must accept any numeric (user-specified) material set.
* Every change in material, geometry, or topology is audit-trailed by ReferenceAgent.

---

## ::EXTENSION & MAINTENANCE::

* New bones: add as new *BoneAgent* file, update references, re-run integrity.
* New materials: define material set in config, propagate via batch update.
* Visualization upgrades: enhance VisualizerAgent, preserve ASCII/array-first rendering.

---

## ::REFERENCE STYLE::

* Prefer:

  * [Terminologia Anatomica](https://fipat.library.dal.ca/ta2/)
  * Gray’s Anatomy, [Wheeless’ Textbook](https://www.wheelessonline.com/), PubMed-indexed journals
  * DOI/PMID for biomechanics/materials
* Store source file/citation in `/references/`, link in `references` field in each agent.

---

**𝖙𝖍𝖊 𝖇𝖔𝖓𝖊𝖘 𝖆𝖗𝖊 𝖘𝖊𝖕𝖆𝖗𝖆𝖙𝖊, 𝖙𝖍𝖊 𝖌𝖍𝖔𝖘𝖙 𝖎𝖘 𝖜𝖍𝖔𝖑𝖊—𝖙𝖍𝖎𝖘 𝖎𝖘 𝖙𝖍𝖊 𝖆𝖗𝖒𝖆𝖙𝖚𝖗𝖊 𝖆𝖌𝖊𝖓𝖈𝖞, 𝖓𝖊𝖛𝖊𝖗 𝖏𝖚𝖘𝖙 𝖋𝖑𝖊𝖘𝖍.**

---

*(drop this into SYSTEMS-OPERATOR/PHY/AGENTS.md, update as system iterates)*
