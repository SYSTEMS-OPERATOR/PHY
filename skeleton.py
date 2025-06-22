from dataclasses import dataclass, asdict
from typing import List, Dict, Optional

@dataclass
class BoneSpec:
    name: str
    bone_type: str
    location: Dict[str, str]
    articulations: List[Dict[str, str]]
    dimensions: Dict[str, Optional[float]]
    function: List[str]
    notable_features: List[str]
    developmental_notes: str
    variations: str
    unique_id: str
    visual_reference: Optional[str] = None

    def to_dict(self) -> Dict:
        return asdict(self)

class Skeleton:
    def __init__(self):
        self.bones: Dict[str, BoneSpec] = {}

    def add_bone(self, bone: BoneSpec) -> None:
        self.bones[bone.unique_id] = bone

    def export(self) -> Dict[str, Dict]:
        return {uid: bone.to_dict() for uid, bone in self.bones.items()}

skeleton = Skeleton()

# Helper function to create bones

def add_bone(name: str, bone_type: str, region: str, uid_suffix: str,
             proximal: str = "", distal: str = "",
             articulations: Optional[List[Dict[str, str]]] = None,
             length: Optional[float] = None, width: Optional[float] = None,
             thickness: Optional[float] = None,
             function: Optional[List[str]] = None,
             features: Optional[List[str]] = None,
             dev_notes: str = "", variations: str = "",
             visual: Optional[str] = None):
    articulations = articulations or []
    function = function or []
    features = features or []
    skeleton.add_bone(BoneSpec(
        name=name,
        bone_type=bone_type,
        location={"region": region, "proximal_connection": proximal, "distal_connection": distal},
        articulations=articulations,
        dimensions={"length_cm": length, "width_cm": width, "thickness_cm": thickness},
        function=function,
        notable_features=features,
        developmental_notes=dev_notes,
        variations=variations,
        unique_id=f"BONE_{uid_suffix}",
        visual_reference=visual,
    ))

# Skull bones
add_bone("Frontal", "flat", "skull", "FRONTAL",
         articulations=[{"bone": "parietal", "joint_type": "suture"}],
         length=13.5, width=14.5, thickness=0.6,
         function=["forms forehead"],
         features=["supraorbital notch"],
         dev_notes="ossifies from two centers", variations="metopic suture persistence")
for side in ("L", "R"):
    add_bone("Parietal", "flat", "skull", f"PAR_{side}",
             proximal="frontal", distal="occipital",
             articulations=[{"bone": "frontal", "joint_type": "suture"}, {"bone": "occipital", "joint_type": "suture"}],
             length=14.0, width=10.0, thickness=0.5,
             function=["protects brain"],
             features=["parietal eminence"],
             dev_notes="intramembranous ossification")
    add_bone("Temporal", "irregular", "skull", f"TEMP_{side}",
             proximal="parietal", distal="mandible",
             articulations=[{"bone": "mandible", "joint_type": "hinge"}],
             length=7.0, width=7.0, thickness=0.5,
             function=["houses ear"],
             features=["mastoid"],
             dev_notes="complex ossification")
add_bone("Occipital", "flat", "skull", "OCCIPITAL",
         proximal="parietal", distal="vertebral column",
         articulations=[{"bone": "atlas", "joint_type": "condyloid"}],
         length=12.0, width=11.0, thickness=0.6,
         function=["protects cerebellum"],
         features=["foramen magnum"],
         dev_notes="multiple ossification centers")
add_bone("Sphenoid", "irregular", "skull", "SPHENOID",
         proximal="frontal", distal="occipital",
         articulations=[{"bone": "frontal", "joint_type": "suture"}],
         length=6.0, width=8.0, thickness=0.5,
         function=["forms skull base"],
         features=["sella turcica"],
         dev_notes="cartilaginous ossification")
add_bone("Ethmoid", "irregular", "skull", "ETHMOID",
         proximal="frontal", distal="vomer",
         articulations=[{"bone": "vomer", "joint_type": "suture"}],
         length=5.0, width=5.0, thickness=0.4,
         function=["supports olfaction"],
         features=["crista galli"],
         dev_notes="cartilaginous origin")
add_bone("Mandible", "irregular", "face", "MANDIBLE",
         proximal="temporal", distal="", articulations=[{"bone": "temporal", "joint_type": "hinge"}],
         length=17.0, width=12.0, thickness=1.0,
         function=["mastication"],
         features=["mental foramen"],
         dev_notes="fuses at symphysis")
# Additional skull and facial bones would continue similarly...

# Hyoid
add_bone("Hyoid", "sesamoid", "neck", "HYOID",
         proximal="", distal="", articulations=[],
         length=2.5, width=3.5, thickness=1.0,
         function=["supports tongue"],
         dev_notes="ossifies from two pairs of horns")

# Auditory ossicles
for side in ("L", "R"):
    for ossicle in ("Malleus", "Incus", "Stapes"):
        add_bone(ossicle, "irregular", "middle ear", f"{ossicle.upper()}_{side}",
                 length=0.8, width=0.4, thickness=0.3,
                 function=["sound transmission"],
                 dev_notes="ossify early")

# Vertebral column
for i in range(1, 8):
    add_bone(f"C{i}", "irregular", "cervical vertebrae", f"C{i}",
             proximal="skull" if i == 1 else f"C{i-1}",
             distal=f"C{i+1}" if i < 7 else "T1",
             function=["support head" if i==1 else "support neck"],
             dev_notes="centers fuse during adolescence")
for i in range(1, 13):
    add_bone(f"T{i}", "irregular", "thoracic vertebrae", f"T{i}",
             proximal=f"T{i-1}" if i>1 else "C7",
             distal=f"T{i+1}" if i<12 else "L1",
             function=["rib articulation"],
             dev_notes="vertebral arch fusion in adolescence")
for i in range(1, 6):
    add_bone(f"L{i}", "irregular", "lumbar vertebrae", f"L{i}",
             proximal=f"T12" if i==1 else f"L{i-1}",
             distal=f"L{i+1}" if i<5 else "sacrum",
             function=["weight bearing"],
             dev_notes="large body for support")
add_bone("Sacrum", "irregular", "sacrum", "SACRUM",
         proximal="L5", distal="coccyx",
         function=["supports pelvis"],
         dev_notes="fusion of 5 sacral vertebrae")
add_bone("Coccyx", "irregular", "coccyx", "COCCYX",
         proximal="sacrum", distal="",
         function=["ligament attachment"],
         dev_notes="fusion of rudimentary vertebrae")

# Sternum and ribs
add_bone("Sternum", "flat", "thorax", "STERNUM",
         proximal="clavicle", distal="ribs",
         function=["protects thoracic organs"],
         features=["manubrium", "xiphoid"],
         dev_notes="fuses from sternebrae")
for side in ("L", "R"):
    for n in range(1, 13):
        add_bone(f"Rib {n}", "flat", "thorax", f"RIB{n}_{side}",
                 proximal="thoracic vertebrae", distal="sternum" if n<=7 else "costal cartilage",
                 function=["protect thoracic cavity"],
                 dev_notes="develop from costal cartilage")

# Shoulder girdle
for side in ("L", "R"):
    add_bone("Clavicle", "long", "shoulder", f"CLAVICLE_{side}",
             proximal="sternum", distal="scapula",
             length=14.0, width=1.0, thickness=1.0,
             function=["strut for shoulder"],
             dev_notes="ossifies first among long bones")
    add_bone("Scapula", "flat", "shoulder", f"SCAPULA_{side}",
             proximal="clavicle", distal="humerus",
             function=["attachment for arm"],
             features=["glenoid fossa"],
             dev_notes="ossifies from multiple centers")

# Arms
for side in ("L", "R"):
    add_bone("Humerus", "long", "upper arm", f"HUMERUS_{side}",
             proximal="scapula", distal="radius/ulna",
             length=35.0, width=2.5, thickness=2.5,
             function=["lever for arm"],
             features=["deltoid tuberosity"],
             dev_notes="growth plates at both ends")
    add_bone("Radius", "long", "forearm", f"RADIUS_{side}",
             proximal="humerus", distal="carpals",
             length=24.0, width=1.5, thickness=1.5,
             function=["forearm rotation"],
             dev_notes="radius head ossifies separately")
    add_bone("Ulna", "long", "forearm", f"ULNA_{side}",
             proximal="humerus", distal="carpals",
             length=25.0, width=1.5, thickness=1.5,
             function=["forearm hinge"],
             dev_notes="olecranon process" )

    # Carpals
    carpals = ["Scaphoid", "Lunate", "Triquetrum", "Pisiform",
               "Trapezium", "Trapezoid", "Capitate", "Hamate"]
    for carpal in carpals:
        add_bone(carpal, "short", "carpal", f"{carpal.upper()}_{side}",
                 proximal="radius/ulna", distal="metacarpals",
                 function=["wrist flexibility"],
                 dev_notes="carpal ossification varies")
    # Metacarpals
    for n in range(1, 6):
        add_bone(f"Metacarpal {n}", "long", "hand", f"META{n}_{side}",
                 proximal="carpals", distal="phalanges",
                 function=["palm support"],
                 dev_notes="growth plate at distal end")
    # Phalanges
    for digit in range(1, 6):
        phalanges = ["Proximal", "Middle", "Distal"] if digit != 1 else ["Proximal", "Distal"]
        for ph_index, ph_name in enumerate(phalanges, 1):
            uid = f"PHAL_{digit}_{ph_index}_{side}"
            add_bone(f"{ph_name} Phalanx {digit}", "long", "finger", uid,
                     proximal="metacarpal" if ph_index == 1 else f"{phalanges[ph_index-2]} phalanx",
                     distal=f"{phalanges[ph_index]} phalanx" if ph_index < len(phalanges) else "",
                     function=["digit movement"],
                     dev_notes="ossification begins distal")

# Pelvic girdle
for side in ("L", "R"):
    add_bone("Hip Bone", "irregular", "pelvis", f"HIP_{side}",
             proximal="sacrum", distal="femur",
             function=["supports trunk"],
             dev_notes="ilium, ischium, pubis fuse in adolescence")

# Legs
for side in ("L", "R"):
    add_bone("Femur", "long", "thigh", f"FEMUR_{side}",
             proximal="hip", distal="tibia",
             length=48.0, width=2.8, thickness=2.8,
             function=["weight bearing"],
             features=["greater trochanter"],
             dev_notes="epiphyseal plates fuse in adulthood")
    add_bone("Patella", "sesamoid", "knee", f"PATELLA_{side}",
             proximal="femur", distal="tibia",
             function=["protect knee joint"],
             dev_notes="ossifies within quadriceps tendon")
    add_bone("Tibia", "long", "leg", f"TIBIA_{side}",
             proximal="femur", distal="tarsals",
             length=40.0, width=2.5, thickness=2.5,
             function=["primary weight bearing"],
             dev_notes="tibial tuberosity prominent")
    add_bone("Fibula", "long", "leg", f"FIBULA_{side}",
             proximal="tibia", distal="tarsals",
             length=40.0, width=1.5, thickness=1.5,
             function=["ankle stability"],
             dev_notes="slender bone")
    # Tarsals
    tarsals = ["Talus", "Calcaneus", "Navicular", "Medial Cuneiform", "Intermediate Cuneiform", "Lateral Cuneiform", "Cuboid"]
    for tarsal in tarsals:
        add_bone(tarsal, "short", "tarsal", f"{tarsal.upper().replace(' ', '_')}_{side}",
                 proximal="tibia/fibula", distal="metatarsals",
                 function=["ankle motion"],
                 dev_notes="ossification varies")
    # Metatarsals
    for n in range(1, 6):
        add_bone(f"Metatarsal {n}", "long", "foot", f"MT{n}_{side}",
                 proximal="tarsals", distal="phalanges",
                 function=["arch support"],
                 dev_notes="growth at distal end")
    # Foot phalanges
    for digit in range(1, 6):
        phalanges = ["Proximal", "Middle", "Distal"] if digit != 1 else ["Proximal", "Distal"]
        for ph_index, ph_name in enumerate(phalanges, 1):
            uid = f"T_PHAL_{digit}_{ph_index}_{side}"
            add_bone(f"{ph_name} Toe Phalanx {digit}", "long", "toe", uid,
                     proximal="metatarsal" if ph_index == 1 else f"{phalanges[ph_index-2]} phalanx",
                     distal=f"{phalanges[ph_index]} phalanx" if ph_index < len(phalanges) else "",
                     function=["toe movement"],
                     dev_notes="ossification begins distal")

if __name__ == "__main__":
    import json
    print(json.dumps(skeleton.export(), indent=2))
