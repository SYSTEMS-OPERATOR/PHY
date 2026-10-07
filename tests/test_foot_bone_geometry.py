"""Independent foot distribution/frame checks; no anatomical fidelity claim."""
import ast
import copy
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from skeleton.visualization import studio_model as sm
from skeleton.visualization.bone_equivalence import bone_audit

ROOT = Path(__file__).resolve().parents[1]


class FootBoneGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = sm.build_armature()
        cls.bones = {p["bone_id"]: p for p in cls.model["parts"] if p["region"] == "feet"}

    def test_exact_independent_foot_inventory_and_unknown_source_dimensions(self):
        tarsals = ("TALUS", "CALCANEUS", "NAVICULAR", "CUBOID", "MEDIAL_CUNEIFORM",
                   "INTERMEDIATE_CUNEIFORM", "LATERAL_CUNEIFORM")
        for side in ("R", "L"):
            wanted = {f"BONE_{code}_{side}" for code in tarsals}
            wanted |= {f"BONE_MT{i}_{side}" for i in range(1, 6)}
            wanted |= {f"BONE_T_PHAL_{i}_{j}_{side}" for i in range(1, 6) for j in range(1, 3 if i == 1 else 4)}
            self.assertEqual(len(wanted), 26)
            self.assertEqual({key for key in self.bones if key.endswith("_"+side)}, wanted)
            for row in self.model["bone_equivalence"]["bones"]:
                if row["bone_id"] not in wanted:
                    continue
                with self.subTest(bone=row["bone_id"]):
                    self.assertEqual(row["source_record_status"], "present")
                    self.assertEqual(row["representation"], "individual_project_proxy")
                    tree = ast.parse((ROOT / row["source_record"]).read_text())
                    dimensions = [ast.literal_eval(k.value) for n in ast.walk(tree) if isinstance(n, ast.Call)
                                  and isinstance(n.func, ast.Name) and n.func.id == "BoneSpec"
                                  for k in n.keywords if k.arg == "dimensions"]
                    self.assertEqual(len(dimensions), 1)
                    self.assertTrue(dimensions[0] and all(v is None for v in dimensions[0].values()))
        self.assertTrue(all(p["role"] == "bone_proxy" for p in self.bones.values()))
        self.assertFalse(any(p.get("grouped_bone_ids") for p in self.bones.values()))

    def test_tarsal_frame_medial_hallux_and_grounding_across_overlays(self):
        for height, pose in ((None, 0), (1676.4, 45), (1800, 80)):
            model = sm.build_armature(height_mm=height, arm_drop_deg=pose)
            bones = {p["bone_id"]: p for p in model["parts"] if p["region"] == "feet"}
            scale = model["scale_from_mean"]
            length = model["reference"]["statistics"]["footlength"]["mean_mm"]*scale
            breadth = model["reference"]["statistics"]["footbreadthhorizontal"]["mean_mm"]*scale
            for side, sign in (("R", 1), ("L", -1)):
                with self.subTest(height=height, pose=pose, side=side):
                    ankle = model["landmarks"]["ankle_"+side]
                    def local(code):
                        v = bones[f"BONE_{code}_{side}"]["center_mm"]
                        return [sign*(v[0]-ankle[0]), v[1]-ankle[1], v[2]]
                    talus, heel = local("TALUS"), local("CALCANEUS")
                    self.assertGreater(talus[2], max(p["center_mm"][2] for p in bones.values() if p["id"].startswith(
                        ("calcaneus_", "navicular_", "cuboid_", "medial_cuneiform_", "intermediate_cuneiform_", "lateral_cuneiform_"))))
                    self.assertAlmostEqual(talus[0], 0)
                    self.assertAlmostEqual(talus[1], 0)
                    self.assertGreater(talus[2], heel[2])
                    self.assertGreater(local("NAVICULAR")[1], talus[1])
                    self.assertGreater(local("CUBOID")[1], heel[1])
                    row = ["MEDIAL_CUNEIFORM", "INTERMEDIATE_CUNEIFORM", "LATERAL_CUNEIFORM", "CUBOID"]
                    self.assertTrue(all(local(a)[0] < local(b)[0] for a, b in zip(row, row[1:])))
                    self.assertTrue(all(local(c)[1] > local("NAVICULAR")[1] for c in row[:3]))
                    self.assertTrue(all(local(f"MT{i}")[0] < local(f"MT{i+1}")[0] for i in range(1, 5)))
                    self.assertLess(local("MT1")[0], 0)  # Hallux toward subject midline.
                    self.assertGreater(local("MT5")[0], 0)
                    heel_mesh = bones[f"BONE_CALCANEUS_{side}"]
                    self.assertEqual(heel_mesh["bounds_mm"]["min"][2], 0)
                    heel_y = ankle[1]-model["foot_proxy_layout"]["heel_behind_ankle_fraction"]*length
                    self.assertAlmostEqual(heel_mesh["bounds_mm"]["min"][1], heel_y, places=4)
                    for p in (p for p in bones.values() if p["bone_id"].endswith("_"+side)):
                        self.assertGreaterEqual(p["bounds_mm"]["min"][2], 0)
                        self.assertTrue(all(abs(v[0]-ankle[0]) <= breadth/2+1e-5 for v in p["vertices"]))
                    tip = bones[f"BONE_T_PHAL_2_3_{side}"]["joint_stations_mm"][1]
                    self.assertAlmostEqual(tip[1]-heel_y, length)

    def test_toe_chain_parents_segments_gaps_and_metatarsal_order(self):
        roots = ("MEDIAL_CUNEIFORM", "INTERMEDIATE_CUNEIFORM", "LATERAL_CUNEIFORM", "CUBOID", "CUBOID")
        for side in ("R", "L"):
            lengths = []
            for digit in range(1, 6):
                codes = [f"MT{digit}"] + [f"T_PHAL_{digit}_{j}" for j in range(1, 3 if digit == 1 else 4)]
                chain = [self.bones[f"BONE_{c}_{side}"] for c in codes]
                self.assertEqual([p["chain_index"] for p in chain], list(range(len(chain))))
                self.assertEqual(chain[0]["topology_parent"], f"BONE_{roots[digit-1]}_{side}")
                lengths.append(math.dist(*chain[0]["joint_stations_mm"]))
                for i, p in enumerate(chain):
                    self.assertEqual(p["digit_index"], digit)
                    a, b = p["joint_stations_mm"]
                    x, y = p["endpoints_mm"]
                    self.assertLess(a[1], b[1])
                    self.assertGreater(math.dist(x, y), 0)
                    self.assertAlmostEqual(math.dist(a, x), p["display_gap_mm"]/2)
                    self.assertAlmostEqual(math.dist(b, y), p["display_gap_mm"]/2)
                    self.assertAlmostEqual(math.dist(x, y), p["center_distance_mm"])
                    if i:
                        self.assertEqual(chain[i-1]["joint_stations_mm"][1], a)
                        self.assertEqual(p["topology_parent"], chain[i-1]["bone_id"])
                        self.assertEqual(chain[i-1]["topology_child"], p["bone_id"])
                self.assertIsNone(chain[-1]["topology_child"])
            self.assertGreater(lengths[1], max(v for i, v in enumerate(lengths) if i != 1))
            # Tarsal parent labels are partial relationships, not fabricated sockets.
            self.assertEqual(self.bones[f"BONE_NAVICULAR_{side}"]["topology_parent"], f"BONE_TALUS_{side}")
            self.assertEqual(self.bones[f"BONE_CUBOID_{side}"]["topology_parent"], f"BONE_CALCANEUS_{side}")

    def test_mirror_includes_winding_stations_anchors_and_side_qualified_provenance(self):
        for key, right in self.bones.items():
            if not key.endswith("_R"):
                continue
            left = self.bones[key[:-1]+"L"]
            with self.subTest(bone=key):
                self.assertEqual(left["mirror_of"], right["id"])
                self.assertEqual(left["vertices"], [[-v[0], v[1], v[2]] for v in right["vertices"]])
                self.assertEqual(left["faces"], [[a, c, b] for a, b, c in right["faces"]])
                self.assertEqual(left["center_mm"], [-right["center_mm"][0], *right["center_mm"][1:]])
                for field in ("endpoints_mm", "joint_stations_mm"):
                    if field in right:
                        self.assertEqual(left[field], [[-v[0], v[1], v[2]] for v in right[field]])
                for field in ("topology_parent", "topology_child"):
                    value = right.get(field)
                    self.assertEqual(left.get(field), value[:-1]+"L" if value else None)
                self.assertEqual(self.model["bone_anchors_mm"][key[:-1]+"L"], left["center_mm"])
                self.assertEqual(left["geometry_inputs"], [v.replace("ankle_R", "ankle_L") for v in right["geometry_inputs"]])
                self.assertEqual(left["foot_frame_side"], "L")

    def test_scale_pose_invariance_hash_and_evidence_status(self):
        bigger = sm.build_armature(height_mm=1800)
        posed = sm.build_armature(arm_drop_deg=0)
        self.assertEqual([p for p in posed["parts"] if p["region"] == "feet"], list(self.bones.values()))
        scale = bigger["scale_from_mean"]
        bones = {p["bone_id"]: p for p in bigger["parts"] if p["region"] == "feet"}
        self.assertEqual(self.model["foot_proxy_layout_sha256"], sm.digest(self.model["foot_proxy_layout"]))
        for key, p in self.bones.items():
            with self.subTest(bone=key):
                for actual, wanted in zip(bones[key]["center_mm"], sm.mul(p["center_mm"], scale)):
                    self.assertAlmostEqual(actual, wanted)
                if "center_distance_mm" in p:
                    self.assertAlmostEqual(bones[key]["center_distance_mm"], p["center_distance_mm"]*scale)
                    self.assertAlmostEqual(bones[key]["display_gap_mm"], p["display_gap_mm"]*scale)
                self.assertEqual(p["foot_layout_sha256"], self.model["foot_proxy_layout_sha256"])
                self.assertEqual((p["dimensional_fidelity"], p["physical_evidence"]), ("unverified", "unmeasured"))
                self.assertTrue(any("#/foot_proxy_layout/" in v for v in p["geometry_inputs"]))
        self.assertEqual(bigger["canon_effect"], "none")
        self.assertFalse(bigger["fabrication_released"])
        self.assertFalse(bigger["bone_equivalence"]["dimensional_fidelity_verified"])

    def test_lost_hallux_identity_cannot_hide_behind_unchanged_mesh_count(self):
        parts = copy.deepcopy(self.model["parts"])
        p = next(p for p in parts if p.get("bone_id") == "BONE_T_PHAL_1_2_R")
        p.pop("bone_id"); p["role"] = "hardware"
        audit = bone_audit(parts)
        self.assertEqual(len(parts), len(self.model["parts"]))
        self.assertEqual(audit["missing_foot_bones"], ["BONE_T_PHAL_1_2_R"])
        self.assertFalse(audit["bone_distribution_complete"])

    def test_invalid_profile_fails_closed(self):
        original = sm.read
        mutations = {
            "missing_tarsal": lambda p: p["tarsals"].pop("TALUS"),
            "extra_digit": lambda p: p["digits"].update({"6": p["digits"]["5"]}),
            "bad_vector": lambda p: p["tarsals"]["TALUS"].update(center=[0, .2]),
            "nonfinite": lambda p: p["tarsals"]["TALUS"]["radii"].__setitem__(0, float("nan")),
            "negative_section": lambda p: p["digits"]["2"]["radii_mm"].__setitem__(0, -1),
            "wrong_hallux_count": lambda p: p["digits"]["1"]["stations"].append([-.36, 1, 9]),
            "wrong_root": lambda p: p["digits"]["1"].update(root="CUBOID"),
            "reversed_station": lambda p: p["digits"]["2"]["stations"][1].__setitem__(1, .1),
            "lateral_hallux": lambda p: p["digits"]["1"]["stations"][-1].__setitem__(0, .49),
            "reversed_cuneiforms": lambda p: p["tarsals"]["MEDIAL_CUNEIFORM"]["center"].__setitem__(0, .18),
            "posterior_navicular": lambda p: p["tarsals"]["NAVICULAR"]["center"].__setitem__(1, .2),
            "low_talus": lambda p: p["tarsals"]["TALUS"]["center"].__setitem__(2, 35),
            "below_floor": lambda p: p["tarsals"]["CALCANEUS"]["center"].__setitem__(2, 17),
            "hovering_heel": lambda p: p["tarsals"]["CALCANEUS"]["center"].__setitem__(2, 19),
            "unclosed_heel": lambda p: p["tarsals"]["CALCANEUS"]["center"].__setitem__(1, .14),
            "bad_heel_offset": lambda p: p.update(heel_behind_ankle_fraction=1),
            "zero_gap": lambda p: p.update(joint_gap_mm=0),
            "consumed_segment": lambda p: p.update(joint_gap_mm=500),
            "thin_first_metatarsal": lambda p: p["digits"]["1"]["radii_mm"].__setitem__(0, 3),
            "short_second_metatarsal": lambda p: p["digits"]["2"]["stations"][1].__setitem__(1, .7),
        }
        for kind, change in mutations.items():
            def altered(path):
                data = original(path)
                if Path(path).name == "f28.json":
                    change(data["foot_proxy_layout"])
                return data
            with self.subTest(kind=kind), patch.object(sm, "read", altered), self.assertRaises(ValueError):
                sm.build_armature()


if __name__ == "__main__":
    unittest.main()
