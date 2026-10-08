"""Independent final-seven identity, geometry, placement and evidence checks."""
import ast
from collections import Counter
import copy
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from skeleton.visualization import studio_model as sm
from skeleton.visualization.bone_equivalence import bone_audit


class EarHyoidGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = sm.build_armature()
        cls.bones = {p["bone_id"]: p for p in cls.model["parts"] if p.get("ear_hyoid_layout_sha256")}

    def test_exact_seven_identities_and_206_coverage_without_source_or_release_adoption(self):
        wanted = {f"BONE_{c}_{s}" for c in ("MALLEUS", "INCUS", "STAPES") for s in ("R", "L")}
        wanted.add("BONE_HYOID")
        self.assertEqual(set(self.bones), wanted)
        audit = self.model["bone_equivalence"]
        self.assertEqual((audit["individual_bone_proxies"], audit["grouped_bones"], audit["unrepresented_bones"]), (206, 0, 0))
        self.assertTrue(audit["bone_distribution_complete"])
        self.assertEqual(audit["missing_ear_bones"], [])
        self.assertEqual(audit["missing_hyoid_bones"], [])
        self.assertEqual(audit["source_records_present"], 193)
        self.assertEqual(len(audit["missing_source_records"]), 13)
        self.assertFalse(audit["dimensional_fidelity_verified"])
        self.assertFalse(audit["fabrication_released"])
        self.assertFalse(audit["physical_evidence_complete"])
        self.assertFalse(self.model["fabrication_released"])
        for p in self.bones.values():
            self.assertEqual(p["source_record_status"], "present")
            tree = ast.parse((sm.ROOT / p["source_record"]).read_text())
            dimensions = [ast.literal_eval(k.value) for n in ast.walk(tree) if isinstance(n, ast.Call)
                          and isinstance(n.func, ast.Name) and n.func.id == "BoneSpec"
                          for k in n.keywords if k.arg == "dimensions"]
            self.assertEqual(dimensions, [{"length_cm": 2.5, "width_cm": 3.5, "thickness_cm": 1.0}]
                             if p["bone_id"] == "BONE_HYOID" else [{"length_cm": 0.8, "width_cm": 0.4, "thickness_cm": 0.3}])

    def test_previous_199_proxies_are_byte_identical(self):
        bones = [p for p in self.model["parts"] if p["role"] == "bone_proxy" and not p.get("ear_hyoid_layout_sha256")]
        self.assertEqual(len(bones), 199)
        self.assertEqual(sm.digest(bones), "5d8610aaee83e5d487739c66818af8c73cc22ee1eca2b49810141766f9f8f6cc")

    def test_ear_chain_lateral_medial_order_and_non_bone_boundaries(self):
        for side, sign in (("R", 1), ("L", -1)):
            chain = [self.bones[f"BONE_{c}_{side}"] for c in ("MALLEUS", "INCUS", "STAPES")]
            self.assertTrue(all(sign*a["center_mm"][0] > sign*b["center_mm"][0] for a, b in zip(chain, chain[1:])))
            self.assertEqual([p["chain_index"] for p in chain], [0, 1, 2])
            self.assertEqual(chain[0]["topology_parent"], None)
            self.assertEqual(chain[0]["topology_child"], f"BONE_INCUS_{side}")
            self.assertEqual(chain[1]["topology_parent"], f"BONE_MALLEUS_{side}")
            self.assertEqual(chain[1]["topology_child"], f"BONE_STAPES_{side}")
            self.assertEqual(chain[2]["topology_parent"], f"BONE_INCUS_{side}")
            self.assertEqual(chain[2]["topology_child"], None)
            self.assertEqual([p["non_bone_connection"] for p in chain], ["tympanic_membrane", None, "oval_window"])
            for p in chain:
                self.assertEqual(p["housing_bone_id"], f"BONE_TEMP_{side}")
                self.assertNotIn(p["housing_bone_id"], p["articulates_with"])
                self.assertFalse(p["motion_implemented"])

    def test_one_hyoid_is_below_jaw_symmetric_open_and_not_articulated(self):
        p = self.bones["BONE_HYOID"]
        jaw = next(p for p in self.model["parts"] if p.get("bone_id") == "BONE_MANDIBLE")
        self.assertLess(p["bounds_mm"]["max"][2], jaw["bounds_mm"]["min"][2])
        self.assertEqual((p["topology_parent"], p["topology_child"]), (None, None))
        self.assertEqual(p["articulates_with"], [])
        self.assertEqual(p["topology_neighbors"], [])
        self.assertFalse(p["closed_centerline"])
        self.assertFalse(p["motion_implemented"])
        self.assertEqual(p["non_bone_support"], "muscle_and_ligament_suspension_unmodeled")
        points = p["centerline_mm"]
        self.assertEqual(p["center_mm"][0], 0)
        self.assertLess(points[0][1], points[len(points)//2][1])
        for a, b in zip(points, reversed(points)):
            self.assertEqual([-a[0], a[1], a[2]], b)
        self.assertEqual(Counter(k for k in self.bones if "HYOID" in k), {"BONE_HYOID": 1})

    def test_reflected_geometry_and_side_specific_topology_source_paths(self):
        for code in ("MALLEUS", "INCUS", "STAPES"):
            right, left = (self.bones[f"BONE_{code}_{s}"] for s in ("R", "L"))
            self.assertEqual(left["vertices"], [[-v[0], v[1], v[2]] for v in right["vertices"]])
            self.assertEqual(left["faces"], [[a, c, b] for a, b, c in right["faces"]])
            self.assertEqual(left["centerline_mm"], [[-v[0], v[1], v[2]] for v in right["centerline_mm"]])
            self.assertEqual(left["mirror_of"], right["id"])
            self.assertEqual(left["ear_hyoid_frame_side"], "L")
            self.assertEqual(left["topology_neighbors"], [v[:-1]+"L" for v in right["topology_neighbors"]])
            self.assertNotEqual(left["source_record"], right["source_record"])

    def test_closed_oriented_connected_solids_and_stirrup_hole(self):
        for key, p in self.bones.items():
            with self.subTest(bone=key):
                directed = Counter((f[i], f[(i+1)%3]) for f in p["faces"] for i in range(3))
                self.assertTrue(all(n == 1 and directed[(b, a)] == 1 for (a, b), n in directed.items()))
                euler = len(p["vertices"])-len(directed)//2+len(p["faces"])
                self.assertEqual(euler, 0 if "STAPES" in key else 2)
                graph = {i: set() for i in range(len(p["vertices"]))}
                volume = 0
                for face in p["faces"]:
                    a, b, c = [p["vertices"][i] for i in face]
                    self.assertTrue(all(math.isfinite(v) for point in (a, b, c) for v in point))
                    self.assertGreater(sum(v*v for v in sm.cross(sm.sub(b, a), sm.sub(c, a))), 1e-10)
                    volume += sum(a[i]*sm.cross(b, c)[i] for i in range(3))/6
                    for i in range(3): graph[face[i]].add(face[(i+1)%3])
                seen, pending = set(), [0]
                while pending:
                    i = pending.pop()
                    if i not in seen: seen.add(i); pending.extend(graph[i]-seen)
                self.assertEqual(len(seen), len(p["vertices"]))
                self.assertGreater(volume, 0)
                if "STAPES" in key:
                    self.assertTrue(p["closed_centerline"])
                    origin = p["center_mm"]
                    self.assertGreater(min(math.hypot(v[1]-origin[1], v[2]-origin[2]) for v in p["vertices"]), .5)

    def test_traceable_scale_across_overlays_and_repeated_build(self):
        self.assertEqual(sm.digest(self.model), sm.digest(sm.build_armature()))
        for height, pose in ((1400, 0), (1676.4, 45), (1900, 80)):
            m = sm.build_armature(height_mm=height, arm_drop_deg=pose)
            bones = {p["bone_id"]: p for p in m["parts"] if p.get("ear_hyoid_layout_sha256")}
            ratio = m["scale_from_mean"]
            self.assertEqual(m["ear_hyoid_proxy_layout_sha256"], sm.digest(m["ear_hyoid_proxy_layout"]))
            for key, original in self.bones.items():
                p = bones[key]
                self.assertEqual((p["dimensional_fidelity"], p["physical_evidence"]), ("unverified", "unmeasured"))
                self.assertEqual(p["ear_hyoid_layout_sha256"], m["ear_hyoid_proxy_layout_sha256"])
                self.assertTrue(any("#/ear_hyoid_proxy_layout/" in v for v in p["geometry_inputs"]))
                for a, b in zip(p["vertices"], original["vertices"]):
                    for v, w in zip(a, b): self.assertAlmostEqual(v, w*ratio, delta=2e-5)
            self.assertEqual(m["canon_effect"], "none")

    def test_lost_ossicle_or_hyoid_cannot_hide_behind_total_mesh_count(self):
        for key, missing_field in (("BONE_STAPES_R", "missing_ear_bones"), ("BONE_HYOID", "missing_hyoid_bones")):
            parts = copy.deepcopy(self.model["parts"])
            p = next(p for p in parts if p.get("bone_id") == key)
            p.pop("bone_id"); p["role"] = "hardware"
            self.assertEqual(len(parts), len(self.model["parts"]))
            audit = bone_audit(parts)
            self.assertEqual(audit[missing_field], [key])
            self.assertFalse(audit["bone_distribution_complete"])

    def test_invalid_profile_mutations_fail_closed(self):
        original = sm.read
        mutations = {
            "missing_ossicle": lambda p: p["ossicles"].pop("INCUS"),
            "extra_ossicle": lambda p: p["ossicles"].update(TOOTH=p["ossicles"]["INCUS"]),
            "revision": lambda p: p.update(revision="EAR_HYOID_V2"),
            "unknown_key": lambda p: p.update(unknown=1),
            "authority": lambda p: p.update(authority=""),
            "origin_nan": lambda p: p.update(ear_origin_head_fraction=[.67, 0, float("nan")]),
            "origin_vector": lambda p: p.update(ear_origin_head_fraction=[.67, 0]),
            "wrong_side": lambda p: p.update(ear_origin_head_fraction=[-.67, -.08, -.25]),
            "ear_above_vault": lambda p: p.update(ear_origin_head_fraction=[.67, -.08, .25]),
            "hyoid_off_midline": lambda p: p.update(hyoid_origin_head_fraction=[.1, .3, -1.05]),
            "hyoid_anterior": lambda p: p.update(hyoid_origin_head_fraction=[0, -.3, -1.05]),
            "hyoid_through_jaw": lambda p: p.update(hyoid_origin_head_fraction=[0, .3, -.91]),
            "item_none": lambda p: p["ossicles"].update(MALLEUS=None),
            "offset_string": lambda p: p["ossicles"]["MALLEUS"].update(center_offset_mm=["5", 0, 0]),
            "outside_head": lambda p: p["ossicles"]["MALLEUS"].update(center_offset_mm=[1000, 0, 0]),
            "reversed_chain": lambda p: p["ossicles"]["MALLEUS"].update(center_offset_mm=[-10, 0, 0]),
            "section_boolean": lambda p: p["ossicles"]["STAPES"].update(radii_mm=True),
            "section_zero": lambda p: p["ossicles"]["STAPES"].update(radii_mm=0),
            "section_infinite": lambda p: p["ossicles"]["STAPES"].update(radii_mm=float("inf")),
            "section_count": lambda p: p["ossicles"]["MALLEUS"].update(radii_mm=[1, 2]),
            "section_consumed": lambda p: p["ossicles"]["STAPES"].update(radii_mm=100),
            "empty_sweep": lambda p: p["ossicles"]["INCUS"].update(centerline_mm=[]),
            "duplicate_station": lambda p: p["ossicles"]["INCUS"]["centerline_mm"].__setitem__(1, p["ossicles"]["INCUS"]["centerline_mm"][0]),
            "open_stapes": lambda p: p["ossicles"]["STAPES"].update(closed_centerline=False),
            "nonplanar_stapes": lambda p: p["ossicles"]["STAPES"]["centerline_mm"].__setitem__(0, [.1, 0, 1.8]),
            "reversed_stapes": lambda p: p["ossicles"]["STAPES"]["centerline_mm"].reverse(),
            "closed_hyoid": lambda p: p["hyoid"].update(closed_centerline=True),
            "hyoid_asymmetry": lambda p: p["hyoid"]["centerline_mm"].__setitem__(-1, [-10, -10, 0]),
            "hyoid_reversed": lambda p: p["hyoid"]["centerline_mm"].reverse(),
            "hyoid_sections": lambda p: p["hyoid"].update(radii_mm=[2, 2, 2, 2, 2, 2, 3]),
        }
        for label, mutate in mutations.items():
            def altered(path):
                p = original(path)
                if Path(path).name == "f28.json": mutate(p["ear_hyoid_proxy_layout"])
                return p
            with self.subTest(mutation=label), patch.object(sm, "read", altered), self.assertRaises(ValueError):
                sm.build_armature()


if __name__ == "__main__":
    unittest.main()
