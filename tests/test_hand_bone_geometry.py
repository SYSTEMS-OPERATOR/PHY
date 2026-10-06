"""Independent hand distribution/frame regressions; no anatomical fidelity claim."""
import copy
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from skeleton.visualization import studio_model as sm
from skeleton.visualization.bone_equivalence import bone_audit


class HandBoneGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = sm.build_armature()
        cls.bones = {p["bone_id"]: p for p in cls.model["parts"] if p["region"] == "hands"}

    def test_exact_independent_hand_inventory_and_source_correspondence(self):
        carpals = ("SCAPHOID", "LUNATE", "TRIQUETRUM", "PISIFORM", "TRAPEZIUM", "TRAPEZOID", "CAPITATE", "HAMATE")
        for side in ("R", "L"):
            wanted = {f"BONE_{code}_{side}" for code in carpals}
            wanted |= {f"BONE_META{i}_{side}" for i in range(1, 6)}
            wanted |= {f"BONE_PHAL_{i}_{j}_{side}" for i in range(1, 6) for j in range(1, 3 if i == 1 else 4)}
            self.assertEqual(len(wanted), 27)
            self.assertEqual({key for key in self.bones if key.endswith("_"+side)}, wanted)
            for row in self.model["bone_equivalence"]["bones"]:
                if row["bone_id"] in wanted:
                    self.assertEqual(row["source_record_status"], "present")
                    self.assertEqual(row["representation"], "individual_project_proxy")
        self.assertTrue(all(p["role"] == "bone_proxy" for p in self.bones.values()))
        self.assertFalse(any(p.get("grouped_bone_ids") for p in self.bones.values()))

    def test_carpal_rows_radial_thumb_and_palmar_pisiform_in_multiple_poses(self):
        for height, pose in ((None, 0), (1676.4, 45), (1800, 80)):
            model = sm.build_armature(height_mm=height, arm_drop_deg=pose)
            bones = {p["bone_id"]: p for p in model["parts"] if p["role"] == "bone_proxy"}
            for side, sign in (("R", 1), ("L", -1)):
                with self.subTest(height=height, pose=pose, side=side):
                    a = math.radians(pose)
                    distal, radial = [sign*math.cos(a), 0, -math.sin(a)], [sign*math.sin(a), 0, math.cos(a)]
                    wrist = model["landmarks"]["wrist_"+side]
                    def local(code, field="center_mm", index=None):
                        point = bones[f"BONE_{code}_{side}"][field]
                        if index is not None:
                            point = point[index]
                        v = [x-y for x, y in zip(point, wrist)]
                        return [math.fsum(x*y for x, y in zip(v, distal)),
                                math.fsum(x*y for x, y in zip(v, radial)), v[1]]
                    proximal = ["SCAPHOID", "LUNATE", "TRIQUETRUM", "PISIFORM"]
                    distal_row = ["TRAPEZIUM", "TRAPEZOID", "CAPITATE", "HAMATE"]
                    self.assertLess(max(local(c)[0] for c in proximal), min(local(c)[0] for c in distal_row))
                    for row in (proximal[:3], distal_row):
                        self.assertTrue(all(local(c)[1] > local(d)[1] for c, d in zip(row, row[1:])))
                    self.assertGreater(local("PISIFORM")[2], local("TRIQUETRUM")[2])
                    self.assertAlmostEqual(local("PISIFORM")[1], local("TRIQUETRUM")[1])
                    self.assertGreater(local("RADIUS", "endpoints_mm", 1)[1], local("ULNA", "endpoints_mm", 1)[1])
                    self.assertGreater(local("META1")[1], local("META2")[1])
                    self.assertGreater(local("META1")[2], local("META2")[2])
                    # Thumb is a separately angled three-bone chain, not a fifth parallel finger.
                    m = bones[f"BONE_META1_{side}"]["joint_stations_mm"]
                    self.assertGreater(abs(math.fsum((m[1][i]-m[0][i])*radial[i] for i in range(3))), 5)

    def test_segment_order_gap_and_middle_fingertip_station_closure(self):
        for side in ("R", "L"):
            for digit in range(1, 6):
                codes = [f"META{digit}"] + [f"PHAL_{digit}_{j}" for j in range(1, 3 if digit == 1 else 4)]
                chain = [self.bones[f"BONE_{c}_{side}"] for c in codes]
                self.assertEqual([p["chain_index"] for p in chain], list(range(len(chain))))
                self.assertEqual(chain[0]["topology_parent"], f"BONE_{('TRAPEZIUM', 'TRAPEZOID', 'CAPITATE', 'HAMATE', 'HAMATE')[digit-1]}_{side}")
                for i, p in enumerate(chain):
                    self.assertEqual(p["digit_index"], digit)
                    a, b = p["joint_stations_mm"]
                    x, y = p["endpoints_mm"]
                    self.assertGreater(math.dist(x, y), 0)
                    self.assertAlmostEqual(math.dist(a, x), p["display_gap_mm"]/2)
                    self.assertAlmostEqual(math.dist(b, y), p["display_gap_mm"]/2)
                    self.assertAlmostEqual(math.dist(x, y), p["center_distance_mm"])
                    if i:
                        self.assertEqual(chain[i-1]["joint_stations_mm"][1], a)
                        self.assertEqual(p["topology_parent"], chain[i-1]["bone_id"])
                        self.assertEqual(chain[i-1]["topology_child"], p["bone_id"])
                self.assertIsNone(chain[-1]["topology_child"])
            for actual, wanted in zip(self.bones[f"BONE_PHAL_3_3_{side}"]["joint_stations_mm"][1], self.model["landmarks"]["fingertip_"+side]):
                self.assertAlmostEqual(actual, wanted)

    def test_mirror_covers_anchors_topology_provenance_and_winding(self):
        for key, right in self.bones.items():
            if not key.endswith("_R"):
                continue
            left = self.bones[key[:-1]+"L"]
            self.assertEqual(left["mirror_of"], right["id"])
            self.assertEqual(left["vertices"], [[-v[0], v[1], v[2]] for v in right["vertices"]])
            self.assertEqual(left["faces"], [[a, c, b] for a, b, c in right["faces"]])
            self.assertEqual(left["center_mm"], [-right["center_mm"][0], *right["center_mm"][1:]])
            for field in ("topology_parent", "topology_child"):
                value = right.get(field)
                self.assertEqual(left.get(field), value[:-1]+"L" if value else None)
            self.assertEqual(self.model["bone_anchors_mm"][key[:-1]+"L"], left["center_mm"])
            self.assertIn("model#/landmarks/wrist_L", left["geometry_inputs"])
            self.assertEqual(left["hand_frame_side"], "L")

    def test_sections_stations_and_gaps_scale_without_canon_adoption(self):
        larger = sm.build_armature(height_mm=1800)
        scale = larger["scale_from_mean"]
        bones = {p["bone_id"]: p for p in larger["parts"] if p["region"] == "hands"}
        for key, p in self.bones.items():
            if "center_distance_mm" in p:
                self.assertAlmostEqual(bones[key]["center_distance_mm"], p["center_distance_mm"]*scale)
                self.assertAlmostEqual(bones[key]["display_gap_mm"], p["display_gap_mm"]*scale)
        self.assertEqual(larger["canon_effect"], "none")
        self.assertFalse(larger["fabrication_released"])
        self.assertFalse(larger["bone_equivalence"]["dimensional_fidelity_verified"])
        self.assertEqual(self.model["hand_proxy_layout_sha256"], sm.digest(self.model["hand_proxy_layout"]))
        for p in self.bones.values():
            self.assertEqual(p["hand_layout_sha256"], self.model["hand_proxy_layout_sha256"])
            self.assertEqual((p["dimensional_fidelity"], p["physical_evidence"]), ("unverified", "unmeasured"))
            self.assertTrue(any("#/hand_proxy_layout/" in v for v in p["geometry_inputs"]))

    def test_missing_hand_identity_is_reported_even_if_mesh_count_is_unchanged(self):
        parts = copy.deepcopy(self.model["parts"])
        p = next(p for p in parts if p.get("bone_id") == "BONE_PHAL_1_2_R")
        p.pop("bone_id"); p["role"] = "hardware"
        audit = bone_audit(parts)
        self.assertEqual(len(parts), len(self.model["parts"]))
        self.assertEqual(audit["missing_hand_bones"], ["BONE_PHAL_1_2_R"])
        self.assertFalse(audit["bone_distribution_complete"])

    def test_invalid_layout_fails_closed(self):
        original = sm.read
        for kind in ("missing_carpal", "row", "negative_section", "bad_vector", "nonfinite", "wrong_thumb_count",
                     "wrong_root", "reversed_station", "ulnar_thumb", "dorsal_pisiform", "zero_gap", "consumed_segment"):
            def altered(path):
                data = original(path)
                if Path(path).name != "f28.json":
                    return data
                p = data["hand_proxy_layout"]
                if kind == "missing_carpal": p["carpals"].pop("SCAPHOID")
                elif kind == "row": p["carpals"]["SCAPHOID"]["row"] = "distal"
                elif kind == "negative_section": p["digits"]["2"]["radii_mm"][0] = -1
                elif kind == "bad_vector": p["carpals"]["SCAPHOID"]["center"] = [0.1, 0.2]
                elif kind == "nonfinite": p["carpals"]["LUNATE"]["radii_mm"][0] = float("nan")
                elif kind == "wrong_thumb_count": p["digits"]["1"]["stations"].append([0.9, 0.86, 18])
                elif kind == "wrong_root": p["digits"]["1"]["root"] = "HAMATE"
                elif kind == "reversed_station": p["digits"]["2"]["stations"][1][0] = 0.1
                elif kind == "ulnar_thumb": p["digits"]["1"]["stations"][-1][1] = -0.8
                elif kind == "dorsal_pisiform": p["carpals"]["PISIFORM"]["center"][2] = -10
                elif kind == "zero_gap": p["joint_gap_mm"] = 0
                else: p["joint_gap_mm"] = 500
                return data
            with self.subTest(kind=kind), patch.object(sm, "read", altered), self.assertRaises(ValueError):
                sm.build_armature()


if __name__ == "__main__":
    unittest.main()
