"""Independent skull identities, coarse topology and provisional mesh contracts."""
from collections import Counter
import copy
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from skeleton.visualization import studio_model as sm
from skeleton.visualization.bone_equivalence import bone_audit


class SkullBoneGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = sm.build_armature()
        cls.bones = {p["bone_id"]: p for p in cls.model["parts"] if p.get("skull_layout_sha256")}

    def test_exact_eight_cranial_fourteen_facial_identities_and_thirteen_missing_records(self):
        cranial = {"BONE_"+c for c in ("FRONTAL", "OCCIPITAL", "SPHENOID", "ETHMOID")}
        cranial |= {f"BONE_{c}_{s}" for c in ("PAR", "TEMP") for s in ("R", "L")}
        facial = {"BONE_MANDIBLE", "BONE_VOMER"}
        facial |= {f"BONE_{c}_{s}" for c in ("MAXILLA", "ZYGOMATIC", "NASAL", "LACRIMAL",
                                            "PALATINE", "INFERIOR_NASAL_CONCHA") for s in ("R", "L")}
        self.assertEqual((len(cranial), len(facial)), (8, 14))
        self.assertEqual(set(self.bones), cranial | facial)
        missing = facial - {"BONE_MANDIBLE"}
        audit = self.model["bone_equivalence"]
        self.assertEqual(set(audit["missing_source_records"]), missing)
        self.assertEqual(audit["source_records_present"], 193)
        self.assertEqual(audit["missing_skull_bones"], [])
        self.assertEqual((audit["individual_bone_proxies"], audit["grouped_bones"], audit["unrepresented_bones"]), (199, 0, 7))
        for key, p in self.bones.items():
            with self.subTest(bone=key):
                self.assertEqual(p["role"], "bone_proxy")
                self.assertEqual(p["source_record_status"], "missing" if key in missing else "present")
                if key in missing:
                    self.assertIsNone(p["source_record"])
                else:
                    self.assertTrue((sm.ROOT / p["source_record"]).is_file())
        self.assertFalse(any(p.get("grouped_bone_ids") for p in self.model["parts"]))

    def test_previous_177_individual_proxies_are_unchanged(self):
        bones = [p for p in self.model["parts"] if p["role"] == "bone_proxy" and p["region"] != "head"]
        self.assertEqual(len(bones), 177)
        # Captured from merged main b96dfc2 before adding the skull overlay.
        self.assertEqual(sm.digest(bones), "44debc67ab8391f2efc9fa982d50dfa719f8d9e87c6e708b38d6cee306fdcce7")

    def test_coarse_vault_septum_face_and_one_adult_mandible_across_overlays(self):
        for height, pose in ((None, 0), (1676.4, 45), (1800, 80)):
            model = sm.build_armature(height_mm=height, arm_drop_deg=pose)
            bones = {p["bone_id"]: p for p in model["parts"] if p.get("skull_layout_sha256")}
            center = lambda c: bones["BONE_"+c]["center_mm"]
            with self.subTest(height=height, pose=pose):
                self.assertGreater(center("FRONTAL")[1], 0)
                self.assertLess(center("OCCIPITAL")[1], 0)
                self.assertGreater(center("ETHMOID")[2], center("VOMER")[2])
                self.assertGreater(center("NASAL_R")[2], center("MAXILLA_R")[2])
                self.assertLess(center("PALATINE_R")[1], center("MAXILLA_R")[1])
                self.assertLess(center("LACRIMAL_R")[0], center("ZYGOMATIC_R")[0])
                self.assertGreater(bones["BONE_PAR_R"]["bounds_mm"]["min"][2], bones["BONE_TEMP_R"]["bounds_mm"]["max"][2])
                jaw = bones["BONE_MANDIBLE"]
                points = jaw["centerline_mm"]
                self.assertEqual(jaw["articulates_with"], ["BONE_TEMP_R", "BONE_TEMP_L"])
                self.assertEqual(jaw["joint_stations_mm"], [points[0], points[-1]])
                self.assertFalse(jaw["closed_centerline"])
                self.assertFalse(jaw["motion_implemented"])
                self.assertEqual(points[len(points)//2][0], 0)
                self.assertLess(points[len(points)//2][2], center("MAXILLA_R")[2])
                self.assertGreater(points[0][2], points[len(points)//2][2])
                self.assertLess(points[0][1], points[len(points)//2][1])
                for a, b in zip(points, reversed(points)):
                    for v, w in zip([-a[0], a[1], a[2]], b):
                        self.assertAlmostEqual(v, w)
                self.assertEqual([p["bone_id"] for p in bones.values() if "MANDIBLE" in p["bone_id"]], ["BONE_MANDIBLE"])

    def test_selected_adjacency_is_reciprocal_connected_and_reflected(self):
        graph = {key: set(p["topology_neighbors"]) for key, p in self.bones.items()}
        for key, neighbors in graph.items():
            self.assertTrue(neighbors <= set(graph))
            self.assertNotIn(key, neighbors)
            for other in neighbors:
                self.assertIn(key, graph[other])
        seen, pending = set(), ["BONE_FRONTAL"]
        while pending:
            key = pending.pop()
            if key not in seen:
                seen.add(key)
                pending.extend(graph[key]-seen)
        self.assertEqual(seen, set(graph))
        for key, right in self.bones.items():
            if not key.endswith("_R"):
                continue
            left = self.bones[key[:-1]+"L"]
            self.assertEqual(left["mirror_of"], right["id"])
            self.assertEqual(left["skull_frame_side"], "L")
            self.assertEqual(left["vertices"], [[-v[0], v[1], v[2]] for v in right["vertices"]])
            self.assertEqual(left["faces"], [[a, c, b] for a, b, c in right["faces"]])
            self.assertGreater(min(v[0] for v in right["vertices"]), 0)
            self.assertLess(max(v[0] for v in left["vertices"]), 0)
            if right["source_record"]:
                self.assertNotEqual(left["source_record"], right["source_record"])

    def test_all_skull_meshes_are_connected_closed_oriented_and_positive_volume(self):
        for key, p in self.bones.items():
            with self.subTest(bone=key):
                directed = Counter((f[i], f[(i+1)%3]) for f in p["faces"] for i in range(3))
                self.assertTrue(all(n == 1 and directed[(b, a)] == 1 for (a, b), n in directed.items()))
                self.assertEqual(len(p["vertices"])-len(directed)//2+len(p["faces"]), 2)
                graph = {i: set() for i in range(len(p["vertices"]))}
                volume = 0
                for f in p["faces"]:
                    a, b, c = [p["vertices"][i] for i in f]
                    self.assertTrue(all(math.isfinite(v) for point in (a, b, c) for v in point))
                    self.assertGreater(sum(v*v for v in sm.cross(sm.sub(b, a), sm.sub(c, a))), 1e-10)
                    volume += sum(a[i]*sm.cross(b, c)[i] for i in range(3))/6
                    for i in range(3):
                        graph[f[i]].add(f[(i+1)%3])
                seen, pending = set(), [0]
                while pending:
                    i = pending.pop()
                    if i not in seen:
                        seen.add(i); pending.extend(graph[i]-seen)
                self.assertEqual(len(seen), len(p["vertices"]))
                self.assertGreater(volume, 0)

    def test_scale_and_geometry_input_hash_are_traceable_without_readiness_claims(self):
        model = sm.build_armature(height_mm=1800)
        bones = {p["bone_id"]: p for p in model["parts"] if p.get("skull_layout_sha256")}
        scale = model["scale_from_mean"]
        self.assertEqual(model["skull_proxy_layout_sha256"], sm.digest(model["skull_proxy_layout"]))
        for key, original in self.bones.items():
            p = bones[key]
            self.assertEqual(p["skull_layout_sha256"], model["skull_proxy_layout_sha256"])
            self.assertEqual((p["dimensional_fidelity"], p["physical_evidence"]), ("unverified", "unmeasured"))
            self.assertTrue(any("#/skull_proxy_layout/" in v for v in p["geometry_inputs"]))
            for a, b in zip(p["vertices"], original["vertices"]):
                for v, w in zip(a, b):
                    self.assertAlmostEqual(v, w*scale, delta=2e-5)
        self.assertFalse(model["fabrication_released"])
        self.assertFalse(model["bone_equivalence"]["bone_distribution_complete"])
        self.assertEqual(model["canon_effect"], "none")

    def test_lost_identity_is_detected_with_unchanged_total_mesh_count(self):
        parts = copy.deepcopy(self.model["parts"])
        p = next(p for p in parts if p.get("bone_id") == "BONE_MANDIBLE")
        p.pop("bone_id"); p["role"] = "hardware"
        self.assertEqual(len(parts), len(self.model["parts"]))
        self.assertEqual(bone_audit(parts)["missing_skull_bones"], ["BONE_MANDIBLE"])

    def test_invalid_skull_profiles_fail_with_value_error(self):
        original = sm.read
        mutations = {
            "missing_shell": lambda p: p["shells"].pop("FRONTAL"),
            "extra_bone": lambda p: p["ellipsoids"].update({"TOOTH": p["ellipsoids"]["VOMER"]}),
            "unknown_parameter": lambda p: p.update(unknown=1),
            "revision": lambda p: p.update(revision="SKULL_V2"),
            "authority": lambda p: p.update(authority=None),
            "shell_boolean": lambda p: p.update(shell_thickness_mm=True),
            "shell_nan": lambda p: p.update(shell_inset_mm=float("nan")),
            "shell_consumed": lambda p: p.update(shell_thickness_mm=500),
            "shell_samples": lambda p: p.update(shell_samples=[True, 8]),
            "shell_null": lambda p: p["shells"].update(FRONTAL=None),
            "shell_pole": lambda p: p["shells"]["FRONTAL"].update(polar_deg=[0, 80]),
            "shell_reversed": lambda p: p["shells"]["FRONTAL"].update(azimuth_deg=[140, 40]),
            "shell_asymmetry": lambda p: p["shells"]["FRONTAL"].update(azimuth_deg=[40, 130]),
            "shell_wrong_side": lambda p: p["shells"]["PAR_R"].update(azimuth_deg=[100, 130]),
            "reversed_vault": lambda p: p["shells"]["PAR_R"].update(polar_deg=[90, 110]),
            "center_vector": lambda p: p["ellipsoids"]["ETHMOID"].update(center=[0, 1]),
            "section_string": lambda p: p["ellipsoids"]["NASAL_R"].update(radii=["0.04", 0.06, 0.14]),
            "section_infinite": lambda p: p["ellipsoids"]["VOMER"].update(radii=[float("inf"), 0.21, 0.15]),
            "section_zero": lambda p: p["ellipsoids"]["NASAL_R"].update(radii=[0, 0.06, 0.14]),
            "cross_midline": lambda p: p["ellipsoids"]["NASAL_R"].update(center=[0, 0.86, -0.12]),
            "off_midline": lambda p: p["ellipsoids"]["VOMER"].update(center=[0.02, 0.39, -0.58]),
            "outside_envelope": lambda p: p["ellipsoids"]["VOMER"].update(center=[0, 0.9, -0.58]),
            "reversed_septum": lambda p: p["ellipsoids"]["ETHMOID"].update(center=[0, 0.36, -0.7]),
            "reversed_palate": lambda p: p["ellipsoids"]["PALATINE_R"].update(center=[0.2, 0.8, -0.77]),
            "sweep_null": lambda p: p["sweeps"].update(ZYGOMATIC_R=None),
            "sweep_empty": lambda p: p["sweeps"]["ZYGOMATIC_R"].update(centerline=[]),
            "sweep_radius": lambda p: p["sweeps"]["ZYGOMATIC_R"].update(radius_mm=500),
            "sweep_duplicate": lambda p: p["sweeps"]["ZYGOMATIC_R"]["centerline"].__setitem__(1, p["sweeps"]["ZYGOMATIC_R"]["centerline"][0]),
            "sweep_hairpin": lambda p: p["sweeps"]["ZYGOMATIC_R"]["centerline"].__setitem__(2, p["sweeps"]["ZYGOMATIC_R"]["centerline"][0]),
            "jaw_asymmetric": lambda p: p["sweeps"]["MANDIBLE"]["centerline"].__setitem__(-1, [-0.6, -0.1, -0.33]),
            "jaw_reversed": lambda p: p["sweeps"]["MANDIBLE"]["centerline"].reverse(),
        }
        for label, mutate in mutations.items():
            def altered(path):
                data = original(path)
                if Path(path).name == "f28.json":
                    mutate(data["skull_proxy_layout"])
                return data
            with self.subTest(mutation=label), patch.object(sm, "read", altered), self.assertRaises(ValueError):
                sm.build_armature()


if __name__ == "__main__":
    unittest.main()
