"""Identity, distribution and provenance regressions, independent of mesh count."""
import copy
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from zipfile import ZipFile

from skeleton.visualization import studio_model as sm
from skeleton.visualization.bone_equivalence import bone_audit, expected_bones, source_record_paths
from skeleton.visualization.core_bone_geometry import plate_xz

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("bone_studio_export", ROOT / "bin/export_phy_studio.py")
exporter = importlib.util.module_from_spec(spec); spec.loader.exec_module(exporter)


class BoneEquivalenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = sm.build_armature()
        cls.parts = {p["id"]: p for p in cls.model["parts"]}
        cls.bones = {p["bone_id"]: p for p in cls.model["parts"] if p["role"] == "bone_proxy"}

    def test_adult_inventory_independent_region_counts_and_digit_rules(self):
        rows = expected_bones()
        self.assertEqual(len({r["bone_id"] for r in rows}), 206)
        self.assertEqual(dict(Counter(r["region"] for r in rows)),
                         {"skull": 22, "ear": 6, "hyoid": 1, "spine": 26, "thorax": 25,
                          "shoulder_girdle": 4, "pelvis": 2, "upper_limb": 6,
                          "lower_limb": 8, "hand": 54, "foot": 52})
        ids = {r["bone_id"] for r in rows}
        for side in ("R", "L"):
            for prefix in ("PHAL", "T_PHAL"):
                self.assertIn(f"BONE_{prefix}_1_2_{side}", ids)
                self.assertNotIn(f"BONE_{prefix}_1_3_{side}", ids)
        self.assertFalse(any("TOOTH" in key for key in ids))

    def test_missing_source_records_remain_explicit(self):
        source = source_record_paths()
        expected_missing = {f"BONE_{code}_{side}" for code in
                            ("MAXILLA", "ZYGOMATIC", "NASAL", "LACRIMAL", "PALATINE", "INFERIOR_NASAL_CONCHA")
                            for side in ("R", "L")} | {"BONE_VOMER"}
        self.assertEqual(len(source), 193)
        self.assertEqual(set(self.model["bone_equivalence"]["missing_source_records"]), expected_missing)
        for row in self.model["bone_equivalence"]["bones"]:
            if row["bone_id"] in expected_missing:
                self.assertIsNone(row["source_record"])
                self.assertEqual(row["source_record_status"], "missing")

    def test_core_exact_coverage_and_non_bone_separation(self):
        required = {f"BONE_{p}{i}" for p, n in (("C", 7), ("T", 12), ("L", 5)) for i in range(1, n+1)}
        required |= {"BONE_SACRUM", "BONE_COCCYX", "BONE_STERNUM"}
        required |= {f"BONE_RIB{i}_{side}" for i in range(1, 13) for side in ("R", "L")}
        required |= {f"BONE_{bone}_{side}" for bone in
                     ("CLAVICLE", "SCAPULA", "HIP", "HUMERUS", "RADIUS", "ULNA", "FEMUR", "PATELLA", "TIBIA", "FIBULA")
                     for side in ("R", "L")}
        self.assertEqual(set(self.bones), required)
        audit = self.model["bone_equivalence"]
        self.assertEqual((audit["individual_bone_proxies"], audit["grouped_bones"], audit["unrepresented_bones"]), (71, 128, 7))
        self.assertEqual(audit["mesh_roles"], {"bone_proxy": 71, "envelope": 2, "form": 29, "hardware": 12, "support": 5})
        self.assertEqual(audit["missing_core_bones"], [])
        self.assertEqual(self.parts["knee_joint_R"]["role"], "hardware")
        self.assertEqual(self.parts["shoulder_bridge"]["role"], "support")
        self.assertFalse(audit["bone_distribution_complete"])

    def test_grouped_hand_foot_skull_are_not_atomic_coverage(self):
        rows = self.model["bone_equivalence"]["bones"]
        grouped = [r for r in rows if r["representation"] == "grouped_form_proxy"]
        self.assertEqual(Counter(r["region"] for r in grouped), {"skull": 22, "hand": 54, "foot": 52})
        self.assertEqual(Counter(r["region"] for r in rows if r["representation"] == "unrepresented"), {"ear": 6, "hyoid": 1})
        self.assertEqual(len(self.parts["palm_R"]["grouped_bone_ids"]), 13)
        self.assertEqual(self.parts["finger_R_1"]["grouped_bone_ids"], ["BONE_PHAL_2_1_R", "BONE_PHAL_2_2_R"])

    def test_spine_levels_ordered_and_individual(self):
        ids = [f"BONE_{p}{i}" for p, n in (("C", 7), ("T", 12), ("L", 5)) for i in range(1, n+1)]
        ids += ["BONE_SACRUM", "BONE_COCCYX"]
        centers = [self.bones[key]["center_mm"] for key in ids]
        self.assertTrue(all(a[2] > b[2] for a, b in zip(centers, centers[1:])))
        self.assertTrue(all(v[0] == 0 for v in centers))
        self.assertEqual(len({tuple(v) for v in centers}), 26)
        self.assertNotIn("neck", self.parts)
        self.assertFalse(any(p.startswith("spine_") for p in self.parts))

    def test_ribs_have_side_level_open_end_and_correct_topology(self):
        for side, sign in (("R", 1), ("L", -1)):
            for level in range(1, 13):
                p = self.bones[f"BONE_RIB{level}_{side}"]
                points = p["centerline_mm"]
                anchor = self.model["bone_anchors_mm"][f"BONE_T{level}"]
                self.assertEqual(p["posterior_attachment"], f"BONE_T{level}")
                self.assertAlmostEqual(points[0][2], anchor[2])
                self.assertAlmostEqual(points[0][1], anchor[1])
                self.assertGreater(min(sign*v[0] for v in points), 0)
                self.assertFalse(p["closed_centerline"])
                self.assertNotEqual(points[0], points[-1])
                self.assertEqual(p["anterior_attachment"], "via_costal_cartilage_to_sternum" if level <= 7 else "via_costal_margin" if level <= 10 else "free")
                self.assertIn("cartilage and joint interfaces are not modeled", p["attachment_status"])

    def test_radius_ulna_separate_and_patella_anterior(self):
        for side, sign in (("R", 1), ("L", -1)):
            r, u = [self.bones[f"BONE_{key}_{side}"]["endpoints_mm"] for key in ("RADIUS", "ULNA")]
            centers = [self.model["landmarks"][f"{key}_{side}"] for key in ("elbow", "wrist")]
            for i in range(2):
                self.assertGreater(sign*(r[i][0]-u[i][0]), 0)
                self.assertAlmostEqual(math.dist(r[i], u[i]), self.model["bone_proxy_layout"]["forearm_separation_mm"][i])
                self.assertEqual([(r[i][j]+u[i][j])/2 for j in range(3)], centers[i])
            patella = self.bones[f"BONE_PATELLA_{side}"]
            knee = self.model["landmarks"][f"knee_{side}"]
            self.assertGreater(patella["bounds_mm"]["min"][1], knee[1])
            self.assertNotEqual(patella["id"], f"knee_joint_{side}")
        self.assertNotIn("forearm_R", self.parts)

    def test_girdle_and_hip_have_plate_not_rail_geometry(self):
        for side in ("R", "L"):
            self.assertEqual(len(self.bones[f"BONE_SCAPULA_{side}"]["vertices"]), 6)
            self.assertEqual(len(self.bones[f"BONE_HIP_{side}"]["vertices"]), 12)
        self.assertFalse(any(p.startswith(("pelvis_ring", "pelvis_bridge", "iliac", "scapular")) for p in self.parts))

    def test_left_bones_are_exact_reflections_with_reversed_winding(self):
        for key, p in self.bones.items():
            if key.endswith("_R"):
                left = self.bones[key[:-1]+"L"]
                self.assertEqual(left["mirror_of"], p["id"])
                self.assertEqual(left["vertices"], [[-v[0], v[1], v[2]] for v in p["vertices"]])
                self.assertEqual(left["faces"], [[a, c, b] for a, b, c in p["faces"]])
                for field in ("endpoints_mm", "centerline_mm"):
                    if field in p:
                        self.assertEqual(left[field], [[-v[0], v[1], v[2]] for v in p[field]])

    def test_invalid_correspondence_and_unearned_evidence_fail_closed(self):
        for kind in ("duplicate_id", "duplicate_bone", "unknown_bone", "role", "group_overlap", "hardware_id", "wrong_region", "unearned_evidence", "missing_provenance"):
            parts = copy.deepcopy(self.model["parts"])
            bone = next(p for p in parts if p["role"] == "bone_proxy")
            second = next(p for p in parts if p["role"] == "bone_proxy" and p is not bone)
            if kind == "duplicate_id": second["id"] = bone["id"]
            elif kind == "duplicate_bone": second["bone_id"] = bone["bone_id"]
            elif kind == "unknown_bone": bone["bone_id"] = "BONE_IMAGINARY"
            elif kind == "role": bone["role"] = "unclassified"
            elif kind == "group_overlap": next(p for p in parts if p["role"] == "form")["grouped_bone_ids"] = [bone["bone_id"]]
            elif kind == "hardware_id": next(p for p in parts if p["role"] == "hardware")["bone_id"] = bone["bone_id"]
            elif kind == "wrong_region": bone["region"] = "hands"
            elif kind == "unearned_evidence": bone["physical_evidence"] = "complete"
            else: bone.pop("geometry_inputs")
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                bone_audit(parts)

    def test_removing_a_bone_does_not_hide_behind_total_mesh_count(self):
        parts = copy.deepcopy(self.model["parts"])
        bone = next(p for p in parts if p.get("bone_id") == "BONE_PATELLA_R")
        bone.pop("bone_id"); bone["role"] = "hardware"
        self.assertEqual(len(parts), len(self.model["parts"]))
        self.assertEqual(bone_audit(parts)["missing_core_bones"], ["BONE_PATELLA_R"])

    def test_provisional_parameters_scale_consistently_and_invalid_inputs_fail(self):
        larger = sm.build_armature(height_mm=1800)
        s = larger["scale_from_mean"]
        radius = next(p for p in larger["parts"] if p.get("bone_id") == "BONE_RADIUS_R")
        ulna = next(p for p in larger["parts"] if p.get("bone_id") == "BONE_ULNA_R")
        self.assertAlmostEqual(math.dist(radius["endpoints_mm"][0], ulna["endpoints_mm"][0]), 18*s)
        original = sm.read
        for key, value in (("forearm_separation_mm", [0, 25]), ("rib_drop_mm", [1]),
                           ("patella_radii_mm", [12, float("nan"), 17]), ("c1_below_vertex_mm", 900),
                           ("rib_free_end_angle_deg", 100), ("rib_sweep_samples", 2),
                           ("spine_y_mm", ["unknown"]*8), ("scapula_vertices_relative_shoulder_mm", [[1, 2]]*3)):
            def altered(path):
                d = original(path)
                if Path(path).name == "f28.json": d["bone_proxy_layout"][key] = value
                return d
            with self.subTest(key=key), patch.object(sm, "read", altered), self.assertRaises(ValueError):
                sm.build_armature()
        with self.assertRaisesRegex(ValueError, "strictly convex"):
            plate_xz([[0, 0], [1, 0], [.2, .2], [1, 1], [0, 1]], 0, 1)

    def test_each_bone_is_traceable_and_no_fidelity_or_release_claims(self):
        for row in self.model["bone_equivalence"]["bones"]:
            self.assertEqual((row["dimensional_fidelity"], row["physical_evidence"]), ("unverified", "unmeasured"))
            if row["representation"] == "individual_project_proxy":
                self.assertTrue(row["geometry_inputs"])
        self.assertEqual(self.model["bone_proxy_layout_sha256"], sm.digest(self.model["bone_proxy_layout"]))
        self.assertEqual(self.model["canon_effect"], "none")
        self.assertFalse(self.model["fabrication_released"])
        self.assertFalse(self.model["physical_evidence_complete"])
        canon_overlay = sm.build_armature(height_mm=1676.4, span_equals_height=True)
        self.assertEqual((canon_overlay["height_mm"], canon_overlay["t_pose_span_mm"]), (1676.4, 1676.4))

    def test_repeated_build_is_exact_and_maquette_stays_separate_unchanged(self):
        self.assertEqual(sm.digest(self.model), sm.digest(sm.build_armature()))
        with tempfile.TemporaryDirectory() as tmp:
            result = exporter.maquette(self.model, Path(tmp))
            self.assertEqual(result["cut_parts"], 26)
            baseline = ROOT / "PROJECTS/PHY_F28/maquette"
            files = [p for p in baseline.iterdir() if p.is_file()]
            self.assertEqual({p.name for p in files}, {p.name for p in Path(tmp).iterdir()})
            for p in files:
                self.assertEqual(p.read_bytes(), (Path(tmp) / p.name).read_bytes(), p.name)

    def test_arm_stations_do_not_depend_on_runtime_float_sum(self):
        # Emulate pre-3.12 left-to-right accumulation so a reversion to sum()
        # fails locally on 3.12 too, not only on the 3.11 CI runner.
        def sequential_sum(values, start=0):
            result = start
            for value in values:
                result += value
            return result

        with patch.object(sm, "sum", sequential_sum, create=True):
            legacy_runtime_model = sm.build_armature()
        self.assertEqual(legacy_runtime_model["design_datums"], self.model["design_datums"])
        self.assertEqual(legacy_runtime_model["maquette_members"], self.model["maquette_members"])

    def test_standard_library_package_is_complete_and_repeatable(self):
        with tempfile.TemporaryDirectory() as tmp:
            one, two = Path(tmp) / "one", Path(tmp) / "two"
            exporter.export(one, include_a0=False)
            exporter.export(two, include_a0=False)
            names = {str(p.relative_to(one)) for p in one.rglob("*") if p.is_file()}
            self.assertEqual(names, {str(p.relative_to(two)) for p in two.rglob("*") if p.is_file()})
            for name in names:
                self.assertEqual((one / name).read_bytes(), (two / name).read_bytes(), name)
            with ZipFile(one / "PHY-Studio-package.zip") as archive:
                self.assertIsNone(archive.testzip())
                self.assertIn("BONE_EQUIVALENCE.json", archive.namelist())
                self.assertIn("CORE_PROXY_LAYOUT.md", archive.namelist())
                self.assertEqual(len(json.loads(archive.read("BONE_EQUIVALENCE.json"))["bones"]), 206)


if __name__ == "__main__":
    unittest.main()
