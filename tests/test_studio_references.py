"""Selected reference custody, native frames and explicit opt-in budget."""
import base64
from copy import deepcopy
import json
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from skeleton.visualization.studio_references import load_native_references, ROOT, LOCK_PATH


class NativeReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        value = os.environ.get("PHY_REF_ROOT")
        if not value:
            raise unittest.SkipTest("Explicit PHY_REF_ROOT checkout required for pinned cross-repository checks")
        cls.reference_root = Path(value)
        cls.lock = json.loads((ROOT / LOCK_PATH).read_text())
        cls.models, cls.evidence, cls.previews = load_native_references(cls.reference_root)

    def test_two_small_previews_and_exact_source_lineage(self):
        self.assertEqual([m["id"] for m in self.models], ["NIH_PELVIS", "NIH_SKIN"])
        self.assertEqual([len(m["parts"]) for m in self.models], [14, 1])
        self.assertEqual([m["derivative"]["preview_triangles"] for m in self.models], [19983, 25000])
        self.assertEqual(sum(len(v) for v in self.previews.values()), 824908)
        for model in self.models:
            self.assertEqual(base64.b64decode(model["glb_base64"]), self.previews[model["preview_path"]])
            self.assertIsNone(model["canonical_transform"])
            self.assertFalse(model["fabrication_released"])
            self.assertFalse(model["source"]["anatomically_qualified"])
            self.assertTrue(all("bone_id" not in p for p in model["parts"]))

    def test_native_bounds_meter_to_millimeter_conversion_only(self):
        for model in self.models:
            self.assertEqual(model["frame"], "source world Y-up, unchanged")
            self.assertEqual(model["units"], "m")
            for part in model["parts"]:
                for key in ("min", "max"):
                    self.assertEqual(part["bounds_mm"][key], [v*1000 for v in part["bounds_m"][key]])
        self.assertIsNone(self.evidence["landmark_packet"]["accepted_landmarks"])

    def test_no_original_glb_reads_or_acquisition(self):
        original = Path.read_bytes
        opened = []
        def observe(path):
            if path.suffix == ".glb":
                opened.append(str(path))
            return original(path)
        with patch.object(Path, "read_bytes", observe):
            load_native_references(self.reference_root)
        self.assertEqual(len(opened), 2)
        self.assertTrue(all("/derived/" in p for p in opened))

    def test_corruption_missing_file_and_symlink_escape_rejected(self):
        for kind in ("hash", "length", "missing", "escape"):
            with tempfile.TemporaryDirectory() as folder:
                target = Path(folder)
                for relative in self.lock["files"]:
                    dest = target / relative; dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(self.reference_root / relative, dest)
                preview = target / "assets/nih/derived/pelvis-preview.glb"
                if kind == "hash":
                    raw = bytearray(preview.read_bytes()); raw[-1] ^= 1; preview.write_bytes(raw)
                elif kind == "length":
                    preview.write_bytes(preview.read_bytes()[:-1])
                elif kind == "missing":
                    preview.unlink()
                else:
                    preview.unlink(); preview.symlink_to(self.reference_root / "assets/nih/derived/pelvis-preview.glb")
                with self.subTest(kind=kind), self.assertRaises(ValueError):
                    load_native_references(target)


if __name__ == "__main__":
    unittest.main()
