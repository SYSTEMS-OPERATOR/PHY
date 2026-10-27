"""Connection identity must survive real source-module discovery and loading."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import warnings

from skeleton.bones import load_bones, load_field


class BoneLoadingConnectionTest(unittest.TestCase):
    def load_fixture(self, definitions, loader):
        """Execute temporary bone modules through the production discovery path."""
        with tempfile.TemporaryDirectory() as directory:
            files = []
            for index, (uid, name, references) in enumerate(definitions):
                values = dict(
                    unique_id=uid, name=name, bone_type="long", location={"region": "test"},
                    articulations=[{"bone": reference} for reference in references],
                    dimensions={}, function=[], notable_features=[],
                    developmental_notes="synthetic loading fixture", variations="",
                )
                path = Path(directory) / f"bone_{index:02d}.py"
                path.write_text("from skeleton.base import BoneSpec\n"
                                f"bone = BoneSpec(**{values!r})\n", encoding="utf-8")
                files.append(path)
            with patch("skeleton.bones.Path.glob", return_value=iter(files)), \
                    patch("skeleton.bones.load_dataset", return_value={}), warnings.catch_warnings():
                warnings.simplefilter("ignore")
                result = loader()
        bones = result.bones.values() if loader is load_field else result
        return {bone.unique_id: bone for bone in bones}

    def test_real_source_connections_are_consistent_between_loading_apis(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            bones, field = load_bones(), load_field()
        links = {bone.unique_id: set(bone.entanglement_links) for bone in bones}
        self.assertIn("BONE_MANDIBLE", links["BONE_TEMP_L"])
        self.assertIn("BONE_TEMP_L", links["BONE_MANDIBLE"])
        self.assertEqual(links, {bone.unique_id: set(bone.entanglement_links)
                                 for bone in field.bones.values()})
        self.assertEqual(len(bones), 206)

    def test_exact_ids_wire_before_the_bones_leave_either_loader(self):
        definitions = [("TARGET", "Target", ()), ("SOURCE", "Source", ("TARGET",))]
        for loader in (load_bones, load_field):
            with self.subTest(loader=loader.__name__):
                bones = self.load_fixture(definitions, loader)
                self.assertEqual(bones["SOURCE"].entanglement_links, ["TARGET"])
                self.assertEqual(bones["TARGET"].entanglement_links, ["SOURCE"])

    def test_ambiguous_names_do_not_leave_guessed_links_in_either_loading_path(self):
        for loader in (load_bones, load_field):
            for order in (("L", "R"), ("R", "L")):
                with self.subTest(loader=loader.__name__, order=order):
                    definitions = [(f"FEMUR_{side}", "Femur", ()) for side in order]
                    definitions.append(("SOURCE", "Source", ("Femur",)))
                    bones = self.load_fixture(definitions, loader)
                    self.assertTrue(all(not bone.entanglement_links for bone in bones.values()))

    def test_exact_id_wins_over_a_display_name_collision_in_both_loaders(self):
        definitions = [("Femur", "Selected target", ()), ("FEMUR_L", "Femur", ()),
                       ("FEMUR_R", "Femur", ()), ("SOURCE", "Source", ("Femur",))]
        for loader in (load_bones, load_field):
            with self.subTest(loader=loader.__name__):
                bones = self.load_fixture(definitions, loader)
                self.assertEqual(bones["SOURCE"].entanglement_links, ["Femur"])
                self.assertEqual(bones["Femur"].entanglement_links, ["SOURCE"])
                self.assertEqual(bones["FEMUR_L"].entanglement_links, [])
                self.assertEqual(bones["FEMUR_R"].entanglement_links, [])

    def test_nontext_targets_reach_both_loaders_without_blocking_valid_neighbors(self):
        for loader in (load_bones, load_field):
            for invalid in (None, 7, ["TARGET"], {"id": "TARGET"}):
                with self.subTest(loader=loader.__name__, reference=invalid):
                    definitions = [("TARGET", "Target", ()),
                                   ("SOURCE", "Source", (invalid, "TARGET"))]
                    bones = self.load_fixture(definitions, loader)
                    self.assertEqual(bones["SOURCE"].entanglement_links, ["TARGET"])
                    self.assertEqual(bones["SOURCE"].articulations,
                                     [{"bone": invalid}, {"bone": "TARGET"}])


if __name__ == "__main__":
    unittest.main()
