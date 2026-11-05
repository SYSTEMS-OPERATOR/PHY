"""Connection identity must survive real source-module discovery and loading."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import warnings

from skeleton.bones import load_bones, load_field
from skeleton.field import SkeletonField
from skeleton.relationships import build_relationship_snapshot


class BoneLoadingConnectionTest(unittest.TestCase):
    def load_fixture(self, definitions, loader, *, overrides=None):
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
                values.update((overrides or {}).get(uid, {}))
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

    def test_malformed_shapes_survive_loading_and_connection_derivation(self):
        definitions = [("SOURCE", "Source", ()), ("TARGET", "Target", ()),
                       ("DECOY", "Decoy", ())]
        shapes = (None, [None, {"bone": "TARGET"}], ({"bone": "DECOY"},))
        canonical = {"parent": "DECOY", "children": ["DECOY"]}
        for loader in (load_bones, load_field):
            for shape in shapes:
                for supplied in ({}, {"connections": {}}, {"connections": canonical}):
                    with self.subTest(loader=loader.__name__, shape=shape, supplied=supplied):
                        bones = self.load_fixture(definitions, loader, overrides={
                            "SOURCE": {"articulations": shape, **supplied}})
                        source = bones["SOURCE"]
                        targets = ["TARGET"] if isinstance(shape, list) else []
                        self.assertEqual(source.entanglement_links, targets)
                        self.assertEqual(bones["TARGET"].entanglement_links, ["SOURCE"] if targets else [])
                        self.assertEqual(bones["DECOY"].entanglement_links, [])
                        self.assertEqual(source.articulations, shape)
                        self.assertEqual(source.connections, canonical if supplied.get("connections") else
                                         {"parent": None, "children": targets})

                        # Register without rewiring: inspect what the loader returned.
                        field = SkeletonField()
                        for bone in bones.values():
                            field.register(bone)
                        graph = build_relationship_snapshot(field)
                        path = "articulations[0]" if isinstance(shape, list) else "articulations"
                        row = next(r for r in graph["relationships"]
                                   if r["relationship_id"] == f"REL:SOURCE:{path}")
                        self.assertEqual(row["resolution_status"], "invalid")
                        self.assertIsNone(row["participants"][1]["bone_id"])
                        self.assertEqual(row["original_declaration"],
                                         None if shape is None or isinstance(shape, list) else
                                         {"invalid_type": "builtins.tuple"})


if __name__ == "__main__":
    unittest.main()
