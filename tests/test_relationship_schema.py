"""Validate the additive artifact contract, including its null qualification boundary."""
from copy import deepcopy
import json
from pathlib import Path
import unittest
import warnings

import jsonschema

from skeleton.bones import load_field
from skeleton.field import SkeletonField
from skeleton.relationships import build_relationship_snapshot


class RelationshipSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / "skeleton/schema/relationship_graph.schema.json"
        schema = json.loads(path.read_text())
        jsonschema.Draft202012Validator.check_schema(schema)
        cls.validator = jsonschema.Draft202012Validator(schema)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cls.graph = build_relationship_snapshot(load_field())

    def test_full_and_empty_partial_snapshots_validate(self):
        self.validator.validate(self.graph)
        self.validator.validate(build_relationship_snapshot(SkeletonField()))
        ids = [r["relationship_id"] for r in self.graph["relationships"]]
        self.assertEqual(len(ids), len(set(ids)))

    def test_contract_refuses_adoption_and_extra_adult_counts(self):
        for change in ("engineering", "kind", "bones", "count", "policy"):
            with self.subTest(change=change):
                graph = deepcopy(self.graph)
                if change == "engineering":
                    graph["relationships"][0]["engineering_definition"] = {"axis": [1, 0, 0]}
                elif change == "kind":
                    graph["relationships"][0]["kind"] = "mechanical_joint"
                elif change == "bones":
                    graph["bones"].append({"bone_id": "REGION_ORBIT", "record_status": "present"})
                elif change == "count":
                    graph["inventory"]["loaded_adult_identities"] = 207
                else:
                    graph["matching_policies"]["rendering"] = "guess"
                self.assertTrue(list(self.validator.iter_errors(graph)))
