"""Resolve exported identities without guessing sides or mechanical topology."""
from typing import Any, Dict, List

from skeleton.validation.validator_agent import describe
from skeleton.identity import IdentityResolver, find_parent_cycles


class ExportIdentity:
    def __init__(self, records: List[Dict[str, Any]]) -> None:
        self.report: Dict[str, Any] = {
            "scope": "identity_and_declared_anatomical_parent_references",
            "invalid_ids": [], "duplicate_ids": [], "unresolved_references": [],
            "parent_cycles": [], "ready": False,
            "mechanical_transforms_validated": False,
        }
        self.resolver = IdentityResolver(records)
        self.names, self.ids = self.resolver.names, self.resolver.ids
        self.report["invalid_ids"] = list(self.resolver.invalid_ids)
        self.report["duplicate_ids"] = list(self.resolver.duplicate_ids)
        self.parents: Dict[str, Any] = {}
        self.parent_references: Dict[str, Any] = {}
        self.adjacency: Dict[str, Any] = {}
        self.children: Dict[str, List[str]] = {uid: [] for uid in sorted(self.ids)}
        if self.fatal:
            return
        for record in records:
            uid = record["unique_id"]
            connections = record.get("connections", {})
            parent = self.resolve(connections.get("parent"), uid, "connections.parent", optional=True)
            self.parent_references[uid] = parent
            self.parents[uid] = parent["bone_id"]
            if parent["bone_id"] is not None:
                self.children[parent["bone_id"]].append(uid)
            declared_children = connections.get("children", [])
            if not isinstance(declared_children, list):
                self.report["unresolved_references"].append({
                    "bone_id": uid, "field": "connections.children",
                    "reference": describe(declared_children), "status": "invalid",
                })
                declared_children = []
            self.adjacency[uid] = [
                self.resolve(reference, uid, f"connections.children[{index}]")
                for index, reference in enumerate(declared_children)
            ]
        for children in self.children.values():
            children.sort()
        self._check_cycles()
        self.report["ready"] = not any(self.report[key] for key in (
            "invalid_ids", "duplicate_ids", "unresolved_references", "parent_cycles"))

    @property
    def fatal(self) -> bool:
        return bool(self.report["invalid_ids"] or self.report["duplicate_ids"])

    def resolve(self, reference: Any, uid: str, field: str, optional: bool = False) -> Dict[str, Any]:
        result = self.resolver.resolve(reference, policy="exact", optional=optional)
        if result["bone_id"] is None and result["status"] != "unspecified":
            self.report["unresolved_references"].append({**result, "bone_id": uid, "field": field})
        return result

    def _check_cycles(self) -> None:
        self.report["parent_cycles"] = find_parent_cycles(self.parents)
