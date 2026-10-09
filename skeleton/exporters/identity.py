"""Resolve exported identities without guessing sides or mechanical topology."""
from collections import Counter, defaultdict
from typing import Any, Dict, List

from skeleton.validation.validator_agent import describe


class ExportIdentity:
    def __init__(self, records: List[Dict[str, Any]]) -> None:
        self.report: Dict[str, Any] = {
            "scope": "identity_and_declared_anatomical_parent_references",
            "invalid_ids": [], "duplicate_ids": [], "unresolved_references": [],
            "parent_cycles": [], "ready": False,
            "mechanical_transforms_validated": False,
        }
        ids = []
        self.names = defaultdict(list)
        for record in records:
            uid = record.get("unique_id")
            if not isinstance(uid, str) or not uid.strip():
                self.report["invalid_ids"].append(describe(uid))
            else:
                ids.append(uid)
                if isinstance(record.get("name"), str):
                    self.names[record["name"]].append(uid)
        self.report["duplicate_ids"] = sorted(uid for uid, count in Counter(ids).items() if count > 1)
        self.ids = set(ids)
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
        result = {"bone_id": None, "reference": reference, "status": "unresolved"}
        if optional and reference in (None, ""):
            result["status"] = "unspecified"
            return result
        if not isinstance(reference, str) or not reference.strip():
            result.update(reference=describe(reference), status="invalid")
        elif reference in self.ids:
            result.update(bone_id=reference, status="id")
            return result
        else:
            candidates = sorted(self.names.get(reference, []))
            if len(candidates) == 1:
                result.update(bone_id=candidates[0], status="unique_display_name")
                return result
            result["status"] = "ambiguous" if candidates else "missing"
            result["candidates"] = candidates
        self.report["unresolved_references"].append({**result, "bone_id": uid, "field": field})
        return result

    def _check_cycles(self) -> None:
        # Follow only the one declared parent per record. Anatomical children
        # are adjacency references, not additional transform-tree parents.
        done = set()
        cycles = set()
        for start in sorted(self.ids):
            path = []
            positions = {}
            current = start
            while current is not None and current not in done:
                if current in positions:
                    cycle = path[positions[current]:]
                    offset = cycle.index(min(cycle))
                    cycle = cycle[offset:] + cycle[:offset]
                    cycles.add(tuple(cycle + [cycle[0]]))
                    break
                positions[current] = len(path)
                path.append(current)
                current = self.parents[current]
            done.update(path)
        self.report["parent_cycles"] = [list(cycle) for cycle in sorted(cycles)]
