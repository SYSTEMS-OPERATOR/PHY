"""Standard-library identity resolution; matching policy is always explicit."""
from collections import Counter, defaultdict

MATCHING_POLICIES = ("exact", "case_insensitive", "id_only")


def describe_reference(value):
    try:
        return repr(value)[:160]
    except (ValueError, OverflowError):
        return f"<{type(value).__name__}>"


class IdentityResolver:
    """Resolve IDs before legacy names, never coerce a reference or guess a side.

    Invalid/duplicate input IDs remain explicit diagnostics. Consumers that
    require a unique identity set must reject ``fatal`` before projecting it.
    ``case_insensitive`` deliberately uses lower(), preserving runtime policy.
    """

    def __init__(self, records):
        ids, invalid = [], []
        self.names = defaultdict(list)
        self.folded_names = defaultdict(list)
        for record in records:
            uid, name = record.get("unique_id"), record.get("name")
            if not isinstance(uid, str) or not uid.strip():
                invalid.append(describe_reference(uid))
                continue
            ids.append(uid)
            if isinstance(name, str):
                self.names[name].append(uid)
                self.folded_names[name.lower()].append(uid)
        self.ids = set(ids)
        self.invalid_ids = sorted(invalid)
        self.duplicate_ids = sorted(uid for uid, n in Counter(ids).items() if n > 1)
        for table in (self.names, self.folded_names):
            for key in table:
                table[key].sort()

    @property
    def fatal(self):
        return bool(self.invalid_ids or self.duplicate_ids)

    def resolve(self, reference, *, policy, optional=False):
        if policy not in MATCHING_POLICIES:
            raise ValueError("Unsupported identity matching policy: " + str(policy))
        result = {"bone_id": None, "reference": reference, "status": "unresolved"}
        if optional and (reference is None or reference == ""):
            return {**result, "status": "unspecified"}
        if not isinstance(reference, str) or not reference.strip():
            return {**result, "reference": describe_reference(reference), "status": "invalid"}
        if reference in self.duplicate_ids:
            return {**result, "status": "ambiguous", "candidates": [reference]}
        if reference in self.ids:
            return {**result, "bone_id": reference, "status": "id"}
        table = self.folded_names if policy == "case_insensitive" else self.names
        key = reference.lower() if policy == "case_insensitive" else reference
        candidates = table.get(key, []) if policy != "id_only" else []
        if len(candidates) == 1:
            return {**result, "bone_id": candidates[0], "status": "unique_display_name"}
        return {**result, "status": "ambiguous" if candidates else "missing",
                "candidates": list(candidates)}


def find_parent_cycles(parents):
    """Follow only declared parents; adjacency and region loops are unrelated."""
    done, cycles = set(), set()
    for start in sorted(parents):
        path, positions, current = [], {}, start
        while current is not None and current not in done:
            if current in positions:
                cycle = path[positions[current]:]
                offset = cycle.index(min(cycle))
                cycle = cycle[offset:] + cycle[:offset]
                cycles.add(tuple(cycle + [cycle[0]]))
                break
            positions[current] = len(path)
            path.append(current)
            current = parents.get(current)
        done.update(path)
    return [list(cycle) for cycle in sorted(cycles)]
