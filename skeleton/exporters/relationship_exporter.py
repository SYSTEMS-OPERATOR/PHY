"""Deterministic relationship inspection packages, published manifest last."""
from pathlib import Path

from skeleton.relationships import RELATIONSHIP_VERSION, encode, relationship_markdown, sha256, validate_relationship_snapshot
from .publication import publish_files


def export_relationship_snapshot(graph, output_root):
    validate_relationship_snapshot(graph)
    root = Path(output_root)
    paths = {"graph": root / "relationship_graph.json", "report": root / "RELATIONSHIPS.md",
             "manifest": root / "relationship_manifest.json"}
    payloads = {paths["graph"]: encode(graph), paths["report"]: relationship_markdown(graph).encode("utf-8")}
    manifest = {"package_version": RELATIONSHIP_VERSION,
                "relationship_version": graph["relationship_version"], "inventory": graph["inventory"],
                "inputs": graph["inputs"], "fabrication_released": False,
                "files": {key: {"path": path.name, "sha256": sha256(payloads[path])}
                          for key, path in paths.items() if key != "manifest"}}
    payloads[paths["manifest"]] = encode(manifest)
    publish_files(payloads, paths["manifest"])
    return {key: str(path) for key, path in paths.items()}
