"""Export a synthetic malformed-declaration fixture through the real Studio path."""
import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch
import warnings

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from skeleton.bones import load_field
from skeleton.visualization.studio_evidence import build_studio_evidence


def main(output):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        field = load_field()
    # Existing adult IDs keep the fixture available in the actual inspectors.
    # Canonical records and geometry are unchanged; these are legacy declarations.
    field.bones["BONE_HYOID"].articulations = [None, {"bone": "BONE_MALLEUS_R"}]
    field.bones["BONE_INCUS_L"].articulations = ({"bone": "BONE_INCUS_R"},)
    field.build_entanglement()

    spec = importlib.util.spec_from_file_location("studio_fixture_export", ROOT / "bin/export_phy_studio.py")
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    with patch.object(exporter, "build_studio_evidence", side_effect=lambda parts:
                      build_studio_evidence(parts, field=field)):
        exporter.export(output, include_a0=False)


if __name__ == "__main__":
    main(Path(sys.argv[1]))
