from docs.compliance_doc_gen import ComplianceDocGen
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class TCFGenerationTest(unittest.TestCase):
    def test_tcf_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            risk, log, out = root / "risk.yaml", root / "log.json", root / "out.pdf"
            risk.write_text("risk: low")
            log.write_text(json.dumps({"ok": True}))
            ComplianceDocGen(risk, log, out).generate()
            self.assertTrue(out.exists())


class TCFCLITest(unittest.TestCase):
    """The CLI must preserve evidence, including failures and missing inputs."""

    script = Path(__file__).resolve().parents[1] / "bin" / "generate_tcf.py"

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.risk = self.root / "risk.yaml"
        self.log = self.root / "test_log.json"
        self.output = self.root / "TCF.pdf"

    def run_cli(self, *arguments):
        return subprocess.run(
            [sys.executable, str(self.script), *arguments],
            cwd=self.root, capture_output=True, text=True, timeout=15,
        )

    def read_packet(self):
        # The existing generator is a stub, not a validated PDF renderer.
        return json.loads(self.output.read_text().split("\n", 1)[1])

    def test_supplied_evidence_is_preserved_and_failed_tests_are_not_replaced(self):
        risk = "risk: unresolved\nnotes: independent review pending\n"
        log = {"tests": [{"id": "retention", "status": "failed"}]}
        encoded_log = json.dumps(log, indent=4) + "\n"
        self.risk.write_text(risk)
        self.log.write_text(encoded_log)

        result = self.run_cli()

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.risk.read_text(), risk)
        self.assertEqual(self.log.read_text(), encoded_log)
        self.assertEqual(self.read_packet(), {"risk": risk, "tests": log})

    def test_missing_evidence_fails_without_creating_inputs_or_replacing_output(self):
        for present in ((), ("risk",), ("log",)):
            with self.subTest(present=present):
                self.risk.unlink(missing_ok=True)
                self.log.unlink(missing_ok=True)
                if "risk" in present:
                    self.risk.write_text("risk: unresolved\n")
                if "log" in present:
                    self.log.write_text('{"tests": []}\n')
                self.output.write_text("previous packet")
                before = {p: p.read_bytes() for p in self.root.iterdir()}

                result = self.run_cli()

                self.assertNotEqual(result.returncode, 0)
                self.assertIn("missing", result.stderr.lower())
                self.assertEqual({p: p.read_bytes() for p in self.root.iterdir()}, before)

    def test_invalid_test_json_is_not_replaced_with_success(self):
        self.risk.write_text("risk: unresolved\n")
        self.log.write_text("{unfinished")
        self.output.write_text("previous packet")
        before = {p: p.read_bytes() for p in self.root.iterdir()}

        self.assertNotEqual(self.run_cli().returncode, 0)

        self.assertEqual({p: p.read_bytes() for p in self.root.iterdir()}, before)

    def test_smoke_mode_uses_explicit_synthetic_inputs_without_claiming_test_success(self):
        self.risk.write_text("risk: unresolved\n")
        self.log.write_text('{"tests": [{"status": "failed"}]}\n')
        before = {p: p.read_bytes() for p in (self.risk, self.log)}

        result = self.run_cli("--smoke")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual({p: p.read_bytes() for p in before}, before)
        packet = self.read_packet()
        self.assertIn("risk: unassessed", packet["risk"])
        self.assertIn("synthetic_smoke_fixture", packet["risk"])
        self.assertEqual(packet["tests"], {
            "evidence_kind": "synthetic_smoke_fixture", "tests": "not_run",
        })

    def test_smoke_mode_does_not_leave_synthetic_evidence_in_working_directory(self):
        result = self.run_cli("--smoke")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(set(self.root.iterdir()), {self.output})
