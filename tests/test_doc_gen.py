import unittest
from pathlib import Path
from docs.compliance_doc_gen import generate_tcf


class DocGenTest(unittest.TestCase):
    def test_generate(self):
        out = Path("tcf_test.json")
        generate_tcf("risk.yaml", "log.json", out)
        self.assertTrue(out.exists())
        out.unlink()


if __name__ == "__main__":
    unittest.main()
