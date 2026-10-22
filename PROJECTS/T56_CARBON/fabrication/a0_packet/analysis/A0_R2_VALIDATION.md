# A0-R2 development validation

This record supports design review, not fabrication release. No physical article,
coupon, independent approval, or measured inspection result is claimed.

- Full local Python suite: 315 passed, 9 skipped, 1162 subtests. Subsequent
  mechanical changes were checked with the focused A0/Studio suite: 40 passed,
  278 subtests. Final stock-envelope wording was checked by repeated export.
- Two A0 exports were byte-identical across 25 issued generated files. Repeated
  Studio packaging was byte-identical across 32 files.
- Fourteen CAD-derived SVG sheets were rendered and visually inspected.
- Offline desktop/mobile Studio smoke and original-pelvis browser checks passed,
  including deterministic exports, rollback, fresh reopen, and no HTTP requests.
- 286 protected canon, skeleton, NIH/review, and maquette files remained
  byte-identical. Mission scope, load-case register, mission envelope, and Studio
  reference lock were unchanged.
- All 64 issued neutral instances have no positive-volume pairwise penetration.
  Seven pitch poses, 35 fixture checks, and five scapular poses were checked.
  These sampled checks do not establish continuous or whole-article clearance.

Seven unresolved legacy placements are explicitly omitted from the issued
subassembly and retained as discrepancies. Bearing fits/internal clearance,
stop compliance and range, friction grounding, remaining interfaces, independent
review, and physical evidence remain release holds. In particular, the assumed
4 mm stop stroke is not established by the pad thickness, and first contact at
the travel limits provides no credited compression allowance within those limits.

GitHub Actions results are reported on the pull request; this local record does
not substitute for CI or physical qualification.
