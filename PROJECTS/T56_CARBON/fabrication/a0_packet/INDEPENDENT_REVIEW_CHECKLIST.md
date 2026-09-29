# T56 A0-R1 independent shop-review checklist

Status: **OPEN — REVIEW REQUIRED BEFORE CUTTING STOCK**

Reviewer records name, date, disposition and linked issue/deviation for every
line. “Reviewed” is not “fabrication released” and is never physical evidence.

## Geometry and assembly

- [ ] Confirm `a0_shoulder_mechanism.json#fabrication_geometry` controls CAD,
  drawings, BOM and calculated sections without contradictory literals.
- [ ] Confirm yoke/sector stock is 8/6 mm and the shaft/sector/hub use the same
  Ø17 h6, 5 x 5 keyed interface.
- [ ] Confirm the tube begins 12 mm from S, seats 40 mm from S and leaves at
  least 3.5 mm nominal shaft-to-tube ligament before tolerance review.
- [ ] Confirm root standoffs realize P/A at Z=35 mm and do not make the base
  plate or isolation layer an untracked primary load path.
- [ ] Review bearing shoulders/caps, end capture, key engagement, hub pinch
  bolts, hard-stop brackets, plunger engagement and tether endpoints.
- [ ] Review all declared intentional contacts and the 35 sampled poses. Do not
  treat the centerline/AABB sampling as a continuous swept-solid proof.
- [ ] Confirm guarding, bench interface and minimum 25 mm tool access using the
  actual shop fixture; these external volumes are not closed by the CAD.

## Load path and procurement

- [ ] Resolve the impact bearing screen: 3.25 kN maximum screened reaction is
  greater than the current unverified 3.0 kN per-bearing static threshold.
- [ ] Verify bearing rating, corner radii, shoulder relief and retention against
  the selected supplier drawing before machining yokes.
- [ ] Verify index-plunger thread, pin/nose geometry, engagement and rated load.
- [ ] Obtain or test the stop-pad force/displacement curve; 4 mm usable travel
  is a provisional calculation input, not accepted material evidence.
- [ ] Review combined shaft bending/torsion with 1.6 keyway factor, key shear and
  bearing, yoke net section, root/bench fasteners and stop attachments.
- [ ] Verify friction-stack material and preload/torque correlation, tether
  fittings/rating, materials, heat treatment and fastener lots.

## Document and evidence controls

- [ ] Repeat-export hashes are identical and every part/assembly STEP imports
  as valid geometry.
- [ ] Drawings, BOM, procedures and first-article template all say A0-R1.
- [ ] No supplier assumption, calculation or blank record is marked measured.
- [ ] Canon regression confirms SOPHY 1.0.0, H=span=1676.4 mm and mirror-derived
  symmetry are unchanged.
- [ ] Fabrication release and physical qualification remain false after review;
  a later signed disposition is required to change either state.

## Disposition

Reviewer: ____________________  Date: __________

Disposition: [ ] revise and resubmit  [ ] acceptable for shop quotation only

Linked deviations/issues: _________________________________________________
