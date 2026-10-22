# A0-R2 build, inspection and bench-test procedure

Status: **PROCEDURE ISSUED FOR SHOP REVIEW — NO TESTS RECORDED**

This procedure applies only to the guarded, non-load-bearing left-side A0
article. A second competent reviewer must approve drawings and the load path
before stock is cut. Human lifting, body support, fall arrest, unguarded motion
and unsupervised operation are prohibited.

R2 adds a dimensioned cartridge subassembly. Undefined scapular stop/plunger and
straight tether placements are excluded from the issued STEP and remain listed
with their original conflicts in `assembly/interface_check_report.json`. They
must be designed and independently accepted before any whole-article build.

The shaft h6 fit, CN residual clearance, friction coupling/grounding, fastener
locking and stop compliance remain critical holds. In particular, nominal first
contact is at -32/+92 degrees; 4 mm tangential compression would overrun those
limits by 3.82 degrees. The existing impact acceptance criterion is unchanged and
currently unfulfilled. Do not execute BT-05 until a reviewed compliant mechanism
or documented requirement disposition resolves this conflict.

## 1. Hold points

| Hold point | Required record | Release condition |
| --- | --- | --- |
| HP-0 document review | signed drawing/BOM deviation log | no unresolved safety- or interface-critical deviation |
| HP-1 incoming material | mill/lot certificates and purchased-part inspection | material minima, bearing ratings and plunger dimensions verified |
| HP-2 machined parts | completed first-article inspection record | every controlling dimension within tolerance |
| HP-3 dry assembly | torque, alignment, retention and guard record | free manual sweep; both hard stops and all passive retainers functional |
| HP-4 guarded proof | raw load/displacement record | LC-SHO-001/003 and LC-MNT-001 criteria pass |
| HP-5 energy tests | raw impact and power-loss record | LC-SHO-004/005 criteria pass; post-test inspection clean |
| HP-6 endurance | cycle, acoustic and thermal record | LC-SHO-002 criteria pass; post-test teardown accepted |

No later hold point cures a failed earlier hold point. Stop, tag the article and
open a deviation/anomaly record after any crack, loss of retention, unexpected
motion, permanent set, datum shift or instrumentation fault.

## 2. Fabrication

1. Confirm `manifest.json` source hash matches the reviewed mechanism JSON and
   confirm the reviewed neutral assembly has the instance count in the current manifest (unresolved envelopes excluded).
2. Manufacture only the reviewed current BOM items; use the part drawings and
   handed-part notes. The BOM also includes unresolved reserved interfaces. Do not machine
   bearing bores or the index-plunger thread until the delivered parts are
   measured and the mating dimensions are confirmed.
3. Machine the base datum hole and face first. Establish the drawing origin from
   that axis and the plate top. Machine all root and bench holes from the same
   setup where practical.
4. Ream link pivots to `10.2 +0.10/-0.00 mm`; hold their pattern to true
   position `0.30 mm`. Match-mark each rocker and its inspected orientation.
5. Finish bearing bores to `40.000..40.025 mm`, shaft journals to `17 h6`, the `5 x 5 mm`
   keyway to the delivered key, hub socket to `25.50 +0.15/-0.00 mm`, and index
   holes to `8.2 +0.10/-0.00 mm`.
6. Cut the square tube to `327 ±0.5 mm`; deburr and break edges 0.2–0.5 mm.
   Reject dents, twist, cracks or straightness over 0.75 mm per 300 mm.
7. Apply the PET barrier and zinc-rich primer at every aluminum/A36 interface.
   Keep bearing seats, friction faces, threads and electrical grounds free of
   coating. Record coating batch and cure.

## 3. First-article inspection

Use calibrated instruments and the supplied inspection template. At minimum:

- verify material/heat/lot trace for every primary part and fastener class;
- inspect base flatness `≤0.30 mm`, origin-hole size, bench pattern and P/A root
  coordinates to `±0.25 mm`;
- inspect rocker/coupler center distances, true position and parallelism;
- record shaft, bearing-bores, index holes and hub socket at three stations;
- record member cut length, section, wall, straightness, as-built mass and center
  of gravity;
- trial-engage every index position with at least 6 mm pin engagement; and
- verify at least 25 mm tool clearance throughout the service envelope.

## 4. Assembly

1. Bolt the isolated base to the rated bench with four M10-10.9 fasteners.
   Project torque is `60 N·m` dry/zinc plated unless the reviewed bench interface
   requires a lower value; record the actual value.
2. Install the four-bar with M10 shoulder bolts, hardened washers and new
   all-metal locknuts. Tighten locknuts only to remove axial play while
   preserving free rotation; target end float is `0.05–0.15 mm`. Do not apply a
   generic clamp-bolt torque through a rotating link.
3. Install the mirrored yokes with M6x20 foot screws and confirm at least 0.5 mm
   underside clearance to the rocker layer. Install SKF 6203-2RSH bearings,
   Ø35-opening C caps (left nose1.95; right nose1.25), keyed 17 mm shaft,
   inner/outer spacers, hub, sector and bolted end washers. Inspect the right
   pocket15 and locating pocket14; verify floating-ring movement in both axial
   directions before tightening any end capture. Measure the entire
   inner stack and select ground shims for 0.05–0.15 mm shaft end play, with
   no bearing preload and free floating-side outer-ring movement. If the stack is
   short, add annular outboard stack shims; if long, add Ø16.8/Ø6.6 shaft-face
   standoffs under end washers. Target0.10 mm end play; accept0.05..0.15 only
   after inspection. Maximum stack shims0.75 / face standoffs0.50; exceeding
   either requires a deviation. Retain at least12 mm end-screw engagement. Approve all
   thread engagement, tightening and locking before assembly.
   Install mirrored stop frames, shouldered crossbars, end washers/bolts and
   recessed pads; record actual pad contact angles and remaining backed travel.
   Supplier plunger and friction attachments must be independently closed. Torque M6-10.9 hub pinch
   bolts to provisional `12 N·m`; witness-mark after torque.
4. Set the dry friction stack to a measured breakaway torque of at least `7 N·m`
   over three trials. Lock and witness-mark the adjuster.
5. Install the 6 mm steel tether with rated thimbles/swages. At every commanded
   pose it must remain slack during normal retention and prevent detached-member
   travel from reaching the guard.
6. Install the 327 mm dummy member to the 28 mm shoulder insertion mark. Verify the
   nominal S-to-E station is `317 mm` and output position is within the stated
   stack; the shoulder/distal offsets are 40/25 mm and the distal insertion is
   47 mm. Confirm the nominal 3.5 mm
   tube-to-shaft ligament before tightening the hub.
   Fit full guarding before applying power or proof load.

## 5. Bench tests

Use remote enable, an accessible emergency stop, a rigid exclusion barrier and
instrumentation with current calibration. Test one hazard at a time in this
order; do not proceed on a failed criterion.

### BT-01 datum and manual sweep

Record S and E at neutral and both scapular stops. Slowly sweep the pitch stage
from -30 to +90 degrees. Verify physical stops at -32/+92 degrees, every index
hole, no collision with the three registered exclusion/service volumes, and no
software dependency for either stop.

### BT-02 static gravity/proof — LC-SHO-001

Install the measured 2.5 kg member-equivalent mass and 0.5 kg payload. Record
deflection at service load, then apply 2× gravity-equivalent proof load for 60 s
at each critical pose. Pass only if elastic deflection is ≤2.0 mm, permanent set
after unload is ≤0.25 mm and inspection finds no slip, crack or retainer change.

### BT-03 directional push/pull — LC-SHO-003

At the E station apply ±100 N in fore/aft and lateral directions, then ±200 N
proof for 60 s. A separate guarded retention test may reach ±300 N. Pass only if
permanent set and fixture datum shift are each ≤0.25 mm and every primary and
secondary retainer remains engaged.

### BT-04 maintenance — LC-MNT-001

With the article supported, perform removal, inspection, reinstallation and
re-zeroing while applying up to 100 N service force. Pass only if root datum
shift is ≤0.25 mm, restored S position error ≤0.5 mm, pitch-zero error ≤0.5
degree, fastener preload change ≤10%, tool clearance ≥25 mm and elapsed service
time ≤30 minutes.

### BT-05 impact and stops — LC-SHO-004

Use a calibrated pendulum/drop fixture to deliver 2.0 J, three times in each
approved direction, behind the exclusion barrier. Pass only if usable stop-pad
travel is ≥4 mm, post-impact alignment/zero shifts are ≤0.5 mm/0.5 degree,
rebound is ≤10 degrees, all retainers remain engaged and teardown finds no crack,
hole elongation, bearing brinelling or hidden root damage.

### BT-06 power loss and single fault — LC-SHO-005

At the critical poses and maximum specified mass/payload, remove actuator power.
Normal retention must limit drop to 2 degrees. Then, with the friction stack
deliberately defeated behind the barrier, confirm the index pin catches within
15 degrees, catch energy remains ≤2.0 J, peak speed ≤300 degrees/s and safe-state
detection is ≤100 ms. Finally verify the ≥1 kN tether prevents detachment after
the primary index is separately defeated. Never defeat two retainers without a
positive external support.

### BT-07 motion, endurance, thermal and acoustic — LC-SHO-002

Command no more than 20 degrees/s and 40 degrees/s². Verify ordinary-stop energy
≤0.25 J, hysteresis ≤1 degree, temperature rise ≤20 °C and sound ≤50 dBA at 1 m
under the recorded room/background method. Complete 10,000 cycles with interval
inspections. Teardown must find no crack, fretting, bearing damage, stop damage,
fastener rotation or retention loss.

## 6. Closeout

Attach raw data, calibration certificates, photographs, material certificates,
deviations and signed pass/fail dispositions to the immutable test record. Only
then may a reviewer change a load case or evidence package to `approved`. The
checked-in templates intentionally contain null measurements and `not_run`
results; they are not evidence of a test.
