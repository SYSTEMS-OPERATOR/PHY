# PHY F28 quarter-scale passive maquette

This is a 408.10 mm tall, externally supported **form study**.
It is the first inexpensive physical demonstration. It is not a load-bearing,
motorized or full-scale armature, and does not use the A0 shoulder mechanism.

## Material and cuts

- 2 A3 cut sheets, actual size, 3 mm birch plywood; CUT paths only.
- 4 mm hardwood dowel: cut 391.60 mm, including 6 mm base engagement.
- Soft 1.5 mm copper wire; nominal ten 12 mm pins plus bridge lashing.
- Wood glue for fixed connections. Tape/spacer stock for locating the formers.

The two base plates laminate into a 6 mm base. The support post is at
(x=0, y=-8) mm relative to the body center; all former holes use that datum.
The SVG holes are nominal Ø4.2 and Ø1.6 mm. No kerf correction is applied.
First cut a scrap coupon with those holes, measure stock and dowel, and adjust
the cutter offset before cutting the sheets. Confirm the 50 mm calibration line.

## Assembly

1. Laminate the two base plates, align the holes and glue the post through both.
2. Mark the post heights in manifest.json (former lower faces above base top).
3. Slide solid transverse pelvis and thorax formers onto the post and fix at
   those marks with glue or taped collars. They are solid silhouette panels,
   not scaled replicas of the curved full-body reference ribs.
4. Lash/glue the hip and shoulder bridges to the post at the pin-axis heights
   in BOM.csv. The broad face of each bridge lies in the frontal XZ plane.
5. Connect the two sets of thigh/shank and upper-arm/forearm/hand links with
   copper pins; capture both ends. These are display hinges, not bearings.
   Set the limb axes against assembly-stencil.svg at 100% print scale.
6. Glue ankle links to the foot silhouettes and the head silhouette to the
   upper post. Use spacers/scrap tabs at the bridges as needed for this glued
   model; these attachments carry no functional load.
7. Check height, shoulder/hip spacing and bilateral alignment with a ruler.
   Record observations; no physical build or dimensional inspection is claimed.

Use the rear post to support the display. No free-standing balance, joint torque,
strength, actuator fit or full-scale manufacturability follows from this model.
The maquette uses the refined reference height and A-pose. Other Studio states
do not alter these cut files; regenerate an explicit profile before new cuts.
