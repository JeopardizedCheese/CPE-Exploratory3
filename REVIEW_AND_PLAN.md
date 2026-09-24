# Review and implementation plan

## Rules extracted from First Project Details.pdf

Source: user-provided PDF, pages 2-6. The fictional story on page 1, including
the rainbow stone, is not an additional tested color class.

| Requirement | Consequence |
| --- | --- |
| Fully assembled robot no larger than A4, 210 x 297 mm | Check CAD footprint including mechanism; PDF gives no separate numerical height limit. |
| Provided two-wheel/two-motor base and ESP32 main controller | Keep motion, timing and mechanism state on ESP32. |
| Optional HuskyLens on request | Potential close-range verification; not required by the PDF. |
| Student-designed laser-cut acrylic and printed plastic parts; two servos provided | Design a simple intake and retention/release mechanism within the footprint. |
| Stones: 50 x 35 x 35 mm; two shapes 37 x 25 x 25 mm | Tune gripper dimensions and vision size checks on actual orientations. |
| Approximately 2100 x 1200 mm arena; supplied overhead USB camera | Calibrate actual floor dimensions and camera geometry. |
| 54 stones, nine each of six colors, piled centrally | Visible detections cannot be assumed to count every stone. |
| Start anywhere in designated start zone | Select and verify robot starting pose; the photo places the zone on the right. |
| Three attempts, five minutes each | ESP32 needs a local run timer and stop state. |
| Round 1 custom gesture remote control; round 2 autonomous; round 3 choice | A keyboard/UDP camera demo does not fulfill manual mode. |
| Correct manual +1; correct autonomous +5; incorrect -1 in both | Prefer abstention and reinspection over low-confidence placement. |
| Mechanical 35%, code 35%, achievement 30% | Allocate development and validation to all three, not vision alone. |

The PDF does not explicitly approve extra arena lights, camera-mounted filters,
floor fiducials, or moving field equipment. Confirm event permission before
altering the shared setup. The scoring zones' exact geometry must be measured;
the included photographs are not calibration data.

## Original findings and implemented response

| Finding | Change |
| --- | --- |
| Four HSV classes only | Explicit missing crimson/sky-blue calibration; all-six readiness gate. |
| Each colored fragment treated as a gem | Separate reference-based silhouette from color voting; fill highlight holes. |
| White stones disappear from color masks | Unknown foreground blocks clearance even without a color label. |
| Zone paint and gems share color | Empty-field reference plus explicit exclusions. |
| Pixel sizes vary with perspective | Four-corner floor mapping and mm-based size/clearance checks. |
| Every frame immediately sent as a target | Consecutive confirmation; immediate removal on disappearance. |
| Center distance ignores stone edges/occluders | Distance from entire object silhouette to other foreground and boundaries. |
| Hardcoded camera controls differ between tools | Shared configured settings applied consistently, with rejected-setting warnings. |
| Receiver can keep obsolete observations | Versioned protocol, sequence check, stop messages and reception timeout. |
| Network credentials in source | Ignored local secrets header and committed placeholder example. |

## Lighting and top-view limitations

White highlights lose hue and saturation. Do not widen every color threshold to
include white: that would make white floor/glare match several classes. Here,
visible colored pixels classify a whole foreground component only when evidence
is adequate. A fully white object remains unknown. Software cannot reconstruct
clipped color or see the underside of an occluded stone from one top view.

Try broad diffuse illumination and reposition glare-producing light sources
where the event setup permits. Lower exposure until colored faces retain detail,
then lock exposure, focus and white balance if the actual camera supports it.
Changing exposure also changes calibration. Compare actual captured frames,
including shadows and motion, rather than assuming a property setter succeeded.
OpenCV documents hardware/backend dependencies for camera controls:
[OpenCV video properties](https://docs.opencv.org/4.12.0/d4/d15/group__videoio__flags__base.html).

If allowed, test a rotatable lens polarizer; with controllable lights, crossed
polarizers on the light and camera may reduce specular glare. Measure the loss
of usable light and retune exposure. This is an optional experiment, not an
assumption about the supplied camera or stone finish.
[Edmund Optics polarization guidance](https://www.edmundoptics.com/knowledge-center/application-notes/illumination/successful-light-polarization-techniques/).

The robot, gripper, hands, pile and walls can hide stones. Large/mixed foreground
is withheld and lost detections are not reused. However, a small visible fragment
or a same-color cluster can still resemble one stone. This pipeline has no depth
sensor or trained instance model. The next physical step is single-stone intake
and close-range verification, plus robot-pose-based dynamic masking. Avoid gem
colors on the robot's visible upper surfaces. A hidden obstacle or an object
indistinguishable from the reference cannot be guaranteed detected.

A floor homography is exact only on its plane (and does not correct lens
distortion). Stone tops are 25-35 mm above the floor, so off-axis localization
has parallax error. For a level camera at height H above the floor, radial floor
projection error is approximately r*h/(H-h), for object height h and radial
distance r from the optical axis. Measure error at the corners and for all stone
heights. Add lens calibration/height compensation or rely on a final close-range
alignment step before grabbing. The current 60 mm clearance is a tunable local
margin, not a navigation clearance for an A4-sized robot.

The reference method assumes a fixed camera and mostly unchanged background.
It deliberately abstains on major changes; small camera shifts or local light
patches may still yield false foreground. Check overlays, recapture only when
the arena is empty, and test under the real lighting. Do not adapt the reference
online while stones sit still, or they may be absorbed into the background.

## Remaining robot implementation sequence

1. Specify actual motor driver, ESP32 board/pins, encoders, servo geometry and
   power supply. Draw/measure the complete 210 x 297 mm envelope.
2. Implement local ESP32 stop, watchdog, five-minute timer and gesture/manual
   input with a dead-man condition. Validate wheels lifted before driving.
3. Measure robot pose (approved marker or other sensor), model its footprint,
   and plan collision-free paths. Vision target coordinates alone are insufficient.
4. Add select -> approach -> reobserve -> single intake -> color verification ->
   matching zone -> release -> verify state transitions. Unknown color means
   retry/reposition, never guess a destination. Handle jams and failed pickup.
5. Train/validate gesture control and conduct complete timed manual/autonomous
   runs. Choose round 3 mode from measured scoring/reliability.

These items are a proposed next implementation plan, not completed features.

## Field acceptance checklist

Record videos and ground-truth labels under bright light, shadow, glare, mixed
light and robot movement. Include all 54 physical stones, sizes and orientations.
For each class report precision/recall, unknown rate, false-target count,
localization error in mm, and end-to-end target age. Keep separate calibration
and evaluation scenes; particularly inspect cyan versus sky blue.

Check white/glared stones near a colored target, touching same/different colors,
hidden stones, gripper overlap, colored zone edges, camera vibration, and stones
near walls. Test unplugged camera, blocked camera reads, Wi-Fi loss, malformed,
duplicate and reordered UDP packets, application restart and empty target lists.
Measure watchdog behavior on ESP32; no successful bench simulation substitutes
for this. Proposed release criteria: no unsafe pickup in the challenge set,
error below the measured intake tolerance, and repeatable complete five-minute
runs. Set numeric accuracy/latency criteria after measuring the actual mechanism.

## Validation performed in this workspace

The final synthetic suite contains 22 test cases. Python module compilation and
CLI help are checked alongside it. The ESP32 sketch was reviewed but not compiled
or flashed: the Arduino toolchain and physical board are not available here.
No camera recording or live arena trial was supplied; HSV values, size limits,
confidence thresholds and clearance require field validation.
