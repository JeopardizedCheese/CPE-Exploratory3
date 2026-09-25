# Gemstone sorting: overhead perception and robot link

This improves the camera/ESP32 prototype from `CPE-Exploratory1-main.zip`.
The implemented scope is perception, camera calibration, target communication,
robot pose from an AprilTag, a manual control/recording tool, and ESP32 motor/servo
firmware with local safety stops. This is **not yet a complete autonomous sorting
robot**: the autonomy loop (select -> approach -> grip -> zone -> release), the
gesture remote, and all hardware-dependent tuning are still open. The firmware
has not been compiled or run on hardware.

Robot-side setup, wiring, AprilTag and test order are described in Thai in
[README_ROBOT.md](README_ROBOT.md).

## Run

Use Python 3.12 and a virtual environment. From this project directory:

```powershell
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python sample_hsv.py 1
.venv\Scripts\python calibrate_arena.py 1
.venv\Scripts\python fake_esp32.py
# In a second terminal:
.venv\Scripts\python detect_live.py 1
# To send to the actual ESP32, substitute its address:
.venv\Scripts\python detect_live.py 1 192.168.1.50 4210
```

On Linux/macOS use `.venv/bin/python` instead of `.venv\Scripts\python`.
Every camera tool accepts `--config <path>`; the default is `calib.json` next to the
scripts.

Use the normal GUI OpenCV package in requirements.txt for camera windows.
The workspace `.runtime` directory is an ignored, headless test dependency only.
Press `q` to quit detection, `m` to display foreground segmentation.

| Tool | Purpose |
| --- | --- |
| `sample_hsv.py` | Sample color ranges per class |
| `calibrate_arena.py` | Floor corners, empty-field reference, exclusion zones |
| `detect_live.py` | Live/replayed detection, UDP v2 targets to port 4210 |
| `fake_esp32.py` | Prints v2 target packets received on localhost:4210 |
| `robot_pose.py` | Robot x, y, heading from the roof AprilTag; detection rate |
| `teleop.py` | Keyboard driving over UDP v3 (port 4211) plus run recording |
| `fake_robot.py` | Simulates the `robot_ctrl` firmware for `teleop.py` tests |
| `target_lock.py` | Planner helper: keep one chosen target despite flicker |

## Calibrate in the actual arena

1. Fix the camera, focus and resolution. Use the same lighting for sampling,
   reference capture and operation. Set tested camera properties in
   `camera_properties` (e.g. `AUTO_WB`, `AUTO_EXPOSURE`, `EXPOSURE`). Values are
   backend dependent; an accepted setting is not proof the hardware applied it.
   All tools apply the same configuration. Do not change it mid-run.
2. Run `sample_hsv.py`. Select 1 violet, 2 cyan, 3 crimson, 4 orange/marigold,
   5 sky blue, 6 lime. Click several colored faces of several physical stones
   per class, including the normal shadow range. Avoid white highlights,
   the table and scoring markers. `u` undoes one patch; `s` saves; `q` quits.
3. **Check the tight color pairs by hand** (see below): crimson/violet,
   crimson/orange and cyan/sky blue.
4. Measure the field and adjust `arena.size_mm`; 2100 x 1200 is approximate in
   the PDF. `calibrate_arena.py` freezes an **empty field** with space. Click
   floor corners in TL, TR, BR, BL order, slightly inside the floor edge, and
   press Enter. The long dimension must run horizontally; otherwise rotate the
   camera view or update dimensions. The receiver in `esp_link.ino` also has
   fixed 2100 x 1200 limits; change them if the measured field differs.
5. On the rectified view, outline each of the six colored scoring zones and
   any fixed fixtures, pressing Enter per polygon. Press `s` to save. These are
   perception exclusions: anything inside them is invisible to detection.
   The reference must include the permanent markings but no stones or robot.
6. Place stones, inspect the foreground with `m`, and verify coordinates against
   ruler measurements. Rectified coordinates start at the top-left floor corner:
   x right, y down, in millimeters. Recalibrate after any camera move/settings change.

`calibrate_arena.py` loads the config when it starts and rewrites the whole file
when it saves; do not edit the file by hand while it is running. It writes
`background.png` into the **same folder as the config file**.

Missing background, corner mapping, or color calibration results in
`setup_required` and no transmitted targets. Empty color ranges are never guessed.

### Checking color boundaries

`sample_hsv.py` pads each sampled hue range by +/-6. Under dim or warm light two
classes can then overlap; pixels claimed by two classes vote for neither, so a
stone becomes unknown. Detection prints one line per blob:

```
candidate=crimson area=2712.0 unit=mm2 size_ok=True color_fraction=0.14 dominance=0.97 margin=0.97 votes={1: 12, 2: 0, 3: 393}
```

| Field | Meaning | Default rule |
| --- | --- | --- |
| `color_fraction` | winning votes / blob area | >= 0.3 (`min_color_fraction`) |
| `dominance` | winning votes / all votes | >= 0.85 |
| `margin` | (1st - 2nd) / 1st | >= 0.65 |
| `votes` | voting pixels per class id | only saturated, bright, single-class pixels vote |

`candidate` is printed before the rules are applied. Place each stone **alone**
and read its line. If a stone gets votes for a neighboring class, move the hue
boundary between the two classes by hand in `hsv` (for example violet `hi` 171,
crimson `lo` 172) and recheck both stones. Do not resample to fix an overlap:
that widens the range again. A low `color_fraction` with high `dominance` usually
means shadow in the blob or a saturation minimum that is too strict.

### Home test setup

For tests away from the arena, keep a separate config in its own folder so the
field `background.png` is not overwritten:

```bash
mkdir home
cp calib.json home/calib_home.json
# edit arena.size_mm (e.g. [300, 200] for a 30 x 20 cm sheet), mm_per_px, exclude_polygons: []
python calibrate_arena.py 0 --config home/calib_home.json
python sample_hsv.py 0 --config home/calib_home.json
python detect_live.py 0 127.0.0.1 --config home/calib_home.json
```

Home HSV values and relaxed thresholds do not transfer to the field.

## Values to set before a real run

| Where | Value | Set from |
| --- | --- | --- |
| `calib.json` `arena` | `size_mm`, `corners_px`, `background.png` | `calibrate_arena.py` in the arena, empty field |
| `calib.json` | `exclude_polygons` | outline the six zones in the arena |
| `calib.json` | `hsv` (all six classes) | `sample_hsv.py` under arena light, then the boundary check |
| `calib.json` | `camera_properties` | lock exposure / white balance if the camera allows |
| `calib.json` | `gem_area_mm2`, `vision.max_gem_extent_mm` | measured single stones in the arena |
| `calib.json` `vision` | `pile_mode: true`, `sticky_frames: 5` | can be set now |
| `calib.json` `vision` | `gripper_width_mm`, `approach_length_mm`, `own_radius_mm`, `clearance_mm` | the finished gripper |
| `calib.json` `vision` | `min_color_fraction` (keep 0.3), `background_delta` (try 45 if shadows inflate blobs) | arena test |
| `calib.json` `robot_tag` | `size_mm` (120, measured print), `height_mm`, `grip_offset_mm`, `footprint_mm` | final roof and arm |
| `calib.json` `robot_tag` | `camera_height_mm`, `camera_floor_xy_mm` | measured; recheck if the camera moves |
| `vision.py` | restore `calibrated = set(color_masks) == set(NAMES)` and the size check; remove the per-blob `print` | after all six classes are sampled |
| `firmware/robot_ctrl/config.h` | driver type, motor pins, `L/R_INVERT`, `MAX_DUTY`, `MIN_DUTY`, `ESTOP_PIN` | actual wiring, wheels-lifted test |
| `firmware/robot_ctrl/config.h` | servo pins, `SERVO_MIN/MAX/START_DEG`, `GRIP_*`, `LIFT_*` | calibrating the arm with the `servo` command |
| `firmware/robot_ctrl/config.h` | `RUN_TIME_MS` back to 300000 if shortened for testing | before every match |
| `firmware/esp_link/esp_link.ino` | 2100 x 1200 coordinate limits | measured field size (only if the receiver is used) |
| both sketches | `secrets.h` Wi-Fi of the team hotspot | venue network |

Values in `home/calib_home.json` are for home tests only; do not copy them.

## Implemented changes

- Foreground segmentation against the empty field groups white highlights and
  colored faces into one connected object. Unchanged floor markings disappear.
- Only sufficiently saturated, visible pixels vote for color. Overlapping HSV
  classes, mixed-color objects, and insufficient evidence become unknown (0).
  Confidence is an evidence fraction, **not a calibrated probability**.
- Unknown foreground, large objects, borders, and excluded zones block pickup
  clearance.
- Targets require four consecutive consistent detections; missing objects are
  removed immediately unless `sticky_frames` is set.
- Large overall exposure/foreground changes suppress output. Small additive
  shifts are compensated for segmentation only, not used to invent color.
- Perspective mapping uses physical-area thresholds; sampling handles red hue
  wrap, edge clicks, outliers, and exact patch undo. Modules are import-safe.
- UDP v2 caps output at eight candidates and 10 packets/second. Receiver clears
  targets on empty/stop messages, rejects reordered packets and expires silence
  after 300 ms. Embedded network credentials were replaced with a local header.

### Pile mode and sticky targets (optional, off by default)

The match starts with all stones in one pile. Touching stones merge into one
blob, and a 60 mm all-around clearance leaves almost nothing pickable. With
`vision.pile_mode` enabled, a blob that fails the normal checks gets a second test:

- **Merged blob:** split into single-color regions. Regions that look like one
  stone (area, length) are candidates. A candidate is pickable if a strip as wide
  as the gripper, leading **away from the pile center** (up to +/-67.5 deg), is
  free of foreground, excluded zones and the arena edge.
- **Single known stone with something nearby:** pickable if any direction has
  such a free strip, preferring the side away from the nearest obstacle.

Pickable stones get `approach_deg`: the heading the robot should drive **along
the strip toward the stone** (0 = +x, 90 = +y, same convention as `robot_pose.py`).
It is added to the v2 target; the ESP32 receiver ignores unknown fields.
`detect_live.py` draws it as an arrow. Buried stones and same-color pairs are
not split.

`vision.sticky_frames` keeps a stable target for up to N frames after it fails a
check, unless a different known color appears at its position.

| `vision` option | Default | Meaning |
| --- | --- | --- |
| `pile_mode` | false | Enable edge picking and single-side clearance |
| `gripper_width_mm` | 60 | Width of the free strip (outer gripper width) |
| `approach_length_mm` | 80 | Length of the free strip |
| `approach_start_mm` | 6 | Strip starts this far ahead of the stone center |
| `own_radius_mm` | 25 | Pixels within this radius count as the stone itself |
| `region_min_mm2` / `region_max_mm2` | 150 / 2000 | Voting area of one-stone regions |
| `max_gem_extent_mm` | 70 | Longest side of one-stone regions |
| `max_approach_turn_deg` | 67.5 | Allowed deviation from straight out of the pile |
| `sticky_frames` | 0 | Frames to hold a target after it drops out |

The gripper-related values are placeholders until the arm exists.

### Local deviations to revert before a real run

The working copy of `vision.py` contains two test-time edits:

- `calibrated = any(color_masks)` instead of `set(color_masks) == set(NAMES)`:
  detection runs with some classes unsampled, so a stone of an unsampled class
  can be labeled as the nearest sampled class.
- The size/extent check is replaced by `plausible = True`: touching same-color
  stones can be sent as one target.

A per-blob `print` also slows each frame noticeably. Restore both checks after
all six classes are sampled in the arena; the two related tests then pass again.

## Protocols and firmware

**v2, vision targets (`detect_live.py` -> port 4210).** Each JSON message contains
`version:2`, sender `session`, increasing `seq`, host wall-clock `t`, `ttl_ms:300`,
`units:"mm"`, `status`, and `targets`. Each target has `color`, `x`, `y`,
`confidence`, and `approach_deg` in pile mode. An empty target list revokes old
targets. v2 is intentionally incompatible with the original pixel-coordinate
`gems` protocol: update sender and receiver together.

`firmware/esp_link/` only logs v2 targets and lights an LED; it does not control
motors. In the current design, planning runs on the PC and this receiver is a
debugging aid.

**v3, robot commands (`teleop.py` or a planner -> port 4211).**

```json
{"v":3, "s":"a1b2c3d4e5f6", "q":42, "c":"drive", "l":0.5, "r":0.5}
```

`s` is a 12-character session, `q` an increasing sequence. Commands: `drive`
(`l`, `r` in -1..1), `start`, `stop`, `reset`, `grip` (`p`: open/close),
`lift` (`p`: up/down), `servo` (`i`, `deg`). `firmware/robot_ctrl/` runs a local
state machine (IDLE -> RUNNING -> DONE after 5 minutes; ESTOP by button or `stop`),
stops the wheels when no `drive` arrives for 300 ms, and returns a status packet
every 200 ms to the sender. Pins and servo angles in `config.h` are placeholders.

For both sketches, copy `secrets.example.h` to `secrets.h`, enter Wi-Fi
credentials, and compile with the ESP32 Arduino core (3.x for `robot_ctrl`) and
ArduinoJson 7. Use a controlled local network (for example your own hotspot);
networks with client isolation block PC -> ESP32 traffic.

The watchdogs measure time **since reception**, not network transit age. No
clock synchronization, authentication or end-to-end acknowledgement is provided.
Robot pose, collision-free approach and a close-range recheck must gate actual
pickup. `isolated` and `approach_deg` describe local silhouette clearance, not
proof that the robot's full route or footprint is clear.

## Tests and recorded video

```powershell
.venv\Scripts\python -m unittest discover -s tests -v
.venv\Scripts\python detect_live.py --video arena.mp4 --headless
```

Replay requires the same camera geometry/reference and never sends UDP.
`teleop.py --camera <n>` records `runs/<time>/video.mp4` for replay, together with
`log.jsonl` (one row per control tick: action, events, ESP32 status, pose) and
`meta.json`.

| Test file | Covers |
| --- | --- |
| `test_vision.py` (22) | six colors, whitening, ambiguous color, unknown neighbors, robot-sized occluders, zones, borders, temporal loss, missing calibration, exposure change, packets, hue wrap |
| `test_pile.py` (7) | edge picking, buried/surrounded stones, same-color pairs, walls, packet field, off by default |
| `test_target_lock.py` (8) | lock through flicker, timeout, occlusion, color change, sticky vision targets |
| `test_robot_pose.py` (6) | position, heading, parallax, size check, wrong id, grip offset |

With the local deviations above, two `test_vision.py` cases fail
(`incomplete_setup_does_not_transmit_targets`, `long_thin_cluster_rejected`).
All tests are synthetic; they do not establish real-camera accuracy or hardware
reliability.

See [REVIEW_AND_PLAN.md](REVIEW_AND_PLAN.md) for PDF rules, design decisions,
physical-lighting improvements, remaining robot work, and field acceptance tests.