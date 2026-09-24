# Gemstone sorting: overhead perception

This improves the camera/ESP32 prototype from `CPE-Exploratory1-main.zip`.
The source ZIP contained no separate plan. The implemented scope is perception,
camera calibration, and target communication. This is **not a complete autonomous
sorting robot**: motor drivers, robot pose, route planning, servos and gesture
control were not present and cannot be verified without the hardware details.

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

Use the normal GUI OpenCV package in requirements.txt for camera windows.
The workspace `.runtime` directory is an ignored, headless test dependency only.
Press `q` to quit detection, `m` to display foreground segmentation.

## Calibrate in the actual arena

1. Fix the camera, focus and resolution. Use the same lighting for sampling,
   reference capture and operation. Set tested camera properties in
   `camera_properties` (e.g. `AUTO_WB`, `AUTO_EXPOSURE`, `EXPOSURE`). Values are
   backend dependent; an accepted setting is not proof the hardware applied it.
   All three tools apply the same configuration. Do not change it mid-run.
2. Run `sample_hsv.py`. Select 1 violet, 2 cyan, 3 crimson, 4 orange/marigold,
   5 sky blue, 6 lime. Click several colored faces of several physical stones
   per class, including the normal shadow range. Avoid white highlights,
   the table and scoring markers. `u` undoes one patch; `s` saves; `q` quits.
   Old four-color thresholds are retained as starting data; recalibrate all six.
   Crimson and sky blue are deliberately empty until measured.
3. Measure the field and adjust `arena.size_mm`; 2100 x 1200 is approximate in
   the PDF. `calibrate_arena.py` freezes an **empty field** with space. Click
   floor corners in TL, TR, BR, BL order and press Enter. The long dimension
   must run horizontally; otherwise rotate the camera view or update dimensions.
4. On the rectified view, outline each of the six colored scoring zones and
   any fixed fixtures, pressing Enter per polygon. Press `s` to save. These are
   perception exclusions, not destination definitions or navigation obstacles.
   The reference must include the permanent markings but no stones or robot.
5. Place stones, inspect the foreground with `m`, and verify coordinates against
   ruler measurements. Rectified coordinates start at the top-left floor corner:
   x right, y down, in millimeters. Recalibrate after any camera move/settings change.

Missing background, corner mapping, or any color calibration results in
`setup_required` and no transmitted targets. Empty color ranges are never guessed.

## Implemented changes

- Foreground segmentation against the empty field groups white highlights and
  colored faces into one connected object. Unchanged floor markings disappear.
- Only sufficiently saturated, visible pixels vote for color. Overlapping HSV
  classes, mixed-color objects, and insufficient evidence become unknown (0).
  Confidence is an evidence fraction, **not a calibrated probability**.
- Unknown foreground, large objects, borders, and excluded zones block pickup
  clearance. Connected piles outside size limits are rejected, not force-split.
- Targets require four consecutive consistent detections; missing objects are
  removed immediately. Reacquisition starts the confirmation count again.
- Large overall exposure/foreground changes suppress output. Small additive
  shifts are compensated for segmentation only, not used to invent color.
- Perspective mapping uses physical-area thresholds; sampling handles red hue
  wrap, edge clicks, outliers, and exact patch undo. Modules are import-safe.
- UDP v2 caps output at eight candidates and 10 packets/second. Receiver clears
  targets on empty/stop messages, rejects reordered packets and expires silence
  after 300 ms. Embedded network credentials were replaced with a local header.

## Protocol and firmware

Each JSON message contains `version:2`, sender `session`, increasing `seq`, host
wall-clock `t`, `ttl_ms:300`, `units:"mm"`, `status`, and `targets`. Each target has
`color`, `x`, `y`, `confidence`. An empty target list revokes old targets. v2 is
intentionally incompatible with the original pixel-coordinate `gems` protocol:
update sender and receiver together.

Copy `firmware/esp_link/secrets.example.h` to `secrets.h`, enter Wi-Fi credentials,
and compile with the ESP32 Arduino core and ArduinoJson 7. The sketch only logs
targets and lights an LED; it does not control motors. Future motor integration
must stop when `clearTarget()` executes and enforce a local five-minute timer.
Coordinates assume the default arena limits in the receiver; change limits if
you configure a different measured field.

The watchdog measures time **since reception**, not network transit age. No
clock synchronization, authentication or end-to-end acknowledgement is provided.
Use a controlled local network; do not use this advisory stream alone as a motor
safety system. Robot pose, collision-free approach and a close-range recheck
must gate actual pickup. `isolated` is local silhouette clearance, not proof that
an approach route or the robot's full footprint is clear.

## Tests and recorded video

```powershell
.venv\Scripts\python -m unittest discover -s tests -v
.venv\Scripts\python detect_live.py --video arena.mp4 --headless
```

Replay requires the same camera geometry/reference and never sends UDP.
Synthetic tests cover all six colors, partial/full whitening, ambiguous color,
unknown neighbors, robot-sized occluders, scoring markers, borders, temporal
loss, missing calibration, exposure change, packet units, and hue wrap.
These tests do not establish real-camera accuracy or hardware reliability.

See [REVIEW_AND_PLAN.md](REVIEW_AND_PLAN.md) for PDF rules, design decisions,
physical-lighting improvements, remaining robot work, and field acceptance tests.
