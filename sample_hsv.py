"""Sample colored faces under match lighting; glare is not color evidence."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from vision import NAMES


def ranges_from_samples(px):
    a = np.asarray(px, dtype=float).reshape(-1, 3)
    a = a[(a[:, 1] >= 60) & (a[:, 2] >= 45)]
    if len(a) < 8:
        return []
    # Unwrap around circular median direction so red can straddle hue zero.
    angle = a[:, 0] * np.pi / 90
    center = np.arctan2(np.sin(angle).mean(), np.cos(angle).mean()) * 90 / np.pi % 180
    unwrapped = (a[:, 0] - center + 90) % 180 - 90 + center
    lo, hi = np.percentile(unwrapped, [5, 95]) + np.array([-6, 6])
    if hi-lo > 45:
        return []  # Mixed colors: sample again, do not create a catch-all range.
    smin = max(60, int(np.percentile(a[:, 1], 5))-25)
    vmin = max(45, int(np.percentile(a[:, 2], 5))-30)
    low, high = int(np.floor(lo)) % 180, int(np.ceil(hi)) % 180
    spans = [(low, high)] if low <= high else [(0, high), (low, 179)]
    return [{'lo': [l, smin, vmin], 'hi': [h, 255, 255]} for l, h in spans]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('camera_index', nargs='?', type=int)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    args = p.parse_args()
    cfg = json.loads(args.config.read_text())
    cam = args.camera_index if args.camera_index is not None else cfg.get('camera_index', 0)
    cap = cv2.VideoCapture(cam)
    samples = {k: [] for k in NAMES}
    current = 1
    frame = None
    def click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and frame is not None:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            patch = hsv[max(0,y-4):min(hsv.shape[0],y+4), max(0,x-4):min(hsv.shape[1],x+4)]
            samples[current].append(patch.reshape(-1,3).tolist())
            print(NAMES[current], 'patches:', len(samples[current]))
    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot open camera')
        for name, value in cfg.get('camera_properties', {}).items():
            prop = getattr(cv2, 'CAP_PROP_'+name, None)
            if prop is None or not cap.set(prop, value):
                print('Camera property rejected:', name)
        cv2.namedWindow('sample')
        cv2.setMouseCallback('sample', click)
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('No camera frame')
            view = frame.copy()
            cv2.putText(view, f'{current} {NAMES[current]} | 1-6 select, click, u undo, s save, q quit',
                        (10,25), 0, .55, (255,255,255), 2)
            cv2.imshow('sample', view)
            k = cv2.waitKey(1) & 255
            if k == ord('q'):
                break
            if ord('1') <= k <= ord('6'):
                current = k-ord('0')
            if k == ord('u') and samples[current]:
                samples[current].pop()
            if k == ord('s'):
                for cid, patches in samples.items():
                    if not patches:
                        continue
                    ranges = ranges_from_samples([pixel for patch in patches for pixel in patch])
                    if ranges:
                        cfg['hsv'][f'{cid}_{NAMES[cid]}'] = ranges
                    else:
                        print('Rejected samples for', NAMES[cid], '- clear with u and resample')
                cfg['camera_index'] = cam
                args.config.write_text(json.dumps(cfg, indent=2))
                print('Saved. Missing:', [name for cid,name in NAMES.items() if not cfg['hsv'].get(f'{cid}_{name}')])
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
