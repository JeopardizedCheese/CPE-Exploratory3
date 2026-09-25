"""Camera or recorded-video detection; see README.md for calibration and protocol."""
import argparse
import json
from pathlib import Path
import socket
import time
import uuid
import cv2
import numpy as np
from vision import Detector, make_packet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('camera_index', nargs='?', type=int)
    parser.add_argument('esp_ip', nargs='?', default='127.0.0.1')
    parser.add_argument('esp_port', nargs='?', type=int, default=4210)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    parser.add_argument('--video', help='Recorded video; UDP disabled for replay')
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    background_path = args.config.parent / cfg.get('background_path', 'background.png')
    background = cv2.imread(str(background_path)) if background_path.exists() else None
    detector = Detector(cfg, background)
    source = args.video or (args.camera_index if args.camera_index is not None else cfg.get('camera_index', 0))
    cap = cv2.VideoCapture(source)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    session = uuid.uuid4().hex[:12]
    seq = 0
    last_send = 0
    show_mask = False
    def send(observations, status):
        nonlocal seq
        seq += 1
        packet = make_packet(observations, cfg, seq, session, status, time.time())
        if not args.video:
            sock.sendto(json.dumps(packet, separators=(',', ':')).encode(), (args.esp_ip, args.esp_port))
        return packet
    try:
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open camera/video: {source}')
        for name, value in cfg.get('camera_properties', {}).items():
            prop = getattr(cv2, 'CAP_PROP_'+name, None)
            if prop is None or not cap.set(prop, value):
                print(f'Warning: camera did not accept {name}={value}')
        while True:
            ok, raw = cap.read()
            if not ok:
                break
            frame, observations, mask, status = detector.process(raw)
            now = time.monotonic()
            if now-last_send >= .1:
                packet = send(observations, status)
                last_send = now
                if args.headless:
                    print(json.dumps(packet))
            if not args.headless:
                for o in observations:
                    eligible = o.stable and o.isolated
                    color = (0, 220, 0) if eligible else (0, 180, 255)
                    point = (round(o.x), round(o.y))
                    cv2.circle(frame, point, 12, color, 2)
                    if o.approach_deg is not None:   # pile mode: arrow = robot's driving direction
                        a = np.radians(o.approach_deg)
                        tail = (round(o.x - 35*np.cos(a)), round(o.y - 35*np.sin(a)))
                        cv2.arrowedLine(frame, tail, point, color, 2, tipLength=.3)
                    cv2.putText(frame, f'{o.color or "?"} {o.confidence:.2f}', point,
                                cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
                cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
                cv2.imshow('detect', frame)
                if show_mask:
                    cv2.imshow('foreground', mask)
                key = cv2.waitKey(1) & 255
                if key == ord('q'):
                    break
                if key == ord('m'):
                    show_mask = not show_mask
                    if not show_mask:
                        cv2.destroyWindow('foreground')
    finally:
        try:
            send([], 'stopped')
        finally:
            cap.release()
            sock.close()
            if not args.headless:
                cv2.destroyAllWindows()


if __name__ == '__main__':
    main()