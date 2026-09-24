"""Conservative overhead perception. Unknown objects still block pickup clearance."""
from dataclasses import dataclass
import cv2
import numpy as np

NAMES = {1: 'violet', 2: 'cyan', 3: 'crimson', 4: 'orange', 5: 'skyblue', 6: 'lime'}


def mask_for(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for r in ranges:
        mask |= cv2.inRange(hsv, np.array(r['lo'], np.uint8), np.array(r['hi'], np.uint8))
    return mask


def warp(frame, cfg):
    arena = cfg.get('arena', {})
    if arena.get('source_size_px') and list(frame.shape[1::-1]) != arena['source_size_px']:
        raise ValueError('Camera resolution changed; recalibrate arena/background')
    corners = arena.get('corners_px')
    if not corners:
        return frame.copy()
    scale = arena.get('mm_per_px', 2)
    if scale <= 0:
        raise ValueError('mm_per_px must be positive')
    width, height = [int(round(v / scale)) for v in arena['size_mm']]
    if width < 2 or height < 2:
        raise ValueError('Arena dimensions and mm_per_px must be positive')
    src = np.array(corners, np.float32)
    if src.shape != (4, 2) or abs(cv2.contourArea(src)) < 100:
        raise ValueError('Four non-degenerate arena corners required')
    dst = np.array([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]], np.float32)
    return cv2.warpPerspective(frame, cv2.getPerspectiveTransform(src, dst), (width, height))


@dataclass
class Observation:
    x: float
    y: float
    color: int
    confidence: float
    area: float
    isolated: bool = False
    stable: bool = False


class Detector:
    def __init__(self, cfg, background=None):
        self.cfg = cfg
        self.options = cfg.get('vision', {})
        self.background = background
        self.previous = []
        self.counts = []

    def process(self, raw):
        frame = warp(raw, self.cfg)
        h, w = frame.shape[:2]
        valid = np.full((h, w), 255, np.uint8)
        for polygon in self.cfg.get('exclude_polygons', []):
            cv2.fillPoly(valid, [np.array(polygon, np.int32)], 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_masks = {int(k.split('_')[0]): mask_for(hsv, v)
                       for k, v in self.cfg['hsv'].items() if v}
        #calibrated = set(color_masks) == set(NAMES) // we will use this when we have all the colors available
        calibrated = any(color_masks)
        ready = self.background is not None and bool(self.cfg.get('arena', {}).get('corners_px')) and calibrated
        status = 'ok' if ready else 'setup_required'
        if self.background is not None:
            if self.background.shape != frame.shape:
                raise ValueError('Reference dimensions changed; recalibrate arena/background')
            delta = frame.astype(np.float32) - self.background.astype(np.float32)
            offset = np.median(delta[valid > 0], axis=0) if np.any(valid) else np.zeros(3)
            difference = np.max(np.abs(delta-offset), axis=2)
            foreground = np.uint8(difference > self.options.get('background_delta', 30)) * 255
            foreground &= valid
            changed = np.count_nonzero(foreground) / max(1, np.count_nonzero(valid))
            if changed > self.options.get('max_foreground_fraction', .3) or np.max(np.abs(offset)) > 35:
                ready, status = False, 'lighting_or_camera_change'
        else:
            foreground = np.zeros((h, w), np.uint8)
            for m in color_masks.values():
                foreground |= m
            foreground &= valid
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        foreground = cv2.morphologyEx(foreground, cv2.MORPH_CLOSE, kernel)
        foreground &= valid
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(foreground)
        for label in range(1, count):
            component = np.uint8(labels == label)*255
            contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(foreground, contours, -1, 255, cv2.FILLED)
        foreground &= valid
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(foreground)
        arena = self.cfg.get('arena', {})
        metric = bool(arena.get('corners_px'))
        scale = float(arena.get('mm_per_px', 2)) if metric else 1.0
        area_limits = self.cfg['gem_area_mm2' if metric else 'gem_area_px']
        gap = self.options.get('clearance_mm', 60) / scale if metric else self.options.get('clearance_px', 45)
        observations = []
        reliable = (hsv[:, :, 1] >= self.cfg.get('min_saturation', 60)) & (hsv[:, :, 2] >= 45)
        ownership = np.zeros((h, w), np.uint8)
        for m in color_masks.values():
            ownership += (m > 0).astype(np.uint8)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.options.get('min_obstacle_px', 12):
                continue
            component = labels == label
            x, y = centroids[label]
            votes = {cid: np.count_nonzero(component & reliable & (m > 0) & (ownership == 1))
                     for cid, m in color_masks.items()}
            ranking = sorted(votes.items(), key=lambda pair: pair[1], reverse=True)
            cid, top = ranking[0] if ranking else (0, 0)
            second = ranking[1][1] if len(ranking) > 1 else 0
            evidence = top / area
            dominance = top / max(1, sum(votes.values()))
            # plausible = area_limits['min'] <= area * scale * scale <= area_limits['max']
            # extent = max(stats[label, cv2.CC_STAT_WIDTH], stats[label, cv2.CC_STAT_HEIGHT]) * scale
            # if metric and extent > self.options.get('max_gem_extent_mm', 70):
            #     plausible = False
            plausible = True

            print(
                f"candidate={NAMES.get(cid, 'unknown')} "
                f"area={area * scale * scale:.1f} "
                f"unit={'mm2' if metric else 'px2'} "
                f"size_ok={plausible} "
                f"color_fraction={evidence:.2f} "
                f"dominance={dominance:.2f} "
                f"margin={(top-second) / max(1, top):.2f} "
                f"votes={votes}"
            )

            if (not plausible or evidence < self.options.get('min_color_fraction', .3)
                    or dominance < .85 or (top-second) / max(1, top) < .65):
                cid = 0

            other = np.uint8((foreground > 0) & ~component)*255
            other[valid == 0] = 255
            other[[0, -1], :] = 255
            other[:, [0, -1]] = 255
            distance = cv2.distanceTransform(255-other, cv2.DIST_L2, 5)
            clearance = float(distance[component].min()) > gap
            observations.append(Observation(float(x), float(y), cid, round(evidence*dominance, 3),
                                            area*scale*scale, clearance and cid != 0))
        used = set()
        new_counts = []
        for obs in observations:
            candidates = [(np.hypot(obs.x-p.x, obs.y-p.y), i) for i, p in enumerate(self.previous)
                          if i not in used and p.color == obs.color and obs.color != 0]
            distance, index = min(candidates, default=(float('inf'), -1))
            hits = 1
            if distance <= self.options.get('max_motion_px', 8):
                used.add(index)
                hits = self.counts[index]+1
            new_counts.append(hits)
            obs.stable = ready and hits >= self.options.get('stable_frames', 4)
        self.previous, self.counts = (observations, new_counts) if ready else ([], [])
        return frame, observations, foreground, status


def make_packet(observations, cfg, seq, session, status, timestamp):
    scale = float(cfg.get('arena', {}).get('mm_per_px', 2))
    targets = [{'color': o.color, 'x': round(o.x*scale, 1), 'y': round(o.y*scale, 1),
                'confidence': o.confidence} for o in observations if o.stable and o.isolated]
    targets = sorted(targets, key=lambda g: g['confidence'], reverse=True)[:8] if status == 'ok' else []
    return {'version': 2, 'session': session, 'seq': seq, 't': timestamp, 'ttl_ms': 300,
            'units': 'mm', 'status': status, 'targets': targets}
