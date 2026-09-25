"""Conservative overhead perception. Unknown objects still block pickup clearance."""
from dataclasses import dataclass, replace
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
    isolated: bool = False      # pickable: clear all around, or clear along approach_deg
    stable: bool = False
    approach_deg: float = None  # robot heading to drive in with (0=+x, 90=+y); None = any side
    held: int = 0               # >0: target kept from an earlier frame (sticky_frames)


class Detector:
    def __init__(self, cfg, background=None):
        self.cfg = cfg
        self.options = cfg.get('vision', {})
        self.background = background
        self.previous = []
        self.counts = []
        self.previous_targets = []

    def _hold_targets(self, observations, counts, ready):
        """Hysteresis: a target that drops out for a few frames stays a target.

        Kept for up to vision.sticky_frames frames (0 = off) unless a different known
        colour now sits at its spot. Ghosts are appended to observations."""
        sticky = self.options.get('sticky_frames', 0)
        if not (sticky and ready):
            self.previous_targets = []
            return
        near = self.options.get('max_motion_px', 8)
        current = [o for o in observations if o.stable and o.isolated]
        for prev in self.previous_targets:
            def close(o):
                return np.hypot(o.x - prev.x, o.y - prev.y) <= near
            if any(o.color == prev.color and close(o) for o in current):
                continue                                   # still a real target
            if any(o.color not in (0, prev.color) and close(o) for o in observations):
                continue                                   # colour changed: do not hold
            if prev.held >= sticky:
                continue
            observations.append(replace(prev, held=prev.held + 1))
            counts.append(self.options.get('stable_frames', 4))
        self.previous_targets = [o for o in observations if o.stable and o.isolated]


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
        blocked = (foreground > 0) | (valid == 0)
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
            if self.options.get('pile_mode', False) and metric and not observations[-1].isolated:
                if cid != 0:
                    # one known stone with something nearby: any single free side is enough
                    approach = _find_approach(blocked, component, x, y,
                                              _away_from_nearest(blocked, component, x, y),
                                              self.options.get('gripper_width_mm', 60) / scale,
                                              self.options.get('approach_length_mm', 80) / scale, np.pi,
                                              self.options.get('approach_start_mm', 6) / scale)
                    if approach is not None:
                        observations[-1].isolated = True
                        observations[-1].approach_deg = approach
                else:
                    for rx, ry, rc, conf, rarea, approach in directional_candidates(
                            component, color_masks, reliable & (ownership == 1), blocked, scale, self.options):
                        observations.append(Observation(rx, ry, rc, conf, rarea, True, False, approach))
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
        self._hold_targets(observations, new_counts, ready)
        self.previous, self.counts = (observations, new_counts) if ready else ([], [])
        return frame, observations, foreground, status


def _corridor_free(blocked, own, cx, cy, direction, width_px, length_px, start_px=0.0):
    """True if a gripper-wide strip from (just ahead of) the stone centre outward is free.
    blocked: foreground or invalid pixels; own: pixels of the stone itself (ignored).
    start_px skips a thin band at the centre line so diagonal neighbours behind the
    gripper's closing line don't block it."""
    ux, uy = np.cos(direction), np.sin(direction)
    vx, vy = -uy * width_px / 2, ux * width_px / 2
    sx, sy = cx + ux * start_px, cy + uy * start_px
    ex, ey = cx + ux * length_px, cy + uy * length_px
    poly = np.array([[sx + vx, sy + vy], [ex + vx, ey + vy], [ex - vx, ey - vy], [sx - vx, sy - vy]])
    h, w = blocked.shape
    if poly[:, 0].min() < 0 or poly[:, 1].min() < 0 or poly[:, 0].max() > w - 1 or poly[:, 1].max() > h - 1:
        return False                                  # corridor leaves the arena
    x0, y0 = np.floor(poly.min(axis=0)).astype(int)
    x1, y1 = np.ceil(poly.max(axis=0)).astype(int) + 1
    strip = np.zeros((y1 - y0, x1 - x0), np.uint8)
    cv2.fillConvexPoly(strip, np.round(poly - [x0, y0]).astype(np.int32), 255)
    hit = blocked[y0:y1, x0:x1] & ~own[y0:y1, x0:x1] & (strip > 0)
    return not np.any(hit)


def _angle_gap(a, b):
    return abs(np.arctan2(np.sin(a - b), np.cos(a - b)))


def _find_approach(blocked, own, cx, cy, preferred, width_px, length_px, max_turn, start_px=0.0):
    """Free outward direction closest to preferred, or None. Returns robot heading in degrees."""
    steps = 16
    options = sorted((preferred + k * 2 * np.pi / steps for k in range(steps)),
                     key=lambda a: _angle_gap(a, preferred))
    for direction in options:
        if _angle_gap(direction, preferred) > max_turn + 1e-6:
            break
        if _corridor_free(blocked, own, cx, cy, direction, width_px, length_px, start_px):
            heading = np.degrees(np.arctan2(-np.sin(direction), -np.cos(direction)))
            return round(float(heading), 1)
    return None


def _away_from_nearest(blocked, own, cx, cy):
    ys, xs = np.nonzero(blocked & ~own)
    if not len(xs):
        return 0.0
    i = np.argmin((xs - cx) ** 2 + (ys - cy) ** 2)
    return float(np.arctan2(cy - ys[i], cx - xs[i]))


def directional_candidates(component, color_masks, votable, blocked, scale, options):
    """Pile handling: pickable single-colour stones on the edge of a merged blob.

    Splits the blob into single-colour regions, keeps regions that look like one stone,
    and accepts a region if a gripper-wide corridor pointing away from the pile is free.
    Returns [(x_px, y_px, color, confidence, area_mm2, approach_deg)].
    """
    width_px = options.get('gripper_width_mm', 60) / scale
    length_px = options.get('approach_length_mm', 80) / scale
    own_radius = options.get('own_radius_mm', 25) / scale
    min_area = options.get('region_min_mm2', 150)
    max_area = options.get('region_max_mm2', 2000)
    max_extent = options.get('max_gem_extent_mm', 70)
    max_turn = np.radians(options.get('max_approach_turn_deg', 67.5))
    start_px = options.get('approach_start_mm', 6) / scale
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    # Work in a window around the blob; the margin fits any corridor that stays in the arena.
    ys, xs = np.nonzero(component)
    margin = int(np.ceil(length_px + width_px + own_radius)) + 2
    y0, x0 = max(0, ys.min() - margin), max(0, xs.min() - margin)
    y1 = min(component.shape[0], ys.max() + margin + 1)
    x1 = min(component.shape[1], xs.max() + margin + 1)
    window = (slice(y0, y1), slice(x0, x1))
    component, votable, blocked = component[window], votable[window], blocked[window]
    color_masks = {cid: m[window] for cid, m in color_masks.items()}
    bx, by = xs.mean() - x0, ys.mean() - y0
    colored = {cid: component & votable & (m > 0) for cid, m in color_masks.items()}
    found = []
    for cid, region_all in colored.items():
        if not region_all.any():
            continue
        others = np.zeros_like(component)
        for other_id, m in colored.items():
            if other_id != cid:
                others |= m
        closed = cv2.morphologyEx(region_all.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(closed)
        for r in range(1, n):
            area_mm2 = stats[r, cv2.CC_STAT_AREA] * scale * scale
            extent = max(stats[r, cv2.CC_STAT_WIDTH], stats[r, cv2.CC_STAT_HEIGHT]) * scale
            if not (min_area <= area_mm2 <= max_area) or extent > max_extent:
                continue
            cx, cy = cents[r]
            region = labels == r
            circle = np.zeros(component.shape, np.uint8)
            cv2.circle(circle, (int(round(cx)), int(round(cy))), int(round(own_radius)), 1, -1)
            near = circle > 0
            own = component & ((near & ~others) | (cv2.dilate(region.astype(np.uint8), kernel) > 0))
            votes_here = np.count_nonzero(region & region_all)
            dominance = votes_here / max(1, votes_here + np.count_nonzero(others & near))
            fraction = votes_here / max(1, np.count_nonzero(own))
            if fraction < options.get('min_color_fraction', .3):
                continue
            outward = np.arctan2(cy - by, cx - bx)
            if np.hypot(cx - bx, cy - by) < 1:
                outward = _away_from_nearest(blocked, own, cx, cy)
            approach = _find_approach(blocked, own, cx, cy, outward, width_px, length_px, max_turn, start_px)
            if approach is not None:
                found.append((float(cx + x0), float(cy + y0), cid, round(fraction * dominance, 3),
                              area_mm2, approach))
    return found


def make_packet(observations, cfg, seq, session, status, timestamp):
    scale = float(cfg.get('arena', {}).get('mm_per_px', 2))
    targets = [{'color': o.color, 'x': round(o.x*scale, 1), 'y': round(o.y*scale, 1),
                'confidence': o.confidence,
                **({'approach_deg': o.approach_deg} if o.approach_deg is not None else {})}
               for o in observations if o.stable and o.isolated]
    targets = sorted(targets, key=lambda g: g['confidence'], reverse=True)[:8] if status == 'ok' else []
    return {'version': 2, 'session': session, 'seq': seq, 't': timestamp, 'ttl_ms': 300,
            'units': 'mm', 'status': status, 'targets': targets}