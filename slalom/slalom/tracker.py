"""LiDAR cone tracker (pure Python, no ROS -> testable offline on bags).

Rule: DISTANCE ONLY FROM THE LIDAR, the camera only gives COLOUR + DIRECTION.
  1. Every scan is cut into clusters (neighbouring beams that are close together).
     Small clusters (< max_width) = cone candidates. Long clusters (walls, floor
     line, people) are ignored.
  2. Camera cone -> LiDAR candidate in that direction -> track (colour from camera,
     position from LiDAR).
  3. Every scan moves each track to the nearest candidate -> the cone is still
     known when it has left the camera view (no blind phase after a pass).
Camera cones without a LiDAR candidate (too far for the LiDAR) are passed on with
the camera distance and confidence 0.5.

Coordinates: car frame, x forward, y left, metres (same as Cone.msg).
"""
import math


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Track:
    __slots__ = ('color', 'x', 'y', 'missed', 'cam_age', 'id')

    def __init__(self, color, x, y, tid):
        self.color, self.x, self.y = color, x, y
        self.missed = 0     # scans without a LiDAR candidate near the track
        self.cam_age = 0    # camera frames since the camera last saw it
        self.id = tid


class ConeTracker:
    def __init__(self, yaw_offset_deg=-1.0, invert=False, min_range=0.15, max_range=2.0,
                 split_dist=0.10, max_width=0.25, cam_window_deg=25.0, cam_tol=0.6,
                 gate=0.30, max_missed=5, drop_x=-0.2, merge_dist=0.25, self_box_x=0.35, self_box_y=0.12):
        self.yaw = math.radians(yaw_offset_deg)
        self.invert = invert
        self.min_range = min_range        # m, closer = own car
        self.max_range = max_range        # m, farther LiDAR points are not used (floor line if the LiDAR is tilted)
        self.split_dist = split_dist      # m, gap between two beams that starts a new cluster
        self.max_width = max_width        # m, wider clusters are no cone (wall, floor, person)
        self.cam_window = math.radians(cam_window_deg)  # camera direction +- this -> LiDAR candidate
        self.cam_tol = cam_tol            # LiDAR candidate may differ this much (relative) from the camera distance
        self.gate = gate                  # m, candidate this close to a track = same cone
        self.max_missed = max_missed      # scans without candidate -> track dropped
        self.drop_x = drop_x              # m, tracks behind the car are dropped
        self.merge_dist = merge_dist
        self.self_box_x = self_box_x      # own car / LiDAR mount: |x| < box_x and |y| < box_y ignored
        self.self_box_y = self_box_y
        self.tracks = []
        self.candidates = []
        self.next_id = 0

    # ---------- LiDAR ----------
    def clusters(self, angle_min, angle_inc, range_min, range_max, ranges):
        """Scan -> list of cone candidates (x, y)."""
        pts = []   # (beam index, x, y)
        for i, r in enumerate(ranges):
            if not (range_min < r < range_max) or math.isinf(r) or math.isnan(r):
                continue
            if r < self.min_range or r > self.max_range + 0.3:
                continue
            a = angle_min + i * angle_inc
            a = _wrap((-a if self.invert else a) - self.yaw)
            x, y = r * math.cos(a), r * math.sin(a)
            if abs(x) < self.self_box_x and abs(y) < self.self_box_y:
                continue
            pts.append((i, x, y, r))
        n_beams = len(ranges)
        groups, cur = [], []
        for p in pts:
            if cur:
                q = cur[-1]
                gap_beams = p[0] - q[0]
                if gap_beams > 3 or math.hypot(p[1] - q[1], p[2] - q[2]) > self.split_dist:
                    groups.append(cur)
                    cur = []
            cur.append(p)
        if cur:
            groups.append(cur)
        # first and last group may be one cluster (scan wraps around at 360 deg)
        if len(groups) > 1:
            a, b = groups[0][0], groups[-1][-1]
            if (a[0] + n_beams - b[0]) <= 3 and math.hypot(a[1] - b[1], a[2] - b[2]) <= self.split_dist:
                groups[0] = groups[-1] + groups[0]
                groups.pop()
        cands = []
        for g in groups:
            width = math.hypot(g[0][1] - g[-1][1], g[0][2] - g[-1][2])
            if width > self.max_width:
                continue
            r = min(p[3] for p in g)
            if r > self.max_range:
                continue
            cx = sum(p[1] for p in g) / len(g)
            cy = sum(p[2] for p in g) / len(g)
            cands.append((cx, cy))
        return cands

    def update_scan(self, angle_min, angle_inc, range_min, range_max, ranges):
        self.candidates = self.clusters(angle_min, angle_inc, range_min, range_max, ranges)
        free = list(self.candidates)
        for tr in sorted(self.tracks, key=lambda t: t.x * t.x + t.y * t.y):
            best, bd = None, self.gate
            for c in free:
                d = math.hypot(c[0] - tr.x, c[1] - tr.y)
                if d < bd:
                    best, bd = c, d
            if best is not None:
                free.remove(best)
                tr.x, tr.y = best
                tr.missed = 0
            else:
                tr.missed += 1
        self._cleanup()

    # ---------- camera ----------
    def lidar_for(self, x, y):
        """LiDAR candidate in the direction of a camera cone, or None."""
        ang = math.atan2(y, x)
        dist = math.hypot(x, y)
        best, bd = None, self.cam_window
        for c in self.candidates:
            da = abs(_wrap(math.atan2(c[1], c[0]) - ang))
            dr = math.hypot(c[0], c[1])
            if da < bd and abs(dr - dist) < self.cam_tol * dist + 0.2:
                best, bd = c, da
        return best

    def update_camera(self, cones):
        """cones: list of (color, x, y) from the camera.
        Returns the cones for the driver: list of (color, x, y, confidence)."""
        for tr in self.tracks:
            tr.cam_age += 1
        out = []
        seen = set()
        for color, x, y in cones:
            c = self.lidar_for(x, y)
            if c is None:
                ang = math.atan2(y, x)
                if not any(abs(_wrap(math.atan2(t.y, t.x) - ang)) < self.cam_window for t in self.tracks):
                    out.append((color, x, y, 0.5))   # too far / not seen by the LiDAR -> camera distance
                continue
            tr = None
            for t in self.tracks:
                if math.hypot(t.x - c[0], t.y - c[1]) < self.gate:
                    tr = t
                    break
            if tr is None:
                tr = Track(color, c[0], c[1], self.next_id)
                self.next_id += 1
                self.tracks.append(tr)
            tr.color = color
            tr.cam_age = 0
            seen.add(tr.id)
        self._cleanup()
        for t in self.tracks:
            if t.x > 0.0:
                out.append((t.color, t.x, t.y, 1.0))
        return out

    def _cleanup(self):
        keep = []
        for tr in self.tracks:
            if tr.x < self.drop_x or tr.missed > self.max_missed:
                continue
            if any(math.hypot(k.x - tr.x, k.y - tr.y) < self.merge_dist for k in keep):
                continue
            keep.append(tr)
        self.tracks = keep
