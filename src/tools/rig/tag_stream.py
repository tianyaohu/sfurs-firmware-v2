"""Background AprilTag pose stream for scripted drives (no window, no keys).

TagStream grabs camera frames in a thread and keeps every pose of one tag with its
time.monotonic() stamp, so a script that drives the robot in the same process can ask
afterwards where the tag was still before and after each move. Pose math comes from
rig_measure; only the threading and the still-window selection live here.

With >= 3 floor tags in view at start, poses are ray-plane intersections with the floor
plane raised by the tag's height (rig_measure floor-plane mode), which is far less tilt-
sensitive than single-tag PnP. The plane is frozen during the first `settle_s` seconds, so
the robot may cover floor tags afterwards. Without floor tags it falls back to single-tag.
"""
import threading
import time

import cv2
import numpy as np

from rig_common import april_dictionary, camera_model, open_source
from rig_measure import average_pose, displacement, floor_basis, path_summary, plane_pose, tag_pose


class TagStream:
    def __init__(self, calibration, camera=0, exposure=100, tag_id=0, tag_mm=100.0, reliable_px=250.0,
                 floor_ids=(10, 11, 12, 13), floor_tag_mm=120.0, tag_height_mm=None, settle_s=1.5):
        """tag_height_mm: tag above the floor tags; None estimates it from single-tag depth."""
        _, self.K, self.dist = camera_model(calibration)
        self.tag_id, self.tag_mm, self.reliable_px = tag_id, tag_mm, reliable_px
        self.camera, self.exposure = camera, exposure
        self.floor_ids, self.floor_tag_mm, self.settle_s = tuple(floor_ids), floor_tag_mm, settle_s
        self.basis = self.origin = None  # floor plane, None in single-tag mode
        self.height_mm = tag_height_mm
        self.floor_seen = 0
        self.floor_centers = []  # camera frame, mm, frozen at start
        self.ready = threading.Event()
        self.frames = []  # (t, pose or None, reliable), from when ready
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

    def start(self):
        self._cap = open_source(self.camera, None, self.exposure)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.ready.wait(self.settle_s + 3.0)
        return self

    @property
    def mode(self):
        return "floor-plane" if self.basis is not None else "single-tag"

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._cap.release()

    def __enter__(self):
        return self.start()

    def __exit__(self, *_exc):
        self.stop()

    def _freeze_floor(self, floor, tag_centers):
        full = [np.median(q, axis=0) for q in floor.values() if len(q) >= 5]
        self.floor_seen = len(full)
        self.floor_centers = full
        if len(full) >= 3:
            self.basis, self.origin = floor_basis(full), np.mean(full, axis=0)
            if self.height_mm is None and tag_centers:
                n = self.basis[2]
                self.height_mm = float(np.median([n @ (c - self.origin) for c in tag_centers]))
        self.ready.set()

    def _run(self):
        detector = cv2.aruco.ArucoDetector(april_dictionary())
        center = np.array([self.K[0, 2], self.K[1, 2]])
        floor, tag_centers, t_start = {i: [] for i in self.floor_ids}, [], time.monotonic()
        while not self._stop.is_set():
            ok, image = self._cap.read()
            t = time.monotonic()
            if not ok:
                continue
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            seen = [] if ids is None else list(ids.ravel())
            if not self.ready.is_set():
                for fid in self.floor_ids:
                    if fid in seen:
                        pose = tag_pose(corners[seen.index(fid)], self.K, self.dist, self.floor_tag_mm)
                        if pose is not None:
                            floor[fid].append(pose[0])
                if self.tag_id in seen:
                    pose = tag_pose(corners[seen.index(self.tag_id)], self.K, self.dist, self.tag_mm)
                    if pose is not None:
                        tag_centers.append(pose[0])
                if t - t_start >= self.settle_s:
                    self._freeze_floor(floor, tag_centers)
                continue
            pose, reliable = None, False
            if self.tag_id in seen:
                c = corners[seen.index(self.tag_id)]
                pose = (plane_pose(c, self.K, self.dist, self.basis, self.origin, self.height_mm or 0.0)
                        if self.basis is not None else tag_pose(c, self.K, self.dist, self.tag_mm))
                reliable = bool(np.hypot(*(c.reshape(4, 2).mean(axis=0) - center)) <= self.reliable_px)
            with self._lock:
                self.frames.append((t, pose, reliable))

    def snapshot(self):
        with self._lock:
            return list(self.frames)


def still_pose(frames, t_from, t_to, min_frames=5, max_spread_mm=3.0):
    """Averaged pose of the tag over [t_from, t_to], or None when it was not seen in
    enough frames or moved more than max_spread_mm (not still)."""
    poses = [p for t, p, _r in frames if t_from <= t <= t_to and p is not None]
    if len(poses) < min_frames:
        return None
    xyz = np.array([p[0] for p in poses])
    if np.max(np.linalg.norm(xyz - np.median(xyz, axis=0), axis=1)) > max_spread_mm:
        return None
    return average_pose(poses)


def move_summary(frames, ref_window, end_window, commanded_mm=None, basis=None):
    """Drive metrics for one move between two still windows (monotonic times): the
    rig_measure path summary plus the direction moved, in the floor frame when `basis`
    (TagStream.basis) is given, else the reference tag frame. None if either end was not
    seen still."""
    ref, end = still_pose(frames, *ref_window), still_pose(frames, *end_window)
    if ref is None or end is None:
        return None
    path = []
    for t, p, reliable in frames:
        if ref_window[1] < t < end_window[0] and p is not None:
            _, yaw, d = displacement(ref, p, basis)
            path.append(dict(t=t, x_mm=float(d[0]), y_mm=float(d[1]), yaw_deg=yaw, reliable=reliable))
    out = path_summary(path, displacement(ref, end, basis), commanded_mm)
    out["direction_deg"] = float(np.degrees(np.arctan2(out["end_y_mm"], out["end_x_mm"])))
    return out


def inside_floor(point, floor_centers, basis):
    """True when `point` (camera frame) lies inside the polygon of the floor tag centres,
    seen along the floor normal: the mat area the tags mark out."""
    if basis is None or len(floor_centers) < 3:
        return False
    pts = np.array([basis[:2] @ c for c in floor_centers])
    q = basis[:2] @ np.asarray(point, float)
    hull = cv2.convexHull(pts.astype(np.float32)).reshape(-1, 2)
    return cv2.pointPolygonTest(hull, (float(q[0]), float(q[1])), False) >= 0
