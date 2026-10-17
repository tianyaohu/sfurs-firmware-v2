"""Background AprilTag pose stream for scripted drives (no window, no keys).

TagStream grabs camera frames in a thread and keeps every pose of one tag with its
time.monotonic() stamp, so a script that drives the robot in the same process can ask
afterwards where the tag was still before and after each move. Pose math comes from
rig_measure; only the threading and the still-window selection live here.
"""
import threading
import time

import cv2
import numpy as np

from rig_common import april_dictionary, camera_model, open_source
from rig_measure import average_pose, displacement, path_summary, tag_pose


class TagStream:
    def __init__(self, calibration, camera=0, exposure=100, tag_id=0, tag_mm=100.0, reliable_px=250.0):
        _, self.K, self.dist = camera_model(calibration)
        self.tag_id, self.tag_mm, self.reliable_px = tag_id, tag_mm, reliable_px
        self.camera, self.exposure = camera, exposure
        self.frames = []  # (t, pose or None, reliable)
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

    def start(self):
        self._cap = open_source(self.camera, None, self.exposure)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._cap.release()

    def __enter__(self):
        return self.start()

    def __exit__(self, *_exc):
        self.stop()

    def _run(self):
        detector = cv2.aruco.ArucoDetector(april_dictionary())
        center = np.array([self.K[0, 2], self.K[1, 2]])
        while not self._stop.is_set():
            ok, image = self._cap.read()
            t = time.monotonic()
            if not ok:
                continue
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            pose, reliable = None, False
            if ids is not None and self.tag_id in ids.ravel():
                c = corners[list(ids.ravel()).index(self.tag_id)]
                pose = tag_pose(c, self.K, self.dist, self.tag_mm)
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


def move_summary(frames, ref_window, end_window, commanded_mm=None):
    """Drive metrics for one move between two still windows (monotonic times): the
    rig_measure path summary plus the direction moved in the reference tag frame.
    None if either end was not seen still."""
    ref, end = still_pose(frames, *ref_window), still_pose(frames, *end_window)
    if ref is None or end is None:
        return None
    path = []
    for t, p, reliable in frames:
        if ref_window[1] < t < end_window[0] and p is not None:
            _, yaw, d = displacement(ref, p)
            path.append(dict(t=t, x_mm=float(d[0]), y_mm=float(d[1]), yaw_deg=yaw, reliable=reliable))
    out = path_summary(path, displacement(ref, end), commanded_mm)
    out["direction_deg"] = float(np.degrees(np.arctan2(out["end_y_mm"], out["end_x_mm"])))
    return out
