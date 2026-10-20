"""Shared camera, calibration and JSON helpers; no hardware opened on import."""
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def save_config(path, data):
    path = Path(path)
    with path.open("w") as f:
        if path.suffix == ".json":
            json.dump(data, f, indent=2, allow_nan=False)
        else:
            yaml.safe_dump(data, f, sort_keys=False)


def camera_model(path):
    data = load_config(path)
    return data, np.asarray(data["K"], float), np.asarray(data["dist"], float)


def open_source(camera=0, video=None, exposure=50):
    if not 0 < exposure <= 100:
        raise ValueError("manual exposure must be 1..100 (100 us units)")
    cap = cv2.VideoCapture(str(video)) if video else cv2.VideoCapture(camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"cannot open {video or camera}")
    if not video:
        for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")),
                            (cv2.CAP_PROP_FRAME_WIDTH, 1280),
                            (cv2.CAP_PROP_FRAME_HEIGHT, 720), (cv2.CAP_PROP_FPS, 30),
                            (cv2.CAP_PROP_AUTO_EXPOSURE, 1),
                            (cv2.CAP_PROP_EXPOSURE, exposure),
                            # 1 buffer halves this UVC webcam to 15 fps (measured); 2 keeps 30 fps, <=1 frame stale.
                            (cv2.CAP_PROP_BUFFERSIZE, 2)):
            if not cap.set(prop, value):
                print(f"camera: property {prop} refused {value}")
        print("camera: requested MJPG 1280x720@30, manual exposure", exposure,
              "; readback exposure", cap.get(cv2.CAP_PROP_EXPOSURE),
              "auto", cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))
    return cap


def april_dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def tag_corners(center, side, yaw=0):
    """World x right, y forward; printed top is +y, corner order TL TR BR BL."""
    xy = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]], float) * side / 2
    c, s = np.cos(yaw), np.sin(yaw)
    return xy @ np.array([[c, s], [-s, c]]) + center
