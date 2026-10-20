#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python>=4.10,<5", "numpy>=1.26", "PyYAML>=6", "reportlab>=4"]
# ///
"""Single AprilTag robot tracker, with calibrated floor and top-tag parallax.

  uv run src/tools/rig/rig_track.py --calibration /tmp/camera.yaml --floor floor.json --out /tmp/rig.jsonl
  uv run src/tools/rig/rig_track.py --floor floor.json --print-tags /tmp/tags
"""
import argparse
import io
import json
import time
from pathlib import Path

import cv2
import numpy as np

from rig_common import (april_dictionary, camera_model, load_config, open_source,
                        tag_corners)


class Tracker:
    def __init__(self, calibration, floor, robot_id=0, tag_mm=100, tag_height_mm=0,
                 tag_offset_mm=(0, 0), tag_yaw_rad=0, max_rms_px=2, min_side_px=16):
        self.K = np.asarray(calibration["K"], float)
        self.dist = np.asarray(calibration["dist"], float)
        self.size = tuple(calibration["image_size"])
        self.floor = floor
        self.robot_id, self.tag_mm, self.height = robot_id, tag_mm, tag_height_mm
        self.offset, self.tag_yaw = np.asarray(tag_offset_mm, float), tag_yaw_rad
        self.max_rms, self.min_side = max_rms_px, min_side_px
        if tag_mm <= 0 or tag_height_mm < 0:
            raise ValueError("tag size must be positive, height nonnegative")
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(april_dictionary(), params)
        self.R, self.C, self.H, self.floor_rms = None, None, None, None
        if "tags" in floor:
            ids = [int(i) for i in floor["tags"]]
            if len(set(ids)) < 4 or robot_id in ids:
                raise ValueError("need >=4 floor tag IDs distinct from robot ID")
        else:
            if "H" in floor:
                self.H = np.asarray(floor["H"], float)
            else:
                src = self.undistort(floor["image_points_px"])
                dst = np.asarray(floor["floor_points_mm"], float)
                self.H, _ = cv2.findHomography(src, dst, 0)
                if self.H is not None:
                    mapped = cv2.perspectiveTransform(src.reshape(1, -1, 2), self.H)[0]
                    self.floor_rms = float(np.sqrt(np.mean((mapped - dst) ** 2)))
            if self.H is None or self.H.shape != (3, 3) or np.linalg.matrix_rank(self.H) != 3:
                raise ValueError("degenerate floor homography")
            if "camera_center_mm" in floor:
                self.C = np.asarray(floor["camera_center_mm"], float)
                if self.C.shape != (3,) or self.C[2] <= self.height:
                    raise ValueError("camera must be above the robot tag")
            if self.height and self.C is None:
                raise ValueError("homography with elevated tag needs camera_center_mm")

    def undistort(self, points):
        return cv2.undistortPoints(np.asarray(points, float).reshape(-1, 1, 2),
                                   self.K, self.dist, P=self.K).reshape(-1, 2)

    def establish_floor(self, tags):
        if self.R is not None or self.H is not None:
            return True
        if not all(int(i) in tags for i in self.floor["tags"]):
            return False
        obj, img = [], []
        for tag_id, points in self.floor["tags"].items():
            points = np.asarray(points, float)
            if points.shape != (4, 2):
                raise ValueError("floor tags need four TL TR BR BL corners in mm")
            obj.extend(np.column_stack((points, np.zeros(4))))
            img.extend(tags[int(tag_id)])
        obj, img = np.asarray(obj, float), np.asarray(img, float)
        ok, rv, tv = cv2.solvePnP(obj, img, self.K, self.dist)
        if not ok:
            return False
        R = cv2.Rodrigues(rv)[0]
        C = (-R.T @ tv).ravel()
        projected = cv2.projectPoints(obj, rv, tv, self.K, self.dist)[0].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((projected - img) ** 2, axis=1))))
        if rms > self.max_rms or C[2] <= self.height or np.any((obj @ R.T + tv.ravel())[:, 2] <= 0):
            return False
        self.R, self.C, self.floor_rms = R, C, rms
        return True

    def plane_points(self, pixels):
        uv = self.undistort(pixels)
        if self.R is not None:
            rays = np.column_stack((uv, np.ones(len(uv)))) @ np.linalg.inv(self.K).T @ self.R
            scale = (self.height - self.C[2]) / rays[:, 2]
            if np.any(scale <= 0):
                raise ValueError("tag ray does not meet the plane in front of camera")
            return (self.C + scale[:, None] * rays)[:, :2]
        xy = cv2.perspectiveTransform(uv.reshape(1, -1, 2), self.H)[0]
        if self.height:
            xy = self.C[:2] + (xy - self.C[:2]) * (self.C[2] - self.height) / self.C[2]
        return xy

    def process(self, image, timestamp, index):
        if image.shape[1::-1] != self.size:
            raise ValueError(f"image {image.shape[1::-1]} differs from calibration {self.size}")
        corners, ids, rejected = self.detector.detectMarkers(image)
        tags = {}
        if ids is not None:
            for i in np.unique(ids):
                matches = np.flatnonzero(ids.ravel() == i)
                if len(matches) == 1:
                    tags[int(i)] = corners[matches[0]].reshape(4, 2)
        row = dict(t=float(timestamp), frame=int(index), detected=False,
                   quality=dict(tag_ids=sorted(tags), rejected_candidates=len(rejected),
                                floor_ready=False))
        ready = self.establish_floor(tags)
        row["quality"].update(floor_ready=ready, floor_rms_px=self.floor_rms if self.R is not None else None,
                              homography_rms_mm=self.floor_rms if self.H is not None else None)
        if not ready or self.robot_id not in tags:
            row["quality"]["reason"] = "floor_unavailable" if not ready else "robot_missing"
            return row
        pix = tags[self.robot_id]
        side = float(np.min(np.linalg.norm(pix - np.roll(pix, 1, axis=0), axis=1)))
        xy = self.plane_points(pix)
        center = xy.mean(axis=0)
        right = (xy[1] + xy[2] - xy[0] - xy[3]) / 2
        yaw = np.arctan2(right[1], right[0])
        expected = tag_corners(center, self.tag_mm, yaw)
        shape_rms = float(np.sqrt(np.mean(np.sum((expected - xy) ** 2, axis=1))))
        row["quality"].update(min_side_px=side, square_rms_mm=shape_rms)
        if side < self.min_side or shape_rms > 0.1 * self.tag_mm:
            row["quality"]["reason"] = "small_or_deformed_tag"
            return row
        theta = float(np.arctan2(np.sin(yaw - self.tag_yaw), np.cos(yaw - self.tag_yaw)))
        c, s = np.cos(theta), np.sin(theta)
        body_center = center - np.array([[c, -s], [s, c]]) @ self.offset
        row.update(detected=True, x_mm=float(body_center[0]), y_mm=float(body_center[1]),
                   theta_rad=theta)
        return row


def print_tags(args, floor):
    from reportlab.lib.utils import ImageReader
    from reportlab.pdfgen import canvas
    sizes = {args.robot_id: args.tag_mm}
    for i, corners in floor.get("tags", {}).items():
        p = np.asarray(corners)
        sizes[int(i)] = float(np.linalg.norm(p[1] - p[0]))
    args.print_tags.mkdir(parents=True, exist_ok=True)
    for i, side in sizes.items():
        pixels = round(side / 25.4 * 300)
        border = round(pixels / 4)
        image = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(april_dictionary(), i, pixels),
                                  border, border, border, border, cv2.BORDER_CONSTANT, value=255)
        path = args.print_tags / f"tag-{i}"
        cv2.imwrite(str(path.with_suffix(".png")), image)
        page_mm = side * (1 + 2 * border / pixels)
        scale = 72 / 25.4
        pdf = canvas.Canvas(str(path.with_suffix(".pdf")), pagesize=(page_mm * scale,) * 2)
        pdf.drawImage(ImageReader(io.BytesIO(cv2.imencode(".png", image)[1].tobytes())),
                      0, 0, width=page_mm * scale, height=page_mm * scale)
        pdf.showPage()
        pdf.save()
        print(f"tag {i}: black outer square {side:g} mm; print PDF at 100%")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--floor", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--camera", type=int, default=0)
    source.add_argument("--video", type=Path)
    parser.add_argument("--video-start-t", type=float, help="original first frame's monotonic time")
    parser.add_argument("--exposure", type=float, default=50)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--tag-mm", type=float, default=100)
    parser.add_argument("--tag-height-mm", type=float, default=0)
    parser.add_argument("--tag-offset-mm", type=float, nargs=2, default=(0, 0))
    parser.add_argument("--tag-yaw-rad", type=float, default=0)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--print-tags", type=Path, help="write printable PNG/PDF tags; no camera")
    args = parser.parse_args(argv)
    floor = load_config(args.floor)
    if args.print_tags:
        print_tags(args, floor)
        return
    if args.calibration is None or args.out is None:
        parser.error("tracking needs --calibration and --out")
    if args.video_start_t is not None and args.video is None:
        parser.error("--video-start-t requires --video")
    tracker = Tracker(camera_model(args.calibration)[0], floor, args.robot_id, args.tag_mm,
                      args.tag_height_mm, args.tag_offset_mm, args.tag_yaw_rad)
    cap = open_source(args.camera, args.video, args.exposure)
    fps = cap.get(cv2.CAP_PROP_FPS)
    if args.video and args.video_start_t is not None and fps <= 0:
        cap.release()
        raise ValueError("video has no valid FPS for timeline reconstruction")
    if args.video and args.video_start_t is None:
        print("video: replay grab timestamps; use --video-start-t to reconstruct capture time")
    start, index, detected = time.monotonic(), 0, 0
    try:
        with args.out.open("w") as output:
            while args.max_frames is None or index < args.max_frames:
                if not cap.grab():
                    break
                grab_t = time.monotonic()
                ok, image = cap.retrieve()
                if not ok:
                    break
                timestamp = args.video_start_t + index / fps if args.video_start_t is not None else grab_t
                row = tracker.process(image, timestamp, index)
                row.update(clock="video_timeline" if args.video_start_t is not None else "host_monotonic",
                           source="video" if args.video else "camera", grab_t=grab_t)
                output.write(json.dumps(row, allow_nan=False) + "\n")
                output.flush()
                index += 1
                detected += row["detected"]
                if index % 150 == 0:
                    print(f"{index} frames, achieved {index / (time.monotonic() - start):.2f} fps, {detected} poses")
                if args.preview:
                    cv2.putText(image, f"frame {index} pose {row.get('detected')}", (20, 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.imshow("rig (q to stop)", image)
                    if cv2.waitKey(1) & 255 == ord("q"):
                        break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if args.preview:
            cv2.destroyAllWindows()
    print(f"{index} frames, {detected} poses; achieved {index / max(time.monotonic() - start, 1e-6):.2f} fps")


if __name__ == "__main__":
    main()
