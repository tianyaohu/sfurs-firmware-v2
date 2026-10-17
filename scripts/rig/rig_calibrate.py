#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python>=4.10,<5", "numpy>=1.26", "PyYAML>=6", "reportlab>=4"]
# ///
"""Print a ChArUco board, collect diverse views, then solve camera intrinsics.

  uv run scripts/rig/rig_calibrate.py board --out /tmp/board
  uv run scripts/rig/rig_calibrate.py capture --out /tmp/views --count 30
  uv run scripts/rig/rig_calibrate.py solve --images '/tmp/views/*.png' --out /tmp/camera.yaml
"""
import argparse
import glob
import io
import time
from pathlib import Path

import cv2
import numpy as np

from rig_common import open_source, save_config


def make_board(args):
    if not 0 < args.marker_mm < args.square_mm:
        raise ValueError("require 0 < marker-mm < square-mm")
    return cv2.aruco.CharucoBoard((args.cols, args.rows), args.square_mm, args.marker_mm,
                                 cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100))


def print_board(args, board):
    from reportlab.lib.utils import ImageReader
    from reportlab.pdfgen import canvas
    width, height = args.cols * args.square_mm, args.rows * args.square_mm
    size = tuple(round(mm / 25.4 * args.dpi) for mm in (width, height))
    image = board.generateImage(size, marginSize=0, borderBits=1)
    png, pdf = args.out.with_suffix(".png"), args.out.with_suffix(".pdf")
    if not cv2.imwrite(str(png), image):
        raise RuntimeError(f"cannot write {png}")
    # PDF carries physical dimensions; PNG viewers may silently rescale a print.
    scale, margin = 72 / 25.4, 10
    page = canvas.Canvas(str(pdf), pagesize=((width + 2 * margin) * scale,
                                           (height + 2 * margin) * scale))
    encoded = cv2.imencode(".png", image)[1].tobytes()
    page.drawImage(ImageReader(io.BytesIO(encoded)), margin * scale, margin * scale,
                   width=width * scale, height=height * scale)
    page.showPage()
    page.save()
    save_config(args.out.with_suffix(".json"), dict(cols=args.cols, rows=args.rows,
                square_mm=args.square_mm, marker_mm=args.marker_mm, dictionary="DICT_5X5_100",
                board_size_mm=[width, height], dpi=args.dpi))
    print(f"{png}, {pdf}: board {width:g} x {height:g} mm; print PDF at 100%")


def capture(args, board):
    args.out.mkdir(parents=True, exist_ok=True)
    detector = cv2.aruco.CharucoDetector(board)
    cap = open_source(args.camera, args.video, args.exposure)
    saved, last = 0, -float("inf")
    start, frames = time.monotonic(), 0
    try:
        while saved < args.count:
            ok, image = cap.read()
            if not ok:
                break
            frames += 1
            corners, ids, _, _ = detector.detectBoard(image)
            if ids is not None and len(ids) >= 8 and time.monotonic() - last >= args.interval:
                path = args.out / f"view-{time.monotonic_ns()}.png"
                if not cv2.imwrite(str(path), image):
                    raise RuntimeError(f"cannot write {path}")
                saved += 1
                last = time.monotonic()
                print(f"saved {saved}/{args.count}: {path}")
            if args.preview:
                if ids is not None:
                    cv2.aruco.drawDetectedCornersCharuco(image, corners, ids)
                cv2.imshow("calibration (q to stop)", image)
                if cv2.waitKey(1) & 255 == ord("q"):
                    break
    finally:
        cap.release()
        if args.preview:
            cv2.destroyAllWindows()
    print(f"{saved} views; achieved {frames / max(time.monotonic() - start, 1e-6):.2f} fps")


def solve(args, board):
    detector = cv2.aruco.CharucoDetector(board)
    objects, images, names, size = [], [], [], None
    paths = sorted({p for pattern in args.images for p in glob.glob(pattern)})
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"cannot read {path}")
        this_size = image.shape[1::-1]
        if size is not None and this_size != size:
            raise ValueError("calibration images have different dimensions")
        size = this_size
        corners, ids, _, _ = detector.detectBoard(image)
        if ids is not None and len(ids) >= 8:
            obj, img = board.matchImagePoints(corners, ids)
            objects.append(obj)
            images.append(img)
            names.append(path)
    if len(objects) < 8:
        raise ValueError(f"need >=8 usable, diverse views; found {len(objects)}")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objects, images, size, None, None)
    errors = [float(np.sqrt(np.mean(np.sum((cv2.projectPoints(o, r, t, K, dist)[0] - i)
                                          ** 2, axis=-1))))
              for o, i, r, t in zip(objects, images, rvecs, tvecs)]
    save_config(args.out, dict(image_size=list(size), K=K.tolist(), dist=dist.ravel().tolist(),
                rms_px=float(rms), views=len(objects), per_view_rms_px=dict(zip(names, errors)),
                board=dict(cols=args.cols, rows=args.rows, square_mm=args.square_mm,
                           marker_mm=args.marker_mm, dictionary="DICT_5X5_100")))
    print(f"{len(objects)}/{len(paths)} usable views; RMS {rms:.4f} px; saved {args.out}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("board", "capture", "solve"):
        p = sub.add_parser(name)
        p.add_argument("--cols", type=int, default=7)
        p.add_argument("--rows", type=int, default=5)
        p.add_argument("--square-mm", type=float, default=30)
        p.add_argument("--marker-mm", type=float, default=22)
        p.add_argument("--out", type=Path, required=True)
        if name == "board":
            p.add_argument("--dpi", type=int, default=300)
        elif name == "capture":
            source = p.add_mutually_exclusive_group()
            source.add_argument("--camera", type=int, default=0)
            source.add_argument("--video", type=Path)
            p.add_argument("--exposure", type=float, default=50)
            p.add_argument("--count", type=int, default=30)
            p.add_argument("--interval", type=float, default=1.0)
            p.add_argument("--preview", action="store_true")
        else:
            p.add_argument("--images", nargs="+", required=True)
    args = parser.parse_args()
    board = make_board(args)
    if args.command == "board":
        print_board(args, board)
    elif args.command == "capture":
        capture(args, board)
    else:
        solve(args, board)


if __name__ == "__main__":
    main()
