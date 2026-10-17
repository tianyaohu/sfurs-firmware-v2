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
    poses, grid = [], np.zeros((3, 3), int)
    # Views already in --out count toward coverage and de-duplication, so top-up sessions show
    # the cumulative map.
    for old in sorted(args.out.glob("view-*.png")):
        image = cv2.imread(str(old))
        corners, ids, _, _ = detector.detectBoard(image)
        if ids is not None and len(ids) >= 8:
            pose = view_pose(corners, image.shape)
            poses.append(pose)
            grid[min(2, int(pose[1] * 3)), min(2, int(pose[0] * 3))] += 1
    if poses:
        print(f"{len(poses)} existing views in {args.out}; coverage so far (top row first):\n"
              + "\n".join(" ".join(f"{n:3d}" for n in row) for row in grid))
    try:
        while saved < args.count:
            ok, image = cap.read()
            if not ok:
                break
            frames += 1
            corners, ids, _, _ = detector.detectBoard(image)
            if ids is not None and len(ids) >= 8 and time.monotonic() - last >= args.interval:
                pose = view_pose(corners, image.shape)
                if all(distinct(pose, p) for p in poses):
                    path = args.out / f"view-{time.monotonic_ns()}.png"
                    if not cv2.imwrite(str(path), image):
                        raise RuntimeError(f"cannot write {path}")
                    saved += 1
                    poses.append(pose)
                    grid[min(2, int(pose[1] * 3)), min(2, int(pose[0] * 3))] += 1
                    print(f"saved {saved}/{args.count}: {path}")
                last = time.monotonic()
            if args.preview:
                if ids is not None:
                    cv2.aruco.drawDetectedCornersCharuco(image, corners, ids)
                h, w = image.shape[:2]
                for r in range(3):
                    for c in range(3):
                        cv2.putText(image, str(grid[r, c]), (int((c + 0.5) * w / 3), int((r + 0.5) * h / 3)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255) if grid[r, c] == 0 else (0, 200, 0), 2)
                cv2.imshow("calibration (q to stop)", image)
                if cv2.waitKey(1) & 255 == ord("q"):
                    break
    finally:
        cap.release()
        if args.preview:
            cv2.destroyAllWindows()
    print(f"{saved} views; achieved {frames / max(time.monotonic() - start, 1e-6):.2f} fps")
    print("views per image region (top row first):\n" + "\n".join(" ".join(f"{n:3d}" for n in row) for row in grid))
    if saved and (grid == 0).any():
        print("WARNING: some regions have no views; cover every region, especially the corners")


def view_pose(corners, shape):
    """Board center (fraction of width/height), size (fraction of width), in-plane angle and aspect."""
    pts = corners.reshape(-1, 2)
    h, w = shape[:2]
    rect = cv2.minAreaRect(pts.astype(np.float32))
    (rw, rh) = rect[1]
    return (pts[:, 0].mean() / w, pts[:, 1].mean() / h, max(rw, rh) / w, rect[2], min(rw, rh) / max(rw, rh, 1e-6))


def distinct(a, b):
    # A board held still produces identical views that add nothing to the fit.
    return (np.hypot(a[0] - b[0], a[1] - b[1]) > 0.08 or abs(a[2] - b[2]) > 0.2 * b[2]
            or abs(a[3] - b[3]) > 10 or abs(a[4] - b[4]) > 0.1)


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
    # k3 is poorly constrained for ordinary webcam lenses and ran away (k3=878) on real views.
    # Start from a typical webcam (~70 deg HFOV, centered); OpenCV's own planar init is poor for small boards.
    flags = cv2.CALIB_USE_INTRINSIC_GUESS | (0 if args.full_distortion else cv2.CALIB_FIX_K3)
    guess = np.array([[0.7 * size[0], 0, size[0] / 2], [0, 0.7 * size[0], size[1] / 2], [0, 0, 1]])
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objects, images, size, guess, None, flags=flags)
    errors = [float(np.sqrt(np.mean(np.sum((cv2.projectPoints(o, r, t, K, dist)[0] - i)
                                          ** 2, axis=-1))))
              for o, i, r, t in zip(objects, images, rvecs, tvecs)]
    save_config(args.out, dict(image_size=list(size), K=K.tolist(), dist=dist.ravel().tolist(),
                rms_px=float(rms), views=len(objects), per_view_rms_px=dict(zip(names, errors)),
                board=dict(cols=args.cols, rows=args.rows, square_mm=args.square_mm,
                           marker_mm=args.marker_mm, dictionary="DICT_5X5_100")))
    print(f"{len(objects)}/{len(paths)} usable views; RMS {rms:.4f} px; saved {args.out}")
    # Low RMS does not mean a valid lens model: near-identical views overfit with tiny residuals.
    problems = []
    if abs(K[0, 0] / K[1, 1] - 1) > 0.03:
        problems.append(f"fx/fy = {K[0, 0] / K[1, 1]:.3f} (square pixels should give ~1.00)")
    if np.max(np.abs(dist.ravel())) > 5:
        problems.append(f"distortion coefficients up to {np.max(np.abs(dist.ravel())):.1f} (expect < ~2)")
    widths = [np.ptp(np.asarray(i).reshape(-1, 2)[:, 0]) / size[0] for i in images]
    if np.median(widths) < 0.25:
        problems.append(f"board spans a median {100 * np.median(widths):.0f}% of image width "
                        "(hold it closer: aim for 30-60%)")
    if problems:
        print("WARNING: implausible calibration, recapture with more varied views:\n  " + "\n  ".join(problems))


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
            p.add_argument("--full-distortion", action="store_true", help="also fit k3")
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
