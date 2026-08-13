#!/usr/bin/env python3
"""Convert a TUM RGB-D sequence into NEPI file_pub_depthmap collections.

Emits the three-file collection the app scans for, per associated frame pair:

    <stamp>_<name>-color_image.png       BGR, cv2-readable
    <stamp>_<name>-depth_map.npy         float32 MILLIMETERS
    <stamp>_<name>-depth_map_image.png   colorized depth for display

plus one nepi_collection_settings.yaml for the folder as a whole, carrying the
field of view computed from the source camera intrinsics.  That file is how the
converted set tells the app what settings it needs, so no dataset is named
anywhere in the app.  Its name matches none of the three collection suffixes
above, which is what keeps it invisible to the app's sort-order grouping.

TUM depth is uint16 PNG where value/5000 = meters.  NEPI depth maps as written
by SaveDataIF are float32 millimeters with no-return pixels clamped to the
sensor max range (verified against zed2 captures: 0 NaN, 0 inf, 0 zeros, ~13%
of pixels sitting exactly at 20000.0).  This matches that convention so the
converted set behaves like a real capture rather than a special case.

The three names share a stamp prefix and sort as color_image < depth_map.npy <
depth_map_image.png under Python's byte ordering, which is what the app's
sort-order grouping in buildCollections() requires.
"""

import argparse
import math
import os
import sys
import datetime

import numpy as np
import cv2
import yaml

TUM_DEPTH_SCALE = 5000.0  # TUM uint16 -> meters

# The settings sidecar the app reads.  Same name as
# NepiFilePubDepthmapApp.FOLDER_SETTINGS_FILE; the app reads it with
# nepi_utils.read_yaml_2_dict().  This script is an offline host-side tool with
# no ROS or nepi_sdk dependency, so it writes with pyyaml directly.
SETTINGS_FILE = 'nepi_collection_settings.yaml'

# Default pinhole intrinsics: the TUM freiburg2 camera.  Overridable for any
# other source, which is the point of computing the FOV rather than assuming it.
FREIBURG2_FX = 520.9
FREIBURG2_FY = 521.0


def fov_deg(focal_px, extent_px):
    """Full angular extent in degrees of a pinhole camera, from one focal axis."""
    return math.degrees(2 * math.atan(extent_px / (2.0 * focal_px)))


def write_settings(dst, width_deg, height_deg, description):
    """Write the folder settings sidecar the app applies on folder selection."""
    settings = {
        'width_deg': round(float(width_deg), 1),
        'height_deg': round(float(height_deg), 1),
        'description': str(description),
    }
    path = os.path.join(dst, SETTINGS_FILE)
    with open(path, 'w') as f:
        yaml.dump(settings, f, default_flow_style=False, sort_keys=False)
    return path, settings


def read_index(path):
    """Parse a TUM rgb.txt / depth.txt index into [(timestamp, relpath)]."""
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 2:
                out.append((float(parts[0]), parts[1]))
    return out


def associate(rgb, depth, max_diff):
    """Pair each depth frame with its nearest rgb frame within max_diff seconds."""
    pairs = []
    rgb_t = [r[0] for r in rgb]
    for dt, dpath in depth:
        i = np.searchsorted(rgb_t, dt)
        best, best_d = None, None
        for j in (i - 1, i):
            if 0 <= j < len(rgb):
                diff = abs(rgb_t[j] - dt)
                if best_d is None or diff < best_d:
                    best, best_d = j, diff
        if best is not None and best_d <= max_diff:
            pairs.append((dt, dpath, rgb[best][1]))
    return pairs


def nepi_stamp(ts):
    """NEPI SaveDataIF-style UTC timestamp token: D2011-09-28T15-33-20p398TzUTC."""
    dt = datetime.datetime.utcfromtimestamp(ts)
    return 'D%sp%03dTzUTC' % (dt.strftime('%Y-%m-%dT%H-%M-%S'),
                              dt.microsecond // 1000)


def colorize(depth_mm, lo, hi):
    """Render a depth map to a JET color image over the valid range only."""
    valid = (depth_mm > 0) & (depth_mm < hi)
    norm = np.zeros(depth_mm.shape, dtype=np.uint8)
    if valid.any():
        span = max(hi - lo, 1e-6)
        scaled = (depth_mm - lo) / span
        norm[valid] = np.clip(scaled[valid] * 255.0, 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    img[~valid] = (255, 255, 255)  # no-return renders white, as in zed2 captures
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src', help='extracted rgbd_dataset_freiburg2_* folder')
    ap.add_argument('dst', help='output collection folder')
    ap.add_argument('--name', default=None, help='node token in filenames')
    ap.add_argument('--max-range-mm', type=float, default=10000.0,
                    help='no-return sentinel; Kinect tops out near 10 m')
    ap.add_argument('--stride', type=int, default=1, help='keep every Nth frame')
    ap.add_argument('--limit', type=int, default=0, help='stop after N collections')
    ap.add_argument('--max-diff', type=float, default=0.02,
                    help='max rgb/depth timestamp offset in seconds')
    ap.add_argument('--fx', type=float, default=FREIBURG2_FX,
                    help='horizontal focal length in pixels of the source camera')
    ap.add_argument('--fy', type=float, default=FREIBURG2_FY,
                    help='vertical focal length in pixels of the source camera')
    args = ap.parse_args()

    name = args.name or os.path.basename(args.src.rstrip('/')).replace(
        'rgbd_dataset_', '')

    rgb = read_index(os.path.join(args.src, 'rgb.txt'))
    depth = read_index(os.path.join(args.src, 'depth.txt'))
    pairs = associate(rgb, depth, args.max_diff)
    print('rgb %d  depth %d  associated %d (max_diff %.3fs)'
          % (len(rgb), len(depth), len(pairs), args.max_diff))
    if not pairs:
        sys.exit('no rgb/depth pairs associated')

    pairs = pairs[::args.stride]
    if args.limit:
        pairs = pairs[:args.limit]

    os.makedirs(args.dst, exist_ok=True)
    written = 0
    skipped = 0
    valid_frac = []
    frame_shape = None

    for ts, dpath, cpath in pairs:
        raw = cv2.imread(os.path.join(args.src, dpath), cv2.IMREAD_UNCHANGED)
        color = cv2.imread(os.path.join(args.src, cpath))
        if raw is None or color is None or raw.ndim != 2:
            skipped += 1
            continue

        frame_shape = raw.shape
        good = raw > 0
        depth_mm = np.full(raw.shape, args.max_range_mm, dtype=np.float32)
        depth_mm[good] = raw[good].astype(np.float32) / TUM_DEPTH_SCALE * 1000.0
        np.clip(depth_mm, 0.0, args.max_range_mm, out=depth_mm)
        valid_frac.append(float(good.sum()) / good.size)

        base = os.path.join(args.dst, '%s_%s' % (nepi_stamp(ts), name))
        cv2.imwrite(base + '-color_image.png', color)
        np.save(base + '-depth_map.npy', depth_mm)
        lo = float(depth_mm[good].min()) if good.any() else 0.0
        cv2.imwrite(base + '-depth_map_image.png',
                    colorize(depth_mm, lo, args.max_range_mm))
        written += 1

    print('wrote %d collections to %s (skipped %d)' % (written, args.dst, skipped))
    if valid_frac:
        print('depth validity: mean %.1f%%  min %.1f%%'
              % (100 * np.mean(valid_frac), 100 * np.min(valid_frac)))

    if frame_shape is None:
        sys.exit('no frames written; no settings file emitted')

    # Source intrinsics over the actual frame size -> the FOV this folder needs.
    # Written into the folder rather than printed for the operator to re-enter,
    # so the app applies it without anyone having to know the dataset.
    h_px, w_px = frame_shape[0:2]
    description = 'TUM RGB-D %s (fx %.1f fy %.1f over %dx%d)' % (
        os.path.basename(args.src.rstrip('/')), args.fx, args.fy, w_px, h_px)
    path, settings = write_settings(args.dst,
                                    fov_deg(args.fx, w_px),
                                    fov_deg(args.fy, h_px),
                                    description)
    print('wrote %s: width_deg %.1f  height_deg %.1f'
          % (path, settings['width_deg'], settings['height_deg']))


if __name__ == '__main__':
    main()
