#!/usr/bin/env python3
#
# Copyright (c) 2026 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://github.com/nepi-engine/nepi_apps)
#
# License: NEPI Applications repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

"""Custom stereo depth library for NEPI.

Produces a metric depth map (distance in millimeters) from a rectified
left/right stereo image pair using OpenCV block matching (CPU, any board) or,
on an NVIDIA Jetson, NVIDIA VPI's semi-global matcher on the GPU (matcher
'vpi' -- see the VPI section below; optional, never loaded elsewhere).

Assumptions
-----------
* The two input images are the same aspect ratio / pixel resolution.
* The images are already RECTIFIED (epipolar lines are horizontal image
  rows).  Block matching relies on this.  If your raw cameras are not
  rectified, calibrate them and pass rectified frames in.
* ``focal_length_px`` and ``baseline_mm`` come from stereo calibration and
  are what turn a raw disparity into a real-world distance.
"""

from __future__ import annotations

import os
import importlib.util

import numpy as np
# cv2 MUST be imported before vpi in any process that uses both -- see the VPI
# section below. Importing it here, at module load, guarantees that for this module.
import cv2


DEFAULT_SETTINGS = {
    ## - Camera Geometry
    # Baseline: distance between the two camera optical centers, in mm.
    "baseline_mm": 60.0,
    # Focal length in px (fx from the camera matrix).
    "focal_length_px": 700.0,
    # sgbm (semi-global, higher quality) or bm (faster, noisier, grayscale only),
    # both on the CPU; or vpi (semi-global on an NVIDIA Jetson GPU, grayscale only).
    "matcher": "sgbm",
    # Convert color inputs to grayscale before matching (required for BM and VPI).
    "convert_to_grayscale": True,

    # - Block matching parameters
    
    #disparity = the difference in pixel locations of said object between 2 cameras
    #minimum disparity allowed, only change if objects are significantly far away from camera, where near-zero disparities never occur.
    "min_disparity": 0,
    # Range of disparities to search.  MUST be a multiple of 16. Larger = can measure closer objects (larger disparities)
    "num_disparities": 128,
    # side length of the matching window. Odd, typically 3-11 for SGBM, 5-21 for BM.
    "block_size": 5,
    # a match must beat the second-best candidate by at least this percent margin, or it is rejected. 0 disables.
    "uniqueness_ratio": 10,
    # max size (in pixels) of a connected disparity blob that gets treated as noise and cleared. 0 disables speckle filtering.
    "speckle_window_size": 100,
    # how much disparity variation is allowed within a blob for it to count as one connected region during speckle filtering.
    "speckle_range": 2,
    # Occlusion filter: left-right consistency check (if no match between pixels,); -1 disables.
    "disp12_max_diff": 1,
    # reduces noise to prevent large disparity jumps due to image noise; higher values = retained sharper texture details; 1-63 is typical
    "pre_filter_cap": 63,

    # - Depth post-processing
    # Distances outside this range are treated as invalid (set to 0).
    "min_depth_mm": 50,
    "max_depth_mm": 5000,
    # k*k median filter applied to the depth map to remove isolated depth pixels without blurring actual edges; must be odd, 0 disables.
    "median_filter_size": 5,

    # - NVIDIA VPI (GPU) matcher only; sgbm/bm ignore these. See _vpi_disparity.
    # Smoothness penalties on VPI's census cost scale (NOT the SGBM P1/P2 scale): P1 <= P2 < 256.
    "vpi_p1": 3,
    "vpi_p2": 48,
    # Best match must beat the second best by this cost ratio; lower rejects more. 1.0 disables.
    "vpi_uniqueness": 0.95,
    # Pixels whose match confidence (0-65535) is below this are invalid.
    "vpi_confidence_threshold": 32767,
    # Aggregate along diagonal paths as well as rows/columns: better quality, slower.
    "vpi_include_diagonals": True,
}


# Internal helpers
def _as_grayscale(image: np.ndarray) -> np.ndarray:
    #Return a single-channel uint8 image
    if image.ndim == 3:
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image


# Matchers that only take a single-channel pair, whatever convert_to_grayscale says.
GRAYSCALE_ONLY_MATCHERS = ("bm", "vpi")


def needs_grayscale(settings: dict) -> bool:
    """True if compute_depth_map will match this configuration in grayscale.

    The one statement of the rule, so a caller that wants to hand over a gray pair
    up front (stereo_settings.matches_in_grayscale) cannot drift from what
    compute_depth_map actually does.
    """
    cfg = {**DEFAULT_SETTINGS, **(settings or {})}
    return (bool(cfg["convert_to_grayscale"]) or
            str(cfg["matcher"]).lower() in GRAYSCALE_ONLY_MATCHERS)

def _build_matcher(settings: dict):
    #Construct an OpenCV stereo matcher from settings
    num_disparities = int(settings["num_disparities"])
    if num_disparities % 16 != 0 or num_disparities <= 0:
        raise ValueError(
            f"num_disparities must be a positive multiple of 16, got {num_disparities}"
        )
    block_size = int(settings["block_size"])
    if block_size % 2 == 0:
        raise ValueError(f"block_size must be odd, current block_size: {block_size}")

    matcher_type = settings["matcher"].lower()
    if matcher_type == "bm":
        matcher = cv2.StereoBM_create(
            numDisparities=num_disparities,
            blockSize=block_size,
        )
        matcher.setMinDisparity(int(settings["min_disparity"]))
        matcher.setUniquenessRatio(int(settings["uniqueness_ratio"]))
        matcher.setSpeckleWindowSize(int(settings["speckle_window_size"]))
        matcher.setSpeckleRange(int(settings["speckle_range"]))
        matcher.setDisp12MaxDiff(int(settings["disp12_max_diff"]))
        matcher.setPreFilterCap(int(settings["pre_filter_cap"]))
        return matcher

    if matcher_type == "sgbm":
        channels = 1 if settings["convert_to_grayscale"] else 3
        return cv2.StereoSGBM_create(
            minDisparity=int(settings["min_disparity"]),
            numDisparities=num_disparities,
            blockSize=block_size,
            # P1/P2 control smoothness, standard heuristic from OpenCV docs. SGBM strictly requires p2 > p1, so big change in depth is penalized over small changes
            P1=8 * channels * block_size ** 2, # P1: penalty on disparity change by +/- 1 between neighboring pixels
            P2=32 * channels * block_size ** 2, # P2: penalty on disparity change by more than 1 between neighboring pixels
            disp12MaxDiff=int(settings["disp12_max_diff"]),
            uniquenessRatio=int(settings["uniqueness_ratio"]),
            speckleWindowSize=int(settings["speckle_window_size"]),
            speckleRange=int(settings["speckle_range"]),
            preFilterCap=int(settings["pre_filter_cap"]),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    raise ValueError(f"Unknown matcher '{settings['matcher']}' (use 'sgbm', 'bm' or 'vpi')")


def _disparity_to_depth_mm(disparity: np.ndarray, settings: dict) -> np.ndarray:
    # Z_mm = focal_length_px * baseline_mm / disparity_px
    
    focal_px = np.float32(settings["focal_length_px"])
    baseline_mm = np.float32(settings["baseline_mm"])

    depth = np.zeros_like(disparity, dtype=np.float32)
    valid = disparity > 0  # non-positive disparity => no/invalid match
    depth[valid] = (focal_px * baseline_mm) / disparity[valid]

    # Clamp to the configured usable range; out-of-range = invalid (0).
    min_mm = float(settings["min_depth_mm"])
    max_mm = float(settings["max_depth_mm"])
    depth[(depth < min_mm) | (depth > max_mm)] = 0.0
    return depth


# NVIDIA VPI (GPU) matcher -- optional, Jetson only
#
# On a board without NVIDIA's VPI python bindings or an NVIDIA GPU (a Raspberry
# Pi 5, most desktops) vpi_available() is False, stereo_settings registers no GPU
# process, and nothing below ever runs. The CPU matchers above do not depend on
# any of it.
#
# Facts measured on a JetPack 5.1 Orin NX in the NEPI 3.3 container (VPI 2.3,
# 2026-11-17) that this code is written around:
#   * `import vpi` takes ~3 s and ~270 MB resident, so it happens lazily on the
#     first GPU pass, never at module load. vpi_available() only looks up the
#     module and a GPU device node.
#   * The container exports DISPLAY=:0 with no reachable X server, and VPI's import
#     initializes EGL against it: ImportError VPI_ERROR_INTERNAL (EGL_BAD_DISPLAY).
#     DISPLAY is hidden for the import only and restored straight after.
#   * Importing vpi BEFORE cv2 makes a later cv2.remap abort the whole process
#     ("double free or corruption") -- VPI drags JetPack's system OpenCV in beside
#     the container's. cv2 is imported at the top of this module, so it is always
#     first here; never move the vpi import above it.
#   * Output is int16 Q10.5 (divide by 32). A pixel with no match is 0x7FFF, not 0,
#     which read as a disparity is ~1024 px -- a false NEAR depth that the min-depth
#     clamp does not catch on a long-focal rig. It is zeroed before anything else.
#   * maxdisp is rounded UP to a multiple of 64 internally (16, 48 and 64 all
#     search 0..63; 80..128 all search 0..127), so only multiples of 64 are
#     accepted -- anything else would make the app's measurable-range report wrong.
#     mindisp is not used: with it set, correct matches dropped to 40-90%.
#   * The census window argument has no effect on the CUDA backend; uniqueness 1.0
#     is identical to disabled; P1 <= P2 < 256 or VPI raises.
#   * Calling from a new thread every pass (the depth loop's oneshot timers) works
#     and does not leak beyond the thread churn the CPU matchers already have.

VPI_INVALID_DISPARITY = 0x7FFF
VPI_DISPARITY_STEP = 64
VPI_MAX_DISPARITY = 256

_vpi_module = None
_vpi_load_error = None


class MatcherUnavailableError(RuntimeError):
    """The selected matcher cannot run on this board at all -- not a bad parameter."""


# Device nodes an NVIDIA GPU driver creates: Jetson iGPU (JetPack 5 and 6) and
# discrete GPUs. VPI being installed is not enough on its own -- an x86 dev box
# can carry VPI with no usable CUDA driver, and its import then fails with
# cudaErrorInsufficientDriver.
_NVIDIA_GPU_NODES = ("/dev/nvhost-ctrl-gpu", "/dev/nvgpu", "/dev/nvidia0")


def vpi_available() -> bool:
    """True if NVIDIA VPI's python module is installed AND an NVIDIA GPU is visible.

    Cheap: looks the module up and checks for a GPU device node, without importing
    VPI. The import can still fail (see _load_vpi), but not on a board with no
    NVIDIA GPU at all, such as a Raspberry Pi.
    """
    try:
        if importlib.util.find_spec("vpi") is None:
            return False
    except (ImportError, ValueError):
        return False
    return any(os.path.exists(node) for node in _NVIDIA_GPU_NODES)


def _load_vpi():
    """Import vpi once, on first use. Raises MatcherUnavailableError if it cannot load.

    A failure is remembered, so a broken install costs one attempt per node start,
    not one per depth pass.
    """
    global _vpi_module, _vpi_load_error
    if _vpi_module is not None:
        return _vpi_module
    if _vpi_load_error is not None:
        raise MatcherUnavailableError(_vpi_load_error)
    display = os.environ.pop("DISPLAY", None)
    try:
        import vpi
        _vpi_module = vpi
    except Exception as e:
        _vpi_load_error = (
            "the GPU matcher could not load NVIDIA VPI (" + type(e).__name__ + ": " +
            str(e) + "). Pick a CPU process (sgbm_1 or bm_1); restart the app after "
            "fixing VPI to try again.")
        raise MatcherUnavailableError(_vpi_load_error)
    finally:
        if display is not None:
            os.environ["DISPLAY"] = display
    return _vpi_module


def _vpi_disparity(left: np.ndarray, right: np.ndarray, settings: dict) -> np.ndarray:
    """Disparity in px (float32, 0.0 = no match) from VPI's CUDA semi-global matcher.

    left/right are single-channel uint8. Speckle filtering runs on the CPU after the
    GPU match, with the same meaning as the SGBM/BM controls (speckle_range in
    disparity px), because VPI has no speckle filter of its own.
    """
    num_disparities = int(settings["num_disparities"])
    if (num_disparities <= 0 or num_disparities % VPI_DISPARITY_STEP != 0 or
            num_disparities > VPI_MAX_DISPARITY):
        raise ValueError(
            f"num_disparities for the GPU matcher must be a multiple of "
            f"{VPI_DISPARITY_STEP} up to {VPI_MAX_DISPARITY}, got {num_disparities}")
    vpi = _load_vpi()
    with vpi.Backend.CUDA:
        out = vpi.stereodisp(
            vpi.asimage(np.ascontiguousarray(left)),
            vpi.asimage(np.ascontiguousarray(right)),
            maxdisp=num_disparities,
            p1=int(settings["vpi_p1"]),
            p2=int(settings["vpi_p2"]),
            uniqueness=float(settings["vpi_uniqueness"]),
            confthreshold=int(settings["vpi_confidence_threshold"]),
            includediagonals=bool(settings["vpi_include_diagonals"]),
        )
        # Own the buffer: filterSpeckles writes in place.
        raw = np.array(out.cpu(), dtype=np.int16, copy=True)
    raw[raw == VPI_INVALID_DISPARITY] = 0

    speckle_size = int(settings["speckle_window_size"])
    if speckle_size > 0:
        # Q10.5, so one disparity px is 32 units.
        cv2.filterSpeckles(raw, 0, speckle_size, int(settings["speckle_range"]) * 32)
    return raw.astype(np.float32) / 32.0



# Main entry point
def compute_depth_map(left_image: np.ndarray, right_image: np.ndarray, settings: dict = None,) -> np.ndarray:
    """Compute a metric depth map from a rectified stereo pair.
    Parameters:

    left_image, right_image : np.ndarray
        cv2 images (BGR or grayscale) from the LEFT and RIGHT cameras.
        Must have identical height / width (resolution).
    settings : dict, optional
        User settings; missing keys are filled from DEFAULT_SETTINGS dict.

    Returns:

    np.ndarray
        (H, W) float32 matrix.  Each entry is the distance to that
        pixel in mm; 0.0 marks pixels with no valid depth.
    """
    if settings is None:
        settings = {}
    # Merge user overrides onto defaults so a partial dict is fine.
    cfg = {**DEFAULT_SETTINGS, **settings}

    if left_image is None or right_image is None:
        raise ValueError("left_image and right_image must both be provided")
    if left_image.shape[:2] != right_image.shape[:2]:
        raise ValueError(
            f"Left/right resolution mismatch: "
            f"{left_image.shape[:2]} vs {right_image.shape[:2]}"
        )

    left = right = None
    if needs_grayscale(cfg):
        left = _as_grayscale(left_image)
        right = _as_grayscale(right_image)
    else:
        left, right = left_image, right_image

    if cfg["matcher"].lower() == "vpi":
        disparity = _vpi_disparity(left, right, cfg)
    else:
        matcher = _build_matcher(cfg)
        # BM/SGBM return disparity as int16 fixed-point scaled by 16.
        raw_disparity = matcher.compute(left, right)
        disparity = raw_disparity.astype(np.float32) / 16.0

    depth_mm = _disparity_to_depth_mm(disparity, cfg)

    ksize = int(cfg["median_filter_size"])
    if ksize >= 3 and ksize % 2 == 1:
        depth_mm = cv2.medianBlur(depth_mm, ksize)

    return depth_mm



#colorized preview for humans (not part of the depth output)
def colorize_depth(depth_mm: np.ndarray, settings: dict = None) -> np.ndarray:
    """Return a BGR uint8 image visualizing the depth map (for debugging)."""
    cfg = {**DEFAULT_SETTINGS, **(settings or {})}
    min_mm, max_mm = float(cfg["min_depth_mm"]), float(cfg["max_depth_mm"])
    norm = np.clip((depth_mm - min_mm) / max(max_mm - min_mm, 1e-6), 0.0, 1.0)
    norm_u8 = (norm * 255).astype(np.uint8)
    colored = cv2.applyColorMap(norm_u8, cv2.COLORMAP_JET)
    colored[depth_mm <= 0.0] = 0  # black = invalid
    return colored
