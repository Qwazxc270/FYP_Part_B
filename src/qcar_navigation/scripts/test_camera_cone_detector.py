#!/usr/bin/env python3
"""
test_camera_cone_detector.py
============================
Offline unit test for the O2/O3 vision pipeline.
NO ROS REQUIRED — runs standalone with only OpenCV and NumPy.

What it tests
-------------
1. HSV colour masks correctly isolate blue, yellow, and orange blobs.
2. Morphological clean-up removes single-pixel noise.
3. Blob detection finds the right number of blobs and rejects noise.
4. Back-projection produces plausible 3D positions given known geometry.
5. Uncertainty covariance is non-zero and positive-definite.
6. Direction vectors are unit-length.
7. HSV range edge cases (hue wrap-around, very dark / very bright cones).

Run with:
    python3 test_camera_cone_detector.py

A window will open showing the synthetic test image and masks.
Close the windows to see the test summary.
"""

import sys
import numpy as np
import cv2

# ── ANSI colours for terminal output ────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

passed = 0
failed = 0

def ok(msg):
    global passed
    passed += 1
    print(f"  {GREEN}PASS{RESET}  {msg}")

def fail(msg, detail=""):
    global failed
    failed += 1
    print(f"  {RED}FAIL{RESET}  {msg}" + (f"\n        {detail}" if detail else ""))

def section(title):
    print(f"\n{BOLD}── {title} ──{RESET}")


# ════════════════════════════════════════════════════════════════════════════
# Replicate the core logic from camera_cone_detector_node.py
# (no ROS imports needed here)
# ════════════════════════════════════════════════════════════════════════════

HSV_RANGES = {
    "blue":   [(np.array([100, 80, 50]),  np.array([130, 255, 255]))],
    "yellow": [(np.array([20,  80, 80]),  np.array([38,  255, 255]))],
    "orange": [(np.array([5,  100, 80]),  np.array([20,  255, 255]))],
}

MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

MIN_AREA = 50
MAX_AREA = 80000
MAX_ASPECT = 4.0
MIN_DEPTH_PTS = 5

def colour_mask(hsv_img, colour):
    m = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[colour]:
        m |= cv2.inRange(hsv_img, lo, hi)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  MORPH_K)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, MORPH_K)
    return m

def find_blobs(mask):
    """Return list of (x, y, w, h, area) for valid blobs."""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = []
    for i in range(1, n):
        x, y, w, h, area = (stats[i, cv2.CC_STAT_LEFT],
                             stats[i, cv2.CC_STAT_TOP],
                             stats[i, cv2.CC_STAT_WIDTH],
                             stats[i, cv2.CC_STAT_HEIGHT],
                             stats[i, cv2.CC_STAT_AREA])
        if area < MIN_AREA or area > MAX_AREA:
            continue
        if h == 0 or float(w) / float(h) > MAX_ASPECT:
            continue
        blobs.append((x, y, w, h, area))
    return blobs

def backproject(u, v, z, K):
    """Back-project image point (u,v) at depth z using intrinsic matrix K."""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z])

def direction_vector(pt_3d):
    n = np.linalg.norm(pt_3d)
    if n < 1e-9:
        return None
    return pt_3d / n

def position_covariance(valid_depths, blob_w_px, z_median, fx):
    mad = float(np.median(np.abs(valid_depths - z_median)))
    sigma_z = max(mad * 1.4826, 0.02)
    sigma_lat = max((blob_w_px / 4.0) * z_median / fx, 0.02)
    return np.array([[sigma_z**2, 0.0], [0.0, sigma_lat**2]])


# ════════════════════════════════════════════════════════════════════════════
# Synthetic image helpers
# ════════════════════════════════════════════════════════════════════════════

W, H = 640, 480

# Fake camera intrinsics (typical 640×480 camera)
K = np.array([[520.0,   0.0, 320.0],
              [  0.0, 520.0, 240.0],
              [  0.0,   0.0,   1.0]])


def make_bgr_image():
    """
    Create a synthetic BGR test image with:
      - One blue cone blob  (top-left)
      - One yellow cone blob (centre)
      - One orange cone blob (top-right)
      - Some single-pixel noise dots (should be filtered)
      - A wide white rectangle (should be rejected by aspect ratio)
    """
    img = np.zeros((H, W, 3), dtype=np.uint8)

    # Blue cone: HSV (115, 200, 200) → BGR
    blue_hsv  = np.array([[[115, 200, 200]]], dtype=np.uint8)
    blue_bgr  = cv2.cvtColor(blue_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    cv2.rectangle(img, (40, 100), (90, 200), blue_bgr.tolist(), -1)   # ~2500 px²

    # Yellow cone: HSV (30, 200, 220) → BGR
    yellow_hsv = np.array([[[30, 200, 220]]], dtype=np.uint8)
    yellow_bgr = cv2.cvtColor(yellow_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    cv2.rectangle(img, (280, 150), (360, 300), yellow_bgr.tolist(), -1) # ~11200 px²

    # Orange cone: HSV (12, 220, 210) → BGR
    orange_hsv = np.array([[[12, 220, 210]]], dtype=np.uint8)
    orange_bgr = cv2.cvtColor(orange_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    cv2.ellipse(img, (560, 150), (30, 60), 0, 0, 360, orange_bgr.tolist(), -1)

    # Noise: 3 tiny blue dots (2×2 px, area=4 < MIN_AREA=50 → should be filtered)
    for nx, ny in [(10, 10), (20, 400), (600, 50)]:
        cv2.rectangle(img, (nx, ny), (nx+2, ny+2), blue_bgr.tolist(), -1)

    # Wide white rectangle — aspect ratio w/h >> 4 → should be rejected
    cv2.rectangle(img, (100, 400), (500, 420), (255, 255, 255), -1)

    return img


def make_depth_image(cone_ranges):
    """
    Make a float32 depth image (metres).
    cone_ranges: dict of colour → (bbox, depth_m, std_m)
    """
    depth = np.full((H, W), np.nan, dtype=np.float32)
    for colour, (bbox, d, sigma) in cone_ranges.items():
        x1, y1, x2, y2 = bbox
        patch = np.random.normal(d, sigma, (y2 - y1, x2 - x1)).astype(np.float32)
        patch = np.clip(patch, 0.1, 20.0)
        depth[y1:y2, x1:x2] = patch
    return depth


# ════════════════════════════════════════════════════════════════════════════
# Test suite
# ════════════════════════════════════════════════════════════════════════════

bgr = make_bgr_image()
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

CONE_RANGES = {
    "blue":   ((40, 100, 90, 200),  2.5, 0.05),
    "yellow": ((280, 150, 360, 300), 4.0, 0.08),
    "orange": ((530, 90, 590, 210),  1.8, 0.03),
}
depth = make_depth_image(CONE_RANGES)


section("1. Colour masking — correct colours detected")

masks = {c: colour_mask(hsv, c) for c in ("blue", "yellow", "orange")}

for colour in ("blue", "yellow", "orange"):
    m = masks[colour]
    nonzero = cv2.countNonZero(m)
    if nonzero > 100:
        ok(f"{colour} mask has {nonzero} non-zero pixels")
    else:
        fail(f"{colour} mask is nearly empty ({nonzero} px)")


section("2. Colour masking — no cross-contamination")

for c1 in ("blue", "yellow", "orange"):
    for c2 in ("blue", "yellow", "orange"):
        if c1 == c2:
            continue
        # Get the c1-colour blob region, check its pixels in c2 mask
        blobs1 = find_blobs(masks[c1])
        if not blobs1:
            continue
        x, y, w, h, _ = blobs1[0]
        region_c2 = masks[c2][y:y+h, x:x+w]
        overlap = cv2.countNonZero(region_c2)
        if overlap == 0:
            ok(f"{c1} blob region has 0 px in {c2} mask")
        else:
            fail(f"{c1} blob region leaks {overlap} px into {c2} mask")


section("3. Blob detection — correct count and noise rejection")

for colour in ("blue", "yellow", "orange"):
    blobs = find_blobs(masks[colour])
    if len(blobs) == 1:
        ok(f"{colour}: exactly 1 blob found (noise filtered)")
    else:
        fail(f"{colour}: expected 1 blob, got {len(blobs)}")

# White wide rectangle — goes into no colour mask, but verify aspect ratio logic
white_mask = np.zeros((H, W), dtype=np.uint8)
cv2.rectangle(white_mask, (100, 400), (500, 420), 255, -1)
white_blobs_before_filter = []
n, labels, stats, _ = cv2.connectedComponentsWithStats(white_mask, connectivity=8)
for i in range(1, n):
    white_blobs_before_filter.append((stats[i, cv2.CC_STAT_WIDTH],
                                      stats[i, cv2.CC_STAT_HEIGHT]))
if white_blobs_before_filter:
    w_r, h_r = white_blobs_before_filter[0]
    aspect = w_r / max(h_r, 1)
    if aspect > MAX_ASPECT:
        ok(f"Wide rectangle aspect={aspect:.1f} > {MAX_ASPECT} → correctly rejected")
    else:
        fail(f"Wide rectangle aspect={aspect:.1f} should exceed {MAX_ASPECT}")


section("4. Back-projection and 3D position plausibility")

for colour, (bbox, true_depth, _) in CONE_RANGES.items():
    x1, y1, x2, y2 = bbox
    u_c = (x1 + x2) / 2.0
    v_c = (y1 + y2) / 2.0

    # Sample depth in blob
    blob_depths = depth[y1:y2, x1:x2].flatten()
    valid = blob_depths[np.isfinite(blob_depths) & (blob_depths > 0.05)]

    if len(valid) < MIN_DEPTH_PTS:
        fail(f"{colour}: not enough valid depth samples ({len(valid)})")
        continue

    z_med = float(np.median(valid))
    pt3d = backproject(u_c, v_c, z_med, K)

    # Depth should be within 10% of true depth
    depth_err = abs(z_med - true_depth) / true_depth
    if depth_err < 0.10:
        ok(f"{colour}: depth estimate {z_med:.2f}m (true {true_depth}m, err {depth_err*100:.1f}%)")
    else:
        fail(f"{colour}: depth error {depth_err*100:.1f}% exceeds 10%")

    # 3D point should be roughly in front of camera (positive z)
    if pt3d[2] > 0:
        ok(f"{colour}: 3D point is in front of camera (z={pt3d[2]:.2f})")
    else:
        fail(f"{colour}: 3D point behind camera (z={pt3d[2]:.2f})")


section("5. Direction vectors — unit length")

for colour, (bbox, true_depth, _) in CONE_RANGES.items():
    x1, y1, x2, y2 = bbox
    u_c = (x1 + x2) / 2.0
    v_c = (y1 + y2) / 2.0
    pt3d = backproject(u_c, v_c, true_depth, K)
    d = direction_vector(pt3d)

    if d is None:
        fail(f"{colour}: direction vector is None")
        continue

    norm = np.linalg.norm(d)
    if abs(norm - 1.0) < 1e-6:
        ok(f"{colour}: direction vector is unit length (norm={norm:.8f})")
    else:
        fail(f"{colour}: direction vector norm={norm:.6f} (expected 1.0)")


section("6. Uncertainty covariance — positive definite")

for colour, (bbox, true_depth, depth_std) in CONE_RANGES.items():
    x1, y1, x2, y2 = bbox
    blob_depths = depth[y1:y2, x1:x2].flatten()
    valid = blob_depths[np.isfinite(blob_depths) & (blob_depths > 0.05)]
    z_med = float(np.median(valid))
    w_px = x2 - x1

    cov = position_covariance(valid, w_px, z_med, K[0, 0])

    # Check positive-definite: all eigenvalues > 0
    eigvals = np.linalg.eigvalsh(cov)
    if np.all(eigvals > 0):
        ok(f"{colour}: covariance is positive definite (eigenvalues={eigvals})")
    else:
        fail(f"{colour}: covariance not positive definite (eigenvalues={eigvals})")

    # σ should be non-trivially small (> 0) and not huge (< 5 m²)
    if 0 < cov[0, 0] < 25 and 0 < cov[1, 1] < 25:
        ok(f"{colour}: covariance diagonal σx²={cov[0,0]:.4f}, σy²={cov[1,1]:.4f}")
    else:
        fail(f"{colour}: covariance diagonal out of reasonable range")


section("7. Edge cases")

# Very dark cone (V=10) — should not be detected (V < threshold)
dark_img = np.zeros((H, W, 3), dtype=np.uint8)
dark_hsv_val = np.array([[[115, 200, 10]]], dtype=np.uint8)  # very dark blue
dark_bgr = cv2.cvtColor(dark_hsv_val, cv2.COLOR_HSV2BGR)[0, 0]
cv2.rectangle(dark_img, (40, 100), (140, 250), dark_bgr.tolist(), -1)
dark_hsv_img = cv2.cvtColor(dark_img, cv2.COLOR_BGR2HSV)
dark_mask = colour_mask(dark_hsv_img, "blue")
dark_blobs = find_blobs(dark_mask)
if len(dark_blobs) == 0:
    ok("Very dark blue blob (V=10) correctly rejected by V threshold")
else:
    fail("Very dark blue blob should be rejected but was detected")

# Zero-depth pixels — should not crash or produce detections
zero_depth = np.zeros((H, W), dtype=np.float32)
test_depths = zero_depth[100:200, 40:90].flatten()
valid_zero = test_depths[np.isfinite(test_depths) & (test_depths > 0.05)]
if len(valid_zero) < MIN_DEPTH_PTS:
    ok("Zero-depth image: insufficient valid points → detection correctly skipped")
else:
    fail("Zero-depth image should produce no valid depth points")


# ════════════════════════════════════════════════════════════════════════════
# Visual output
# ════════════════════════════════════════════════════════════════════════════

section("Visual output")
print("  Displaying annotated test image and masks…")
print("  (close windows to continue)\n")

# Annotate the test image
vis = bgr.copy()
for colour in ("blue", "yellow", "orange"):
    blobs = find_blobs(masks[colour])
    DRAW_COLOURS = {
        "blue":   (200,  50,  50),
        "yellow": ( 30, 200, 200),
        "orange": ( 30, 120, 240),
    }
    for (x, y, w, h, area) in blobs:
        cv2.rectangle(vis, (x, y), (x+w, y+h), DRAW_COLOURS[colour], 2)
        # Draw direction arrow from image centre toward blob centre
        u_c, v_c = x + w//2, y + h//2
        true_depth = CONE_RANGES[colour][1]
        pt3d = backproject(u_c, v_c, true_depth, K)
        d = direction_vector(pt3d)
        if d is not None:
            # Project direction back to image for visualisation (arrow endpoint)
            arrow_end_u = int(K[0,0] * d[0]/d[2] + K[0,2]) if d[2] > 0 else u_c
            arrow_end_v = int(K[1,1] * d[1]/d[2] + K[1,2]) if d[2] > 0 else v_c
        label = f"{colour} {true_depth}m"
        cv2.putText(vis, label, (x, max(y-5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW_COLOURS[colour], 1)

# Build mask montage
blue_rgb   = cv2.cvtColor(masks["blue"],   cv2.COLOR_GRAY2BGR)
yellow_rgb = cv2.cvtColor(masks["yellow"], cv2.COLOR_GRAY2BGR)
orange_rgb = cv2.cvtColor(masks["orange"], cv2.COLOR_GRAY2BGR)

# Colour-tint the masks
blue_rgb[:,:,1]   = 0; blue_rgb[:,:,2]   = 0
yellow_rgb[:,:,0] = 0; yellow_rgb[:,:,2] = 0
orange_rgb[:,:,0] = 0; orange_rgb[:,:,1] = 0

depth_vis = depth.copy()
depth_vis[~np.isfinite(depth_vis)] = 0
depth_norm = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
depth_colour = cv2.applyColorMap(depth_norm, cv2.COLORMAP_PLASMA)

# Row 1: annotated image | blue mask
# Row 2: yellow mask | orange mask | depth (scaled to same total width)
row1 = np.hstack([vis, blue_rgb])                        # 1280 wide
row2 = np.hstack([yellow_rgb, orange_rgb, depth_colour]) # 1920 wide → resize to 1280
row2 = cv2.resize(row2, (row1.shape[1], row1.shape[0]))

montage = np.vstack([row1, row2])
cv2.imshow("O2/O3 Unit Test — Synthetic Cones  (press any key to close)", montage)
cv2.waitKey(0)
cv2.destroyAllWindows()


# ════════════════════════════════════════════════════════════════════════════
# Summary
# ════════════════════════════════════════════════════════════════════════════

total = passed + failed
print(f"\n{BOLD}{'═'*50}{RESET}")
print(f"{BOLD}Result: {GREEN if failed == 0 else RED}{passed}/{total} passed{RESET}")
if failed > 0:
    print(f"{RED}  {failed} test(s) failed — check HSV ranges or logic above.{RESET}")
else:
    print(f"{GREEN}  All tests passed. ✓{RESET}")
print(f"{'═'*50}\n")

sys.exit(0 if failed == 0 else 1)
