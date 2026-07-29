import cv2
import numpy as np

NMS_TH           = 0.2
ROI_SMOOTH_ALPHA = 0.65

# Fixed YCrCb human-skin bounds (Peer et al.) — single source of truth for the
# SEARCH Gate-4 pre-scan (run_tracker) and the SearchDetector frame pre-scan.
SKIN_CR_MIN, SKIN_CR_MAX = 133, 173
SKIN_CB_MIN, SKIN_CB_MAX =  77, 127

PAD_X      = 0.35
PAD_UP_Y   = 1
PAD_DOWN_Y = 0.15
MIN_PAD    = 10

SAMPLE_REL_SIZE = 0.28
MIN_SAMPLE_SIZE = 24

DELTA_CR_FIXED      = 18
DELTA_CB_FIXED      = 18
USE_STD_BASED_DELTA = True
STD_K               = 2.0

CR_RANGE_LIMITS = (0, 255)
CB_RANGE_LIMITS = (0, 255)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def skin_ratio(img_bgr, cr_min, cr_max, cb_min, cb_max):
    """Fraction of pixels whose Cr/Cb fall inside the given bounds (0.0 if empty)."""
    if img_bgr.size == 0:
        return 0.0
    ycrcb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YCrCb)
    cr = ycrcb[:, :, 1]
    cb = ycrcb[:, :, 2]
    mask = (cr >= cr_min) & (cr <= cr_max) & (cb >= cb_min) & (cb <= cb_max)
    return float(np.count_nonzero(mask)) / mask.size


def _has_results(res):
    if res is None:
        return False
    try:
        return len(res) > 0
    except TypeError:
        return getattr(res, "size", 0) > 0


def pick_best_result_with_index(results):
    best = None
    best_idx = -1
    best_score = -1.0
    for i, r in enumerate(results):
        score = float(r[-1])
        if score > best_score:
            best_score = score
            best = r
            best_idx = i
    return best, best_idx


def get_landmarks_xy(r):
    lm = r[4:18]
    xs = [float(lm[i]) for i in range(0, 14, 2)]
    ys = [float(lm[i]) for i in range(1, 14, 2)]
    return xs, ys


def bbox_from_landmarks_asymmetric(x1, y1, x2, y2, xs, ys, W, H):
    min_x = min([x1, x2] + xs)
    max_x = max([x1, x2] + xs)
    min_y = min([y1, y2] + ys)
    max_y = max([y1, y2] + ys)

    bw = max_x - min_x
    bh = max_y - min_y

    pad_x    = max(int(PAD_X * bw), MIN_PAD)
    pad_up   = max(int(PAD_UP_Y * bh), MIN_PAD)
    pad_down = max(int(PAD_DOWN_Y * bh), MIN_PAD)

    ex1 = int(clamp(min_x - pad_x,   0, W - 1))
    ex2 = int(clamp(max_x + pad_x,   0, W - 1))
    ey1 = int(clamp(min_y - pad_up,  0, H - 1))
    ey2 = int(clamp(max_y + pad_down, 0, H - 1))

    return ex1, ey1, ex2, ey2


def smooth_roi(prev_roi, new_roi, alpha):
    if prev_roi is None:
        return new_roi
    px1, py1, px2, py2 = prev_roi
    nx1, ny1, nx2, ny2 = new_roi
    return (
        int(alpha * px1 + (1 - alpha) * nx1),
        int(alpha * py1 + (1 - alpha) * ny1),
        int(alpha * px2 + (1 - alpha) * nx2),
        int(alpha * py2 + (1 - alpha) * ny2),
    )


def _median(lst):
    a = sorted(lst)
    n = len(a)
    if n == 0:
        return 0.0
    if n % 2 == 1:
        return a[n // 2]
    return 0.5 * (a[n // 2 - 1] + a[n // 2])


def palm_center_from_landmarks(xs, ys):
    return int(_median([float(x) for x in xs])), int(_median([float(y) for y in ys]))


def compute_skin_thresholds_from_roi(frame_bgr, roi, palm_xy):
    ex1, ey1, ex2, ey2 = roi
    roi_img = frame_bgr[ey1:ey2 + 1, ex1:ex2 + 1]
    if roi_img.size == 0:
        return None

    h, w = roi_img.shape[:2]
    side = int(max(MIN_SAMPLE_SIZE, SAMPLE_REL_SIZE * min(w, h)))

    px = int(clamp(palm_xy[0], ex1, ex2))
    py = int(clamp(palm_xy[1], ey1, ey2))
    rx, ry = px - ex1, py - ey1

    sx1 = int(clamp(rx - side // 2, 0, w - 1))
    sy1 = int(clamp(ry - side // 2, 0, h - 1))
    sx2 = int(clamp(rx + side // 2, 0, w - 1))
    sy2 = int(clamp(ry + side // 2, 0, h - 1))

    sample = roi_img[sy1:sy2 + 1, sx1:sx2 + 1]
    if sample.size == 0:
        return None

    ycrcb   = cv2.cvtColor(sample, cv2.COLOR_BGR2YCrCb)
    Cr      = ycrcb[:, :, 1].astype("float32")
    Cb      = ycrcb[:, :, 2].astype("float32")
    cr_mean = float(Cr.mean())
    cb_mean = float(Cb.mean())

    if USE_STD_BASED_DELTA:
        dcr = max(DELTA_CR_FIXED, int(STD_K * float(Cr.std())))
        dcb = max(DELTA_CB_FIXED, int(STD_K * float(Cb.std())))
    else:
        dcr, dcb = DELTA_CR_FIXED, DELTA_CB_FIXED

    Cr_min = int(clamp(cr_mean - dcr, *CR_RANGE_LIMITS))
    Cr_max = int(clamp(cr_mean + dcr, *CR_RANGE_LIMITS))
    Cb_min = int(clamp(cb_mean - dcb, *CB_RANGE_LIMITS))
    Cb_max = int(clamp(cb_mean + dcb, *CB_RANGE_LIMITS))

    return (Cr_min, Cr_max, Cb_min, Cb_max)


def draw_landmarks(frame, xs, ys):
    for x, y in zip(xs, ys):
        cv2.circle(frame, (int(x), int(y)), 4, (0, 0, 255), -1)
