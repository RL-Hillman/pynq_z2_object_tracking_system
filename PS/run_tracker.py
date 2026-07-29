"""
run_tracker.py — primary runtime for PYNQ-Z2 HDMI output.

Runs the full state machine on PYNQ Linux, rendering live video directly to
the HDMI monitor via PYNQ HDMI OUT (hdmi_out.writeframe).

Usage (PYNQ terminal):
    python run_tracker.py

Hardware mode is auto-detected: the real FPGA interface is used when the pynq
package is importable (i.e. on the board); otherwise the software stub runs for
PC-side development (Windows/Linux, no FPGA required). Set the environment
variable USE_STUB=1 to force stub mode anywhere.
"""

import importlib.util
import sys
import os
import time
from enum import Enum, auto

import cv2

# ── Tool imports ──────────────────────────────────────────────────────────────
# Use the script's own directory so the runner works regardless of cwd.
TOOLS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools')
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

from mp_palmdet import MPPalmDet

from fpga_interface import (
    StubFPGAInterface, PYNQFPGAInterface,
    BIT_SERVO_HOLD, BIT_SEARCH_SWEEP_ENABLE,
)
from hand_validator import HandValidator
from search_detector import SearchDetector

from tracker_utils import (
    _has_results,
    pick_best_result_with_index,
    get_landmarks_xy,
    bbox_from_landmarks_asymmetric,
    smooth_roi,
    palm_center_from_landmarks,
    compute_skin_thresholds_from_roi,
    clamp,
    draw_landmarks,
    skin_ratio,
    ROI_SMOOTH_ALPHA,
    NMS_TH,
    SKIN_CR_MIN, SKIN_CR_MAX, SKIN_CB_MIN, SKIN_CB_MAX,
)

# ── Configuration ─────────────────────────────────────────────────────────────

# Hardware mode — auto-detected. Real PYNQ-Z2 interface when pynq is importable
# (i.e. running on the board); software stub otherwise. USE_STUB=1 in the
# environment forces the stub even on the board.
USE_STUB       = (os.environ.get('USE_STUB') == '1'
                  or importlib.util.find_spec('pynq') is None)
BITSTREAM_PATH = 'design_1.bit'
FPGA_BASE_ADDR = 0x43C30000   # SEG_tracker_core_0_tracker_core_AXI_reg

# Camera
CAMERA_INDEX     = 0
FRAME_W, FRAME_H = 640, 480
CAP_BUFFER_SIZE  = 1   # V4L2: keep only the latest frame, minimises display lag

# Model
MODEL_PATH = os.path.join(TOOLS_DIR, 'palm.onnx')

# Model sensitivity 
MODEL_SCORE_TH = 0.50

# Per-frame confirmation gate applied inside run_search_frame (Gate 1).
# Slightly above MODEL_SCORE_TH so very low-confidence frames are discarded,
# while high-confidence detections pass quickly 
SEARCH_MIN_SCORE     = 0.75
CALIB_MIN_SCORE      = 0.75   # must match SEARCH_MIN_SCORE — the stored best_result
                               # already passed the SEARCH gate, so calibration should
VALIDATION_MIN_SCORE = 0.75

# SEARCH stability —  frames
SEARCH_STABLE_N = 4

# ROI size / aspect sanity bounds
MIN_HAND_W =  60
MIN_HAND_H =  60
MAX_HAND_W = 300
MAX_HAND_H = 300
MIN_ASPECT = 0.35
MAX_ASPECT = 2.80

# Landmark geometry — real palm keypoints must span at least this many pixels
# in each axis.  Clustered landmarks indicate a false detection.
MIN_LANDMARK_SPAN = 15   # pixels

# Inter-frame stability
MAX_CENTER_JUMP = 70   # max pixels the ROI center may travel between frames

# Fixed skin color pre-scan (Gate 4 in run_search_frame). The Cr/Cb bounds
# themselves (Peer et al.) are shared constants in tracker_utils.
SKIN_PRESCAN_RATIO = 0.16   # min skin-pixel fraction in candidate ROI

# Calibration sanity check (uses the COMPUTED Cr/Cb thresholds)
MIN_SKIN_RATIO   = 0.25
SKIN_THR_PADDING = 8     # expand Cr/Cb ±N counts before writing to FPGA

# Async validation
VALIDATION_INTERVAL  = 1.5   # seconds between validation triggers
MAX_VALIDATION_FAILS = 3     # consecutive AI-validation failures before giving up
                             # when the FPGA geometry is NOT healthy (fast drop)
OVERLAP_MIN_RATIO    = 0.30  # min overlap÷min-area between the AI detection box and
                             # the FPGA current ROI for a validation to pass

# Geometry-aware LOST: while the FPGA still reports a solid, hand-sized skin blob
# in a real ROI, tolerate more consecutive AI misses (motion blur / fast motion)
# — but a hard cap still forces recovery so a lock onto a skin-coloured non-hand
# (e.g. a face) cannot persist forever. All tunable; retune on hardware.
MAX_VALIDATION_FAILS_HEALTHY = 8    # hard cap on consecutive misses with healthy geometry
HEALTHY_SKIN_MIN             = 800  # min SKIN_PIXEL_COUNT to call the geometry "healthy"
HEALTHY_BBOX_MIN             = 40   # min FPGA bbox w/h (px) to call the geometry "healthy"

# Validation ROI padding
VAL_PAD = 20   # pixels added around FPGA bbox for the validation crop

# Display
SHOW_LANDMARKS = True   # set True to overlay landmarks for debugging
WINDOW_NAME    = 'PYNQ Hand Tracker'
MAX_CAMERA_FAILS = 30        # consecutive read failures before aborting

# ── State machine ─────────────────────────────────────────────────────────────

class Phase(Enum):
    INIT        = auto()
    SEARCH      = auto()
    CALIBRATION = auto()
    HANDOFF     = auto()
    TRACKING    = auto()
    LOST        = auto()


class TrackerState:
    def __init__(self):
        self.phase             = Phase.INIT
        self.stable_count      = 0
        self.last_roi          = None   # (x1,y1,x2,y2) smoothed ROI from SEARCH (FPGA)
        self.display_roi       = None   # (x1,y1,x2,y2) box around hand for display
        self.best_result       = None   # raw MPPalmDet detection 
        self.last_thr          = None   # (Cr_min,Cr_max,Cb_min,Cb_max) raw
        self.last_thr_padded   = None   # padded values written to FPGA

    def reset_search(self):
        self.stable_count = 0
        self.last_roi     = None
        self.display_roi  = None
        self.best_result  = None

    def reset_all(self):
        self.reset_search()
        self.last_thr        = None
        self.last_thr_padded = None


# ── ROI geometry / skin helpers ───────────────────────────────────────────────

def roi_is_reasonable(roi):
    """Return True only if the ROI dimensions and aspect ratio match a real hand."""
    x1, y1, x2, y2 = roi
    w = x2 - x1
    h = y2 - y1
    aspect = w / max(1, h)
    if w < MIN_HAND_W or h < MIN_HAND_H:
        return False
    if w > MAX_HAND_W or h > MAX_HAND_H:
        return False
    if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
        return False
    return True


def roi_center(roi):
    x1, y1, x2, y2 = roi
    return ((x1 + x2) // 2, (y1 + y2) // 2)


def center_distance(roi_a, roi_b):
    ax, ay = roi_center(roi_a)
    bx, by = roi_center(roi_b)
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5


def roi_contains_skin(frame, roi):
    """Return True if >= SKIN_PRESCAN_RATIO of the ROI pixels fall in the
    standard human-skin YCrCb range.

    Applied before incrementing stable_count so background objects (shelves,
    walls, books) are rejected on the very first frame they appear, even when
    the model's confidence is above 0.90.
    """
    x1, y1, x2, y2 = roi
    patch = frame[y1:y2 + 1, x1:x2 + 1]
    return skin_ratio(patch, SKIN_CR_MIN, SKIN_CR_MAX,
                      SKIN_CB_MIN, SKIN_CB_MAX) >= SKIN_PRESCAN_RATIO


def _landmarks_spread_ok(xs, ys):
    """Return False if the 7 palm landmarks are too tightly clustered.

    Background false-positives often produce landmarks bunched near one point.
    A real hand's palm keypoints always span at least MIN_LANDMARK_SPAN pixels
    in both axes at any practical working distance.
    """
    if len(xs) < 2:
        return True
    return (max(xs) - min(xs)) >= MIN_LANDMARK_SPAN and \
           (max(ys) - min(ys)) >= MIN_LANDMARK_SPAN


# ── Phase helpers ─────────────────────────────────────────────────────────────

def run_search_frame(frame, state, results):
    """One SEARCH iteration. Returns True when stable_count reaches SEARCH_STABLE_N.

    `results` is the pre-computed output of MPPalmDet.infer(), supplied by
    SearchDetector.poll(). `frame` is used only for Gate 4 skin-pixel
    validation and frame-boundary clamping.

    Five sequential gates guard each stable_count increment:
      Gate 1   — model confidence  >= SEARCH_MIN_SCORE
      Gate 2   — ROI size + aspect are physically hand-like
      Gate 2.5 — landmark spread >= MIN_LANDMARK_SPAN (rejects clustered false positives)
      Gate 3   — ROI center did not jump > MAX_CENTER_JUMP pixels from last frame
      Gate 4   — ROI contains >= SKIN_PRESCAN_RATIO skin-colored pixels (rejects shelves)

    Any failing gate resets stable_count to 0.  Failing Gate 3 keeps the new
    candidate position so the counter can rebuild from there.
    """
    H, W = frame.shape[:2]

    if not _has_results(results):
        state.reset_search()
        return False

    best, _ = pick_best_result_with_index(results)
    if best is None:
        state.reset_search()
        return False

    # Gate 1: model confidence
    best_score = float(best[-1])
    if best_score < SEARCH_MIN_SCORE:
        state.reset_search()
        return False

    x1, y1, x2, y2 = map(int, best[:4])
    xs, ys = get_landmarks_xy(best)
    candidate_roi = bbox_from_landmarks_asymmetric(x1, y1, x2, y2, xs, ys, W, H)

    # Gate 2: physical size and aspect ratio
    if not roi_is_reasonable(candidate_roi):
        state.reset_search()
        return False

    # Gate 2.5: landmark spread — rejects background objects whose keypoints
    # cluster at one point instead of spanning a real palm structure
    if not _landmarks_spread_ok(xs, ys):
        state.reset_search()
        return False

    # Gate 3: inter-frame stability — prevents accumulating across two different objects
    if state.last_roi is not None:
        if center_distance(candidate_roi, state.last_roi) > MAX_CENTER_JUMP:
            state.stable_count = 0
            state.last_roi     = candidate_roi   # start fresh from this position
            state.best_result  = best
            return False

    # Gate 4: skin color pre-scan
    if not roi_contains_skin(frame, candidate_roi):
        state.reset_search()
        return False

    roi_smoothed      = smooth_roi(state.last_roi, candidate_roi, ROI_SMOOTH_ALPHA)
    state.last_roi    = roi_smoothed
    state.best_result = best
    state.stable_count += 1

    # Tight display rectangle — landmark extent + symmetric 30 % padding.
    # Does NOT use PAD_UP_Y so it stays on the hand and does not reach the face.
    lm_x1 = int(clamp(min(xs) - max(int(0.30 * (max(xs) - min(xs))), 10), 0, W - 1))
    lm_x2 = int(clamp(max(xs) + max(int(0.30 * (max(xs) - min(xs))), 10), 0, W - 1))
    lm_y1 = int(clamp(min(ys) - max(int(0.30 * (max(ys) - min(ys))), 10), 0, H - 1))
    lm_y2 = int(clamp(max(ys) + max(int(0.30 * (max(ys) - min(ys))), 10), 0, H - 1))
    state.display_roi = (lm_x1, lm_y1, lm_x2, lm_y2)

    return state.stable_count >= SEARCH_STABLE_N


def run_calibration(frame, state, fpga):
    """Calibrate skin-color thresholds and write to FPGA.

    Returns True on success, False on failure — the caller resets search
    state and restarts SEARCH on failure.

    Failure conditions:
      - missing state
      - detection score < CALIB_MIN_SCORE
      - compute_skin_thresholds_from_roi() returns None
      - < MIN_SKIN_RATIO of ROI pixels match the computed Cr/Cb range
    """
    best = state.best_result
    roi  = state.last_roi
    if best is None or roi is None:
        return False

    score = float(best[-1])
    if score < CALIB_MIN_SCORE:
        return False

    xs, ys  = get_landmarks_xy(best)
    palm_xy = palm_center_from_landmarks(xs, ys)
    calib   = compute_skin_thresholds_from_roi(frame, roi, palm_xy)
    if calib is None:
        return False

    Cr_min, Cr_max, Cb_min, Cb_max = calib

    # Skin-pixel ratio sanity check using the computed thresholds
    ex1, ey1, ex2, ey2 = roi
    roi_skin = skin_ratio(frame[ey1:ey2 + 1, ex1:ex2 + 1],
                          Cr_min, Cr_max, Cb_min, Cb_max)
    if roi_skin < MIN_SKIN_RATIO:
        print(f'[CALIB] Rejected: skin_ratio={roi_skin:.2f} < {MIN_SKIN_RATIO}')
        return False

    # Tolerance padding before FPGA write
    cr_min_p = int(clamp(Cr_min - SKIN_THR_PADDING, 0, 255))
    cr_max_p = int(clamp(Cr_max + SKIN_THR_PADDING, 0, 255))
    cb_min_p = int(clamp(Cb_min - SKIN_THR_PADDING, 0, 255))
    cb_max_p = int(clamp(Cb_max + SKIN_THR_PADDING, 0, 255))

    roi_w = ex2 - ex1 + 1
    roi_h = ey2 - ey1 + 1

    # Seed the FPGA's initial ROI + skin thresholds. From here the PL owns and
    # self-updates its internal ROI; the PS does not write ROI again this session.
    print(f'[CALIB] ROI   x={ex1}  y={ey1}  w={roi_w}  h={roi_h}')
    fpga.write_roi(ex1, ey1, roi_w, roi_h)
    print(f'[CALIB] SKIN  Cr[{cr_min_p},{cr_max_p}]  Cb[{cb_min_p},{cb_max_p}]')
    fpga.write_skin_thresholds(cr_min_p, cr_max_p, cb_min_p, cb_max_p)

    state.last_thr        = (Cr_min, Cr_max, Cb_min, Cb_max)
    state.last_thr_padded = (cr_min_p, cr_max_p, cb_min_p, cb_max_p)

    print(f'[CALIB] score={score:.2f}  skin_ratio={roi_skin:.2f}')
    print(f'[CALIB] Raw   Cr[{Cr_min},{Cr_max}] Cb[{Cb_min},{Cb_max}]')
    print(f'[CALIB] Sent  Cr[{cr_min_p},{cr_max_p}] Cb[{cb_min_p},{cb_max_p}] (pad±{SKIN_THR_PADDING})')
    return True


def do_handoff(fpga, validator, state):
    """Switch FPGA from sweep/search mode to full hardware tracking."""
    fpga.write_ctrl(
        track_enable=True,
        servo_enable=True,
        servo_hold=False,
        search_sweep_enable=False,
    )
    validator.reset()
    state.stable_count = 0
    state.display_roi  = None   # hide hand rectangle once FPGA takes over
    print('[HANDOFF] FPGA tracking enabled → TRACKING')


def do_lost_recovery(fpga, validator, state):
    """Emergency stop → restart servo sweep → reset all Python state → SEARCH."""
    fpga.set_ctrl_bit(BIT_SERVO_HOLD, True)          # priority-1 freeze; enter_search_mode below clears it to start sweep
    enter_search_mode(fpga)
    validator.reset()
    state.reset_all()
    state.phase = Phase.SEARCH
    print('[LOST] Recovery: servo halted → sweep resumed → SEARCH')


def enter_search_mode(fpga):
    fpga.write_ctrl(
        track_enable=False,
        servo_enable=True,
        servo_hold=False,
        search_sweep_enable=True,
    )


def fpga_geometry_healthy(td) -> bool:
    """True if the FPGA is still reporting a plausible hand-sized skin blob in a
    real (non-degenerate, non-full-frame) ROI.

    Used to extend LOST tolerance when the AI validator misses (motion blur, fast
    motion) but the hardware is still holding a solid target. `skin_pixel_count`
    is generic matched-pixel mass despite the name.
    """
    if td.skin_pixel_count < HEALTHY_SKIN_MIN:
        return False
    if td.bbox_w < HEALTHY_BBOX_MIN or td.bbox_h < HEALTHY_BBOX_MIN:
        return False
    if td.bbox_w > FRAME_W or td.bbox_h > FRAME_H:
        return False
    if td.current_roi_w <= 0 or td.current_roi_h <= 0:
        return False
    if td.current_roi_w >= FRAME_W and td.current_roi_h >= FRAME_H:
        return False   # reverted to full frame → not actually locked
    return True


def lost_after_validation(fails: int, geometry_healthy: bool) -> bool:
    """Decide whether to declare LOST after the validator's base patience is
    exhausted (it already reported invalid this cycle).

    Healthy FPGA geometry extends tolerance up to the hard cap
    MAX_VALIDATION_FAILS_HEALTHY; unhealthy geometry gives up immediately.
    """
    return (fails >= MAX_VALIDATION_FAILS_HEALTHY) or (not geometry_healthy)


# ── OSD ───────────────────────────────────────────────────────────────────────

_PHASE_COLORS = {
    Phase.INIT:        (128, 128, 128),
    Phase.SEARCH:      (0,   165, 255),   # orange
    Phase.CALIBRATION: (0,   255, 255),   # yellow
    Phase.HANDOFF:     (255, 255,   0),   # cyan
    Phase.TRACKING:    (0,   255,   0),   # green
    Phase.LOST:        (0,     0, 255),   # red
}


def draw_osd(frame, state, tracking_data=None):
    """Draw a compact dark panel sized to exactly fit the active text rows."""
    x0, y0    = 10, 10
    panel_w   = 300
    font      = cv2.FONT_HERSHEY_SIMPLEX
    fs        = 0.45
    thick     = 1
    gap       = 18
    pad       = 8

    has_roi = state.last_roi is not None
    has_thr = state.last_thr is not None
    has_td  = tracking_data is not None

    n_rows   = 2 + has_roi + has_thr + has_td
    panel_h  = pad + n_rows * gap + pad // 2

    cv2.rectangle(frame, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)

    y = y0 + pad + gap - 2

    phase_color = _PHASE_COLORS.get(state.phase, (0, 255, 255))
    cv2.putText(frame, f'Phase: {state.phase.name}',
                (x0 + pad, y), font, fs, phase_color, thick)
    y += gap

    cv2.putText(frame, f'Stable: {state.stable_count}/{SEARCH_STABLE_N}',
                (x0 + pad, y), font, fs, (255, 255, 255), thick)
    y += gap

    if has_roi:
        x1, y1, x2, y2 = state.last_roi
        cv2.putText(frame, f'ROI {x1},{y1}  {x2-x1}x{y2-y1}',
                    (x0 + pad, y), font, fs, (255, 255, 255), thick)
        y += gap
        if state.display_roi is not None:
            dx1, dy1, dx2, dy2 = state.display_roi
            cv2.rectangle(frame, (dx1, dy1), (dx2, dy2), (0, 255, 0), 2)

    if has_thr:
        cr_min, cr_max, cb_min, cb_max = state.last_thr
        cv2.putText(frame, f'Cr[{cr_min},{cr_max}] Cb[{cb_min},{cb_max}]',
                    (x0 + pad, y), font, fs, (255, 255, 0), thick)
        y += gap

    if has_td:
        cv2.putText(frame,
                    f'CX={tracking_data.cx} CY={tracking_data.cy} SKIN={tracking_data.skin_pixel_count}',
                    (x0 + pad, y), font, fs, (0, 255, 0), thick)
        y += gap
        rx, ry = tracking_data.current_roi_x, tracking_data.current_roi_y
        rw, rh = tracking_data.current_roi_w, tracking_data.current_roi_h
        cv2.putText(frame, f'FPGA ROI {rx},{ry}  {rw}x{rh}',
                    (x0 + pad, y), font, fs, (0, 255, 0), thick)
        if rw > 0 and rh > 0:
            cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)

    return frame


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    hdmi_out = None  # PYNQ HDMI OUT handle; set below when USE_STUB=False

    # FPGA interface
    if USE_STUB:
        fpga = StubFPGAInterface()
        print('[INIT] Running in STUB mode (no FPGA required)')
    else:
        fpga = PYNQFPGAInterface(BITSTREAM_PATH, FPGA_BASE_ADDR)
        print(f'[INIT] PYNQ overlay loaded: {BITSTREAM_PATH}')

    enter_search_mode(fpga)

    # Model threshold is deliberately low (MODEL_SCORE_TH = 0.50); the per-frame
    # SEARCH_MIN_SCORE gate inside run_search_frame does the fine filtering.
    det = MPPalmDet(MODEL_PATH, nmsThreshold=NMS_TH, scoreThreshold=MODEL_SCORE_TH)
    print(f'[INIT] Palm detector loaded  model_th={MODEL_SCORE_TH}  gate={SEARCH_MIN_SCORE}')

    # Async validator
    validator = HandValidator(
        detector=det,
        interval_seconds=VALIDATION_INTERVAL,
        max_consecutive_failures=MAX_VALIDATION_FAILS,
        min_score=VALIDATION_MIN_SCORE,
        overlap_min_ratio=OVERLAP_MIN_RATIO,
    )

    # Async SEARCH detector — runs MPPalmDet in background so the display
    # loop is not blocked during SEARCH (mirrors the HandValidator pattern).
    search_det = SearchDetector(det)

    # Camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   CAP_BUFFER_SIZE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        raise RuntimeError(f'Cannot open camera index {CAMERA_INDEX}')

    if USE_STUB:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    else:
        from pynq.lib.video import VideoMode, PIXEL_BGR
        assert isinstance(fpga, PYNQFPGAInterface)
        # Use the already-loaded custom overlay so design_1.bit is not overwritten
        # by a second BaseOverlay load. design_1.bit must include the HDMI OUT IP.
        hdmi_out = fpga._overlay.video.hdmi_out
        hdmi_out.configure(VideoMode(FRAME_W, FRAME_H, 24), PIXEL_BGR)
        hdmi_out.start()
        print('[INIT] PYNQ HDMI OUT configured')

    state = TrackerState()
    state.phase = Phase.SEARCH
    print(f'[INIT] Complete → {state.phase.name}')
    if USE_STUB:
        print('Press "q" in the video window to quit.')
    else:
        print('Press Ctrl-C to quit.')

    _camera_fail_count = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                _camera_fail_count += 1
                if _camera_fail_count >= MAX_CAMERA_FAILS:
                    raise RuntimeError(
                        f'Camera failed {MAX_CAMERA_FAILS} consecutive reads — aborting'
                    )
                print(f'[WARN] Camera read failed ({_camera_fail_count}/{MAX_CAMERA_FAILS}) — retrying')
                time.sleep(0.1)   # give a transient USB/V4L2 hiccup time to clear
                continue
            _camera_fail_count = 0

            now = time.time()
            tracking_data = None

            # ── SEARCH ───────────────────────────────────────────────────────
            if state.phase == Phase.SEARCH:
                # poll() before submit() so a result from a finished thread is
                # consumed before submit() is allowed to start a new one.
                ready, det_results, det_frame = search_det.poll()
                search_det.submit(frame)

                stable = False
                if ready:
                    stable = run_search_frame(det_frame, state, det_results)

                if USE_STUB and SHOW_LANDMARKS and state.last_roi is not None and state.best_result is not None:
                    xs, ys = get_landmarks_xy(state.best_result)
                    draw_landmarks(frame, xs, ys)

                if stable:
                    search_det.reset()
                    fpga.set_ctrl_bit(BIT_SERVO_HOLD, True)
                    fpga.set_ctrl_bit(BIT_SEARCH_SWEEP_ENABLE, False)
                    state.phase = Phase.CALIBRATION
                    print(f'[SEARCH→CALIBRATION] stable after {state.stable_count} frames')

            # ── CALIBRATION ───────────────────────────────────────────────────
            elif state.phase == Phase.CALIBRATION:
                success = run_calibration(frame, state, fpga)
                if success:
                    state.phase = Phase.HANDOFF
                    print('[CALIBRATION→HANDOFF]')
                else:
                    # Calibration rejected the ROI — resume sweep and re-search
                    enter_search_mode(fpga)
                    state.reset_search()
                    state.phase = Phase.SEARCH
                    search_det.reset()
                    print('[CALIBRATION→SEARCH] calibration rejected, restarting search')

            # ── HANDOFF ───────────────────────────────────────────────────────
            elif state.phase == Phase.HANDOFF:
                try:
                    do_handoff(fpga, validator, state)
                    state.phase = Phase.TRACKING
                except Exception as e:
                    print(f'[HANDOFF] Failed: {e} — recovering to SEARCH')
                    do_lost_recovery(fpga, validator, state)
                    search_det.reset()

            # ── TRACKING ──────────────────────────────────────────────────────
            elif state.phase == Phase.TRACKING:
                try:
                    tracking_data = fpga.read_tracking_data()
                except Exception as e:
                    print(f'[TRACKING] FPGA read error: {e} — recovering to SEARCH')
                    do_lost_recovery(fpga, validator, state)
                    search_det.reset()
                    continue

                # The PL owns its ROI now: it segments skin, recomputes the
                # centroid / bbox / skin-pixel-count, and self-updates the ROI it
                # tracks, exposing it via the CURRENT_ROI registers. The PS only
                # validates that this ROI still holds a real hand, and is the sole
                # authority on giving up — the PL never reports LOCKED or LOST.
                td = tracking_data

                # Async periodic validation against the FPGA's *current* ROI:
                # run MPPalmDet and confirm a real hand sits inside the window the
                # PL is tracking. Repeated failure → reset state and return to SEARCH.
                if validator.is_due(now):
                    val_roi = (
                        max(0,       td.current_roi_x - VAL_PAD),
                        max(0,       td.current_roi_y - VAL_PAD),
                        min(FRAME_W, td.current_roi_x + td.current_roi_w + VAL_PAD),
                        min(FRAME_H, td.current_roi_y + td.current_roi_h + VAL_PAD),
                    )
                    validator.start_validation(frame, val_roi, now)

                result_ready, is_valid = validator.poll_result()
                if result_ready and not is_valid:
                    # Base patience exhausted. Healthy FPGA geometry buys more
                    # time (blur / fast motion); the hard cap still forces LOST.
                    fails   = validator.failure_count
                    healthy = fpga_geometry_healthy(td)
                    if lost_after_validation(fails, healthy):
                        print(f'[VALIDATION] give up (fails={fails} '
                              f'healthy={healthy}) → SEARCH')
                        do_lost_recovery(fpga, validator, state)
                        search_det.reset()
                    else:
                        print(f'[VALIDATION] miss {fails}× but FPGA geometry '
                              f'healthy → holding')

            # ── LOST guard (should not linger here) ───────────────────────────
            elif state.phase == Phase.LOST:
                do_lost_recovery(fpga, validator, state)
                search_det.reset()

            # ── Display ───────────────────────────────────────────────────────
            # IMPORTANT:
            # In hardware mode, do not draw landmarks, rectangles, text, or any OSD on
            # the frame before calling hdmi_out.writeframe().
            #
            # In the current hardware architecture, the same frame written to HDMI OUT is
            # also streamed through the VDMA/broadcaster path into tracker_core.
            # Therefore, any debug drawing added before writeframe() becomes part of the
            # video stream processed by the FPGA.
            #
            # This can contaminate the hardware color filter, especially because the
            # tracker_core relies on Cr/Cb skin-color thresholds.
            #
            # Keep the frame clean before hdmi_out.writeframe(). Draw debug overlays only
            # in stub mode, or only after creating a separate display-only copy.
            if USE_STUB:
                draw_osd(frame, state, tracking_data)
                cv2.imshow(WINDOW_NAME, frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
            else:
                assert hdmi_out is not None
                outframe = hdmi_out.newframe()
                outframe[:] = frame          # clean frame — tracker_core sees uncontaminated video
                hdmi_out.writeframe(outframe)

    except KeyboardInterrupt:
        print('\nInterrupted by user')

    finally:
        try:
            fpga.set_ctrl_bit(BIT_SERVO_HOLD, True)
            fpga.write_ctrl(
                track_enable=False, servo_enable=False,
                servo_hold=True, search_sweep_enable=False,
            )
        except Exception:
            pass
        cap.release()
        if USE_STUB:
            cv2.destroyAllWindows()
        elif hdmi_out is not None:
            hdmi_out.stop()
        print('Shutdown complete')


if __name__ == '__main__':
    main()
