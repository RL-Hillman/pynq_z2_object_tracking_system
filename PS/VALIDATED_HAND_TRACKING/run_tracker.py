"""
run_tracker.py — PYNQ-Z2 hand tracker.
Architecture
------------
PL is the continuous tracker.  The PS only:
  1) finds a HAND with palm.onnx,
  2) learns Cr/Cb thresholds from that hand and seeds the PL,
  3) validates the PL result asynchronously:
       - once every VALIDATION_INTERVAL_SEC (default 1.0 s), and
       - urgently on every new PL snapshot that reports tracking_lost=1.

Validation never rewrites a healthy PL ROI.  If a validation sees the hand in
the PL ROI, hardware is left alone.
After the configured validation miss limit is reached, the PS returns to
full-frame hand SEARCH, but the PL remains enabled and continues tracking
with its current ROI and thresholds. The PL is only briefly disabled when
the PS finds a valid hand and re-seeds it with a new ROI and thresholds.
The palm detector always runs in a background worker, so inference does not
block the clean camera -> HDMI/VDMA -> PL path.

Required files next to this script:
    fpga.py
    mp_palmdet.py
    palm.onnx
    design_1.bit
    design_1.hwh

Typical Jupyter use:
    %run run_tracker.py
"""

from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from mp_palmdet import MPPalmDet
from fpga import FPGA

# =============================================================================
# BOARD / VIDEO CONFIGURATION
# =============================================================================
DESIGN_NAME = "design_1"
RUNTIME_VERSION = "2026-10-02-hand-tracking-final-v3"
BITSTREAM_PATH = os.path.join(HERE, f"{DESIGN_NAME}.bit")
MODEL_PATH = os.path.join(HERE, "palm.onnx")
FPGA_BASE_ADDR = 0x43C30000
CAMERA_INDEX = 0
EXPECTED_FRAME_WIDTH = 640
EXPECTED_FRAME_HEIGHT = 480
VIDEO_BPP = 24
CAMERA_BUFFER_SIZE = 1


# =============================================================================
# HAND DETECTION / VALIDATION
# =============================================================================
VALIDATION_INTERVAL_SEC = 1.0
PERIODIC_MISSES_BEFORE_SEARCH = 2
LOST_MISSES_BEFORE_SEARCH = 1
RECOVERY_MIN_SCORE = 0.75
MODEL_SCORE_TH = 0.50
NMS_TH = 0.20
SEARCH_MIN_SCORE = 0.75
VALIDATION_MIN_SCORE = 0.75
OVERLAP_MIN_RATIO = 0.30
VAL_PAD = 0


# =============================================================================
# HAND COLOUR / INITIAL ROI
# =============================================================================
SKIN_CR_MIN, SKIN_CR_MAX = 133, 173
SKIN_CB_MIN, SKIN_CB_MAX = 77, 127
SKIN_MIN_RATIO = 0.16
PAD_X = 0.35
PAD_UP_Y = 1.00
PAD_DOWN_Y = 0.15
MIN_PAD = 10
SKIN_PCT_LO, SKIN_PCT_HI = 3, 97
SKIN_MARGIN = 12
MIN_SKIN_SAMPLES = 200
SKIN_MAX_BAND = 45


# =============================================================================
# PL / PS SYNCHRONIZATION
# =============================================================================
SNAPSHOT_STALL_TIMEOUT_SEC = 2.0
SNAPSHOT_POLL_SEC = 0.001
MAX_SNAPSHOT_STALLS = 3


# =============================================================================
# CAMERA / RUNTIME SAFETY
# =============================================================================
MAX_CAMERA_FAILS = 30
CAMERA_WATCHDOG_SEC = 5.0
CAMERA_RETRY_SLEEP_SEC = 0.01
RUN_THREAD_JOIN_TIMEOUT_SEC = 3.0


# =============================================================================
# NOTEBOOK STATUS / OUTPUT
# =============================================================================
CONSOLE_VERBOSE = False
PL_UI_INTERVAL_SEC = 1.0


def log(*args, **kwargs):
    if CONSOLE_VERBOSE:
        print(*args, **kwargs)


def validate_configuration():
    """Fail fast on invalid user-editable configuration values."""
    if EXPECTED_FRAME_WIDTH <= 0 or EXPECTED_FRAME_HEIGHT <= 0:
        raise ValueError("EXPECTED_FRAME_WIDTH/HEIGHT must be positive")
    if VIDEO_BPP <= 0:
        raise ValueError("VIDEO_BPP must be positive")
    if CAMERA_INDEX < 0:
        raise ValueError("CAMERA_INDEX must be >= 0")
    if CAMERA_BUFFER_SIZE < 1:
        raise ValueError("CAMERA_BUFFER_SIZE must be >= 1")
    if VALIDATION_INTERVAL_SEC <= 0:
        raise ValueError("VALIDATION_INTERVAL_SEC must be > 0")
    if PERIODIC_MISSES_BEFORE_SEARCH < 1 or LOST_MISSES_BEFORE_SEARCH < 1:
        raise ValueError("validation miss limits must be >= 1")
    for name, value in (
        ("RECOVERY_MIN_SCORE", RECOVERY_MIN_SCORE),
        ("MODEL_SCORE_TH", MODEL_SCORE_TH),
        ("NMS_TH", NMS_TH),
        ("SEARCH_MIN_SCORE", SEARCH_MIN_SCORE),
        ("VALIDATION_MIN_SCORE", VALIDATION_MIN_SCORE),
        ("OVERLAP_MIN_RATIO", OVERLAP_MIN_RATIO),
        ("SKIN_MIN_RATIO", SKIN_MIN_RATIO),
    ):
        if not (0.0 <= value <= 1.0):
            raise ValueError(f"{name} must be in the range 0..1")
    if VAL_PAD < 0 or MIN_PAD < 0:
        raise ValueError("VAL_PAD and MIN_PAD must be >= 0")
    if PAD_X < 0 or PAD_UP_Y < 0 or PAD_DOWN_Y < 0:
        raise ValueError("ROI padding values must be >= 0")
    if not (0 <= SKIN_CR_MIN <= SKIN_CR_MAX <= 255):
        raise ValueError("SKIN Cr range must satisfy 0 <= MIN <= MAX <= 255")
    if not (0 <= SKIN_CB_MIN <= SKIN_CB_MAX <= 255):
        raise ValueError("SKIN Cb range must satisfy 0 <= MIN <= MAX <= 255")
    if not (0 <= SKIN_PCT_LO < SKIN_PCT_HI <= 100):
        raise ValueError("SKIN percentiles must satisfy 0 <= LO < HI <= 100")
    if not (0 <= SKIN_MARGIN <= 255):
        raise ValueError("SKIN_MARGIN must be in the range 0..255")
    if MIN_SKIN_SAMPLES < 1:
        raise ValueError("MIN_SKIN_SAMPLES must be >= 1")
    if not (1 <= SKIN_MAX_BAND <= 255):
        raise ValueError("SKIN_MAX_BAND must be in the range 1..255")
    if SNAPSHOT_STALL_TIMEOUT_SEC <= 0 or SNAPSHOT_POLL_SEC <= 0:
        raise ValueError("snapshot timing values must be > 0")
    if MAX_SNAPSHOT_STALLS < 1:
        raise ValueError("MAX_SNAPSHOT_STALLS must be >= 1")
    if MAX_CAMERA_FAILS < 1:
        raise ValueError("MAX_CAMERA_FAILS must be >= 1")
    if CAMERA_WATCHDOG_SEC <= 0:
        raise ValueError("CAMERA_WATCHDOG_SEC must be > 0")
    if CAMERA_RETRY_SLEEP_SEC < 0:
        raise ValueError("CAMERA_RETRY_SLEEP_SEC must be >= 0")
    if RUN_THREAD_JOIN_TIMEOUT_SEC <= 0:
        raise ValueError("RUN_THREAD_JOIN_TIMEOUT_SEC must be > 0")
    if PL_UI_INTERVAL_SEC <= 0:
        raise ValueError("PL_UI_INTERVAL_SEC must be > 0")


# =============================================================================
# BASIC HELPERS
# =============================================================================

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def has_results(res) -> bool:
    if res is None:
        return False
    try:
        return len(res) > 0
    except TypeError:
        return getattr(res, "size", 0) > 0


def pick_best(results):
    """Highest-confidence palm detection, or None."""
    if not has_results(results):
        return None
    return max(results, key=lambda r: float(r[-1]))


def get_landmarks_xy(r):
    lm = r[4:18]
    xs = [float(lm[i]) for i in range(0, 14, 2)]
    ys = [float(lm[i]) for i in range(1, 14, 2)]
    return xs, ys


def bbox_from_landmarks(x1, y1, x2, y2, xs, ys, fw, fh):
    """Padded initial PL ROI around the palm box + seven landmarks."""
    min_x = min([x1, x2] + xs)
    max_x = max([x1, x2] + xs)
    min_y = min([y1, y2] + ys)
    max_y = max([y1, y2] + ys)
    pad_x = max(int(PAD_X * (max_x - min_x)), MIN_PAD)
    pad_up = max(int(PAD_UP_Y * (max_y - min_y)), MIN_PAD)
    pad_down = max(int(PAD_DOWN_Y * (max_y - min_y)), MIN_PAD)
    ex1 = int(clamp(min_x - pad_x, 0, fw - 1))
    ey1 = int(clamp(min_y - pad_up, 0, fh - 1))
    ex2 = int(clamp(max_x + pad_x, 0, fw - 1))
    ey2 = int(clamp(max_y + pad_down, 0, fh - 1))
    return ex1, ey1, ex2, ey2


def skin_ratio(img_bgr, cr_min, cr_max, cb_min, cb_max) -> float:
    if img_bgr is None or img_bgr.size == 0:
        return 0.0
    ycrcb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YCrCb)
    cr = ycrcb[:, :, 1]
    cb = ycrcb[:, :, 2]
    mask = (
        (cr >= cr_min)
        & (cr <= cr_max)
        & (cb >= cb_min)
        & (cb <= cb_max)
    )
    return float(np.count_nonzero(mask)) / float(mask.size)


def compute_skin_thresholds(frame, roi_xyxy):
    """Fit robust Cr/Cb bounds to clean skin pixels inside the detected hand."""
    ex1, ey1, ex2, ey2 = [int(v) for v in roi_xyxy]
    roi_img = frame[ey1 : ey2 + 1, ex1 : ex2 + 1]
    if roi_img.size == 0:
        return None
    ycrcb = cv2.cvtColor(roi_img, cv2.COLOR_BGR2YCrCb)
    cr = ycrcb[:, :, 1]
    cb = ycrcb[:, :, 2]
    skin = (
        (cr >= SKIN_CR_MIN)
        & (cr <= SKIN_CR_MAX)
        & (cb >= SKIN_CB_MIN)
        & (cb <= SKIN_CB_MAX)
    ).astype(np.uint8)
    # Remove mixed colour pixels on the skin/background boundary.
    skin = cv2.erode(skin, np.ones((3, 3), np.uint8)).astype(bool)
    crs = cr[skin]
    cbs = cb[skin]
    if crs.size < MIN_SKIN_SAMPLES:
        return None
    cr_lo, cr_hi = np.percentile(crs, (SKIN_PCT_LO, SKIN_PCT_HI))
    cb_lo, cb_hi = np.percentile(cbs, (SKIN_PCT_LO, SKIN_PCT_HI))
    if (cr_hi - cr_lo) > SKIN_MAX_BAND or (cb_hi - cb_lo) > SKIN_MAX_BAND:
        return None
    m = SKIN_MARGIN
    return (
        int(clamp(cr_lo - m, SKIN_CR_MIN, SKIN_CR_MAX)),
        int(clamp(cr_hi + m, SKIN_CR_MIN, SKIN_CR_MAX)),
        int(clamp(cb_lo - m, SKIN_CB_MIN, SKIN_CB_MAX)),
        int(clamp(cb_hi + m, SKIN_CB_MIN, SKIN_CB_MAX)),
    )


def overlap_ratio(box_xyxy, roi_xyxy) -> float:
    """Intersection / smaller area, for detector box versus PL validation ROI."""
    ix1 = max(float(box_xyxy[0]), float(roi_xyxy[0]))
    iy1 = max(float(box_xyxy[1]), float(roi_xyxy[1]))
    ix2 = min(float(box_xyxy[2]), float(roi_xyxy[2]))
    iy2 = min(float(box_xyxy[3]), float(roi_xyxy[3]))
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    a = max(0.0, float(box_xyxy[2]) - float(box_xyxy[0])) * max(
        0.0, float(box_xyxy[3]) - float(box_xyxy[1])
    )
    b = max(0.0, float(roi_xyxy[2]) - float(roi_xyxy[0])) * max(
        0.0, float(roi_xyxy[3]) - float(roi_xyxy[1])
    )
    m = min(a, b)
    return inter / m if m > 0 else 0.0


def validation_roi_from_pl(td, fw, fh, pad):
    """Convert PL x/y/w/h into detector xyxy validation box."""
    x1 = max(0, int(td.roi_x) - int(pad))
    y1 = max(0, int(td.roi_y) - int(pad))
    x2 = min(int(fw), int(td.roi_x) + int(td.roi_w) + int(pad))
    y2 = min(int(fh), int(td.roi_y) + int(td.roi_h) + int(pad))
    return x1, y1, x2, y2


def best_hand_in_roi(results, val_roi):

    if not has_results(results):
        return False, 0.0, 0.0, 0.0
    best_score = 0.0
    best_overlap = 0.0
    global_best_score = 0.0
    present = False
    rx1, ry1, rx2, ry2 = map(float, val_roi)
    for r in results:
        score = float(r[-1])
        global_best_score = max(global_best_score, score)
        if score < VALIDATION_MIN_SCORE:
            continue
        box = (float(r[0]), float(r[1]), float(r[2]), float(r[3]))
        ov = overlap_ratio(box, val_roi)
        cx = 0.5 * (box[0] + box[2])
        cy = 0.5 * (box[1] + box[3])
        center_inside = rx1 <= cx < rx2 and ry1 <= cy < ry2
        if center_inside and (
            ov > best_overlap or (ov == best_overlap and score > best_score)
        ):
            best_overlap = ov
            best_score = score
        if center_inside and ov >= OVERLAP_MIN_RATIO:
            present = True
    return present, best_score, best_overlap, global_best_score


# =============================================================================
# PALM WORKER — ONE BACKGROUND INFERENCE AT A TIME
# =============================================================================

@dataclass(frozen=True)
class PalmJobResult:
    job: str
    results: object
    frame: object
    val_roi: tuple | None


class AsyncPalmWorker:
    """Run MPPalmDet.infer() off the video/PL thread."""

    def __init__(self, detector: MPPalmDet):
        self._detector = detector
        self._lock = threading.Lock()
        self._running = False
        self._ready = False
        self._result = None
        self._generation = 0

    def idle(self):
        with self._lock:
            return not self._running and not self._ready

    def submit(self, job, frame, val_roi=None) -> bool:
        with self._lock:
            if self._running or self._ready:
                return False
            self._running = True
            gen = self._generation
        threading.Thread(
            target=self._run,
            args=(str(job), frame.copy(), val_roi, gen),
            daemon=True,
            name=f"palm-{job}",
        ).start()
        return True

    def _run(self, job, frame, val_roi, generation):
        try:
            results = self._detector.infer(frame)
        except Exception as exc:
            print(f"[ERROR] palm inference: {exc}", flush=True)
            results = None
        item = PalmJobResult(job, results, frame, val_roi)
        with self._lock:
            self._running = False
            if generation != self._generation:
                return
            self._result = item
            self._ready = True

    def poll(self):
        with self._lock:
            if not self._ready:
                return None
            item = self._result
            self._result = None
            self._ready = False
            return item

    def reset(self):
        with self._lock:
            self._generation += 1
            self._ready = False
            self._result = None


# =============================================================================
# MINIMAL NOTEBOOK STATUS UI
# =============================================================================

class NotebookHandUI:
    """Minimal Jupyter display: PS state + latest PL tracking data."""

    def __init__(self, on_close=None):
        self._closed = False
        self._on_close = on_close
        self._ps_widget = None
        self._config_widget = None
        self._validation_widget = None
        self._event_widget = None
        self._pl_widget = None
        self._close_button = None
        self._widget_box = None

    def set_status(self, text):
        if self._ps_widget is not None:
            try:
                self._ps_widget.value = (
                    "<pre style='margin:0'>PS | " + str(text) + "</pre>"
                )
            except Exception:
                pass

    def set_initial_config(self, roi, thresholds, score=None):
        if self._config_widget is None:
            return
        cr_min, cr_max, cb_min, cb_max = thresholds
        score_text = "" if score is None else f" | score={float(score):.2f}"
        try:
            self._config_widget.value = (
                "<pre style='margin:0'>"
                f"PS LOCK | ROI=({roi[0]},{roi[1]},{roi[2]},{roi[3]}) | "
                f"Cr=[{cr_min},{cr_max}] | Cb=[{cb_min},{cb_max}]"
                f"{score_text}"
                "</pre>"
            )
        except Exception:
            pass

    def set_pl_data(self, td):
        if self._pl_widget is None or td is None:
            return
        try:
            self._pl_widget.value = (
                "<pre style='margin:0'>"
                f"PL | snapshot={td.snapshot} | "
                f"object_center=({td.object_cx},{td.object_cy}) | "
                f"object_size={td.object_w}x{td.object_h} | "
                f"pixels={td.pixel_count} | "
                f"ROI=({td.roi_x},{td.roi_y},{td.roi_w},{td.roi_h}) | "
                f"pred=({td.pred_cx},{td.pred_cy}) | "
                f"lost={int(td.tracking_lost)}"
                "</pre>"
            )
        except Exception:
            pass

    def set_validation(self, text):
        if self._validation_widget is not None:
            try:
                self._validation_widget.value = (
                    "<pre style='margin:0'>PS " + str(text) + "</pre>"
                )
            except Exception:
                pass

    def set_event(self, text):
        if self._event_widget is not None:
            try:
                self._event_widget.value = (
                    "<pre style='margin:0'>PS EVENT | " + str(text) + "</pre>"
                )
            except Exception:
                pass

    def mark_search(self, reason="SEARCHING FOR HAND"):
        self.set_status(reason)

    def mark_tracking(self):
        self.set_status("TRACKING | validation active")

    def show(self):
        try:
            import ipywidgets as widgets
            from IPython.display import display
        except Exception as exc:
            raise RuntimeError(
                "This notebook status display requires ipywidgets on the PYNQ image"
            ) from exc
        self._ps_widget = widgets.HTML(
            value="<pre style='margin:0'>PS | SEARCHING FOR HAND...</pre>"
        )
        self._config_widget = widgets.HTML(
            value="<pre style='margin:0'>PS LOCK | waiting for hand...</pre>"
        )
        self._validation_widget = widgets.HTML(
            value="<pre style='margin:0'>PS VALIDATION | waiting for hand lock...</pre>"
        )
        self._event_widget = widgets.HTML(
            value="<pre style='margin:0'>PS EVENT | tracker started</pre>"
        )
        self._pl_widget = widgets.HTML(
            value="<pre style='margin:0'>PL | waiting for hand lock...</pre>"
        )
        self._close_button = widgets.Button(
            description="STOP & CLOSE",
            button_style="danger",
            tooltip="Safely stop tracking and release hardware resources",
        )
        self._close_button.on_click(self._close_clicked)
        self._widget_box = widgets.VBox([
            self._ps_widget,
            self._config_widget,
            self._validation_widget,
            self._event_widget,
            self._pl_widget,
            self._close_button,
        ])
        display(self._widget_box)
        return self

    def _close_clicked(self, _button=None):
        if self._closed:
            return
        if self._on_close is not None:
            self._on_close()

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._widget_box is not None:
            try:
                self._widget_box.close()
            except Exception:
                pass
            self._widget_box = None
        self._close_button = None


# =============================================================================
# REAL PYNQ HAND TRACKER
# =============================================================================

class Phase(Enum):
    SEARCH = auto()
    TRACKING = auto()


class PYNQHandTracker:
    """Hand-only PYNQ runtime: PL tracks, PS acquires and validates."""

    def __init__(
        self,
        bitstream_path: str = BITSTREAM_PATH,
        model_path: str = MODEL_PATH,
        base_address: int = FPGA_BASE_ADDR,
        camera_index: int = CAMERA_INDEX,
    ):
        validate_configuration()
        if not os.path.exists(bitstream_path):
            raise FileNotFoundError(f"bitstream not found: {bitstream_path}")
        hwh_path = os.path.splitext(bitstream_path)[0] + ".hwh"
        if not os.path.exists(hwh_path):
            raise FileNotFoundError(
                f"PYNQ needs the .hwh metadata next to the .bit: {hwh_path}"
            )
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"palm model not found: {model_path}")
        self.fpga = FPGA(bitstream_path, base_address)
        self.fpga.set_tracking_enable(False)
        fw, fh = self.fpga.read_frame_size()
        log(f"[PL] FRAME_SIZE={fw}x{fh}", flush=True)
        if (fw, fh) != (EXPECTED_FRAME_WIDTH, EXPECTED_FRAME_HEIGHT):
            raise RuntimeError(
                f"PL reports {fw}x{fh} but this runtime expects "
                f"{EXPECTED_FRAME_WIDTH}x{EXPECTED_FRAME_HEIGHT}"
            )
        self.frame_width = int(fw)
        self.frame_height = int(fh)
        self.detector = MPPalmDet(
            model_path,
            nmsThreshold=NMS_TH,
            scoreThreshold=MODEL_SCORE_TH,
        )
        self.worker = AsyncPalmWorker(self.detector)
        self.hdmi_out = self._setup_hdmi_out()
        self.cap = self._open_camera(camera_index)
        self._run_thread = None
        self._run_stop = threading.Event()
        self._ui = None
        self._closed = False
        self._hdmi_running = False
        self.phase = Phase.SEARCH
        self._pl_roi = (0, 0, self.frame_width, self.frame_height)
        self._pl_lost = False
        self._pl_pred = (0, 0)
        self._last_tracking_data = None
        self._initial_roi = None
        self._pl_thresholds = None
        self._last_validation_submit = 0.0
        self._last_validation_ok = None
        self._validation_fail_count = 0
        self._last_validation_job = None
        self._last_validation_score = 0.0
        self._last_validation_overlap = 0.0
        # Each new LOST snapshot requests urgent validation.  If inference is
        # already busy, multiple hardware reports coalesce into one pending urgent
        # check; as soon as the worker is free it runs before periodic validation.
        self._lost_validation_requested = False
        self._lost_reports = 0

    # -------------------------------------------------------------------------
    # Hardware / camera setup
    # -------------------------------------------------------------------------

    def _setup_hdmi_out(self):
        from pynq.lib.video import VideoMode, PIXEL_BGR
        overlay = self.fpga.overlay
        try:
            hdmi_out = overlay.video.hdmi_out
        except AttributeError as exc:
            raise RuntimeError(
                "overlay.video.hdmi_out not found; expected /video/hdmi_out"
            ) from exc
        hdmi_out.configure(
            VideoMode(self.frame_width, self.frame_height, VIDEO_BPP), PIXEL_BGR
        )
        return hdmi_out

    def _open_camera(self, index):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open camera index {index}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        return cap

    def _ensure_hdmi_running(self):
        if not self._hdmi_running:
            self.hdmi_out.start()
            self._hdmi_running = True

    def _stop_hdmi_stream(self):
        if self._hdmi_running:
            try:
                self.hdmi_out.stop()
            finally:
                self._hdmi_running = False

    # -------------------------------------------------------------------------
    # PL frame path / coherent reads
    # -------------------------------------------------------------------------

    def _apply_pl_snapshot(self, td):
        self._pl_roi = (
            int(td.roi_x),
            int(td.roi_y),
            int(td.roi_w),
            int(td.roi_h),
        )
        self._pl_lost = bool(td.tracking_lost)
        self._pl_pred = (int(td.pred_cx), int(td.pred_cy))
        self._last_tracking_data = td
        return td

    def _feed_pl(self, frame):
        """Send a CLEAN frame to the PL.  Never draw PS graphics on this frame."""
        outframe = self.hdmi_out.newframe()
        outframe[:] = frame
        self.hdmi_out.writeframe(outframe)

    def _wait_for_new_snapshot(self, baseline, timeout=SNAPSHOT_STALL_TIMEOUT_SEC):
        baseline = int(baseline) & 0xFFFFFFFF
        deadline = time.monotonic() + float(timeout)
        while not self._run_stop.is_set():
            snap = int(self.fpga.read_snapshot()) & 0xFFFFFFFF
            if snap != baseline:
                td = self.fpga.read_tracking_data()
                if (int(td.snapshot) & 0xFFFFFFFF) != baseline:
                    return self._apply_pl_snapshot(td)
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"PL snapshot counter did not advance from {baseline} "
                    f"within {timeout:.3f}s"
                )
            time.sleep(SNAPSHOT_POLL_SEC)
        return None

    def _feed_pl_and_sync(self, frame):
        # Capture the baseline before submitting the frame, then wait for the PL
        # to publish a newer coherent result for work performed after that point.
        baseline = int(self.fpga.read_snapshot()) & 0xFFFFFFFF
        self._feed_pl(frame)
        return self._wait_for_new_snapshot(baseline)

    # -------------------------------------------------------------------------
    # Acquisition / validation
    # -------------------------------------------------------------------------

    def _lock_from_detection(self, frame, results):
        recovering = self._initial_roi is not None
        best = pick_best(results)
        if best is None or float(best[-1]) < SEARCH_MIN_SCORE:
            return False
        x1, y1, x2, y2 = map(int, best[:4])
        xs, ys = get_landmarks_xy(best)
        ex1, ey1, ex2, ey2 = bbox_from_landmarks(
            x1,
            y1,
            x2,
            y2,
            xs,
            ys,
            self.frame_width,
            self.frame_height,
        )
        candidate_img = frame[ey1 : ey2 + 1, ex1 : ex2 + 1]
        if (
            skin_ratio(
                candidate_img,
                SKIN_CR_MIN,
                SKIN_CR_MAX,
                SKIN_CB_MIN,
                SKIN_CB_MAX,
            )
            < SKIN_MIN_RATIO
        ):
            return False
        # Calibrate on the tight detector/landmark box, without the large ROI pad.
        hx1 = int(clamp(min([x1, x2] + xs), 0, self.frame_width - 1))
        hy1 = int(clamp(min([y1, y2] + ys), 0, self.frame_height - 1))
        hx2 = int(clamp(max([x1, x2] + xs), 0, self.frame_width - 1))
        hy2 = int(clamp(max([y1, y2] + ys), 0, self.frame_height - 1))
        thresholds = compute_skin_thresholds(frame, (hx1, hy1, hx2, hy2))
        if thresholds is None:
            return False
        roi = (ex1, ey1, ex2 - ex1 + 1, ey2 - ey1 + 1)
        self.fpga.set_tracking_enable(False)
        self.fpga.write_initial_roi(*roi)
        self.fpga.write_object_thresholds(*thresholds)
        self.fpga.set_tracking_enable(True)
        self._initial_roi = roi
        self._pl_thresholds = thresholds
        self._pl_roi = roi
        self._pl_lost = False
        self._pl_pred = (0, 0)
        self._validation_fail_count = 0
        self._last_validation_ok = True
        self._last_validation_job = "LOCK"
        self._last_validation_score = float(best[-1])
        self._last_validation_overlap = 1.0
        self._last_validation_submit = time.monotonic()
        self._lost_validation_requested = False
        log(
            f"[LOCK] roi={roi} "
            f"Cr[{thresholds[0]},{thresholds[1]}] "
            f"Cb[{thresholds[2]},{thresholds[3]}] "
            f"score={float(best[-1]):.2f} -> PL TRACKING",
            flush=True,
        )
        if self._ui is not None:
            self._ui.set_initial_config(roi, thresholds, float(best[-1]))
            self._ui.set_validation(
                "VALIDATION | LOCK OK"
            )
            self._ui.mark_tracking()
            self._ui.set_event(
                "HAND FOUND AGAIN -> PL RE-SEEDED"
                if recovering
                else "HAND FOUND -> INITIAL PL LOCK"
            )
        return True

    def _consume_worker_result(self):
        item = self.worker.poll()
        if item is None:
            return
        if item.job == "search":
            if self.phase != Phase.SEARCH:
                return
            if self._lock_from_detection(item.frame, item.results):
                self.worker.reset()
                self.phase = Phase.TRACKING
            return
        if item.job not in ("periodic", "lost"):
            return
        # Ignore validation results that arrive after we already returned to SEARCH.
        if self.phase != Phase.TRACKING or item.val_roi is None:
            return
        present, score, ov, global_score = best_hand_in_roi(
            item.results, item.val_roi
        )
        self._last_validation_job = item.job
        self._last_validation_ok = bool(present)
        self._last_validation_score = float(score)
        self._last_validation_overlap = float(ov)
        if present:
            self._validation_fail_count = 0
            log(
                f"[VALIDATE-{item.job.upper()}] HAND OK "
                f"score={score:.2f} overlap={ov:.2f} -> PL unchanged",
                flush=True,
            )
            if self._ui is not None:
                self._ui.set_validation(
                    f"VALIDATION | {item.job.upper()} OK | "
                    f"score={score:.2f} overlap={ov:.2f} | PL UNCHANGED"
                )
                self._ui.set_event(
                    f"{item.job.upper()} VALIDATION OK -> PL UNCHANGED"
                )
            return
        # IMPORTANT FAST-RECOVERY PATH:
        # This inference was already run on the FULL frame.  If a strong hand
        # exists but is no longer inside the PL ROI, do not throw that result
        # away and wait for more misses.  Re-seed immediately from THIS SAME
        # detector result.
        if global_score >= RECOVERY_MIN_SCORE:
            log(
                f"[VALIDATE-{item.job.upper()}] hand outside PL ROI "
                f"(global score={global_score:.2f}) -> immediate re-seed",
                flush=True,
            )
            if self._ui is not None:
                self._ui.set_validation(
                    f"VALIDATION | {item.job.upper()}: HAND OUTSIDE ROI -> "
                    "IMMEDIATE RECOVERY"
                )
                self._ui.set_event(
                    f"HAND FOUND OUTSIDE PL ROI | score={global_score:.2f} -> RECOVERING"
                )
            if self._lock_from_detection(item.frame, item.results):
                self._validation_fail_count = 0
                self._last_validation_job = f"{item.job}-RECOVER"
                self._last_validation_ok = True
                self.phase = Phase.TRACKING
                return
            # A detector box existed but failed the skin/threshold safety gates.
            # Fall through to the miss policy rather than writing a weak seed.
        self._validation_fail_count += 1
        limit = (
            LOST_MISSES_BEFORE_SEARCH
            if item.job == "lost"
            else PERIODIC_MISSES_BEFORE_SEARCH
        )
        log(
            f"[VALIDATE-{item.job.upper()}] MISS "
            f"inside_score={score:.2f} overlap={ov:.2f} "
            f"global_score={global_score:.2f} "
            f"fails={self._validation_fail_count}/{limit}",
            flush=True,
        )
        if self._ui is not None:
            self._ui.set_validation(
                f"VALIDATION | {item.job.upper()} MISS | "
                f"fails={self._validation_fail_count}/{limit} | "
                f"global_score={global_score:.2f}"
            )
            self._ui.set_event(
                f"{item.job.upper()} MISS | "
                f"fails={self._validation_fail_count}/{limit}"
            )
        if self._validation_fail_count >= limit:
            self._return_to_search(
                f"{item.job.upper()} validation could not verify the hand"
            )

    def _return_to_search(self, reason):
        """Put only the PS into full-frame SEARCH; keep the PL tracker running.

        The current PL ROI/thresholds remain active and the PL keeps receiving
        frames.  The PL is disabled only momentarily when a new valid hand is
        found and we atomically re-seed ROI/thresholds, or during shutdown.
        """
        self.worker.reset()
        self.phase = Phase.SEARCH
        self._validation_fail_count = 0
        self._last_validation_ok = False
        self._lost_validation_requested = False
        self._last_validation_submit = 0.0
        log(
            f"[SEARCH] {reason} -> PS full-frame palm search; PL keeps tracking",
            flush=True,
        )
        if self._ui is not None:
            self._ui.mark_search(
                f"SEARCH — {reason}; PS looking for hand while PL keeps tracking..."
            )
            self._ui.set_validation(
                "VALIDATION | reacquiring hand; PL still active"
            )
            self._ui.set_event(
                f"PS LOST VALIDATION -> SEARCHING AGAIN | {reason} | "
                "PL CONTINUES"
            )

    def _schedule_detection(self, frame, now):
        """Priority: SEARCH > LOST urgent > 1-second periodic validation."""
        if not self.worker.idle():
            return
        if self.phase == Phase.SEARCH:
            if self.worker.submit("search", frame):
                return
        if self.phase != Phase.TRACKING or self._last_tracking_data is None:
            return
        td = self._last_tracking_data
        # LOST reports are urgent and take precedence over the one-second timer.
        if self._lost_validation_requested:
            roi = validation_roi_from_pl(
                td,
                self.frame_width,
                self.frame_height,
                pad=0,  # exact current PL ROI for LOST validation
            )
            if self.worker.submit("lost", frame, val_roi=roi):
                self._lost_validation_requested = False
                self._last_validation_submit = now
                if self._ui is not None:
                    self._ui.set_validation(
                        "VALIDATION | LOST -> checking now..."
                    )
                return
        if now - self._last_validation_submit >= VALIDATION_INTERVAL_SEC:
            roi = validation_roi_from_pl(
                td,
                self.frame_width,
                self.frame_height,
                pad=VAL_PAD,
            )
            if self.worker.submit("periodic", frame, val_roi=roi):
                self._last_validation_submit = now
                if self._ui is not None:
                    self._ui.set_validation(
                        "VALIDATION | checking..."
                    )

    # -------------------------------------------------------------------------
    # Main tracking thread
    # -------------------------------------------------------------------------

    def start(self):
        if self._closed:
            raise RuntimeError("tracker is closed; create a new PYNQHandTracker")
        if self._run_thread is not None and self._run_thread.is_alive():
            return True
        self.fpga.set_tracking_enable(False)
        self.worker.reset()
        self.phase = Phase.SEARCH
        self._validation_fail_count = 0
        self._lost_validation_requested = False
        self._last_validation_submit = 0.0
        self._ensure_hdmi_running()
        self._run_stop.clear()
        self._run_thread = threading.Thread(
            target=self.run,
            daemon=True,
            name="pynq-hand-tracker",
        )
        self._run_thread.start()
        if self._run_thread is None or not self._run_thread.is_alive():
            raise RuntimeError("tracking thread failed to start")
        log("[MODE] SEARCH hand -> PL tracking", flush=True)
        return True

    def run(self):
        camera_fails = 0
        snapshot_stalls = 0
        last_camera_ok = time.monotonic()
        last_pl_ui_update = 0.0
        try:
            while not self._run_stop.is_set():
                ok, frame = self.cap.read()
                if not ok or frame is None:
                    camera_fails += 1
                    if (
                        camera_fails >= MAX_CAMERA_FAILS
                        or time.monotonic() - last_camera_ok >= CAMERA_WATCHDOG_SEC
                    ):
                        print("[CAMERA] camera stalled; stopping run loop", flush=True)
                        break
                    time.sleep(CAMERA_RETRY_SLEEP_SEC)
                    continue
                camera_fails = 0
                last_camera_ok = time.monotonic()
                if (
                    frame.shape[1] != self.frame_width
                    or frame.shape[0] != self.frame_height
                ):
                    frame = cv2.resize(
                        frame,
                        (self.frame_width, self.frame_height),
                        interpolation=cv2.INTER_LINEAR,
                    )
                now = time.monotonic()
                # Consume a completed palm inference before scheduling another one.
                self._consume_worker_result()
                if self.phase == Phase.SEARCH and self._initial_roi is None:
                    # Initial acquisition: no PL target exists yet, so only feed
                    # clean frames while the PS searches for the first hand.
                    self._feed_pl(frame)
                    snapshot_stalls = 0
                else:
                    # TRACKING, or PS re-search after a previous lock:
                    # the PL remains active and continues processing every frame.
                    try:
                        td = self._feed_pl_and_sync(frame)
                        if td is None:
                            break
                        snapshot_stalls = 0
                    except TimeoutError as exc:
                        snapshot_stalls += 1
                        print(
                            f"[PL] snapshot stall "
                            f"{snapshot_stalls}/{MAX_SNAPSHOT_STALLS}: {exc}",
                            flush=True,
                        )
                        if snapshot_stalls >= MAX_SNAPSHOT_STALLS:
                            print(
                                "[PL] repeated snapshot stalls; stopping run loop",
                                flush=True,
                            )
                            break
                        continue
                    except Exception as exc:
                        print(f"[VIDEO/PL] frame failed: {exc}", flush=True)
                        break
                    # Every new snapshot with LOST=1 requests an urgent validation.
                    if td.tracking_lost:
                        self._lost_reports += 1
                        self._lost_validation_requested = True
                        log(
                            f"[PL-LOST] snapshot={td.snapshot} report={self._lost_reports} "
                            "-> urgent validation requested",
                            flush=True,
                        )
                        if self._ui is not None:
                            self._ui.set_event(
                                f"PL LOST | snapshot={td.snapshot} | "
                                "urgent PS validation requested"
                            )
                    if (
                        self._ui is not None
                        and now - last_pl_ui_update >= PL_UI_INTERVAL_SEC
                    ):
                        self._ui.set_pl_data(td)
                        last_pl_ui_update = now
                # SEARCH, urgent LOST validation, or normal 1-second validation.
                self._schedule_detection(frame, now)
                # Notebook status panels are updated from PS/PL events above.
        finally:
            try:
                self.fpga.set_tracking_enable(False)
            except Exception:
                pass
            if self._ui is not None and not self._closed:
                self._ui.set_status("TRACKER STOPPED — check console output")

    # -------------------------------------------------------------------------
    # Notebook control / status / shutdown
    # -------------------------------------------------------------------------

    def run_interactive(self):
        """Show the minimal PS/PL status display and start searching immediately."""
        print(f"[RUNTIME] {RUNTIME_VERSION}", flush=True)
        self._ui = NotebookHandUI(on_close=self.close).show()
        self._ui.mark_search("SEARCHING FOR HAND...")
        self.start()
        return self

    def status(self):
        td = self._last_tracking_data
        return {
            "phase": self.phase.name,
            "tracking": bool(self.phase == Phase.TRACKING),
            "initial_roi": self._initial_roi,
            "thresholds": self._pl_thresholds,
            "pl_snapshot": None if td is None else int(td.snapshot),
            "pl_roi": tuple(self._pl_roi),
            "pl_pred": tuple(self._pl_pred),
            "pl_lost": bool(self._pl_lost),
            "validation_job": self._last_validation_job,
            "validation_ok": self._last_validation_ok,
            "validation_score": self._last_validation_score,
            "validation_overlap": self._last_validation_overlap,
            "validation_fail_count": self._validation_fail_count,
            "lost_reports": self._lost_reports,
            "urgent_lost_validation_pending": bool(
                self._lost_validation_requested
            ),
        }

    def stop_tracking(self):
        self._run_stop.set()
        try:
            self.fpga.set_tracking_enable(False)
        except Exception:
            pass
        thread = self._run_thread
        if thread is None:
            return True
        if threading.current_thread() is thread:
            return False
        thread.join(timeout=RUN_THREAD_JOIN_TIMEOUT_SEC)
        stopped = not thread.is_alive()
        if stopped:
            self._run_thread = None
        return stopped

    def close(self):
        if self._closed:
            return
        self._closed = True
        ui = self._ui
        self._ui = None
        if ui is not None:
            try:
                ui.close()
            except Exception:
                pass
        stopped = self.stop_tracking()
        self.worker.reset()
        if not stopped:
            print(
                "[SHUTDOWN] tracking_enable=0, but run thread did not stop "
                "before timeout; leaving HDMI/camera allocated to avoid a race",
                flush=True,
            )
            return
        try:
            self._stop_hdmi_stream()
            log("[SHUTDOWN] HDMI/VDMA stopped", flush=True)
        except Exception as exc:
            print(f"[SHUTDOWN] HDMI stop failed: {exc}", flush=True)
        try:
            self.cap.release()
            log("[SHUTDOWN] camera released", flush=True)
        except Exception as exc:
            print(f"[SHUTDOWN] camera release failed: {exc}", flush=True)
        log("[SHUTDOWN] COMPLETE", flush=True)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
# `%run run_tracker.py` starts automatic hand search immediately.
tracker = None
if __name__ == "__main__":
    tracker = PYNQHandTracker()
    tracker.run_interactive()
