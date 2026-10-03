"""
full_system_simulation.py

PC simulation of the PYNQ-Z2 object-tracking system.

The simulator provides a live desktop view for selecting and tracking an object:

    - A normal live camera/debug window is always visible.
    - Press S at the moment you want to choose the object.
    - The current frame freezes in an ROI selection window.
    - Draw the ROI and press ENTER/SPACE.
    - Tracking starts immediately.
    - "REAL SYSTEM - exact PL HDMI" shows the simulated hardware output.
    - Hardware parameters are edited in HWConfig at the top of this file before each run.


Continuous tracking:
    rgb_to_ycrcb
      -> roi_mask_generator
      -> mask_cleaner
      -> object_measurement
      -> roi_tracker_update
      -> object_display

No PYNQ board is required. This runs on a normal PC with OpenCV + NumPy.


Keys:
    S       select/reselect object from the current live frame
    R       reset tracking and return to live selection mode
    Q/ESC   quit

"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace

import cv2
import numpy as np


# =============================================================================
# SIMULATION PARAMETERS - EDIT THESE BEFORE RUNNING
# =============================================================================
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


# Defaults below match the current VHDL.
# Change these values before running to test a different hardware tuning.
# When satisfied, copy the chosen values back to the VHDL.

@dataclass(frozen=True)
class HWConfig:
    # vision_tracker_pkg.vhd
    vel_max: int = 120

    # mask_cleaner generic map in vision_tracker_logic_top.vhd
    kernel_size: int = 5

    # roi_tracker_update generic map in vision_tracker_logic_top.vhd
    pad_base: int = 8
    pad_max: int = 40

    max_track_roi_w: int = 200
    max_track_roi_h: int = 200

    count_floor: int = 150
    count_shift: int = 3

    loss_base: int = 10
    loss_ramp: int = 5
    loss_min: int = 5
    loss_max: int = 40

    max_loss_w: int = 240
    max_loss_h: int = 240

    # object_display defaults
    roi_border_thickness: int = 2
    center_radius: int = 4


BASELINE = HWConfig()

# PS seed algorithm used by run_object_tracker_pynq.py.
INNER_FRAC = 0.72
THRESH_PERCENTILE_LOW = 5
THRESH_PERCENTILE_HIGH = 95
THRESH_MARGIN = 6


# =============================================================================
# BASIC HELPERS
# =============================================================================


def clamp(value: int, lo: int, hi: int) -> int:
    if hi < lo:
        hi = lo
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def div_trunc_zero(num: int, den: int) -> int:
    """VHDL integer division semantics: truncate toward zero."""
    if den <= 0:
        raise ValueError("den must be positive")
    if num >= 0:
        return num // den
    return -((-num) // den)


def clamp_bbox(bbox, fw=FRAME_WIDTH, fh=FRAME_HEIGHT):
    x, y, w, h = [int(round(v)) for v in bbox]
    x = max(0, min(fw - 1, x))
    y = max(0, min(fh - 1, y))
    w = max(1, min(fw - x, w))
    h = max(1, min(fh - y, h))
    return x, y, w, h


def inner_crop(img, frac=INNER_FRAC):
    h, w = img.shape[:2]
    nw = max(2, int(round(w * frac)))
    nh = max(2, int(round(h * frac)))
    x1 = max(0, (w - nw) // 2)
    y1 = max(0, (h - nh) // 2)
    x2 = min(w, x1 + nw)
    y2 = min(h, y1 + nh)
    return img[y1:y2, x1:x2]


def normalized_config(cfg: HWConfig) -> HWConfig:
    """Keep tunable values inside legal ranges used by the current VHDL model."""
    k = max(3, int(cfg.kernel_size))
    if k % 2 == 0:
        k += 1

    loss_min = max(0, int(cfg.loss_min))
    loss_max = max(loss_min, int(cfg.loss_max))

    return replace(
        cfg,
        vel_max=max(1, int(cfg.vel_max)),
        kernel_size=k,
        pad_base=max(0, int(cfg.pad_base)),
        pad_max=max(0, int(cfg.pad_max)),
        max_track_roi_w=clamp(int(cfg.max_track_roi_w), 1, FRAME_WIDTH),
        max_track_roi_h=clamp(int(cfg.max_track_roi_h), 1, FRAME_HEIGHT),
        count_floor=max(0, int(cfg.count_floor)),
        count_shift=clamp(int(cfg.count_shift), 0, 16),
        loss_base=max(0, int(cfg.loss_base)),
        loss_ramp=max(0, int(cfg.loss_ramp)),
        loss_min=loss_min,
        loss_max=loss_max,
        max_loss_w=clamp(int(cfg.max_loss_w), 1, FRAME_WIDTH),
        max_loss_h=clamp(int(cfg.max_loss_h), 1, FRAME_HEIGHT),
        roi_border_thickness=max(1, int(cfg.roi_border_thickness)),
        center_radius=max(0, int(cfg.center_radius)),
    )


# =============================================================================
# rgb_to_ycrcb.vhd - FRAME-LEVEL RESULT
# =============================================================================


def rgb_to_crcb_hdl(frame_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Exact integer Cr/Cb arithmetic used by rgb_to_ycrcb.vhd."""
    b = frame_bgr[:, :, 0].astype(np.int64)
    g = frame_bgr[:, :, 1].astype(np.int64)
    r = frame_bgr[:, :, 2].astype(np.int64)

    cb = ((-43 * r - 85 * g + 128 * b) >> 8) + 128
    cr = ((128 * r - 107 * g - 21 * b) >> 8) + 128

    if (
        np.any(cb < 0) or np.any(cb > 255)
        or np.any(cr < 0) or np.any(cr > 255)
    ):
        raise OverflowError("RGB -> Cr/Cb escaped expected 8-bit range")

    return cr.astype(np.uint8), cb.astype(np.uint8)


# =============================================================================
# INITIAL Cr/Cb LEARNING - SAME METHOD AS CURRENT PYNQ RUNTIME
# =============================================================================


def learn_pl_thresholds(bgr: np.ndarray) -> tuple[int, int, int, int]:
    cr, cb = rgb_to_crcb_hdl(inner_crop(bgr))
    crv = cr.reshape(-1)
    cbv = cb.reshape(-1)

    if len(crv) < 10:
        return 0, 255, 0, 255

    cr_lo, cr_hi = np.percentile(
        crv, [THRESH_PERCENTILE_LOW, THRESH_PERCENTILE_HIGH]
    )
    cb_lo, cb_hi = np.percentile(
        cbv, [THRESH_PERCENTILE_LOW, THRESH_PERCENTILE_HIGH]
    )

    return (
        int(max(0, np.floor(cr_lo) - THRESH_MARGIN)),
        int(min(255, np.ceil(cr_hi) + THRESH_MARGIN)),
        int(max(0, np.floor(cb_lo) - THRESH_MARGIN)),
        int(min(255, np.ceil(cb_hi) + THRESH_MARGIN)),
    )


# =============================================================================
# roi_mask_generator.vhd - FRAME-LEVEL RESULT
# =============================================================================


def raw_roi_mask_hdl(
    frame_bgr: np.ndarray,
    roi: tuple[int, int, int, int],
    thresholds: tuple[int, int, int, int],
) -> np.ndarray:
    """
    Current VHDL behavior:
      * ROI right/bottom boundaries are exclusive.
      * Cr/Cb limits are inclusive.
      * pixels outside the frame-stable ROI are zero.
    """
    rx, ry, rw, rh = map(int, roi)
    cr_min, cr_max, cb_min, cb_max = map(int, thresholds)

    raw = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype=np.uint8)

    x0 = clamp(rx, 0, FRAME_WIDTH)
    y0 = clamp(ry, 0, FRAME_HEIGHT)
    x1 = clamp(rx + rw, 0, FRAME_WIDTH)
    y1 = clamp(ry + rh, 0, FRAME_HEIGHT)

    if x1 <= x0 or y1 <= y0:
        return raw

    cr, cb = rgb_to_crcb_hdl(frame_bgr[y0:y1, x0:x1])

    hit = (
        (cr >= cr_min)
        & (cr <= cr_max)
        & (cb >= cb_min)
        & (cb <= cb_max)
    )

    raw[y0:y1, x0:x1] = hit.astype(np.uint8)
    return raw


# =============================================================================
# mask_cleaner.vhd - FUNCTIONAL FRAME RESULT
# =============================================================================


def clean_mask_hdl(raw_mask: np.ndarray, config: HWConfig) -> np.ndarray:
    cfg = normalized_config(config)
    k = cfg.kernel_size
    radius = k // 2
    threshold = (k * k) // 2 + 1

    kernel = np.ones((k, k), dtype=np.uint8)
    sums = cv2.filter2D(
        raw_mask,
        ddepth=cv2.CV_16U,
        kernel=kernel,
        anchor=(-1, -1),
        borderType=cv2.BORDER_CONSTANT,
    )

    cleaned = (sums >= threshold).astype(np.uint8)

    # The HDL exposes filtered_mask_valid only where a complete KxK window exists.
    if radius > 0:
        cleaned[:radius, :] = 0
        cleaned[-radius:, :] = 0
        cleaned[:, :radius] = 0
        cleaned[:, -radius:] = 0

    return cleaned


# =============================================================================
# object_measurement.vhd - FRAME RESULT
# =============================================================================


@dataclass
class Measurement:
    n: int = 0
    cx: int = 0
    cy: int = 0
    w: int = 0
    h: int = 0
    min_x: int = 0
    min_y: int = 0
    max_x: int = 0
    max_y: int = 0


def measure_clean_mask_hdl(cleaned_mask: np.ndarray) -> Measurement:
    ys, xs = np.nonzero(cleaned_mask)
    n = int(xs.size)

    if n == 0:
        return Measurement()

    sum_x = int(xs.astype(np.int64).sum())
    sum_y = int(ys.astype(np.int64).sum())

    cx = sum_x // n
    cy = sum_y // n

    min_x = int(xs.min())
    max_x = int(xs.max())
    min_y = int(ys.min())
    max_y = int(ys.max())

    return Measurement(
        n=n,
        cx=cx,
        cy=cy,
        w=max_x - min_x + 1,
        h=max_y - min_y + 1,
        min_x=min_x,
        min_y=min_y,
        max_x=max_x,
        max_y=max_y,
    )


# =============================================================================
# roi_tracker_update.vhd - FRAME-LEVEL STATE MODEL
# =============================================================================


class HDLTrackerModel:
    """Frame-level equivalent of the current roi_tracker_update.vhd."""

    def __init__(self, config: HWConfig = BASELINE):
        self.config = normalized_config(config)
        self.roi = (0, 0, FRAME_WIDTH, FRAME_HEIGHT)

        self.previous_center_x = 0
        self.previous_center_y = 0
        self.previous_smoothed_velocity_x = 0
        self.previous_smoothed_velocity_y = 0
        self.previous_center_valid = False

        self.min_accepted_pixels = self.config.count_floor
        self.loss_ramp_pixels = 0

        self.predicted_center_x = 0
        self.predicted_center_y = 0

        self.tracking_lost = False
        self.skip_first_measurement = True
        self.last_action = "DISABLED"

    def set_config(self, config: HWConfig):
        self.config = normalized_config(config)

        # Keep tracker state inside ranges representable by the configured values.
        self.previous_smoothed_velocity_x = clamp(
            self.previous_smoothed_velocity_x,
            -self.config.vel_max,
            self.config.vel_max,
        )
        self.previous_smoothed_velocity_y = clamp(
            self.previous_smoothed_velocity_y,
            -self.config.vel_max,
            self.config.vel_max,
        )
        self.loss_ramp_pixels = clamp(
            self.loss_ramp_pixels, 0, self.config.loss_max
        )

    def load_initial_roi(self, roi: tuple[int, int, int, int]):
        cfg = self.config
        x, y, w, h = map(int, roi)

        if w == 0 or h == 0:
            x, y, w, h = 0, 0, FRAME_WIDTH, FRAME_HEIGHT
        else:
            w = clamp(w, 1, FRAME_WIDTH)
            h = clamp(h, 1, FRAME_HEIGHT)
            x = clamp(x, 0, FRAME_WIDTH - w)
            y = clamp(y, 0, FRAME_HEIGHT - h)

        self.roi = (x, y, w, h)
        self.previous_center_x = 0
        self.previous_center_y = 0
        self.previous_smoothed_velocity_x = 0
        self.previous_smoothed_velocity_y = 0
        self.previous_center_valid = False
        self.min_accepted_pixels = cfg.count_floor
        self.loss_ramp_pixels = 0
        self.predicted_center_x = 0
        self.predicted_center_y = 0
        self.tracking_lost = False
        self.skip_first_measurement = True
        self.last_action = "SEED"

    def calculate_min_accepted_pixels(self, w: int, h: int) -> int:
        cfg = self.config
        threshold = (int(w) * int(h)) // (2 ** cfg.count_shift)
        return max(cfg.count_floor, threshold)

    def calculate_requested_loss_axis(
        self,
        previous_smoothed_velocity: int,
        loss_ramp_pixels: int,
    ) -> tuple[int, int]:
        cfg = self.config
        v = int(previous_smoothed_velocity)

        if v > 0:
            main = clamp(
                abs(v) // 4 + cfg.loss_base + loss_ramp_pixels,
                cfg.loss_min,
                cfg.loss_max,
            )
            secondary = clamp(
                main // 2,
                cfg.loss_min,
                cfg.loss_max,
            )
            return secondary, main

        if v < 0:
            main = clamp(
                abs(v) // 4 + cfg.loss_base + loss_ramp_pixels,
                cfg.loss_min,
                cfg.loss_max,
            )
            secondary = clamp(
                main // 2,
                cfg.loss_min,
                cfg.loss_max,
            )
            return main, secondary

        main = clamp(
            cfg.loss_base + loss_ramp_pixels,
            cfg.loss_min,
            cfg.loss_max,
        )
        return main, main

    def update(self, m: Measurement):
        cfg = self.config

        # First completed measurement after seed is discarded by the VHDL.
        if self.skip_first_measurement:
            self.skip_first_measurement = False
            self.last_action = "SKIP_FIRST"
            return

        # ACCEPT
        if m.n >= self.min_accepted_pixels:
            accept_cx = clamp(m.cx, 0, FRAME_WIDTH - 1)
            accept_cy = clamp(m.cy, 0, FRAME_HEIGHT - 1)
            accept_w = clamp(m.w, 1, FRAME_WIDTH)
            accept_h = clamp(m.h, 1, FRAME_HEIGHT)

            if self.previous_center_valid:
                raw_vx = clamp(
                    accept_cx - self.previous_center_x,
                    -cfg.vel_max,
                    cfg.vel_max,
                )
                raw_vy = clamp(
                    accept_cy - self.previous_center_y,
                    -cfg.vel_max,
                    cfg.vel_max,
                )
            else:
                raw_vx = 0
                raw_vy = 0

            smooth_vx = div_trunc_zero(
                raw_vx + self.previous_smoothed_velocity_x, 2
            )
            smooth_vy = div_trunc_zero(
                raw_vy + self.previous_smoothed_velocity_y, 2
            )

            predicted_x = accept_cx + smooth_vx
            predicted_y = accept_cy + smooth_vy

            padding_x = clamp(
                cfg.pad_base + abs(smooth_vx) // 2,
                0,
                cfg.pad_max,
            )
            padding_y = clamp(
                cfg.pad_base + abs(smooth_vy) // 2,
                0,
                cfg.pad_max,
            )

            roi_w = clamp(
                accept_w + 2 * padding_x,
                1,
                cfg.max_track_roi_w,
            )
            roi_h = clamp(
                accept_h + 2 * padding_y,
                1,
                cfg.max_track_roi_h,
            )

            roi_x = clamp(
                predicted_x - roi_w // 2,
                0,
                FRAME_WIDTH - roi_w,
            )
            roi_y = clamp(
                predicted_y - roi_h // 2,
                0,
                FRAME_HEIGHT - roi_h,
            )

            self.roi = (roi_x, roi_y, roi_w, roi_h)
            self.predicted_center_x = predicted_x
            self.predicted_center_y = predicted_y
            self.previous_center_x = accept_cx
            self.previous_center_y = accept_cy
            self.previous_smoothed_velocity_x = smooth_vx
            self.previous_smoothed_velocity_y = smooth_vy
            self.previous_center_valid = True
            self.min_accepted_pixels = self.calculate_min_accepted_pixels(
                accept_w, accept_h
            )
            self.loss_ramp_pixels = 0
            self.tracking_lost = False
            self.last_action = "ACCEPT"
            return

        # LOSS
        self.tracking_lost = True
        self.last_action = "LOSS"

        x, y, roi_w, roi_h = self.roi

        can_expand = (
            self.previous_center_valid
            and (
                roi_w < cfg.max_loss_w
                or roi_h < cfg.max_loss_h
            )
        )

        if not can_expand:
            self.last_action = "LOSS_MAX"
            return

        req_left, req_right = self.calculate_requested_loss_axis(
            self.previous_smoothed_velocity_x,
            self.loss_ramp_pixels,
        )
        req_up, req_down = self.calculate_requested_loss_axis(
            self.previous_smoothed_velocity_y,
            self.loss_ramp_pixels,
        )

        rem_w = cfg.max_loss_w - roi_w if roi_w < cfg.max_loss_w else 0
        rem_h = cfg.max_loss_h - roi_h if roi_h < cfg.max_loss_h else 0

        expanded_w = roi_w + clamp(
            req_left + req_right, 0, rem_w
        )

        vx = self.previous_smoothed_velocity_x
        if vx > 0:
            grow_left = clamp(
                rem_w - req_right, 0, req_left
            )
        elif vx < 0:
            grow_left = clamp(req_left, 0, rem_w)
        else:
            grow_left = clamp(req_left, 0, rem_w // 2)

        expanded_h = roi_h + clamp(
            req_up + req_down, 0, rem_h
        )

        vy = self.previous_smoothed_velocity_y
        if vy > 0:
            grow_up = clamp(rem_h - req_down, 0, req_up)
        elif vy < 0:
            grow_up = clamp(req_up, 0, rem_h)
        else:
            grow_up = clamp(req_up, 0, rem_h // 2)

        new_x = clamp(
            x - grow_left,
            0,
            FRAME_WIDTH - expanded_w,
        )
        new_y = clamp(
            y - grow_up,
            0,
            FRAME_HEIGHT - expanded_h,
        )

        self.roi = (new_x, new_y, expanded_w, expanded_h)
        self.loss_ramp_pixels = clamp(
            self.loss_ramp_pixels + cfg.loss_ramp,
            0,
            cfg.loss_max,
        )
        self.last_action = "LOSS_EXPAND"


# =============================================================================
# COMPLETE PL PIPELINE MODEL
# =============================================================================


class PLModel:
    def __init__(self, config: HWConfig = BASELINE):
        self.config = normalized_config(config)
        self.tracker = HDLTrackerModel(self.config)
        self.thresholds = (0, 255, 0, 255)
        self.last_measurement = Measurement()
        self.frame_index = 0

    def set_config(self, config: HWConfig):
        self.config = normalized_config(config)
        self.tracker.set_config(self.config)

    def seed(
        self,
        roi: tuple[int, int, int, int],
        thresholds: tuple[int, int, int, int],
    ):
        self.thresholds = tuple(map(int, thresholds))
        self.tracker.load_initial_roi(roi)
        self.last_measurement = Measurement()
        self.frame_index = 0

    def process_frame(self, frame_bgr: np.ndarray):
        if (
            frame_bgr.shape[1] != FRAME_WIDTH
            or frame_bgr.shape[0] != FRAME_HEIGHT
        ):
            frame_bgr = cv2.resize(
                frame_bgr,
                (FRAME_WIDTH, FRAME_HEIGHT),
                interpolation=cv2.INTER_LINEAR,
            )

        # roi_mask_generator captures the tracking ROI at frame start.
        frame_roi = tuple(self.tracker.roi)

        # object_display also captures its tracking geometry at display-frame start.
        display_measured = (
            self.last_measurement.cx,
            self.last_measurement.cy,
        )
        display_predicted = (
            self.tracker.predicted_center_x,
            self.tracker.predicted_center_y,
        )
        display_lost = bool(self.tracker.tracking_lost)

        raw = raw_roi_mask_hdl(
            frame_bgr,
            frame_roi,
            self.thresholds,
        )
        clean = clean_mask_hdl(raw, self.config)
        measurement = measure_clean_mask_hdl(clean)

        self.tracker.update(measurement)
        self.last_measurement = measurement
        self.frame_index += 1

        return {
            "frame": frame_bgr,
            "frame_roi": frame_roi,
            "raw_mask": raw,
            "clean_mask": clean,
            "measurement": measurement,
            "display_measured": display_measured,
            "display_predicted": display_predicted,
            "display_lost": display_lost,
        }


# =============================================================================
# object_display.vhd - FRAME-LEVEL VISUALIZATION
# =============================================================================


def render_hdl_display(result, config: HWConfig, tracking_enable=True):
    cfg = normalized_config(config)
    clean = result["clean_mask"]
    img = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)

    # x"FFFFFF" - cleaned object mask
    img[clean.astype(bool)] = (255, 255, 255)

    if not tracking_enable:
        return img

    rx, ry, rw, rh = result["frame_roi"]
    right = rx + rw - 1
    bottom = ry + rh - 1

    # x"00FF00" - ROI border
    t = cfg.roi_border_thickness
    if rw > 0 and rh > 0:
        for d in range(t):
            yt = ry + d
            yb = bottom - d
            xl = rx + d
            xr = right - d

            if 0 <= yt < FRAME_HEIGHT:
                img[yt, max(0, rx):min(FRAME_WIDTH, right + 1)] = (0, 255, 0)
            if 0 <= yb < FRAME_HEIGHT:
                img[yb, max(0, rx):min(FRAME_WIDTH, right + 1)] = (0, 255, 0)
            if 0 <= xl < FRAME_WIDTH:
                img[max(0, ry):min(FRAME_HEIGHT, bottom + 1), xl] = (0, 255, 0)
            if 0 <= xr < FRAME_WIDTH:
                img[max(0, ry):min(FRAME_HEIGHT, bottom + 1), xr] = (0, 255, 0)

    # x"FF0000" in VHDL appears as red in the intended RGB display.
    # OpenCV frame storage is BGR, so red is (0,0,255).
    mx, my = result["display_measured"]
    r = cfg.center_radius
    if 0 <= mx < FRAME_WIDTH:
        img[max(0, my-r):min(FRAME_HEIGHT, my+r+1), mx] = (0, 0, 255)
    if 0 <= my < FRAME_HEIGHT:
        img[my, max(0, mx-r):min(FRAME_WIDTH, mx+r+1)] = (0, 0, 255)

    # x"0000FF" - predicted centre blue X, hidden while tracking_lost=1.
    if not result["display_lost"]:
        px, py = result["display_predicted"]
        for d in range(-r, r + 1):
            x1, y1 = px + d, py + d
            x2, y2 = px + d, py - d
            if 0 <= x1 < FRAME_WIDTH and 0 <= y1 < FRAME_HEIGHT:
                img[y1, x1] = (255, 0, 0)
            if 0 <= x2 < FRAME_WIDTH and 0 <= y2 < FRAME_HEIGHT:
                img[y2, x2] = (255, 0, 0)

    return img



# =============================================================================
# DESKTOP SIMULATION
# =============================================================================

HDMI_WINDOW = "REAL SYSTEM - exact PL HDMI"
DEBUG_WINDOW = "SIM DEBUG - camera"
SELECT_WINDOW = "SELECT OBJECT - ENTER/SPACE confirm, C cancel"



def create_windows():
    # Simulated physical HDMI output from object_display.vhd.
    cv2.namedWindow(HDMI_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(HDMI_WINDOW, FRAME_WIDTH, FRAME_HEIGHT)

    # Live camera/debug image.
    cv2.namedWindow(DEBUG_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(DEBUG_WINDOW, FRAME_WIDTH, FRAME_HEIGHT)

    black = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    cv2.imshow(HDMI_WINDOW, black)
    cv2.imshow(DEBUG_WINDOW, black)
    cv2.waitKey(1)


def resize_frame(frame):
    if frame.shape[1] == FRAME_WIDTH and frame.shape[0] == FRAME_HEIGHT:
        return frame

    return cv2.resize(
        frame,
        (FRAME_WIDTH, FRAME_HEIGHT),
        interpolation=cv2.INTER_LINEAR,
    )


def select_object_from_frame(frame, model, config):
    """
    Desktop stand-in for the final Jupyter freeze/select/start step.

    S freezes the current unmodified camera frame.
    The ROI and Cr/Cb thresholds are then created with the same initial
    software algorithm used by run_object_tracker_pynq.py.
    """
    roi = cv2.selectROI(
        SELECT_WINDOW,
        frame,
        showCrosshair=True,
        fromCenter=False,
    )

    try:
        cv2.destroyWindow(SELECT_WINDOW)
    except cv2.error:
        pass

    x, y, w, h = [int(v) for v in roi]

    if w <= 1 or h <= 1:
        print("[SELECT] cancelled", flush=True)
        return False, None, None

    roi = clamp_bbox((x, y, w, h))
    x, y, w, h = roi

    crop = frame[y:y + h, x:x + w]
    if crop.size == 0:
        print("[SELECT] invalid crop", flush=True)
        return False, None, None

    thresholds = learn_pl_thresholds(crop)

    model.set_config(config)
    model.seed(roi, thresholds)

    print(
        f"[SELECT] ROI={roi} "
        f"Cr[{thresholds[0]},{thresholds[1]}] "
        f"Cb[{thresholds[2]},{thresholds[3]}]",
        flush=True,
    )

    return True, roi, thresholds


def put_line(img, text, y, scale=0.48):
    cv2.putText(
        img,
        str(text),
        (10, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        img,
        str(text),
        (10, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def draw_debug(frame, model, result, tracking, config, initial_roi, thresholds):
    """
    Diagnostic PC view.
    This does not feed back into tracking; it only makes tuning easier.
    """
    img = frame.copy()

    if tracking and result is not None:
        rx, ry, rw, rh = result["frame_roi"]
        cv2.rectangle(
            img,
            (rx, ry),
            (rx + rw - 1, ry + rh - 1),
            (0, 255, 0),
            2,
        )

        m = result["measurement"]
        if m.n > 0:
            cv2.rectangle(
                img,
                (m.min_x, m.min_y),
                (m.max_x, m.max_y),
                (0, 220, 220),
                1,
            )
            cv2.circle(
                img,
                (m.cx, m.cy),
                4,
                (0, 0, 255),
                -1,
            )

        px = model.tracker.predicted_center_x
        py = model.tracker.predicted_center_y

        if 0 <= px < FRAME_WIDTH and 0 <= py < FRAME_HEIGHT:
            cv2.drawMarker(
                img,
                (px, py),
                (255, 0, 0),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=12,
                thickness=2,
            )

    if not tracking:
        lines = [
            "LIVE CAMERA - press S when you want to select the object",
            "S select | Q/ESC quit",
        ]
    else:
        m = model.last_measurement
        lines = [
            f"TRACKING | action={model.tracker.last_action} "
            f"lost={int(model.tracker.tracking_lost)}",
            f"ROI(next)={model.tracker.roi} "
            f"pred=({model.tracker.predicted_center_x},"
            f"{model.tracker.predicted_center_y})",
            f"N={m.n} minN={model.tracker.min_accepted_pixels} "
            f"lossRamp={model.tracker.loss_ramp_pixels}",
        ]

        if thresholds is not None:
            lines.append(
                f"Cr[{thresholds[0]},{thresholds[1]}] "
                f"Cb[{thresholds[2]},{thresholds[3]}]"
            )

        lines.extend([
            f"K={config.kernel_size} "
            f"PAD={config.pad_base}/{config.pad_max} "
            f"TRACK_MAX={config.max_track_roi_w}x{config.max_track_roi_h}",
            f"COUNT={config.count_floor},shift={config.count_shift} "
            f"LOSS={config.loss_base}/{config.loss_ramp}/"
            f"{config.loss_min}/{config.loss_max}",
            f"LOSS_MAX={config.max_loss_w}x{config.max_loss_h} "
            f"VEL_MAX={config.vel_max}",
            "S new object | R reset | Q quit",
        ])

    y = 22
    for line in lines:
        put_line(img, line, y)
        y += 20

    return img


def main():
    parser = argparse.ArgumentParser(
        description="Current PYNQ-Z2 tracker - exact frame-level PC tuning simulation"
    )
    parser.add_argument(
        "--video",
        help="optional video file instead of webcam",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="camera index (default 0)",
    )
    args = parser.parse_args()

    source = args.video if args.video else args.camera
    cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open source: {source!r}")

    if not args.video:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    create_windows()

    config = normalized_config(BASELINE)
    model = PLModel(config)

    tracking = False
    result = None
    initial_roi = None
    thresholds = None

    print("", flush=True)
    print("============================================================", flush=True)
    print(" CURRENT PYNQ-Z2 HW TRACKER TUNING SIMULATOR", flush=True)
    print("============================================================", flush=True)
    print("S       select / reselect object", flush=True)
    print("R       reset tracking", flush=True)
    print("Q/ESC   quit", flush=True)
    print("", flush=True)
    print(
        "The debug window is the live camera. "
        "The HDMI window is the simulated VHDL object_display output.",
        flush=True,
    )
    print(
        "To test different hardware values, edit HWConfig at the top of the file and run again.",
        flush=True,
    )
    print("", flush=True)

    while True:
        ok, frame = cap.read()

        if not ok or frame is None:
            if args.video:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = cap.read()

            if not ok or frame is None:
                break

        frame = resize_frame(frame)


        if tracking:
            result = model.process_frame(frame)
            hdl_display = render_hdl_display(
                result,
                config,
                tracking_enable=True,
            )
        else:
            result = None
            hdl_display = np.zeros(
                (FRAME_HEIGHT, FRAME_WIDTH, 3),
                dtype=np.uint8,
            )
            put_line(
                hdl_display,
                "Waiting for object selection - press S in either window",
                35,
                0.60,
            )

        debug = draw_debug(
            frame,
            model,
            result,
            tracking,
            config,
            initial_roi,
            thresholds,
        )

        cv2.imshow(HDMI_WINDOW, hdl_display)
        cv2.imshow(DEBUG_WINDOW, debug)

        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), ord("Q"), 27):
            break

        if key in (ord("s"), ord("S")):
            # Use the untouched current camera frame for initial object selection.
            selected, roi, learned_thresholds = select_object_from_frame(
                frame.copy(),
                model,
                config,
            )

            if selected:
                tracking = True
                result = None
                initial_roi = roi
                thresholds = learned_thresholds

        elif key in (ord("r"), ord("R")):
            tracking = False
            result = None
            initial_roi = None
            thresholds = None
            model = PLModel(config)

            print(
                "[RESET] tracking disabled; press S for a new object",
                flush=True,
            )


    cap.release()
    cv2.destroyAllWindows()
    print("[CLOSE] camera released", flush=True)


# =============================================================================
# SELF CHECK
# =============================================================================

def _self_check():
    cfg = normalized_config(BASELINE)

    assert cfg.kernel_size == 5
    assert cfg.pad_base == 8
    assert cfg.pad_max == 40
    assert cfg.max_track_roi_w == 200
    assert cfg.max_track_roi_h == 200
    assert cfg.count_floor == 150
    assert cfg.count_shift == 3
    assert cfg.loss_base == 10
    assert cfg.loss_ramp == 5
    assert cfg.loss_min == 5
    assert cfg.loss_max == 40
    assert cfg.max_loss_w == 240
    assert cfg.max_loss_h == 240
    assert cfg.vel_max == 120

    black = np.zeros((20, 20, 3), dtype=np.uint8)
    cr, cb = rgb_to_crcb_hdl(black)
    assert np.all(cr == 128)
    assert np.all(cb == 128)

    thresholds = learn_pl_thresholds(black)
    assert thresholds == (122, 134, 122, 134), thresholds


if __name__ == "__main__":
    _self_check()
    main()
