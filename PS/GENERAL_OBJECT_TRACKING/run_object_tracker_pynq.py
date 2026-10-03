"""
run_object_tracker_pynq.py

PYNQ-Z2 runtime for hardware object tracking.

System flow:
    1) Shows the live camera preview.
    2) Lets the user freeze a frame and select ONE initial ROI.
    3) Learns ONLY the initial Cr/Cb limits from that selected ROI.
    4) Writes the initial ROI + thresholds to the PL once.
    5) Streams clean camera frames through HDMI/VDMA to the PL.
    6) Reads and displays the coherent AXI-Lite tracking result from the PL.

After START TRACKING, the PL performs the continuous object tracking.

Required files next to this script on the PYNQ:
    fpga.py
    design_1.bit
    design_1.hwh

Typical Jupyter use:
    %matplotlib notebook
    from run_object_tracker_pynq import PYNQObjectTracker

    tracker = PYNQObjectTracker()
    tracker.run_interactive()
"""

from __future__ import annotations

import os
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# =============================================================================
# BOARD / VIDEO CONFIGURATION
# =============================================================================

DESIGN_NAME = "design_1"
BITSTREAM_FILENAME = f"{DESIGN_NAME}.bit"
BITSTREAM_PATH = os.path.join(HERE, BITSTREAM_FILENAME)

FPGA_BASE_ADDR = 0x43C30000
CAMERA_INDEX = 0

RUNTIME_VERSION = "2026-10-03-pl-tracking-v5"

EXPECTED_FRAME_WIDTH = 640
EXPECTED_FRAME_HEIGHT = 480
VIDEO_BPP = 24

CAMERA_BUFFER_SIZE = 1


# =============================================================================
# OBJECT COLOUR LEARNING
# =============================================================================

# Fraction of the selected ROI used for colour learning (central crop only).
INNER_FRAC = 0.72

# Percentile bounds used to reject colour outliers in the selected ROI.
COLOR_PERCENTILE_LOW = 5.0
COLOR_PERCENTILE_HIGH = 95.0

# Expansion added to both sides of the learned Cr/Cb ranges.
COLOR_MARGIN = 6

# Minimum number of pixels required for meaningful colour learning.
MIN_COLOR_SAMPLES = 10


# =============================================================================
# PL / PS SYNCHRONIZATION
# =============================================================================

SNAPSHOT_STALL_TIMEOUT_SEC = 2.0
SNAPSHOT_POLL_SEC = 0.002
MAX_SNAPSHOT_STALLS = 3

# Read/print/update the complete coherent PL status at this interval.
# Every video frame still synchronizes only through REG_SNAPSHOT_COUNTER.
PL_STATUS_INTERVAL_SEC = 1.0


# =============================================================================
# CAMERA / RUNTIME SAFETY
# =============================================================================

MAX_CAMERA_FAILS = 30
CAMERA_WATCHDOG_SEC = 5.0
CAMERA_RETRY_SLEEP_SEC = 0.01
RUN_THREAD_JOIN_TIMEOUT_SEC = 3.0
PREVIEW_THREAD_JOIN_TIMEOUT_SEC = 1.5


# =============================================================================
# NOTEBOOK PREVIEW
# =============================================================================

PREVIEW_FPS = 15.0
PREVIEW_JPEG_QUALITY = 80


# =============================================================================
# SHUTDOWN
# =============================================================================

HDMI_BLACK_HOLD_SEC = 0.12


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

    if not (0.0 < INNER_FRAC <= 1.0):
        raise ValueError("INNER_FRAC must satisfy 0 < INNER_FRAC <= 1")
    if not (0.0 <= COLOR_PERCENTILE_LOW < COLOR_PERCENTILE_HIGH <= 100.0):
        raise ValueError(
            "COLOR percentiles must satisfy "
            "0 <= LOW < HIGH <= 100"
        )
    if not (0 <= COLOR_MARGIN <= 255):
        raise ValueError("COLOR_MARGIN must be in the range 0..255")
    if MIN_COLOR_SAMPLES < 1:
        raise ValueError("MIN_COLOR_SAMPLES must be >= 1")

    if SNAPSHOT_STALL_TIMEOUT_SEC <= 0:
        raise ValueError("SNAPSHOT_STALL_TIMEOUT_SEC must be > 0")
    if SNAPSHOT_POLL_SEC <= 0:
        raise ValueError("SNAPSHOT_POLL_SEC must be > 0")
    if MAX_SNAPSHOT_STALLS < 1:
        raise ValueError("MAX_SNAPSHOT_STALLS must be >= 1")
    if PL_STATUS_INTERVAL_SEC <= 0:
        raise ValueError("PL_STATUS_INTERVAL_SEC must be > 0")

    if MAX_CAMERA_FAILS < 1:
        raise ValueError("MAX_CAMERA_FAILS must be >= 1")
    if CAMERA_WATCHDOG_SEC <= 0:
        raise ValueError("CAMERA_WATCHDOG_SEC must be > 0")
    if CAMERA_RETRY_SLEEP_SEC < 0:
        raise ValueError("CAMERA_RETRY_SLEEP_SEC must be >= 0")
    if RUN_THREAD_JOIN_TIMEOUT_SEC <= 0:
        raise ValueError("RUN_THREAD_JOIN_TIMEOUT_SEC must be > 0")
    if PREVIEW_THREAD_JOIN_TIMEOUT_SEC <= 0:
        raise ValueError("PREVIEW_THREAD_JOIN_TIMEOUT_SEC must be > 0")

    if PREVIEW_FPS <= 0:
        raise ValueError("PREVIEW_FPS must be > 0")
    if not (0 <= PREVIEW_JPEG_QUALITY <= 100):
        raise ValueError("PREVIEW_JPEG_QUALITY must be in the range 0..100")

    if HDMI_BLACK_HOLD_SEC < 0:
        raise ValueError("HDMI_BLACK_HOLD_SEC must be >= 0")


# =============================================================================
# INITIAL COLOUR LEARNING + EXACT HDL Cr/Cb CONVERSION
# =============================================================================

def inner_crop(img, frac=INNER_FRAC):
    h, w = img.shape[:2]
    nw = max(2, int(round(w * frac)))
    nh = max(2, int(round(h * frac)))
    x1 = max(0, (w - nw) // 2)
    y1 = max(0, (h - nh) // 2)
    x2 = min(w, x1 + nw)
    y2 = min(h, y1 + nh)
    return img[y1:y2, x1:x2]


def rgb_to_crcb_hdl(frame_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Exact integer arithmetic used by rgb_to_ycrcb.vhd."""
    b = frame_bgr[:, :, 0].astype(np.int64)
    g = frame_bgr[:, :, 1].astype(np.int64)
    r = frame_bgr[:, :, 2].astype(np.int64)

    cb = ((-43 * r - 85 * g + 128 * b) >> 8) + 128
    cr = ((128 * r - 107 * g - 21 * b) >> 8) + 128

    return cr.astype(np.uint8), cb.astype(np.uint8)


def learn_pl_thresholds(bgr):
    """Learn the four initial Cr/Cb threshold bytes written to the PL."""
    cr, cb = rgb_to_crcb_hdl(inner_crop(bgr))
    crv = cr.reshape(-1)
    cbv = cb.reshape(-1)

    if len(crv) < MIN_COLOR_SAMPLES:
        return 0, 255, 0, 255

    cr_lo, cr_hi = np.percentile(
        crv,
        [COLOR_PERCENTILE_LOW, COLOR_PERCENTILE_HIGH],
    )
    cb_lo, cb_hi = np.percentile(
        cbv,
        [COLOR_PERCENTILE_LOW, COLOR_PERCENTILE_HIGH],
    )

    return (
        int(max(0, np.floor(cr_lo) - COLOR_MARGIN)),
        int(min(255, np.ceil(cr_hi) + COLOR_MARGIN)),
        int(max(0, np.floor(cb_lo) - COLOR_MARGIN)),
        int(min(255, np.ceil(cb_hi) + COLOR_MARGIN)),
    )


# =============================================================================
# NOTEBOOK UI
# =============================================================================

class NotebookPLOnlyUI:
    """Live preview, initial ROI selection, and live PL tracking-data display."""

    def __init__(
        self,
        cap,
        on_confirm=None,
        on_quit=None,
        preview_fps=PREVIEW_FPS,
    ):
        self.cap = cap
        self._on_confirm = on_confirm
        self._on_quit = on_quit
        self.preview_fps = max(1.0, float(preview_fps))

        self._state = "LIVE"
        self._latest_frame = None
        self._frozen_frame = None
        self.roi = None

        self._frame_lock = threading.Lock()
        self._preview_stop = threading.Event()
        self._preview_thread = None
        self._closed = False

        self._image_widget = None
        self._status_widget = None
        self._pl_data_widget = None
        self._config_widget = None
        self._action_button = None
        self._close_button = None
        self._widget_box = None

        self._fig = None
        self._selector = None

    @staticmethod
    def _jpeg_bytes(frame):
        ok, buf = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), PREVIEW_JPEG_QUALITY],
        )
        if not ok:
            raise RuntimeError("could not encode notebook preview frame")
        return buf.tobytes()

    def _read_camera_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None

        if (
            frame.shape[1] != EXPECTED_FRAME_WIDTH
            or frame.shape[0] != EXPECTED_FRAME_HEIGHT
        ):
            frame = cv2.resize(
                frame,
                (EXPECTED_FRAME_WIDTH, EXPECTED_FRAME_HEIGHT),
                interpolation=cv2.INTER_LINEAR,
            )
        return frame

    def _set_status(self, text):
        if self._status_widget is not None:
            try:
                self._status_widget.value = f"<b>{text}</b>"
            except Exception:
                pass

    def set_status(self, text):
        self._set_status(text)

    def set_initial_config(self, roi, thresholds):
        if self._config_widget is None:
            return

        cr_min, cr_max, cb_min, cb_max = thresholds
        try:
            self._config_widget.value = (
                "<pre style='margin:0'>"
                "INITIAL CONFIG | "
                f"ROI=({roi[0]},{roi[1]},{roi[2]},{roi[3]}) | "
                f"Cr=[{cr_min},{cr_max}] | "
                f"Cb=[{cb_min},{cb_max}] | "
                "</pre>"
            )
        except Exception:
            pass

    def set_pl_data(self, td):
        """Display the latest tracking data reported by the PL."""
        if self._pl_data_widget is None:
            return

        try:
            self._pl_data_widget.value = (
                "<pre style='margin:0'>"
                f"PL DATA | snapshot={td.snapshot} | "
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

    def _set_action(self, description, style=""):
        if self._action_button is not None:
            try:
                self._action_button.description = description
                self._action_button.button_style = style
                self._action_button.disabled = False
            except Exception:
                pass

    # -------------------------------------------------------------------------
    # Live preview
    # -------------------------------------------------------------------------

    def _preview_loop(self):
        period = 1.0 / self.preview_fps

        while not self._preview_stop.is_set() and not self._closed:
            t0 = time.monotonic()
            frame = self._read_camera_frame()

            if frame is not None:
                with self._frame_lock:
                    self._latest_frame = frame.copy()

                if self._image_widget is not None:
                    try:
                        self._image_widget.value = self._jpeg_bytes(frame)
                    except Exception:
                        pass

            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    def _start_preview(self):
        self._stop_preview()
        self._preview_stop.clear()
        self._preview_thread = threading.Thread(
            target=self._preview_loop,
            daemon=True,
            name="pynq-pl-only-live-preview",
        )
        self._preview_thread.start()

    def _stop_preview(self):
        self._preview_stop.set()
        thread = self._preview_thread
        if thread is not None and thread.is_alive():
            if threading.current_thread() is not thread:
                thread.join(timeout=PREVIEW_THREAD_JOIN_TIMEOUT_SEC)
        self._preview_thread = None

    # -------------------------------------------------------------------------
    # UI states
    # -------------------------------------------------------------------------

    def enter_live_preview(self):
        if self._closed:
            return

        self._close_roi_figure()
        self._state = "LIVE"
        self.roi = None
        self._frozen_frame = None

        frame = self._read_camera_frame()
        if frame is not None:
            with self._frame_lock:
                self._latest_frame = frame.copy()

            if self._image_widget is not None:
                try:
                    self._image_widget.value = self._jpeg_bytes(frame)
                except Exception:
                    pass

        self._set_status(
            "LIVE CAMERA — CAPTURE FRAME -> select initial ROI and START PL TRACKING"
        )
        self._set_action("CAPTURE FRAME", "info")
        self._start_preview()

        print(
            "[LIVE] Capture one frame and select the initial object ROI.",
            flush=True,
        )

    def _capture_live_frame(self):
        if self._closed or self._state != "LIVE":
            return

        with self._frame_lock:
            frame = (
                None
                if self._latest_frame is None
                else self._latest_frame.copy()
            )

        if frame is None:
            self._set_status("Waiting for the first camera frame...")
            return

        self._stop_preview()
        self._state = "ROI"
        self._frozen_frame = frame
        self.roi = None

        if self._image_widget is not None:
            try:
                self._image_widget.value = self._jpeg_bytes(frame)
            except Exception:
                pass

        self._set_status(
            "FRAME FROZEN — draw a tight rectangle around the object; "
            "then press START PL TRACKING"
        )
        self._set_action("CONFIRM ROI / START", "success")
        self._show_roi_figure(frame)

        print(
            "[SELECT] draw one tight initial ROI for hardware tracking",
            flush=True,
        )

    def _show_roi_figure(self, frame):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import RectangleSelector

        self._close_roi_figure()

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._fig, ax = plt.subplots(figsize=(9, 6))
        ax.imshow(rgb)
        ax.set_title("Draw the INITIAL ROI for the PL")
        ax.set_xlim(0, rgb.shape[1])
        ax.set_ylim(rgb.shape[0], 0)

        def onselect(eclick, erelease):
            if self._closed or self._state != "ROI":
                return

            x1, y1 = eclick.xdata, eclick.ydata
            x2, y2 = erelease.xdata, erelease.ydata
            if None in (x1, y1, x2, y2):
                return

            xa, xb = sorted((int(round(x1)), int(round(x2))))
            ya, yb = sorted((int(round(y1)), int(round(y2))))

            w = xb - xa
            h = yb - ya

            if w <= 1 or h <= 1:
                return

            self.roi = (xa, ya, w, h)
            ax.set_title(
                f"Initial ROI={self.roi} — press START PL TRACKING"
            )
            self._set_status(
                f"Initial ROI={self.roi} — press START PL TRACKING"
            )
            self._set_action("START PL TRACKING", "success")

            try:
                self._fig.canvas.draw_idle()
            except Exception:
                pass

            print(f"[SELECT] initial_roi={self.roi}", flush=True)

        self._selector = RectangleSelector(
            ax,
            onselect,
            useblit=True,
            button=[1],
            minspanx=2,
            minspany=2,
            spancoords="pixels",
            interactive=True,
        )
        plt.show()

    def _confirm_roi(self):
        if self._closed or self._state != "ROI":
            return

        if self.roi is None:
            self._set_status(
                "Draw a rectangle first, then press START PL TRACKING"
            )
            return

        frame = self._frozen_frame.copy()
        roi = tuple(self.roi)

        self._set_status(
            "STARTING — calculating initial Cr/Cb and seeding the PL..."
        )
        self._set_action("STARTING...", "warning")

        try:
            ok = True
            if self._on_confirm is not None:
                ok = bool(self._on_confirm(frame, roi))
        except Exception as exc:
            import traceback
            print(f"[START-ERROR] {type(exc).__name__}: {exc}", flush=True)
            traceback.print_exc()
            ok = False

        if not ok:
            if not self._closed:
                self._state = "ROI"
                self._set_status(
                    "START FAILED — ROI remains selected. "
                    "Check the printed error and retry."
                )
                self._set_action("START PL TRACKING", "success")
            return

        self._close_roi_figure()
        self._state = "TRACKING"
        self._set_status(
            "PL TRACKING RUNNING — live hardware tracking data shown below"
        )
        self._set_action("PL TRACKING ACTIVE", "")

        if self._action_button is not None:
            try:
                self._action_button.disabled = True
            except Exception:
                pass

        print(
            "[START] PL configured and hardware tracking started.",
            flush=True,
        )

    def _request_quit(self):
        if self._closed:
            return

        print("[BUTTON] STOP & CLOSE -> safe shutdown", flush=True)
        if self._on_quit is not None:
            self._on_quit()

    def _action_clicked(self, _button=None):
        if self._state == "LIVE":
            self._capture_live_frame()
        elif self._state == "ROI":
            self._confirm_roi()
        elif self._state == "TRACKING":
            self._set_status(
                "PL TRACKING: press STOP & CLOSE to stop"
            )

    def show(self):
        try:
            import ipywidgets as widgets
            from IPython.display import display
        except Exception as exc:
            raise RuntimeError(
                "This notebook UI requires ipywidgets on the PYNQ image"
            ) from exc

        self._status_widget = widgets.HTML(
            value="<b>Starting camera...</b>"
        )
        self._config_widget = widgets.HTML(
            value=(
                "<pre style='margin:0'>"
                "INITIAL CONFIG | waiting for selection..."
                "</pre>"
            )
        )
        self._pl_data_widget = widgets.HTML(
            value=(
                "<pre style='margin:0'>"
                "PL DATA | waiting for tracking..."
                "</pre>"
            )
        )
        self._image_widget = widgets.Image(
            format="jpeg",
            width=EXPECTED_FRAME_WIDTH,
            height=EXPECTED_FRAME_HEIGHT,
        )
        self._action_button = widgets.Button(
            description="CAPTURE FRAME",
            button_style="info",
            icon="camera",
        )
        self._close_button = widgets.Button(
            description="STOP & CLOSE",
            button_style="danger",
            icon="stop",
        )

        self._action_button.on_click(self._action_clicked)
        self._close_button.on_click(lambda _b: self._request_quit())

        help_html = widgets.HTML(
            value=(
                "<span><b>Hardware tracking:</b> "
                "select the initial ROI, start tracking, and view live PL data."
                "</span>"
            )
        )

        self._widget_box = widgets.VBox([
            self._status_widget,
            self._config_widget,
            self._pl_data_widget,
            self._image_widget,
            widgets.HBox([self._action_button, self._close_button]),
            help_html,
        ])

        display(self._widget_box)
        self.enter_live_preview()
        return self

    def _close_roi_figure(self):
        if self._selector is not None:
            try:
                self._selector.set_active(False)
            except Exception:
                pass

            try:
                self._selector.disconnect_events()
            except Exception:
                pass

            self._selector = None

        if self._fig is not None:
            try:
                import matplotlib.pyplot as plt
                plt.close(self._fig)
            except Exception:
                pass

            self._fig = None

    def close(self, notify=False):
        if self._closed:
            return

        self._closed = True
        self._stop_preview()
        self._close_roi_figure()

        if self._widget_box is not None:
            try:
                self._widget_box.close()
            except Exception:
                pass
            self._widget_box = None

        if notify and self._on_quit is not None:
            self._on_quit()


# =============================================================================
# REAL BOARD PL-ONLY RUNTIME
# =============================================================================

class PYNQObjectTracker:
    """Real PYNQ-Z2 runtime for hardware object tracking."""

    def __init__(
        self,
        bitstream_path: str = BITSTREAM_PATH,
        base_address: int = FPGA_BASE_ADDR,
        camera_index: int = CAMERA_INDEX,
    ) -> None:
        validate_configuration()

        from fpga import FPGA

        if not os.path.exists(bitstream_path):
            raise FileNotFoundError(
                f"bitstream not found: {bitstream_path}"
            )

        hwh_path = os.path.splitext(bitstream_path)[0] + ".hwh"
        if not os.path.exists(hwh_path):
            raise FileNotFoundError(
                f"PYNQ needs the matching .hwh metadata next to the .bit: {hwh_path}"
            )

        self.fpga = FPGA(bitstream_path, base_address)

        fw, fh = self.fpga.read_frame_size()
        print(f"[PL] FRAME_SIZE={fw}x{fh}", flush=True)

        if (fw, fh) != (
            EXPECTED_FRAME_WIDTH,
            EXPECTED_FRAME_HEIGHT,
        ):
            raise RuntimeError(
                f"PL reports {fw}x{fh} but this runtime expects "
                f"{EXPECTED_FRAME_WIDTH}x{EXPECTED_FRAME_HEIGHT}"
            )

        self.hdmi_out = self._setup_hdmi_out()
        self.cap = self._open_camera(camera_index)

        self._run_thread = None
        self._run_stop = threading.Event()
        self._acquisition_ui = None

        self._closed = False
        self._hdmi_running = False
        self.tracking = False

        self._initial_roi = None
        self._pl_thresholds = None
        self._last_tracking_data = None

        # Keep hardware tracking disabled until the user explicitly starts.
        self.fpga.set_tracking_enable(False)

    # -------------------------------------------------------------------------
    # Hardware/video setup
    # -------------------------------------------------------------------------

    def _setup_hdmi_out(self):
        from pynq.lib.video import VideoMode

        overlay = self.fpga.overlay

        try:
            hdmi_out = overlay.video.hdmi_out
        except AttributeError as exc:
            raise RuntimeError(
                "overlay.video.hdmi_out not found; expected /video/hdmi_out"
            ) from exc

        hdmi_out.configure(
            VideoMode(
                EXPECTED_FRAME_WIDTH,
                EXPECTED_FRAME_HEIGHT,
                VIDEO_BPP,
            )
        )
        return hdmi_out

    def _open_camera(self, index):
        cap = cv2.VideoCapture(index)

        if not cap.isOpened():
            raise RuntimeError(
                f"cannot open camera index {index}"
            )

        cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            EXPECTED_FRAME_WIDTH,
        )
        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            EXPECTED_FRAME_HEIGHT,
        )
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

    def _clear_and_stop_hdmi(self):
        if not self._hdmi_running:
            return

        try:
            self.fpga.set_tracking_enable(False)
        except Exception:
            pass

        # Shutdown-only blanking. This is not part of the tracking algorithm.
        try:
            self.fpga.write_object_thresholds(255, 0, 255, 0)
        except Exception as exc:
            print(
                f"[SHUTDOWN] could not blank PL thresholds: {exc}",
                flush=True,
            )

        try:
            black = np.zeros(
                (
                    EXPECTED_FRAME_HEIGHT,
                    EXPECTED_FRAME_WIDTH,
                    3,
                ),
                dtype=np.uint8,
            )
            outframe = self.hdmi_out.newframe()
            outframe[:] = black
            self.hdmi_out.writeframe(outframe)

            time.sleep(HDMI_BLACK_HOLD_SEC)
            print("[SHUTDOWN] HDMI cleared to black", flush=True)
        except Exception as exc:
            print(
                f"[SHUTDOWN] final black frame failed: {exc}",
                flush=True,
            )

        try:
            self._stop_hdmi_stream()
            print("[SHUTDOWN] HDMI/VDMA stopped", flush=True)
        except Exception as exc:
            print(
                f"[SHUTDOWN] HDMI/VDMA stop failed: {exc}",
                flush=True,
            )

    # -------------------------------------------------------------------------
    # PL reads / video feed
    # -------------------------------------------------------------------------

    def _read_pl(self):
        """Read one coherent PL tracking snapshot."""
        td = self.fpga.read_tracking_data()
        self._last_tracking_data = td
        return td

    def _wait_for_new_snapshot(
        self,
        baseline,
        timeout=SNAPSHOT_STALL_TIMEOUT_SEC,
    ):
        """Wait until the cheap PL snapshot counter advances.

        This is the per-frame synchronization path.  It intentionally reads only
        REG_SNAPSHOT_COUNTER; the full AXI-Lite tracking register set is sampled
        separately at PL_STATUS_INTERVAL_SEC.
        """
        baseline = int(baseline) & 0xFFFFFFFF
        deadline = time.monotonic() + float(timeout)

        while not self._run_stop.is_set():
            snap = int(self.fpga.read_snapshot()) & 0xFFFFFFFF

            if snap != baseline:
                return snap

            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"PL snapshot counter did not advance from {baseline} "
                    f"within {timeout:.3f}s"
                )

            time.sleep(SNAPSHOT_POLL_SEC)

        return None

    def _feed_pl(self, frame):
        """Write one clean camera frame to the PYNQ HDMI/VDMA path."""
        outframe = self.hdmi_out.newframe()
        outframe[:] = frame
        self.hdmi_out.writeframe(outframe)

    def _feed_pl_and_sync(self, frame):
        """Feed one frame and wait only for the PL snapshot counter to advance."""
        pre_write_baseline = (
            int(self.fpga.read_snapshot()) & 0xFFFFFFFF
        )

        self._feed_pl(frame)

        return self._wait_for_new_snapshot(
            pre_write_baseline
        )

    # -------------------------------------------------------------------------
    # Public control
    # -------------------------------------------------------------------------

    def start_tracking(self, frame, bbox):
        """Configure the selected object and start hardware tracking."""
        if self._closed:
            raise RuntimeError(
                "tracker is closed; create a new PYNQObjectTracker"
            )

        if (
            self._run_thread is not None
            and self._run_thread.is_alive()
        ):
            if not self.stop_tracking():
                raise RuntimeError(
                    "previous tracking thread did not stop"
                )

        roi = tuple(bbox)
        x, y, w, h = roi
        crop = frame[y:y + h, x:x + w]

        thresholds = learn_pl_thresholds(crop)

        print(
            f"[INITIAL] ROI={roi} "
            f"Cr[{thresholds[0]},{thresholds[1]}] "
            f"Cb[{thresholds[2]},{thresholds[3]}]",
            flush=True,
        )

        # Disable while bringing up the video path.
        self.fpga.set_tracking_enable(False)

        print(
            "[PL-START] starting HDMI/VDMA with tracking disabled",
            flush=True,
        )
        self._ensure_hdmi_running()

        # Write the initial tracking configuration to the PL.
        self.fpga.write_initial_roi(*roi)
        self.fpga.write_object_thresholds(*thresholds)
        self.fpga.set_tracking_enable(True)

        self._initial_roi = roi
        self._pl_thresholds = thresholds
        self._last_tracking_data = None

        if self._acquisition_ui is not None:
            self._acquisition_ui.set_initial_config(
                roi,
                thresholds,
            )
        print("[MODE] PL hardware tracking active", flush=True)

        self.tracking = True
        self._start_run_thread()

        if (
            self._run_thread is None
            or not self._run_thread.is_alive()
        ):
            self.tracking = False
            self.fpga.set_tracking_enable(False)
            raise RuntimeError(
                "tracking thread failed to start"
            )

        return True

    def run(self):
        """Stream camera frames to the PL and display coherent tracking results."""
        camera_fails = 0
        snapshot_stalls = 0
        last_camera_ok = time.monotonic()
        last_pl_status_time = 0.0

        while not self._run_stop.is_set():
            ok, frame = self.cap.read()

            if not ok or frame is None:
                camera_fails += 1

                if (
                    camera_fails >= MAX_CAMERA_FAILS
                    or time.monotonic() - last_camera_ok
                    >= CAMERA_WATCHDOG_SEC
                ):
                    print(
                        "[CAMERA] camera stalled; stopping run loop",
                        flush=True,
                    )
                    break

                time.sleep(CAMERA_RETRY_SLEEP_SEC)
                continue

            camera_fails = 0
            last_camera_ok = time.monotonic()

            if (
                frame.shape[1] != EXPECTED_FRAME_WIDTH
                or frame.shape[0] != EXPECTED_FRAME_HEIGHT
            ):
                frame = cv2.resize(
                    frame,
                    (
                        EXPECTED_FRAME_WIDTH,
                        EXPECTED_FRAME_HEIGHT,
                    ),
                    interpolation=cv2.INTER_LINEAR,
                )

            try:
                if self._feed_pl_and_sync(frame) is None:
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
                print(
                    f"[VIDEO/PL] frame submission failed: {exc}",
                    flush=True,
                )
                break

            # The per-frame path above reads only REG_SNAPSHOT_COUNTER.
            # Read the complete coherent PL status only at the same interval at
            # which we print/update the UI, avoiding unnecessary AXI-Lite reads.
            now = time.monotonic()
            if now - last_pl_status_time >= PL_STATUS_INTERVAL_SEC:
                td = self._read_pl()

                print(
                    f"[PL] snapshot={td.snapshot} "
                    f"object_center=({td.object_cx},{td.object_cy}) "
                    f"object_size={td.object_w}x{td.object_h} "
                    f"pixels={td.pixel_count} "
                    f"ROI=({td.roi_x},{td.roi_y},{td.roi_w},{td.roi_h}) "
                    f"pred=({td.pred_cx},{td.pred_cy}) "
                    f"lost={int(td.tracking_lost)}",
                    flush=True,
                )

                if self._acquisition_ui is not None:
                    self._acquisition_ui.set_pl_data(td)
                    self._acquisition_ui.set_status(
                        "PL TRACKING | live hardware data updated from AXI-Lite"
                    )

                last_pl_status_time = now

        self.tracking = False

    def stop_tracking(self):
        self._run_stop.set()

        try:
            self.fpga.set_tracking_enable(False)
        except Exception:
            pass

        self.tracking = False
        return self._join_run_thread()

    def _begin_selection(self):
        if self._closed:
            return

        stopped = self.stop_tracking()

        if not stopped:
            print(
                "[SELECT] run thread is still active; "
                "refusing to reuse camera",
                flush=True,
            )
            return

        self._clear_and_stop_hdmi()

        if self._acquisition_ui is None:
            self._acquisition_ui = NotebookPLOnlyUI(
                self.cap,
                on_confirm=self._on_selection_confirmed,
                on_quit=self.close,
            ).show()
        else:
            self._acquisition_ui.enter_live_preview()

    def run_interactive(self):
        print(f"[RUNTIME] {RUNTIME_VERSION}", flush=True)
        print(
            "[UI] CAPTURE FRAME -> select initial ROI -> START PL TRACKING. "
            "Live PL tracking data is displayed during operation.",
            flush=True,
        )
        self._begin_selection()

    def status(self):
        td = self._last_tracking_data

        return {
            "tracking": bool(self.tracking),
            "mode": "PL_TRACKING",
            "initial_roi": self._initial_roi,
            "thresholds": self._pl_thresholds,
            "pl_snapshot": (
                None if td is None else int(td.snapshot)
            ),
            "pl_object_center": (
                None
                if td is None
                else (int(td.object_cx), int(td.object_cy))
            ),
            "pl_object_size": (
                None
                if td is None
                else (int(td.object_w), int(td.object_h))
            ),
            "pl_pixel_count": (
                None if td is None else int(td.pixel_count)
            ),
            "pl_roi": (
                None
                if td is None
                else (int(td.roi_x), int(td.roi_y), int(td.roi_w), int(td.roi_h))
            ),
            "pl_pred": (
                None
                if td is None
                else (int(td.pred_cx), int(td.pred_cy))
            ),
            "pl_lost": (
                None if td is None else bool(td.tracking_lost)
            ),
        }

    # -------------------------------------------------------------------------
    # Threading / shutdown
    # -------------------------------------------------------------------------

    def _on_selection_confirmed(self, frame, roi):
        return self.start_tracking(frame, roi)

    def _start_run_thread(self):
        if (
            self._run_thread is not None
            and self._run_thread.is_alive()
        ):
            return

        self._run_stop.clear()
        self._run_thread = threading.Thread(
            target=self.run,
            daemon=True,
            name="pynq-pl-only-tracker",
        )
        self._run_thread.start()

    def _join_run_thread(self):
        thread = self._run_thread

        if thread is None:
            return True

        if threading.current_thread() is thread:
            return False

        thread.join(
            timeout=RUN_THREAD_JOIN_TIMEOUT_SEC
        )
        stopped = not thread.is_alive()

        if stopped:
            self._run_thread = None

        return stopped

    def close(self):
        if self._closed:
            return

        self._closed = True

        ui = self._acquisition_ui
        self._acquisition_ui = None

        if ui is not None:
            try:
                ui.close(notify=False)
            except Exception:
                pass

        stopped = self.stop_tracking()

        if not stopped:
            print(
                "[SHUTDOWN] tracking_enable=0, but run thread "
                "did not stop before timeout; HDMI/camera left "
                "allocated to avoid driver race",
                flush=True,
            )
            return

        self._clear_and_stop_hdmi()

        try:
            self.cap.release()
            print("[SHUTDOWN] camera released", flush=True)
        except Exception as exc:
            print(
                f"[SHUTDOWN] camera release failed: {exc}",
                flush=True,
            )

        try:
            import matplotlib.pyplot as plt
            plt.close("all")
        except Exception:
            pass

        print(
            "[SHUTDOWN] COMPLETE: PL disabled; "
            "HDMI/VDMA stopped; camera released",
            flush=True,
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
