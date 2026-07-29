"""
Async periodic hand validator for the PYNQ-Z2 hand-tracking system.

During TRACKING the palm detector (MPPalmDet) can take 200-500 ms on the
ARM Cortex-A9. Running it synchronously would stall the main loop.

HandValidator launches the detector in a daemon background thread and
exposes two non-blocking calls:
  is_due(now)                         → should we start a new validation?
  start_validation(frame, roi, now)   → launch background inference (returns immediately)
  poll_result()                       → check if the thread finished (returns immediately)

IMPORTANT: the validation ROI passed to start_validation() must be the FPGA's
current ROI (REG_CURRENT_ROI_X/Y/W/H — the window the PL is tracking right now),
NOT state.last_roi, because state.last_roi is the stale initial SEARCH ROI.
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Optional

import numpy as np

from tracker_utils import _has_results, pick_best_result_with_index

if TYPE_CHECKING:
    from mp_palmdet import MPPalmDet

DEFAULT_VALIDATION_INTERVAL = 3.0   # seconds between validation triggers
DEFAULT_MAX_FAILURES        = 3     # consecutive failures before reporting invalid
DEFAULT_MIN_SCORE           = 0.90  # detection must meet this confidence to count as valid
DEFAULT_OVERLAP_MIN_RATIO   = 0.30  # min overlap÷min-area for a detection to count as valid


class HandValidator:
    """Non-blocking periodic hand presence validator.

    Wraps MPPalmDet in a background thread so the main tracking loop is
    never blocked. Call is_due() → start_validation() → poll_result()
    on successive frames during TRACKING.
    """

    def __init__(
        self,
        detector: "MPPalmDet",
        interval_seconds: float = DEFAULT_VALIDATION_INTERVAL,
        max_consecutive_failures: int = DEFAULT_MAX_FAILURES,
        min_score: float = DEFAULT_MIN_SCORE,
        overlap_min_ratio: float = DEFAULT_OVERLAP_MIN_RATIO,
    ) -> None:
        self._det               = detector
        self._interval          = interval_seconds
        self._max               = max_consecutive_failures
        self._min_score         = min_score
        self._overlap_min_ratio = overlap_min_ratio

        self._failure_count: int   = 0
        self._last_time:     float = 0.0
        self._running:       bool  = False
        self._result_ready:  bool  = False
        self._last_result:   bool  = True
        self._generation:    int   = 0

        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    # ── Public non-blocking API ───────────────────────────────────────────────

    def is_due(self, current_time: float) -> bool:
        """Return True if it is time to start a new validation run.

        Does NOT start the run — call start_validation() separately.
        Returns False while a background thread is already running.
        """
        with self._lock:
            running = self._running
        return (not running) and (
            (current_time - self._last_time) >= self._interval
        )

    def start_validation(
        self,
        frame_bgr: np.ndarray,
        validation_roi: tuple[int, int, int, int],
        current_time: float,
    ) -> None:
        """Launch background inference (non-blocking).

        validation_roi is the FPGA's current ROI (the window the PL is tracking),
        in corner form (x1, y1, x2, y2), clamped to [0, FRAME_W] x [0, FRAME_H]
        with an optional VAL_PAD margin. Success = a detected palm whose box
        overlaps this ROI by >= overlap_min_ratio (overlap ÷ min-area) with
        score >= min_score.

        Silently returns if a thread is already running (no double-launch).
        """
        with self._lock:
            if self._running:
                return
            self._running      = True
            self._result_ready = False
            gen = self._generation

        self._last_time = current_time
        frame_copy = frame_bgr.copy()   # snapshot to prevent data race

        self._thread = threading.Thread(
            target=self._run_thread,
            args=(frame_copy, validation_roi, gen),
            daemon=True,
        )
        self._thread.start()

    def poll_result(self) -> tuple[bool, bool]:
        """Check whether the background thread has produced a result.

        Returns:
            (result_ready, is_still_valid)

        If result_ready is False the thread has not finished yet; assume the
        hand is still present (is_still_valid=True) and poll again next frame.
        Calling this is always instant — it never blocks.
        """
        with self._lock:
            if not self._result_ready:
                return False, True
            self._result_ready = False
            return True, self._last_result

    def reset(self) -> None:
        """Reset failure counter and timer. Does NOT kill a running thread."""
        with self._lock:
            self._generation   += 1
            self._failure_count = 0
            self._last_time     = 0.0
            self._last_result   = True
            self._result_ready  = False

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def failure_count(self) -> int:
        with self._lock:
            return self._failure_count

    # ── Background thread ─────────────────────────────────────────────────────

    def _run_thread(
        self,
        frame_bgr: np.ndarray,
        validation_roi: tuple[int, int, int, int],
        generation: int,
    ) -> None:
        """Runs in background thread. Never call directly."""
        try:
            results = self._det.infer(frame_bgr)  # full frame — model needs context

            success = False
            if _has_results(results):
                best, _ = pick_best_result_with_index(results)
                if best is not None:
                    best_score = float(best[-1])
                    # Does the detected box overlap the ROI the PL is tracking?
                    # Overlap ÷ min-area (NOT IoU): the FPGA ROI (skin blob + pad)
                    # and the model's palm box are different sizes, and IoU would
                    # reject a hand that is clearly inside the tracked window.
                    dx1, dy1, dx2, dy2 = (float(best[0]), float(best[1]),
                                          float(best[2]), float(best[3]))
                    rx1, ry1, rx2, ry2 = validation_roi
                    ix1, iy1 = max(dx1, rx1), max(dy1, ry1)
                    ix2, iy2 = min(dx2, rx2), min(dy2, ry2)
                    inter    = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
                    det_area = max(0.0, dx2 - dx1) * max(0.0, dy2 - dy1)
                    roi_area = max(0.0, rx2 - rx1) * max(0.0, ry2 - ry1)
                    min_area = min(det_area, roi_area)
                    overlap  = (inter / min_area) if min_area > 0 else 0.0
                    success  = (best_score >= self._min_score) \
                        and (overlap >= self._overlap_min_ratio)

        except Exception as e:
            print(f'[VALIDATOR] inference error: {e}')
            success = False

        with self._lock:
            self._running = False                  # always clear before generation check
            if self._generation != generation:
                return                             # stale thread — discard result
            if success:
                self._failure_count = 0
                self._last_result   = True
            else:
                self._failure_count += 1
                self._last_result = self._failure_count < self._max
            self._result_ready = True
