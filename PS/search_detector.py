"""
SearchDetector — non-blocking palm detection for the SEARCH phase.

Runs MPPalmDet.infer() in a background daemon thread so the main loop
continues at camera frame rate during SEARCH instead of blocking 200–500 ms
per frame on the ARM Cortex-A9.

Pattern mirrors HandValidator: poll() before submit() each frame so that a
result produced by a finished thread is consumed before a new thread starts.
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Optional

import cv2
import numpy as np

from tracker_utils import (
    skin_ratio,
    SKIN_CR_MIN, SKIN_CR_MAX, SKIN_CB_MIN, SKIN_CB_MAX,
)

if TYPE_CHECKING:
    from mp_palmdet import MPPalmDet

# Pre-scan that skips inference when no skin is visible in the frame.
_PRESCAN_MIN_RATIO = 0.005   # 0.5 % of pixels — permissive rejection, not a gate
_PRESCAN_SCALE     = 4       # downsample 640→160, 480→120 before scanning


def _frame_has_skin(frame_bgr: np.ndarray) -> bool:
    """True if the downsampled frame contains any skin-coloured pixels.

    Runs on a 1/4-resolution copy so it completes in < 2 ms on ARM.
    The threshold is intentionally very low — this is a fast skip filter,
    not a detection gate.
    """
    h, w = frame_bgr.shape[:2]
    small = cv2.resize(frame_bgr, (w // _PRESCAN_SCALE, h // _PRESCAN_SCALE),
                       interpolation=cv2.INTER_NEAREST)
    return skin_ratio(small, SKIN_CR_MIN, SKIN_CR_MAX,
                      SKIN_CB_MIN, SKIN_CB_MAX) >= _PRESCAN_MIN_RATIO


class SearchDetector:
    """Non-blocking wrapper around MPPalmDet for the SEARCH phase.

    Call poll() BEFORE submit() each frame to avoid losing results::

        sd = SearchDetector(det)
        # Per-frame in SEARCH:
        ready, det_results, det_frame = sd.poll()   # consume previous result first (instant)
        sd.submit(frame)                              # launch next inference if idle (non-blocking)
        if ready:
            stable = run_search_frame(det_frame, state, det_results)

    Call reset() on any transition out of SEARCH so stale results are discarded.
    """

    def __init__(self, detector: "MPPalmDet", skin_prescan: bool = True) -> None:
        self._det     = detector
        self._prescan = skin_prescan

        self._lock         = threading.Lock()
        self._running      = False
        self._result_ready = False
        self._result       = None   # raw det.infer() output set by worker thread
        self._frame        = None   # frame snapshot the result was computed from

        self._thread: Optional[threading.Thread] = None
        self._generation: int = 0

    def submit(self, frame: np.ndarray) -> bool:
        """Submit frame for background inference.

        Returns False (no-op) if a thread is already running or a result is
        waiting to be consumed by poll(). Returns False if the skin pre-scan
        finds no skin-coloured pixels. Otherwise starts the background thread
        and returns True.

        Always call poll() before submit() in the same loop iteration.
        """
        with self._lock:
            if self._running or self._result_ready:
                return False   # busy or result not yet consumed

        if self._prescan and not _frame_has_skin(frame):
            return False   # no skin visible — skip inference to save CPU

        frame_copy = frame.copy()   # snapshot to prevent data race with main thread

        with self._lock:
            if self._running or self._result_ready:
                return False   # re-check after prescan (TOCTOU guard)
            self._running      = True
            self._result_ready = False
            gen = self._generation

        self._thread = threading.Thread(
            target=self._run_thread, args=(frame_copy, gen), daemon=True)
        self._thread.start()
        return True

    def poll(self) -> tuple[bool, object, object]:
        """Return (result_ready, results, frame_snapshot).

        frame_snapshot is the exact frame the detector was run on — pass it
        to run_search_frame so Gate 4 checks the right pixels. Both results
        and frame_snapshot are None when result_ready is False.

        result_ready is True exactly once per completed inference — consuming
        the result resets the flag. Always instant; never blocks.
        """
        with self._lock:
            if not self._result_ready:
                return False, None, None
            r = self._result
            f = self._frame
            self._result       = None
            self._frame        = None
            self._result_ready = False
        return True, r, f

    def reset(self) -> None:
        """Discard any pending result. Running thread (if any) completes in
        background but its output is silently dropped."""
        with self._lock:
            self._result       = None
            self._frame        = None
            self._result_ready = False
            self._generation  += 1

    def _run_thread(self, frame: np.ndarray, generation: int) -> None:
        try:
            result = self._det.infer(frame)
        except Exception as e:
            print(f'[SEARCH] inference error: {e}')
            result = None
        with self._lock:
            self._running = False              # always clear before generation check
            if self._generation != generation:
                return                         # stale thread — discard result
            self._result       = result
            self._frame        = frame         # already a deep copy from submit()
            self._result_ready = True
