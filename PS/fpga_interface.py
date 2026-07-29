"""
FPGA abstraction layer for the PYNQ-Z2 hand-tracking system.

Provides StubFPGAInterface (software testing without hardware) and
PYNQFPGAInterface (real AXI-Lite MMIO on the board). 

AXI register map is the software-hardware contract for the VHDL design.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass

# ── AXI Register Map ─────────────────────────────────────────────────────────
# Write registers (PS → PL). ROI is the INITIAL seed only: it is written once at
# calibration/handoff. During TRACKING the PL owns and updates its own internal
# ROI — the PS never writes ROI again until the next handoff.
REG_CTRL             = 0x00
REG_ROI_X            = 0x04
REG_ROI_Y            = 0x08
REG_ROI_W            = 0x0C
REG_ROI_H            = 0x10
REG_CR_MIN           = 0x14
REG_CR_MAX           = 0x18
REG_CB_MIN           = 0x1C
REG_CB_MAX           = 0x20
# Offset 0x24 was formerly REG_MIN_AREA — removed. The PL no longer does
# semantic LOST detection, so it needs no PS-supplied area threshold; any
# minimum pixel-count floor is now a PL-internal constant.
# Offsets 0x28–0x2C were formerly REG_VALID_K / REG_LOST_M.
# Offsets 0x30–0x3C were formerly KP / KI / DEADBAND / STEP_LIMIT.
# All of these are now hardcoded constants inside the PL and are
# not addressable from the PS.

# Read registers (PL → PS). The PL reports only geometric measurements and the
# ROI it is currently tracking. It makes NO semantic decision — there is no
# LOCKED / LOST / quality flag. "Is this still a hand?" is decided by the PS
# (HandValidator) from the CURRENT_ROI registers below.
REG_CX               = 0x44
REG_CY               = 0x48
REG_BBOX_W           = 0x4C
REG_BBOX_H           = 0x50
REG_SKIN_PIXEL_COUNT = 0x54   # formerly REG_AREA: count of skin-classified pixels
REG_CURRENT_ROI_X    = 0x58   # the ROI the PL is tracking right now (PL-updated)
REG_CURRENT_ROI_Y    = 0x5C
REG_CURRENT_ROI_W    = 0x60
REG_CURRENT_ROI_H    = 0x64
# Offset 0x40 was formerly REG_STATUS (LOCKED/LOST bits) — removed; reserved.

# ── CTRL bit positions ────────────────────────────────────────────────────────
# Servo priority (VHDL must honour in descending order, applied PER AXIS —
# the PL drives two servos: pan follows error_x, tilt follows error_y):
#   1. SERVO_HOLD=1         → freeze at current position (highest priority)
#   2. TRACK_ENABLE=1       → follow centroid PD controller
#   3. SEARCH_SWEEP_ENABLE=1 → slow left-right sweep (pan only; tilt re-centres
#                              during SEARCH — its sweep input is tied 0 in PL)
#   4. (else)               → return to / hold centre
BIT_TRACK_ENABLE       = 0
BIT_SERVO_ENABLE       = 1
BIT_SERVO_HOLD         = 2
BIT_SEARCH_SWEEP_ENABLE = 3  # physical servo sweep action during SEARCH
# Former bit 3 (SEARCH_MODE) removed: SEARCH_SWEEP_ENABLE already expresses
# the only hardware behaviour SEARCH needs from the PL, so the semantic flag
# was redundant. SWEEP_ENABLE moved 4 → 3 (no AXI layer existed yet).

_BIT_NAMES = {0: 'TRACK_EN', 1: 'SERVO_EN', 2: 'SERVO_HOLD', 3: 'SWEEP_EN'}


@dataclass
class TrackingData:
    """Geometry the PL reports each frame, plus the ROI it is tracking.

    No status flags: the PL does not decide LOCKED/LOST. The PS validates the
    current_roi_* window against the palm model to decide if it is still a hand.
    """
    cx: int = 0
    cy: int = 0
    bbox_w: int = 0
    bbox_h: int = 0
    skin_pixel_count: int = 0
    current_roi_x: int = 0
    current_roi_y: int = 0
    current_roi_w: int = 0
    current_roi_h: int = 0


class FPGAInterface(abc.ABC):
    """Abstract base for FPGA communication.

    Concrete subclasses implement only write_register / read_register.
    All higher-level helpers are implemented here so both subclasses
    inherit them identically.

    The _ctrl_shadow tracks the current CTRL register value so that
    set_ctrl_bit() can change one bit without inadvertently clearing others.
    Never write REG_CTRL via write_register() directly — always use
    write_ctrl() or set_ctrl_bit().
    """

    def __init__(self) -> None:
        self._ctrl_shadow: int = 0

    # ── Abstract ──────────────────────────────────────────────────────────────

    @abc.abstractmethod
    def write_register(self, offset: int, value: int) -> None:
        ...

    @abc.abstractmethod
    def read_register(self, offset: int) -> int:
        ...

    # ── CTRL helpers ──────────────────────────────────────────────────────────

    def write_ctrl(
        self,
        track_enable: bool,
        servo_enable: bool,
        servo_hold: bool,
        search_sweep_enable: bool,
    ) -> None:
        """Compose and write all four CTRL bits atomically."""
        word = 0
        if track_enable:        word |= (1 << BIT_TRACK_ENABLE)
        if servo_enable:        word |= (1 << BIT_SERVO_ENABLE)
        if servo_hold:          word |= (1 << BIT_SERVO_HOLD)
        if search_sweep_enable: word |= (1 << BIT_SEARCH_SWEEP_ENABLE)
        self._ctrl_shadow = word
        print(f'[FPGA] CTRL  track={int(track_enable)} servo={int(servo_enable)} '
              f'hold={int(servo_hold)} sweep={int(search_sweep_enable)}')
        self.write_register(REG_CTRL, word)

    def set_ctrl_bit(self, bit: int, value: bool) -> None:
        """Set a single CTRL bit, preserving all others via shadow register.

        Use for emergency-stop cases (e.g., BIT_SERVO_HOLD) where changing
        only one bit without a full write_ctrl() call is necessary.
        """
        print(f'[FPGA] SET_BIT  {_BIT_NAMES.get(bit, bit)}={int(value)}')
        if value:
            self._ctrl_shadow |= (1 << bit)
        else:
            self._ctrl_shadow &= ~(1 << bit)
        self.write_register(REG_CTRL, self._ctrl_shadow)

    # ── Parameter writes ──────────────────────────────────────────────────────

    def write_roi(self, x: int, y: int, w: int, h: int) -> None:
        self.write_register(REG_ROI_X, int(x))
        self.write_register(REG_ROI_Y, int(y))
        self.write_register(REG_ROI_W, int(w))
        self.write_register(REG_ROI_H, int(h))

    def write_skin_thresholds(
        self, cr_min: int, cr_max: int, cb_min: int, cb_max: int
    ) -> None:
        print(f'[FPGA] SKIN   Cr[{cr_min},{cr_max}]  Cb[{cb_min},{cb_max}]')
        self.write_register(REG_CR_MIN, int(cr_min))
        self.write_register(REG_CR_MAX, int(cr_max))
        self.write_register(REG_CB_MIN, int(cb_min))
        self.write_register(REG_CB_MAX, int(cb_max))

    # ── Data reads ────────────────────────────────────────────────────────────

    def read_tracking_data(self) -> TrackingData:
        """Read the PL's geometric measurements and its current tracking ROI.

        No status word: the PL reports geometry only. Deciding whether the
        current ROI still holds a real hand is the PS's job (HandValidator).
        """
        return TrackingData(
            cx=self.read_register(REG_CX),
            cy=self.read_register(REG_CY),
            bbox_w=self.read_register(REG_BBOX_W),
            bbox_h=self.read_register(REG_BBOX_H),
            skin_pixel_count=self.read_register(REG_SKIN_PIXEL_COUNT),
            current_roi_x=self.read_register(REG_CURRENT_ROI_X),
            current_roi_y=self.read_register(REG_CURRENT_ROI_Y),
            current_roi_w=self.read_register(REG_CURRENT_ROI_W),
            current_roi_h=self.read_register(REG_CURRENT_ROI_H),
        )


# ── StubFPGAInterface ─────────────────────────────────────────────────────────

class StubFPGAInterface(FPGAInterface):
    """Software-only FPGA stub for development without hardware.

    Plain register file. The read registers start at a centred-hand default so a
    stub-mode run shows something during TRACKING; write_register overrides any
    value (tests write the geometry they want, then read it back).
    """

    # Default read-register values: a centred hand with the ROI around it.
    _DEFAULT_READS = {
        REG_CX: 320, REG_CY: 240,
        REG_BBOX_W: 120, REG_BBOX_H: 140,
        REG_SKIN_PIXEL_COUNT: 9000,
        REG_CURRENT_ROI_X: 230, REG_CURRENT_ROI_Y: 90,
        REG_CURRENT_ROI_W: 180, REG_CURRENT_ROI_H: 300,
    }

    def __init__(self) -> None:
        super().__init__()
        self._regs: dict[int, int] = dict(self._DEFAULT_READS)

    def write_register(self, offset: int, value: int) -> None:
        self._regs[offset] = int(value)

    def read_register(self, offset: int) -> int:
        return self._regs.get(offset, 0)


# ── PYNQFPGAInterface ─────────────────────────────────────────────────────────

class PYNQFPGAInterface(FPGAInterface):
    """Real FPGA interface using PYNQ Overlay + MMIO.

    pynq is imported lazily inside __init__ so this module can be imported
    on a development PC without pynq installed. An ImportError only surfaces
    when actually instantiating this class (i.e., USE_STUB=False).
    """

    def __init__(
        self,
        bitstream_path: str,
        base_address: int,
        address_range: int = 0x1000,   # 4 KB — standard AXI4-Lite slave minimum page
    ) -> None:
        super().__init__()
        from pynq import Overlay, MMIO  # deferred import
        self._overlay = Overlay(bitstream_path)
        self._mmio = MMIO(base_address, address_range)

    def write_register(self, offset: int, value: int) -> None:
        self._mmio.write(offset, int(value))

    def read_register(self, offset: int) -> int:
        return self._mmio.read(offset)
