"""
AXI-Lite register interface for the PYNQ-Z2 vision tracker.

The register map here is the software half of the contract with the VHDL in
hdl/vision_tracker_ip*.vhd — that HDL is the source of truth. Offsets and the
object-threshold packing below mirror it exactly; do not change one without the
other.
"""

from __future__ import annotations

from dataclasses import dataclass

# ── Register map (see hdl/vision_tracker_ip_slave_lite_v1_0_S_AXI_TRACK.vhd) ──
# Writes (PS → PL)
REG_TRACKING_CONTROL = 0x00   # bit 0 = tracking_enable (the only control bit)
REG_INITIAL_ROI_X    = 0x14
REG_INITIAL_ROI_Y    = 0x18
REG_INITIAL_ROI_W    = 0x1C
REG_INITIAL_ROI_H    = 0x20
REG_OBJECT_THRESHOLDS  = 0x24   # packed: cr_min | cr_max<<8 | cb_min<<16 | cb_max<<24

# Reads (PL → PS)
REG_TRACKING_LOST    = 0x04   # bit 0
REG_SNAPSHOT_COUNTER = 0x08   # +1 whenever the PL publishes a new coherent snapshot
REG_FRAME_WIDTH      = 0x0C
REG_FRAME_HEIGHT     = 0x10
REG_OBJECT_CX        = 0x28
REG_OBJECT_CY        = 0x2C
REG_OBJECT_W         = 0x30
REG_OBJECT_H         = 0x34
REG_OBJECT_PIXELS    = 0x38
REG_ROI_X            = 0x3C
REG_ROI_Y            = 0x40
REG_ROI_W            = 0x44
REG_ROI_H            = 0x48
REG_PRED_CX          = 0x4C   # signed
REG_PRED_CY          = 0x50   # signed


def _signed32(v: int) -> int:
    return v - (1 << 32) if v & 0x80000000 else v


def pack_object_thresholds(cr_min: int, cr_max: int, cb_min: int, cb_max: int) -> int:
    """Pack the four 8-bit Cr/Cb bounds into the 0x24 word, byte order per
    vision_tracker_ip.vhd:341-344 (cr_min=LSB ... cb_max=MSB)."""
    return (cr_min & 0xFF) | (cr_max & 0xFF) << 8 \
         | (cb_min & 0xFF) << 16 | (cb_max & 0xFF) << 24


@dataclass(frozen=True)
class TrackingData:
    """One coherent snapshot of the PL's tracking result registers."""
    object_cx: int
    object_cy: int
    object_w: int
    object_h: int
    pixel_count: int
    roi_x: int
    roi_y: int
    roi_w: int
    roi_h: int
    pred_cx: int
    pred_cy: int
    tracking_lost: bool
    snapshot: int = 0


class FPGA:
    """Real PYNQ AXI-Lite interface. pynq is imported lazily so this module
    imports fine on a dev PC without a board."""

    def __init__(self, bitstream_path: str, base_address: int,
                 address_range: int = 0x1000) -> None:
        from pynq import Overlay, MMIO   # deferred: only needed on the board
        self.overlay = Overlay(bitstream_path)          # public: run_tracker reads hdmi_out off it
        self._mmio = MMIO(base_address, address_range)
        self.tracking_enable = False                    # last value we wrote to 0x00

    def _read(self, off: int) -> int:
        return self._mmio.read(off)

    def _write(self, off: int, val: int) -> None:
        self._mmio.write(off, int(val) & 0xFFFFFFFF)

    # ── Config writes (PS → PL) ───────────────────────────────────────────────

    def read_frame_size(self) -> tuple[int, int]:
        return self._read(REG_FRAME_WIDTH), self._read(REG_FRAME_HEIGHT)

    def read_snapshot(self) -> int:
        """Cheap freshness probe: the counter advances when new PL data is ready."""
        return self._read(REG_SNAPSHOT_COUNTER)

    def set_tracking_enable(self, on: bool) -> None:
        on = bool(on)
        if on != self.tracking_enable:
            print(f'[CTRL] tracking_enable {int(self.tracking_enable)} -> {int(on)}')
        self.tracking_enable = on
        self._write(REG_TRACKING_CONTROL, 1 if on else 0)

    def write_initial_roi(self, x: int, y: int, w: int, h: int) -> None:
        self._write(REG_INITIAL_ROI_X, x)
        self._write(REG_INITIAL_ROI_Y, y)
        self._write(REG_INITIAL_ROI_W, w)
        self._write(REG_INITIAL_ROI_H, h)

    def write_object_thresholds(self, cr_min: int, cr_max: int,
                                cb_min: int, cb_max: int) -> None:
        self._write(
            REG_OBJECT_THRESHOLDS,
            pack_object_thresholds(cr_min, cr_max, cb_min, cb_max)
        )

    # ── Result read (PL → PS) ─────────────────────────────────────────────────

    def read_tracking_data(self, retries: int = 5) -> TrackingData:
        # Retry until the snapshot counter is unchanged across the whole read, so
        # every field belongs to the same PL snapshot (no mid-read publish).
        # Bounded, so a fast-publishing PL can never wedge the caller — after
        # `retries` we accept the last read (each field is still individually valid).
        for _ in range(retries):
            snap = self._read(REG_SNAPSHOT_COUNTER)
            td = TrackingData(
                object_cx=self._read(REG_OBJECT_CX),
                object_cy=self._read(REG_OBJECT_CY),
                object_w=self._read(REG_OBJECT_W),
                object_h=self._read(REG_OBJECT_H),
                pixel_count=self._read(REG_OBJECT_PIXELS),
                roi_x=self._read(REG_ROI_X),
                roi_y=self._read(REG_ROI_Y),
                roi_w=self._read(REG_ROI_W),
                roi_h=self._read(REG_ROI_H),
                pred_cx=_signed32(self._read(REG_PRED_CX)),
                pred_cy=_signed32(self._read(REG_PRED_CY)),
                tracking_lost=bool(self._read(REG_TRACKING_LOST) & 1),
                snapshot=snap,
            )
            if self._read(REG_SNAPSHOT_COUNTER) == snap:
                break
        return td


if __name__ == '__main__':   # ponytail: smallest runnable check for the HDL contract
    assert pack_object_thresholds(133, 173, 77, 127) == (127 << 24) | (77 << 16) | (173 << 8) | 133
    assert pack_object_thresholds(0, 0, 0, 0) == 0
    assert pack_object_thresholds(0xFF, 0xFF, 0xFF, 0xFF) == 0xFFFFFFFF
    assert _signed32(0) == 0
    assert _signed32(759) == 759
    assert _signed32((1 << 32) - 120) == -120   # HW sign-extends predicted center

    # read_tracking_data retries until the snapshot counter is stable across the read
    class _Fake:
        def __init__(self, snaps): self.snaps = list(snaps)
        def _read(self, off):
            return self.snaps.pop(0) if off == REG_SNAPSHOT_COUNTER else 0
    coherent = FPGA.__new__(FPGA); coherent._read = _Fake([7, 7])._read
    assert coherent.read_tracking_data().snapshot == 7          # accepted first try
    torn = FPGA.__new__(FPGA); torn._read = _Fake([1, 2, 3, 3])._read
    assert torn.read_tracking_data().snapshot == 3              # retried, then coherent

    class _Never:                                               # counter never stable
        def __init__(self): self.n = 0
        def _read(self, off):
            if off == REG_SNAPSHOT_COUNTER:
                self.n += 1
                return self.n
            return 0
    never = FPGA.__new__(FPGA); never._read = _Never()._read
    assert never.read_tracking_data(retries=3) is not None      # bounded — returns, never wedges
    print('fpga.py self-check ok')