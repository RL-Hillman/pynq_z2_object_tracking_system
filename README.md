# PYNQ-Z2 FPGA Based Real-Time Object Tracking System

An **FPGA-based real-time object tracking system** implemented on the **PYNQ-Z2**.

The core of the system is a **custom VHDL tracking engine** running entirely in the FPGA's **Programmable Logic (PL)**. Once initialized, the FPGA performs the complete frame-by-frame tracking process in hardware, including object segmentation, mask filtering, object measurement, centroid calculation, motion estimation, prediction of the object's next position, adaptive ROI control, loss detection and recovery, and debug-video generation.

The **Processing System (PS)** is responsible for camera interfacing, frame delivery, initialization, configuration, user interaction, optional high-level supervision, and displaying the tracking information received from the PL.

The project includes **two software operating modes**, both using the same hardware tracker:

- **General Object Tracking** - the user selects an object once, and the PS learns the initial color thresholds and provides the initial ROI. From that point, the **FPGA performs the complete continuous tracking process autonomously**.

- **Validated Hand Tracking** - demonstrates how an **AI model can be combined with the hardware tracker**. A palm-detection model running on the PS is used to acquire a hand and periodically validate that the FPGA is still tracking the correct target.


## System Architecture and Vivado Integration

The tracking system is implemented as a custom Vivado IP, `vision_tracker_ip`, integrated directly into the PYNQ-Z2 video pipeline.

The custom IP provides:

- AXI4-Stream video input (`S_AXIS_VIDEO`),
- processed/debug AXI4-Stream video output (`VIDEO_OUT`),
- AXI-Lite control/status interface (`S_AXI_TRACK`),
- and the complete internal VHDL tracking pipeline.

### Custom Tracker Integration into the Video Pipeline

<p align="center">
  <img src="docs/hdmi_tracker_integration.png" width="100%">
</p>

The figure above shows the actual Vivado integration of `vision_tracker_ip_0` inside the HDMI output subsystem, where the custom tracker is inserted **in-line in the FPGA video stream** so that incoming pixels pass directly through the hardware tracking pipeline.

The surrounding blocks perform the following functions:

1. **`pixel_unpack`** - converts the incoming packed video stream to the 24-bit pixel format used by the processing path.
2. **`axis_register_slice_0`** - provides an AXI4-Stream register stage before the tracker.
3. **`axis_clk_to_tracker`** - transfers the video stream from the **142 MHz HDMI/video clock domain** into the dedicated **100 MHz tracking domain**.
4. **`vision_tracker_ip_0`** - executes the VHDL tracking pipeline and updates the tracking data.
5. **`axis_clk_from_tracker`** - transfers the processed stream from the tracking domain back to the **142 MHz HDMI/video clock domain**..
6. **`color_convert` and HDMI frontend** - format the resulting video stream and drive the physical TMDS output.

### AXI Interfaces

The custom `vision_tracker_ip` uses two AXI interfaces with separate roles:

- **AXI4-Stream** - transports the video stream through the hardware tracking pipeline.
- **AXI-Lite** - provides configuration and status communication between the PS and the tracker.

#### AXI4-Stream Video

The `S_AXIS_VIDEO` interface transports 24-bit RGB pixels together with the standard AXI4-Stream video control signals:

- `tdata` - RGB pixel data,
- `tvalid` - indicates that the current pixel data is valid and ready to be transferred,
- `tuser` - start of frame,
- `tlast` - end of line,
- `tready` - indicates that the receiving block is ready to accept the current pixel, enabling AXI4-Stream flow control.

#### AXI-Lite Control and Status

The PS configures and observes the tracker through the 32-bit `S_AXI_TRACK` AXI-Lite interface.

The tracker is memory-mapped at:

```text
0x43C30000
```
##### Register Map

| Offset | Direction | Register | Description |
|---:|:---:|---|---|
| **PS → PL Configuration** | | | |
| `0x00` | PS → PL | `TRACKING_CONTROL` | Bit 0 enables tracking |
| `0x14` | PS → PL | `INITIAL_ROI_X` | Initial ROI X |
| `0x18` | PS → PL | `INITIAL_ROI_Y` | Initial ROI Y |
| `0x1C` | PS → PL | `INITIAL_ROI_WIDTH` | Initial ROI width |
| `0x20` | PS → PL | `INITIAL_ROI_HEIGHT` | Initial ROI height |
| `0x24` | PS → PL | `OBJECT_THRESHOLDS` | Packed Cr/Cb bounds |
| **PL → PS Status** | | | |
| `0x04` | PL → PS | `TRACKING_STATUS` | Bit 0 = tracking lost |
| `0x08` | PL → PS | `SNAPSHOT_COUNTER` | Increments on each coherent published result |
| `0x0C` | PL → PS | `FRAME_WIDTH` | Hardware frame width |
| `0x10` | PL → PS | `FRAME_HEIGHT` | Hardware frame height |
| `0x28` | PL → PS | `OBJECT_CENTER_X` | Measured centroid X |
| `0x2C` | PL → PS | `OBJECT_CENTER_Y` | Measured centroid Y |
| `0x30` | PL → PS | `OBJECT_WIDTH` | Measured object width |
| `0x34` | PL → PS | `OBJECT_HEIGHT` | Measured object height |
| `0x38` | PL → PS | `OBJECT_PIXEL_COUNT` | Number of matching pixels |
| `0x3C` | PL → PS | `TRACKING_ROI_X` | Current ROI X |
| `0x40` | PL → PS | `TRACKING_ROI_Y` | Current ROI Y |
| `0x44` | PL → PS | `TRACKING_ROI_WIDTH` | Current ROI width |
| `0x48` | PL → PS | `TRACKING_ROI_HEIGHT` | Current ROI height |
| `0x4C` | PL → PS | `PREDICTED_CENTER_X` | Signed predicted X |
| `0x50` | PL → PS | `PREDICTED_CENTER_Y` | Signed predicted Y |

The four 8-bit object thresholds are packed into `OBJECT_THRESHOLDS` (`0x24`) as:

```text
bits  7:0   = Cr min
bits 15:8   = Cr max
bits 23:16  = Cb min
bits 31:24  = Cb max
```

#### Snapshot Counter

The PL increments `SNAPSHOT_COUNTER` whenever a new tracking result is published. This allows the PS to detect when fresh tracking data is available.

When the PS reads the complete tracking result, `fpga.py` uses this counter to ensure that **all values being read belong to the same processed frame**. `fpga.py` checks the counter both before and after reading the tracking registers; if it changes during the read, a new result was published before the read was completed, so the registers are read again.

## FPGA / PL Design

The FPGA implements a complete frame-to-frame tracking feedback loop. For each frame, the PL measures the object, estimates its motion, predicts its next position, and uses the predicted position and motion information to determine the ROI for the following frame.

```text
Frame N
   ↓
Object measurement
   ↓
Center and size
   ↓
Velocity estimation
   ↓
Next-position prediction
   ↓
Speed-dependent ROI sizing and positioning
   ↓
ROI for Frame N+1
```

### FPGA Processing Pipeline

The tracking engine is implemented as a sequence of dedicated VHDL blocks:

```mermaid
flowchart TD
    A[24-bit RGB Video Stream] --> B[rgb_to_ycrcb]
    B --> C[roi_mask_generator]
    C --> D[mask_cleaner]
    D --> E[object_measurement]
    E --> F[roi_tracker_update]

    D --> G[object_display]
    F --> G

    F --> H[Coherent PS Result Snapshot]
    G --> I[Debug Video Stream]
```

### 1. `rgb_to_ycrcb.vhd`

Converts each 24-bit RGB pixel to YCrCb using fixed-point integer arithmetic:

```text
Y  = ( 77R + 150G +  29B) >> 8
Cb = ((-43R - 85G + 128B) >> 8) + 128
Cr = ((128R -107G - 21B) >> 8) + 128
```

Object classification is based exclusively on the **Cr and Cb chrominance components** of the YCrCb color space. The **Y (luminance) component is excluded from the tracking process**, making the system less sensitive to changes in lighting conditions.

### 2. `roi_mask_generator.vhd`

Tracks the coordinates of the incoming raster and produces a one-bit object mask.

A pixel is classified as part of the target only when:

```text
pixel is inside the current ROI
AND cr_min <= Cr <= cr_max
AND cb_min <= Cb <= cb_max
```

Pixels outside the ROI are forced to zero. Limiting classification to the current ROI reduces the search region and prevents unrelated areas of the frame from participating in the measurement.


### 3. `mask_cleaner.vhd`

Cleans the binary object mask using a generic odd-sized **K × K majority filter**.

For any odd kernel size `K`:

```text
Kernel:              K × K
Window pixels:       K²
Majority threshold:  (K² + 1) / 2
```

For each pixel, the module examines the corresponding `K × K` neighborhood. The output pixel is set to `1` when at least `(K² + 1) / 2` pixels in the window are `1`; otherwise, it is set to `0`.

This removes isolated mask noise while preserving regions where object-classified pixels form a local majority.

The module uses **line buffers and a sliding window** for continuous raster processing. After the initial raster-alignment delay, it supports **one output pixel for every valid input pixel**.

The filtered output is re-aligned to a complete frame raster so downstream blocks receive correctly positioned pixel coordinates and do not need to compensate for the filter offset.

### 4. `object_measurement.vhd`

Calculates the position, size, and center of the detected object from the cleaned binary mask.

As each valid pixel arrives, every pixel classified as part of the object contributes to four running measurements:

```text
pixel_count    Number of matching pixels

sum_x          Sum of the X coordinates of all matching pixels
sum_y          Sum of the Y coordinates of all matching pixels

min_x / max_x  Leftmost and rightmost matching-pixel coordinates
min_y / max_y  Topmost and bottommost matching-pixel coordinates
```

The coordinate limits define the object's bounding box:

```text
object_width  = max_x - min_x + 1
object_height = max_y - min_y + 1
```

The accumulated coordinate sums are used to calculate the object centroid:

```text
centroid_x = sum_x / pixel_count
centroid_y = sum_y / pixel_count
```

The centroid therefore represents the average position of all matching object pixels, rather than simply the geometric center of the bounding box.

The implementation separates the work into two clocked processes:

1. A frame accumulator updates `pixel_count`, `sum_x`, `sum_y`, `min_x`, `max_x`, `min_y`, and `max_y` continuously as matching pixels arrive.
2. At the end of the frame, the accumulated values are captured as a completed-frame snapshot, and two sequential dividers calculate `centroid_x` and `centroid_y`.

While centroid division is being performed for the completed frame, the accumulator can already begin processing pixels from the next frame.

### 5. `roi_tracker_update.vhd`

This block contains the main hardware tracking state machine.

At the end of each measured frame, the tracker first determines whether the new object measurement is reliable enough to use.

#### Measurement acceptance decision

The decision is based on the number of matching object pixels reported by `object_measurement.vhd`.

The tracker uses a dynamic minimum pixel requirement:

```text
min_accepted_pixels = max(
    COUNT_FLOOR,
    previous_object_width × previous_object_height / 2^COUNT_SHIFT
)
```

The threshold is recalculated after every accepted measurement, allowing it to scale with the tracked object's size.

| Parameter | Description |
|---|---|
| `COUNT_FLOOR` | Minimum number of matching pixels required for acceptance |
| `COUNT_SHIFT` | Required ratio between the number of matching pixels and the object's bounding-box area |

This allows the required pixel count to increase with object size while maintaining a minimum threshold for smaller objects.

For each completed frame:

```text
if matching_pixel_count >= min_accepted_pixels
    → ACCEPT

else
    → LOSS
```

#### <ins>**ACCEPT**</ins>
When the measurement contains enough matching pixels, it is accepted as the new object position.

The tracker then:

1. stores the new measured center and size,
2. calculates raw X/Y velocity from the previous accepted center,
3. limits the velocity using `VEL_MAX`,
4. smooths the velocity using the previous accepted velocity,
5. predicts the next object center,
6. increases ROI padding according to movement speed,
7. builds a new ROI around the predicted position,
8. publishes the completed tracking state.

**Velocity smoothing**

```text
smoothed_velocity =
    (raw_velocity + previous_smoothed_velocity) / 2
```

**Prediction**

```text
predicted_center =
    measured_center + smoothed_velocity
```

Normal tracking behavior is controlled by the following parameters:

| Parameter | Description |
|---|---|
| `VEL_MAX` | Maximum allowed velocity |
| `PAD_BASE` | Base ROI padding |
| `PAD_MAX` | Maximum ROI padding |
| `MAX_TRACK_ROI_W` | Maximum normal-tracking ROI width |
| `MAX_TRACK_ROI_H` | Maximum normal-tracking ROI height |

The ROI padding increases with the magnitude of the object's motion:

```text
padding = clamp(
    PAD_BASE + abs(smoothed_velocity) / 2,
    0,
    PAD_MAX
)
```

This keeps the ROI relatively tight when the object is moving slowly while providing additional search space when faster motion is detected.

#### <ins>**LOSS**</ins>
When the number of matching pixels falls below `min_accepted_pixels`, the measurement is not trusted and `tracking_lost` is asserted.

Because there is no reliable new object position, the tracker does not perform another full prediction from an untrusted measurement.

Instead, it expands the ROI around the previous predicted position.

The loss search is **direction-aware**: the last accepted velocity is used to provide more search space in the direction in which the object was previously moving.

The search region grows progressively when several consecutive frames are missed.

Loss recovery behavior is controlled by the following parameters:

| Parameter | Description |
|---|---|
| `LOSS_BASE` | Base expansion applied during loss |
| `LOSS_RAMP` | Additional expansion after consecutive missed frames |
| `LOSS_MIN` | Minimum expansion |
| `LOSS_MAX` | Maximum expansion |
| `MAX_LOSS_W` | Maximum loss-search ROI width |
| `MAX_LOSS_H` | Maximum loss-search ROI height |

This allows the search area to expand gradually when tracking becomes uncertain while keeping the ROI bounded.

> **Implementation note:** The tracking logic is implemented as a multi-stage registered state machine to maintain a short combinational path between registers.


### 6. `object_display.vhd`

Generates a hardware debug image with a black background and overlays the current tracking state:

- cleaned mask — white,
- ROI border — green,
- measured center — red `+`,
- predicted center — blue `X`.

The current display parameters are:

```text
ROI border thickness: 2 px
Center-marker radius: 4 px
```

### 7. `vision_tracker_logic_top.vhd`

Structurally connects the complete tracker pipeline and creates a coherent result snapshot for the PS.

All PS-facing tracking outputs are captured together only after `roi_tracker_update` has completed the frame update. A 32-bit snapshot counter is incremented whenever a new coherent result becomes available.

This allows software to determine whether a fresh result exists without reading a partially updated set of registers.

### 8. `vision_tracker_pkg.vhd`

`vision_tracker_pkg.vhd` is the central compile-time configuration package for resolution-dependent tracker parameters.

Current configuration:

```vhdl
constant FRAME_WIDTH  : positive := 640;
constant FRAME_HEIGHT : positive := 480;
constant VEL_MAX      : positive := 120;
```

Changing the frame dimensions in this package causes the required internal widths to be derived automatically at elaboration time, including:

- X/Y coordinate widths,
- ROI width/height widths,
- full-frame pixel-count width,
- X/Y centroid accumulator widths,
- signed predicted-center widths.

For example, the predicted-center width is automatically large enough to represent positions temporarily outside the visible frame by up to `VEL_MAX`.

This makes the hardware architecture reusable: the designer changes the central frame configuration and rebuilds the FPGA instead of manually updating width declarations throughout every VHDL module.

### Hardware tuning parameters

Algorithmic tuning values are also centralized at the integration boundary through the generic map of `roi_tracker_update`.

The current build uses:

```text
PAD_BASE       = 8
PAD_MAX        = 40
MAX_TRACK_ROI  = 200 × 200
COUNT_FLOOR    = 150
COUNT_SHIFT    = 3
LOSS_BASE      = 10
LOSS_RAMP      = 5
LOSS_MIN       = 5
LOSS_MAX       = 40
MAX_LOSS_ROI   = 240 × 240
```

These values control the physical behavior of the hardware tracker: how tightly it follows the target, how the acceptance threshold scales, how far it predicts, and how aggressively it expands the search region after a missed frame.

They are **compile-time hardware parameters**. Changing them means rebuilding the PL implementation, while runtime object-specific values such as the initial ROI and Cr/Cb thresholds are written by the PS through AXI-Lite.

The main edit points are:

- `PL/src/vision_tracker_pkg.vhd` — frame dimensions, `VEL_MAX`, and automatically derived widths.
- `PL/src/vision_tracker_logic_top.vhd` — the `roi_tracker_update` generic map used to tune padding, ROI limits, acceptance thresholds and loss behavior.

This separation gives the design two useful configuration levels:

| Configuration type | Examples | How it is changed |
|---|---|---|
| **Compile-time PL architecture / tuning** | frame size, `VEL_MAX`, padding, maximum ROI sizes, loss behavior | VHDL package / generics → rebuild bitstream |
| **Runtime object configuration** | tracking enable, initial ROI, Cr/Cb thresholds | PS → AXI-Lite registers |


---

## Why This Architecture Fits an FPGA

The tracker combines two workloads that map naturally to programmable logic:

- **Per-pixel streaming work** — color conversion, ROI tests, thresholding and mask filtering can operate continuously as pixels arrive.
- **Frame-level state updates** — object measurement, motion estimation, prediction and ROI updates are deterministic and can be implemented as registered state-machine stages.
- **Local feedback** — the ROI generated from frame N is already available in hardware for frame N+1, without a software round trip.

The PS is therefore reserved for tasks where software flexibility is more useful: camera interfacing, initialization, user interaction, diagnostics and optional high-level validation.

---




---

# Operating Modes

## 1. General Object Tracking

<p align="center">
  <img src="docs/general_object1.gif" width="48%">
  <img src="docs/general_object_2.gif" width="48%">
</p>

<p align="center"><i>Two examples of the general-purpose FPGA tracker following different user-selected objects.</i></p>

Directory:

```text
PS/GENERAL_OBJECT_TRACKING/
```

Main runtime:

```text
run_object_tracker_pynq.py
```

This is the cleanest demonstration of the hardware tracker.

### Flow

1. The PS displays the live USB-camera preview.
2. The user captures one frame.
3. The user draws an initial ROI around the target object.
4. The PS learns Cr/Cb limits from the central region of that selected object.
5. The PS writes the initial ROI and thresholds to the FPGA.
6. Tracking is enabled.
7. From this point, the PS continues to deliver video frames, but **the continuous tracking state is owned by the PL**.
8. For every new frame, the PL independently performs segmentation, filtering, object measurement, centroid calculation, velocity estimation, prediction, ROI movement/resizing and loss recovery.
9. The ROI calculated by the PL is automatically used for the following frame without a software tracking decision in between.

> **General-object mode is the clearest demonstration of the hardware architecture:** after initialization, software does not continuously calculate where the object is or where the next ROI should be.

The current initialization algorithm uses:

```text
Central ROI fraction:      0.72
Low percentile:            5%
High percentile:          95%
Cr/Cb margin:               6
Minimum color samples:     10
```

The purpose of the percentile limits is to reject color outliers before expanding the final Cr/Cb range by the configured margin.

### Initial ROI selection examples

The general-object mode begins with a one-time user selection of the target ROI. The PS then learns the initial Cr/Cb range from the selected object and seeds the hardware tracker.

<table>
  <tr>
    <td width="50%" align="center"><img src="docs/general_object_initial_roi_green_object.jpg" width="100%"></td>
    <td width="50%" align="center"><img src="docs/general_object_initial_roi_mug.jpg" width="100%"></td>
  </tr>
  <tr>
    <td align="center"><b>Initial ROI — Example 1</b></td>
    <td align="center"><b>Initial ROI — Example 2</b></td>
  </tr>
</table>

### Live PL tracking data

During operation, the notebook can display the current hardware state read from the tracker through AXI-Lite, including the published snapshot number, measured object center and size, pixel count, current ROI, predicted center and loss state.

<p align="center">
  <img src="docs/general_object_pl_tracking_data.jpg" width="90%">
</p>

---

## 2. Validated Hand Tracking

<p align="center">
  <img src="docs/hand.gif" width="58%">
</p>

<p align="center"><i>The same FPGA tracking engine combined with PS-side palm acquisition and periodic high-level validation.</i></p>

Directory:

```text
PS/VALIDATED_HAND_TRACKING/
```

Main runtime:

```text
run_tracker.py
```

This mode demonstrates how the same hardware tracker can be combined with a higher-level software detector.

The included `palm.onnx` model is used for **acquisition and validation**, not for continuous frame-by-frame tracking.

### Flow

```mermaid
flowchart LR
    A[Full-frame Palm Search on PS] --> B[Hand ROI + Cr/Cb Learning]
    B --> C[Seed PL Tracker]
    C --> D[Continuous FPGA Tracking]
    D --> E[Periodic / Urgent PS Validation]
    E -->|Hand valid inside PL ROI| D
    E -->|Validation failures| A
```

The PS:

1. finds a hand using `palm.onnx`,
2. derives an ROI and Cr/Cb thresholds,
3. seeds the PL tracker,
4. performs asynchronous validation approximately once per second,
5. requests urgent validation when a fresh PL snapshot reports `tracking_lost = 1`.

Healthy tracking is intentionally left alone: a successful validation does **not** continuously overwrite the PL ROI.

Palm inference runs in a background worker so it does not block the camera-to-video-to-PL path.

Current validation interval:

```text
1.0 second
```

### PS/PL validation status

The validated-hand runtime exposes the interaction between the software validator and the continuously running PL tracker. The status output below shows PS search/lock/validation events together with the current FPGA snapshot and tracking state.

<p align="center">
  <img src="docs/validated_hand_tracking_status.jpg" width="90%">
</p>

---

# Hardware Setup

## Required Hardware

- **PYNQ-Z2** development board
- **USB webcam**
- **Ethernet cable** between the PYNQ-Z2 and the computer
- **HDMI cable** for the video output
- **Monitor/display with HDMI input** for displaying the tracking result
- USB cable for the board's serial/UART connection
- Computer running Windows
- PYNQ-compatible SD card/image

## Required Software

On the computer:

- PuTTY or another serial terminal
- Web browser

On the PYNQ image / runtime:

- PYNQ Python environment
- OpenCV (`cv2`)
- NumPy
- Jupyter / IPython widgets used by the interactive interface

The validated-hand mode also uses OpenCV DNN to load the included `palm.onnx` model.

---

# Connecting to the PYNQ-Z2

The following procedure is intended for a direct Ethernet connection between a Windows computer and the PYNQ-Z2.

## Step 1 — Find the COM Port

1. Connect the PYNQ-Z2 to the computer through its serial/UART USB connection.
2. Press **Windows + X**.
3. Open **Device Manager**.
4. Open **Ports (COM & LPT)**.
5. Identify the COM port assigned to the PYNQ-Z2, for example:

```text
COM3
```

The exact COM number may be different on every computer.

## Step 2 — Open a Serial Session in PuTTY

Open PuTTY and configure:

```text
Connection type: Serial
Serial line:     COMx
Speed:           115200
```

Replace `COMx` with the COM port found in Device Manager.

Click **Open**.

Press **Enter** until the PYNQ terminal prompt appears, then run:

```bash
ifconfig eth0
```

Find the IPv4 address shown next to `inet`.

For example:

```text
inet 192.168.2.99
```

In this example the board's Ethernet address is:

```text
192.168.2.99
```

## Step 3 — Configure the Computer's Ethernet Address

Open the Windows settings for the Ethernet adapter and configure IPv4 manually.

The computer and PYNQ-Z2 must be on the **same subnet**, but they must **not use the same IP address**.

For example, if the board reports:

```text
PYNQ-Z2: 192.168.2.99
```

configure the computer as:

```text
IP address:  192.168.2.1
Subnet mask: 255.255.255.0
Gateway:     leave blank
DNS:         leave blank
```

So the direct connection becomes:

```text
Computer: 192.168.2.1
PYNQ-Z2:  192.168.2.99
```

If the board reports a different subnet, choose a free computer address in that same subnet instead.

## Step 4 — Open the PYNQ Web Interface

In a browser, open:

```text
http://pynq:9090
```

If prompted for the default PYNQ password:

```text
xilinx
```

If the hostname `pynq` does not resolve, use the board IP found with `ifconfig`, for example:

```text
http://192.168.2.99:9090
```

---

# Running the Project on the Board

## Prepare the Overlay Files

Both runtime scripts expect the matching `.bit` and `.hwh` files to be **in the same directory as the Python runtime**.

The repository stores the master copies under:

```text
PL/design_1.bit
PL/design_1.hwh
```

Before running a mode, place both files next to that mode's script.

### General Object Tracking directory should contain

```text
PS/GENERAL_OBJECT_TRACKING/
├── fpga.py
├── run_object_tracker_pynq.py
├── design_1.bit
└── design_1.hwh
```

### Validated Hand Tracking directory should contain

```text
PS/VALIDATED_HAND_TRACKING/
├── fpga.py
├── mp_palmdet.py
├── palm.onnx
├── run_tracker.py
├── design_1.bit
└── design_1.hwh
```

The `.bit` and `.hwh` files must be the matching pair generated from the same Vivado design.

---

## Run General Object Tracking

Open a Jupyter notebook in:

```text
PS/GENERAL_OBJECT_TRACKING/
```

Run:

```python
%matplotlib notebook
from run_object_tracker_pynq import PYNQObjectTracker

tracker = PYNQObjectTracker()
tracker.run_interactive()
```

### User flow

1. Wait for the live camera preview.
2. Click **CAPTURE FRAME**.
3. Draw a rectangle around the object to track.
4. Confirm the ROI and start tracking.
5. Observe the live PL status and hardware debug output.
6. Use **STOP & CLOSE** for a clean shutdown.

Once tracking has started, the continuous tracker is running in the PL.

---

## Run Validated Hand Tracking

Open a Jupyter notebook in:

```text
PS/VALIDATED_HAND_TRACKING/
```

Run:

```python
%run run_tracker.py
```

The runtime automatically starts full-frame hand search. After a valid hand is acquired, it learns the hand's chrominance limits, seeds the FPGA tracker and enters continuous PL tracking with periodic PS validation.

---

# Hardware-Matched PC Simulation

The repository also contains:

```text
full_system_simulation.py
```

The simulator reproduces the current tracking algorithm at frame level and is intended for algorithm tuning before committing new constants to VHDL and rebuilding the FPGA design.

It models:

```text
rgb_to_ycrcb
      ↓
roi_mask_generator
      ↓
mask_cleaner
      ↓
object_measurement
      ↓
roi_tracker_update
      ↓
object_display
```

The default `HWConfig` values match the current VHDL top-level configuration.

### Run with a webcam

```bash
python full_system_simulation.py
```

or choose another camera:

```bash
python full_system_simulation.py --camera 1
```

### Run with a recorded video

```bash
python full_system_simulation.py --video path/to/video.mp4
```

### Controls

```text
S       Select / reselect object
R       Reset tracking
Q / ESC Quit
```

The simulator opens two useful windows:

- **SIM DEBUG** — original camera image plus diagnostic information.
- **REAL SYSTEM - exact PL HDMI** — software rendering of the expected hardware debug output.

> **TODO — Add simulation screenshot here**  
> Suggested file: `docs/hardware_matched_simulation.png`

This simulation was used as a development tool for changing tracking parameters quickly before transferring the selected values back into the VHDL implementation.

---

# Current Hardware Configuration

| Parameter | Current value |
|---|---:|
| Frame width | 640 px |
| Frame height | 480 px |
| Mask-cleaner kernel | 5 × 5 |
| Majority threshold | 13 / 25 |
| Velocity limit (`VEL_MAX`) | 120 px/frame |
| Normal ROI base padding | 8 px |
| Normal ROI maximum padding | 40 px |
| Maximum normal-tracking ROI | 200 × 200 px |
| Pixel-count floor | 150 |
| Pixel-count area divisor | 8 |
| Loss base expansion | 10 px |
| Loss ramp | 5 px/update |
| Loss expansion clamp | 5–40 px |
| Maximum loss ROI | 240 × 240 px |
| Debug ROI border | 2 px |
| Center-marker radius | 4 px |

## Changing the Hardware Configuration

The tracker was designed so that important hardware parameters can be changed without rewriting the complete design.

### Resolution and width-dependent parameters

Edit:

```text
PL/src/vision_tracker_pkg.vhd
```

For example:

```vhdl
constant FRAME_WIDTH  : positive := 640;
constant FRAME_HEIGHT : positive := 480;
constant VEL_MAX      : positive := 120;
```

The package automatically derives the dependent coordinate, size, pixel-count, accumulator and prediction widths.

### Tracking-behavior parameters

The `roi_tracker_update` generic map in `vision_tracker_logic_top.vhd` controls values such as:

- normal ROI padding,
- maximum normal ROI size,
- minimum accepted-pixel floor,
- area scaling of the acceptance threshold,
- loss expansion base/ramp,
- maximum loss expansion,
- maximum loss ROI dimensions.

This makes it possible to tune the behavior for a different application and then synthesize a new hardware implementation.

### Runtime parameters

Object-specific values do not require a new bitstream. The PS can write:

- tracking enable,
- initial ROI X/Y/W/H,
- Cr minimum/maximum,
- Cb minimum/maximum.

After these values seed the tracker, the evolving center, prediction and ROI are once again generated by the PL.

---


# FPGA Implementation Results

The final Vivado implementation confirms that the complete system — including the PYNQ video infrastructure and the custom hardware tracker — fits comfortably in the Zynq-7000 device and meets the specified timing constraints.

## Operating Frequency

The custom `vision_tracker_ip` operates in a dedicated **100 MHz FPGA clock domain**.

The surrounding video design also contains a **142 MHz** clock domain, so AXI4-Stream clock converters are used before and after the custom tracker to cross between the video and tracking domains safely.

## Timing Closure

Post-implementation Vivado analysis confirmed that **all user-specified timing constraints are met**, with no failing setup or hold endpoints.

Timing closure was treated as an architectural requirement during the VHDL design rather than as a final cleanup step. Several computationally heavier operations were deliberately split across registered stages so that the tracker would not depend on long combinational paths.

For example, the accepted-frame update in `roi_tracker_update` is pipelined across separate stages for:

```text
Velocity calculation
        ↓
Prediction
        ↓
ROI sizing
        ↓
State / ROI commit
```

The loss-recovery path is also split into separate expansion and commit stages.

In `object_measurement`, frame accumulation is separated from centroid division. After one frame has been accumulated, the X and Y centroid calculations proceed using sequential dividers while the next frame can already begin accumulating.

Together with the dedicated **100 MHz tracking clock domain** and the AXI4-Stream clock-domain converters surrounding the tracker, these design choices allowed the implemented system to meet its timing requirements.

## Full-System FPGA Resource Utilization

The resource figures below represent the **complete implemented PYNQ-Z2 design**, not only `vision_tracker_ip`.

They include the video/HDMI infrastructure, AXI interconnects, clocking logic, VDMA-related logic, control infrastructure and the custom hardware tracker.

<p align="center">
  <img src="docs/full_system_fpga_utilization.png" width="70%">
</p>

Despite including the complete video and control infrastructure, the implemented design uses less than one quarter of the available LUT and FF resources and less than 10% of the available BRAM and DSP resources.

This leaves substantial programmable-logic capacity for future extensions such as additional image features, multiple-object support or further hardware acceleration.

---

# Repository Structure

```text
pynq_z2_object_tracking_system/
│
├── PL/
│   ├── design_1.bit
│   ├── design_1.hwh
│   │
│   ├── hdl/
│   │   ├── vision_tracker_ip.vhd
│   │   └── vision_tracker_ip_slave_lite_v1_0_S_AXI_TRACK.vhd
│   │
│   └── src/
│       ├── mask_cleaner.vhd
│       ├── object_display.vhd
│       ├── object_measurement.vhd
│       ├── rgb_to_ycrcb.vhd
│       ├── roi_mask_generator.vhd
│       ├── roi_tracker_update.vhd
│       ├── vision_tracker_logic_top.vhd
│       └── vision_tracker_pkg.vhd
│
├── PS/
│   ├── GENERAL_OBJECT_TRACKING/
│   │   ├── fpga.py
│   │   └── run_object_tracker_pynq.py
│   │
│   └── VALIDATED_HAND_TRACKING/
│       ├── fpga.py
│       ├── mp_palmdet.py
│       ├── palm.onnx
│       └── run_tracker.py
│
├── docs/
│   ├── general_object1.gif
│   ├── general_object_2.gif
│   ├── hand.gif
│   ├── general_object_initial_roi_green_object.jpg
│   ├── general_object_initial_roi_mug.jpg
│   ├── general_object_pl_tracking_data.jpg
│   ├── validated_hand_tracking_status.jpg
│   ├── hdmi_tracker_integration.png
│   └── full_system_fpga_utilization.png
│
├── full_system_simulation.py
└── README.md
```

---

# Limitations

The current tracker is intentionally lightweight and hardware-oriented. Its main limitations are:

- The target is represented primarily by **Cr/Cb chrominance ranges**, so a visually similar object entering the active ROI may also match.
- Large illumination or appearance changes can make the original learned threshold range less representative.
- Normal tracking and loss-recovery ROI sizes are intentionally bounded.
- The current implementation tracks one target at a time.
- General-object initialization still requires the user to select the first ROI.
- In validated-hand mode, palm acquisition/validation is a PS-side neural-network operation rather than an FPGA accelerator.
- USB-camera capture is handled by the PS; the project accelerates the tracking pipeline rather than implementing a direct sensor-to-FPGA camera interface.

---

# Future Work

Possible extensions include:

- automatic threshold adaptation while the object appearance changes,
- multiple-object tracking,
- additional hardware features beyond chrominance,
- direct camera-to-PL streaming,
- higher resolutions and additional video formats,
- deeper pipelining / resource optimization after timing analysis,
- FPGA acceleration of the acquisition/validation neural network,
- improved confidence scoring for ambiguous object masks,
- runtime-adjustable tracker parameters through additional AXI-Lite registers.

---

# Remaining Documentation Media

The following engineering visuals can still be added before the final public release:

1. **Physical hardware setup photo** — PYNQ-Z2, USB camera, Ethernet, HDMI cable and display.
2. **FPGA debug-output screenshot** — white mask, green ROI, red measured center and blue predicted center.
3. **Full Vivado Block Design screenshot** — top-level Zynq PS, VDMA/video hierarchy, AXI control and HDMI subsystem.
4. **Polished architecture diagram** — PS/PL partitioning and video/control data paths.
5. **Polished FPGA pipeline diagram** — the VHDL processing stages.
6. **Hardware-matched PC simulation screenshot**.

Recommended filenames:

```text
docs/
├── hardware_setup.jpg
├── hdmi_tracking_output.png
├── vivado_block_design.png
├── system_architecture.png
├── fpga_pipeline.png
└── hardware_matched_simulation.png
```

## Summary

This project implements a **stateful, autonomous FPGA object tracker** on the PYNQ-Z2. Software handles camera interfacing, initialization and optional high-level validation, while the continuous frame-by-frame tracking feedback loop remains in programmable logic.

The result is a hardware-centric vision platform that combines a reusable VHDL tracking pipeline, AXI-based PS/PL integration, HDMI output, configurable tracking behavior and verified post-implementation timing/resource results.
