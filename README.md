<img width="1456" height="819" alt="image" src="https://github.com/user-attachments/assets/ca04455a-648c-4ffd-90da-a95f24741eef" />


# BOTICS ALPHA — AI Vision Pipeline

On-device AI vision pipeline for the BOTICS Alpha robotics competition:
camera → **Grove Vision AI V2** (on-device NPU inference) → serial bridge →
**Arduino Mega 2560** → coordinate mapping → robotic arm pick/sort/place.

```
OV5647 CSI camera → Grove Vision AI V2 (Ethos-U55 NPU, on-device inference)
    → XIAO (onboard header, UART) → Arduino Mega 2560
    → coordinate mapping → robotic arm → pick / sort / place
```

The PC/Python code in this repo (`python/`) is for **debugging, calibration,
and visualization only** — it is not part of the competition-time data path.
At competition time, inference runs entirely on GV2's Ethos-U55 NPU and
results are relayed to the Mega by the XIAO board seated in GV2's expansion
header, with no PC involved.

**Status: hardware-confirmed.** The current model
(`models_v2_yolov8n/best_full_integer_quant_vela.tflite`) has been flashed to
a real GV2 board and detects all 4 classes simultaneously at 89–97%
confidence, with bounding boxes and center points verified pixel-accurate in
the PC-side visualizer. See [Current status](#current-status) below.

---

## Table of contents

- [Classes](#classes-fixed--do-not-reorder-or-rename)
- [Repository layout](#repository-layout)
- [Quick start](#quick-start)
- [Project history: why the model changed](#project-history-why-the-model-changed)
- [Model details (v2, current)](#model-details-v2-current)
- [Hardware connections](#hardware-connections)
- [Serial / communication protocol](#serial--communication-protocol)
- [PC-side visualization: coordinate mapping notes](#pc-side-visualization-coordinate-mapping-notes)
- [Troubleshooting](#troubleshooting)
- [Current status](#current-status)
- [Remaining work](#remaining-work)
- [Requirements](#requirements)

---

## Classes (fixed — do not reorder or rename)

```
0  red_patient
1  yellow_patient
2  green_patient
3  sample
```

## Repository layout

```
models_v2_yolov8n/   CURRENT / RECOMMENDED model — YOLOv8n, imgsz=192.
                      best.pt                                (PyTorch weights)
                      best_full_integer_quant_vela.tflite     <-- FLASH THIS ONE
                      best_full_integer_quant_pre_vela.tflite (pre-Vela, for debugging)
                      labels.txt, metadata.json, vela_compile_summary.csv,
                      validation_confidence_report.json
models/              DEPRECATED — original YOLO26 model, kept for reference
                      only. Root-caused as unsupported by SenseCraft/GV2
                      firmware. Do not deploy. See project history below.
dataset/             dataset_report.md (v1 findings) + yolo_dataset_v2/
                      (the real labeled dataset used to train the v2 model)
python/              gv2_serial_test.py   — low-level AT-command probe (model/info/invoke)
                      gv2_detect.py        — live detection + OpenCV visualization
                      calibration.py       — pixel-to-robot homography calibration
                      gv2_to_mega_bridge.py — PC stand-in for the XIAO bridge (dev/test)
arduino/             gv2_to_mega/gv2_to_mega.ino       (Mega: parses bridge line protocol)
                      xiao_gv2_bridge/xiao_gv2_bridge.ino (XIAO: GV2 -> Mega relay, competition setup)
tools/               inspect_model.py, validate_tflite.py, validate_dataset.py,
                      build_dataset_v2.py, compare_tflite_vs_pt_v2.py
calibration/         calibration_points.json (template — fill via python/calibration.py)
docs/                YOLOV8N_PIPELINE.md   — full v2 model story (start here)
                      DEPLOYMENT.md         — flashing / running steps
                      HARDWARE_CONNECTION.md — wiring, ports, baud rates
                      SERIAL_PROTOCOL.md    — AT protocol + bridge line format
                      TROUBLESHOOTING.md    — ranked, evidence-based `boxes: []` checklist
requirements.txt
```

## Quick start

1. **Flash the model.** Upload `models_v2_yolov8n/best_full_integer_quant_vela.tflite`
   to Grove Vision AI V2 via [SenseCraft AI](https://sensecraft.seeed.cc),
   with the 4 class labels above in that exact order. Full steps in
   [`docs/DEPLOYMENT.md`](docs/DEPLOYMENT.md).
2. **Verify the flash took:**
   ```bash
   python python/gv2_serial_test.py --port COM13 --baud 921600 --cmd model
   ```
   `"size"` should be a real non-zero byte count. `0` means re-flash.
3. **Run live detection with visualization:**
   ```bash
   python python/gv2_detect.py --port COM13 --baud 921600 --threshold 0.5
   ```
4. **Calibrate and bridge to the Mega** once detections look correct — see
   [Remaining work](#remaining-work) and [`docs/DEPLOYMENT.md`](docs/DEPLOYMENT.md) §6.

## Project history: why the model changed

### Phase 1 — v1 (YOLO26) failed on real hardware

The original model (`models/best.pt`, `models/best_full_integer_quant_vela.tflite`)
was a **YOLO26n** model. It compiled cleanly with Vela and ran on GV2's
Ethos-U55 NPU at ~107 ms/inference with 100% NPU offload — but returned
`boxes: []` on **every single invocation**, even at `confidence threshold = 1`
across 60+ consecutive frames.

Root cause, confirmed on Seeed's own community forum (not guessed): **Seeed
staff stated SenseCraft firmware does not currently support YOLO26 model
metadata.** The model's math was fine — the firmware simply can't parse a
YOLO26 output into detections. This is a supported-architecture problem, not
an export or dataset defect.

### Phase 2 — v2 (YOLOv8n) rebuild

Seeed's own official GV2 deployment docs document **YOLOv8n** as a supported
architecture, with a specific documented pipeline:

```bash
yolo train detect model=yolov8n.pt data=./data.yaml imgsz=192
yolo export model=best.pt format=tflite imgsz=192 int8
vela --accelerator-config ethos-u55-64 <exported>.tflite
```

Note **imgsz=192**, not 224 — a deliberate difference from the v1 model.

**Dataset recovery.** The originally-uploaded `all_400_images_for_labeling.zip`
had zero labels. A later upload, `Seperate_Images.zip` (4 separately-trained
SenseCraft single-class models), was found to contain real pixel-space
bounding-box annotations (`annotations.json`, one per class folder) collected
during the user's own SenseCraft training workflow. `tools/build_dataset_v2.py`
converts these into a combined 4-class YOLO dataset: 400 images (100/class),
340 train / 60 val. See [`dataset/dataset_report.md`](dataset/dataset_report.md)
for the full v1 dataset findings and known generalization caveat (training
images are single-object/plain-background, not the cluttered multi-object
competition scene).

**Training.** 80/80 epochs, `imgsz=192, batch=16, patience=25`. Final
validation: precision 0.993, recall 1.0, mAP50 0.995, mAP50-95 0.964.

**Export toolchain pitfall (found and fixed).** `ultralytics>=8.4.83`'s
default TFLite export backend (`litert_torch`) produces **NCHW** input layout
and **float32-wrapped** I/O — incompatible with a clean Vela compile (it
reproduces `DEQUANTIZE`/`QUANTIZE` CPU-fallback warnings). Fixed by pinning
`ultralytics==8.3.203` in an isolated virtualenv, which uses the classic
TensorFlow SavedModel → `TFLiteConverter` path and produces clean **NHWC
INT8** I/O matching Seeed's documented format.

**Vela compile result:** 0 CPU-fallback ops, 261/261 NPU ops (100%), no
DEQUANTIZE/QUANTIZE warnings — a fully NPU-offloaded compile.

**Offline validation (pre-hardware):**
- Structural: `tools/validate_tflite.py --imgsz 192` — 7/7 checks PASS.
- Confidence test: all 60 held-out validation images run through both
  PyTorch (ground truth) and the pre-Vela INT8 TFLite (the exact arithmetic
  Vela compiles 1:1). **60/60 correct top-class agreement**, mean confidence
  0.965 (PyTorch) / 0.725 (INT8), **0/60 zero-detections** — a direct,
  evidence-based contrast with v1's persistent `boxes: []`.

Full narrative: [`docs/YOLOV8N_PIPELINE.md`](docs/YOLOV8N_PIPELINE.md).

### Phase 3 — hardware deployment: confirmed working

`best_full_integer_quant_vela.tflite` was flashed to a real GV2 board via
SenseCraft. The live SenseCraft preview and device logger confirmed **all 4
classes detecting simultaneously at 89–97% confidence** — the original
`boxes: []` problem is fully resolved and hardware-confirmed, not just
offline-validated.

### Phase 4 — PC-side visualization and coordinate-mapping fixes

With hardware detection confirmed, `python/gv2_detect.py` (used for
debugging/visualization) initially drew bounding boxes offset from the real
object. Two distinct bugs were root-caused and fixed, evidence-first (a
temporary frame-dimension debug print was added to get real diagnostic data
before the second fix, rather than guessing):

1. **Scale mismatch** — GV2's `AT+INVOKE` box coordinates are in the model's
   192×192 input space, but the preview JPEG is a different (square, 240×240)
   resolution. Fixed with `scale_x = frame.shape[1] / 192`,
   `scale_y = frame.shape[0] / 192`, computed fresh per frame and applied to
   `x, y, w, h`.
2. **Center vs. corner convention** — even after scaling, boxes stayed offset
   right+down by about half their own size. Root cause: GV2 reports `(x, y)`
   as the box **center** (standard YOLO `cx, cy, w, h`), not the top-left
   corner the code originally assumed. Fixed via a `BOX_XY_IS_CENTER` toggle
   in `gv2_detect.py`.

Both fixes were confirmed visually correct against real hardware for
`red_patient` and `yellow_patient` (box tight on the object, center dot
exactly centered).

## Model details (v2, current)

| | |
|---|---|
| Architecture | YOLOv8n (Ultralytics 8.3.203, classic SavedModel→TFLiteConverter export) |
| Input | `[1, 192, 192, 3]` NHWC, INT8, scale `0.003875432536005974`, zero_point `-128` |
| Output | `[1, 8, 756]` INT8, scale `0.011811849661171436`, zero_point `-128` — 8 = 4 box coords + 4 class scores; 756 = anchor count at strides 8/16/32 for imgsz=192 (24²+12²+6²) |
| Vela compile | `--accelerator-config ethos-u55-64` → 0 CPU-fallback ops, 261/261 NPU ops (100%), no DEQUANTIZE/QUANTIZE warnings |
| SRAM / Flash | 818.02 KiB / 2852.47 KiB |
| Training | 80 epochs, imgsz=192, batch=16, patience=25, CPU |
| Val metrics | precision 0.993, recall 1.0, mAP50 0.995, mAP50-95 0.964 |
| Offline confidence test | 60/60 held-out images correct top-class (PyTorch + pre-Vela INT8 agree), 0/60 zero-detections |
| Hardware validation | **Confirmed** — all 4 classes detect simultaneously on real GV2, 89–97% confidence |

Full machine-readable metadata: [`models_v2_yolov8n/metadata.json`](models_v2_yolov8n/metadata.json).

**Known limitation (carried over from v1, not new):** training images are
single-object, plain-background SenseCraft captures — not the cluttered,
multi-object competition mat. Strong validation metrics reflect performance
on data resembling the training distribution; real-arena generalization
should be spot-checked once the full scene is available.

## Hardware connections

| Device | Role |
|---|---|
| OV5647 CSI camera | Mounted on GV2 |
| Grove Vision AI V2 | On-device inference (Ethos-U55 NPU) |
| XIAO (GV2 onboard header) | Relays detections to the Mega over UART via `Seeed_Arduino_SSCMA` |
| Arduino Mega 2560 | Final robot controller — receives detections, drives arm/motors |
| PC | Debug / calibration / visualization only — not in the competition-time loop |

- **Camera → GV2**: CSI ribbon connector; confirm FPC orientation.
- **GV2 → PC (debug link)**: USB-C, e.g. `COM13` @ 921600 baud. Used by
  `gv2_serial_test.py`, `gv2_detect.py`, `gv2_to_mega_bridge.py`.
- **GV2 → XIAO**: onboard expansion header, no external wiring —
  `HardwareSerial atSerial(0)` + `Seeed_Arduino_SSCMA`.
- **XIAO → Mega (competition data path)**: second UART, wired externally,
  common GND required. Mega side: `Serial1` (RX1=19, TX1=18) — do **not** use
  `SoftwareSerial`. XIAO side: `MEGA_TX_PIN`/`MEGA_RX_PIN` constants at the
  top of `xiao_gv2_bridge.ino` — confirm against your exact XIAO pinout
  before wiring.
- **Mega → arm**: not yet specified in this repo — `gv2_to_mega.ino` parses
  and prints the mapped detection, and marks exactly where motor/servo calls
  go (`routeDetection()`).
- **Power**: tie all grounds together across GV2/camera/Mega supplies.

Full detail: [`docs/HARDWARE_CONNECTION.md`](docs/HARDWARE_CONNECTION.md).

## Serial / communication protocol

**GV2 ↔ PC** — SSCMA-Micro AT protocol
([spec](https://github.com/Seeed-Studio/SSCMA-Micro/blob/1.0.x/docs/protocol/at_protocol.md)):

```
AT+INVOKE=1,0,1        # one inference, result only (no image, lower latency)
AT+INVOKE=1,0,0        # one inference, includes image (for OpenCV preview)
```

Event response:
```json
{"type":1,"data":{"count":1,"perf":[8,365,0],"boxes":[[87,83,77,65,70,0]]}}
```
`boxes`: `[x, y, w, h, score, target_id]` — `x,y` is the box **center** in
192×192 model space, `score` is 0–100, `target_id` maps to the fixed class
order above.

**GV2 ↔ XIAO** — Seeed's `Seeed_Arduino_SSCMA` Arduino library
(`AI.invoke(1, false, true)` ≡ `AT+INVOKE=1,0,1`).

**XIAO/PC-bridge → Mega** — fixed ASCII line, emitted identically by
`xiao_gv2_bridge.ino` (competition) or `gv2_to_mega_bridge.py` (dev/test):

```
D,<class_id>,<confidence_x100>,<x>,<y>,<w>,<h>,<cx_x10>,<cy_x10>\n
```

Example: `D,0,94,87,83,77,65,1255,1155\n` → `red_patient`, confidence 0.94,
box `(87,83,77,65)`, center `(125.5, 115.5)`.

Full detail: [`docs/SERIAL_PROTOCOL.md`](docs/SERIAL_PROTOCOL.md).

## PC-side visualization: coordinate mapping notes

`python/gv2_detect.py` is the debug/visualization script (not part of the
competition data path). Two constants at the top control the coordinate
mapping described in [Phase 4](#phase-4--pc-side-visualization-and-coordinate-mapping-fixes) above:

```python
MODEL_INPUT_SIZE = 192      # model's coordinate space
BOX_XY_IS_CENTER = True     # GV2 reports box center, not top-left corner
```

`to_detections()` computes `scale_x`/`scale_y` fresh from the decoded
frame's actual dimensions each call, then converts center-based
`(cx, cy, w, h)` to a top-left-corner box for drawing while keeping the
center for the on-screen marker. A temporary `debug_frame_shape()` print
(called right after decoding each frame) was added while diagnosing the
mapping — safe to remove once the mapping has been spot-checked across all
4 classes.

## Troubleshooting

Work through [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) **in
order** if `boxes: []` reappears:

1. Is a valid model actually resident on GV2? (`gv2_serial_test.py --cmd model`, check `"size" != 0`)
2. Does the model file validate structurally? (`tools/validate_tflite.py`)
3. Is the camera actually delivering an image? (`gv2_detect.py --once`, check for image decode)
4. Model + camera OK but still empty — check firmware confidence threshold,
   model "algorithm type" tag, SSCMA firmware version vs. architecture support.
5. Getting boxes but multi-object detection is unreliable — likely a dataset
   generalization issue (see [`dataset/dataset_report.md`](dataset/dataset_report.md)), not a pipeline bug.

## Current status

| Step | Status |
|---|---|
| v2 model export, Vela compile, offline validation | **Done** |
| Model flashed to real GV2 | **Done** |
| GV2 returns non-empty boxes on real camera | **Confirmed** — all 4 classes, 89–97% confidence |
| PC-side visualization coordinate mapping (scale + center/corner) | **Fixed and confirmed** (red_patient, yellow_patient) |
| green_patient / sample visual spot-check | Pending |
| Pixel-to-robot calibration (`calibration.py`) | Not yet run with real points |
| XIAO ↔ Mega hardware bridge | Code delivered, not hardware-tested; `MEGA_TX_PIN`/`MEGA_RX_PIN` pin assumptions unverified |
| Arm/motor control logic (`routeDetection()`) | Not implemented — awaiting arm kinematics/driver specs |
| Full end-to-end field test (real competition scene) | Not yet run |

## Remaining work

1. Spot-check `green_patient` and `sample` with the corrected `gv2_detect.py`.
2. Run `python python/calibration.py collect` then `fit` with real measured
   points to build `calibration/calibration_points.json` / `homography.json`
   — do not hard-code coordinates.
3. Bridge GV2 → Mega for development:
   `python python/gv2_to_mega_bridge.py --gv2-port COM13 --mega-port COM7`.
4. Flash `arduino/gv2_to_mega/gv2_to_mega.ino` to the Mega and confirm parsed
   detections print over USB Serial.
5. Wire in arm/motor control inside `routeDetection()` once calibration is
   validated.
6. Full scene test: red_patient + yellow_patient + green_patient + sample all
   in frame simultaneously, confirming 4 correct simultaneous detections —
   this is the real bar for "done," beyond individual-class checks.
7. Remove the temporary `debug_frame_shape()` diagnostic from `gv2_detect.py`
   once the mapping has been confirmed stable across all classes.

## Requirements

```
# PC-side tooling only — GV2 itself needs no Python, it runs on-device.
pyserial>=3.5
opencv-python>=4.8
numpy>=1.24
Pillow>=10.0

# Only needed if you re-touch best.pt (retrain/re-export/re-run Vela):
tensorflow>=2.15
ultralytics==8.3.203   # pinned — see "Export toolchain pitfall" above
ethos-u-vela>=4.0
```

See [`requirements.txt`](requirements.txt).
