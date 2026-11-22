# YOLO11n Cone Detection Model

## Model Files

| File | Description |
|------|-------------|
| `best.pt` | Trained Ultralytics YOLO11n PyTorch weights |
| `best.onnx` | Standard ONNX model for host-side inference |
| `best.blob` | OpenVINO blob for OAK-D (RVC2) on-device inference |
| `test_v3_live_oak.py` | Live DepthAI camera detection test |

## Model Information

- Architecture: YOLO11n (object detection)
- Training: From scratch, 250 epochs
- Dataset: 452 labeled images
- Input size: 416 x 416
- Number of classes: 4
- Validation mAP50: 0.949

### Class Order (Important)

The exact model class indices are:

| ID | Class |
|----|-------|
| 0 | blue cone |
| 1 | pink cone |
| 2 | red cone |
| 3 | yellow cone |

Note: These IDs are NOT the same as the ROS2 `Cone.color` constants.

## Standard ONNX Export

The `best.onnx` file was exported directly from `best.pt` using Ultralytics 8.4.215.

```python
from ultralytics import YOLO

model = YOLO("best.pt")
model.export(
    format="onnx",
    imgsz=416,
    batch=1,
    opset=12,
    dynamic=False,
    simplify=False,
    nms=False,
    device="cpu",
)
```

Verified ONNX properties:

- Opset: 12
- Input: `images`, shape `[1, 3, 416, 416]`
- Output: `output0`, shape `[1, 8, 3549]`
- Output contains raw predictions; preprocessing, decoding and NMS are handled by the inference application.
- The ONNX model passed `onnx.checker.check_model`. ONNX Runtime inference on the Raspberry Pi has not yet been benchmarked.

## OAK-D Blob

The `best.blob` file was produced using the Luxonis YOLO Conversion Tool.

- Target: OAK-D RVC2 / Myriad X
- OpenVINO version: 2022.1
- SHAVEs: 6
- Input resolution: 416 x 416
- Model family: YOLO11
- Classes: 4

The blob was tested on a separate OAK-D-W camera using DepthAI 3.12.2 on a Windows laptop. Neural network inference runs on the OAK-D chip.

## Live Camera Test

Install the required Python packages:

```bash
pip install depthai==3.12.2 opencv-python
```

Connect a compatible OAK-D camera and run:

```bash
python test_v3_live_oak.py
```

The test script:
- Loads `best.blob` from the same directory.
- Displays live camera frames with detected cones and confidence scores.
- Press `S` to save a screenshot.
- Press `Q` to quit.
- Saves screenshots in `field_tests_v3/`.

The test requests a 5 FPS camera stream; actual inference throughput was not independently benchmarked.

## ROS2 Integration

The ROS2 AI detector should publish `slalom_interfaces/ConeArray` on `/cones/ai`.

YOLO class ID to ROS2 cone color mapping:

| YOLO ID | ROS2 color |
|---------|------------|
| 0 blue | 4 (BLUE) |
| 1 pink | 2 (PINK) |
| 2 red | 3 (RED) |
| 3 yellow | 0 (YELLOW) |

ROS2 cone positions `x` and `y` must be expressed in meters in the vehicle coordinate frame, not pixel coordinates.