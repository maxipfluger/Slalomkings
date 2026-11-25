import time
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai

ROOT = Path(__file__).resolve().parent
BLOB = ROOT / "best.blob"
SAVE_DIR = ROOT / "field_tests_v3"
SAVE_DIR.mkdir(exist_ok=True)

LABELS = ["Blue", "Pink", "Red", "Yellow"]
COLORS = [
    (255, 0, 0),
    (200, 100, 255),
    (0, 0, 255),
    (0, 255, 255),
]

print("=== YOLO11n V3 LIVE FIELD TEST ===", flush=True)
print("Press S to save screenshot, Q to quit.", flush=True)

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()
    cam_out = cam.requestOutput(
        (416, 416),
        dai.ImgFrame.Type.BGR888p,
        fps=5
    )

    frame_queue = cam_out.createOutputQueue(
        maxSize=4, blocking=False
    )

    nn = pipeline.create(dai.node.NeuralNetwork)
    nn.setBlobPath(str(BLOB))
    nn.input.setBlocking(False)
    cam_out.link(nn.input)

    parser = pipeline.create(dai.node.DetectionParser)
    parser.setBlobPath(str(BLOB))
    parser.setNNFamily(dai.DetectionNetworkType.YOLO)
    parser.setSubtype("yolov11")
    parser.setClasses([
        "blue cone", "pink cone",
        "red cone", "yellow cone"
    ])
    parser.setCoordinateSize(4)
    parser.setConfidenceThreshold(0.35)
    parser.setIouThreshold(0.5)
    nn.out.link(parser.input)

    detection_queue = parser.out.createOutputQueue(
        maxSize=4, blocking=False
    )

    pipeline.start()
    print("Camera and YOLO running.", flush=True)

    frame = None
    detections = []
    last_detection_time = 0
    total_detection_frames = 0
    frames_with_cones = 0
    start = time.monotonic()

    while time.monotonic() - start < 180:
        frame_msg = frame_queue.tryGet()
        det_msg = detection_queue.tryGet()

        if frame_msg is not None:
            frame = frame_msg.getCvFrame().copy()

        if det_msg is not None:
            detections = list(det_msg.detections)
            last_detection_time = time.monotonic()
            total_detection_frames += 1
            if detections:
                frames_with_cones += 1

        if frame is None:
            time.sleep(0.02)
            continue

        display = frame.copy()
        height, width = display.shape[:2]

        # Hide stale detections.
        active = (
            detections
            if time.monotonic() - last_detection_time < 0.6
            else []
        )

        for d in active:
            cls = int(d.label)
            if not 0 <= cls < len(LABELS):
                continue

            x1 = max(0, min(width-1, int(d.xmin * width)))
            y1 = max(0, min(height-1, int(d.ymin * height)))
            x2 = max(0, min(width-1, int(d.xmax * width)))
            y2 = max(0, min(height-1, int(d.ymax * height)))

            color = COLORS[cls]
            cv2.rectangle(display, (x1,y1), (x2,y2), color, 2)

            label = f"{LABELS[cls]} {d.confidence:.2f}"
            cv2.putText(
                display, label,
                (x1, max(18, y1-7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48, color, 2
            )

        cv2.putText(
            display,
            f"OAK YOLO11n | Detections: {len(active)}",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (255,255,255), 2
        )

        cv2.imshow("ECE148 - YOLO11n V3 OAK Field Test", display)

        key = cv2.waitKey(20) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = SAVE_DIR / f"oak_test_{name}.jpg"
            cv2.imwrite(str(path), display)
            print("Saved screenshot:", path, flush=True)

    cv2.destroyAllWindows()

    print("\n=== FIELD TEST SUMMARY ===")
    print("Detection frames:", total_detection_frames)
    print("Frames with at least one detection:", frames_with_cones)
    print("Screenshots folder:", SAVE_DIR)
    print("TEST FINISHED")
