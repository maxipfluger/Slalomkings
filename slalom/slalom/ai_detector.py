"""AI cone detector: Yilin's own YOLO11n (trained from scratch, models/best.onnx) -> /cones/ai
Runs on the Pi CPU with OpenCV DNN (no extra install). Same image topic as the OpenCV detector,
so both run in parallel. Positions with the same pinhole formula as detector.py.

ROS:     ros2 run slalom ai_detector --ros-args -p model:=/path/best.onnx
Offline: python3 ai_detector.py best.onnx bild.jpg   (prints the detections)
"""
import math
import sys
import time

import cv2
import numpy as np

# YOLO class id -> Cone.msg colour (models/README.md)
YOLO_TO_CONE = {0: 4, 1: 2, 2: 3, 3: 0}    # blue, pink, red, yellow
NAMES = {0: 'yellow', 1: 'orange', 2: 'pink', 3: 'red', 4: 'blue'}
SIZE = 416


def letterbox(img):
    h, w = img.shape[:2]
    s = SIZE / max(h, w)
    nw, nh = int(round(w * s)), int(round(h * s))
    out = np.full((SIZE, SIZE, 3), 114, np.uint8)
    dx, dy = (SIZE - nw) // 2, (SIZE - nh) // 2
    out[dy:dy + nh, dx:dx + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    return out, s, dx, dy


def detect(net, img, conf_th=0.4, iou_th=0.5):
    """BGR image -> list of (cone colour, conf, x, y, w, h) in pixels of the original image."""
    lb, s, dx, dy = letterbox(img)
    blob = cv2.dnn.blobFromImage(lb, 1 / 255.0, (SIZE, SIZE), swapRB=True)
    net.setInput(blob)
    out = net.forward()[0]           # (8, 3549): cx, cy, w, h, 4 class scores
    if out.shape[0] > out.shape[1]:
        out = out.T
    scores = out[4:]
    cls = scores.argmax(0)
    conf = scores.max(0)
    keep = conf > conf_th
    boxes, confs, ids = [], [], []
    for cx, cy, bw, bh, c, k in zip(out[0][keep], out[1][keep], out[2][keep], out[3][keep], conf[keep], cls[keep]):
        x = (cx - bw / 2 - dx) / s
        y = (cy - bh / 2 - dy) / s
        boxes.append([float(x), float(y), float(bw / s), float(bh / s)])
        confs.append(float(c))
        ids.append(int(k))
    res = []
    if boxes:
        for i in np.array(cv2.dnn.NMSBoxes(boxes, confs, conf_th, iou_th)).flatten():
            x, y, w, h = boxes[i]
            res.append((YOLO_TO_CONE.get(ids[i], 0), confs[i], x, y, w, h))
    return res


def position(x, w, h, bildBreite, hfovGrad, hoehe):
    """Pixel box -> (x forward, y left) in m (same as detector.py)."""
    f = (bildBreite / 2) / math.tan(math.radians(hfovGrad) / 2)
    vorne = f * hoehe / max(h, 1.0)
    links = vorne * (bildBreite / 2 - (x + w / 2)) / f
    return vorne, links


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from slalom_interfaces.msg import Cone, ConeArray

    class AiDetector(Node):
        def __init__(self):
            super().__init__('ai_detector')
            p = self.declare_parameter
            model = p('model', '/home/projects/ros2_ws/src/Slalomkings/models/best.onnx').value
            self.conf = p('confidence', 0.4).value
            self.hfov = p('hfov_deg', 69.0).value
            self.hoehe = p('cone_height', 0.17).value
            self.max_dist = p('max_dist', 3.0).value
            self.period = 1.0 / p('max_rate', 5.0).value    # Hz, limit CPU load on the Pi
            self.net = cv2.dnn.readNetFromONNX(model)
            self.pub = self.create_publisher(ConeArray, p('cones_topic', '/cones/ai').value, 10)
            self.create_subscription(Image, p('image_topic', '/camera/color/image_raw').value,
                                     self.neuesBild, qos_profile_sensor_data)
            self.last = 0.0
            self.n = 0
            self.dt_sum = 0.0
            self.get_logger().info('AI detector (YOLO11n) laeuft -> /cones/ai, Modell ' + model)

        def neuesBild(self, msg):
            now = time.monotonic()
            if now - self.last < self.period or msg.encoding not in ('bgr8', 'rgb8'):
                return
            self.last = now
            roh = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            bild = roh[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                bild = cv2.cvtColor(bild, cv2.COLOR_RGB2BGR)
            t = time.monotonic()
            dets = detect(self.net, bild, self.conf)
            self.dt_sum += time.monotonic() - t
            aus = ConeArray()
            aus.header = msg.header
            for farbe, c, x, y, w, h in dets:
                px, py = position(x, w, h, bild.shape[1], self.hfov, self.hoehe)
                if px > self.max_dist:
                    continue
                cone = Cone()
                cone.color = farbe
                cone.x = float(px)
                cone.y = float(py)
                cone.confidence = float(c)
                aus.cones.append(cone)
            self.pub.publish(aus)
            self.n += 1
            if self.n % 10 == 0:
                txt = ', '.join(f'{NAMES[c.color]} {c.confidence:.2f} ({c.x:.2f}, {c.y:+.2f})' for c in aus.cones)
                self.get_logger().info(f'{1000 * self.dt_sum / 10:.0f} ms/Bild | {len(aus.cones)} Pylonen: {txt}')
                self.dt_sum = 0.0

    rclpy.init(args=args)
    node = AiDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) > 2 and not sys.argv[1].startswith('--'):
        net = cv2.dnn.readNetFromONNX(sys.argv[1])
        img = cv2.imread(sys.argv[2])
        for d in detect(net, img):
            print(NAMES[d[0]], f'{d[1]:.2f}', [round(v) for v in d[2:]])
    else:
        main()
