"""OAK-D camera node WITH our own YOLO11n running ON the camera chip (Myriad X).
Replaces the course camera node (multi_cam): the OAK-D can only be opened by one program.

Publishes
  /camera/color/image_raw  (sensor_msgs/Image, bgr8, 1280x800, ~11 Hz)  -> OpenCV detector works as before
  /cones/ai                (slalom_interfaces/ConeArray)                -> YOLO result, computed on the camera
The Pi only receives the finished boxes -> almost no CPU load for YOLO (power problem, Evan 25.11.).

Start (camera node of the course must be stopped first, see car_scripts/start_ai_cam.sh):
  python3 /home/projects/ros2_ws/src/Slalomkings/slalom/slalom/oak_ai.py
  ros2 run slalom oak_ai          (after colcon build)
"""
import math
import time

import depthai as dai

# YOLO class id -> Cone.msg colour (models/README.md)
YOLO_TO_CONE = {0: 4, 1: 2, 2: 3, 3: 0}    # blue, pink, red, yellow
NAMES = {0: 'yellow', 1: 'orange', 2: 'pink', 3: 'red', 4: 'blue'}
NN_SIZE = 416


def position(x, w, h, bildBreite, hfovGrad, hoehe):
    """Pixel box -> (x forward, y left) in m (same formula as detector.py / ai_detector.py)."""
    f = (bildBreite / 2) / math.tan(math.radians(hfovGrad) / 2)
    vorne = f * hoehe / max(h, 1.0)
    links = vorne * (bildBreite / 2 - (x + w / 2)) / f
    return vorne, links


def box_to_pixels(d, img_w, img_h, letterbox):
    """Normalised detection on the 416x416 NN input -> (x, y, w, h) in pixels of the full image."""
    if letterbox:
        s = NN_SIZE / max(img_w, img_h)                 # 1280x800 -> 0.325
        dx = (NN_SIZE - img_w * s) / 2
        dy = (NN_SIZE - img_h * s) / 2
        x1 = (d.xmin * NN_SIZE - dx) / s
        x2 = (d.xmax * NN_SIZE - dx) / s
        y1 = (d.ymin * NN_SIZE - dy) / s
        y2 = (d.ymax * NN_SIZE - dy) / s
    else:                                               # stretched to 416x416
        x1, x2 = d.xmin * img_w, d.xmax * img_w
        y1, y2 = d.ymin * img_h, d.ymax * img_h
    return x1, y1, x2 - x1, y2 - y1


def build_pipeline(blob, img_w, img_h, fps, conf, iou, letterbox):
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    img_out = cam.requestOutput((img_w, img_h), type=dai.ImgFrame.Type.BGR888i, fps=fps)
    mode = dai.ImgResizeMode.LETTERBOX if letterbox else dai.ImgResizeMode.STRETCH
    nn_in = cam.requestOutput((NN_SIZE, NN_SIZE), type=dai.ImgFrame.Type.BGR888p, resizeMode=mode, fps=fps)

    nn = pipeline.create(dai.node.NeuralNetwork)
    nn.setBlobPath(blob)
    nn.input.setBlocking(False)
    nn_in.link(nn.input)

    parser = pipeline.create(dai.node.DetectionParser)
    parser.setBlobPath(blob)
    parser.setNNFamily(dai.DetectionNetworkType.YOLO)
    if hasattr(parser, 'setSubtype'):   # depthai >= 3.x newer; 3.1.0 detects the YOLO head by its output names
        parser.setSubtype('yolov11')
    parser.setClasses(['blue cone', 'pink cone', 'red cone', 'yellow cone'])
    parser.setCoordinateSize(4)
    parser.setConfidenceThreshold(conf)
    parser.setIouThreshold(iou)
    nn.out.link(parser.input)

    img_q = img_out.createOutputQueue(maxSize=2, blocking=False)
    det_q = parser.out.createOutputQueue(maxSize=4, blocking=False)
    return pipeline, img_q, det_q


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from slalom_interfaces.msg import Cone, ConeArray

    class OakAi(Node):
        def __init__(self):
            super().__init__('oak_ai')
            p = self.declare_parameter
            blob = p('blob', '/home/projects/ros2_ws/src/Slalomkings/models/best.blob').value
            self.img_w = p('width', 1280).value
            self.img_h = p('height', 800).value
            fps = p('fps', 11.0).value
            conf = p('confidence', 0.5).value   # 0.4 let ghost cones through (blue 0.44 in bag t10)
            self.hfov = p('hfov_deg', 69.0).value
            self.hoehe = p('cone_height', 0.17).value
            self.max_dist = p('max_dist', 3.0).value
            self.letterbox = p('letterbox', True).value
            self.publish_image = p('publish_image', True).value
            self.edge = p('edge_px', 6.0).value          # box closer than this to the image edge -> cut off -> ignored
            self.horizon = p('horizon', 0.20).value      # box bottom above this image fraction -> not on the floor
            self.n_rej = 0
            self.pub_img = self.create_publisher(Image, p('image_topic', '/camera/color/image_raw').value, 10)
            self.pub_cones = self.create_publisher(ConeArray, p('cones_topic', '/cones/ai').value, 10)

            self.pipeline, self.img_q, self.det_q = build_pipeline(
                blob, self.img_w, self.img_h, fps, conf, 0.5, self.letterbox)
            self.pipeline.start()
            self.n_img = 0
            self.n_det = 0
            self.t0 = time.monotonic()
            self.create_timer(0.01, self.poll)
            self.get_logger().info(f'OAK-D + YOLO11n auf der Kamera laeuft ({dai.__version__}) -> '
                                   f'/camera/color/image_raw + /cones/ai, Blob {blob}')

        def poll(self):
            frame = self.img_q.tryGet()
            if frame is not None and self.publish_image:
                bild = frame.getCvFrame()
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                msg.height, msg.width = bild.shape[0], bild.shape[1]
                msg.encoding = 'bgr8'
                msg.is_bigendian = 0
                msg.step = msg.width * 3
                msg.data = bild.tobytes()
                self.pub_img.publish(msg)
                self.n_img += 1

            dets = self.det_q.tryGet()
            if dets is None:
                return
            aus = ConeArray()
            aus.header.stamp = self.get_clock().now().to_msg()
            aus.header.frame_id = 'camera'
            for d in dets.detections:
                farbe = YOLO_TO_CONE.get(int(d.label))
                if farbe is None:
                    continue
                x, y, w, h = box_to_pixels(d, self.img_w, self.img_h, self.letterbox)
                # 25.11. like detector.py: a cone cut off at the image edge (left/right/bottom) gives a
                # wrong distance + direction (bag t12: phantom yellow at 0.6 m after every pass, pink seen
                # as red when close) -> ignore it. Cone above the horizon -> not on the floor -> ignore.
                m = self.edge
                if x < m or x + w > self.img_w - m or y + h > self.img_h - m or y + h < self.horizon * self.img_h:
                    self.n_rej += 1
                    continue
                px, py = position(x, w, h, self.img_w, self.hfov, self.hoehe)
                if px > self.max_dist or px <= 0.0:
                    continue
                cone = Cone()
                cone.color = farbe
                cone.x = float(px)
                cone.y = float(py)
                cone.confidence = float(d.confidence)
                aus.cones.append(cone)
            self.pub_cones.publish(aus)
            self.n_det += 1
            if self.n_det % 20 == 0:
                dt = time.monotonic() - self.t0
                txt = ', '.join(f'{NAMES[c.color]} {c.confidence:.2f} ({c.x:.2f}, {c.y:+.2f})' for c in aus.cones)
                self.get_logger().info(f'YOLO {self.n_det / dt:.1f} Hz, Bild {self.n_img / dt:.1f} Hz, '
                                       f'{self.n_rej} am Rand verworfen | {len(aus.cones)} Pylonen: {txt}')
                self.n_det = self.n_img = self.n_rej = 0
                self.t0 = time.monotonic()

        def destroy_node(self):
            try:
                self.pipeline.stop()
            except Exception:
                pass
            super().destroy_node()

    rclpy.init(args=args)
    node = OakAi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
