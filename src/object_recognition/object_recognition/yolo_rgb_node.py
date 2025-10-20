#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time


class YoloRGBNode(Node):
    def __init__(self):
        super().__init__('yolo_rgb_node')
        self.bridge = CvBridge()

        # === YOLOモデルの読み込み ===
        self.model = YOLO('/home/araishogo/hotel_ws/src/object_recognition/best_1.pt')

        # === ROSトピック購読・配信設定 ===
        self.sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.callback, 10)
        self.pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)

        # === 表示用ウィンドウ ===
        cv2.startWindowThread()
        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)

        # === 間引き設定 ===
        self.frame_count = 0
        self.skip_frames = 10  # ← 15fps入力の場合 → 約1.5fps推論
        self.last_infer_time = 0.0
        self.min_interval_sec = 0.5  # 0.5秒間隔（= 2fps上限）

        self.get_logger().info("✅ YOLO RGB Node Ready (with frame skipping enabled)")

    def callback(self, msg):
        self.frame_count += 1

        # 間引きロジック：フレーム数と時間両方で制御
        now = time.time()
        if (self.frame_count % self.skip_frames != 0) and (now - self.last_infer_time < self.min_interval_sec):
            return  # このフレームはスキップ

        self.last_infer_time = now

        # === 推論開始 ===
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        t0 = time.time()
        results = self.model(img, verbose=False)[0]
        infer_time = (time.time() - t0) * 1000.0  # ms
        self.get_logger().info(f"Inference took {infer_time:.1f} ms")

        det_array = Detection2DArray()
        det_array.header = msg.header

        # === 検出結果ループ ===
        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < 0.2:
                continue
            cls = int(box.cls[0])
            name = self.model.names[cls]

            x1, y1, x2, y2 = box.xyxy[0]
            cx = float((x1 + x2) / 2.0)
            cy = float((y1 + y2) / 2.0)

            det = Detection2D()
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            det_array.detections.append(det)

            label = f"{name} {conf:.2f}"
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(img, label, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # === 結果を表示・送信 ===
        cv2.imshow("YOLO Detection", img)
        cv2.waitKey(1)
        self.pub.publish(det_array)


def main(args=None):
    rclpy.init(args=args)
    node = YoloRGBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
