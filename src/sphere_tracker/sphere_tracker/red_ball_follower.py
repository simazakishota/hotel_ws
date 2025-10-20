import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import TwistStamped


class SphereTracker(Node):
    def __init__(self):
        super().__init__('sphere_tracker') 
        cv2.startWindowThread()
        self.model = PinholeCameraModel()
        self.camera_info_received = False
        self.latest_depth = None
        self.camera_frame_id = None

        # === HSV赤色範囲 (Hue 0-10 and 170-180)
        self.lower_red1 = np.array([0, 100, 50]) 
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 20, 0])
        self.upper_red2 = np.array([180, 255, 255])

        # === パラメータ群 ===
        self.outlier_threshold_m = 0.10  # 外れ値とみなす深度差の閾値 [m]
        self.ema_alpha = 0.30    # 深度値の指数移動平均(EMA)の平滑化係数 (0に近いほど滑らか)
        self.min_depth_margin_m = 0.05    # 深度マージンの最小値 (壁や床との干渉回避用) [m]
        self.min_pixels_for_mean = 1000   # 平均を計算するために必要な最小ピクセル数
        self.depth_std_gate_m = 0.5   # 深度標準偏差の許容値 [m] (これ以上はノイズとして排除)
        self.circularity_gate_search = 0.8    # 円形度ゲート（探索中）: 1に近いほど円形
        self.circularity_gate_locked = 0.6   # 円形度ゲート（ロック中）: 
        self.solidity_gate = 0.6  # ソリディティ（凸包に対する充実度）の閾値
        self.circle_fill_gate = 0.6   # 円が塗りつぶされている割合の閾値
        self.min_area_px = 3000  # 検出対象とする最小面積 [px] 
      
        self.positions_buffer = []
        self.last_pub_time = self.get_clock().now()
        self.publish_interval = 0.005  # 秒
    
        self.allowed_miss_search = 3   # 探索中に許容する未検出フレーム数 (連続で見失っても追跡継続)
        self.allowed_miss_locked = 7   # ロック中に許容する未検出フレーム数 (より長く追従を維持)

        self.morph_kernel_search = 4  #50    # 探索中に使うモルフォロジー演算のカーネルサイズ (ノイズ除去用)
        self.morph_kernel_locked = 4  #60   # ロック中に使うモルフォロジー演算のカーネルサイズ (より強力に平滑化)

        self.state = 'SEARCH'
        self.lock_count = 0
        self.miss_count = 0
        self.prev_px = None
        self.prev_r_px = None

        self.traj_points = []
        self.max_traj_len = 200  # 軌跡描画のための保存する点の数（50フレーム分）

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_pub = self.create_publisher(Image, 'sphere_tracker/image_annotated', 10)
        self.pose_pub  = self.create_publisher(PoseStamped, '/target_position', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds',10)
        self.target_cam_pub = self.create_publisher(PoseStamped, '/target_camera', 10)

        #d405リアルセンス
        # カラー画像
        #self.create_subscription(
        #        Image, '/camera/camera/color/image_rect_raw', self.image_callback, 10)
        # 深度画像（カラーにアライン済み）
        #self.create_subscription(
        #        Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        # カメラ情報（depthの方を使う）
        #self.create_subscription(
        #        CameraInfo, '/camera/camera/depth/camera_info', self.camera_info_callback, 10)

       #D435iリアルセンス
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)

        self.get_logger().info("SphereTracker ready: publishes filtered, offsetted /target_position")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.camera_frame_id = msg.header.frame_id

            self.fx = self.model.fx()
            self.fy = self.model.fy()
            self.cx0 = self.model.cx()
            self.cy0 = self.model.cy()

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _predict_roi(self, w, h):
        if self.prev_px is None or self.prev_r_px is None:
            return (0, 0, w, h)
        u, v = self.prev_px
        r = max(20, 5 * int(self.prev_r_px)) #（認識半径✕ 5）と 20 でピクセルが大きい方をROIの範囲とする
        return (max(0, u - r), max(0, v - r), min(w, u + r), min(h, v + r))

    def _circularity(self, contour):
        A = cv2.contourArea(contour)
        if A <= 1e-6:
            return 0.0
        P = cv2.arcLength(contour, True)
        return (4.0 * math.pi * A) / (P * P + 1e-12)

    def image_callback(self, msg):
        self.get_logger().info(f"[DEBUG] === image_callback START ===")

        if not self.camera_info_received:
            return
        if self.latest_depth is None:
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link', self.camera_frame_id, now, timeout=Duration(seconds=0.5)
            )
            self.get_logger().info(
                f"[DEBUG] Camera-to-base TF: x={trans.transform.translation.x:.3f}, "
                f"y={trans.transform.translation.y:.3f}, z={trans.transform.translation.z:.3f}"
            )

        except Exception as e:
            self.get_logger().warn(f"[TF] Could not get transform: {e}")
            return

        color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = color_img.shape[:2]

        # ブラー + HSV変換
        blur = cv2.GaussianBlur(color_img, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # ROIによるマスク生成
        if self.state == 'LOCKED':
            x0, y0, x1, y1 = self._predict_roi(w, h)
            hsv_roi = hsv[y0:y1, x0:x1]
            mask1 = cv2.inRange(hsv_roi, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv_roi, self.lower_red2, self.upper_red2)
            mask_roi = cv2.bitwise_or(mask1, mask2)
            offset = (x0, y0)
            self.get_logger().info(f"Using ROI: ({x0},{y0})-({x1},{y1})")
        else:
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask_roi = cv2.bitwise_or(mask1, mask2)
            offset = (0, 0)
            self.get_logger().info("Using FULL image for detection")
        # === モルフォロジー前のマスク（二値化画像）をデバッグ表示 ===
        mask_before_morph = mask_roi.copy()
        debug_mask_pre = cv2.resize(mask_before_morph, None, fx=0.5, fy=0.5)
        #cv2.imshow("Mask Pre-Morph", debug_mask_pre)
        #cv2.waitKey(1)

       #=== モルフォロジー処理 ===
       # 状態に応じてモルフォロジーのカーネルサイズを選択
        ksz = self.morph_kernel_locked if self.state == 'LOCKED' else self.morph_kernel_search
        # 楕円形のカーネルを作成（ノイズ除去や穴埋めに使う）
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksz, ksz))
        # モルフォロジー処理：小さなゴミを消す
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, kernel, 1)
        # モルフォロジー処理：マスクの穴を埋める
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_CLOSE, kernel, 1)

        # === HSVマスク（真っ白画像）をデバッグ表示 ===
        debug_mask = cv2.resize(mask_roi, None, fx=0.5, fy=0.5)
        #cv2.imshow("HSV Mask", debug_mask)
        #cv2.waitKey(1)

        # 輪郭を抽出（白領域を物体として切り出す）
        contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # === 輪郭抽出直後のデバッグ表示 ===

        # ROIオフセットを反映（stateがLOCKEDのとき）
        # === デバッグ表示用：オフセットを補正したコピー ===
        debug_contours = color_img.copy()
        if self.state == 'LOCKED':
            contours_dbg = [cnt + np.array([[offset]], dtype=cnt.dtype) for cnt in contours]
        else:
            contours_dbg = contours

        cv2.drawContours(debug_contours, contours_dbg, -1, (0, 255, 0), 2)  # 全輪郭を緑で描画
        debug_small_contours = cv2.resize(debug_contours, None, fx=0.5, fy=0.5)
        #cv2.imshow("Contours Raw", debug_small_contours)
        #cv2.waitKey(1)

        # 面積でフィルタリング（2000px〜40000px の範囲内のものだけ残す）
        filtered_contours = [cnt for cnt in contours if 2000 < cv2.contourArea(cnt) < 40000]
        #ベストな輪郭を探すための初期化
        best_score = -1.0
        best_contour = None

        for cnt in filtered_contours:
            area = cv2.contourArea(cnt)

            # 円形度
            circ = self._circularity(cnt)
            if circ < 0.6:
                continue  # 円っぽくなければ無視

            # 凸包と面積の比率（中身の詰まり具合）
            hull = cv2.convexHull(cnt)
            solidity = area / (cv2.contourArea(hull) + 1e-6)

            # 外接円での塗りつぶし率（fill率）
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            fill = area / (math.pi * radius * radius + 1e-6)

            # スコア計算（円っぽさを一番重視）
            score = (circ * 2.0) + (solidity * 1.0) + (fill * 1.0)

            if score > best_score:
                best_score = score
                best_contour = cnt
                best_circ = circ
                best_sol = solidity
                best_fill = fill

        if not contours:
            self._on_miss("no contours")
            return

        if best_contour is None:
            self._on_miss("no valid contour (shape filter)")
            return

        largest = best_contour
        area = cv2.contourArea(largest)
        self.get_logger().info(f"Largest contour area={area:.1f}, threshold={self.min_area_px}")
        if area < self.min_area_px:
            self._on_miss(f"small area {area:.1f}")
            return

        largest[:, :, 0] += offset[0]
        largest[:, :, 1] += offset[1]

        (x_c, y_c), radius = cv2.minEnclosingCircle(largest)
        cx, cy = int(x_c), int(y_c)
        r_px = float(radius)
        self.prev_px = (cx, cy)
        self.prev_r_px = r_px
        self.get_logger().info(f"Circle center=({cx},{cy}), r_px={r_px:.1f}")

        # 軌跡バッファに現在の中心を追加
        self.traj_points.append((cx, cy))
        if len(self.traj_points) > self.max_traj_len:
            self.traj_points.pop(0)

        circ = self._circularity(largest)
        hull = cv2.convexHull(largest)
        solidity = cv2.contourArea(largest) / (cv2.contourArea(hull)+1e-6)
        fill = cv2.contourArea(largest) / (math.pi*(r_px**2)+1e-6)

        circ_gate = self.circularity_gate_locked if self.state == 'LOCKED' else self.circularity_gate_search

        pass_count = sum([circ >= circ_gate, solidity >= self.solidity_gate, fill >= self.circle_fill_gate])
        if pass_count < 3:
            self._on_miss("shape gate failed")
            return

        # === 深度マスク作成：内部だけ使うために erode 追加 ===
        blob_mask = np.zeros_like(self.latest_depth, dtype=np.uint8)
        cv2.drawContours(blob_mask, [largest], -1, 255, thickness=cv2.FILLED)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        blob_mask = cv2.erode(blob_mask, kernel, iterations=1)  # 輪郭の内側だけ使う

        # === 有効な深度ピクセル抽出 ===
        depth_roi_mm = self.latest_depth[(blob_mask == 255)].astype(np.float32)
        valid_mm = depth_roi_mm[(depth_roi_mm > 0) & np.isfinite(depth_roi_mm)]
        depth_m_array = valid_mm / 1000.0  # 単位変換：mm → m

        if depth_m_array.size < self.min_pixels_for_mean:
            self._on_miss("depth insufficient")
            return

        # === 外れ値除去：中央値 ±0.2m 以内に限定 ===
        median = np.median(depth_m_array)
        abs_dev = np.abs(depth_m_array - median)
        filtered = depth_m_array[abs_dev < 0.2]  # 20cm以内に限定

        self.get_logger().info(f"Filtered depth pixels={filtered.size}")
        if filtered.size < self.min_pixels_for_mean:
            self._on_miss("depth too noisy after filtering")
            return

        # === 平均と標準偏差を再計算 ===
        depth_m = float(filtered.mean())
        std_m = float(filtered.std())

        self.get_logger().info(f"Depth mean={depth_m:.3f}m, std={std_m:.3f}m (gate={self.depth_std_gate_m})")
        if std_m > self.depth_std_gate_m:
            self._on_miss("depth std too large")
            return

        self.get_logger().info(f"Publishing target: ({depth_m:.3f} m) at pixel=({cx},{cy})")

        self.get_logger().info(f"[DEBUG] cx={cx}, cy={cy}, depth_m={depth_m:.3f}")

      # === 最終的に選ばれた赤玉だけを描画するデバッグウィンドウ ===
        debug_final = color_img.copy()
        cv2.drawContours(debug_final, [largest], -1, (0, 0, 255), 3)  # 赤色で輪郭
        cv2.circle(debug_final, (cx, cy), int(r_px), (255, 0, 0), 2)  # 青円で外接円
        cv2.circle(debug_final, (cx, cy), 3, (0, 255, 255), -1)       # 黄点で中心点

        # 軌跡を線で描画（青）
        for i in range(1, len(self.traj_points)):
            pt1 = self.traj_points[i - 1]
            pt2 = self.traj_points[i]
            cv2.line(debug_final, pt1, pt2, (0, 255, 0), 2)
        debug_small = cv2.resize(debug_final, None, fx=0.5, fy=0.5)
        #cv2.imshow("Final Red Ball", debug_small)
        #cv2.waitKey(1)

        if self.state != 'LOCKED':
            self.get_logger().warn("Switching to LOCKED state")
            self.state = 'LOCKED'
            self.lock_count = 1
            self.miss_count = 0
        else:
            self.lock_count += 1

        # === カメラ座標における赤玉位置を計算 ===
        ray = self.model.projectPixelTo3dRay((cx, cy))  # (u,v) → カメラ座標の方向ベクトル
        ray = np.array(ray, dtype=np.float32)
        ray_norm = ray / np.linalg.norm(ray)
        pos_cam = ray_norm * depth_m  # 単位：メートル

        self.get_logger().info(
              f"[camera frame] Ball center: x={pos_cam[0]:.3f} m, y={pos_cam[1]:.3f} m, z={pos_cam[2]:.3f} m"
        )
        adjusted_depth_m = max(0.01, depth_m)  # 負値防止
        pos_cam = ray_norm * adjusted_depth_m


        # --- TF変換に使うPoseStampedの作成 ---
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.camera_frame_id
        pose_msg.header.stamp = rclpy.time.Time().to_msg()  # ← 今の時刻を指定（未来エラー回避）

        pose_msg.pose.position.x = float(pos_cam[0])
        pose_msg.pose.position.y = float(pos_cam[1])
        pose_msg.pose.position.z = float(pos_cam[2])

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 0.0
        self.target_cam_pub.publish(pose_msg)
        try:
                self.get_logger().info("[DEBUG] Calling tf_buffer.transform()")
                pose_trans = self.tf_buffer.transform(
                        pose_msg,
                        target_frame='base_link',
                        timeout=Duration(seconds=0.5)
                )
                self.get_logger().info(f"[DEBUG] Transformed pose: {pose_trans.pose.position.x:.3f}, {pose_trans.pose.position.y:.3f}, {pose_trans.pose.position.z:.3f}")
                self.prev_cam_pos = np.array([
                        pose_trans.pose.position.x,
                        pose_trans.pose.position.y,
                        pose_trans.pose.position.z
                ])

                pos = np.array([
                        pose_trans.pose.position.x,
                        pose_trans.pose.position.y,
                        pose_trans.pose.position.z
                ])
                self.positions_buffer.append(pos)

                now = self.get_clock().now()
                elapsed = (now - self.last_pub_time).nanoseconds * 1e-9

                if elapsed > self.publish_interval and len(self.positions_buffer) > 0:
                        avg = np.mean(self.positions_buffer, axis=0)

                        avg_pose = PoseStamped()
                        avg_pose.header.frame_id = "base_link"
                        avg_pose.header.stamp = now.to_msg()
                        avg_pose.pose.position.x = float(avg[0])
                        avg_pose.pose.position.y = float(avg[1])
                        avg_pose.pose.position.z = float(avg[2])
                        avg_pose.pose.orientation.w = 1.0

                        self.pose_pub.publish(avg_pose)
                        self.get_logger().info(
                                f"Published averaged pose: ({avg[0]:.3f}, {avg[1]:.3f}, {avg[2]:.3f})"
                        )

                        # バッファをクリアして次の3秒を準備
                        self.positions_buffer.clear()
                        self.last_pub_time = now

        except Exception as e:
                self.get_logger().warn(f"[ERROR] Failed to transform pose: {e}")

    def _on_miss(self, reason: str):
        self.miss_count += 1
        self.lock_count = 0
        thr = self.allowed_miss_locked if self.state == 'LOCKED' else self.allowed_miss_search
        self.get_logger().warn(f"MISS[{self.miss_count}/{thr}] reason={reason}")
        if self.miss_count >= thr:
            if self.state != 'LOST':
                self.get_logger().warn(f"-> LOST (reason={reason})")
            self.state = 'LOST'
            self.prev_px, self.prev_r_px = None, None


def main(args=None):
    rclpy.init(args=args)
    node = SphereTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
