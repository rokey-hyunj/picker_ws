import time
import math
import os
import sys
import numpy as np
from ultralytics import YOLO
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Bool
# 🚀 [수정/추가] 이미지 메시지 발행을 위해 필요
from sensor_msgs.msg import Image 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory


class YOLOWebcamPublisher(Node):
    def __init__(self, model):
        super().__init__('cctvcam_publisher')
        self.model = model
        self.confidences = []
        self.max_object_count = 0
        self.classNames = model.names
        self.bridge = CvBridge()

        # 🚀 [추가] 이미지 전송을 위한 QoS 프로파일 정의 (best_effort)
        qos_profile_image = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # 유실되더라도 빨리 보냄 (FPS 중요)
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # 버퍼에 가장 최근 프레임 1개만 유지
        )
        
        # 🚀 [수정] Bool 상태를 발행하는 기존 Publisher
        self.bool_publisher = self.create_publisher(Bool, 'cctvcam/roi_status', 10) 
        
        # 🚀 [추가] 처리된 이미지를 발행하는 Publisher
        # 🚀 [수정] 이미지 Publisher에 QoS 프로파일 적용
        # self.image_publisher = self.create_publisher(Image, 'cctvcam/image_processed', 10)
        self.image_publisher = self.create_publisher(
            Image, 
            'cctvcam/image_processed', 
            qos_profile=qos_profile_image # QoS 적용
        )
        
        self.should_shutdown = False

        self.bool = False
        self.in_roi_since = None  # ROI 안에 들어온 시간 기록용

        # --- 웹캠 열기 ---
        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam.")
            raise RuntimeError("Webcam not available")

        # 🚀 [추가] 카메라 FPS를 30으로 설정 요청 (카메라가 지원해야 함)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        # 🚀 [수정] 타이머 주기를 30Hz에 맞게 0.033으로 설정 /기존 0.1
        self.timer = self.create_timer(0.033, self.process_frame)

    def process_frame(self):
        if self.should_shutdown:
            return

        ret, img = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from webcam.")
            return

        h, w, _ = img.shape

        # --- 평행사변형 ROI 정의 ---
        roi_points = np.array([
            [25, 125],   # P1
            [600, 100],  # P2
            [640, 340],  # P4
            [0, 340]     # P3
        ], dtype=np.int32)

        # 평행사변형(ROI) 그리기
        cv2.polylines(img, [roi_points], isClosed=True, color=(0, 255, 255), thickness=2)
        cv2.putText(img, "MY ROI", (25, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # YOLO 추론 # process_frame 함수 내부 수정
        # results = self.model(img, stream=True) # <--- 기존
        results = self.model(img, stream=True, imgsz=320) # 🚀 [수정] 입력 해상도를 320x320으로 줄여서 추론 속도 향상
        object_count = 0
        fontScale = 1
        yolo_boxes = []

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                label = self.classNames.get(cls, f"class_{cls}")

                yolo_boxes.append((x1, y1, x2, y2, label, confidence))
                object_count += 1

        # 이번 프레임에서 ROI 안에 들어온 물체가 하나라도 있는지 확인
        any_inside = False

        for (x1, y1, x2, y2, label, confidence) in yolo_boxes:
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # --- 평행사변형 ROI 내부 여부 판단 ---
            inside = cv2.pointPolygonTest(roi_points, (cx, cy), False)
            inside_roi = inside >= 0  # 경계 포함해서 ROI로 취급

            if inside_roi:
                any_inside = True

            color = (0, 255, 0) if inside_roi else (0, 0, 255)

            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, f"{label}: {confidence}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 0.5초 이상 ROI 안에 있으면 True, 아니면 False
        now = time.time()
        if any_inside:
            if self.in_roi_since is None:
                self.in_roi_since = now  # 처음 들어온 시점 기록
            if now - self.in_roi_since >= 0.5:
                self.bool = True
        else:
            self.in_roi_since = None
            self.bool = False

        # 개수 표시 + 해상도 표시 + Bool publish
        self.max_object_count = max(self.max_object_count, object_count)

        cv2.putText(img, f"Objects_count: {object_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 1)

        cv2.putText(img, f"{w}x{h}", (w - 200, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # ----------------------------------------------------
        # 🚀 [수정/추가] 이미지 토픽 발행
        # ----------------------------------------------------
        # 1. OpenCV 이미지를 ROS 이미지 메시지로 변환 (BGR 형식)
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        
        # 2. 이미지 Publisher로 발행
        self.image_publisher.publish(ros_image)
        
        # 3. Bool 상태 Publisher로 발행
        self.bool_publisher.publish(Bool(data=self.bool))

        # 4. ❌ 로컬 화면 표시 기능 제거
        # cv2.imshow("Webcam", img)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.get_logger().info("q pressed, stopping frame processing.")
        #     self.should_shutdown = True

    def destroy_node(self):
        # 리소스 정리
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        # ❌ 로컬 창 정리 코드 제거
        # cv2.destroyAllWindows() 
        super().destroy_node()


def main():
    # YOLO 모델 경로 설정
    try:
        package_share_dir = get_package_share_directory('picker')
        model_path = os.path.join(package_share_dir, 'models', 'cctv_final.pt')
    except:
        # 패키지를 찾을 수 없는 경우 스크립트 위치 기준 상대 경로 사용
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, '../../../models/cctv_final.pt')

    if not os.path.exists(model_path):
        print(f"❌ File not found: {model_path}")
        exit(1)

    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"❌ Unsupported model format: {suffix}")
        exit(1)

    rclpy.init()
    node = YOLOWebcamPublisher(model)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🔴 Ctrl+C received. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Shutdown complete.")
        sys.exit(0)