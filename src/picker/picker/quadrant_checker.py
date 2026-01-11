from ultralytics import YOLO
import cv2
import math
import numpy as np


def draw_rotated_axis(img, origin, angle_deg, axis_length=50):

    angle_rad = math.radians(angle_deg)

    x_dir = np.array([math.cos(angle_rad), math.sin(angle_rad)])
    y_dir = np.array([-math.sin(angle_rad), math.cos(angle_rad)])

    origin = np.array(origin, dtype=np.int32)

    x_pos = (origin + x_dir * axis_length).astype(int)
    x_neg = (origin - x_dir * axis_length).astype(int)

    y_pos = (origin + y_dir * axis_length).astype(int)
    y_neg = (origin - y_dir * axis_length).astype(int)

    cv2.line(img, tuple(x_neg), tuple(x_pos), (0, 0, 255))
    cv2.putText(img, "+X", tuple(x_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
  
    cv2.line(img, tuple(y_neg), tuple(y_pos), (0, 255, 0))
    cv2.putText(img, "+Y", tuple(y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
    cv2.putText(img, "-Y", tuple(y_neg), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    cv2.circle(img, tuple(origin), 4, (255, 255, 255), -1)

    return img

def get_quadrant(point, origin, angle_deg):

    px, py = point
    ox, oy = origin

    vx = px - ox
    vy = py - oy

    angle_rad = math.radians(angle_deg)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)

    x_p =  c * vx + s * vy
    y_p = -s * vx + c * vy

    eps = 1e-6
    if abs(x_p) < eps and abs(y_p) < eps:
        return 0, (x_p, y_p)

    if x_p > 0 and y_p > 0:
        quad = 4
    elif x_p < 0 and y_p > 0:
        quad = 3
    elif x_p < 0 and y_p < 0:
        quad = 2
    elif x_p > 0 and y_p < 0:
        quad = 1
    else:
        quad = 0

    return quad

class QuadChecker:
    def __init__(self):
        self.model = YOLO("webcam_final.pt")
        cam_source = "/dev/video0"
        
        self.cap = cv2.VideoCapture(cam_source)
        if not self.cap.isOpened():
            raise IOError(f"웹캠 소스 {cam_source}를 열 수 없습니다.")
        
        self.angle = 40

    def checker(self):
        while self.cap.isOpened():
            success, frame = self.cap.read()
            if not success:
                break

            h, w = frame.shape[:2]
            origin = (w//2, h//2)

            results = self.model(frame, classes=[0], conf=0.5, verbose=False)
            annotated_frame = frame.copy()

            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                quad = get_quadrant((cx, cy), origin, self.angle)

                cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                cv2.circle(annotated_frame, (int(cx), int(cy)), 3, (0, 255, 255), -1)
                text = f"Q{quad}"
                cv2.putText(annotated_frame, text, (int(x1), int(y1) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            annotated_frame = draw_rotated_axis(annotated_frame, origin, self.angle, axis_length=700)
            cv2.imshow("Quadrant Checker", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
        print("프로그램을 종료합니다.")


if __name__ == '__main__':
    try:
        tracker = QuadChecker()
        tracker.checker()
    except Exception as e:
        print(f"오류 발생: {e}")