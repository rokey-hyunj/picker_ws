# 🛒 Picker

ROS2와 Turtlebot4를 활용한 자율 주행 의류 매장 재고 관리 시스템입니다. YOLO 기반 객체 인식과 다중 로봇 협업을 통해 매장 내 재고 파악 및 정리 작업을 자동화합니다.

## 🔧 기술 스택

### 로봇 플랫폼 & 프레임워크
![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![TurtleBot4](https://img.shields.io/badge/TurtleBot4-iRobot-00A4EF?style=for-the-badge&logo=irobot&logoColor=white)

### AI & 컴퓨터 비전
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![YOLO](https://img.shields.io/badge/YOLOv8-Ultralytics-00FFFF?style=for-the-badge&logo=yolo&logoColor=black)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white)

### 내비게이션 & 센서
![Nav2](https://img.shields.io/badge/Nav2-Navigation-22314E?style=for-the-badge&logo=ros&logoColor=white)
![LiDAR](https://img.shields.io/badge/LiDAR-RPLIDAR-FF6B6B?style=for-the-badge&logo=lidar&logoColor=white)
![OAK-D](https://img.shields.io/badge/OAK--D-RGB--D-4CAF50?style=for-the-badge&logo=camera&logoColor=white)

### 통신 & 제어
![ROS2 Topics](https://img.shields.io/badge/ROS2-Topics-22314E?style=for-the-badge&logo=ros&logoColor=white)
![ROS2 Actions](https://img.shields.io/badge/ROS2-Actions-22314E?style=for-the-badge&logo=ros&logoColor=white)
![DDS](https://img.shields.io/badge/DDS-FastDDS-00599C?style=for-the-badge&logo=dds&logoColor=white)

## 📋 목차
- [프로젝트 개요](#-프로젝트-개요)
- [시스템 아키텍처](#-시스템-아키텍처)
- [주요 기능](#-주요-기능)
- [기술 스택](#-기술-스택)
- [프로젝트 구조](#-프로젝트-구조)
- [설치 및 실행](#-설치-및-실행)
- [노드 설명](#-노드-설명)
- [통신 프로토콜](#-통신-프로토콜)

## 🎯 프로젝트 개요

의류 매장에서 Turtlebot4 로봇들이 협업하여 다음 작업을 수행합니다:

1. **고객 추적 시스템**: 웹캠과 OAK-D 카메라를 활용한 고객 감지 및 추적
2. **재고 확인 시스템**: YOLO 기반 의류 박스 개수 자동 카운팅
3. **다중 로봇 협업**: Phase 기반 작업 순서 제어 및 충돌 방지
4. **실시간 모니터링**: CCTV 카메라를 통한 ROI 영역 감시

## 🏗 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    매장 환경 (Map)                           │
│                                                             │
│  ┌──────────┐         ┌──────────┐        ┌──────────┐      │
│  │ 웹캠     │         │ CCTV     │        │ 박스     │       │
│  │ (고객감지)│         │ (ROI감시)│         │ (재고)   │      │
│  └────┬─────┘         └────┬─────┘        └────┬─────┘      │
│       │                    │                   │            │
│  ┌────▼──────────────┬─────▼──────────────┬────▼──────────┐ │
│  │  Robot 1          │  Robot 2           │  Robot 3      │ │
│  │  (고객 추적)      │  (재고 확인)       │  (재고 확인)  │ │
│  │  - Webcam YOLO    │  - Box YOLO        │  - Box YOLO   │ │
│  │  - OAK-D Camera   │  - LiDAR           │  - LiDAR      │ │
│  │  - Nav2           │  - Phase Control   │  - Phase Ctrl │ │
│  └───────────────────┴────────────────────┴───────────────┘ │
│                                                             │
│              ROS2 Communication Layer                       │
│  ┌─────────────────────────────────────────────────────┐    │
│  │ Topics: /cmd_vel, /scan, /oakd/*, /camera/*         │    │
│  │ Actions: NavigateToPose                             │    │
│  │ Custom: /current_phase, /ROI_robot_detected         │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

## ✨ 주요 기능

### 1. 고객 추적 시스템 (`customer_tracking.py`)

**다중 카메라 기반 지능형 추적**
- **웹캠 모드**: 4사분면 YOLO 객체 감지로 고객 위치 파악
  - 사분면별 목표 좌표 자동 설정
  - Nav2 기반 자율 주행
- **OAK-D 추적 모드**: Depth 정보를 활용한 정밀 추적
  - 1.1m 거리 유지 주행
  - 실시간 거리 및 각도 보정
  - 타겟 상실 시 360도 회전 재탐색
- **상태 전환**: 웹캠 감지 → 주행 → OAK-D 추적 → 재탐색 → 웹캠 복귀

**주요 파라미터**
```python
follow_distance = 1.1m      # 추적 거리
k_v = 0.8                   # 선속도 게인
k_w = 1.2                   # 각속도 게인
lost_timeout = 3.0s         # 타겟 상실 판단 시간
```

### 2. 재고 확인 시스템 (`obstacle_avoidance.py`)

**5단계 Phase 기반 자율 작업**

```
Phase 0: 주문 대기
   ↓
Phase 1: 1차 진입 (안전거리 0.5m)
   ↓
Phase 2: 박스 위치 접근 (안전거리 0.15m)
   ↓
Phase 3: YOLO 박스 카운팅 (ROI 점유)
   ↓
Phase 4: 도착지 이동
   ↓
Phase 5: 도킹 복귀
```

**스마트 충돌 회피**
- LiDAR 기반 실시간 장애물 감지
- FOV 50° 정지 영역, 100° 회피 방향 결정
- 좌/우 평균 거리 비교로 최적 회피 방향 선택
- 긴급 정지 → 후진 → 회피 회전 시퀀스

**다중 로봇 교통 제어**
- Phase 통신으로 작업 상태 공유
- ROI Bool 토픽으로 중요 영역 점유 알림
- Robot2가 작업 중이면 Robot3 대기

### 3. CCTV 모니터링 시스템 (`web_pub.py`)

**실시간 ROI 영역 감시**
- 평행사변형 ROI 정의 및 시각화
- YOLO 객체가 0.5초 이상 ROI 내 존재 시 Bool True 발행
- 30Hz 고속 이미지 처리 및 발행
- `/cctvcam/roi_status`, `/cctvcam/image_processed` 토픽 제공

## 🛠 기술 스택

### 하드웨어
- **Turtlebot4**: 메인 로봇 플랫폼
- **OAK-D Camera**: RGB-D 카메라 (Depth + RGB)
- **LiDAR**: RPLIDAR A1/A2 (장애물 감지)
- **Webcam**: USB 카메라 (고객 감지)
- **CCTV**: 고정 카메라 (ROI 모니터링)

### 소프트웨어
- **ROS2 Humble**: 로봇 운영 체제
- **Nav2**: 자율 주행 내비게이션 스택
- **YOLO v8**: 실시간 객체 인식
  - `webcam_final.pt`: 고객 감지 모델
  - `amr_final.pt`: 고객 추적 모델
  - `clothes_final.pt`: 박스 카운팅 모델
  - `cctv_final.pt`: CCTV 모니터링 모델
- **OpenCV**: 이미지 처리
- **TF2**: 좌표 변환

### Python 라이브러리
```
ultralytics      # YOLO
cv_bridge        # ROS-OpenCV 변환
tf_transformations
numpy
```

## 📁 프로젝트 구조

```
picker/
├── models/                 # YOLO 모델 파일
│   ├── webcam_final.pt     # 웹캠 고객 감지 모델
│   ├── amr_final.pt        # OAK-D 추적 모델
│   ├── clothes_final.pt    # 박스 카운팅 모델
│   └── cctv_final.pt       # CCTV 모니터링 모델
│
└── src/
    └── picker/
        └── picker/
            ├── __init__.py
            ├── customer_tracking.py    # 고객 추적 노드
            ├── obstacle_avoidance.py   # 재고 확인 노드
            ├── web_pub.py              # CCTV 모니터링 노드
            └── quadrant_checker.py     # 사분면 계산 유틸리티
```

## 🚀 설치 및 실행

### 1. 의존성 설치

```bash
# ROS2 Humble 설치 (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop

# Turtlebot4 패키지
sudo apt install ros-humble-turtlebot4-*

# Python 패키지
pip install ultralytics opencv-python numpy --break-system-packages
pip install ament-index-python --break-system-packages
```

### 2. 워크스페이스 설정

```bash
# 프로젝트 클론
git clone https://github.com/rokey-hyunj/picker_ws.git

# 빌드
cd picker_ws
colcon build --packages-select picker
source install/setup.bash
```

### 3. 실행

**터미널 1: 고객 추적 로봇**
```bash
ros2 run picker customer_tracking
```

**터미널 2: 재고 확인 로봇 (Robot2)**
```bash
ros2 run picker obstacle_avoidance --ros-args -r __ns:=/robot2
```

**터미널 3: 재고 확인 로봇 (Robot3)**
```bash
ros2 run picker obstacle_avoidance --ros-args -r __ns:=/robot3
```

**터미널 4: CCTV 모니터링**
```bash
ros2 run picker web_pub
```

## 📡 노드 설명

### Customer Tracking Node

**발행 토픽**
- `/webcam/annotated_frame` (Image): 웹캠 YOLO 결과 이미지
- `/amr2/annotated_frame` (Image): OAK-D YOLO 결과 이미지
- `/amr2/cmd_vel` (Twist): 속도 제어 명령

**구독 토픽**
- `/amr2/oakd/stereo/image_raw` (Image): Depth 이미지
- `/amr2/oakd/rgb/image_raw/compressed` (CompressedImage): RGB 이미지
- `/amr2/oakd/rgb/camera_info` (CameraInfo): 카메라 내부 파라미터

**액션 클라이언트**
- `navigate_to_pose` (NavigateToPose): Nav2 주행 명령

### Obstacle Avoidance Node

**발행 토픽**
- `/{namespace}/cmd_vel` (Twist): 속도 제어
- `/camera/box_count` (Int32): 감지된 박스 개수
- `/{namespace}/current_phase` (Int32): 현재 작업 단계
- `/ROI_robot_detected` (Bool): ROI 점유 상태
- `/{namespace}/cmd_audio` (AudioNoteVector): 도착 알림음

**구독 토픽**
- `/{namespace}/scan` (LaserScan): LiDAR 데이터
- `/{namespace}/oakd/rgb/preview/image_raw` (Image): YOLO 입력 이미지
- `/{namespace}/box_order_goals` (PoseArray): 작업 명령 (박스 위치, 도착지)
- `/robot3/current_phase` or `/robot2/current_phase` (Int32): 다른 로봇 Phase
- `/ROI_robot_detected` (Bool): 다른 로봇의 ROI 점유 상태

### Web Publisher Node

**발행 토픽**
- `/cctvcam/roi_status` (Bool): ROI 내 객체 감지 여부
- `/cctvcam/image_processed` (Image): YOLO 처리된 이미지 (30Hz)

## 🔄 통신 프로토콜

### Phase 상태 코드

| Phase | 의미 | 안전거리 | 설명 |
|-------|------|----------|------|
| 0 | 대기 중 | - | 주문 수신 대기 |
| 1 | 1차 진입 | 0.5m | 작업 영역 접근 |
| 2 | 박스 접근 | 0.15m | 정밀 접근 |
| 3 | 박스 카운팅 | 0.15m | ROI 점유 중 |
| 4 | 도착지 이동 | 0.5m | 결과 전달 |
| 5 | 도킹 복귀 | - | 충전 복귀 |

### ROI 통신 규칙

```python
# Robot2가 Phase 3일 때
set_roi_status(True)   # 다른 로봇에게 대기 요청

# Robot3의 대기 조건
if (other_phase in [1,2,3]) or is_roi_occupied:
    wait_at_safe_position()
```

### 주문 메시지 포맷 (PoseArray)

```python
# box_order_goals 토픽
poses[0]: 박스 위치 (x, y, z)
poses[1]: 도착지 위치 (x, y, z)
```

## 🎮 주요 파라미터 조정

### 고객 추적

```python
# customer_tracking.py
follow_distance = 1.1          # 추적 유지 거리
k_v = 0.8                      # 전진 게인
k_w = 1.2                      # 회전 게인
max_linear_speed = 0.25        # 최대 선속도
max_angular_speed = 0.5        # 최대 각속도
lost_timeout = 1.0             # 타겟 상실 대기
oakd_lost_timeout = 3.0        # 추적 포기 시간
```

### 재고 확인

```python
# obstacle_avoidance.py
emergency_dist = 0.40          # 긴급 정지 거리
arrival_radius = 0.05          # 도착 판정 반경
```

### CCTV 모니터링

```python
# web_pub.py
timer_period = 0.033           # 30Hz 처리
roi_threshold = 0.5            # ROI 판정 시간 (초)
```

## 🐛 트러블슈팅

### 카메라 연결 실패
```bash
# 카메라 장치 확인
ls /dev/video*

# 권한 부여
sudo chmod 666 /dev/video0
```

### 모델 로딩 실패
```bash
# 모델 파일 경로 확인
ros2 pkg prefix picker
ls $(ros2 pkg prefix picker)/share/picker/models/
```

### Nav2 서버 연결 실패
```bash
# Nav2 재시작
ros2 launch turtlebot4_navigation nav2.launch.py
```

### 다중 로봇 통신 안됨
```bash
# 네임스페이스 확인
ros2 node list
ros2 topic list

# Phase 토픽 모니터링
ros2 topic echo /robot2/current_phase
ros2 topic echo /robot3/current_phase
```

## 👥 기여자

- 박제준(팀장): CCTV 모니터링 시스템 개발, 시스템 아키텍쳐 고안
- 조재범/이승준: 웹 기반 주문 내역 관리 시스템 개발
- 황익주/배민혁: 장애물 회피 알고리즘 개발, 다중 로봇 협업 프로토콜 설계
- 안효원/김현종: 고객 추적 시스템 개발
- 전홍주/정예찬: 재고 관리 시스템 개발

**⚠️ 주의사항**
- 실제 매장 환경에서 테스트 전 안전 거리 파라미터를 충분히 조정하세요
- LiDAR 센서의 정상 작동을 항상 확인하세요
- 다중 로봇 운용 시 충분한 공간을 확보하세요