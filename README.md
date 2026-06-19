# [26-1 지능형로보틱스, 송다은 교수] (최종프로젝트) Shuttle-R: 자율 셔틀콕 수거 로봇

## 팀 정보

| 이름 | 학과 / 학번 |
| --- | --- |
| Siti Hajar Asyiqin Binti Fazillah | 소프트웨어학부 컴퓨터공학전공 / 2371004 |

본 프로젝트는 1인 개인 프로젝트로, 역할 분담 항목은 해당사항이 없습니다.

## 링크

- **YouTube (발표 영상, 미등록/Unlisted):** https://youtu.be/R4c_BBfqKxI
- **GitHub:** https://github.com/HajarFazillah/shuttle-r
- **README (영문 버전):** [README_EN.md](./README_EN.md)

## AI 사용 여부 및 사용 내용

프로젝트 전반에 걸쳐 Claude Code를 활용하였으며, 프로젝트의 주된 작성자라기보다는 디버깅 및 학습 보조 도구로 사용하였습니다:

- **디버깅**: DDS/FastRTPS 디스커버리 실패, TF lookup 오류, AMCL pose 문제 등 터미널 에러 로그를 해석하고, 통합 테스트 과정에서 문제의 근본 원인을 탐색하는 데 적극적으로 활용하였습니다.
- **파라미터 튜닝**: Nav2 파라미터(goal tolerance, costmap 설정 등)를 조정하여 최종 접근 시 진동(oscillation) 문제와 "No valid trajectories" 오류 등을 해결하는 데 도움을 받았습니다.
- **노드 구현 가이드**: 전체 시스템 아키텍처, 노드 구조, 설계 결정(detection, seeking, collection 노드 간 협업 방식 등)은 제가 직접 계획하였습니다. ROS2 전체 프로젝트를 처음 만들어보는 것이었기 때문에, 수업에서 배운 개념(topic, tf2, service 등)을 실제 동작하는 노드 코드로 구현하는 과정에서 AI를 가이드로 활용하였습니다 - 전체를 자동 생성하는 용도로 사용하지는 않았습니다.
- **문서화**: 이 README를 구조화하고 다듬는 데 부분적으로 도움을 받았습니다.

요약하자면: 프로젝트의 구조, 설계, 통합에 관한 결정은 제가 직접 내렸으며, AI 활용은 디버깅, 파라미터 튜닝, 그리고 수업에서 배운 내용을 바탕으로 노드 코드를 구현하는 과정에 집중되었습니다.

## 참고 자료

1. **[Nav2] Navigating with ROS2** - https://docs.nav2.org/
2. **[YOLO]** Cao, Z. et al. (2021). *Detecting the shuttlecock for a badminton robot: A YOLO based approach*. Expert Systems with Applications. https://www.sciencedirect.com/science/article/abs/pii/S0957417420306436
3. **[Dataset]** Roboflow 셔틀콕 감지 데이터셋 - https://universe.roboflow.com/computervision-ach8c/shuttlecock-rzwox
4. **[Reference Robot]** Acme Robotics, Mobile Autonomous Robot (MARIO-COM) - https://github.com/bharadwaj-chukkala/MARIO-COM
5. **[YO-CSA-T]** Real-time Badminton Tracking System (YOLOv8 기반, 2025) - https://arxiv.org/abs/2501.06472

---

# Shuttle-R

Shuttle-R은 ROS2 Humble + Gazebo Fortress(Ignition) 환경에서 TurtleBot4를 이용해
코트에 흩어져 있는 배드민턴 셔틀콕을 탐지·수거·반납하는 자율 로봇 시뮬레이션
프로젝트입니다.

본 프로젝트는 내비게이션과 위치추정을 위해 기존 ROS2/Nav2/SLAM 패키지를
기반으로 하나, 셔틀콕 탐지, 3D 위치 추정, 수거 메커니즘, 그리고 조정(coordination)
로직은 본 프로젝트의 핵심 기여로서 직접 구현한 커스텀 노드입니다.

## 기술 스택
- ROS2 Humble, Gazebo Fortress (Ignition), Ubuntu 22.04
- TurtleBot4 (Standard)
- Nav2 (AMCL + NavfnPlanner + DWB controller), SLAM Toolbox (매핑)
- OpenCV (HSV 기반 셔틀콕 탐지)
- image_geometry + tf2 (픽셀 → 3D 역투사)

## 패키지 구조 (`src/shuttler_sim`)
- `worlds/empty_court.sdf` - 경계 벽, 네트, 드롭오프 존(NE 코너), scoop/hopper
  모델, 셔틀콕 3개가 배치된 배드민턴 코트 월드
- `launch/empty_world.launch.py` - Gazebo(headless) 실행, 로봇 스폰, RGB/depth/
  camera-info/lidar/clock 토픽 브릿지, 자동 언도킹, 탐지 노드 실행
- `config/nav2.yaml` - 튜닝된 Nav2 파라미터(최종 접근 진동 문제 해결을 위해
  goal tolerance를 완화)
- `maps/court_map.{pgm,yaml}` - AMCL 위치추정을 위한 SLAM 생성 코트 맵

## 커스텀 노드

### `shuttlecock_detector`

로봇의 RGB 카메라로부터 들어오는 영상에서 주황색 셔틀콕 스커트를 HSV 기반
색상 검출(H:5-20, S:120-255, V:80-255)로 탐지합니다. 노이즈 제거를 위해
morphological open/close 연산을 적용하고, 컨투어(contour)를 면적(5-2000px)
기준으로 필터링한 다, `/shuttlecock_detections`에 `vision_msgs/Detection2DArray`를,
`/shuttlecock_detection/debug_image`에 주석이 달린 디버그 이미지를 퍼블리시합니다.

### `shuttlecock_seeker`

탐지 결과 + depth 이미지 + 카메라 내부 파라미터(intrinsics)를 구독합니다.
탐지된 셔틀콕 픽셀 중심을 depth 샘플링(최소 5px 패치)과 핀홀 카메라 모델을
이용해 3D 좌표로 역투사하고, tf2를 통해 map 프레임으로 변환한 다음 Nav2에
`NavigateToPose` 목표를 전송합니다.

잘못된 타겟을 피하기 위한 필터링 로직:

- 이미 수거 완료된 셔틀콕이 있는 gather point 근처의 탐지는 무시
- 로봇 자체의 픽업 반경 내 탐지는 무시 (자기 자신 오탐지 방지)
- 코트 경계 밖의 타겟은 무시

유효한 타겟을 찾지 못했을 때의 복구(recovery) 동작:

- 제자리에서 회전하며 주변을 스캔 (최대 3회)
- 그래도 타겟이 없으면 알려진 셔틀콕 위치 근처의 사전 정의된 탐색 지점으로 이동

`/shuttlecocks_collected`, `/shuttlecocks_deposited` 토픽을 통해 collector와
조정하며, 배치(batch)가 가득 찼거나(또는 보이는 모든 셔틀콕을 수거했을 때)
deposit 동작을 트리거합니다. 설정된 총 개수(3개)가 모두 deposit되면 동작을
종료합니다.

### `shuttlecock_collector`

물리적인 수거 및 deposit 작업 주기를 관리합니다. Gazebo ground truth
(`ign model -m turtlebot4 -p`)를 사용해 로봇의 실제 위치를 파악하고, scoop
위치를 계산한 다음, 0.5m 이내의 셔틀콕을 `ign service set_pose`를 통해 로봇의
hopper 슬롯으로 텔레포트시켜 수거합니다. 로봇이 이동하는 동안 수거된
셔틀콕은 hopper에 실린 상태로 유지됩니다(매 사이클마다 재텔레포트). 모든
위치 판단(수거, dropoff, 추적)은 Gazebo ground truth를 사용하므로 AMCL
드리프트의 영향을 받지 않습니다.

로봇이 gather point에 도달하면(0.6m 이내), 보유 중인 모든 셔틀콕을 드롭오프
존 내 격자(grid) 형태로 deposit합니다. seeker가 배치/deposit 주기를 조정할
수 있도록 수거 및 deposit 개수를 퍼블리시합니다.

### `scoop_follower`

Gazebo ground truth(`ign model -m turtlebot4 -p`)를 조회하여 로봇의 실제
위치를 파악하고, `scoop_assembly`와 `hopper_bin` Gazebo 모델을 1Hz로 고정
오프셋 위치에 텔레포트시켜 로봇에 단단히 부착된 것처럼 유지합니다. busy
guard와 백그라운드 스레딩을 사용하여 `ign service` 서브프로세스가 누적되는
것을 방지합니다.

또한 5초마다 로봇의 ground truth 위치를 `/initialpose`에 재퍼블리시하여
AMCL이 실제 위치에 고정되도록 합니다. 이는 대칭적인 코트 레이아웃에서
발생할 수 있는 AMCL 파티클 필터 드리프트를 방지합니다.

### `teleop_keyboard`

수동 주행 및 테스트를 위한 WASD/방향키 텔레오퍼레이션 노드입니다.

## ROS2 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/shuttlecock_detections` | `Detection2DArray` | 탐지된 셔틀콕의 2D 바운딩 박스 |
| `/shuttlecock_detection/debug_image` | `Image` | 탐지 결과가 표시된 카메라 디버그 이미지 |
| `/shuttlecocks_collected` | `Int32` | 누적 수거된 셔틀콕 총 개수 |
| `/shuttlecocks_deposited` | `Int32` | gather point에 deposit된 셔틀콕 총 개수 |
| `/camera/image_raw` | `Image` | Gazebo에서 브릿지된 RGB 카메라 |
| `/camera/depth/image_raw` | `Image` | 브릿지된 depth 카메라 (32FC1, 미터 단위) |
| `/camera/camera_info` | `CameraInfo` | 브릿지된 카메라 내부 파라미터 |
| `/scan` | `LaserScan` | SLAM/AMCL을 위해 브릿지된 RPLidar |

## 실행 방법

### 사전 준비사항

- Ubuntu 22.04, ROS2 Humble, Gazebo Fortress (Ignition)
- TurtleBot4 패키지 (`turtlebot4_ignition_bringup`, `turtlebot4_navigation`)
- **매 실행 전 재부팅** - 이전 세션에서 남은 FastRTPS 공유 메모리 파일이
  DDS 디스커버리를 막을 수 있습니다. 재부팅이 이를 확실하게 정리하는
  유일한 방법입니다.

### 빌드

```bash
cd ~/shuttler_ws
colcon build --symlink-install
source install/setup.bash
```

### 단계별 실행

총 7개의 터미널을 엽니다. **모든 터미널**에서 먼저 환경을 소싱해야 합니다:

```bash
source /opt/ros/humble/setup.bash && source ~/shuttler_ws/install/setup.bash
```

이후 아래 단계를 **순서대로** 진행하며, 각 단계의 준비 완료 신호를 기다립니다:

#### 1단계 - T1: Gazebo + 로봇 + 브릿지 + 탐지 노드 실행

```bash
ros2 launch shuttler_sim empty_world.launch.py
```

**"Undock Goal Succeeded"** (또는 "OK creation of entity" 출력 후 약 40초)를
기다립니다.

#### 2단계 - T2: 위치추정(AMCL) 실행

```bash
ros2 launch turtlebot4_navigation localization.launch.py \
  map:=$HOME/shuttler_ws/maps/court_map.yaml \
  use_sim_time:=true \
  params_file:=$HOME/shuttler_ws/install/shuttler_sim/share/shuttler_sim/config/nav2.yaml
```

**"Managed nodes are active"**를 기다립니다.

#### 3단계 - 초기 pose 설정 (소싱된 아무 터미널에서)

```bash
ros2 topic pub --times 10 --rate 1 /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
```

**중요:** 명령이 끝난 후 30초를 기다린 다음, AMCL이 수렴했는지 확인합니다:

```bash
ros2 topic echo /amcl_pose --once
```

반환된 위치가 `x: 1.0, y: 0.0` 근처인지 확인합니다. 응답이 없거나 잘못된
값이 나오면 더 기다린 후 다시 시도하세요. **AMCL이 올바르게 확인되기 전까지
다음 단계로 진행하지 마세요.**

#### 4단계 - T3: Nav2 실행

```bash
ros2 launch turtlebot4_navigation nav2.launch.py \
  use_sim_time:=true \
  params_file:=$HOME/shuttler_ws/install/shuttler_sim/share/shuttler_sim/config/nav2.yaml
```

**"Managed nodes are active"**를 기다립니다.

#### 5단계 - T4: scoop follower 실행

```bash
ros2 run shuttler_sim scoop_follower
```

"Scoop follower started"가 출력되어야 합니다. 이 시점부터 5초마다 Gazebo
ground truth로부터 AMCL 재시드(re-seed)가 시작됩니다.

#### 6단계 - T5: collector 실행

```bash
ros2 run shuttler_sim shuttlecock_collector
```

"Shuttlecock collector started, tracking 3 shuttlecocks"가 출력되어야
합니다.

#### 7단계 - T6: seeker 실행 (반드시 마지막에 실행)

```bash
ros2 run shuttler_sim shuttlecock_seeker
```

seeker가 실행되면 탐지, 내비게이션, 수거, deposit으로 이어지는 자율
파이프라인이 시작됩니다. **중간에 개입하지 말고** 자율적으로 동작하도록
두세요. 다음 로그가 출력되면 실행이 완료된 것입니다:
"Target of 3 shuttlecocks deposited - stopping."

### GUI (선택사항 - 녹화/스크린샷 용도)

시뮬레이션은 기본적으로 최적의 성능을 위해 headless(GUI 없이) 모드로
실행됩니다. 이는 본 환경에 맞춘 의도적인 선택입니다: 개발은 별도의 GPU가
없는 머신(소프트웨어/`llvmpipe` 렌더링만 가능)에서 진행되었고, Gazebo GUI를
계속 열어두면 시뮬레이션 속도가 너무 느려져(실시간 대비 약 0.02-0.03배, 성능
노트 참고) 반복 개발이 사실상 어려웠습니다. 따라서 headless로 실행하고
확인이 필요할 때만 GUI를 잠깐 띄우는 방식이 현실적인 해결책이었습니다.

**별도의 GPU가 있거나 더 여유 있는 사양의 머신을 사용하는 경우**, headless로
실행하는 대신 처음부터 GUI를 켜고 실행할 수 있습니다 - 이 경우 시뮬레이션이
실시간에 가깝게 유지되어 개발과 데모가 훨씬 매끄러워집니다. 이를 위해서는
`empty_world.launch.py`에서 headless 플래그를 제거하거나, headless 실행 후
`ign gazebo -g`로 GUI를 별도로 붙이는 대신 `ign gazebo -s`(headless) 대신
일반적인 `ign gazebo` 명령으로 바로 실행하면 됩니다.

현재(headless 우선) 설정에서 로봇을 시각적으로 확인하려면, 별도의 터미널에서
GUI 창을 엽니다:

```bash
ign gazebo -g
```

**주의:** GPU 없이 소프트웨어 렌더링만 사용하는 환경에서는 GUI를 켤 경우
시뮬레이션 속도가 실시간 대비 약 0.02-0.03배로 떨어집니다. 스크린샷/녹화
용도로만 잠깐 켰다가 닫아 속도를 복원하세요. GUI를 닫아도 로봇은 headless
상태로 계속 동작합니다.

### 문제 해결

| 문제 | 해결 방법 |
| --- | --- |
| 노드 간 디스커버리 실패 / TF 누락 | 머신을 재부팅하세요. 남아있는 `/dev/shm/fastrtps_*` 파일이 DDS를 막고 있을 수 있습니다. |
| AMCL pose가 잘못된 좌표를 반환 | 초기 pose를 재퍼블리시(`--times 10`)하고 30초 기다린 후 다시 확인하세요. |
| scoop_follower가 시작 시 크래시 | Gazebo가 아직 준비되지 않은 상태입니다. 언도킹이 완료될 때까지 기다린 후 재시도하세요. |
| Nav2에서 "No valid trajectories" 반복 출력 | 초기 내비게이션 중에는 정상적인 현상입니다. Nav2 recovery(spin + clear costmap)가 자동으로 해결합니다. |
| seeker가 음수 좌표로 이동 | AMCL이 드리프트된 상태입니다. seeker를 종료하고 초기 pose를 재퍼블리시 후 확인한 다음 재시작하세요. scoop_follower의 AMCL 재시드가 이를 예방해줍니다. |
| GUI를 켰을 때 시뮬레이션이 너무 느림 | GUI(`ign gazebo -g` 창)를 닫으세요. 시뮬레이션은 headless로 계속 동작합니다. |
| 노드 실행 시 `PackageNotFoundError` 발생 | 다시 소싱하세요: `source ~/shuttler_ws/install/setup.bash` |

### 성능 노트

- **Headless** (기본값): 실시간 대비 약 0.2-0.4배, 전체 실행에 약 15-25분 소요
- **GUI 사용 시**: 실시간 대비 약 0.02-0.03배 - 시각적 확인 용도로만 짧게 사용
- 항상 매 실행 전 재부팅하여 안정성을 확보

## 데모 시나리오

월드에는 코트의 한쪽 절반에 셔틀콕 3개가 배치되어 있습니다:

- `shuttlecock_1` 위치 (3.0, 0.8)
- `shuttlecock_5` 위치 (4.0, 1.5)
- `shuttlecock_3` 위치 (6.0, 3.0)

로봇은 (1.0, 0.0)에서 출발하여 각 셔틀콕을 탐지·이동·수거한 후, NE 코너의
드롭오프 존(7.3, 4.3)에 배치 단위로 deposit합니다. 3개 모두 deposit되면
파이프라인이 종료됩니다.

## 진행 상태

탐지 → 3D 위치추정 → 내비게이션 → scoop 수거 → 배치 deposit → 탐색 복구로
이어지는 전체 자율 파이프라인이 headless 시뮬레이션 환경에서 end-to-end로
검증되었으며, 셔틀콕 3개를 모두 성공적으로 수거 및 deposit하였습니다.

## 한계 및 향후 개선 방향

### 현재 한계

- **소프트웨어 렌더링 한정**: GPU 없이 `llvmpipe`로만 시뮬레이션이 동작하여, headless 기준 실시간 대비 약 0.2-0.4배, GUI 사용 시 약 0.02-0.03배로 속도가 제한됨 - 셔틀콕 3개 기준 전체 실행에 15-25분의 실제 시간이 소요됨
- **scoop/hopper 부착이 물리적이 아닌 이동학적(kinematic) 방식**: scoop과 hopper 모델은 물리 조인트로 연결된 것이 아니라 로봇을 따라 텔레포트되는 방식이므로, 환경과의 현실적인 상호작용(예: 충돌 기반 수거)이 불가능함
- **HSV 기반 탐지의 한계**: 탐지기는 시뮬레이션 내 셔틀콕 재질에 맞춰진 고정된 주황색 HSV 범위에 의존하므로, 재학습 없이는 실제 환경의 조명, 그림자, 셔틀콕 색상 변화에 일반화되기 어려움
- **고정된 셔틀콕 위치**: collector는 셔틀콕의 ground-truth 위치를 동적으로 감지하는 대신 월드 파일에 정의된 스폰 좌표를 기준으로 추적하므로, 물리적 힘이나 외부 요인으로 위치가 이동된 셔틀콕을 처리할 수 없음
- **반쪽 코트 시나리오로 한정**: 셔틀콕 3개와 드롭오프 존이 모두 코트의 같은 절반에 위치하여, 로봇이 네트를 넘거나 풀코트 레이아웃을 처리하는 상황을 다루지 않음
- **동적 객체에 대한 회피 기능 없음**: Nav2의 costmap은 정적인 SLAM 맵만을 고려하므로, 코트 위의 다른 로봇이나 사람은 회피 대상에 포함되지 않음

### 향후 개선 방향

- **딥러닝 기반 탐지**: HSV 필터링 대신 학습된 객체 탐지 모델(예: YOLOv8)을 도입하여 다양한 조명 및 배경 조건에서도 견고하게 동작하는 실제 환경용 셔틀콕 인식 구현
- **물리적 scoop 조인트**: scoop을 접촉 센서가 포함된 Gazebo 조인트로 부착하여, 텔레포트가 아닌 실제 물리 상호작용을 통해 수거가 이루어지도록 개선
- **동적 셔틀콕 추적**: 고정된 스폰 좌표 대신 비전 파이프라인을 활용해 셔틀콕의 지면 위치를 지속적으로 추적
- **풀코트 지원**: 다수의 gather point, 네트 횡단 내비게이션, 그리고 정규 규격 코트에 더 많은 셔틀콕을 위한 커버리지 기반 탐색 전략 추가
- **GPU 가속 렌더링**: 디스크리트 GPU가 탑재된 머신에서 실행하여 실시간 시뮬레이션을 달성, 더 빠른 반복 개발과 현실적인 센서 노이즈 구현 가능
- **실제 로봇 배포**: 실제 OAK-D 카메라를 장착한 물리적 TurtleBot4에 파이프라인을 이식하여 실제 배드민턴 코트 환경에서 탐지 및 내비게이션 검증
