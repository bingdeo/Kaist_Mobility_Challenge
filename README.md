# 문제 1-1

## Overview

단일 CAV가 폐곡선(Closed-loop) 형태의 트랙을 따라 모든 구간을 경유한 뒤 시작점으로 복귀하는 것을 목표로 한다. 경로 추종을 위해 Pure Pursuit 기반의 기하학적 조향각에 PID 제어기를 결합하여 횡방향 오차를 보정하고, 안정적인 주행을 확보한다.

---

## Dependencies

이 패키지는 ROS 2 Foxy 환경에서 동작하며, 다음의 의존성을 가진다.

### ROS 2 Packages

* **rclpy**: 노드 실행, 타이머 및 통신 관리
* **geometry_msgs**: `PoseStamped` (위치 수신), `Accel` (제어 명령 송신)
* **ament_index_python**: 패키지 경로 및 리소스(JSON 파일) 탐색

### Python Standard Libraries

* **math**: 삼각함수 연산 및 좌표 변환
* **json**: waypoint 데이터 parsing
* **os**: 파일 시스템 경로 처리

---

## Solution Approach

### 1. Waypoint Processing

전체 경로 데이터를 매 주기마다 탐색하는 비효율을 방지하기 위해 **Search Window** 를 적용하였다.

* **Window Search**: 이전 주기에서 찾은 인덱스를 기준으로 전후 일정 범위(`search_window = 600`) 내에서만 가장 가까운 점(Nearest Point)을 탐색한다.
* **Unwrapped Index (`idx_u`)**: 폐곡선 트랙에서 인덱스가 0으로 초기화되는 현상을 방지하기 위해, waypoint 인덱스를 누적 증가시키는 방식으로 주행 진척도를 연속적으로 관리한다.

---

### 2. Target Point Selection

* **Fixed Lookahead Index**: 본 문제에서는 차량 속도가 일정하게 유지되므로, 거리 기반 lookahead 계산 대신 현재 인덱스를 기준으로 일정 개수(+60)의 waypoint를 목표점으로 선택하여 연산 복잡도를 줄였다.
* **Continuous Index Handling**: 트랙은 폐곡선 구조이므로 주행이 진행됨에 따라 waypoint 인덱스가 리스트의 길이(`len`)를 초과할 수 있다.
  이때 목표점이 시작점으로 불연속적으로 이동하는 것을 방지하기 위해, 인덱스는 계속 증가시키되 실제 좌표 접근 시에는 나머지 연산(Modulo, `% len`)을 적용하여 주행의 연속성을 유지한다.
* **Coordinate Transformation**: 선택된 목표점의 Global 좌표를 차량 기준의 Local 좌표계(`x_r`, `y_r`)로 변환하여 제어 입력 계산에 사용한다.

---

### 3. 경로 추종

최종 조향각 `delta`는 Pure Pursuit 기반 조향각과 PID 보정항의 합으로 결정된다.

```math
\delta = \delta_{ff} + \delta_{fb}
```

* **Pure Pursuit 기반 조향각**

  * 목표점까지의 곡률(`kappa`)을 계산하여 기본 조향각을 생성한다.
  * 곡선 구간 진입 시 선제적으로 조향하여 경로 이탈을 줄이는 효과를 제공한다.
* **Feedback (PID Controller)**

  * 차량과 경로 사이의 횡방향 오차(Cross-track Error, `e_y`)를 0으로 수렴시키기 위해 PID 제어를 수행한다.
  * Pure Pursuit 단독 사용 시 발생할 수 있는 코너 구간의 안쪽 침투 또는 외측 이탈을 보정한다.
  * **Anti-Windup**: 적분항(I)의 과도한 누적을 방지하기 위해 제한값(`i_limit`)을 적용하여 제어 발산을 방지한다.

---

### 4. Actuation

계산된 조향각(`delta`)과 기준 속도(`self.v_ref`)를 Kinematic Bicycle Model에 적용하여 차량 제어에 사용되는 yaw rate(ωz) 명령으로 변환한다.

---

# 문제 1-2

## Overview

두 대의 차량(CAV1, CAV2)이 주어진 구간을 모두 경유하면서 차량 간 충돌 없이 주행하는 것을 목표로 한다. 서로 다른 Domain ID(1, 2)를 사용하는 차량들은 Domain Bridge를 통해 V2V 메시지를 교환하며, ETA(Estimated Time of Arrival)를 기반으로 종방향 속도를 조절하여 충돌을 회피한다.

---

## Dependencies

문제 1-1의 의존성에 더해 V2V 통신을 위한 다음 패키지가 추가된다.

* **domain_bridge**: 서로 다른 ROS_DOMAIN_ID 간 토픽 통신 중계
* **pkg_p1_2.msg**: `V2VState` (차량 ID, 위치, 속도, ETA, Zone 정보 등 공유)

---

## System Architecture

### Domain Isolation & Bridging

* CAV1은 **ID 1**, CAV2는 **ID 2**를 사용하며, 시뮬레이터와 분리된 **ID 99**에서 Domain Bridge가 동작한다.
* 각 차량은 자신의 주행 상태(`V2VState`)를 발신하고, 상대 차량의 상태(`peer_state`)를 수신하여 충돌 회피 의사결정에 활용한다.

---

## Solution Approach

경로 추종은 문제 1-1과 동일한 방식을 사용하며, 종방향 속도 제어는 다음의 충돌 회피 알고리즘을 따른다.

### 1. 경로 추종

* 문제 1-1과 동일한 경로 추종 및 횡방향 제어 방식을 사용한다.

---

### 2. Conflict Zone 인식

* **Zone Database**: 사전에 정의된 위험 구간 정보(`conflict_map_json`)를 로드하여 `self.conflict_map` 딕셔너리에 저장한다.
* **Localization**: `Collision_Avoidance` 모듈에서 실시간 차량 좌표(`my_x`, `my_y`)가 특정 Zone의 바운더리 내에 포함되는지를 검사하여 현재 주행 중인 `zone_id`를 갱신한다.

---

### 3. 충돌 회피 알고리즘

충돌 회피 로직은 `Collision_Avoidance.py` 모듈에서 수행된다. 각 차량은 상대 차량과 자신의 ETA를 계산하여, 해당 Conflict Zone을 더 늦게 통과할 것으로 예상되는 차량이 감속하여 양보하는 전략을 사용한다.

---

### 3-1. 안전 거리 확인 로직

* 두 차량 간 유클리드 거리가 1.5m 이상인 경우, 물리적 충돌 가능성이 낮다고 판단하여 ETA 기반 회피 로직을 수행하지 않고 기존 속도(`v_cmd`)를 유지한다.
* 충돌 위험 구간에 해당하더라도 차량 간 물리적 거리가 충분히 확보된 경우에는 불필요한 감속을 방지하여 주행 효율을 유지한다.

---

### 3-2. ETA 비교 및 양보 로직

* 각 차량은 현재 위치에서 해당 Zone의 충돌 시작 지점까지 도달하는 데 필요한 시간인 ETA를 계산하여 공유한다.
* 내 ETA(`my_eta`)와 상대 차량의 ETA(`peer_eta`)를 비교한다.

  * `my_eta < peer_eta`: 내가 더 빠르게 Zone을 통과할 것으로 예상되므로 감속 없이 주행한다.
  * `my_eta >= peer_eta`: 내가 더 늦게 도착하거나 유사한 시점에 도착할 것으로 예상되는 경우 양보를 위해 감속한다.

---

### 3-3. 속도 제어

* 양보가 필요한 경우, 상대 차량이 Zone을 통과한 이후 안전하게 진입할 수 있도록 목표 속도를 재계산한다.
* 상대 차량의 도착 시간(`peer_eta`)에 안전 마진(`+1.0s` 또는 `+1.5s`)을 더하여 나의 목표 도착 시간(`margin_time`)을 설정한다.

```math
v_{real} = \frac{my_{eta} \times v_{cmd}}{margin_{time}}
```

* 위 식은 현재 속도(`v_cmd`)로 주행할 경우의 예상 도착 시간(`my_eta`)을, 안전 마진이 포함된 목표 도착 시간(`margin_time`)에 맞추기 위해 속도를 비례적으로 감속하는 방식이다.
곱한 후, margin_time으로 나누어 정적 속도 할당이 아닌, 상황에 따라 속도가 달라지는 동적할당 방식을 사용하였다.
---

# 문제 2

## Overview

CAV가 트랙 내의 장애물을 회피하고 원래 차선으로 복귀하거나 추월을 수행하여 **최단 시간 내에 목적지에 도착**하는 것을 목표로 한다. 외곽 도로 외의 주행 불가 영역을 침범하지 않도록 다중 차선 데이터를 활용한 **State Machine** 기반의 동적 차선 변경 알고리즘을 구현하였다.

---

## Dependencies

문제 1-1의 의존성에 더해 다음 패키지가 추가된다.

### ROS 2 Packages

* **domain_bridge**: 시뮬레이터와 제어 노드(CAV) 간 토픽 통신 중계
* **pkg_p2**: 다중 차선 데이터 파일 및 제어 로직 포함

### Python Libraries

* **functools**: 콜백 함수 관리

---

## Solution Approach

### 1. 경로 추종

기본적인 경로 추종 및 횡방향 제어 로직은 문제 1-1과 동일하다.

* **Difference**: 문제 2에서는 고정된 단일 경로가 아닌, State Machine에 의해 선택된 현재 차선(Active Lane)의 waypoint를 실시간 목표점으로 사용한다.

---

### 2. Multi-Lane Data Management

장애물 회피 및 최단 시간 주행을 위해 3개의 차선 데이터를 모두 메모리에 로드하여 관리한다.

* 초기화 단계에서 1, 2, 3차선 경로 데이터를 파싱하여 리스트 형태로 저장한다.
* 사전에 검증된 차선 경로(`json`)를 사용함으로써 주행 불가 영역 침범을 원천적으로 방지한다.

---

### 3. Frenet Coordinate Estimation

트랙 내에서의 위치 인식과 차선 변경 시점 결정을 위해 Global 좌표를 Frenet 좌표계(`s`, `d`)로 변환한다.

* 현재 차량 위치를 트랙 진행 거리(`s`)와 횡방향 오차(`d`)로 표현한다.
* 계산된 `s` 값은 장애물 접근 여부 판단 및 State Machine의 트리거 조건으로 활용된다.

---

### 4. State Machine (Overtaking & Merging Strategy)

주행 거리(`s`)에 따라 주행 상태(`state`)를 전이하며, 사전에 정의된 우선순위(Priority)에 따라 목표 차선을 결정한다.

* **Trigger Condition**: 전방 차량과의 거리가 설정값(`follow_dist`)보다 가까워질 경우 차선 변경을 시도한다.
* **Lane Priority (Overtaking)**:

  1. **High Priority**: 주행 가능 시 Lane 3(Index 2) → Lane 2(Index 1) 순으로 우선 고려하여 추월 및 고속 주행을 수행한다.
  2. **Low Priority**: Lane 1(Index 0)은 Lane 2와 Lane 3이 모두 불가능한 경우에만 선택하거나, 현재 차선을 유지하며 감속하는 보수적인 전략을 적용한다.
* **Merge Constraint**: 합류 구간에 접근(`merge_approach_lock`)할 경우, 안전 확보를 위해 Lane 2로 차선을 유도하여 합류 구간에서의 충돌 위험을 줄인다.

---

### 5. Adaptive Speed Control

급격한 차선 변경이나 회피 기동 시 차량의 안정성을 확보하기 위해 조향각에 따라 목표 속도를 능동적으로 조절한다.

```math
v_{target} = \frac{v_{ref}}{1.0 + k \times |\delta|}
```

* 조향각(`delta`)이 클수록 목표 속도를 낮추어 미끄러짐이나 차량 불안정성을 방지한다.
* 직선 구간에서는 기준 속도를 유지하여 최단 시간 주행 목표를 달성한다.

---

# 문제 3

## Overview

**사지교차로 및 회전교차로** 환경에서 총 4대의 CAV가 충돌 없이 주행하는 협력 제어 기술 개발을 목표로 한다. 다수의 차량이 동시에 진입하는 상황에서 V2V 통신을 통해 주행 우선순위를 결정하고, 예측 불가능한 차량(HV)과의 상호작용을 고려하여 안전성을 확보한다.

---

## Dependencies

기본적인 의존성은 문제 1-1과 동일하다. 여기에 4대 차량 간의 독립적인 통신과 복잡한 연산을 위한 패키지가 추가되었다.

### Added ROS 2 Packages

- **domain_bridge**: 4개의 독립된 도메인(CAV 1~4)과 시뮬레이터 간의 대규모 토픽 통신 중계
- **p3_smyd_v2v**: `V2VState` (다중 차량 상태, Zone ID, ETA 공유)

## System Architecture

### Scalable V2V Network

차량 대수가 4대로 증가함에 따라 통신 간섭을 최소화하기 위해 **Multi-Instance Domain Bridge**를 구축했다.

- 각 CAV(ID 1~4)는 독립된 ROS_DOMAIN_ID를 가지며, Bridge를 통해 시뮬레이터 및 타 차량과 데이터를 격리된 상태로 교환한다.
- `all_peers` 해시맵을 통해 나를 제외한 3대 차량의 실시간 주행 정보(위치, 속도, ETA)를 동기화하여 Global 충돌 위험을 감지한다.

---

## Solution Approach

### 1. 경로 추종

기본적인 경로 추종 및 횡방향 제어 로직은 문제 1-1과 동일하다. 단, 4대의 차량이 단일 코드 베이스(`p3.py`)를 공유하되, 실행 시 주입되는 `self_id` 파라미터에 따라 고유한 역할과 우선순위를 수행한다.

---

### 2. 사지교차로 & 회전교차로 Coordination

사지교차로와 회전교차로는 다수의 차량이 얽히는 **Conflict Zone**으로 정의된다.

- **Zone Database Integration**: 교차로 진입 전, 자신이 속한 Zone ID를 식별하고 해당 구역에 진입 예정인 다른 CAV들의 ETA(도착 예정 시간)를 비교한다.
- **Priority Protocol**:
    - **Shortest ETA First**: 교차로 진입까지 남은 시간(ETA)이 더 짧은 차량이 우선적으로 통행할 권한을 갖는다.
    - **Tie-Breaker**: 진입 임계 시간(`margin_time`) 내에 여러 차량의 ETA가 비슷하게 겹칠 경우, 차량 ID 기반의 우선순위를 적용하여 동시 진입으로 인한 교착 상태를 방지한다.
    - **Yielding**: 내가 양보해야 할 경우(`yield`), 교차로 진입 전 안전 구역에서 목표 속도를 감속하여 상대 차량이 먼저 통과하도록 유도한다.

---

### 3. HV Handling

협력 제어가 불가능한 HV가 존재하는 구간에서는 센서 데이터를 기반으로 한 방어 운전 로직이 적용된다.

### 3-1. HV Intention Prediction

HV의 주행 방향 벡터 변화량인 **Slope($\theta$)** 를 실시간으로 계산하여 거동을 예측한다.

- **진입 조건**: 두 HV의 위치 간 기울기를 의미하는 `abs(hv_slope)` 값이 임계값(1.0) 미만이면 안정 상태로 판단하여 진입을 시도하고, 임계값 이상이 되면 진입을 보류한다.

### 3-2. Adaptive Gap Acceptance

HV와의 상대 거리와 위치 관계에 따라 동적으로 속도를 제어한다.

- **가속 모드**: 충돌 위험이 낮고 안전거리가 충분한 경우, 속도를 **2.0 m/s**로 높여 주행하며 합류한다.
- **Emergency Stop**: HV와의 유클리드 거리가 위험 수준(**0.6m 미만**)으로 좁혀지면 즉시 정지(`v_cmd = 0.0`)하여 측면 충돌을 방지한다.

### 3-3. Active Safety

합류 구간에서 HV가 내 차량의 후방으로 빠르게 접근하는 경우, 감속은 오히려 추돌 사고를 유발할 수 있다.

- **Escape Logic**: 내 차량이 HV보다 전방(`diff > 0`)에 있고 거리가 0.5m 미만으로 좁혀지면, **강제 가속(`v_cmd > 2.0`)** 을 수행하여 후방 추돌 위험으로부터 벗어난다.

## 환경구성

### OS
- Ubuntu 20.04 LTS (Focal Fossa)

### ROS 2
- ROS 2 Foxy Fitzroy
- 미들웨어(DDS): 기본 RMW 사용 (FastDDS)

## 폴더구조

```
/ (root)
├── Dockerfile
├── entrypoint.sh
├── README.md  # - Dependency, 코드 구조에 대한 간략한 설명
├── Mobility_Challenge_Simulator/
├── pkg_p1_1/
│   ├── launch/
│   │   └── competition.launch.py # 런치 파일
│   ├── pkg_p1_1/
│   │   └── p1_1.py # 메인 노드
│   ├── resource/
│   ├── waypoints/
│   │   └── problem1-1_path.json # 차량의 경로
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── pkg_p1_2/
│   ├── config/
│   │   └── v2v_bridge.yaml # 도메인 브릿지
│   │   └── zone_database.json # 사전 할당하는 위험 구역 DB
│   ├── launch/
│   │   └── competition.launch.py # 런치 파일
│   ├── msg/
│   │   └── VV2VState.msg # 도메인브릿지를 통해 주고받는 커스텀 생성한 메시지 양식
│   ├── pkg_p1_2/
│   │   └── p1_2.py # 메인 노드
│   │   └── Collision_Avoidance.py # 메인 노드에서 사용하는 충돌회피 로직 모듈
│   ├── resource/
│   ├── waypoints/
│   │   └── path_cav1.json # cav1의 경로
│   │   └── path_cav2.json # cav2의 경로
│   ├── CMakeLists.txt # msg(ament_cmake)를 활용하기 위해 사용
│   ├── p1_2 
│   ├── package.xml 
│   ├── setup.cfg
│   └── setup.py # .py 노드(ament_python)를 활용하기 위해 사용
├── pkg_p2/
│   ├── config/
│   │   └── p2_bridge.yaml # 도메인 브릿지
│   ├── launch/
│   │   └── competition.launch.py # 런치 파일
│   ├── msg/
│   │   └── VV2VState.msg # 도메인브릿지를 통해 주고받는 커스텀 생성한 메시지 양식
│   ├── pkg_p2/
│   │   └── p2.py # 메인 노드
│   ├── resource/
│   ├── waypoints/
│   │   └── p2_lane1.json # 1차로(가장 바깥)의 경로
│   │   └── p2_lane2.json # 2차로(중간 차로)의 경로
│   │   └── p2_lane3.json # 3차로(가장 안쪽)의 경로
│   ├── package.xml 
│   ├── setup.cfg
│   └── setup.py 
├── pkg_p1_2/
│   ├── config/
│   │   └── 3_zone_database.json # 사전 할당하는 위험 구역 DB
│   │   └── bridge_cav1.yaml # 도메인 브릿지 # 2, 3, 4 to cav1
│   │   └── bridge_cav2.yaml # 도메인 브릿지 # 1, 3, 4 to cav2
│   │   └── bridge_cav3.yaml # 도메인 브릿지 # 1, 2, 4 to cav3
│   │   └── bridge_cav4.yaml # 도메인 브릿지 # 1, 2, 3 to cav4
│   ├── launch/
│   │   └── competition.launch.py # 런치 파일
│   ├── msg/
│   │   └── V2VState.msg # 도메인브릿지를 통해 주고받는 커스텀 생성한 메시지 양식
│   ├── pkg_p3/
│   │   └── Collision_Avoidance3.py # 메인 노드에서 사용하는 충돌회피 로직 모듈
│   │   └── p3.py # 메인 노드
│   ├── resource/
│   ├── waypoints/
│   │   └── 3_cav1.json # cav1의 경로
│   │   └── 3_cav2.json # cav2의 경로
│   │   └── 3_cav3.json # cav3의 경로
│   │   └── 3_cav4.json # cav4의 경로
│   ├── CMakeLists.txt # msg(ament_cmake)를 활용하기 위해 사용
│   ├── p3 
│   ├── package.xml 
│   ├── setup.cfg
│   └── setup.py # .py 노드(ament_python)를 활용하기 위해 사용
└──
```
## docker 빌드

```bash
docker build -t team_test:latest .
```

## 시뮬레이터 실행

이 저장소에 포함된 `Mobility_Challenge_Simulator/`를 사용합니다.

1) (필수) X11 허용 

```bash
xhost +local:root
```

2) (Docker) 시뮬레이터 실행

```bash
docker run --rm -it --net=host --ipc=host \
  -e RUN_MODE=sim \
  -e ROS_DOMAIN_ID=100 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --name kmc_sim \
  team_test:latest
```

## 컨테이너 실행 형식

### 문제 1-1에 사용하는 알고리즘 (PROBLEM_ID=1)
```bash
docker run --rm --net=host --ipc=host -e PROBLEM_ID=1 team_test:latest
```

### 문제 1-2에 사용하는 알고리즘 (PROBLEM_ID=2)
```bash
docker run --rm --net=host --ipc=host -e PROBLEM_ID=2 team_test:latest
```

### 문제 2에 사용하는 알고리즘 (PROBLEM_ID=3)
```bash
docker run --rm --net=host --ipc=host -e PROBLEM_ID=3 team_test:latest
```

### 문제 3에 사용하는 알고리즘 (PROBLEM_ID=4)
```bash
docker run --rm --net=host --ipc=host -e PROBLEM_ID=4 team_test:latest
```