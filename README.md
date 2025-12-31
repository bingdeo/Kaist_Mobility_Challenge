# SMYD – Mobility Challenge Setup / Run Guide

현재 구현 상태(1-2):
- 각 CAV는 **자기 경로 follower**로 주행한다.
- 각 CAV는 **zone_id / in_danger / eta**를 계산해서 **V2V 토픽으로 publish**합니다.
- Domain Bridge로 서로의 V2V 토픽이 **/peer/...**로 전달되는지 확인 가능합니다.
- **감속(yield) 로직은 아직 미구현**입니다. (V2V 통신 + zone/ETA 산출 검증 단계)

---

## 0) 폴더/파일 구성

패키지: `smyd`

주요 파일:
- `config/v2v_bridge.yaml` : Domain Bridge 설정
- `waypoints/path_p1_2_cav1.json` : cav1 경로
- `waypoints/path_p1_2_cav2.json` : cav2 경로
- `tools/zone_database.json` : zone/conflict map (monitor/danger 판단용) 만들 때 사용한 파일은 'tools/zone_generator.py'

실행 노드:
- `p1_2_follower_cav1`
- `p1_2_follower_cav2`

---

## 1) 설치/빌드

### 1-1. 팀원 사용(필수)
본인 Mobility_Challenge_Simulator 워크스페이스의 `src/` 아래에 clone:

```bash
cd ~/Mobility_Challenge_Simulator/src
git clone https://github.com/InHyunseo/kaist_project_smyd smyd
````

빌드:

```bash
cd ~/Mobility_Challenge_Simulator
colcon build --symlink-install
```

### 1-2. 터미널 공통 source

새 터미널을 열 때마다 아래 2줄을 실행해야 합니다.

```bash
source /opt/ros/foxy/setup.bash
source ~/Mobility_Challenge_Simulator/install/setup.bash
```

---

## 2) 도메인 구성(현재 운용)

* **Domain 100**: Simulator launch
* **Domain 99** : Domain Bridge 실행
* **Domain 1**  : CAV1 follower 실행
* **Domain 2**  : CAV2 follower 실행
* 추가로 통신 확인용 echo 터미널 2개:

  * Domain 1에서 `/peer/cav2/v2v_state` echo
  * Domain 2에서 `/peer/cav1/v2v_state` echo

---

## 3) Simulator 실행 (Domain 100)

> 팀에서 쓰는 simulator launch 파일로 실행합니다. 아래는 “도메인만 100으로 두고 launch한다”는 원칙 예시입니다.

```bash
export ROS_DOMAIN_ID=100
ros2 launch ros2 launch simulator_launch simulator_launch.py
```

---

## 4) Domain Bridge 실행 (Domain 99)

```bash
export ROS_DOMAIN_ID=99
CFG="$(ros2 pkg prefix smyd)/share/smyd/config/v2v_bridge.yaml"
ros2 run domain_bridge domain_bridge "$CFG"
```

### 4-1. v2v_bridge.yaml 내용(참고)

브릿지는 아래를 전달합니다.

* Domain 1 → Domain 2

  * `/cav1/v2v_state` → (Domain2에서) `/peer/cav1/v2v_state`
* Domain 2 → Domain 1

  * `/cav2/v2v_state` → (Domain1에서) `/peer/cav2/v2v_state`

---

## 5) CAV 실행 (Domain 1 / Domain 2)

### 5-1) CAV1 실행 (Domain 1)

```bash
export ROS_DOMAIN_ID=1

ros2 run smyd p1_2_follower_cav1 --ros-args \
  -p waypoints_json:=$(ros2 pkg prefix smyd)/share/smyd/waypoints/path_p1_2_cav1.json \
  -p conflict_map_json:=$(ros2 pkg prefix smyd)/share/smyd/tools/zone_database.json \
  -p v_ref:=0.6 \
  -p w_max:=5.0 \
  -p my_share_topic:=/cav1/v2v_state \
  -p peer_share_topic:=/peer/cav2/v2v_state
```

### 5-2) CAV2 실행 (Domain 2)

```bash
export ROS_DOMAIN_ID=2

ros2 run smyd p1_2_follower_cav2 --ros-args \
  -p waypoints_json:=$(ros2 pkg prefix smyd)/share/smyd/waypoints/path_p1_2_cav2.json \
  -p conflict_map_json:=$(ros2 pkg prefix smyd)/share/smyd/tools/zone_database.json \
  -p v_ref:=0.6 \
  -p w_max:=5.0 \
  -p my_share_topic:=/cav2/v2v_state \
  -p peer_share_topic:=/peer/cav1/v2v_state
```

---

## 6) V2V 통신 확인 (topic echo)

### 6-1) Domain 1에서 CAV2 peer 상태 확인

```bash
export ROS_DOMAIN_ID=1
ros2 topic echo /peer/cav2/v2v_state
```

### 6-2) Domain 2에서 CAV1 peer 상태 확인

```bash
export ROS_DOMAIN_ID=2
ros2 topic echo /peer/cav1/v2v_state
```

---

## 7) V2V 메시지 필드 의미 (중요)

현재 V2V 메시지 타입은 **`geometry_msgs/msg/AccelStamped`**를 “컨테이너”로 재활용합니다.
그래서 echo 결과에 `accel.linear.x/y/z` 형태로 보이지만, 의미는 아래처럼 **우리가 정한 규약**입니다.

* `accel.linear.x`  → **zone_id**

  * zone에 해당 없으면 `-1`
* `accel.linear.y`  → **in_danger** (0 또는 1)

  * conflict 구간 내부(점유)면 `1`
  * monitor 구간이면 `0`
* `accel.linear.z`  → **eta** (초, s)

  * monitor 구간에서 conflict_start까지 남은 거리/속도로 ETA 계산
  * zone 밖이면 `1e9` (의미 없음)
* `header.stamp`    → 송신 시각

> 참고: `accel.linear.x` 같은 “필드 이름”은 메시지 정의(`AccelStamped`)가 고정이라 바꿀 수 없습니다.
> 헷갈림을 줄이려면 추후 `smyd_msgs/msg/V2VState` 같은 커스텀 msg로 교체 가능합니다.

---

## 8) 제어 토픽 `/Accel` (시뮬레이터 규격)

시뮬레이터에 실제로 주는 주행 명령은 **`/Accel`** 입니다.
타입: `geometry_msgs/msg/Accel`

이 대회 시뮬레이터에서는 관례적으로 아래 의미로 사용됩니다:

* `linear.x`  : 목표 전진 속도 (m/s)
* `angular.z` : 목표 요레이트 (rad/s)

확인:

```bash
export ROS_DOMAIN_ID=1
ros2 topic echo /Accel
```

> 주의: V2V 상태 토픽(`/cav*/v2v_state`, `/peer/.../v2v_state`)과 `/Accel`은 **완전히 별개**입니다.
> 이름이 Accel/AccelStamped라 비슷해 보일 뿐, 하나는 “제어”, 하나는 “통신 상태”입니다.

---

## 9) 지금 구현된 것 vs 미구현(다음 단계)

### 구현됨

* Pure Pursuit follower(각자 경로)
* conflict_map 기반 zone 판별(기본)
* zone_id / in_danger / eta V2V publish
* bridge를 통해 `/peer/...`로 수신 확인 가능

### 미구현(해야 할 것)

* **감속(yield) 결정 로직**

  * 같은 zone_id에서 ETA 비교로 양보 차량 결정
  * tie-break (예: cav1 우선)
  * fail-safe (peer timeout 시 안전하게 기본 동작)
* 실제 감속 제어

  * `/Accel` publish 시 `v_cmd`를 낮추는 로직 적용

---

## 10) 트러블슈팅

### 10-1) peer 토픽이 안 보임

1. Bridge가 떠있는지 확인 (Domain 99)
2. `v2v_bridge.yaml`의 토픽/타입 일치 확인
3. 각 터미널에서 `source ~/Mobility_Challenge_Simulator/install/setup.bash` 했는지 확인

### 10-2) conflict_map 파일 못 찾음

설치 경로 확인:

```bash
ls $(ros2 pkg prefix smyd)/share/smyd/tools/conflict_map_p1_2.json
```

### 10-3) “echo가 보이는데 값이 -1, 1e9만 나온다”

* 차량이 아직 monitor zone에 들어가지 않았거나,
* conflict_map의 인덱스 기준과 현재 follower의 idx 매칭이 어긋난 경우입니다.
  (추후 zone 판별 로직/인덱스 확인 필요)

---

## 11) 실행 순서 요약(필수 터미널 5개)

1. **Domain 100**: simulator launch
2. **Domain 99** : domain_bridge 실행
3. **Domain 1**  : cav1 follower 실행
4. **Domain 2**  : cav2 follower 실행
5. **Domain 1/2**: 각각 peer 토픽 echo로 통신 확인(2개 터미널 권장)

```
```
