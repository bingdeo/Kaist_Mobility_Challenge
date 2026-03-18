#!/usr/bin/env bash
set -euo pipefail

# ROS setup scripts may reference variables that are unset; avoid `set -u` failures while sourcing.
set +u
source /opt/ros/foxy/setup.bash
source /ws/install/setup.bash
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY=0

: "${RUN_MODE:=algorithm}"  # algorithm|sim (default: algorithm)

run_simulator() {
  cd /ws
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
  exec ros2 launch simulator_launch simulator_launch.py
}

run_algorithm() {
  : "${PROBLEM_ID:=1}"  # 대회 과제 번호 (예: 1|2|3)

  # =========================
  # =========================
  case "${PROBLEM_ID}" in
    1)
      exec ros2 launch pkg_p1_1 competition.launch.py problem:=1
      ;;
    2)
      exec ros2 launch pkg_p1_2 competition.launch.py problem:=2
      ;;
    3)
      exec ros2 launch pkg_p2 competition.launch.py problem:=3
      ;;
    4)
      exec ros2 launch pkg_p3 competition.launch.py problem:=4
      ;;
    *)
      echo "TODO: 참가팀 패키지 실행을 entrypoint.sh에 연결하세요" >&2
      exit 2
      ;;
  esac
}

case "${RUN_MODE}" in
  sim|simulator)
    run_simulator
    ;;
  algorithm)
    run_algorithm
    ;;
  team) # backward-compatible alias
    run_algorithm
    ;;
  *)
    echo "Invalid RUN_MODE=${RUN_MODE} (expected algorithm|sim)" >&2
    exit 2
    ;;
esac
