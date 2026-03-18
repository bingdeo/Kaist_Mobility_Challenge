FROM ros:foxy-ros-base-focal

SHELL ["/bin/bash", "-c"]


# =========================
# 1) Base (ROS 2 Foxy) + simulator dependencies
# =========================
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy
ENV ROS_WS=/ws
WORKDIR ${ROS_WS}

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    pkg-config \
    libeigen3-dev \
    libglm-dev \
    nlohmann-json3-dev \
    # libyaml-cpp-dev \
    # libx11-dev \
    # libxext-dev \
    # libxrandr-dev \
    # libxcursor-dev \
    # libxfixes-dev \
    # libxi-dev \
    # libudev-dev \
    # libgl-dev \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || true \
  && rosdep update

# =========================
# 2) simulator: install SDL3
# =========================
# RUN git clone --branch release-3.2.2 --depth 1 https://github.com/libsdl-org/SDL.git /tmp/SDL \
#   && cmake -S /tmp/SDL -B /tmp/SDL/build \
#       -DCMAKE_BUILD_TYPE=Release \
#       -DSDL_STATIC=OFF \
#       -DSDL_SHARED=ON \
#       -DSDL_TESTS=OFF \
#   && cmake --build /tmp/SDL/build -j"$(nproc)" \
#   && cmake --install /tmp/SDL/build \
#   && ldconfig \
#   && rm -rf /tmp/SDL

RUN mkdir -p ${ROS_WS}/src

# =========================
# 3) Simulator packages
#    - Copy only package.xml
# =========================
# COPY Mobility_Challenge_Simulator/src/communication_manager/package.xml ${ROS_WS}/src/communication_manager/package.xml
COPY Mobility_Challenge_Simulator/src/domain_bridge/package.xml ${ROS_WS}/src/domain_bridge/package.xml
# COPY Mobility_Challenge_Simulator/src/hv_handler/package.xml ${ROS_WS}/src/hv_handler/package.xml
# COPY Mobility_Challenge_Simulator/src/scene_srv/package.xml ${ROS_WS}/src/scene_srv/package.xml
# COPY Mobility_Challenge_Simulator/src/simulator/package.xml ${ROS_WS}/src/simulator/package.xml
# COPY Mobility_Challenge_Simulator/src/simulator_launch/package.xml ${ROS_WS}/src/simulator_launch/package.xml

# =========================
# 4) packages
# =========================
# COPY pkg_p1_1/package.xml ${ROS_WS}/src/pkg_p1_1/package.xml
# COPY pkg_p1_2/package.xml ${ROS_WS}/src/pkg_p1_2/package.xml
# COPY pkg_p2/package.xml ${ROS_WS}/src/pkg_p2/package.xml
COPY pkg_p3/package.xml ${ROS_WS}/src/pkg_p3/package.xml

# =========================
# 5) Install dependencies with rosdep
# =========================
RUN apt-get update \
  && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
  && rm -rf /var/lib/apt/lists/*

# =========================
# 6) Copy sources
# =========================
# simulator packages
# COPY Mobility_Challenge_Simulator/src/ ${ROS_WS}/src/

# Example packages (pkg_example_*)
# # TODO (TEAM): pkg_example_*를 참가팀 패키지로 교체
# COPY pkg_p1_1/ ${ROS_WS}/src/pkg_p1_1/
# COPY pkg_p1_2/ ${ROS_WS}/src/pkg_p1_2/
# COPY pkg_p2/ ${ROS_WS}/src/pkg_p2/
COPY pkg_p3/ ${ROS_WS}/src/pkg_p3/
COPY Mobility_Challenge_Simulator/src/domain_bridge/ ${ROS_WS}/src/domain_bridge/


# TODO (TEAM): Copy your team packages source code
# COPY <YOUR_PKG>/ ${ROS_WS}/src/<YOUR_PKG>/
# COPY src/ ${ROS_WS}/src/   # If you keep your packages under ./src

# COPY Mobility_Challenge_Simulator/profile*.json ${ROS_WS}/
# # 2) 끝 공백 버전(profile1.json␠)도 같이 생성
# RUN for i in 1 2 3 4; do \
#       if [ -f "/ws/profile${i}.json" ]; then \
#         cp -f "/ws/profile${i}.json" "/ws/profile${i}.json "; \
#       fi; \
#     done

COPY entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --merge-install \
       --packages-select pkg_p3 domain_bridge\
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

ENV ROS_LOCALHOST_ONLY=0
ENTRYPOINT ["/entrypoint.sh"]
