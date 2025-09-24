# -----------------------------------------------------------
# Fire Drone RL Sim — ROS 2 Jazzy (ros-base) + Gazebo (Harmonic)
# -----------------------------------------------------------
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC

# Base tools & ROS helpers
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl wget gnupg lsb-release git \
    build-essential cmake ninja-build pkg-config \
    python3 python3-dev python3-pip python3-venv \
    python3-colcon-common-extensions python3-vcstool \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------
# Gazebo Sim (Harmonic only on Noble)
# -----------------------------------------------------------
ARG GZ_DISTRO=harmonic
RUN wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-${GZ_DISTRO} \
      libogre-next-dev ogre-next-tools libgl1-mesa-dri mesa-utils libvulkan1 \
      libx11-6 libxkbcommon-x11-0 libxcb1 libxrandr2 libxrender1 libxi6 libxext6 libxfixes3 libxxf86vm1 \
    && rm -rf /var/lib/apt/lists/*

# ROS ↔ Gazebo bridge utilities (Jazzy + Harmonic)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
 && rm -rf /var/lib/apt/lists/*

# (Optional) PX4 build/runtime deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    zip unzip tar rsync \
    clang clang-tidy \
    genromfs xsltproc \
    python3-empy python3-jinja2 python3-toml python3-numpy python3-yaml \
    libeigen3-dev libopencv-dev protobuf-compiler \
 && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------
# Python: create venv + install helpers (avoid PEP 668)
# -----------------------------------------------------------
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --upgrade --no-cache-dir pip setuptools wheel && \
    /opt/venv/bin/pip install --no-cache-dir \
      kconfiglib "empy<4" jinja2 numpy packaging pyserial toml pyyaml \
      psutil jsonschema pandas future pymavlink mavproxy \
      lark pyros-genmsg pyros-genpy catkin_pkg rosdistro rospkg

# Some PX4 helper scripts call /usr/bin/python3 directly — add a tiny shim
RUN PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install --no-cache-dir \
      pyros-genmsg pyros-genpy lark future jinja2 "empy<4"

# Make the venv the default Python for all sessions and builds
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:${PATH}"
ENV COLCON_PYTHON_EXECUTABLE=/opt/venv/bin/python3
ENV PX4_CMAKE_ARGS="-DPython3_EXECUTABLE=/opt/venv/bin/python3 -DPYTHON_EXECUTABLE=/opt/venv/bin/python3"

# -----------------------------------------------------------
# Prepare workspace & clone PX4 (pinned)
# -----------------------------------------------------------
RUN mkdir -p /repo/ws/src && git config --system --add safe.directory '*'
WORKDIR /repo/ws

# Pin PX4 to a known-good tag for Noble/Jazzy/Harmonic
ARG PX4_TAG=v1.16.0
RUN git clone --depth 1 --branch ${PX4_TAG} https://github.com/PX4/PX4-Autopilot.git /repo/ws/src/px4 && \
    cd /repo/ws/src/px4 && git submodule update --init --recursive

# Add overrides directly into rcS so they always run (and save)
RUN bash -lc ' \
  cd /repo/ws/src/px4 && \
  mkdir -p ROMFS/px4fmu_common/init.d-posix && \
  { \
    echo ""; \
    echo "# --- Fire Drone sim overrides (appended by Docker build) ---"; \
    echo "# Disable magnetometer requirement (no mag in sim)"; \
    echo "param set SYS_HAS_MAG 0"; \
    echo "param set EKF2_MAG_CHECK 0"; \
    echo ""; \
    echo "# Don’t demand RC input (drive via MAVLink)"; \
    echo "param set COM_RC_IN_MODE 1"; \
    echo ""; \
    echo "# Data link loss action: Disabled for sim bringup"; \
    echo "param set NAV_DLL_ACT 0"; \
    echo ""; \
    echo "# (Optional) Pin IMU IDs if needed:"; \
    echo "# param set CAL_ACC0_ID 1310988"; \
    echo "# param set CAL_GYRO0_ID 1310988"; \
    echo ""; \
    echo "# Persist to parameters.bson so next run starts clean"; \
    echo "param save"; \
  } >> ROMFS/px4fmu_common/init.d-posix/rcS'


# Gazebo env
ENV GZ_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins
ENV GZ_RENDER_ENGINE=ogre2
ENV GZ_SIM_RESOURCE_PATH=/repo/ws/src:/repo/ws/install/share:/repo/ws/src/px4/Tools/simulation/gz/models

# QoL: source env on shell
RUN echo 'source /opt/ros/jazzy/setup.bash || true' >> /root/.bashrc && \
    echo '[ -f /repo/ws/install/setup.bash ] && source /repo/ws/install/setup.bash' >> /root/.bashrc && \
    echo 'export GZ_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:${GZ_PLUGIN_PATH:-}' >> /root/.bashrc && \
    echo 'export GZ_RENDER_ENGINE=ogre2' >> /root/.bashrc && \
    echo 'export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+${GZ_SIM_RESOURCE_PATH}:}/repo/ws/src:/repo/ws/install/share:/repo/ws/src/px4/Tools/simulation/gz/models"' >> /root/.bashrc && \
    echo 'export COLCON_PYTHON_EXECUTABLE=/opt/venv/bin/python3' >> /root/.bashrc && \
    echo 'export PX4_CMAKE_ARGS="-DPython3_EXECUTABLE=/opt/venv/bin/python3 -DPYTHON_EXECUTABLE=/opt/venv/bin/python3"' >> /root/.bashrc



CMD ["/bin/bash"]

