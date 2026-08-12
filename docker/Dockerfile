# =============================================================================
# ecza-robotu — Multi-stage Dockerfile
# Base: osrf/ros:humble-ros-base (~500 MB, headless)
#
# Stages:
#   deps     — installs rosdep dependencies (cached layer)
#   build    — colcon build with --merge-install into /ws/install
#   runtime  — minimal runtime image, non-root rosuser (UID 1000)
# =============================================================================

# ─────────────────────────────────────────────────────────────────────────────
# Stage 1: deps — resolve and install all rosdep dependencies
# ─────────────────────────────────────────────────────────────────────────────
FROM ros:humble-ros-base AS deps

SHELL ["/bin/bash", "-c"]

WORKDIR /ws

# ── rf2o_laser_odometry (built from source — no arm64 apt package) ──────────
# Laser-scan odometry. Fused into the EKF as an absolute x/y/yaw source so the
# state estimate no longer depends on the (known-miscalibrated) wheel encoders.
# Same arrangement MentorPi uses: driver/controller/config/ekf.yaml there has
# odom0=odom_raw (encoder velocities) + odom1=odom_rf2o (laser pose).
#
# The sed patch is REQUIRED, not cosmetic. rf2o only processes scans while
# GT_pose_initialized is true, and that flag starts false whenever
# init_pose_from_topic is non-empty — it only flips after one message arrives
# on that topic. The upstream default is "/base_pose_ground_truth", which
# nothing in this stack publishes, so rf2o silently discards every scan and
# logs "Waiting for laser_scans" forever. rcl rejects an empty value in
# `-p name:=`, so the default has to be fixed at compile time.
#
# Cloned BEFORE `COPY src/` so this layer stays cached when workspace sources
# change (COPY merges into src/, it does not replace the directory).
RUN git clone --depth 1 --branch ros2 \
        https://github.com/MAPIRlab/rf2o_laser_odometry.git \
        src/rf2o_laser_odometry && \
    sed -i 's|"init_pose_from_topic", "/base_pose_ground_truth"|"init_pose_from_topic", ""|' \
        src/rf2o_laser_odometry/src/CLaserOdometry2DNode.cpp && \
    grep -q '"init_pose_from_topic", ""' \
        src/rf2o_laser_odometry/src/CLaserOdometry2DNode.cpp

# Copy source packages so rosdep can inspect package.xml files.
# src/ may be empty during initial scaffold; a placeholder keeps the COPY valid.
COPY src/ src/

# cmake_modules skip-key: rf2o declares it but it is not packaged for Humble;
# the CMake config it wants (FindEigen) is provided by the eigen3_cmake_module
# that ships with the ROS 2 desktop deps already present in this base image.
RUN apt-get update -qq && \
    rosdep update --rosdistro humble && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro humble \
        -y \
        --skip-keys "micro_ros_agent ament_python cmake_modules" && \
    rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────────────────────────────────────────
# Stage 2: build — compile with colcon using --merge-install
# Using --merge-install (NOT --symlink-install) so the runtime stage can copy
# only /ws/install without needing the build/ directory.
# ─────────────────────────────────────────────────────────────────────────────
FROM deps AS build

SHELL ["/bin/bash", "-c"]

WORKDIR /ws

RUN source /opt/ros/humble/setup.bash && \
    colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_cohesion+ || \
    # If src/ is empty there is nothing to build; create an empty install tree
    # so the runtime COPY succeeds.
    mkdir -p /ws/install

# ─────────────────────────────────────────────────────────────────────────────
# Stage 3: runtime — minimal image, only /ws/install copied from build stage
# ─────────────────────────────────────────────────────────────────────────────
FROM ros:humble-ros-base AS runtime

SHELL ["/bin/bash", "-c"]

# Copy only the merged install tree — build/ and src/ are discarded.
COPY --from=build /ws/install /ws/install

# kisak-mesa PPA: rviz2 (the only GL/Mesa consumer in this image) hits a
# reproducible "libGL error: failed to create drawable" -> SIGSEGV with
# Ubuntu 22.04's stock Mesa 23.2.1 under both llvmpipe and the RPi5's real
# V3D driver. ROS2 Humble requires staying on Ubuntu 22.04, so we can't
# rebase the whole image, but a newer Mesa can be layered on top via this
# well-established backport PPA. Only affects rviz (no other service uses
# GL), so low risk to the rest of the stack.
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends software-properties-common && \
    # kisak-mesa's main channel dropped Jammy (22.04) support entirely, so
    # it silently offers nothing newer than stock. kisak-mesa/turtle is the
    # frozen/stable variant maintained specifically for older LTS releases
    # like this one — use that instead.
    add-apt-repository -y ppa:kisak/turtle && \
    apt-get update -qq && \
    rm -rf /var/lib/apt/lists/*

# Install apt-sourced ROS2 packages that are not part of the workspace src/.
# joy:                  joy_linux node for gamepad input (joy service)
# diagnostic-aggregator: aggregates /diagnostics (diagnostics service)
# rplidar-ros:          RPLidar A2M12 driver (lidar service)
# nav2:                 autonomous navigation stack (navigation service)
# slam-toolbox:         online SLAM for map building
# robot-localization:   EKF sensor fusion (encoder odom + IMU gyro) for ekf service
# python3-smbus/i2c-tools: GY-85 IMU driver (imu service) reads over I2C
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
        ros-humble-joy \
        ros-humble-joy-linux \
        ros-humble-diagnostic-aggregator \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
        # Explicit so apt resolves these from kisak-mesa (newer) rather
        # than just satisfying rviz2's minimum version requirement from
        # the older Ubuntu 22.04 repos.
        libgl1-mesa-dri \
        libglx-mesa0 \
        libegl-mesa0 \
        # Qt display backends: wayland-native + XWayland (xcb) fallback
        qtwayland5 \
        libxcb-xinerama0 \
        libxcb-icccm4 \
        libxcb-image0 \
        libxcb-keysyms1 \
        libxcb-randr0 \
        libxcb-render-util0 \
        libxcb-xkb1 \
        libxkbcommon-x11-0 \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-mecanum-drive-controller \
        ros-humble-topic-based-ros2-control \
        ros-humble-controller-manager \
        ros-humble-rplidar-ros \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-nav2-rviz-plugins \
        ros-humble-slam-toolbox \
        ros-humble-robot-localization \
        python3-smbus \
        i2c-tools \
        # Xvfb + x11vnc: self-contained virtual X server for rviz2,
        # bypassing the host's XWayland/labwc compositor entirely. That
        # path had a persistent, unfixable "failed to create drawable"
        # GLX crash rooted in cross-process DRI3 buffer-sharing between
        # the container and the host compositor. Xvfb needs no external
        # compositor negotiation at all — same well-tested pattern used
        # for headless CI GL rendering.
        xvfb \
        x11vnc && \
    rm -rf /var/lib/apt/lists/*

# Create non-root operator user UID 1000.
# privileged: true is only needed on the joy service; all other services run
# as rosuser for least-privilege operation.
RUN useradd --uid 1000 --create-home --shell /bin/bash rosuser

# Validation: confirm rosuser was created (fails build if useradd silently erred)
RUN id rosuser

# Copy entrypoint that sources ROS2 overlays before exec-ing the command.
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ws

USER rosuser

ENTRYPOINT ["/entrypoint.sh"]
