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

# Copy source packages so rosdep can inspect package.xml files.
# src/ may be empty during initial scaffold; a placeholder keeps the COPY valid.
COPY src/ src/

RUN apt-get update -qq && \
    rosdep update --rosdistro humble && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro humble \
        -y \
        --skip-keys "micro_ros_agent ament_python" && \
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

# Install apt-sourced ROS2 packages that are not part of the workspace src/.
# joy:                  joy_linux node for gamepad input (joy service)
# diagnostic-aggregator: aggregates /diagnostics (diagnostics service)
RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
        ros-humble-joy \
        ros-humble-joy-linux \
        ros-humble-diagnostic-aggregator \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-mecanum-drive-controller \
        ros-humble-topic-based-ros2-control \
        ros-humble-controller-manager && \
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
