# Tasks: micro-ROS WiFi Agent & Hardware Integration

**Feature branch**: `004-hardware-bridge`
**Input**: `specs/004-hardware-bridge/spec.md`
**Tests**: Not requested — no test tasks included.
**Organization**: Grouped by user story; each story is independently testable.
**Note**: This feature is primarily configuration and launch integration (not new
source code); most tasks produce YAML, launch files, and documentation.

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create the hardware integration package and establish the agent
configuration skeleton.

- [ ] T001 Create ROS2 package `ecza_hardware_bridge` with `ros2 pkg create --build-type ament_cmake ecza_hardware_bridge` in `src/ecza_hardware_bridge/`
- [ ] T002 [P] Add `package.xml` with `<depend>` on `rclcpp`, `diagnostic_msgs`, `micro_ros_agent` (exec depend) in `src/ecza_hardware_bridge/package.xml`
- [ ] T003 [P] Create `src/ecza_hardware_bridge/config/agent_params.yaml` with default micro-ROS agent transport parameters: `transport: udp4`, `port: 8888`, `middleware: dds`, `verbose: 4`

**Checkpoint**: `colcon build --packages-select ecza_hardware_bridge` succeeds.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: The micro-ROS agent must be startable and reachable before any
user story can be independently tested.

- [ ] T004 Verify `micro-ros-agent` is installed on the host (`ros2 run micro_ros_agent micro_ros_agent --help` or equivalent); document the install command (`apt install ros-humble-micro-ros-agent`) in `src/ecza_hardware_bridge/README.md`
- [ ] T005 Create `src/ecza_hardware_bridge/launch/agent.launch.py` that starts `micro_ros_agent` with transport parameters loaded from `config/agent_params.yaml` (UDP port 8888, verbose 4) per FR-001, FR-002, FR-006
- [ ] T006 [P] Add `src/ecza_hardware_bridge/config/wifi_requirements.md` documenting the required WiFi environment: same subnet, recommended 2.4 GHz band, `sudo ufw allow 8888/udp` command per FR-008

**Checkpoint**: Running `ros2 launch ecza_hardware_bridge agent.launch.py` starts the micro-ROS agent listening on UDP 8888; `ss -ulnp | grep 8888` confirms the port is open.

---

## Phase 3: User Story 1 — ESP32 Appears as a ROS2 Node over WiFi (Priority: P1) 🎯 MVP

**Goal**: The micro-ROS agent is running, the ESP32 (pre-provisioned with agent
IP and port from feature 001) connects, and `/rover/esp32_firmware_node` appears
in `ros2 node list` within 10 seconds.

**Independent Test**: Launch the agent, power on the ESP32 on the same WiFi
network, wait ≤10 s, run `ros2 node list` → `/rover/esp32_firmware_node` present.
Run `ros2 topic list` → `/wheel_velocities` and `/firmware_status` present.

- [ ] T007 [US1] Document the ESP32 WiFi provisioning precondition in `src/ecza_hardware_bridge/README.md`: SSID, password, agent IP must be flashed into NVS (links to feature 001 provisioning tool) per FR-001
- [ ] T008 [US1] Add a manual verification script `scripts/verify_esp32_connection.sh` that runs `ros2 node list | grep esp32_firmware_node` with a 10 s timeout and exits 0/1 per SC-001 in `src/ecza_hardware_bridge/scripts/verify_esp32_connection.sh`
- [ ] T009 [US1] Validate that `/wheel_velocities` published from a host `ros2 topic pub` reaches the ESP32 and produces motor motion within the 150 ms budget; document this as a manual acceptance test step in `src/ecza_hardware_bridge/README.md` per SC-002, FR-003

**Checkpoint**: `ros2 node list` shows the ESP32 node within 10 s of powering on; `ros2 topic echo /firmware_status` shows status messages.

---

## Phase 4: User Story 2 — micro-ROS Agent Lifecycle Management (Priority: P2)

**Goal**: The agent is included in the bringup launch (feature 005 will compose
this), handles restarts cleanly, and the ESP32 reconnects automatically without
firmware restart.

**Independent Test**: Start the agent via launch file. Kill the agent process.
Verify ESP32 watchdog fires (motors stop within 500 ms). Restart the agent.
Verify ESP32 reconnects and `/wheel_velocities` commands flow again — all within
10 s, no manual steps.

- [ ] T010 [US2] Configure `respawn=True` and `respawn_delay=2.0` (seconds) on the `micro_ros_agent` node action in `src/ecza_hardware_bridge/launch/agent.launch.py` per FR-005, US2-AC1
- [ ] T011 [US2] Add a launch argument `agent_port` to `agent.launch.py` that overrides the default port from YAML, so the port can be changed without editing the config file per FR-006
- [ ] T012 [US2] Document the reconnection behaviour and the `respawn_delay` rationale in `src/ecza_hardware_bridge/README.md`: explain that the ESP32 firmware retries micro-ROS connection indefinitely and will reconnect within ~2–5 s of agent restart per SC-003

**Checkpoint**: Kill and restart the agent → ESP32 reconnects within 10 s without firmware restart; motors resume responding to commands.

---

## Phase 5: User Story 3 — Firmware Diagnostics via ROS2 Topic (Priority: P3)

**Goal**: The firmware status published by the ESP32 micro-ROS node on
`/firmware_status` is forwarded to `/diagnostics` as
`diagnostic_msgs/DiagnosticArray` so `rqt_robot_monitor` can display it without
an adapter node.

**Independent Test**: Launch the agent + diagnostics aggregator. Subscribe to
`/diagnostics`. Confirm ESP32 firmware data appears (≥1 Hz). Trigger a fault
flag in the firmware (disable an enable pin); confirm the fault appears in the
next `/diagnostics` message.

- [ ] T013 [US3] Create `src/ecza_hardware_bridge/src/firmware_diagnostics_node.cpp` (or `.py`): subscribes to `/firmware_status` (`std_msgs/msg/String` JSON), deserialises the fields, and re-publishes as `diagnostic_msgs/DiagnosticArray` on `/diagnostics` at pass-through rate per FR-004, US3-AC3
- [ ] T014 [US3] Add `<depend>diagnostic_msgs</depend>` and register the `firmware_diagnostics_node` as a component in `src/ecza_hardware_bridge/CMakeLists.txt`
- [ ] T015 [US3] Update `src/ecza_hardware_bridge/launch/agent.launch.py` to also start `firmware_diagnostics_node` per FR-004

**Checkpoint**: `ros2 topic echo /diagnostics` shows ESP32 status fields at ≥1 Hz; `rqt_robot_monitor` displays them without additional configuration.

---

## Phase 6: Polish & Cross-Cutting Concerns

- [ ] T016 [P] Add `src/ecza_hardware_bridge/config/multi_board_note.md` documenting the unique-node-name requirement for future multi-board setups (node name derived from MAC) per Edge Cases
- [ ] T017 [P] Add a launch argument `verbose` to `agent.launch.py` (default: 4) and a `quiet` preset (verbose: 1) for production use per FR-006
- [ ] T018 Expand `src/ecza_hardware_bridge/README.md` with: WiFi environment requirements, udev rule for USB console (flash only), quickstart steps for this package in isolation, link to feature 001 NVS provisioning tool

---

## Dependencies & Execution Order

```
Phase 1 (Setup)
    └── Phase 2 (Foundational — agent launchable)
            ├── Phase 3 (US1 — P1) 🎯 MVP [requires ESP32 firmware from 001]
            │       └── Phase 4 (US2 — P2) [respawn policy on top of US1 agent]
            ├── Phase 5 (US3 — P3) [diagnostics adapter node]
            └── Phase 6 (Polish)
```

**Cross-feature dependency**: US1 requires the ESP32 firmware (feature 001,
specifically T010–T016) to be flashed and provisioned before it can be verified.

---

## Task Count Summary

| Phase | Tasks | Parallelizable |
|-------|-------|---------------|
| Phase 1 — Setup | 3 (T001–T003) | 2 |
| Phase 2 — Foundational | 3 (T004–T006) | 1 |
| Phase 3 — US1 (P1) | 3 (T007–T009) | 0 |
| Phase 4 — US2 (P2) | 3 (T010–T012) | 0 |
| Phase 5 — US3 (P3) | 3 (T013–T015) | 0 |
| Phase 6 — Polish | 3 (T016–T018) | 2 |
| **Total** | **18** | **5** |
