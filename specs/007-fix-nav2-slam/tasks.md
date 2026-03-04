# Tasks: Full Nav2 SLAM Stack — Fix All Errors

**Feature Branch**: `007-fix-nav2-slam`  
**Spec**: [spec.md](spec.md) · **Plan**: [plan.md](plan.md)  
**Total tasks**: 30  
**MVP scope**: Phase 1 + 2 + 3 (US1 — SLAM builds map, Nav2 lifecycle clean)

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Parallelisable — different files, no dependency on incomplete tasks
- **[US1/2/3]**: User story label (see spec.md)
- Exact file paths included in every task description

---

## Phase 1: Setup — Verify & Stabilise Current State

**Purpose**: Confirm the correct branch is active, all code changes are staged,
and config files are syntactically valid. All Phase 2+ work depends on a clean
baseline.

- [ ] T001 Confirm current branch is `007-fix-nav2-slam` and all nav/config changes are committed: `git branch --show-current && git status --short` — must show branch `007-fix-nav2-slam`; any modified files in `src/ecza_navigation/`, `src/ecza_description/`, `docker/`, `docker-compose.yaml`, `scripts/` must be staged or committed so they are included in the Docker build context
- [ ] T002 [P] Validate YAML syntax of both nav config files: `python3 -c "import yaml; yaml.safe_load(open('src/ecza_navigation/config/nav2_params.yaml'))" && echo nav2_params:OK` and `python3 -c "import yaml; yaml.safe_load(open('src/ecza_navigation/config/slam_params.yaml'))" && echo slam_params:OK` — both must print OK
- [ ] T003 [P] Verify `docker/Dockerfile` already contains `ros-humble-nav2-rviz-plugins` in the apt install block: `grep ros-humble-nav2-rviz-plugins docker/Dockerfile` — must return a match
- [ ] T004 [P] Verify `src/ecza_description/launch/hardware.launch.py` remappings block contains both `("/controller_manager/tf_odometry", "/tf")` and `("/controller_manager/odometry", "/odom")`: `grep -A8 "remappings=" src/ecza_description/launch/hardware.launch.py`

**Checkpoint**: Branch confirmed, YAML valid, Dockerfile and launch file baseline confirmed.

---

## Phase 2: Foundational — Deploy Config Fixes

**Purpose**: All source-file fixes are already applied. This phase verifies them,
rebuilds the Docker image to include all changes, and force-recreates the running
containers. Must be complete before any user story can be tested.

⚠️ **CRITICAL**: Nav2 lifecycle will not reach `active` state until T005–T009 are done.

- [ ] T005 [VERIFY] [P] Confirm the 8 missing BT plugins are absent from `plugin_lib_names` in `src/ecza_navigation/config/nav2_params.yaml`: `grep -c "are_error_codes\|nav_through_poses_action\|nav_to_pose_action\|speed_limit_action\|would_a_" src/ecza_navigation/config/nav2_params.yaml` — must return 0; total `bt_node` entries must be ≤ 51 (`grep -c "bt_node" src/ecza_navigation/config/nav2_params.yaml`)
- [ ] T006 [VERIFY] [P] Confirm `transform_timeout: 1.0` is set in `src/ecza_navigation/config/slam_params.yaml`: `grep "transform_timeout" src/ecza_navigation/config/slam_params.yaml` — must show `1.0`, not `0.5`
- [ ] T007 [VERIFY] [P] Confirm `base_frame_id: "base_link"` and `robot_model_type: "nav2_amcl::OmnidirectionalMotionModel"` are set in `src/ecza_navigation/config/nav2_params.yaml`: `grep "base_frame_id\|robot_model_type" src/ecza_navigation/config/nav2_params.yaml`
- [ ] T008 Rebuild Docker image with all fixes: `docker compose -f docker-compose.yaml build 2>&1 | tail -5` — verify exit code 0
- [ ] T009 Force-recreate nav-profile containers with the new image: `docker compose -f docker-compose.yaml --profile nav up -d --force-recreate navigation lidar` — then verify: `docker logs ecza-robotu-navigation-1 2>&1 | grep -c "Could not load library"` must return 0 and `docker logs ecza-robotu-navigation-1 2>&1 | grep "All requested nodes are now active"` must match

**Checkpoint**: `docker logs ecza-robotu-navigation-1` shows no `Could not load library` errors and lifecycle manager logs `All requested nodes are now active`.

---

## Phase 3: User Story 1 — SLAM Maps Environment During Navigation (Priority: P1) 🎯 MVP

**Goal**: SLAM Toolbox builds a live occupancy-grid map while the robot drives.
Map appears in RViz. No continuous error spam in logs.

**Independent Test**: Open RViz, drive robot with joystick — map tiles expand,
robot arrow tracks position, no red error boxes in RViz.

- [ ] T010 [P] [US1] Verify `/odom` topic has exactly 1 publisher (FR-004): `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && ros2 topic info /odom"` — must show `Publisher count: 1`
- [ ] T011 [P] [US1] Verify `navigation_launch.py` (not `bringup_launch.py`) is active in SLAM mode (FR-005): `docker exec ecza-robotu-navigation-1 bash -c "ps aux | grep navigation_launch"` — must show `navigation_launch.py` in the process list
- [ ] T012 [P] [US1] Verify `odom→base_link` TF is available in the navigation container within 5 s of startup: `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -8"` — must resolve without timeout
- [ ] T013 [US1] Verify SLAM Toolbox stops logging `Failed to compute odom pose` within 30 s of containers starting: `docker logs ecza-robotu-navigation-1 --since 30s 2>&1 | grep -c "Failed to compute odom"` — count must be 0
- [ ] T014 [US1] Verify `/map` topic has 1 publisher within 10 s of lidar publishing `/scan`: `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && ros2 topic info /map"` — must show `Publisher count: 1`
- [ ] T015 [US1] Verify full TF chain `map→odom→base_link→laser_frame` in navigation container (SC-005): `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 run tf2_ros tf2_echo map laser_frame 2>&1 | head -8"` — must resolve with delay < 0.5 s
- [ ] T016 [US1] Verify `/diagnostics` is being published during navigation (Constitution §V): `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /diagnostics 2>&1 | head -4"` — must show average rate > 0
- [ ] T017 [US1] Drive the robot ~1 m with the joystick and confirm in RViz: new map cells appear, robot pose arrow tracks physical motion. Quantitative check: `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /map 2>&1 | head -3"` — must show rate > 0.1 Hz during motion

**Checkpoint (US1 done)**: Map visible in RViz, no ERROR logs, TF chain intact, `/odom` has 1 publisher, diagnostics publishing. US1 independently deliverable.

---

## Phase 4: User Story 2 — Nav2 Goal Navigation from RViz (Priority: P2)

**Goal**: Operator places a Nav2 Goal in RViz; robot drives autonomously to it.
Nav2 Panel shows navigation state (Running → Succeeded).

**Independent Test**: With partial map built, click Nav2 Goal in RViz toolbar,
place goal on known-free area — robot moves within 3 s.

- [ ] T018 [P] [US2] Verify Nav2 action servers are active (SC-001): `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && ros2 action list | grep navigate"` — must show `/navigate_to_pose` and `/navigate_through_poses`
- [ ] T019 [US2] Verify `nav2_rviz_plugins/GoalTool` appears in RViz toolbar and `Navigation 2` panel is visible in the RViz sidebar (visual inspection after opening RViz with `bash scripts/rviz.sh`)
- [ ] T020 [US2] Send a test Nav2 goal via CLI to confirm the full action pipeline (SC-004): `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}\""` — robot must start moving within 3 s
- [ ] T021 [US2] Place a Nav2 Goal via RViz GoalTool (click-drag on the map) and confirm: global plan appears (blue line), local plan appears (cyan line), robot drives to goal, Nav2 Panel reports Succeeded (SC-004)

**Checkpoint (US2 done)**: Autonomous navigation functional from RViz. US2 independently deliverable on top of US1.

---

## Phase 5: User Story 3 — Map Save & Reload for AMCL Navigation (Priority: P3)

**Goal**: Save the SLAM map, restart in nav mode, robot localises with AMCL.

**Independent Test**: Save map, restart with `NAV_MODE=nav MAP=/maps/room.yaml`,
drive robot — AMCL pose updates without errors.

- [ ] T022 [US3] Ensure `./maps/` directory exists with write permissions on the host before saving: `mkdir -p /home/master/Workspace/ecza-robotu/maps`
- [ ] T023 [US3] Save the SLAM map: `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_saver_cli -f /maps/my_map"` — verify `/maps/my_map.pgm` and `/maps/my_map.yaml` exist on host: `ls maps/my_map.*`
- [ ] T024 [P] [US3] Confirm `src/ecza_navigation/config/nav2_params.yaml` amcl section: `grep "base_frame_id\|robot_model_type\|transform_tolerance" src/ecza_navigation/config/nav2_params.yaml` — must show `base_link`, `OmnidirectionalMotionModel`, `1.0`
- [ ] T025 [US3] Restart navigation in nav mode with saved map: `NAV_MODE=nav MAP=/maps/my_map.yaml docker compose -f docker-compose.yaml --profile nav up -d --force-recreate navigation` — note: pass env vars on the command line, not by editing `docker-compose.yaml`
- [ ] T026 [US3] Verify AMCL is publishing `map→odom` TF within 10 s and `/amcl_pose` has a publisher: `docker exec ecza-robotu-navigation-1 bash -c "source /opt/ros/humble/setup.bash && ros2 topic info /amcl_pose"`
- [ ] T027 [US3] Drive robot 0.5 m with joystick and confirm AMCL pose updates (particle cloud shifts in RViz) with no `Failed to compute odom pose` warnings in `docker logs ecza-robotu-navigation-1 --since 30s`

**Checkpoint (US3 done)**: Map persistence and AMCL localisation working. Full cycle: map → save → reload → navigate.

---

## Final Phase: Polish & Cross-Cutting Concerns

- [ ] T028 [P] Update `docker-compose.yaml` comment block for the `navigation` service to document the correct restart procedure: `docker compose -f docker-compose.yaml --profile nav up -d --force-recreate navigation lidar` (not `docker restart`)
- [ ] T029 [P] Update `scripts/launch.sh` `--nav` stop/restart path to use `--force-recreate` so that relaunching the nav stack always picks up the latest image
- [ ] T030 Commit all config and launch file changes on branch `007-fix-nav2-slam` with message: `fix(nav2): fix bt_navigator plugins, SLAM TF timeout, RViz Nav2 goal tool`

---

## Dependency Graph

```
T001 ─┬─ T002
      ├─ T003  (Phase 1: can run in parallel after T001)
      └─ T004
              ↓
T005 ─┐
T006 ─┤ → T008 → T009  (Phase 2: verify fixes → rebuild → force-recreate)
T007 ─┘
              ↓
T010 ─┐
T011 ─┤
T012 ─┤ → T013 → T014 → T015 → T016 → T017  (Phase 3: US1 — parallel checks then sequential)
              ↓
T018 ─┐ → T019 → T020 → T021  (Phase 4: US2 — goal navigation)
              ↓
T022 → T023 ─┬─ T024
             └─ T025 → T026 → T027  (Phase 5: US3 — map save/reload)
              ↓
T028 ─┐
T029 ─┤ → T030  (Final: polish, commit)
```

## Parallel Execution Opportunities

**In Phase 1 (after T001)**: T002, T003, T004 can run simultaneously.

**In Phase 2 (after T001)**: T005, T006, T007 can run simultaneously (different files). T008 depends on all three.

**After T009 (containers recreated)**: T010, T011, T012 can run simultaneously (different topic/process checks).

**After US1 passes (T017)**:
- T018 and T019 can run in parallel (US2)
- T022 and T024 can run in parallel (US3 prep)

## Implementation Notes

- **Never use `docker restart`** — always use `docker compose --force-recreate` after a rebuild
- **SLAM mode** = `navigation_launch.py` only (no `bringup_launch.py`, no AMCL, no map_server)
- **Nav mode** = `bringup_launch.py` with `map:=` and AMCL active; pass `NAV_MODE`/`MAP` as shell env vars on the `docker compose` command line
- T005, T006, T007 are **verification** tasks — all fixes are already applied to source files but uncommitted
- The Docker image predates some fixes; T008 (rebuild) is mandatory before any runtime verification
