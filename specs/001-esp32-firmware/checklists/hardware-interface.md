# Hardware Interface Checklist: ESP32-S3 Motor Firmware

**Purpose**: Lightweight author self-review of requirements quality, focused on
hardware interface correctness (BTS7960B RPWM/LPWM signals, GB37-520 motor
mapping, speed-to-PWM conversion). Edge case requirements are included in scope.
**Created**: 2026-02-24
**Feature**: `specs/001-esp32-firmware/spec.md`
**Depth**: Lightweight (author pre-PR sanity pass)
**Primary focus**: Hardware interface correctness (Q2-B)
**Secondary focus**: Safety/watchdog, edge case coverage

---

## Requirement Completeness

- [ ] CHK001 — Is the exact PWM frequency (or acceptable range) specified for the BTS7960B RPWM/LPWM signals? FR-002 defines the logic polarity but does not document a required PWM frequency; the BTS7960B datasheet specifies a maximum of 25 kHz — is this constraint captured? [Completeness, Gap, Spec §FR-002]

- [ ] CHK002 — Is the enable-pin behaviour fully specified for all states? The Assumptions section states enable pins are "held HIGH during normal operation" but does not define the enable-pin state during watchdog stop, brown-out, or fault. Should enable pins go LOW on any of these events? [Completeness, Gap, Spec §Assumptions]

- [ ] CHK003 — Are the four GPIO pin assignments (RPWM, LPWM, EN per channel × 4 motors) specified or deferred? FR-002 and FR-003 describe signal logic but no pin mapping is documented. Is this intentionally deferred to the plan phase, and is that deferral explicitly noted? [Completeness, Assumption, Spec §FR-002]

- [ ] CHK004 — Is the speed-to-PWM duty-cycle calibration constant defined, or is its derivation process specified? FR-009 mentions a "configurable maximum speed parameter" and the edge case mentions "rounding MUST be documented" — but neither the formula nor the calibration procedure appear in the spec. [Completeness, Gap, Spec §FR-009]

- [ ] CHK005 — Are the WheelCommand message type and topic name specified? FR-011 says the firmware subscribes to "the wheel velocity topic" but does not name the topic or define the ROS2 message type (`std_msgs/Float32MultiArray`, custom type, or `sensor_msgs`?). SC-006 requires this to be fully documented — is the spec itself complete on this point? [Completeness, Ambiguity, Spec §FR-011, §SC-006]

- [ ] CHK006 — Are the FirmwareStatus topic name and ROS2 message type specified? FR-007 defines the payload fields but not the topic name, message type package, or QoS profile. SC-006 requires the interface to be fully documented. [Completeness, Gap, Spec §FR-007, §SC-006]

- [ ] CHK007 — Is the QoS profile (reliability, durability, history depth) defined for both the subscriber and publisher topics? micro-ROS imposes QoS constraints relative to the agent; a mismatch silently drops messages. [Completeness, Gap, Spec §FR-011]

---

## Requirement Clarity

- [ ] CHK008 — Is "within 100 ms of message receipt" (FR-002, SC-001) measured from WiFi packet arrival, micro-ROS deserialization completion, or PWM signal change? The measurement point is ambiguous and affects how the acceptance test is instrumented. [Clarity, Ambiguity, Spec §FR-002, §SC-001]

- [ ] CHK009 — Is "clamp to the maximum" (FR-004) defined as a per-wheel clamp or a global clamp applied before the per-wheel split? If the kinematics node sends an already-scaled command, a second clamp in firmware could introduce unexpected behaviour. [Clarity, Ambiguity, Spec §FR-004]

- [ ] CHK010 — Is "configurable timeout period" (FR-005) quantified with an explicit default and acceptable range? The Assumptions section states the default is 500 ms, but the FR itself does not reference this default — is the link between FR-005 and the Assumptions section explicit enough for an implementer? [Clarity, Spec §FR-005, §Assumptions]

- [ ] CHK011 — Is the speed unit (rad/s) explicitly stated in FR-001 or FR-002, or only in the Assumptions section? An implementer reading only the FRs would not know the expected unit; the unit should appear in the FR or the WheelCommand entity definition. [Clarity, Spec §FR-001, §Key Entities]

- [ ] CHK012 — Is "malformed or incomplete" (FR-008) defined? What exactly qualifies a message as malformed — wrong length, failed checksum, out-of-range values, or all of the above? The WheelCommand entity describes "an optional checksum" — optional in what sense? [Clarity, Ambiguity, Spec §FR-008, §Key Entities]

- [ ] CHK013 — Is "fault condition (over-current, over-temp)" (US3 AC-3) defined in terms of how the firmware detects it? Does the BTS7960B expose a fault signal pin that the ESP32 must poll or interrupt on? This detection mechanism is not mentioned in the FRs or Assumptions. [Clarity, Gap, Spec §US3-AC3]

---

## Hardware Interface Correctness

- [ ] CHK014 — Is the RPWM/LPWM polarity convention (FR-002: LPWM=forward, RPWM=reverse) consistent with the actual BTS7960B wiring convention? The spec defines the convention but does not note that this is a wiring decision — swapping the wires reverses it. Is the convention tied to a specific wiring diagram reference? [Consistency, Spec §FR-002]

- [ ] CHK015 — Is the shoot-through guard (edge case: simultaneous RPWM+LPWM non-zero) defined as a firmware assertion, a hardware interlock, or both? The edge case says the firmware MUST treat it as a fault, but does not specify whether the fault causes immediate stop, error flag only, or both. [Clarity, Spec §Edge Cases]

- [ ] CHK016 — Is the rounding behaviour for rad/s → PWM duty cycle conversion formally specified (floor, ceiling, or round-half-up)? The edge case section says "rounding MUST be documented" but the spec itself does not yet document the rounding rule. [Completeness, Gap, Spec §Edge Cases]

- [ ] CHK017 — Are the GB37-520 encoder signals (if used for speed feedback) addressed in the spec? The Assumptions note that encoder resolution is "to be confirmed during plan/calibration phase" — but the FRs do not mention closed-loop control at all. Is open-loop operation intentional and explicitly stated? [Completeness, Assumption, Spec §Assumptions]

- [ ] CHK018 — Is the partial-motor-failure policy (edge case: one driver fails) consistent with the watchdog policy? Both result in "stop all motors" but the edge case says "set an error flag" while FR-005/FR-006 define watchdog recovery. Does the partial-motor-failure state also allow recovery via a new command, or does it require a reset? [Consistency, Spec §Edge Cases, §FR-005, §FR-006]

---

## Safety & Watchdog Requirements

- [ ] CHK019 — Is the safe state on brown-out (FR-010) defined beyond "stop all motors"? Specifically: does the firmware hold RPWM=0/LPWM=0 actively, or does it rely on the BTS7960B's built-in behaviour when PWM signals are absent? If the ESP32 resets, GPIO pins default to input mode — does that produce a safe output on the BTS7960B? [Completeness, Gap, Spec §FR-010]

- [ ] CHK020 — Is the hardware watchdog (mentioned in the brown-out edge case) distinct from the software watchdog (FR-005) and are both documented with their respective timeout values? The spec mentions both but does not specify the hardware watchdog timeout or whether it is the ESP32's built-in TWDT/WDT. [Completeness, Gap, Spec §FR-005, §Edge Cases]

- [ ] CHK021 — Is the micro-ROS session loss detection mechanism (FR-013) specified beyond "detect loss"? micro-ROS does not provide a single session-loss callback — is the detection method (heartbeat timeout, executor spin failure, or rmw error code) documented? [Clarity, Gap, Spec §FR-013]

- [ ] CHK022 — Is there a requirement for what the firmware does between session loss detection and the watchdog firing? For example: does it attempt reconnection, or does it wait in stopped state indefinitely until power-cycled? [Coverage, Gap, Spec §FR-013]

---

## Scenario & Edge Case Coverage

- [ ] CHK023 — Are requirements defined for the startup sequence before WiFi is connected? US2-AC3 says "motors remain stopped" before the first command, but there is no requirement covering the firmware state while actively attempting WiFi association (could take seconds). [Coverage, Gap, Spec §US2-AC3]

- [ ] CHK024 — Is there a requirement for the maximum number of reconnection attempts or a reconnection backoff policy? If the agent IP is wrong or the network is down, should the firmware retry indefinitely or report a provisioning error? [Coverage, Gap, Spec §FR-012, §FR-013]

- [ ] CHK025 — Are requirements defined for concurrent faults — e.g., watchdog timeout firing at the same time as a motor driver fault? Which state takes precedence, and what does the status report show? [Coverage, Edge Case, Spec §FR-005, §FR-007]

- [ ] CHK026 — Is "independent test" for US3 (status reporting) accurately described? The test says "Read the serial/USB output" but the Assumptions explicitly state USB is for flash/debug only and all operational data is via WiFi. The independent test should reference WiFi/micro-ROS readback, not USB. [Consistency, Spec §US3-IndependentTest, §Assumptions]

---

## Acceptance Criteria Quality

- [ ] CHK027 — Can SC-003 ("8 distinct motion patterns, no wheel spinning in the wrong direction") be objectively measured without a per-wheel encoder or tachometer? The spec should specify the measurement method, or note that encoder readback from the GB37-520 is required for this criterion. [Measurability, Spec §SC-003]

- [ ] CHK028 — Is SC-004 ("30 minutes continuous operation without crashing") defined with a specific command rate and load profile? "Repeated velocity commands" is vague — at what frequency and with what variety of commands? [Measurability, Ambiguity, Spec §SC-004]

- [ ] CHK029 — Is SC-006 ("micro-ROS topic interface fully documented") a success criterion for the spec or for the plan/implementation? If it refers to documentation that does not yet exist, is there a tracking item to produce it? [Completeness, Spec §SC-006]

---

## Dependencies & Assumptions

- [ ] CHK030 — Is the assumption "wheel radius is not needed by the firmware" traceable to the kinematics spec (002-mecanum-kinematics)? If the kinematics node changes its output unit from rad/s to m/s, this assumption breaks — is the cross-spec dependency captured? [Dependency, Assumption, Spec §Assumptions]

- [ ] CHK031 — Is the assumption "WiFi latency under 20 ms" validated or merely asserted? The spec states "higher latency will cause watchdog stops; this is acceptable" — but is there a requirement on the minimum watchdog timeout value that ensures normal latency does not trigger false watchdog stops? [Measurability, Assumption, Spec §Assumptions]
