# Quickstart: Docker Runtime for Ecza Robotu

**Feature**: `006-docker-runtime`
**Last updated**: 2026-02-24

This guide goes from a bare host machine to a driving rover in the minimum
number of steps. Read every section before running any command.

---

## Prerequisites

| Item | Minimum version | Check |
|------|----------------|-------|
| OS | Ubuntu 22.04 LTS | `lsb_release -a` |
| Docker Engine | 24.0 | `docker --version` |
| Docker Compose plugin | v2.20 | `docker compose version` |
| Available RAM | 4 GB | — |
| Free disk space | 6 GB (image + deps) | `df -h` |

> **Why no older Docker?** Compose v2 syntax (`docker compose` not `docker-compose`)
> is required. Compose v1 is unsupported.

---

## Step 1 — Host setup (once per machine)

### 1a. Firewall — allow micro-ROS agent port

```bash
sudo ufw allow 8888/udp
sudo ufw status   # verify rule appears
```

### 1b. Joystick udev rule

```bash
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-joystick.rules
KERNEL=="js[0-9]*", SUBSYSTEM=="input", GROUP="input", MODE="0660"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 1c. Add your user to the `input` group

```bash
sudo usermod -aG input "$USER"
```

> **Log out and back in** (or `newgrp input` in the current shell) for the
> group change to take effect. The joy service runs privileged, so this step
> only matters if you ever test the joystick outside Docker.

---

## Step 2 — F710 joystick hardware

1. On the back of the F710 find the **MODE** switch.
2. Set it to **D** (DirectInput).

> If the switch is on **X**, ROS2's `joy_linux` node will not publish on
> `/joy`. The LED pattern changes: D-mode shows solid green on the mode button.

---

## Step 3 — ESP32 firmware provisioning

The firmware must be flashed once with your network credentials and the host
machine's IP address.

Edit the following constants in the ESP32 firmware source before flashing:

```c
#define WIFI_SSID     "your-ssid"
#define WIFI_PASSWORD "your-password"
#define AGENT_IP      "192.168.x.x"   // IP of the machine running Docker
#define AGENT_PORT    8888
```

Flash via USB (USB is for programming only; operational data uses WiFi):

```bash
idf.py -p /dev/ttyUSB0 flash
```

After flashing, disconnect USB. The rover communicates only over WiFi at
runtime.

---

## Step 4 — Configuration

```bash
cp config/rover_params.yaml.example config/rover_params.yaml
```

Open `config/rover_params.yaml` and set `wheel_radius` to the measured value
(in metres). Every other parameter has a working default.

> **Wheel radius must be measured.** Using the wrong value makes all velocity
> commands incorrect. Measure the loaded radius (with the rover on the floor
> under its own weight).

> **No rebuild needed for config changes.** `config/rover_params.yaml` is
> bind-mounted read-only into every container at `/config/rover_params.yaml`.
> To apply a changed parameter value, run `docker compose restart` — there is
> **no need** to run `docker compose build` again. `docker compose build` is
> only required when ROS2 source code changes.

---

## Step 5 — Build the Docker image

```bash
docker compose build
```

This is required only once per machine, and again after any code change.
The build takes 5–15 minutes on first run (downloads ~500 MB base image and
ROS2 dependencies).

---

## Step 6 — Launch

Use the provided wrapper script (validates Docker/Compose versions automatically):

```bash
bash scripts/launch.sh
```

To also record all topics to `./bags/`:

```bash
RECORD=true bash scripts/launch.sh
```

The script translates `RECORD=true` to `COMPOSE_PROFILES=record` for you.

To stop:

```bash
docker compose down
```

> **Tip**: `bash scripts/launch.sh -d` runs in detached mode. View logs with
> `docker compose logs -f`.

> **Direct Compose**: You can also call `docker compose up` directly, but
> `scripts/launch.sh` is preferred as it verifies minimum Docker/Compose
> versions before launch.

---

## Step 7 — Verify (Tier 1)

Confirm all containers are running:

```bash
docker compose ps
```

Expected output: all services in `running` state. The `recorder` service
appears only when `RECORD=true`.

---

## Step 8 — Verify (Tier 2)

Wait up to 30 seconds after launch, then run:

```bash
bash docker/healthcheck.sh --tier2
```

Expected output (exit code 0):

```
topics:
  /joy                  active
  /cmd_vel              active
  /wheel_velocities     active
  /diagnostics          active
esp32_node:             connected
```

If `esp32_node: waiting` appears, the ESP32 has not connected yet. Check:
- Both devices are on the same WiFi network
- `AGENT_IP` in the firmware matches the host machine's current IP
- UDP 8888 is allowed: `sudo ufw status`

---

## Step 9 — Drive

1. Hold the **right bumper** (enable button, dead-man switch).
2. Move the **left stick** to translate (forward/back/strafe).
3. Move the **right stick left/right** to rotate.

Release the right bumper to stop all wheel outputs immediately.

---

## Common issues

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `/joy` topic absent | F710 not in D-mode | Slide MODE switch to **D** |
| ESP32 not connecting | Wrong `AGENT_IP` or firewall | Re-flash firmware; check `ufw` |
| `permission denied /dev/input` | Group membership not active | Log out and back in |
| `docker compose build` fails on rosdep | Network timeout | Retry; check VPN/proxy |
| Rover moves wrong direction | `wheel_radius` incorrect | Measure and update YAML |
| All containers exit immediately | Missing `rover_params.yaml` | Run step 4 |
