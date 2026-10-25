# Crazyflie Simulation (24-774 Team 1)

Isaac Sim 5.1 simulation for the Crazyflie 2.1, running the **real firmware code** (stock PID controller, motor mixing, and Kalman filter) through the `cffirmware` Python bindings.

This folder (`sim/`) lives inside our fork of `bitcraze/crazyflie-firmware`. The repo root README is Bitcraze's firmware README. This file covers only the simulation.

> **Status:** `sim-baseline-v1` (commit `89197588`). Both EKF modes passed the baseline bounded-flight checks with noise off and on, using seed 7. Noise-on closed-loop flight has substantial horizontal drift; this is not a claim of accurate real-world flight. Physical parameters and noise levels are **placeholders** until calibrated against real flight data. See [baseline results](BASELINE_EKF.md#6-observed-results) and [limitations](#placeholders-and-limitations).

---

## How it works

The logical pipeline in `ekf_closed_loop` is:

```
PhysX true state
   -> simulated sensors (IMU, Flow, ToF)        sim/sensors.py
   -> stock EKF (kalmanCore, C code)            sim/estimators/firmware_ekf.py
   -> stock controllerPid + powerDistribution   sim/controllers/firmware_pid.py
   -> actuator model (4 motor forces + yaw)     sim/actuators.py
   -> PhysX (single rigid body)
```

- **PhysX** computes the drone motion. The drone is one rigid body. Propellers are visual only.
- **Firmware C code** (controller, mixer, EKF core) is compiled from this repo into `cffirmware`. The simulator calls these routines, not the complete RTOS firmware or hardware drivers. The physics/main loop is 1 kHz; individual routines and sensors run at their firmware rates.
- **Sensors** are generated from ground truth using the same measurement models the firmware EKF uses, plus optional noise and delay.

The PID uses the previous completed EKF output. The EKF worker then processes the current measurements, matching the stock copy-before-wakeup ordering. [BASELINE_EKF.md](BASELINE_EKF.md#1-stock-order-and-rates) documents the exact order and source lines.

Shared C algorithms can be tested in sim and reused on hardware after adding the required bindings and hardware integration. The baseline has no custom NIS gate. Passing in sim does not validate hardware timing, sensor fidelity, or the complete flight safety logic.

---

## Repo layout

```
24774-crazyflie-sim/
├── src/ bindings/ Makefile ...   firmware source and Python bindings
├── build/                        build outputs, git-ignored (cffirmware, plots, CSV)
└── sim/
    ├── README.md                 this file
    ├── Dockerfile                Isaac Sim 5.1 + C build tools
    ├── build_scene.py            generates scenes/arena.usd
    ├── run_sim.py                main entry point
    ├── sensors.py                IMU / Flow / ToF models
    ├── actuators.py              motor command -> force/torque
    ├── assets/crazyflie/          bundled local drone USD and schema layer
    ├── scenes/arena.usd          walls + obstacles (no drone)
    ├── BASELINE_EKF.md           source audit, assumptions, measured results
    ├── plot_flight.py / plot_ekf.py
    ├── tests/                    unit tests and headless PhysX smoke tests
    ├── controllers/              base.py (interface), firmware_pid.py, altitude_pid.py, ...
    └── estimators/               firmware_ekf.py
```

---

## Hardware this sim matches

| Item | Value |
|---|---|
| Control board | Crazyflie 2.1 |
| Propellers | **47-17** (the Crazyflie 2.1+ propellers). Matches default `cf2_defconfig`. |
| Flow deck | **v2** (PMW3901 + VL53L1x ToF) |
| Ranger deck | On the real drone. **Not simulated yet.** |

**This baseline assumes 47-17 propellers.** `cf2_defconfig` selects CF2; the defaults in [Kconfig](../Kconfig) select `CONFIG_CRAZYFLIE_21_PLUS` and battery thrust compensation. The [actuator model](actuators.py) checks those settings and uses the corresponding parameters from [platform_defaults_cf2.h](../src/platform/interface/platform_defaults_cf2.h). Different propellers require matching firmware settings and actuator parameters; the current model is not calibrated for them.

---

## Setup (one time)

### 0. Requirements

- Native x86_64 Ubuntu 22.04 or 24.04. This guide targets Linux Docker Engine, not Docker Desktop or WSL2; this project's WSL2 workflow has not been validated.
- NVIDIA GPU with RT cores and a compatible driver. NVIDIA's published Isaac Sim 5.1 minimum lists RTX 4080, 16 GB VRAM, 32 GB RAM, and 50 GB SSD storage. Allow extra disk space for Docker layers, caches, and logs. GPUs without RT cores, including A100/H100, are unsupported. See the [official requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html).
- For the GUI commands below, a local graphical session with X11 or XWayland, a valid `DISPLAY`, and the `xhost` utility.

### 1. NVIDIA driver, Docker Engine, NVIDIA Container Toolkit

```bash
nvidia-smi                                  # driver check, must print a GPU table

# On a new development machine without Docker Engine:
curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
sudo sh /tmp/get-docker.sh
sudo usermod -aG docker "$USER"              # then log out and log back in
```

These use Docker's development convenience installer; the alternative is [Docker's Ubuntu apt-repository installation](https://docs.docker.com/engine/install/ubuntu/). Install the NVIDIA Container Toolkit using its [official install guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), then configure it on the host:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --runtime=nvidia --gpus all ubuntu:22.04 nvidia-smi
```

The last command must print a GPU table. Runtime configuration and restarting Docker are one-time setup steps, not steps to repeat while simulations are running.

### 2. GitHub SSH key

If you do not already have a GitHub SSH key:

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"   # press Enter for defaults
cat ~/.ssh/id_ed25519.pub                            # copy the whole line
```

Reuse an existing key instead of overwriting it.

GitHub: avatar -> Settings -> SSH and GPG keys -> New SSH key -> paste. Then:

```bash
ssh -T git@github.com    # expect "Hi <username>! You've successfully authenticated..."
```

GitHub deliberately returns exit code 1 after successful authentication because it provides no interactive shell; check the message. See [GitHub's SSH test instructions](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/testing-your-ssh-connection).

Ask Jerry to add you as a collaborator on the repo so you can push.

### 3. Clone (with submodules)

```bash
git clone --recursive git@github.com:Jerry031902/24774-crazyflie-sim.git ~/24774-crazyflie-sim
```

`--recursive` is required. Without it, the bindings build fails (e.g. `arm_math.h: No such file or directory`). If you already cloned without it:

```bash
cd ~/24774-crazyflie-sim
git submodule update --init --recursive
```

### 4. File permissions for the container

The image runs as user `isaac-sim` (uid **1234**). Give that uid access to the repo, and add default ACL entries for both the container and your actual host uid. Run on the host:

```bash
setfacl -R    -m "u:1234:rwX,u:$(id -u):rwX" ~/24774-crazyflie-sim
setfacl -R -d -m u:1234:rwX ~/24774-crazyflie-sim
setfacl -R -d -m "u:$(id -u):rwX" ~/24774-crazyflie-sim
getfacl -n ~/24774-crazyflie-sim  # numeric output: user:1234:rwx and default:user:1234:rwx
```

If `setfacl` is missing, install it on the host with `sudo apt-get install acl`. Default ACLs cover normally created files; applications that explicitly create private files can still restrict the effective ACL mask.

### 5. Build the Docker image

```bash
cd ~/24774-crazyflie-sim
docker build -t cf-sim:dev sim/
```

This uses [sim/Dockerfile](Dockerfile) and pulls `nvcr.io/nvidia/isaac-sim:5.1.0` on the first build. [sim/.dockerignore](.dockerignore) excludes everything except the Dockerfile from the context. Source, scenes, and build outputs are bind-mounted, not copied into the image.

### 6. Create the container

On the host, create a persistent background container, then open a shell:

```bash
xhost '+si:localuser:#1234'
docker run --name cf_sim_dev -d --runtime=nvidia --gpus all \
    --entrypoint bash \
    -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$HOME/24774-crazyflie-sim:/workspace" \
    --network=host \
    cf-sim:dev -lc 'sleep infinity'
docker exec -it cf_sim_dev bash
```

The `xhost` entry grants local X access to uid 1234, matching this image's user with the standard Docker Engine configuration above. `cf-sim:dev` is the **image**; `cf_sim_dev` is the **container name**. `PRIVACY_CONSENT=Y` opts into NVIDIA data collection and can be omitted to opt out, as documented in the [Isaac Sim container guide](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html).

Later sessions, reuse the same container from the host:

```bash
xhost '+si:localuser:#1234'
docker start cf_sim_dev                # if stopped
docker exec -it cf_sim_dev bash        # enter, or open another shell
```

Inside the container, `~` is `/isaac-sim`, and the repo is at `/workspace`. Prompt `isaac-sim@...` = inside, `<you>@...` = host. Do git operations on the **host** (your SSH key is not in the container).

### 7. Build the firmware Python bindings (inside the container)

```bash
cd /workspace
make cf2_defconfig      PYTHON=/isaac-sim/python.sh
make bindings_python    PYTHON=/isaac-sim/python.sh
make test_python        PYTHON=/isaac-sim/python.sh     # expect all tests passed
```

Bindings **must** be built with Isaac Sim's Python 3.11 (`/isaac-sim/python.sh`). Do not use `pixi` (Python 3.13) or the root `cffirmware_setup.py` (hard-coded Python 3.8).

The Dockerfile already sets `PYTHON=/isaac-sim/python.sh` and `PYTHONPATH=/workspace/build`; no manual export is needed in this image. Check the import:

```bash
/isaac-sim/python.sh -c 'import cffirmware; print(cffirmware.__file__)'
```

The expected path is `/workspace/build/cffirmware.py`. `Makefile` invokes `bindings/setup.py`; the root `cffirmware_setup.py` is a legacy packaging helper copied into `build/`, not the extension build entry point.

**Rebuild the bindings every time you change firmware C code, then restart the Python/simulation process.** For header or build-configuration changes, force extension recompilation after regenerating the bindings:

```bash
make bindings_python PYTHON=/isaac-sim/python.sh
/isaac-sim/python.sh bindings/setup.py build_ext --inplace --force
make test_python PYTHON=/isaac-sim/python.sh
```

This image contains native C-extension build tools. Building/flashing the STM32 firmware additionally requires the firmware toolchain described in the repo's [firmware build guide](../docs/building-and-flashing/build.md).

### 8. Generate the scene

`scenes/arena.usd` and `assets/crazyflie/` are already tracked. Regenerate the arena only after changing its constants; this command overwrites `arena.usd` and does not add the drone:

```bash
/isaac-sim/python.sh /workspace/sim/build_scene.py
```

---

## Running

All commands below run **inside the container**, unless labeled host. GUI is the default. `--target-z` defaults to 1.0 m.

| Command | What it does |
|---|---|
| `/isaac-sim/python.sh /workspace/sim/run_sim.py` | Default: simple Python altitude PID, target z = 1 m |
| `/isaac-sim/python.sh /workspace/sim/run_sim.py --mode free_fall` | No thrust: drone drops and rests |
| `/isaac-sim/python.sh /workspace/sim/run_sim.py --mode hover_ff` | Constant total thrust = m·g; no position feedback |
| `/isaac-sim/python.sh /workspace/sim/run_sim.py --mode altitude_pid --target-z 1.0` | Explicit Python altitude-only PID |
| `/isaac-sim/python.sh /workspace/sim/run_sim.py --mode firmware_pid --demo` | Firmware PID on ground truth: takeoff, 10 s settled hover, move to x = 0.5 m |
| `/isaac-sim/python.sh /workspace/sim/run_sim.py --mode ekf_shadow --demo --noise off` | Ground-truth PID; stock EKF runs alongside and does not control flight |
| `/isaac-sim/python.sh /workspace/sim/run_sim.py --mode ekf_closed_loop --demo --noise off` | Simulated sensors -> stock EKF -> firmware PID |

For either EKF mode, use `--noise on` to add simulated sensor noise; `--noise off` is the default. `--seed 7` selects the default repeatable noise realization. Noise-off retains the firmware filters, process noise, and measurement covariance. `--noise on` is rejected in non-EKF modes.

The EKF demos use a fixed schedule: 0-10 s takeoff/settle, 10-20 s hover, 20-30 s move/settle to (0.5, 0, 1). The ground-truth firmware demo instead switches phases after meeting position/speed tolerances. GUI demos continue holding the final target; `--headless --demo` exits when the demo completes. `--steps N` bounds any mode by physics steps; 1000 steps represent 1 simulation second, not necessarily 1 wall-clock second.

### Logs and plots

`run_sim.py` writes CSV only when `--demo` or `--csv PATH` is supplied; both options require a firmware controller mode. Default paths, relative to the repo root:

| Mode | `--demo` output |
|---|---|
| `firmware_pid` | `build/firmware_pid/demo.csv` |
| `ekf_shadow` | `build/ekf_baseline/ekf_shadow_noise_off.csv` (or `_on.csv`) |
| `ekf_closed_loop` | `build/ekf_baseline/ekf_closed_loop_noise_off.csv` (or `_on.csv`) |

EKF runs also write a sibling `.measurements.csv`. Existing files at the chosen paths are overwritten on a new run or Stop/Play reset. Choose `--csv` for separate experiments. `run_sim.py` does **not** automatically create plots or JSON reports.

After the corresponding demo has finished and its CSV has closed:

```bash
/isaac-sim/python.sh /workspace/sim/plot_flight.py /workspace/build/firmware_pid/demo.csv
/isaac-sim/python.sh /workspace/sim/plot_ekf.py /workspace/build/ekf_baseline/ekf_closed_loop_noise_off.csv
```

Each plot script writes a sibling `.png`; `plot_ekf.py` also prints metrics to stdout and requires the sibling measurement CSV.

For all four matched EKF runs (shadow/closed-loop, noise off/on), with CSVs, plots, per-run JSON, and `summary.json` generated together:

```bash
/isaac-sim/python.sh /workspace/sim/tests/smoke_firmware_ekf.py
```

This headless runner writes to `build/ekf_baseline/` by default. Its `--output-dir` selects a different directory; `--mode ekf_closed_loop --noise on --seed 7` selects one combination. The smoke runner's default `--noise both` differs from `run_sim.py`'s `--noise off`.

The existing motor/sign-test and ground-truth flight runner writes to `build/firmware_pid/` (`flight.csv`, `flight.png`, `sign_tests.json`, `summary.json`):

```bash
/isaac-sim/python.sh /workspace/sim/tests/smoke_firmware_pid.py
```

From the host, without entering the container:

```bash
docker exec -it cf_sim_dev /isaac-sim/python.sh /workspace/sim/run_sim.py --mode firmware_pid --demo
```

---

## Adding your own code

### A controller

Implement the interface in [controllers/base.py](controllers/base.py):

```python
from controllers.base import Controller

class MyController(Controller):
    def reset(self): ...
    def compute(self, state: dict, target: dict) -> dict: ...
```

Register the controller in `run_sim.py`'s `make_controller()` and argparse mode choices. Return `total_thrust_N` for a single body-Z force, or four `motor_commands` in M1-M4 order. A motor controller also needs inclusion in `MOTOR_MODES` so the actuator is initialized. Existing firmware logging expects additional telemetry keys; adapt it if your controller returns a different schema. Controllers are not auto-discovered.

Recommended workflow for anything that will fly on the real drone:

1. Prototype in Python, test in sim.
2. Write the C version. For a controller, see the [out-of-tree controller guide](../docs/development/oot.md#oot-controllers) and [example](../examples/app_out_of_tree_controller/README.md): implement `controllerOutOfTreeInit`, `controllerOutOfTreeTest`, and `controllerOutOfTree`, and enable `CONFIG_CONTROLLER_OOT=y` in your project configuration.
3. Add the required headers/declarations to `bindings/cffirmware.i` and sources/dependencies to `bindings/setup.py`, rebuild bindings, then add a Python wrapper that calls the new C entry point. The current wrapper calls `controllerPid()` directly; selecting an OOT firmware config alone does not switch the simulator's controller.
4. Run the C version in sim, confirm it matches the Python version.
5. Build the hardware firmware with the STM32 toolchain and validate it on hardware before relying on sim results.

### Firmware changes

- **Add, do not delete.** We compare stock vs custom (stock EKF vs gated EKF, PID vs LQR), so stock code must stay available behind a parameter switch.
- Do not edit `cf2_defconfig`. Put project config changes in a separate config file.

---

## Conventions

| Item | Convention |
|---|---|
| Units | Public ground-truth state uses m, s, rad; actuator outputs use N and N·m. Firmware structs and diagnostic logs also use deg, deg/s, Gs, and Flow counts; names/comments specify conversions. |
| World frame | Z-up, matches firmware |
| Body frame | x forward, y left, z up |
| Legacy pitch | Firmware Euler `pitch` is **inverted** relative to right-handed pitch. Quaternion is not. The ground-truth path converts in `firmware_pid.py`; closed-loop uses the stock EKF's already-converted state. |
| Motor numbering / spin direction | From firmware `power_distribution_quadrotor.c`. Verified by sign tests. |
| Rates | 1 kHz physics, IMU, and main loop; render every 10 steps. Position 100 Hz, attitude/rate 500 Hz, EKF predict 100 Hz. Flow 100 Hz, ToF v2 40 Hz (nominal). These are simulation-time rates. |

---

## Placeholders and limitations

| Item | Current value / status | Replace with |
|---|---|---|
| `MASS_KG` | 0.030 (placeholder) | Measured total mass with battery + Flow deck + Ranger deck |
| `INERTIA_KGM2` | `(1.6572e-5, 1.6656e-5, 2.9262e-5)` kg·m² in `run_sim.py`; placeholders, not measured for this vehicle | Estimate including battery and decks |
| Motor dynamics | None (force changes instantly) | Kelvin's motor time constant |
| Sensor noise (IMU, Flow, ToF) | Assumed values in `sensors.py` | Values from real hover logs |
| Sensor bias, delay | Bias not modeled; configurable additional delays all default to zero. IMU filtering still adds lag. | Values from real logs |
| Flow noise vs EKF assumption | Noise-on Gaussian standard deviation and stock measurement standard deviation are both 2.0 firmware counts. Noise-off keeps the latter. | Measured noise and separately specified fault cases. Matching one standard deviation does not establish EKF optimality or eliminate Gaussian tail events; no custom gate is present. |
| Battery voltage compensation | Firmware config enables it, but the simulation does not run the voltage-to-PWM stage. The adapter converts the pre-compensation force command directly to N. | Battery/motor model if needed |
| Flow / ToF geometry | Stock measurement models on a flat floor; no camera images or scene raycasts, no texture failure/dropout model | Scene-dependent sensing if needed |
| EKF observability | Flow/ToF provide no absolute XY or yaw reference; noise-on position drift is expected | External reference only if available on the real system |
| Ranger deck (horizontal ToF) | Not simulated | Raycast model (to do) |
| Aerodynamics (drag, ground effect) | No explicit model | Identify from real logs if needed |

These are idealized baseline results, not a validated prediction of real-flight accuracy. The complete arming/watchdog/emergency state machines are not simulated. See [BASELINE_EKF.md](BASELINE_EKF.md) for measurement assumptions and the actual stability criteria.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `groups: cannot find name for group ID ...` | Harmless warning when entering the container. Ignore. |
| `Permission denied` on a file in `/workspace` | Check `ls -l` and `getfacl` on the host. A missing ACL, parent-directory access, or restrictive ACL mask can cause this. Reapply the setup ACL entries to the affected path. If the entry exists but its mask blocks it, `chmod 664 <file>` or `chmod 775 <dir>` can restore the mask; replace the placeholders with actual paths. |
| `Failed to open stage ...usd` | Check that the repo mount, local USD path, and referenced asset files exist and are readable. Check permissions before regenerating the scene. |
| Isaac Sim window does not appear | Run `xhost '+si:localuser:#1234'` on the host. Check the container's `DISPLAY`, the `/tmp/.X11-unix` mount, and that the host has a graphical session. |
| First launch looks frozen | Shader compilation. Wait a few minutes. |
| `ModuleNotFoundError: cffirmware` | Build the bindings using Python 3.11. `cf-sim:dev` already sets `PYTHONPATH=/workspace/build`; if using another environment, set it explicitly. |
| Sim behaves like old firmware code | You changed C code but did not rebuild bindings. |
| `docker: Error ... name "/cf_sim_dev" is already in use` | Reuse it with `docker start cf_sim_dev` if stopped, then `docker exec -it cf_sim_dev bash`. |

---

## Git rules

- Never commit `build/` outputs or large files (GitHub rejects files > 100 MB).
- Run `git status` before every commit.
- Work on your own branch, open a PR to merge.
- Baseline reference: tag `sim-baseline-v1` at commit `89197588` (the implementation before this README was added).
