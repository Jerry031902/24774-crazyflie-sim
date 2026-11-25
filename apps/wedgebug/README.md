# WedgeBug on the Crazyflie

This app flies the WedgeBug navigator on a Crazyflie 2.1 with a Flow deck v2 and a Multi-ranger deck. The navigator is [src/wedgebug.c](src/wedgebug.c), the same C code the Isaac Sim missions fly through `cffirmware`. [src/wedgebug_app.c](src/wedgebug_app.c) does on the drone what [sim/mission_flight.py](../../sim/mission_flight.py) does in the sim:

1. take off and settle;
2. call `wedgebugStep()` at 50 Hz with the EKF state and the Multi-ranger readings;
3. send the output as a hover setpoint (body vx/vy, yaw rate, fixed height);
4. hold at the goal, then land.

The host side lives in [real/](../../real): `wedgebug_mission.py` uploads a mission, starts it, logs it and can abort it. `plot_wedgebug.py` plots a run.

**Status:** the app builds for the STM32 (`-Werror`) and passes a host test in a 2-D world ([test/](test)). It has not flown yet. Everything below about real flight is what to check, not what has been seen.

| File | What it is |
|---|---|
| [src/wedgebug.c](src/wedgebug.c), [src/wedgebug.h](src/wedgebug.h) | Navigator core, unchanged; also compiled into the sim's Python bindings |
| [src/wedgebug_app.c](src/wedgebug_app.c) | Flight wrapper: state machine, frames, Multi-ranger frames, guards, params/logs |
| [build.sh](build.sh), [Makefile](Makefile), [app-config](app-config) | Build and flash |
| [test/](test) | Host test: the app compiled against stubs and flown in a 2-D world |
| [real/wedgebug_mission.py](../../real/wedgebug_mission.py) | Host runner: upload, start, log, abort |
| [real/missions/](../../real/missions) | Mission files (same fields as `sim/navigation/missions.py`) |
| [real/plot_wedgebug.py](../../real/plot_wedgebug.py) | Plot a recorded run |

Changes outside this folder:
- [src/modules/src/range.c](../../src/modules/src/range.c) gains `rangeGetUpdateCount()`, a per-direction count of `rangeSet()` calls. The app uses it to tell a new 10 Hz Multi-ranger frame from a held one.
- [range.h](../../src/modules/interface/range.h) now documents that `rangeGet()` returns millimetres. It always did, even though the old comment said metres.

## Build and flash

One-time setup: `git submodule update --init --recursive`, the `arm-none-eabi-gcc` toolchain, and cflib/cfloader as in [real/README.md](../../real/README.md).

```bash
apps/wedgebug/build.sh                                   # -> apps/wedgebug/build/cf2.bin
apps/wedgebug/build.sh flash                             # build, then flash radio://0/80/2M/E7E7E7E701
apps/wedgebug/build.sh flash radio://0/80/2M/E7E7E7E7E7  # another address
```

Kbuild will not build in a directory whose path contains a space, such as `24-774 ACSI Project`. `build.sh` handles this by mirroring the repo, including `.git` (about 1 GB with the build output; later runs copy only what changed), to `~/.cache/wedgebug-build` and building there. Set `WEDGEBUG_BUILD_DIR` to use another place. In a clone without spaces, plain `cd apps/wedgebug && make -j && make cload` works.

`flash` warm-boots the running Crazyflie into its bootloader over the radio (`cfloader ... -w URI`). Close cfclient first.

If the drone is not running a firmware that answers the radio, cold-boot it instead:
1. With it off, hold the power button about 3 s until the blue LEDs blink.
2. Run `cfloader flash apps/wedgebug/build/cf2.bin stm32-fw`.

To check the flash worked, open cfclient → Parameters: the groups `wedgebug` and `wbnav` should be there. The console shows `WEDGEBUG: WedgeBug app ready`. Until a mission starts the app sends nothing, so the drone flies from cfclient as before.

## Test plan

Run each step from the repo root with the Python that has cflib (`real/requirements.txt`). Look at the result before moving to the next step.

0. **Host test:** `make -C apps/wedgebug/test`. All 13 scenarios should pass. They cover:
   - the box, the rotated/offset start frame, and the full `matlab_main` mission;
   - the baseline navigator and an unreachable goal;
   - every guard, a missing deck, and a bad mission.
1. **Bench dry run, motors never start:** `python real/wedgebug_mission.py real/missions/straight.json --dry-run --dry-seconds 30`. While it prints a status line every second:
   - hold a hand in front of each sensor in turn. **F/L/B/R must drop for the matching side**; left is the drone's left, +y.
   - Away from walls, the ranges should read `inf`.
2. **Straight flight, no obstacle:** `python real/wedgebug_mission.py real/missions/straight.json`. Expected sequence:
   - takeoff at 0.2 m/s to 0.5 m, settle;
   - fly 1.5 m nose-first, hold 2 s, land.
   - Over open floor, F should read `inf`, not about 2 m (see *Floor in the cone* below).
3. **One box:** set up a box about 0.5 × 0.5 m and at least 1 m tall, with its near face 1.5 m ahead. Then run `python real/wedgebug_mission.py real/missions/one_box.json`. The drone should:
   - stop about 1 m short and sweep ±45°;
   - go round the shorter side and reach (3, 0).
4. **Then:** waypoints (`--target` repeated, or a mission file), walls long enough to force wall-following, `--baseline` against the default timestamped frames, and the `--nav` settings.

For the first runs, slower speeds give more time to react: `--nav vCruise=0.3 --nav vFollow=0.2`. The sim defaults are 0.5 and 0.3.

Each run writes `real/data/<UTC time>_wedgebug_<name>/` containing:
- `meta.json`: the mission, every parameter, firmware revision and events;
- `wb_state.csv`, `wb_cmd.csv`, `estimate.csv`, `status.csv`;
- `console.txt`: the drone's console.

Plot a run with `python real/plot_wedgebug.py real/data/<run>`. It needs matplotlib.

## Mission frame and placement

Targets, start and fence use the mission (room) frame, the same frame as `sim/navigation/missions.py`. The app resets the EKF just before takeoff, so the EKF frame starts at the drone with x along its nose. **Place the drone at `start` with its nose pointing `psi0`** (counter-clockwise from the room's +x). The app maps between the two frames. With the example missions (start (0, 0), psi0 0), the mission frame is simply "x straight ahead from where it took off".

The position comes from the Flow deck, so it drifts: expect centimetres to decimetres of error by the end of a mission. A target counts as reached within 0.10 m of the **estimate**.

## What it does

| State (`wedgebug.state`) | |
|---|---|
| 0 IDLE | Motors off, no setpoints sent. Waits for `wedgebug.start = 1` |
| 1 PREFLIGHT | Resets the EKF and waits 2 s. Checks the decks, Kalman, tilt < 15°, z < 0.2 m, EKF at its origin, and that Multi-ranger frames are arriving. Then arms |
| 2 TAKEOFF | Climbs at 0.2 m/s to `height`, holding the start x/y and `psi0` |
| 3 SETTLE | Holds until within 0.10 m and 0.10 m/s for 1 s (10 s limit) |
| 4 NAVIGATE | `wedgebugStep()` at 50 Hz, output sent as a hover setpoint |
| 5 ARRIVE | Every target handled: brakes until below 0.05 m/s |
| 6 HOLD | Position hold for `holdTime` |
| 7 LAND | Descends at 0.2 m/s where it is, then stops the motors and gives control back to the radio |

While the app flies, its setpoints use priority `COMMANDER_PRIORITY_EXTRX`, so **cfclient flight commands and cflib `send_stop_setpoint()` are ignored until it lands**. Ways to stop it:

| How | Effect |
|---|---|
| Ctrl+C in `wedgebug_mission.py` | `wedgebug.start = 0`: lands where it is |
| Ctrl+C a second time | Emergency stop (`cf.supervisor.send_emergency_stop()`): motors off at once, the drone drops. Reboot needed |
| A hand held over the top (up sensor < `upStop` for 0.3 s) | Lands |
| cfclient: set `wedgebug.start` to 0 | Lands |

The guards below also land the drone. `wedgebug.reason` says why a mission ended:

| Reason | | Reason | |
|---|---|---|---|
| 1 | done | 9 | navigation took longer than `maxTime` |
| 2 | `start` set to 0 | 10 | left the fence box, or z > `zMax` |
| 3 | bad mission parameters (nothing flew) | 11 | no radio link for `linkLoss` s |
| 4 | Flow deck, Multi-ranger or Kalman missing | 12 | no Multi-ranger frame for 0.5 s while navigating |
| 5 | preflight check failed | 13 | hand over the up sensor |
| 6 | no Multi-ranger frames before takeoff | 14 | supervisor tumbled/crashed/locked/emergency stop (motors already cut) |
| 7 | supervisor did not allow flight | 15 | non-finite estimate (motors cut) or navigator output (lands) |
| 8 | did not settle over the start | 16 | battery low |

Losing the radio does **not** stop a mission immediately: the app lands after `linkLoss` seconds (default 1 s; 0 disables).

## Parameters and logs

`wedgebug.*` parameters take effect at the next start, except the fence and guards, which apply at once.

| `wedgebug.` | Default | |
|---|---|---|
| `start` | 0 | 1: fly; 0: land. The app sets it back to 0 when a mission ends |
| `nTargets`, `x0`/`y0` … `x7`/`y7` | 1, (1, 0) | Targets in order (waypoints, then the goal), mission frame [m] |
| `startX`, `startY`, `psi0` | 0, 0, 0 | Takeoff point [m] and heading [rad] in the mission frame |
| `height` | 0.5 | Flight height, 0.2 … 1.0 m |
| `maxTime`, `holdTime` | 120, 2 | Navigation time limit; hover at the goal [s] |
| `fenceXMin` … `fenceYMax`, `zMax` | −1, 3, −2, 2, 1.5 | Fence box, mission frame [m]. The start and every target must be inside |
| `linkLoss` | 1.0 | Land after this long without radio [s]; 0 = keep flying |
| `upStop` | 0.2 | An up reading under this lands [m]; 0 = off |
| `latency` | 0.1 | Multi-ranger measurement-to-delivery delay used to pair each frame with its yaw [s] |
| `tsRanges` | 1 | 1: timestamped frames, as the sim missions use; 0: baseline navigator |

`wbnav.*` holds every `wedgebugParams_t` field. Names and defaults are those of `sim/navigation/params.py` (`p.nav`), and they are copied into the navigator at each start. The runner sets them with `--nav NAME=VALUE`.

The log group `wedgebug` holds:
- `state`, `reason`, `mode`, `phase`, `wp`;
- `x`, `y`, `yaw` (mission frame);
- `cmdVx`, `cmdVy`, `cmdYawRate`;
- the navigator's ranges `rF`/`rL`/`rB`/`rR` [m] (inf = no return);
- `seq` (frames), `sweeps`, `hits`, `reached`, `unreachable`, `count` (missions started).

## Differences from the sim to keep in mind

- **Invalid vs. no return.** The stock Multi-ranger driver writes 32767 mm for any reading that is not `RANGE_VALID`, so on the drone "invalid" and "nothing in range" both reach the navigator as `inf`. The sim can tell them apart (`NaN` for invalid).
- **Frame timing.** A frame counts as new once all four horizontal sensors have been rewritten. It is paired with the yaw from `latency` (0.1 s) earlier, the sim's ranger model. That delay is not measured on this deck. If sweeps look rotated, adjust `latency`.
- **Floor in the cone.** Each sensor sees a 27° cone. At height h, its lower edge meets the floor about h / tan(13.5°) ≈ 4.2·h ahead: 2.1 m at 0.5 m height, more when pitched forward. The sim flies at 1.0 m, where this is just past the 4 m range. If F reads about 2 m over open floor, fly higher. Obstacles need to cover the cone at flight height, so make them clearly taller than the flight height.
- **No Safety layer.** The Primary/Backup range supervisor in [sim/safety](../../sim/safety) is not on the drone. The navigator's own stopping rules (`kStop`, `dWall`, `dClose`) and the coarse guards above are all there is.
- **Takeoff and settle.** The app ramps from the floor and settles within 0.10 m / 0.10 m/s. The sim commands the height directly and uses 0.05 / 0.05.

## Host test

`make -C apps/wedgebug/test` compiles three files unchanged against stub headers ([test/stubs](test/stubs)):
- `wedgebug_app.c`
- `wedgebug.c`
- the firmware's `range.c`

It then flies each scenario in its own process. The world is a kinematic drone following the app's setpoints through first-order lags, an EKF frame that restarts at the drone on reset, and a Multi-ranger model: 9-ray 27° cones, 2 mm noise, 32767 sentinel, measured every 100 ms and written 100 ms later in the driver's order.

Run `apps/wedgebug/test/build/test_app -v obstacle` to see the app's console for one scenario.

This tests the app's logic: states, frames, frame detection and guards. It does not model the PID, Flow-deck drift, attitude, or how the real sensor behaves.
