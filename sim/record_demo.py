#!/usr/bin/env python3
"""Record the existing stock EKF flight, using real Isaac Sim viewport frames.

Container: /isaac-sim/python.sh /workspace/sim/record_demo.py
Raw frames and full-rate CSVs go in ignored build/demo_capture/. The host-side
encoder produces the presentation MP4, poster and frame telemetry in sim/demos/.
No USD files, physics parameters, estimator or controller code are modified.
"""
import argparse
import asyncio
import csv
import ctypes
import json
from pathlib import Path
import time

SIM_DIR = Path(__file__).resolve().parent
FPS = 30
WIDTH, HEIGHT = 1920, 1080
DURATION_S = 30.0
CAMERA_EYE = (-0.55, -1.8, 1.15)
CAMERA_TARGET = (0.22, 0.0, 0.57)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=SIM_DIR.parent / 'build/demo_capture')
    parser.add_argument('--duration', type=float, default=DURATION_S)
    parser.add_argument('--noise', choices=('off', 'on'), default='off')
    args = parser.parse_args()
    if not 1 / FPS <= args.duration <= DURATION_S:
        parser.error('--duration must be between 1/30 and 30 seconds')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    frames = output / 'frames'
    frames.mkdir(exist_ok=True)
    # Refuse to mix multiple recordings into the same image sequence.
    if any(frames.iterdir()):
        parser.error(f'Frame directory is not empty: {frames}')

    # Report a broken CUDA environment before launching an unusable GUI.
    cuda = ctypes.CDLL('libcuda.so.1')
    result = cuda.cuInit(0)
    if result != 0:
        raise RuntimeError(f'CUDA cuInit failed with code {result}; restore host CUDA before recording')

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': False, 'width': WIDTH, 'height': HEIGHT,
                         'window_width': 1600, 'window_height': 1000})
    log = telemetry = None
    try:
        import run_sim as runtime
        from flight_demo import FlightLog, TimedHoverMoveDemo
        from estimators.firmware_ekf import state_dict
        from isaacsim.core.utils.stage import get_current_stage
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        from pxr import Gf, Usd, UsdGeom

        sim, controller = runtime.setup_simulation('ekf_closed_loop', noise=args.noise == 'on', seed=7)
        stage = get_current_stage()
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError('No GUI viewport available')
        viewport.resolution = (WIDTH, HEIGHT)
        viewport.fill_frame = False
        # This is a presentation camera only. Source arena.usd remains untouched.
        camera_path = '/World/PresentationCamera'
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.CreateFocalLengthAttr(24.0)
        camera.CreateHorizontalApertureAttr(36.0)
        camera.CreateVerticalApertureAttr(20.25)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
        viewport.camera_path = camera_path
        set_camera_view(eye=CAMERA_EYE, target=CAMERA_TARGET, camera_prim_path=camera_path)
        # Warm up shaders without advancing physics or firmware ticks.
        for _ in range(30):
            sim.render()
        runtime.reset_runtime(sim, controller)
        demo = TimedHoverMoveDemo(1.0)
        log = FlightLog(output / 'flight.csv')
        telemetry = (output / 'frames.csv').open('w', newline='')
        fields = ['frame', 'time_s', 'phase', 'x_m', 'y_m', 'z_m',
                  'est_x_m', 'est_y_m', 'est_z_m', 'target_x_m', 'target_y_m', 'target_z_m',
                  'screen_x_px', 'screen_y_px']
        writer = csv.DictWriter(telemetry, fieldnames=fields)
        writer.writeheader()
        steps, count = 0, round(args.duration * FPS)
        view_matrix = UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
        projection = camera.GetCamera(Usd.TimeCode.Default()).frustum.ComputeProjectionMatrix()
        world_to_clip = view_matrix * projection
        for frame in range(count):
            # Integer 1 ms physics steps, alternating 33/34 steps per video frame.
            wanted_steps = round(frame * 1000 / FPS)
            while steps < wanted_steps:
                if not app.is_running():
                    raise RuntimeError('Window closed before recording finished')
                state = runtime.read_state(steps * runtime.PHYSICS_DT)
                target = demo.target(state)
                command = controller.compute(state, target)
                runtime.apply_command(command)
                log.write(state, target, command, demo.phase)
                sim.step(render=False)
                steps += 1
            state = runtime.read_state(steps * runtime.PHYSICS_DT)
            target = demo.target(state)
            before = sim.current_time_step_index
            # Force-update PhysX/Fabric before capture, then drain the async GPU
            # capture. render() explicitly disables physics during app.update().
            sim.render()
            image_path = frames / f'{frame:05d}.png'
            capture = capture_viewport_to_file(viewport, str(image_path))
            task = asyncio.ensure_future(capture.wait_for_result())
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline and app.is_running():
                sim.render()
                if task.done() and image_path.exists():
                    break
            if not task.done() or not image_path.exists():
                raise RuntimeError(f'Capture timed out: {image_path}')
            task.result()
            if sim.current_time_step_index != before:
                raise RuntimeError('Capture unexpectedly advanced physics')
            position = state['position_m']
            clip = world_to_clip.Transform(Gf.Vec3d(*map(float, position)))
            estimate = state_dict(controller.ekf.external_state)
            row = dict(frame=frame, time_s=state['time_s'], phase=demo.phase,
                       screen_x_px=(clip[0] + 1) * WIDTH / 2,
                       screen_y_px=(1 - clip[1]) * HEIGHT / 2)
            for axis, value, est, goal in zip('xyz', position, estimate['position_m'], target['position_m']):
                row[f'{axis}_m'], row[f'est_{axis}_m'], row[f'target_{axis}_m'] = value, est, goal
            writer.writerow(row)
            if frame % FPS == 0:
                telemetry.flush()
                print(f'RECORD {frame}/{count}: t={state["time_s"]:.3f}s {demo.phase} pos={position.tolist()}', flush=True)
        # Include the endpoint sample, matching run_sim.py --demo (t=30.000).
        while steps <= round(args.duration * 1000):
            state = runtime.read_state(steps * runtime.PHYSICS_DT)
            target = demo.target(state)
            command = controller.compute(state, target)
            runtime.apply_command(command)
            log.write(state, target, command, demo.phase)
            sim.step(render=False)
            steps += 1
        final = runtime.read_state(steps * runtime.PHYSICS_DT)
        metadata = dict(mode='ekf_closed_loop', noise=args.noise, seed=7, fps=FPS,
                        duration_s=args.duration, frames=count, resolution=[WIDTH, HEIGHT],
                        physics_dt_s=runtime.PHYSICS_DT, physics_end_s=steps * runtime.PHYSICS_DT,
                        mass_kg=runtime.MASS_KG,
                        inertia_kg_m2=runtime.INERTIA_KGM2, camera_eye=CAMERA_EYE,
                        camera_target=CAMERA_TARGET, final_position_m=final['position_m'].tolist(),
                        ekf_resets=controller.ekf.reset_count,
                        limitations=['Mass and inertia are placeholders.', 'Propellers are visual-only.',
                                     'Flow/ToF use analytical models, not rendered images/raycasting.',
                                     'No custom NIS gate, LQR, avoidance or emergency braking.'],
                        timing='30 fps at 1x simulation time, each frame sampled to nearest 1 ms physics tick.')
        (output / 'capture.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print('RECORD_COMPLETE ' + json.dumps(metadata), flush=True)
        sim.stop()
    except Exception:
        import traceback
        traceback.print_exc()
        import omni.kit.app
        omni.kit.app.get_app().post_quit(1)
        raise
    finally:
        if log is not None:
            log.close()
        if telemetry is not None:
            telemetry.close()
        app.close()


if __name__ == '__main__':
    main()
