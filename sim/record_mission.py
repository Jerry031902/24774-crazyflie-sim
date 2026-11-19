#!/usr/bin/env python3
"""Capture actual Isaac viewport frames and encode the same mission to MP4.

Run with Isaac Sim Python; PNGs, CSVs and the report remain beside flight.mp4.
Rendering/capture never advances physics. Video time follows simulation time.
"""
import argparse
import asyncio
import csv
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

SIM = Path(__file__).resolve().parent


def png_complete(path):
    """Kit may finish its capture future before the PNG writer has flushed."""
    try:
        with path.open('rb') as image:
            if image.read(8) != b'\x89PNG\r\n\x1a\n':
                return False
            image.seek(-12, 2)
            return image.read() == b'\x00\x00\x00\x00IEND\xaeB`\x82'
    except (OSError, ValueError):
        return False


def wait_for_capture(task, path, render, is_running, timeout=30.):
    """Drain render updates until both the capture and image write finish."""
    deadline = time.monotonic() + timeout
    completed = False
    while is_running() and time.monotonic() < deadline:
        if task.done() and not completed:
            task.result()  # Preserve actual capture exceptions.
            completed = True
        if completed and png_complete(path):
            return
        render()  # runtime.render disables physics; keep driving Kit/file callbacks.
    if not task.done():
        task.cancel()
    raise RuntimeError(f'Viewport capture/image write did not complete: {path}')


def encode_video(ffmpeg, output, fps, count):
    subprocess.run([
        str(ffmpeg), '-y', '-loglevel', 'error', '-framerate', str(fps),
        '-start_number', '0', '-i', str(output / 'frames' / '%05d.png'),
        '-frames:v', str(count), '-an', '-c:v', 'libx264', '-preset', 'fast',
        '-crf', '20', '-pix_fmt', 'yuv420p', '-threads', '2',
        '-movflags', '+faststart', str(output / 'flight.mp4'),
    ], check=True)


def json_safe(value):
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main():
    from navigation.missions import MISSIONS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission', choices=sorted(MISSIONS), default='single_obstacle')
    parser.add_argument('--safety', choices=('on', 'off'), default='on')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fps', type=int, default=20)
    parser.add_argument('--max-seconds', type=float, default=120.)
    parser.add_argument('--hold-seconds', type=float, default=3.)
    parser.add_argument('--popup-at', type=float, default=None,
                        help='Single-obstacle experiment: enable the box at this navigation-relative second')
    parser.add_argument('--ffmpeg', type=Path,
                        default=Path(shutil.which('ffmpeg') or SIM.parent / 'build/video_tools/ffmpeg'))
    args = parser.parse_args()
    if not 1 <= args.fps <= 60:
        parser.error('--fps must be between 1 and 60')
    if not math.isfinite(args.max_seconds) or args.max_seconds <= 0:
        parser.error('--max-seconds must be finite and positive')
    if not math.isfinite(args.hold_seconds) or args.hold_seconds < 0:
        parser.error('--hold-seconds must be finite and nonnegative')
    if args.popup_at is not None and (args.mission != 'single_obstacle'
            or not math.isfinite(args.popup_at) or args.popup_at < 0):
        parser.error('--popup-at requires single_obstacle and a finite nonnegative time')
    if not args.ffmpeg.is_file():
        parser.error(f'FFmpeg is missing: {args.ffmpeg}')
    # Fail before launching Isaac if the encoder cannot run on this machine.
    subprocess.run([str(args.ffmpeg), '-version'], check=True, stdout=subprocess.DEVNULL)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        parser.error('Output directory must be empty; use a new recording folder')
    frames = output / 'frames'
    frames.mkdir()
    width, height = 1280, 720
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': False, 'width': width, 'height': height,
                         'window_width': 1400, 'window_height': 900})
    flight = ranger_log = telemetry = mission_run = None
    popup = None
    count, tick, done_at = 0, 0, None
    metadata = dict(status='incomplete', mission=args.mission, safety=args.safety,
                    fps=args.fps, resolution=[width, height],
                    source='actual Isaac Sim viewport captures',
                    mode='firmware_pid', navigator='firmware', ranger_noise=False,
                    seed=7, physics_dt_s=.001)
    try:
        import run_sim as runtime
        from build_mission_scene import build_mission_scene
        from flight_demo import FlightLog
        from mission_flight import WedgeBugMission
        from navigation.params import default_params
        from ranger import RangerLog
        from safety import SafetyConfig
        from isaacsim.core.utils.stage import get_current_stage
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        mission = MISSIONS[args.mission]
        scene = SIM / 'scenes' / mission.scene
        if not scene.is_file():
            build_mission_scene(mission, scene)
        (x0, x1), (y0, y1) = mission.xlim, mission.ylim
        cx, cy = (x0+x1)/2, (y0+y1)/2
        span = max(x1-x0, y1-y0)
        eye, target = (cx, cy-.75*span, 1.65*span), (cx, cy, 0.)
        sim, controller = runtime.setup_simulation(
            'firmware_pid', target_z_m=1., seed=7, ranger=True, ranger_noise=False,
            scene_path=scene, initial_position=(*mission.start, runtime.INITIAL_POSITION[2]),
            camera=(eye, target))
        stage = get_current_stage()
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError('No active viewport; recording requires the GUI')
        viewport.resolution = (width, height)
        viewport.fill_frame = False
        camera_path = '/World/RecordingCamera'
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.CreateFocalLengthAttr(28.)
        camera.CreateHorizontalApertureAttr(36.)
        camera.CreateVerticalApertureAttr(20.25)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 100.))
        viewport.camera_path = camera_path
        set_camera_view(eye=eye, target=target, camera_prim_path=camera_path)
        # Display-only floor rings: no physics or sensor colliders are added.
        for name, xy, color in (('Start', mission.start, (.2,.45,.9)),
                                ('Goal', mission.goal, (.1,.8,.35))):
            ring = UsdGeom.BasisCurves.Define(stage, '/World/Recording'+name)
            ring.CreateTypeAttr('linear')
            ring.CreateCurveVertexCountsAttr([49])
            ring.CreatePointsAttr([Gf.Vec3f(xy[0]+.18*math.cos(a*math.tau/48),
                                          xy[1]+.18*math.sin(a*math.tau/48), .015)
                                  for a in range(49)])
            ring.CreateWidthsAttr([.015]*49)
            ring.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        for _ in range(30):
            sim.render()
        if args.popup_at is not None:
            from popup_experiment import PopupExperiment
            prim = stage.GetPrimAtPath('/World/Obstacles/'+mission.obstacles[0]['name'])
            if not prim.IsValid() or not prim.HasAPI(UsdPhysics.CollisionAPI):
                raise RuntimeError('Popup obstacle collider was not found')
            def set_popup_enabled(enabled):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(bool(enabled))
                UsdGeom.Imageable(prim).GetVisibilityAttr().Set('inherited' if enabled else 'invisible')
            popup = PopupExperiment(mission, set_popup_enabled, args.popup_at)
            # Disable before physics reset so the first Ranger sample sees the
            # empty room. Both visual and collision state change at appearance.
            set_popup_enabled(False)
        runtime.reset_runtime(sim, controller)
        config = SafetyConfig() if args.safety == 'on' else None
        mission_run = WedgeBugMission(mission, default_params(), 1.,
                                     backend='firmware', safety_config=config)
        if popup is not None:
            popup.prepare(mission_run)
        flight = FlightLog(output / 'flight.csv')
        ranger_log = RangerLog(output / 'flight.ranger.csv', runtime._ranger)
        telemetry = (output / 'frames.csv').open('w', newline='')
        writer = csv.DictWriter(telemetry, fieldnames=[
            'frame','time_s','phase','safety_state','safety_reason',
            'x_m','y_m','z_m','screen_x_px','screen_y_px'])
        writer.writeheader()
        transform = UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
        projection = camera.GetCamera(Usd.TimeCode.Default()).frustum.ComputeProjectionMatrix()
        world_to_clip = transform * projection
        max_tick = round(args.max_seconds / runtime.PHYSICS_DT)
        while True:
            wanted = min(round(count / args.fps / runtime.PHYSICS_DT), max_tick)
            while tick < wanted:
                if not app.is_running():
                    raise RuntimeError('Recording window was closed before completion')
                state = runtime.read_state(tick * runtime.PHYSICS_DT)
                if popup is not None:
                    popup.update(state, mission_run)
                readings = runtime.sample_ranger(state)
                ranger_log.write(readings)
                if popup is not None:
                    popup.observe(readings)
                command_target = mission_run.target(state)
                command = controller.compute(state, command_target)
                runtime.apply_command(command)
                flight.write(state, command_target, command, mission_run.phase, mission_run.log_fields())
                if mission_run.done and done_at is None:
                    done_at = state['time_s']
                sim.step(render=False)
                tick += 1
            state = runtime.read_state(tick * runtime.PHYSICS_DT)
            before = sim.current_time_step_index
            sim.render()
            path = frames / f'{count:05d}.png'
            capture = capture_viewport_to_file(viewport, str(path))
            task = asyncio.ensure_future(capture.wait_for_result())
            wait_for_capture(task, path, sim.render, app.is_running)
            if sim.current_time_step_index != before:
                raise RuntimeError('Viewport capture unexpectedly advanced physics')
            position = state['position_m']
            clip = world_to_clip.Transform(Gf.Vec3d(*map(float, position)))
            s = mission_run.safety
            writer.writerow(dict(frame=count, time_s=state['time_s'], phase=mission_run.phase,
                safety_state=s.state if s else 'OFF', safety_reason=s.reason if s else '',
                x_m=position[0], y_m=position[1], z_m=position[2],
                screen_x_px=(clip[0]+1)*width/2, screen_y_px=(1-clip[1])*height/2))
            if count % args.fps == 0:
                telemetry.flush()
                print(f'RECORD t={state["time_s"]:.1f}s {mission_run.phase} '
                      f'Safety:{s.state if s else "OFF"} frames={count+1}', flush=True)
            count += 1
            if tick >= max_tick or (done_at is not None and state['time_s'] >= done_at+args.hold_seconds):
                break
        flight.close(); flight = None
        ranger_log.close(); ranger_log = None
        telemetry.close(); telemetry = None
        metadata.update(status='captured', stop_reason='goal_hold' if done_at is not None else 'time_limit',
                        summary=mission_run.summary(), camera_eye=eye, camera_target=target)
        print('Encoding actual viewport frames to MP4...', flush=True)
        encode_video(args.ffmpeg, output, args.fps, count)
        metadata['status'] = 'encoded'
        print(f'VIDEO_SAVED {output / "flight.mp4"}', flush=True)
        print(f'Mission summary: {mission_run.summary()}', flush=True)
    except BaseException as error:
        import traceback
        traceback.print_exc()  # Print before Kit shutdown can terminate Python.
        metadata['status'] = 'error'
        metadata['error'] = f'{type(error).__name__}: {error}'
        # Kit can terminate Python during close(); retain a failing exit code.
        import omni.kit.app
        omni.kit.app.get_app().post_quit(1)
        raise
    finally:
        try:
            for resource in (flight, ranger_log, telemetry):
                if resource is not None:
                    resource.close()
            if mission_run is not None:
                mission_run.write_safety_report(output / 'flight.safety.json')
                metadata['summary'] = mission_run.summary()
            metadata.update(frames=count, video_duration_s=count/args.fps,
                            physics_end_s=tick*.001)
            if popup is not None:
                metadata['popup'] = popup.report()
            (output / 'capture.json').write_text(json.dumps(json_safe(metadata), indent=2, allow_nan=False)+'\n')
        finally:
            app.close()


if __name__ == '__main__':
    main()
