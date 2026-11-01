# Crazyflie baseline demo

这段视频用于 Project Update presentation，内容是现有 stock baseline 的一次 Isaac Sim 5.1 运行。没有修改 PPT。

| File | 用途 |
|---|---|
| `crazyflie_ekf_baseline_noise_off.mp4` | 1080p presentation 版，白底 / CMU red，带 phase、simulation time、true position 和 drone close-up |
| `crazyflie_ekf_baseline_noise_off_clean.mp4` | 同次运行的完整 viewport，无文字、无 UI，适合自己在 PPT 中排版 |
| `crazyflie_ekf_baseline_noise_off.png` | 15 s hover 时的 poster，可以设置为 PowerPoint 的 video poster frame |
| `crazyflie_ekf_baseline_noise_off.frames.csv` | 每个 video frame 对应的 true / estimated position、target 和 simulation timestamp |
| `crazyflie_ekf_baseline_noise_off.json` | Capture settings、model assumptions、final position 和 encoding information |
| `flight_analysis.json` | 同次运行的 full-rate flight log 分析结果 |
| `video_validation.json` | 两个 MP4 的完整 decode、900 frames、codec、duration 和 faststart 检查 |

两个 MP4 都是 **30 s、1920 × 1080、30 fps、H.264 / yuv420p、无 audio**。每秒视频对应一秒 simulation time。由于 capture 会等待 renderer，录制本身的 wall time 会更长。右侧 close-up 是同一帧的 crop，不是第二次 flight。

## 内容与范围

- Initial position: `(0, 0, 0.1)` m，沿用 `run_sim.py`。
- 0–10 s：takeoff / settle，target `(0, 0, 1)` m。
- 10–20 s：hover，target 保持不变。
- 20–30 s：move / settle，target `(0.5, 0, 1)` m。
- Mode: `ekf_closed_loop`，sensor noise `off`，seed `7`。
- Pipeline: simulated IMU / Flow / ToF → stock `kalmanCore` → stock `controllerPid` → `powerDistribution` → motor forces / yaw torque → PhysX。
- PID 使用 EKF state；ground truth 用于生成 simulated measurements 和记录 telemetry。

**这是 simulation baseline。** `MASS_KG = 0.030` 和 inertia 仍是 placeholder；propellers 是 visual-only，force / torque 作用在 single rigid body 上。Flow / ToF 使用现有 analytical measurement model；本次没有测试真实 camera texture、NIS gate、LQR、obstacle avoidance 或 emergency braking。画面中的 pillars 是现有 arena 的一部分，不代表已经具备 avoidance。

## 本次 flight 验证

来源为同次 1 kHz flight CSV，置信度高：

| 检查 | 结果 |
|---|---|
| `sim/plot_ekf.py` 现有 stability checks | 全部通过 |
| 最后记录的 true position | `(0.500899, -0.000001, 0.999729)` m |
| Hover 阶段最大 Z tracking error | `0.018772` m |
| EKF resets / motor saturated steps | `0 / 0` |
| 与之前 headless `noise off` baseline 比较 | 30,001 行 X / Y / Z position 全部一致 |

这些结果仅描述本次 noise-off simulation。完整 metrics 和 checks 保存在 `flight_analysis.json`。

## PowerPoint

使用 PowerPoint 的 **Insert → Video → This Device** 选择一个 MP4。建议先插 presentation 版；如果 slide 已经有自己的 title / annotations，可以用 `_clean.mp4`。原来的 presentation 和 Project Update assignment 文件都没有修改。

## 复现

在 repo root 执行。`cf_sim_dev` 必须运行，GPU / display 可用，`/workspace` 指向本 repo，Python 3.11 的 `cffirmware` 已在 `build/` 中。

```bash
# Capture，使用新的空目录。会打开 Isaac Sim GUI，并在录完后关闭。
docker exec cf_sim_dev /isaac-sim/python.sh /workspace/sim/record_demo.py \
  --output /workspace/build/demo_capture_new

# Encode，在 host 上执行。需要 Pillow、Liberation Sans font 和 PATH 中的 ffmpeg。
python3 sim/encode_demo.py \
  --input build/demo_capture_new \
  --output sim/demos/new_recording
```

`record_demo.py` 复用 `run_sim.py` 的 setup / state / actuator 和 `flight_demo.py` 的 schedule / CSV logger。physics / firmware tick 为 1 ms；每 33 或 34 个 physics steps 保存一帧，选择最接近 30 fps timestamp 的 tick。等待 capture 时只调用 `sim.render()`，并检查 physics step index 没有变化。

本次 full-rate logs 在 `build/demo_capture/flight.csv` 和 `flight.measurements.csv`，`build/` 已被 `.gitignore` 忽略。临时 PNG sequence 在两个 MP4 完整 decode 验证后已清理，以免占用约 2 GB。需要重新 encode 时，先按上面的命令重新 capture。Source `arena.usd` 和 firmware files 不会被保存或更改。

**来源 / 置信度：** pipeline、schedule 和参数来自 repo source；输出规格、flight metrics 来自本次实际 capture / CSV / decode 检查，置信度高。真实硬件性能尚未由这段视频验证。
