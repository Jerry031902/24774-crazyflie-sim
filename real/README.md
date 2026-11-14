# Real Crazyflie setup 与 baseline 数据采集

这里保存真机 logger、setup 和 calibration 计划。`sim/` 负责 simulation；本目录使用 host Python 和 Crazyradio，不需要启动 Isaac Sim。

**验证状态：** 已检查本机 cflib 0.1.34、当前 repo firmware、已有 TOC cache，并做 offline fault-injection tests。置信度高的是 API、variable/type、units 和 Python cleanup 行为。**修正版尚未连接真机验证，radio delivery、实际 landing 和飞行稳定性未知。** 测试不会初始化 CRTP driver 或访问 radio。

## 1. Hardware 与 firmware 记录

| Item | 当前约定 / 记录方式 |
|---|---|
| Control board | Crazyflie 2.1，来源：用户确认 |
| Propellers | 47-17，来源：用户确认；sim 使用 `cf2_defconfig` 的 2.1+ propeller 参数 |
| Decks | Flow deck v2，底部 PMW3901 + VL53L1X；Multi-ranger deck 顶部。称重时全部安装 |
| Radio URI | 默认 `radio://0/80/2M/E7E7E7E701`，来源：原有 `hover_test.py` 和用户提供的 setup；用实际 scan 结果核对，可通过 `--uri` 修改 |
| Firmware | 每次自动保存参数表中的 `firmware.revision0/revision1/modified` 原值；另外在 session JSON 填 release/commit、build config。repo HEAD 不等于机载 firmware commit |
| Python packages | [requirements.txt](requirements.txt) 固定了本次检查的 cflib/cfclient version；运行时再记录实际 cflib version |

机载 firmware 的 propeller selection、PID gains 和 EKF settings 需要与记录一致。logger 会选择 PID / EKF，但不会把所有 gains 恢复为 stock defaults，也不能由 TOC 自动推断所有 compile-time options。

## 2. Host setup

在 native Ubuntu host 上运行。现有 `cf_sim_dev` 没有配置 Crazyradio USB passthrough；这不是说 Docker 本身不能使用 USB。

```bash
cd ~/24774-crazyflie-sim
sudo apt-get install python3-venv libxcb-cursor0 libxcb-xinerama0
python3 -m venv real/.venv
source real/.venv/bin/activate
python -m pip install -r real/requirements.txt
```

USB permissions 按 [Bitcraze 官方说明](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/)设置。下面只需在 host 配置一次；已有同样规则时不用重复创建：

```bash
sudo groupadd -f plugdev
sudo usermod -aG plugdev "$USER"
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-bitcraze.rules
# Crazyradio
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Crazyradio bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie USB
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新 login，然后重新插入 Crazyradio。在 activated venv 中运行 `cfclient`，按实际 radio address scan/connect。开始 logger 前关闭 cfclient，避免两个程序争用 radio。

来源：[cflib installation](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/install/)、[cfclient installation](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/)。

## 3. 每次实验先填写什么

在 repo root 执行一次，然后编辑 `real/session.json`。复制示例时不要覆盖已经填好的 session：

```bash
cp -n real/session.example.json real/session.json
```

未知值保持 JSON `null`，不要填 sim 的 placeholder 充当 measurement。更换 battery、propeller、deck 或 firmware 后更新 session。

| 信息 | 如何记录 | 用途 |
|---|---|---|
| `drone_id`、`battery_id` | 给本机和每块 battery 一个固定 label | 区分 hardware 和 battery 差异 |
| `hardware` | board、propeller、decks 的实际配置 | 保证比较的是同一套 hardware |
| `mass_g`、`mass_readings_g`、`scale_resolution_g` | 装完整配置称重；建议重复 3 次，记录全部读数和秤的实际 resolution | `MASS_KG = mass_g / 1000` |
| `height_uncertainty_m` | 卷尺测量的不确定度 | 区分 ToF bias 与人工测量误差 |
| `--known-height-m` | 每个 static-height run 单独填写，量到 ToF sensor 的测距参考位置 | ToF bias/noise，不能填 commanded hover height |
| `surface`、`lighting` | 地面材质/纹理、灯光状态；有 lux measurement 才填数字 | Flow noise 与有效性依赖场景 |
| `motor_propeller_condition` | M1-M4 的检查结果、更换过哪一件、是否存在异常 vibration | 避免把损坏 motor/prop 的数据当作正常 calibration |
| `firmware_label`、`firmware_build_config` | cfclient/version 信息；已知的 git commit、47-17 selection 和配置文件 | 重现机载 algorithm/config；不能用 host repo commit 代填 |
| `external_reference` | 如有 mocap / calibrated camera，记录设备、坐标系、同步方式、文件名；没有就 null | 真正的 position/attitude error 需要独立 reference |
| `ambient_temperature_c`、`notes` | 测量了才填；记录异常、boot/稳定等待、任何手动干预 | 解释 drift 和 bias 变化 |

每次 run 自动保存：CLI arguments、UTC 时间、logger hash、host repo commit、Python/cflib version、CRTP protocol version、机载 firmware revision 字段、运行前/运行中的完整 parameter snapshot、log types/bytes、phase events、packet counts/max gaps、failure/interruption 状态。`meta.json` 会复制当次 session 内容，因此以后修改 session 不会改变旧实验。

## 4. 按优先级采集

这是实验安排建议，置信度中高；时长和重复次数是本项目计划，不是 sensor 厂商给出的 calibration 标准。

| 优先级 | 实验 | 时长 / 条件 | 要留下的数据 |
|---|---|---|---|
| P0-1 | 整机 mass | motors off；装 battery、Flow v2、Multi-ranger；建议 0.1 g resolution，重复 3 次 | mass 原始读数、平均值、battery ID、配置 |
| P0-2a | 静止 IMU | 平稳放置，boot/calibration 期间不动；记录 60 s，建议重复 3 次 | filtered gyro/acc、温度/放置条件、全部原始 CSV |
| P0-2b | 已知高度的 ToF / Flow | 0.3、0.5、1.0 m 各 30 s，建议各 3 次；有纹理、光照稳定的地面 | tape height、height uncertainty、ToF、Flow raw counts、status/quality、地面与光照 |
| P0-3a | 首次低高度 smoke flight | 修复已知 motor/prop 问题后，先 0.5 m hover 5 s | 确认 logger 出数据、Z ramp、hover、landing 和 stop；不是正式 calibration 数据 |
| P0-3b | 正式 hover baseline | 0.5 m、1.0 m 各 30-60 s；建议各 3 次，记录 battery 状态 | estimated position/velocity/attitude、IMU、Flow/ToF、4 PWM、voltage、setpoints、phase |
| P1 | sensor bias / dynamics / motor characterization | 根据 P0 结果设计独立实验 | 见第 7 节；P0 数据不足时保持 unknown |
| P1，Ranger | 已知 wall distance | motors off；front/back/left/right/up 分别对已知距离的板；例如 0.2、0.5、1.0、2.0 m，各 30 s | optical face 到板的实际距离、direction、材质、角度、全部 range readings（包括 invalid） |

架高时量到 Flow deck 的 ToF sensor，不是 drone 顶部。支架不能挡住 ToF/Flow 的视线，地面需要足够大的有效区域。`--height` 是飞行 command，**架高静止实验不要传这个选项**。

以下从 repo root、activated venv 执行。每条 command 单独运行，检查结果后再做下一次：

```bash
python real/log_flight.py --name static_table_r1 --duration 60 --metadata real/session.json
python real/log_flight.py --name static_h030_r1 --duration 30 --known-height-m 0.30 --metadata real/session.json
python real/log_flight.py --name static_h050_r1 --duration 30 --known-height-m 0.50 --metadata real/session.json
python real/log_flight.py --name static_h100_r1 --duration 30 --known-height-m 1.00 --metadata real/session.json
```

把示例 `--known-height-m` 改成该次实际测到的距离。以下 command **会起飞**，必须从地面开始，不能从上述支架起飞：

```bash
python real/log_flight.py --name hover_smoke_050 --height 0.5 --duration 5 --metadata real/session.json
python real/log_flight.py --name hover_050_r1 --height 0.5 --duration 45 --metadata real/session.json
python real/log_flight.py --name hover_100_r1 --height 1.0 --duration 45 --metadata real/session.json
```

每次使用 `_r1`、`_r2`、`_r3` 区分 repeat。URI 不同就追加 `--uri`；额外观察写在 `--note` 中。默认输出相对于 script 的 `real/data/`，不依赖 shell 当前目录，也可以用 `--out` 指定目录。

Multi-ranger 采集增加 `--ranger`，例如下面这条 **只 log，不起飞**：

```bash
python real/log_flight.py --name ranger_front_050_r1 --duration 30 --ranger \
  --metadata real/session.json --note "front sensor optical face to wall: measured 0.500 m; matte white wall; normal incidence"
```

先实际测距，再把 note 改成该次真实条件。不要用 `--known-height-m` 表示水平 wall distance。
`--ranger` 会验证 `deck.bcMultiranger=1`，新增 `ranger.csv`，记录 `range.front/back/left/right/up`。
这些 uint16 readings 单位是 mm，`32767` 表示 invalid，不是 32.767 m 的障碍物；启动时的 `0` 也不应直接当成有效 obstacle。
Logger 保留 raw readings，不会把 invalid 替换为零或删掉。分别计算 valid fraction、valid readings 的 bias/std。
该功能已通过 offline logger tests，尚未在真机上验证。

## 5. logger 的行为与限制

### Logging

启动时按实际 TOC 检查全部变量、native types 和 block size。所有 block 都收到第一包，并且 data 没有 stale/error 后，才进入采集或 flight。记录过程检查 asynchronous error、disconnect、nonfinite values 和 block 是否停止出数据。缺少变量时直接报告失败，不自动省略字段。

| CSV | 变量 | 当前 firmware payload | 默认 logging rate |
|---|---|---|---|
| `imu.csv` | `gyro.x/y/z`、`acc.x/y/z` | 24 bytes | 100 Hz |
| `pos_vel.csv` | `stateEstimate.x/y/z`、`vx/vy/vz` | 24 bytes | 100 Hz |
| `attitude.csv` | `stateEstimate.roll/pitch/yaw`、`pm.vbat` | 16 bytes | 100 Hz |
| `flow_motor.csv` | `motion.deltaX/deltaY`、`range.zrange`、`motor.m1..m4`、`motion.motion/squal/shutter/std/outlierCount` | 23 bytes | 100 Hz |
| `status.csv` | `kalman.varPX/varPY/varPZ`、`supervisor.info` | 14 bytes | 20 Hz |
| `ranger.csv`，仅 `--ranger` | `range.front/back/left/right/up` | 10 bytes | 10 Hz；period 为 `max(100, --period)` ms |
| `setpoints.csv`，仅 flight | host time、phase、commanded XYZ、yaw | host 文件，不占 log block | nominal 50 Hz |

前五个 CSV 都有 `timestamp_ms`、`timestamp_unwrapped_ms`、`host_elapsed_s`、`phase`。`--period` 控制前四个 block；status period 为 `max(50, --period)` ms。100 Hz 是 logging 请求速率，不是每个 sensor 的独立 sample rate，也不是保证到包率。默认共约 420 个 log packets/s；是否有足够 radio bandwidth 必须看实际 counts/gaps。可以先用 static run 检查，必要时用 `--period 20` 降低 logging load。

可选的 `ranger.csv` 使用相同 timestamp/phase 列，默认额外请求 10 packets/s。Multi-ranger driver 的 nominal task period 是 100 ms，实际 data-ready 等待可能使它更慢；log packet 时间仍不是 sensor acquisition 时间。

`meta.json` 中 counts 和 gaps 用于发现 logging 问题，**不是精确的 sensor dropout counter**。CSV 约每秒 flush，正常退出时 close；进程强杀、掉电或 storage failure 仍可能丢数据，不能承诺绝不丢失。

### Flight

`--height 0`，默认值，只读 logging，不 arm、不 reset、不改 PID/EKF 参数，也不发送 motor/position command；若观察到 motor PWM 非零，static run 标记 failed。

`--height > 0` 会检查 Flow v2，并选择 `stabilizer.estimator=2`、`stabilizer.controller=1`，等待 parameter acknowledgement。要求 `kalman.initialX/Y/Z/Yaw=0`，reset 一次 EKF，然后等待 2 s。这只是 settling wait，不是 EKF 收敛证明。检查 near-ground ToF、estimated Z、upright attitude、Flow status、motors off，再请求 arming，并等待 `supervisor.info` 报告 armed/can-fly。

主线程用公开的 `send_position_setpoint(x,y,z,yaw)` API 发 command：固定 X=Y=0、yaw=0；Z 以 0.2 m/s 从 0 ramp 到目标，settle 3 s，保持 `--duration` 秒，最后 ramp 到 0，再发送 stop/disarm。没有后台 setpoint thread。height 限制为 0.1-1.0 m，duration 最多 120 s，flight logging period 最多 50 ms。这些是本轮实验限制，不是经过真机认证的安全范围。

control mode 与 sim 的 XYZ position + absolute yaw 一致；**trajectory 不完全相同**：当前 sim demo 是 altitude step 后 hover/move，这里是 ramp takeoff 后固定点 hover。对比时使用稳态 hover 区间；比较 takeoff transient 前，应先给 sim 输入同样的 ramp。

第一次 Ctrl+C、一般 exception 或 logging failure 会尝试执行有限时长的 descent，然后 stop/disarm，且 run 标记 interrupted/failed。landing 再次中断或 link loss 会转入 stop/disarm best effort。**radio 断开时不能保证 command 送达，stop/disarm 也可能让空中的 drone 直接落下。Ctrl+C 不是独立可靠的 emergency stop。** 不使用 Ctrl+Z 暂停一个正在发送 flight command 的进程。

来源：[cflib commander API](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/cflib/crazyflie/commander/)、本机 cflib 0.1.34 source、[firmware position decoder](../src/modules/src/crtp_commander_generic.c)、[supervisor bitfield](../src/modules/src/supervisor.c)。已记录的 run-time PID/EKF selection 会保留到后续修改或 power cycle，logger 不自动恢复先前 controller。

## 6. 分析前必须区分的量

1. **Firmware estimate 与 ground truth。** `stateEstimate.*` 是 EKF output。没有 external reference 时，可以报告 estimated drift/jitter，不能把它命名为真实 position error 或 EKF estimation error。`stateEstimate.pitch` 使用 legacy inverted sign。
2. **Filtered IMU 与 filter 前的 noise。** `gyro.*` 是 deg/s，`acc.*` 是 Gs；都经过 firmware filter，gyro 还经过 boot bias correction。水平静止的 acc 期望约 `(0,0,1)` Gs。单个姿态的 mean 不能完整分离 accelerometer bias、scale 和 tilt。Hover variation 包括真实 motion/vibration，不能全部当 measurement noise。
3. **Flow raw frame 与 EKF frame。** `dpixelx=-motion.deltaY`、`dpixely=-motion.deltaX`。stock 要求 `motion.motion==0xB0` 且 count 的绝对值小于 100。保留所有 raw log，在分析时分别报告 valid fraction 与 rejected readings；不要只保存筛选后的好数据。静止有效 readings 的 mean/std 可表征这套条件下的 residual/noise，不保证是理想 white Gaussian noise。
4. **Logging time 与 sensor time。** `timestamp_ms` 是 firmware 生成 log packet 的时间，24-bit，约 4.66 h wrap；不是 acquisition timestamp。unwrap 在每个 block 内完成，若 run 刚好跨 wrap 起点，应核对各 block 的 epoch。不同 block 用有 tolerance 的 time alignment，不做 timestamp 必须完全相等的 join。host phase 在接收时标记，边界附近需要留 margin。
5. **ToF 重复值。** Flow v2 ToF nominal 40 Hz，100 Hz logging 会重复读到旧值；Flow nominal 100 Hz 也不与 log task 同步。不能用“数值相同”判断 duplicate，因为独立的新 reading 也可能相同。当前 stock log 没有 acquisition counter/dt，精确逐 sample 重建需要后续 instrumentation。
6. **PWM 与 force command。** 真机 `motor.m1..m4` 是 battery compensation 后的 uint16 PWM；sim 的 `motor_commands` 是 compensation 前的 force command。真机 `motor.m1req..m4req` 同样已包含 battery compensation，也不是替代的 pre-compensation log。不能直接比较两边是否都等于 40000。

来源：[IMU filtering](../src/hal/src/sensors_bmi088_bmp3xx.c)、[state/log variables](../src/modules/src/stabilizer.c)、[Flow axis/validity](../src/deck/drivers/src/flowdeck_v1v2.c)、[ToF driver](../src/deck/drivers/src/zranger2.c)、[log timestamp](../src/modules/src/log.c)、[motor voltage conversion](../src/drivers/src/motors.c)。上述 code 行为置信度高；噪声拟合方法是实验建议。

## 7. 哪些 sim variables 后面需要 replace

**这一轮先收集数据，不自动修改 sim 或 stock firmware。** 每个替换值应附 run ID、计算方法、单位、不确定度和验证结果。

| 优先级 | 要标定的量 | 需要什么真机信息 | 当前 file / variable / value | 后续如何处理 |
|---|---|---|---|---|
| P0，直接换 | 整机 mass | 完整配置 mass，多次称重 | `sim/run_sim.py`: `MASS_KG=0.030` kg | 用 `mass_g / 1000` 替换，注明对应 battery/decks |
| P0，已换（2026-11 static bench） | Gyro injected noise | motors-off filtered gyro、实际 logging rate；多个静止 run | `sim/sensors.py`: `GYRO_MEASURED_STD_DEG_S=(0.114, 0.169, 0.126)`，按 80 Hz LPF gain 0.416 换算成 injected std | sim 在 1 kHz filter **之前**加 noise。用同一 80 Hz LPF 和 logging sampling 后的 std/频谱匹配，不能直接填真机 filtered std |
| P0，已换（2026-11 static bench） | Acc injected noise | 已知静止姿态的 filtered acc、gravity reference、重复 run | `sim/sensors.py`: `ACCEL_MEASURED_STD_G=(0.00133, 0.00101, 0.00138)`，per-axis，按 30 Hz LPF gain 0.258 换算 | 去掉正确 gravity projection 后分析；按同一 30 Hz LPF 和采样匹配。只有一个 scalar 参数；若轴间差异大，需要先扩展成 per-axis model |
| P0，拟合后换 | Flow injected noise | 多高度、有效 status 的 raw counts、surface/lighting、mean/std/valid fraction | `sim/sensors.py`: `FLOW_NOISE_STD_COUNTS=2.0` | 在 firmware-aligned frame、interface counts 单位拟合。若 height/surface dependence、bias 或 quantization 显著，扩展 model，而不是强行只换 scalar |
| P0，已换（只测了 0.5-0.7 m） | ToF injected noise | 多个 tape heights、`range.zrange`、重复 run、height uncertainty | `sim/sensors.py`: `TOF_NOISE_STD_M=0.00206`（测得 1.65-2.06 mm，取上限） | mm 转 m；分别报告 mean bias 与 std。需要时改成 range-dependent noise model |
| P1，noise 已换；bias/heights 未测 | Multi-ranger noise / bias / optical origins | 已知 wall distance、sensor optical face position、方向、材质与入射角，含 invalid readings | `sim/ranger.py`: `RANGER_NOISE_STD_M=0.002`（测得 1.4-2.0 mm，取上限）, `RANGER_BIAS_M=0`, `RANGER_ORIGINS_BODY_M` 用 datasheet 位置，高度未测 | 单独拟合，不能直接复用 downward ToF calibration；若方向差异大，扩展 per-direction model。4 m 是 nominal maximum range，32767 是 firmware invalid sentinel |
| P1，当前不能直接换 | Gyro/acc/Flow/ToF bias | IMU 多姿态/温度实验、ToF tape reference、静止 Flow | **目前没有 bias constants** | 新增 sensor-model bias 参数后再注入。不要把 gyro boot 校正前 bias 当成已校正 output 的 residual bias |
| P1，独立识别 | Inertia | 含 battery/decks 的 geometry/mass distribution 或受控 dynamics experiment | `sim/run_sim.py`: `INERTIA_KGM2=(1.6572e-5,1.6656e-5,2.9262e-5)` | 需要关于 COM 的 inertia，单位 kg·m²；静止/hover log 不足以识别 Ixx/Iyy/Izz |
| P1，独立识别 | Additional sensor delay | 同步 external reference / sensor acquisition timing | `sim/sensors.py`: `GYRO_DELAY_MS=ACCEL_DELAY_MS=FLOW_DELAY_MS=TOF_DELAY_MS=0` | 现有 log timestamp 与 host arrival 不能分离 sensor delay、radio latency、filter lag。先识别，再填整数 ms；不要重复加入 filter 已产生的 lag |
| P1，新增 model | Motor response time | thrust/RPM step response 的同步记录，包含 motor/prop/battery 配置 | `sim/actuators.py`: 当前 instantaneous，**没有 motor time-constant variable** | 新增 motor dynamics state 和参数后验证，hover average 不足以估计 time constant |
| P1，核对 geometry | Arm length / motor positions / COM / deck offset | 实物几何、battery/deck placement、COM 测量或估计 | `ARM_LENGTH_M=0.046`；`MOTOR_POSITIONS_BODY_M` 由其推导；`FLOW_DECK_POSITION_BODY_M=zeros(3)` | 区分物理 geometry 和 stock EKF 假设。COM 目前由 `run_sim.py` 固定为零，需要新参数才能支持实测 offset；不要只改 Flow truth offset 却忘记记录 estimator mismatch |
| P1，核对后建 physical model | PWM-voltage-thrust curve / yaw reaction torque | 同步 PWM、voltage、测得 force/torque；多个 operating points | `sim/actuators.py`: `THRUST_MIN_N=0.012817578393224994`, `THRUST_MAX_N=0.12`, `THRUST_TO_TORQUE_M=0.0069928948992470565` | 这些来自 47-17 firmware，不是随意的 placeholder。`THRUST_MAX_N` 同时定义 command scale，不能直接拿实测最大 thrust 覆盖；模型还会检查 compiled value。若实物不同，需区分 stock command conversion 与新 physical actuator model |

`MASS_KG` 更新后，Python altitude PID 的 `MAX_THRUST_N=2*MASS_KG*9.81` 自动重算；这是测试限幅，不是实测最大 thrust，不应用它代替 motor characterization。

以下 **保留 stock baseline**，不因为 measurement noise 变了就一起修改：

| 类别 | Variables / settings |
|---|---|
| EKF measurement uncertainty | `FLOW_EKF_STD_COUNTS`、`TOF_STD_A_M`、`TOF_STD_B_M`、`TOF_DISTANCE_A_M`、`TOF_DISTANCE_B_M` |
| EKF model / validity checks | `FLOW_RESOLUTION`、`FLOW_NPIX`、`FLOW_THETAPIX`、`FLOW_MIN_HEIGHT_M`、`TOF_HALF_CONE_RAD`、`FLOW_OUTLIER_LIMIT`、`TOF_RANGE_LIMIT_M` |
| Firmware scheduling/filter settings | `IMU_HZ`、`FLOW_PERIOD_MS`、`TOF_PERIOD_MS`、`GYRO_LPF_HZ`、`ACCEL_LPF_HZ`。只有确认机载 firmware 配置不同，才重新审核；nominal rate 不等于 arrival jitter |
| Controller/estimator | stock PID gains、stock EKF process noise/covariance、measurement handling；没有新增 NIS gate |

一个重要区别：**simulated sensor noise 描述现实有多吵；EKF covariance 描述 stock firmware 认为 sensor 有多可靠。** 两者可以不同，baseline 不应为了让它们相同而偷偷调 EKF。

对真机 PWM 的初步分析可使用该机载版本的 `VMOTOR2THRUST0..3`，以 `V_motor ≈ V_battery * PWM / 65535` 代入 cubic curve，再检查稳态 sum of thrust 是否接近 mass × g。当前 repo 的系数位于 [platform_defaults_cf2.h](../src/platform/interface/platform_defaults_cf2.h)。这只是用 stock curve 做 consistency check，不是独立验证或识别该 curve；倾斜、voltage filtering、损坏 motor 和空气动力影响都要与结论一起报告。

## 8. Week 1 baseline deliverable

交付以下内容，先保留原始 data，再产生 analysis：

1. Hardware/firmware manifest：mass 原始读数、battery/prop/deck 配置、机载 revision、parameters、Python versions。
2. P0 static runs：原始 CSV + `meta.json`，按 height/run ID 列出 sample counts、时间间隔/gaps、有效 Flow 比例、gyro/acc mean/std、ToF bias/std、Flow mean/std。ToF bias = mean(distance_m) - measured_sensor_height_m；初次统计注明 repeated-read 限制。
3. P0 hover runs：保存 `setpoints.csv`；只用 `phase=hover` 的内部稳定区间计算 stats，排除 takeoff/settle/landing 和 phase 边界。记录 estimated XYZ/velocity/attitude、PWM、voltage、IMU 与 Flow/ToF 的 time series。
4. Comparison report：同 units、同 sampling/filter/control mode 对比 sim；注明真实 trajectory 是 ramp、没有 external reference 时没有 ground-truth error。报告每个 repeat，不能只选最好的一次。
5. Calibration table：每个参数的旧值、新值/unknown、单位、run ID、拟合方法、不确定度，以及是否已应用。当前只承诺 mass 可直接替换；inertia、motor time constant、delay 未识别就保留 unknown。

logger 本身负责采集，不会把某个 std 自动写回 sim，也不会把所有 hover variation 自动解释为 sensor noise。后续 analysis 应保留 static residual、真实 motion 和 estimator error 的区别。

## 9. Tests、Git 与 troubleshooting

以下只运行 offline tests，不访问 radio：

```bash
python -m unittest discover -s real/tests -v
python real/log_flight.py --help
```

Tests 覆盖 native TOC packet size、第一包缺失、async error、stale data、disconnect、timestamp wrap、close 后迟到的 callback、partial startup cleanup，以及 takeoff/hover/landing/arming 中断后的 stop/disarm 尝试。mock 验证控制流程，不验证物理飞行。

`real/.gitignore` 忽略 `cache/`、`.venv/`、Python cache 和工作用 `session.json`。每个 run 的 session snapshot 留在 `meta.json`。少量代表性 baseline raw data 可以随项目 commit，但需要显式选择 run，不把全部临时试验都自动加入 Git。

本目录原有 `hover_test.py` 是旧 experiment，会执行 forward/back motion，不是这次 logger，也没有经过本次 cleanup 修改。首次 baseline 使用 `log_flight.py`。

| 问题 | 处理 |
|---|---|
| `Resource busy` / radio unavailable | 关闭 cfclient 和其他 radio client；核对 udev/group/实际 URI |
| Missing TOC variable | 核对机载 firmware 和 log TOC；保存 error，不直接删掉 required variable 后飞 |
| 第一包 timeout / stale logging | 先 static 检查 link 和 bandwidth；查看 meta counts/gaps，可尝试 `--period 20` |
| Flow status invalid | 检查光照、纹理、距离和视线；不要把 invalid reading 归入纯 sensor noise |
| Flight selection/initial-state 检查失败 | 核对 PID/EKF 与 `kalman.initial*`；不同实验坐标原点需要显式设计 |
| `meta.json` 的 status 是 failed/interrupted | 保留该 run 用于诊断，不按正常 completed baseline 使用；查看 error/cleanup errors |
| Motor PWM 不平衡或 attitude 抖动 | 可能涉及 prop/motor、安装、COM、估计或控制问题，不能仅凭四条曲线唯一定位某个 motor |

正式 flight 前先解决已知 motor/prop 异常，完成 motors-off logging，再做短低高度 smoke flight。修正版的真机验证结果应补在对应 run 的 metadata/report 中。
