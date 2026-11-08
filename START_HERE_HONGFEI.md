# Hongfei：水平 LQR 集成与运行说明

代码基于 Hongfei 分支提交 `96259a3cf0f0911f121c4336335455ef3be446a3`。
这是一份可编译、可测试的实现；**不是已经完成三周实机任务的声明**。

## 先看现在完成了什么

- 同一份 C 水平 LQR 同时供机载 PID 控制链和 Python 固件绑定调用。
- 水平控制在 100 Hz 更新；原版高度、姿态和角速度 PID 保留。
- 运行参数 `hlqr.enable=0/1` 切换水平 PID/LQR，默认 0。
- 支持世界系位置保持、导航现有的机体系速度命令、非零 yaw。
- 提供显式制动覆盖接口，丢弃旧位置目标；不包含 Kelvin 尚待移植的自动障碍判定策略。
- Isaac 模式 `firmware_lqr` / `ekf_lqr`、统一 PID 对比时间表、Flow/ToF 延迟配置。
- 实机日志记录实际角度参考、模式、限幅、更新次数、水平计算耗时与周期。
- 可选 0.25 m 水平往返脚本，原有中断、降落和停机收尾保留。

## 在组员已经能运行仿真的电脑上

先把补丁应用到仓库。保留团队已验证的板型、螺旋桨和 `.config`，不要因为这份补丁重新选择硬件配置。
下面命令在原有 Isaac 容器/环境的仓库根目录执行：

```bash
git submodule update --init --recursive
make bindings_python PYTHON=/isaac-sim/python.sh
/isaac-sim/python.sh bindings/setup.py build_ext --inplace --force
export PYTHONPATH="$PWD/build:$PYTHONPATH"

/isaac-sim/python.sh -m unittest discover -s sim/tests -p 'test_lqr_bindings.py' -v
```

先跑一对无估计器的 PhysX 测试：

```bash
/isaac-sim/python.sh sim/lqr/compare.py --run --scenario truth --seeds 7 --out build/lqr/physx_truth
```

再跑 EKF 的正常、噪声、噪声加延迟测试：

```bash
/isaac-sim/python.sh sim/lqr/compare.py --run --scenario ekf_clean ekf_noise ekf_noise_delay --seeds 7 11 23 --out build/lqr/physx_ekf
```

结果目录自动生成真实运行的 CSV、命令记录、终端日志、`metrics.json`、`metrics.csv`、`comparison.png`。
已有结果不会被覆盖；重复实验请换一个 `--out`。
若失败，先看对应 `.stdout.txt`；未生成的结果不会被伪造。
完整批次为 18 次独立仿真，Isaac 启动可能较慢。每次独立进程避免固件静态变量互相污染。

想先观察动画：

```bash
/isaac-sim/python.sh sim/run_sim.py --mode firmware_lqr --comparison-demo --hover-seconds 10 --ranger off
```

已有导航场景生成好后，可运行：

```bash
/isaac-sim/python.sh sim/run_sim.py --mode firmware_lqr --mission matlab_main
```

该命令验证导航接口，不代表自动制动已接入；`safety/matlab-braking` 分支仍需协调合并与端到端验证。

## 控制模型和接口

状态为世界系 `[x,y,vx,vy]`，单位 m、m/s；世界 Z 向上。
机体采用仓库现有 FLU：X 向前、Y 向左、Z 向上。
输入为 Crazyflie legacy `[roll_ref,pitch_ref]`，设计内部用 rad，固件接口用 deg。
legacy pitch 与标准右手欧拉 pitch 符号相反。yaw 为绕世界 +Z 的右手正方向。

在 yaw=0、小倾角、近悬停、原版姿态环能较快跟随的近似下：

```text
dx/dt  = vx
dy/dt  = vy
dvx/dt = -g * pitch_ref - beta * vx
dvy/dt = -g * roll_ref  - beta * vy
beta = 0.5 /s  [假设值]
Ts = 0.01 s

A = [0 0 1 0; 0 0 0 1; 0 0 -beta 0; 0 0 0 -beta]
B = [0 0; 0 0; 0 -g; -g 0]
Q = diag(20,20,4,4)
R = diag(300,300)

P = Q + Ad' P Ad - Ad' P Bd (R + Bd' P Bd)^-1 Bd' P Ad
K = (R + Bd' P Bd)^-1 Bd' P Ad
u = -K * (state - hold_reference)

K ~= [0, -0.255541802, 0, -0.209479167;
      -0.255541802, 0, -0.209479167, 0]
```

`sim/lqr/design.py` 使用精确 ZOH 和离散 Riccati 迭代；测试用 SciPy 的矩阵指数和 DARE 独立交叉验证。
C 编译时检查 POSITION_RATE=100。要改变频率，需要重新设计离散模型，不能只改调用速度。

非零 yaw 时先在世界系算期望水平加速度，再旋转到机体水平轴：

```text
ax_body =  cos(yaw)*ax_world + sin(yaw)*ay_world
ay_body = -sin(yaw)*ax_world + cos(yaw)*ay_world
pitch_ref = -ax_body/g
roll_ref  = -ay_body/g
```

当前 X/Y 阻力、权重和增益对称，因此上述转换与固定 yaw 水平设计一致。
若以后两轴参数不一样，需要重新处理旋转后的耦合，不能只替换一个数。

| 请求 | 行为 |
|---|---|
| `position_m=(x,y,z)` | XY 位置保持，速度参考为零；Z 用原版高度 PID |
| `velocity_body_m_s=(vx,vy)` | 机体系速度跟踪；忽略 `position_m` 的 XY，保留 Z |
| `yaw_rad` / `yaw_rate_rad_s` | 按现有绝对 yaw / yaw-rate 接口交给原版内环 |
| `brake=True` | 仿真接口强制零水平速度、零 yaw-rate，保留高度 |
| 手动姿态、零推力停机 | 保留原版处理，不由 LQR 覆盖 |

速度模式使用同一个速度反馈增益和 `beta*v_ref` 阻力前馈，不积累虚拟位置目标。
这是明确的速度跟踪模式，**不是另一次四状态位置 LQR 最优性证明**。
连续轨迹的非零速度/加速度前馈还没有接入 position-hold 模式；与 Jerry 的连续轨迹接口需要另行明确。

固件 C 接口：
- `horizontalLqrSetEnabled(bool)`，对应 `hlqr.enable`。
- `horizontalLqrSetBrake(bool)`，对应 `hlqr.brake`。在 LQR 自动水平模式下优先于位置/速度参考。
- 制动调用方负责障碍判定、有效期、释放/恢复和 yaw 指令；手动停机不被覆盖。
- 上层安全监督器应每次明确提供当前状态，不应把一次参数写入当成完整看门狗。

限幅为 roll、pitch **各自** +/-10 deg，不是合成倾角最大 10 deg。
仿真 PID 和实机对比脚本也设为相同的逐轴限制。
这仍是带饱和的近悬停控制，线性闭环极点并不能证明所有饱和情况稳定。

## 参数更新

```bash
python sim/lqr/design.py --beta 0.5 --q-position 20 --q-velocity 4 --r-angle 300 --write-header
```

生成 `build/lqr/design.json` 和 `src/modules/interface/controller/horizontal_lqr_gains.h`。
改完必须重新编译 Python 绑定和固件。
目前 beta、质量、惯量和理想电机模型都不是新测量结果。
当前默认 PhysX 执行器没有显式气动阻力，需与 Kelvin 确认设计/仿真参数差异。
增大位置权重或降低输入权重通常使控制更积极；必须重新比较超调、限幅及高度误差。

## 固件与实机

在你们已验证的固件工具链中，用同一硬件配置编译：

```bash
make -j4
```

代码已加入 Kbuild，不需要另建 out-of-tree 控制器。
仍选择 `stabilizer.controller=1`；LQR 是其中可切换的水平模块。
沿用团队已有流程烧录；本代码包没有执行烧录或飞行。

在已确认原版悬停正常、且新固件参数/日志存在后，分别进行监督下的悬停实验：

```bash
python real/log_flight.py --name pid_hold --height 0.3 --duration 10 --horizontal pid --lqr-log
python real/log_flight.py --name lqr_hold --height 0.3 --duration 10 --horizontal lqr
```

定点确认后才做短航点：

```bash
python real/log_flight.py --name lqr_short --height 0.3 --duration 5 --horizontal lqr --waypoint-x 0.25
```

原有 `--uri`、`--metadata`、`--ranger` 参数继续可用。
脚本只在执行飞行时设置参数；被动记录不会自动起飞。
`--waypoint-x` 依次执行悬停、X 方向目标、返回；降落阶段保持当时水平参考。
该脚本没有自动避障，不能代替 Kelvin 的安全监督器。

`lqr.csv` 中的 `controller.roll/pitch` 是 PID 与 LQR 都适用的实际角度参考，
不能用 PID 模式下未更新的 `hlqr.rollRef/pitchRef` 来算 PID 控制用量。
`lqr_timing.csv` 记录水平更新周期与计算时间；它不是完整 stabilizer 的执行时间。
完整控制循环时间还应结合现有 stabilizer 日志/机载测量。
真机 stateEstimate 是估计值；没有独立定位参考时不能把它当真实位置误差。

## 验证证据与三周任务状态

本机完成：主机编译真实 C PID/LQR/电机分配、完整 cffirmware 绑定编译，
76 项测试通过；另有一条上游 NumPy 协议弃用提示。
原有 EKF 测试的单精度 DC 容差从小数位判定改为 5e-5 m/s²：
相关传感器、滤波器、EKF 和 subsampler 源码核对为未改变，主机编译器下约 3 ppm 偏差。

CPU 对比使用同一份实际 C 控制器、四电机分配和非线性四元数刚体模型。
噪声/延迟场景为合成 XY 状态反馈，**不是 stock EKF 的噪声/延迟实验**。
它检查软件链和基本动态，不替代团队的 PhysX 仿真。

当前权重下，无阻力 CPU 场景的 0.5 m 阶跃：
- PID：位置移动段 RMSE 约 0.117 m，超调 11.67%，角度用量 0.0322 rad²·s。
- LQR：位置移动段 RMSE 约 0.132 m，超调 8.11%，角度用量 0.00655 rad²·s。
- 表现是平缓控制与跟踪速度之间的取舍，不能写成“LQR 所有指标优于 PID”。

| 周任务 | 此包完成的部分 | 仍需团队执行 |
|---|---|---|
| Week 1 | 模型、离散化、符号/单位/yaw/估计接口、增益复现 | 与 Jiaming/Kelvin 确认接口；基线烧录、实机日志证据 |
| Week 2 | C LQR、限幅、真实内环连接、非线性 CPU 对比、PhysX/EKF 对比工具 | 在组员电脑跑 PhysX 批次；据结果调权重并报告真实指标 |
| Week 3 | 机载源码入口、可选 PID/LQR、导航速度接口、显式制动优先接口、实机记录脚本 | 目标板编译/烧录、Kelvin 自动制动策略集成、机载耗时、定点/短飞与日志 |

## 本地 CPU 复现（可选）

需要 C 编译器、Python、NumPy、SciPy（独立数值测试）、Matplotlib（绘图）。

```bash
python sim/lqr/native.py
python -m unittest discover -s sim/lqr -p test_lqr.py -v
python sim/lqr/cpu_demo.py
```

Windows 可设置 `LQR_CC` 为 zig.exe 完整路径；Linux 默认寻找 gcc/clang。
CPU 结果在 `build/lqr/cpu/`。无论 CPU 或 PhysX，均按真实生成的 CSV 报告，缺失实验保持待验证。

