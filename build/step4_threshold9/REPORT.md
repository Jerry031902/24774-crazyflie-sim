# Step 4：threshold 9.0 matched comparison

80/80 cases 已执行完毕，未运行 threshold 6.63。全部使用 simulation：flow-empty、flow-shuttle-x、closed loop、noise on；threshold=9.0、maxReject=10，未调参。73 个 runs 跑满 62 s，7 个提前结束均为 zero 的 20 m position-bound failure。所有 runs 的 contact count 为 0。

Step 3 已按明确的 20 个 paths commit 为 `28a5e3b2fc98056bdaeecbccab3855612d034fdb`；没有 push。[完整 file list 和 commit stat](step3_commit.txt)：20 files，+1067/−42。Step 4 的 source changes 尚未 commit，见 [完整 diff](step4_review.patch) 和 [file list / stat](step4_diff_stat.txt)。vendor/CMSIS 未动。

[全部 per-seed、mean/SD、paired differences 和 better counts](analysis/TABLES.md) · [每个变差的 seed/metric](analysis/REGRESSIONS.md) · [per-seed CSV](analysis/per_seed.csv) · [paired CSV](analysis/paired.csv) · [summary CSV](analysis/summary.csv)

**固定设置和定义。** Seeds 为 7、17、27、37、47；noise seed=fault seed，两个 modes 按同 seed 配对。spike probability=0.1，scale factor=0.5，fault window=[26,30) s，post 5 s=[30,35) s。off 使用相同的时间段。XY RMS=`sqrt(mean(ex²+ey²))`，max 为 XY error norm 的最大值。所有 ± 都是 seeds 之间的 sample SD，ddof=1；Δ=gated−stock。

Recorded metadata 确认每个 run 的 mass=0.033 kg，gyro pre-filter std=(0.27394133,0.40610601,0.30277726) deg/s，accel pre-filter std=(0.00516457,0.00392197,0.00535873) G，ToF std=0.00206 m，flow std=2 counts。[Metadata/hash 检查](run_integrity.json) 和 [原始数据 SHA-256 manifest](raw_manifest.json) 已保存。

Recovery 保持 Step 3 定义：相对于同 mode、同 seed 的 clean reference，estimate error ≤ reference+0.05 m、tracking error ≤ reference+0.10 m，连续保持 1 s，分别计算 estimate、tracking 和 joint。Recovery=0 不是 error=0。未观察到 recovery 标为 censored；mean/SD 只含观察到 recovery 的 seeds，n 明列。不能把不同 recovered subsets 的 mean 当成完整 5-seed comparison。

**1. Clean cost。** 没有看到 gated 在所有 seeds 一致变差。Full-run XY estimate max：stock **58.90 ± 24.24 cm**，gated **59.95 ± 23.70 cm**，paired Δ **+1.05 ± 7.87 cm**，3/5 改善、2/5 变差。Seed 7 的 +6.25 cm 在本组 seed spread 的范围内，不能单凭它认定系统性的 +6 cm penalty；也不能用 spread 大来宣称 gate 没有成本。该 paired mean 的 descriptive 95% t interval 是 [−8.72,+10.82] cm，5 seeds 不足以证明 equivalence。四项 RMS 的 mean 都略降，但 during 只有 3/5 改善、post 只有 2/5 改善。Clean rejection=**0.253 ± 0.054%**，forced=0。Seed 27/37 的 clean stock 和 gated 都未通过 final tracking check。

**2. Zero / scale / dropout。**

- **Zero 明显更糟。** Stock 有 3/5 跑满 62 s，gated 为 0/5；gated 全部在 30.903–31.609 s 越界。Fault-active rejection=0%，window 结束后 X clean rejection=90.098 ± 0.070%，Y=40.677 ± 39.419%。Gated forced counts 按 seed 顺序为 **14、19、16、12、11**，mean=14.4 ± 3.21，全部在正常 flow 恢复后发生。During estimate/tracking RMS 的 Δ 为 +7.97 ± 12.67 / +7.28 ± 11.67 cm，各有 4/5 变差。两个 modes 均为 0/5 joint recovery。Gated 没有一个完整 post-5-s window，因此其 5 s RMS 和 paired Δ 均为 NA；stock 只有 3 个完整窗口，不能冒充 5-seed mean。Observed estimate max 的 stock/gated mean 为 11.99 ± 1.19 / 17.64 ± 0.53 m，但 stock 也有 2 个截短 run，所以不能将这个 observed difference 当成完整 62 s 的 mean difference。对 stock 完整的 seeds 7/17/27，gated 在提前结束前的 max 已更大，已足以证明这三个 paired max 变差。
- **Scale 0.5 也有代价。** Post estimate/tracking RMS Δ=+4.28 ± 7.25 / +4.43 ± 7.19 cm，full estimate/tracking max Δ=+4.74 ± 7.34 / +4.58 ± 6.42 cm，四项均为 4/5 变差。During estimate RMS 也是 4/5 变差，虽然 mean 因另一个 seed 的改善而略降。Fault-active rejection 只有 0.675 ± 0.360%，post-5-s clean rejection=0.660 ± 0.251%，forced=0。全部 runs 跑满 62 s，两个 modes 均无 joint recovery。这里没有 zero 那种 forced/lockout，但 gate 也没有有效挡住 scale fault。
- **Dropout 结果混合，没有 zero 那样的 lockout。** 每个 run 都有 400 个 active samples 没有 enqueued，active rejection rate=NA，不能计作 rejected。Forced=0；post-5-s clean rejection=0.380 ± 0.217%，与本轮 clean group 相同。六项 error metrics 的 mean 都略降，但都有个别 seeds 变差；full tracking max 甚至是 3/5 变差。Joint recovery 为 stock 1/5、gated 3/5：seed 37 从 1.20 s 变为 2.19 s；seed 17/47 则从 censored 变为 22.11/3.21 s；seed 7/27 两侧均 censored。Window 内没有 gate decisions，不能把这些差异解释为 gate 检测到了 dropout。

Zero 的 logs 显示：fault 内低于 threshold 的错误 flow 被接受，正常 flow 恢复后反而出现持续的高 NIS。此时高 NIS 反映了偏离的 state estimate，而不一定是坏 measurement。拒绝这些正常 updates 与更慢的 reacquisition 一致；Nth full-weight forced accept 没能避免越界。见 [seed 7 的 error / NIS / decisions](analysis/zero_seed7_reacquisition.png)。这是从本轮 logs 作出的解释，不代表真实 PMW3901 在无 texture 时必然产生 zero。

**3. Spikes 从哪里开始有一致改善。** 事先固定的 descriptive 判据是：四项 during/post RMS 都有负的 mean Δ，且每项至少 4/5 seeds 改善。最先满足的是 **10 counts**；5 counts 只有 3–4/5 seeds 改善，full estimate max mean 还增加 0.51 cm。20 counts 的四项 RMS 改善数为 5/4/5/5；50 counts 的六项 error metrics 全部 5/5 改善。这个判据不是 population significance 的声明。Fault-active rejection 分别为 27.63 ± 7.49%、98.44 ± 0.61%、100%、100%；forced 全部为 0。Gated 20/50 的 5 个 state CSV 全部 byte-identical。仍有 flight-check regression：seed 27 的 spike 20 和 50 是 stock-pass / gated-fail 的 final target-error check，即使 spike 50 的六项主要 error metrics 都更好。

**所有 cases 的 paired error differences。** 单位 cm；括号为 better seeds / 5。完整 stock/gated mean ± SD 和逐 seed 数值见 [TABLES.md](analysis/TABLES.md)。obs 表示不同长度 run 的 observed max difference；不能当作完整 62 s 的比较。? 表示 indeterminate。


| Case | During estimate RMS | During tracking RMS | Post estimate RMS | Post tracking RMS | Full estimate max | Full tracking max |
| --- | --- | --- | --- | --- | --- | --- |
| off | -2.55 ± 4.51 (3/5) | -2.26 ± 4.67 (3/5) | -0.40 ± 5.12 (2/5) | -0.50 ± 4.97 (2/5) | 1.05 ± 7.87 (3/5) | -1.44 ± 8.02 (3/5) |
| spike 5 | -2.31 ± 5.89 (4/5) | -2.74 ± 3.27 (3/5) | -0.93 ± 7.11 (3/5) | -1.01 ± 7.04 (3/5) | 0.51 ± 7.57 (2/5) | -3.06 ± 7.13 (4/5) |
| spike 10 | -6.27 ± 5.94 (4/5) | -4.35 ± 8.59 (4/5) | -8.36 ± 6.05 (5/5) | -8.16 ± 5.63 (5/5) | -3.57 ± 15.22 (3/5) | -4.72 ± 15.98 (4/5) |
| spike 20 | -13.57 ± 8.88 (5/5) | -10.73 ± 11.91 (4/5) | -19.71 ± 11.11 (5/5) | -19.21 ± 10.21 (5/5) | -11.17 ± 24.56 (4/5) | -13.35 ± 19.80 (4/5) |
| spike 50 | -42.62 ± 19.74 (5/5) | -42.31 ± 17.74 (5/5) | -60.83 ± 33.11 (5/5) | -60.75 ± 33.05 (5/5) | -60.91 ± 37.20 (5/5) | -65.06 ± 37.64 (5/5) |
| zero | 7.97 ± 12.67 (1/5) | 7.28 ± 11.67 (1/5) | NA (0 complete pairs) | NA (0 complete pairs) | obs 564.56 ± 68.98 (0/5; ?2) | obs 99.17 ± 106.50 (0/5; ?2) |
| dropout | -2.53 ± 5.77 (4/5) | -2.88 ± 5.49 (3/5) | -1.22 ± 5.66 (3/5) | -1.50 ± 5.63 (3/5) | -3.29 ± 7.59 (4/5) | -0.47 ± 10.60 (2/5) |
| scale 0.5 | -0.13 ± 6.06 (1/5) | -0.60 ± 5.06 (3/5) | 4.28 ± 7.25 (1/5) | 4.43 ± 7.19 (1/5) | 4.74 ± 7.34 (1/5) | 4.58 ± 6.42 (1/5) |



**Gate 和 joint recovery。** Rate 在每个 seed 内先按 X/Y scalar updates pooled，再对 seeds 求 mean/SD；X、Y 分别的数据和所有分母见完整表。Stock 的 rejection / forced 均为 0（off/dropout 的 active rate 为 NA）。Zero 的 post rate 只覆盖提前结束前的数据。


| Case | Gated active rejection % | Gated clean rejection % | Gated post-5-s clean % | Gated forced count | Stock joint recovery s | Gated joint recovery s |
| --- | --- | --- | --- | --- | --- | --- |
| off | NA | 0.253 ± 0.054 | 0.380 ± 0.217 | 0.000 ± 0.000 | NA (n=0) | NA (n=0) |
| spike 5 | 27.629 ± 7.488 | 0.260 ± 0.057 | 0.380 ± 0.217 | 0.000 ± 0.000 | 6.013 ± 8.555 (n=5) | 5.568 ± 9.990 (n=5) |
| spike 10 | 98.436 ± 0.610 | 0.255 ± 0.057 | 0.380 ± 0.217 | 0.000 ± 0.000 | 7.107 ± 11.112 (n=4) | 1.410 ± 3.153 (n=5) |
| spike 20 | 100.000 ± 0.000 | 0.255 ± 0.057 | 0.380 ± 0.217 | 0.000 ± 0.000 | 0.000 ± 0.000 (n=2) | 0.336 ± 0.752 (n=5) |
| spike 50 | 100.000 ± 0.000 | 0.255 ± 0.057 | 0.380 ± 0.217 | 0.000 ± 0.000 | 15.970 ± 2.121 (n=2) | 0.336 ± 0.752 (n=5) |
| zero | 0.000 ± 0.000 | 3.251 ± 0.954 | 65.388 ± 19.710 | 14.400 ± 3.209 | NA (n=0) | NA (n=0) |
| dropout | NA | 0.243 ± 0.055 | 0.380 ± 0.217 | 0.000 ± 0.000 | 1.200 ± NA (n=1) | 9.170 ± 11.218 (n=3) |
| scale 0.5 | 0.675 ± 0.360 | 0.267 ± 0.066 | 0.660 ± 0.251 | 0.000 ± 0.000 | NA (n=0) | NA (n=0) |



**Mean 变差的 performance metrics。** 下表包含所有存在可比较 paired Δ、且 mean 变差的 error/recovery metrics。Censored recovery 的 paired n 可能小于 5。另有 zero 的 observed max 增加，已在上文单列。每个 seed 的全部变差项，包括 mean 改善但个别 seed 变差的项目、clean rejection 增加、forced count 增加和 flight checks，均在 [完整 regression 清单](analysis/REGRESSIONS.md)。


| Case | Metric | Unit | Paired Δ mean ± SD | Comparable n | Worse seeds |
| --- | --- | --- | --- | --- | --- |
| off | full_estimate_max_m | cm | 1.048 ± 7.868 | 5 | 2/5 |
| spike 5 | full_estimate_max_m | cm | 0.515 ± 7.574 | 5 | 3/5 |
| spike 5 | recovery_tracking_s | s | 1.707 ± 3.818 | 5 | 1/5 |
| zero | during_estimate_rms_m | cm | 7.972 ± 12.667 | 5 | 4/5 |
| zero | during_tracking_rms_m | cm | 7.281 ± 11.669 | 5 | 4/5 |
| dropout | recovery_estimate_s | s | 0.990 ± NA | 1 | 1/5 |
| dropout | recovery_tracking_s | s | 0.134 ± 0.746 | 3 | 2/5 |
| dropout | recovery_joint_s | s | 0.990 ± NA | 1 | 1/5 |
| scale 0.5 | post_5s_estimate_rms_m | cm | 4.277 ± 7.246 | 5 | 4/5 |
| scale 0.5 | post_5s_tracking_rms_m | cm | 4.428 ± 7.189 | 5 | 4/5 |
| scale 0.5 | full_estimate_max_m | cm | 4.737 ± 7.339 | 5 | 4/5 |
| scale 0.5 | full_tracking_max_m | cm | 4.579 ± 6.424 | 5 | 4/5 |



所有 cases 的 X/Y/pooled clean rejection rates 都高于 stock=0，范围包括全程 clean、post-5-s clean 和 all-post-window clean。Zero 的 forced count 也增加；这是发生次数的增加，不能单凭 count 把一次 forced accept 判为错误。对应的所有 seed、axis、scope 和数值均已列入 regression 文件，没有省略。

**验证与文件。** 40/40 stock–gated pairs 的实际 flow noise 和 fault schedules 匹配；30/30 spike-amplitude schedule comparisons 匹配。每个 run 的 pre-fault state prefix 与同 mode clean reference 匹配，实际 C NIS/decision/streak/counters/labels 检查通过。三个 fresh seed-7 compatibility runs 的 17 个既有 CSV 与 Step 3 byte-identical。Default-off 的 `--no-render` / `--no-plots` 只用于减少 visual updates 和逐 flight PNG；记录的 physics、controller、sensor 行为已经通过这些 CSV comparisons 验证。所有原始 CSV 已保留，不因 fail 或截短而删除。

最终 Python unit suite 是 **118 tests：117 pass，1 个原有 failure**；相对保存的 failure baseline 新增失败为 **0**。7 个新增 statistics tests 全部通过。Teammate-owned sources 的 hashes 未变；Safety flights 本步未重跑，native binary 未重建且 hash 未变。STM32 toolchain 在这台机器上不可用，未验证 embedded build。详见 [test baseline delta](test_baseline_delta.json)、[unit log](sim_tests_final.log)、[run integrity](run_integrity.json)。

[Clean paired plot](analysis/clean_pairs.png) · [Spikes paired plot](analysis/paired_spikes.png) · [Zero/dropout/scale paired plot](analysis/paired_zero_dropout_scale.png) · [Zero reacquisition plot](analysis/zero_seed7_reacquisition.png)

**参数建议，尚未执行。** maxReject=10 在 zero 后没有及时接回正常 flow；我建议另行比较更短的 maxReject，以量化 reacquisition 与 outlier protection 的 tradeoff，但本轮数据不能替代这项 comparison 来选新 default。原先考虑的 forced-update R inflation 只能减小坏 forced sample 的 correction；面对 zero 后正常 flow 被拒的情况，降低 correction 也可能拖慢 recovery，因此也需要独立验证。本轮 threshold、maxReject、R 都保持不变。工作停在 threshold 9.0 报告，未开始 6.63，也未 commit Step 4。

