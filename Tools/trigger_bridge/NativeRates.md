# 原生触发频率依据与 1000 Hz 固定调度表

AGENTS.md：2026-10-27，依据本地 `ROC-RK3588S-PC` 分支 commit `2f629949b0` 分析，未运行原生 PX4 测频。区分参数值、正常消息触发、备用定时、输出率和实际 Run 频率。

## 分析结论

原生版本主要由消息触发，没有一份能静态直接读取的、覆盖全部 WorkItem 的固定运行频率表。实际 Run 率取决于传感器输入、订阅累计样本、输入队列、条件分支及 CPU 调度。已有 parameters.bson 还可能覆盖默认值，set-default 不等于强制设置。

本次提供的是**按参考分支参数及消息链推导的初始触发配置**，不是已测得的原生运行率。所有频率均可以通过 `--rate` 或 JSON 修改；最终对比前应使用相同输入实测原生 Run 和有效输出率，再替换配置。

## 参数及代码证据

下列路径和行号均针对参考分支，而非当前已修改工作树。可用 `git show ROC-RK3588S-PC:路径` 复核。

| 项目 | 分支依据 | 含义 |
| --- | --- | --- |
| IMU 积分默认值 | `src/modules/sensors/vehicle_imu/imu_parameters.c:50`，IMU_INTEG_RATE=200 | 目标积分/输出率，不直接等于 VehicleIMU::Run 率 |
| HITL 覆盖 | `posix-configs/firefly/HITL.config:39–40`，EKF2_PREDICT_US=1000、IMU_INTEG_RATE=1000 | HITL 初始配置；保留已有参数时仍可能不同 |
| 角速度发布目标 | `src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.c:140`，IMU_GYRO_RATEMAX=400 | 结合实际采样率计算 required_updates 和最小发布间隔 |
| 角速度实际限频逻辑 | `VehicleAngularVelocity.cpp:133–154,925–944` | 回调批次、样本时间戳和发布阈值共同作用，不是严格 400 Hz 定时器 |
| Real IMU 驱动离散化 | `src/lib/drivers/gyroscope/PX4Gyroscope.cpp:76`、`src/drivers/imu/invensense/icm42688p/ICM42688P.hpp:73–75`、`ICM42688P.cpp:325–336` | 驱动从 IMU_GYRO_RATEMAX 取目标速率，再按 1000 μs FIFO 样本间隔取整；默认 400 Hz 的 2.5 ms 在该计算中可变成 3 ms，进一步影响积分批次与实际消息率 |
| 加速度回调批次 | `VehicleAcceleration.cpp:106–116` | 按 IMU_INTEG_RATE 和实际样本间隔设置 required_updates |
| IMU Run 与发布的差别 | `VehicleIMU.cpp:685–733`，UpdateIntegratorConfiguration | 回调批次由积分样本数与队列深度决定；一次 Run 可消费多条数据，不能用积分率冒充实测 Run 率 |
| 磁力计原始输入 | `src/drivers/magnetometer/bosch/bmm150/BMM150.hpp:131`、`BMM150.cpp:352` | BMM150 20 Hz ODR、50 ms 周期；异常/初始化重试另计 |
| 磁场结果发布目标 | `src/modules/sensors/sensor_params_mag.c:105`，SENS_MAG_RATE=15 | 结果发布率限制，不能把它当作 vehicle_magnetometer 的触发率 |
| 气压计原始轮询 | `src/drivers/barometer/invensense/icp201xx/ICP201XX.cpp:295`，1_s/30 | 原始驱动名义 30 Hz 轮询，不保证每次均发布有效压力 |
| 空气数据发布目标 | `src/modules/sensors/vehicle_air_data/params.c:55`，SENS_BARO_RATE=20 | 结果发布限制，与原始 30 Hz 轮询和 WorkItem 回调率不同 |
| EKF 预测默认周期 | `src/modules/ekf2/module.yaml:10–20`，10000 μs | 名义 100 Hz 滤波更新；实际为 IMU 更新的整数倍 |
| EKF Run/姿态输出 | `src/modules/ekf2/EKF2.cpp:648–653` | 每次获得新 IMU 时更新输出预测器并发布姿态 |
| EKF 位置输出 | `src/modules/ekf2/EKF2.cpp:702–709` | 仅 `_ekf.update()` 成功时发布 local_position；不能把所有 EKF 输出都标成 100 Hz |
| 位置控制/悬停推力 | `MulticopterPositionControl.cpp:65`、`MulticopterHoverThrustEstimator.cpp:62` | 由 vehicle_local_position 消息触发 |
| 姿态/角速度控制 | `mc_att_control_main.cpp:78`、`MulticopterRateControl.cpp:67` | 分别跟随 vehicle_attitude 和 vehicle_angular_velocity |
| 控制分配 | `ControlAllocator.cpp:93–104` | torque/thrust 两路回调，按队列状态可能合并；名义跟随角速度控制链 |
| 仿真执行器输出 | `src/lib/mixer_module/mixer_module.cpp:317–340` | 优先消息回调；4 ms/250 Hz 是部分情形的 fallback，不是统一原生频率 |

VehicleAngularVelocity、VehicleAcceleration、Sensors 的 10 ms，EKF2/位置控制的 100 ms，以及磁场/气压/分配的 50 ms 备用定时都没有直接用作正常频率。

参考分支 `posix-configs/firefly/rk3588s_mc.config` 未覆盖上述积分/预测默认值，因此 Real 初始表按默认值推导。分支里另有 `posix-configs/phytiumpi/RealFlight.config`，也不能仅凭文件名就当作 RK3588 的实际启动配置。本次没有修改任何 PX4 启动文件、参数或仿真模式。

## 初始调度表

| 同 tick 发送次序 | 目标 | Real 请求 Hz | HITL 请求 Hz | 选择依据与限制 |
| --- | --- | --- | --- | --- |
| 1 | vehicle_magnetometer | 20 | 20 | Real 按 BMM150 原始速率；HITL 是初始假设，需按仿真输入修订 |
| 2 | vehicle_air_data | 30 | 30 | Real 按 ICP201XX 轮询；HITL 是初始假设，需按仿真输入修订 |
| 3 | vehicle_imu | 200 | 1000 | 积分/输出目标映射为初始请求率，尚未标定原生 Run 率 |
| 4 | vehicle_acceleration | 200 | 1000 | IMU_INTEG_RATE 所确定的回调批次目标 |
| 5 | vehicle_angular_velocity | 400 | 400 | IMU_GYRO_RATEMAX 名义目标 |
| 6 | sensors | 200 | 1000 | vehicle_imu → sensor_combined，单实例 EKF 配置 |
| 7 | ekf2 | 200 | 1000 | 随 IMU 消费；其 Run 不只在完整滤波更新时发生 |
| 8 | mc_pos_control | 100 | 1000 | EKF local_position 名义发布率，需足够 IMU 输入 |
| 9 | mc_hover_thrust_estimator | 100 | 1000 | 随 local_position，位置控制先发请求以更新其设定值 |
| 10 | mc_att_control | 200 | 1000 | EKF 高频姿态输出预测器路径 |
| 11 | mc_rate_control | 400 | 400 | 随角速度结果 |
| 12 | control_allocator | 400 | 400 | 随力矩/推力结果 |
| 13 | pwm_out_sim | 不启用 | 400 | HITL 执行器消息路径；Real 的硬件 PWM 未接入手动调度 |

表中是固定请求率，既不承诺每次请求都生成独立 Run，也不承诺有效输出频率。输入不足、发布限频、积分、预测节流、无新数据及队列合并均可能降低有效执行/输出次数。

## 1000 Hz tick 的具体含义

调度器使用主机单调时钟，以绝对时间 `epoch + tick × 1 ms` 等待，避免每轮相对 sleep 累积漂移。每个 tick 按 JSON tasks 数组顺序检查到期项。

对频率 f、相位 p，逻辑第 k 次释放对应 tick `p + ceil(k × 1000/f)`。频率使用有理数计算：

- 1000 Hz：每 tick 一次；200 Hz：每 5 tick 一次。
- 400 Hz：理想周期 2.5 ms，映射到 tick 0、3、5、8、10……，间隔交替 3/2 ms。
- 30 Hz：理想周期 33.333… ms，映射到 tick 0、34、67、100……。

这是固定逻辑周期到 1 ms 网格的确定性量化，不把 400 Hz 悄悄取整为 500 Hz。若必须让相邻释放严格相隔 2.5 ms，就需要更细 tick 或独立子 tick 定时，不能同时限定为仅在 1 ms tick 上释放。

Python/Linux 不保证每次准时唤醒。主机错过 tick 时跳到当前 tick，对每个目标最多发一次，并统计过期释放；不集中补发历史触发。JSON 可设置 phase_ticks，但本次全部默认 0。

该顺序只约束 UDP 发送请求，不等待上游计算完成。UDP/LAN 可能乱序，独立 WorkQueue 也可能先运行下游。固定发送顺序不等于完成顺序或严格 DAG 依赖，这是本次固定基线的边界。

## 自动运行与回复

一次命令完成整张表的接管和调度：

```sh
python3 Tools/trigger_bridge/client.py --host 192.168.1.5 schedule --profile hitl
python3 Tools/trigger_bridge/client.py --host 192.168.1.5 schedule --profile real --duration 30 --summary run.json
python3 Tools/trigger_bridge/client.py schedule --profile real --dry-run
python3 Tools/trigger_bridge/client.py schedule --profile hitl --exclude vehicle_magnetometer --rate vehicle_air_data=50
```

实际工作区 Python 使用 AGENTS.md 指定的 `/home/leeziy/Software/ProgramFiles/Conda/envs/onnx/bin/python`；上面的 python3 也适用于复制脚本到其他安装有 Python 的主机。

- 默认读取同目录 schedules.json，`--config` 可指定自定义同结构文件。tasks 列表顺序即发送顺序；每项包含 target、instance、rate_hz、phase_ticks。
- `--rate TARGET=HZ` 调整请求率，不修改 PX4 内部参数；`--exclude TARGET` 排除该配置未启用的模块。VehicleIMU 多实例可在 JSON 中逐项配置，EKF2 仅允许实例 0。
- 启动逐个 SET_MODE MANUAL，等待 OK。仅收到明确 BUSY/NOT_READY 时，在 setup-timeout 内另发新请求；超时/丢失回复不会重传原包。UNKNOWN_TARGET 等错误终止启动并尝试恢复已接管目标。
- 周期阶段不逐包等待 ACK，使用非阻塞接收按 request_id 匹配回复。丢失确认只记超时，不重发触发。ACK 错误则退出调度并恢复目标。
- 正常结束、Ctrl-C、SIGTERM 或处理到的异常会按逆序恢复尝试接管过的目标为 NATIVE。恢复失败会明确报告；没有查询接口，无法保留接管前未知的 MANUAL 配置。强制 kill 或主机崩溃不保证恢复。
- 每秒打印汇总，结束输出每个任务的发送、确认、超时、跳过释放、发送失败和未确认总数。ACCEPTED 只表示入队请求已处理。可保存为 JSON。

此处 tick 属于发送主机的 CLOCK_MONOTONIC 时间，不改 PX4 的 SITL/HITL/SIH/replay/lockstep 时间源，也没有实现主机时钟与仿真时钟同步。lockstep 暂停或变速时不能将主机 1 kHz 当作仿真 1 kHz。
