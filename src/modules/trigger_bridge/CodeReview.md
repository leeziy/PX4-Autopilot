# Manual Trigger 与 UDP Bridge 静态分析

AGENTS.md：2026-10-27，仅执行源代码分析、Python 语法解析、格式与交叉引用检查。没有编译、自动化行为测试、仿真、板端验证或开销测量；下列结论不替代这些验证。

## 调度路径

1. 13 类目标在派生构造函数结束处显式注册，初始模式 NATIVE；EKF2 只注册单实例路径，VehicleIMU 使用真实实例号。
2. ScheduleNow、ScheduleDelayed、ScheduleOnInterval、ScheduleAt 在同一控制锁下检查自动调度许可；uORB 的现有回调最终调用 ScheduleNow，因此不需修改各个回调注册分支。
3. MANUAL 中仍记录定时调度意图，但不注册 HRT 定时；返回 NATIVE 恢复该意图并启动一次 Run。特别覆盖 MixingOutput 的周期 fallback，避免只启动一次后不再周期运行。
4. 模式实际改变时，running=true 返回 BUSY，不等待执行结束。队列出队到标记 running 在同一控制临界区；成功进入 MANUAL 时已经排除在途 Run、清除旧排队项、取消定时，之后的回调也受门控。
5. TRIGGER 在同一临界区检查注册状态、初始化就绪、停止状态和模式，再调用原 WorkQueue::Add。IntrusiveQueue 的原去重逻辑不变；没有额外排队层或请求 ID 缓存。
6. 未保留 producer 实例快照；输入读取仍发生在原 Run 内，消息队列、最新值和多源消费语义继续由原模块决定。

## 生命周期与锁顺序

- 固定 64 项注册表，不在 UDP 触发路径动态分配；重复 ID 或表满时记录错误并保留该对象原生行为。
- 控制锁为 POSIX 递归 pthread mutex，仅在 CONFIG_MODULES_TRIGGER_BRIDGE 启用时编译。所有新增嵌套路径先取控制锁，再取 WorkQueue 锁；Run 本体不持有控制锁，允许不同队列并行计算。
- Queue 在出队时取得独立 slot/generation ticket。Run 可能自行销毁对象，结束时只使用 ticket 更新表项，不再访问对象指针；退役表项在在途 Run 结束前不复用。
- 13 个派生析构入口先关闭外部访问；五个传感器 Stop 同样关闭访问。外部销毁等待在途 Run 返回，等待时释放控制锁；同一 Run 自行销毁时不等待自己。
- 七个 ModuleBase 模块和 EKF2 的 request_stop 显式调用 ScheduleForStop，停止执行越过 MANUAL 门控。
- 普通 hrt_cancel 只移除定时器，不能保证已取出的回调完成。新增可选 POSIX hrt_cancel_sync，等待回调及 HRT 后处理退出；目标注销时在不持控制锁的情况下等待，避免定时回调等待控制锁造成死锁。
- 已注册目标在执行中拒绝 ChangeWorkQueue，避免同一对象同时在旧、新队列运行。当前 PWMSim 调用 updateSubscriptions(false)，不请求自动切换；其他非目标保持原接口行为。

## UDP 与客户端

- 请求固定 24 字节；接收缓冲为 25 字节，避免超长包被截成合法 24 字节请求。
- 按字节读取，无未对齐整数解引用；网络字段偏移与 Python 的 `!IBBBBQIHH` 一致。
- 未知版本/保留位、错误命令/模式不调用目标接口；回复包不再回复，防止两个 Bridge 相互响应。
- request_id 原样回显，不作为去重键；一个合法 TRIGGER 数据报对应一次入队调用。
- socket 非阻塞，poll 使用有限超时；停止线程不依赖外部再发送数据。接收循环不运行目标计算。
- Python 使用 connected UDP，校验回复来源、长度、opcode、请求号和目标，使用绝对超时避免其他回复无限延长等待；超时不重发。

## 构建与边界

- 新模块 Kconfig 被 src/modules/Kconfig 的通配 rsource 发现，CMake 显式依赖 px4_work_queue，公共源文件加入该库。
- Firefly 与 OrangePi RK3588S 默认 board 配置仅新增 Bridge 功能选项。已撤回此前对 EKF2 多实例编译选项和三个启动脚本的修改；单实例 EKF2 按实验启动参数配置。
- 模块仅依赖 PLATFORM_POSIX，不以 BOARD_LINUX_TARGET 作为功能开关。ManualTrigger.cpp 按功能选项加入公共库；关闭功能时保留原 WorkQueue::Run 及原调度入口，所有目标注册/注销/退出钩子均编译移除，没有 NuttX 后端。
- 不修改 land_detector、flight_mode_manager，保持此前回退结果；其 Marker 不恢复。
- 功能启用后的 NATIVE 模式也增加了控制锁与调度 bookkeeping 开销，尚未量化。全局控制锁可能造成短时调度竞争；不能宣称运行时性能或实时保证。
- ready 表示至少执行过一次，不表示状态估计有效。无新输入时，Run 可能不进行主要计算或不发布输出。
- Python 的旧单目标诊断入口逐条等待回复；新增 schedule 入口使用 1000 Hz tick 和异步 TRIGGER 确认。固定逻辑周期量化到 1 ms 网格，主机过载时跳过过期释放，不补发历史突发；仍不提供硬实时保证。
- UDP 丢包、乱序及重复按简单协议语义处理，不保证恰好一次；跨实例重建不识别过期网络命令。
- SITL/HITL/SIH/replay/lockstep 不改平台选择、仿真类型、启动脚本或时间推进接口。保留这些路径不代表手动释放与 lockstep 的运行组合已经验证。

后续仍需按原计划第 6 项进行构建和运行验证，特别覆盖模式往返切换、并发停止、初始化、MixingOutput 周期恢复，以及锁与 Bridge 的实际开销。

## 本轮实际完成的机械检查

- Python AST 语法解析通过；网络字段格式 `!IBBBBQIHH` 的长度为 24 字节。
- C++ 枚举、Python 目标表及 13 个目标类的注册一一对应，目标编号唯一。
- 13 个目标修改前后的 SYS_kill Marker 序列一致；两个已排除模块仍逐字节匹配 ROC-RK3588S-PC。
- 两个 RK3588S 默认 board 配置、公共库源文件和 Bridge CMake 依赖引用检查通过；原 EKF2 构建选项和启动脚本均恢复。
- 新文档本地链接和代码块分隔检查通过；修改/新增 C/C++ 按仓库 AStyle 配置格式化，git diff --check 通过。
- 在源码文本层仅展开新增功能开关，关闭功能后的 28 个已有 C/C++ 文件与修改前版本的代码 token 序列一致；保留其他 PX4 预处理条件。本检查不是编译或二进制一致性验证。
- 三个此前修改的 POSIX 启动脚本已逐字节恢复；board 差异仅新增 Bridge 功能选项，不修改 EKF2、POSIX、lockstep 或仿真模式配置。

这些检查只覆盖语法文本、配置引用与代码路径，不表示 C++ 已编译或并发行为已通过测试。

## 1000 Hz 外部调度器补充检查

参考分支 commit、频率参数、回调路径和 driver 周期的依据见 [NativeRates.md](../../../Tools/trigger_bridge/NativeRates.md)。本轮没有连接 UDP/PX4，也没有重新编译或进行板端测频。

- Python AST 和 Real/HITL 两种 dry-run 配置加载通过。
- 离线遍历 1000 个逻辑 tick，Real 的 12 项、HITL 的 13 项分别生成配置要求的每秒请求数量。
- 检查 400 Hz 的 3/2 ms 量化、30 Hz 的 34/33/33 ms 量化、相位偏移和错过 tick 时的跳过计数。
- 检查同 tick 不重复释放，过期作业不集中补发，频率覆盖/排除配置以及非法频率拒绝。
- 周期发送不逐包等待，回复按目标、实例和请求号匹配，超时不重发。回复预算和未确认请求数有上限。
- 模式接管失败也进入恢复路径；收到明确 BUSY/NOT_READY 才重新发起 SET_MODE，丢失回复不重传。退出后的实际模式仍取决于恢复命令是否交付成功。

配置频率是源码推导的初始请求率；VehicleIMU 等任务的 Run 率不能由发布参数唯一确定，HITL 的磁场/气压输入率也未在该脚本中固定，不能把这些配置写成原生实测数据。
