# Demo 设计方案（联调线 wip/pwm-dds）

> 状态：2026-08-25，描述 **wip/pwm-dds 分支**（DDS 联调线，基于 6f60db2 +
> 同事 processone-dds-gpio-pwm-v1 diff 合入 + 双方评审整改）。
> 双方契约细节见 [DDS-INTEGRATION.md](DDS-INTEGRATION.md)。

## 0. 两条代码线说明（避免混淆）

| 分支 | 内容 | 状态 |
| --- | --- | --- |
| `wip/pwm-dds-enhance`（本文档） | DDS 联调线 + radxa 增强迁移：双 topic 心跳、supervisor/reply 线程、PWM 100kHz/12.5-87.5%、心跳看门狗、rk_timer tick（opt-in） | 当前联调重点 |
| `wip/pwm-dds` | DDS 联调线（增强迁移前的基线） | 归档 |
| `radxa-demo`（主线） | 旧单 topic 设计 + 上午本地优化（rk_timer、PWM 100kHz、看门狗、优先级/pin 核） | 优化已逐个迁入本分支；待联调打通后与新设计融合 |

联调线中的 "线程优先级 0 + pin 核 0" 已从主线提前迁入（评审第 4 项）。

## 1. 目标

在 ROCK 5B+（rock_5b_plus/rk3588/smp）+ Zephyr 4.3.1 上打通
**Zephyr ⇄ Linux 双 topic DDS 心跳闭环**：

- 1ms 周期信号链（**临时调试值 50ms/20Hz**（Kconfig 默认，可 `hb period` 重设），见 §3）
- 两路 PWM（100kHz、12.5%/87.5% 每控制点交替）
- 三路 GPIO 链路打点（GPIO1 tick / GPIO2 控制点 / GPIO3 Linux 回复）
- DDS：micro-ROS + ZVisor 共享内存 transport；Agent 断开自动重建会话

## 2. 硬件引脚规划

| 信号链角色 | 40-pin | SoC 引脚 | GPIO/引脚 | PWM | Pinmux |
| --- | --- | --- | --- | --- | --- |
| PWM1 | Pin 31 | PWM0_M2 = GPIO1_A2 | gpio1 / 2 | pwm0 ch0 | pwm0m2_pins |
| PWM2 | Pin 27 | PWM7_IR_M3 = GPIO4_C6 | gpio4 / 22 | pwm7 ch0 | pwm7m3_pins |
| GPIO1（tick 打点） | Pin 37 | GPIO0_A0 | gpio0 / 0 | — | mux 0 |
| GPIO2（控制点打点） | Pin 28 | GPIO4_C5 | gpio4 / 21 | — | mux 0 |
| GPIO3（回复打点） | Pin 29 | GPIO1_A3 | gpio1 / 3 | — | mux 0 |

- 三路 GPIO 初始全低；每控制点翻转 → 方波（1ms 周期 = 500Hz；临时 50ms = 10Hz）。

## 3. 信号时序

```
systick ISR (1ms/临时50ms; 或 rk_timer 硬件定时器, opt-in)
        -> GPIO1 toggle, timer_tick_seq++
        |  k_sem_give（仅唤醒提示，tick_seq 是真值源）
        v
控制线程 (prio 0, pin 核0):
  GPIO2/PWM1 按绝对相位 (timer_ticks & 1) 翻转   <- 唤醒合并也不错相
  hb_send(seq++)                                 <- 单槽 pending，永不阻塞
        |                                        <- Agent 慢时 seq 合并/跳号=预期
        v
DDS supervisor 线程 (prio 5): 发布 pending -> Linux 收到 -> 回复
        |
        v
reply 线程 (prio 4): 每个回复 -> GPIO3 toggle + PWM2 翻转（计数信号量，不合并）
（回复看门狗：Linux 静默 >5ms 时本地兜底翻转 GPIO3/PWM2，stall/恢复各打一行日志）
```

- **相位绝对化**：GPIO2/PWM1 不再用局部状态翻转，而是 `timer_ticks & 1` 推导，
  负载下 sem 唤醒合并（奇数次）也不会导致与 GPIO1 失相；`missed_tick_count`
  统计合并掉的 tick。
- **心跳合并语义（预期行为）**：`hb_send` 只登记最新 seq，实际发布由 supervisor
  线程驱动；链路慢于发送节奏时中间心跳被合并、seq 跳号——这正是演示要观察的
  "流程丢失"。Linux 每收一个回一个，GPIO3 节奏由 Linux 回复节奏决定。
- **临时周期**：`DEMO_CONTROL_PERIOD_MS = 50`（Kconfig 默认）为联调调试值，可 `hb period <ms>` 运行时重设；恢复为 1 后
  心跳 1kHz、GPIO 500Hz。

## 4. 软件架构

### 线程模型（与 DDS-INTEGRATION.md §4 一致）

| 线程 | 优先级 | 职责 |
| --- | --- | --- |
| signal-chain 控制线程 | 0（最高），pin 核 0 | GPIO2/PWM1 + hb_send |
| reply 线程 | 4 | GPIO3/PWM2 翻转 |
| micro-ROS supervisor | 5 | 实体创建/销毁、spin、Agent ping、会话重建 |

### 模块划分

```
demo/src/
├── main.c           # 后端感知的启动文案
├── signal_chain.c/h # 信号链：tick ISR → 控制线程 → 回复回调；周期调优接口
├── heartbeat_dds.h  # 心跳抽象（hb_init / hb_send，非阻塞语义）
├── dds_stub.c       # 打桩后端（-DHEARTBEAT_STUB=y 时编译）
├── dds_microros.c   # P2 真 DDS：双 topic + supervisor/reply 线程
├── dds_diag.h       # 诊断快照结构 + 只读 getter 声明
├── dds_shell.c      # "dds" 诊断 shell（status/stats/shm）
└── hb_shell.c       # "hb" 业务 shell（period/freq，信号链周期调优）
```

### DDS 后端要点

- **双 topic**（`zephyr_int32_publisher` / `host_int32_publisher`，std_msgs/Int32）
- **best_effort QoS 两侧一致**（否则 QoS 不匹配收不到数据）
- 节点名 `processone_zephyr_heartbeat`；transport = ZVisor shmem
- Agent 断线：ping 探测（500ms 间隔 / 100ms 超时 / 2 次失败）→ 销毁实体 →
  周期重试重建；`hb_send` 在会话未就绪时返回 `-ENOTCONN` 并计数。

## 5. 日志

- 控制台默认静默（仅 ERR），每秒（1000 ticks，50ms 周期下即 50s）一行的
  健康日志为**严格简化版**：`hb t=%u c=%u m=%ld tx=%u rx=%u`。
- 完整字段（last_linux / gpio_errors / pwm_errors / tx_offline）及调试降频方法
  见 `src/signal_chain.c` 中该行上方注释。

## 6. Demo shell（集中式：`dds` 诊断 + `hb` 业务调优）

### 6.1 `dds` 命令集（心跳链路诊断，针对"tx slot busy"排障）

| 命令 | 用途 |
| --- | --- |
| `dds status` | 会话（UP/DOWN、重建次数）、实体 6 步位图、TX/RX 槽位状态与帧长、线程状态与栈余量、pending/received seq |
| `dds stats [reset]` | 信号链计数（tx/rx/missed/errors）、DDS 计数（rebuilds/publish_fail/ping_fail）、transport 六计数器；reset 清 transport 计数 |
| `dds shm [dump [n]]` | 槽位状态 + rsp 窗口 hexdump（协议级对帧） |

### 6.2 `hb` 命令集（信号链业务调优）

控制周期是业务参数（决定 GPIO/PWM/心跳节奏），归 `hb` 而非 `dds`：

| 命令 | 用途 |
| --- | --- |
| `hb period [ms]` | 查询/设置控制周期（1..1000ms，实时生效，反复可调）；默认值来自 Kconfig `DEMO_CONTROL_PERIOD_MS`（当前 50ms 调试值） |
| `hb freq [hz]` | 同上，以频率视角（1..1000Hz，换算成 ms 写入同一变量） |

限制：systick 定时器下最小有效周期 1ms；亚毫秒待 rk_timer 迁回后支持。

### 实现要点
- **transport 零改动**：槽位按 GPA 直读 shm 窗口，六计数器直接 extern 引用；
  "RSP 消费延迟"指标（ISR 到达→释放槽位）需改 transport，**待双方拍板后单独加**；
- shell 轮询串口后端（中断驱动 uart2 挂起，见 TODO #9）、线程优先级 5（低于控制线程 0），
  不污染控制路径；
- Kconfig `DEMO_SHELL`（默认 y，可关省 RAM）；stub 后端同样可运行；
- 排障线索：`rsp=BUSY` 且 `wr_wait` 增长 = zephyr TX 被堵；`isr` 与 `rd` 差距大 =
  中断到了但消费不及时；`rd_to` 高 = RSP 迟迟无数据。

## 7. 构建命令（备查）

单一 `prj.conf`，后端由编译开关选择（Kconfig `DEMO_HEARTBEAT_STUB`，默认 n）：

```bash
# 正式版（真 DDS / micro-ROS / ZVisor shmem，默认）
west build -b rock_5b_plus/rk3588/smp tasks/pwm-rk3588/demo -p -- \
  -DZEPHYR_SDK_INSTALL_DIR=/com/zephyrproject/sdk/v0.16.9

# 打桩版（模拟 Linux 往返）
west build -b rock_5b_plus/rk3588/smp tasks/pwm-rk3588/demo -p -- \
  -DZEPHYR_SDK_INSTALL_DIR=/com/zephyrproject/sdk/v0.16.9 \
  -DHEARTBEAT_STUB=y

# rk_timer 硬件 tick（opt-in，需 zephyr 树含 rockchip counter 驱动与 timer0 节点）
west build -b rock_5b_plus/rk3588/smp tasks/pwm-rk3588/demo -p -- \
  -DZEPHYR_SDK_INSTALL_DIR=/com/zephyrproject/sdk/v0.16.9 \
  -DCONFIG_DEMO_RK_TIMER=y
```

## 8. 已知事项与后续计划

见 [DDS-INTEGRATION.md](DDS-INTEGRATION.md) §7：
- 同事 baseline 的 self-test / `record_*` 实现待补全；
- zephyr 侧三个已知小问题（publish 失败丢 seq、last_linux 覆盖、ping 阻塞）；
- **已从主线迁入本分支**：PWM 100kHz/12.5-87.5%、心跳看门狗、rk_timer tick
  （opt-in，`DEMO_RK_TIMER`）、GPIO dt_flags 清理；优先级/pin 核此前已迁入；
  `LOG_OVERRIDE_LEVEL` 决策不迁（与联调诊断/health 日志冲突，已记录）；
- 剩余：shell 延迟干预、板内延迟自测；最终融合两条线。

## 9. 验证方法

| 观测点 | 预期（1ms 周期时） | 预期（临时 50ms 周期） |
| --- | --- | --- |
| GPIO1（Pin 37） | 500Hz 方波 | 25Hz 方波 |
| GPIO2（Pin 28） | 滞后 GPIO1 一个调度延迟 | 同左 |
| GPIO3（Pin 29） | 由 Linux 回复节奏驱动（≤500Hz，负载下变慢） | 同左 |
| PWM1/2 | 100kHz，12.5/87.5 交替，控制点对齐各自 GPIO | 同左（控制点 50ms 一次） |
| 串口 | `hb t=.. c=.. m=.. tx=.. rx=..` 一行/秒（50ms 周期下一行/50s） | — |
