# Demo 设计方案（联调线 wip/pwm-dds）

> 状态：2026-08-25，描述 **wip/pwm-dds 分支**（DDS 联调线，基于 6f60db2 +
> 同事 processone-dds-gpio-pwm-v1 diff 合入 + 双方评审整改）。
> 双方契约细节见 [DDS-INTEGRATION.md](DDS-INTEGRATION.md)。

## 0. 两条代码线说明（避免混淆）

| 分支 | 内容 | 状态 |
| --- | --- | --- |
| `wip/pwm-dds`（本文档） | DDS 联调线：**双 topic** 心跳、supervisor/reply 线程、systick + 10kHz PWM | 当前联调重点 |
| `radxa-demo`（主线） | 旧单 topic 设计 + 上午本地优化（rk_timer 硬件定时器、PWM 100kHz/12.5-87.5%、心跳看门狗、优先级/pin 核整改） | 联调打通后把优化迁入，再与联调线融合 |

联调线中的 "线程优先级 0 + pin 核 0" 已从主线提前迁入（评审第 4 项）。

## 1. 目标

在 ROCK 5B+（rock_5b_plus/rk3588/smp）+ Zephyr 4.3.1 上打通
**Zephyr ⇄ Linux 双 topic DDS 心跳闭环**：

- 1ms 周期信号链（**临时调试值 20ms/50Hz**，见 §3）
- 两路 PWM（10kHz、25%/75% 每控制点交替）
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

- 三路 GPIO 初始全低；每控制点翻转 → 方波（1ms 周期 = 500Hz；临时 20ms = 25Hz）。

## 3. 信号时序

```
systick ISR (1ms/临时20ms)  -> GPIO1 toggle, timer_tick_seq++
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
```

- **相位绝对化**：GPIO2/PWM1 不再用局部状态翻转，而是 `timer_ticks & 1` 推导，
  负载下 sem 唤醒合并（奇数次）也不会导致与 GPIO1 失相；`missed_tick_count`
  统计合并掉的 tick。
- **心跳合并语义（预期行为）**：`hb_send` 只登记最新 seq，实际发布由 supervisor
  线程驱动；链路慢于发送节奏时中间心跳被合并、seq 跳号——这正是演示要观察的
  "流程丢失"。Linux 每收一个回一个，GPIO3 节奏由 Linux 回复节奏决定。
- **临时周期**：`CONTROL_PERIOD_MS = 20`（50Hz）为联调调试值；恢复为 1 后
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
├── signal_chain.c   # 信号链：tick ISR → 控制线程 → 回复回调
├── heartbeat_dds.h  # 心跳抽象（hb_init / hb_send，非阻塞语义）
├── dds_stub.c       # P1 打桩（默认后端）
└── dds_microros.c   # P2 真 DDS：双 topic + supervisor/reply 线程
```

### DDS 后端要点

- **双 topic**（`zephyr_int32_publisher` / `host_int32_publisher`，std_msgs/Int32）
- **best_effort QoS 两侧一致**（否则 QoS 不匹配收不到数据）
- 节点名 `processone_zephyr_heartbeat`；transport = ZVisor shmem
- Agent 断线：ping 探测（500ms 间隔 / 100ms 超时 / 2 次失败）→ 销毁实体 →
  周期重试重建；`hb_send` 在会话未就绪时返回 `-ENOTCONN` 并计数。

## 5. 日志

- 控制台默认静默（仅 ERR），每秒（1000 ticks，20ms 周期下即 20s）一行的
  健康日志为**严格简化版**：`hb t=%u c=%u m=%ld tx=%u rx=%u`。
- 完整字段（last_linux / gpio_errors / pwm_errors / tx_offline）及调试降频方法
  见 `src/signal_chain.c` 中该行上方注释。

## 6. 构建命令（备查）

```bash
# P1（打桩）
west build -b rock_5b_plus/rk3588/smp tasks/pwm-rk3588/demo -p -- \
  -DZEPHYR_SDK_INSTALL_DIR=/com/zephyrproject/sdk/v0.16.9

# P2（真 DDS / ZVisor shmem）
west build -b rock_5b_plus/rk3588/smp tasks/pwm-rk3588/demo -p -- \
  -DZEPHYR_SDK_INSTALL_DIR=/com/zephyrproject/sdk/v0.16.9 \
  -DEXTRA_CONF_FILE=prj_ros.conf
```

## 7. 已知事项与后续计划

见 [DDS-INTEGRATION.md](DDS-INTEGRATION.md) §6：
- 同事 baseline 的 self-test / `record_*` 实现待补全；
- zephyr 侧三个已知小问题（publish 失败丢 seq、last_linux 覆盖、ping 阻塞）；
- 联调打通后从主线迁入：rk_timer、PWM 100kHz、心跳看门狗、shell 延迟干预、
  板内延迟自测，最终融合两条线。

## 8. 验证方法

| 观测点 | 预期（1ms 周期时） | 预期（临时 20ms 周期） |
| --- | --- | --- |
| GPIO1（Pin 37） | 500Hz 方波 | 25Hz 方波 |
| GPIO2（Pin 28） | 滞后 GPIO1 一个调度延迟 | 同左 |
| GPIO3（Pin 29） | 由 Linux 回复节奏驱动（≤500Hz，负载下变慢） | 同左 |
| PWM1/2 | 10kHz，25/75 交替，控制点对齐各自 GPIO | 同左（控制点 20ms 一次） |
| 串口 | `hb t=.. c=.. m=.. tx=.. rx=..` 一行/秒（20ms 周期下一行/20s） | — |
