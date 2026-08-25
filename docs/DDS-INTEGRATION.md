# DDS 联调契约（Zephyr ⇄ Linux，双方信息同步）

> 版本：2026-08-25，基于同事 `processone-dds-gpio-pwm-v1` diff 合入后的定稿。
> 本文档随 demo 仓库同步；任何一侧修改契约前先改本文档。

## 1. Topic 契约（双 topic，按方向分离）

| 方向 | Topic 名 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| Zephyr → Linux | `zephyr_int32_publisher` | `std_msgs/msg/Int32` | zephyr 心跳（seq 单调递增） |
| Linux → Zephyr | `host_int32_publisher` | `std_msgs/msg/Int32` | linux 心跳回复（seq 单调递增） |

- Linux 侧：订阅 `zephyr_int32_publisher`，发布 `host_int32_publisher`（镜像结构）。
- Zephyr 节点名：`processone_zephyr_heartbeat`。

## 2. QoS 契约

- 两侧 pub/sub 均使用 **best-effort** reliability（zephyr 侧
  `rclc_publisher_init_best_effort` / `rclc_subscription_init_best_effort`）。
- ⚠️ Linux 侧必须同样配 best_effort；若一侧仍为默认 reliable，QoS 不匹配，
  将表现为"链路已通但收不到任何样本"。

## 3. 心跳语义（含预期行为）

- Zephyr：每个 1ms 控制周期调用一次 `hb_send(seq++)`。**发送侧是单槽 pending**：
  `hb_send` 只登记最新 seq，实际发布由 DDS supervisor 线程驱动。
- **seq 跳号是预期现象**：Agent/链路慢于 1kHz 时，中间的心跳被合并（只发最新值）。
  这正是演示要观察的"流程丢失"。
- Linux：每收到一个 zephyr 心跳，`seq_l++` 并回复一个（节奏由收到节奏决定）。
- Zephyr 回复处理：每个收到的回复触发一次 GPIO3/PWM2 翻转（计数信号量，不合并）。

## 4. 线程模型（zephyr 侧）

| 线程 | 优先级 | 职责 |
| --- | --- | --- |
| signal-chain 控制线程 | **0（最高）**，**pin 核 0** | GPIO2/PWM1 控制点 + hb_send（永不阻塞） |
| reply 线程 | 4 | 回复回调（GPIO3/PWM2 翻转） |
| micro-ROS supervisor 线程 | 5 | 实体创建/销毁、spin、Agent ping（500ms 间隔，100ms 超时，2 次失败重建会话） |

- tick 中断在核 0 投递，控制线程同核 → 无跨核唤醒抖动。

## 5. 控制台日志

- 每秒一行的健康日志（**严格简化版**）：
  `hb t=<ticks> c=<controls> m=<missed> tx=<seq> rx=<rx_count>`
- 字段含义与完整版（last_linux/gpio_errors/pwm_errors/tx_offline）见
  `src/signal_chain.c` 中该行上方注释；调试时可调大 `HEALTH_PERIOD_TICKS`
  （如 10000）降低打印频率，避免打印扰动 1ms 控制路径。

## 6. DDS 诊断 shell（`dds` 命令集，已落地）

zephyr 侧串口提供集中式诊断命令（实现 `src/dds_shell.c`）：

| 命令 | 输出 |
| --- | --- |
| `dds status` | shm 槽位状态/帧长、信号链计数、会话 UP/DOWN、实体位图、pending/received seq、线程状态与栈余量 |
| `dds stats [reset]` | 信号链/DDS/transport 全计数；reset 清 transport 计数 |
| `dds shm [dump [n]]` | 槽位状态 + rsp 窗口 hexdump |

业务调优命令集 `hb`（控制周期是信号链业务参数，不属 DDS 诊断）：

| 命令 | 输出 |
| --- | --- |
| `hb period [ms]` | 查询/设置控制周期（1..1000ms，实时生效）；默认 Kconfig `DEMO_CONTROL_PERIOD_MS`（当前 50ms 调试值） |
| `hb freq [hz]` | 同上，以频率视角（1..1000Hz） |

- transport 模块当前零改动（槽位按 GPA 直读、计数器 extern 引用）；
- **待定**："RSP 消费延迟"指标（ISR 到达→释放槽位的时间差）需在 transport 埋点，
  拍板后单独加，用于直接佐证 "tx slot busy = zephyr 消费不及时"。
- 排障线索：`rsp=BUSY` + `wr_wait` 增长 = 双槽互相卡死；`isr >> rd` = 消费不及时；
  `rd_to` 高 = RSP 无数据。

## 7. 已知事项 / 后续迁移清单（双方知情）

- [ ] 同事 baseline 中的"GPIO/PWM API self-test"实现与 `record_*` 函数体不在
      diff 中，合入版为占位 stub（`[COLLEAGUE BASELINE]` 标注）——需同事补全。
- [ ] zephyr 侧已知小问题：publish 失败时该 seq 已消费（丢一个心跳）；
      多回复排队时 `last_linux_seq` 被最新值覆盖（翻转次数正确、数值错位）；
      `rmw_uros_ping_agent` 阻塞 100ms，Agent 假死期间订阅接收周期性中断。
- [ ] 上午已完成的本地优化待后续迁移（暂不并入本分支）：
      rk_timer 硬件定时器（替代 systick）、PWM 100kHz + 12.5/87.5%、
      心跳超时看门狗、shell 延迟干预命令、板内延迟自测。
- [ ] 联调成功后合并策略：同事的修复类改动（会话重建、非阻塞 hb_send、
      计数信号量、绝对相位、missed 统计）吸收进主线 radxa-demo。
