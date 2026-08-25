# 最终 Demo 设计方案（PWM + DDS 心跳）

> 状态：2026-08-24 晚，已按当前实现与讨论结论更新（rk_timer 硬件定时器、smp、
> 线程优先级/亲和性整改、500Hz GPIO 表述修正）。
> 背景：ROCK 5B+（rock_5b_plus/rk3588/**smp** 变体）+ Zephyr 4.3.1
> （zephyr 树分支 `wip/hongquan.li/ProcessONE-RTOS-Demo`，demo 仓库
> `tasks/pwm-rk3588/demo` radxa-demo 分支）+ micro-ROS jazzy。

## 1. 目标

在 ROCK 5B+ 上实现"硬件定时中断 → GPIO 打点 → PWM 控制 → DDS 双向心跳"的闭环 demo：

- 1ms 周期信号链，由 RK3588 总线定时器（rk_timer，硬件自动重载）驱动
- 两路 PWM 输出（100kHz、占空比 12.5%/87.5% 每 1ms 交替）
- 三路 GPIO 作为链路时序观测点
- Zephyr ⇄ Linux 双向心跳：P1 打桩（默认）、P2 真 DDS（micro-ROS + ZVisor
  共享内存 transport，已编码并构建通过，待联调）

演示叙事：**RTOS 侧（GPIO1/2、PWM1）延迟低且稳定；Linux 侧（GPIO3、PWM2）
延迟大、抖动随负载恶化。** 讲解材料见
[讲解稿-路人30秒.md](讲解稿-路人30秒.md) / [讲解稿-技术人员2分钟.md](讲解稿-技术人员2分钟.md)。

## 2. 硬件引脚规划（已确认）

| 信号链角色 | 40-pin | SoC 引脚 | GPIO 控制器/引脚 | PWM 通道 | Pinmux | 控制器 MMIO |
| --- | --- | --- | --- | --- | --- | --- |
| PWM1 | Pin 31 | PWM0_M2 = GPIO1_A2 | gpio1 / 2 | `pwm0` ch0 | `RK_PINMUX(1,2,11)` = pwm0m2_pins | 0xfd8b0000（PMU1） |
| PWM2 | Pin 27 | PWM7_IR_M3 = GPIO4_C6 | gpio4 / 22 | `pwm7` ch0 | `RK_PINMUX(4,22,11)` = pwm7m3_pins | 0xfebd0030 |
| GPIO1（1ms ISR 打点） | Pin 37 | GPIO0_A0 | gpio0 / 0 | — | mux 0 | 0xfd8a0000（PMU 域） |
| GPIO2（处理线程打点） | Pin 28 | GPIO4_C5 | gpio4 / 21 | — | mux 0 | 0xfec50000（BUS 域） |
| GPIO3（回复回调打点） | Pin 29 | GPIO1_A3 | gpio1 / 3 | — | mux 0 | 0xfec20000（BUS 域） |

- GPIO 引脚默认即 GPIO 功能（mux 0），无需 pinctrl；两路 PWM 的 pinctrl 在
  demo overlay（`boards/rock_5b_plus_rk3588_smp.overlay`）中使能。
- 三路 GPIO 初始态：`GPIO_OUTPUT_INACTIVE`（全低）。
- ⚠️ 频率表述：每 1ms 翻转一次电平 → 半周期 1ms → **500Hz 方波**（"1kHz 方波"
  是错误说法；"每秒翻转 1000 次"才是对的）。

## 3. 信号时序设计（1ms rk_timer 基准）

```
tick N (硬件定时器, 1ms)      tick N+1
  |                             |
  ├─ ISR (counter top 回调):      ├─ ...
  |   GPIO1 翻转 (500Hz 方波)
  |   k_sem_give → 唤醒处理线程
  |
  ├─ 处理线程 (优先级 0, pin 核0):
  |   GPIO2 翻转
  |   PWM1 占空比翻转 (12.5↔87.5)
  |   hb_send(seq_z++)
  |
  ├─ (模拟 Linux 往返延迟: 200µs + 0~200µs LCG 抖动, P1)
  |
  ├─ 回复回调 (workqueue; 看门狗兜底同路径):
  |   GPIO3 翻转
  |   PWM2 占空比翻转 (12.5↔87.5)
```

- **tick 来源**：RK3588 总线定时器 timer0（`drivers/counter/counter_rockchip_timer.c`），
  FREE_RUNNING 模式硬件自动重载，top 回调在 ISR 上下文执行，**无软件重装抖动**。
  CRU 时钟自举（`CONFIG_ROCKCHIP_RK_TIMER_CRU_INIT=y`）：强制 24MHz 源、解 gate，
  不依赖 bootloader 时钟状态。
- **PWM 时序**：周期 10µs（100kHz）；占空比每 1ms 控制点交替（12.5↔87.5），完整翻转
  周期 2ms。控制点生效机制：`pwm_set_dt` 用 LOCK 位，在下一 PWM 周期边界生效
  （量化延迟 0~10µs，无毛刺）。tick（1ms）是 PWM 周期（10µs）的整数倍。
- **心跳语义**：Zephyr 处理线程发 `seq_z++`；Linux 收到后 `seq_l++` 回复；
  Zephyr 收到回复触发 GPIO3 + PWM2 翻转。

## 4. 软件架构

### 线程/上下文划分（当前实现）

| 上下文 | 职责 | 关键配置 |
| --- | --- | --- |
| rk_timer ISR（1ms） | GPIO1 翻转 + `k_sem_give` | counter top 回调；SPI 289→GIC INTID 321，**路由在核 0**（GICv3 按 `irq_enable()` 所在 PE 路由，POST_KERNEL 阶段=启动核） |
| 处理线程 | 等 sem → GPIO2 翻转 → PWM1 翻转 → hb_send | **优先级 0（最高），pin 核 0**（`SCHED_CPU_MASK_PIN_ONLY`），栈 1024B |
| 回复回调（system workqueue） | GPIO3 翻转 → PWM2 翻转 | P1 由桩定时器触发；P2 由订阅回调 `k_work_submit` 触发；**心跳看门狗（k_timer 5ms，ISR 上下文）超时时本地兜底同一路径** |
| P2 spin 线程（micro-ROS executor） | 订阅接收 → 转交 workqueue | 优先级 `NUM_PREEMPT_PRIORITIES-2`，栈 4096B |

**实时性整改（2026-08-24，已提交）**：
- 处理线程从最低优先级提到 **0**（任何就绪线程都会推迟 GPIO2/PWM1 控制点）；
- `CONFIG_LOG_OVERRIDE_LEVEL=1`：全部模块编译期压到 ERR（UART 日志是隐藏延迟源）；
- 处理线程 pin 核 0：与 ISR 同核，消除跨核唤醒延迟与 cache-cold 抖动；
- 说明：GIC SPI 的核间亲和性无法在 arm64 zephyr 上设置（无 `irq_set_affinity`
  实现），但 GICv3 驱动已把 SPI 固定在启动核，等效达成"ISR+消费线程同核"。

### 模块划分

```
demo/src/
├── main.c          # 初始化 + 启动
├── signal_chain.c  # 信号链：timer ISR → 线程 → GPIO/PWM 翻转
├── heartbeat_dds.h # DDS 心跳抽象层（P1/P2 共用接口）
├── dds_stub.c      # P1 打桩（模拟 Linux 往返；默认后端）
└── dds_microros.c  # P2 真 micro-ROS（ZVisor shmem transport）
```

### DDS 心跳抽象接口

```c
typedef void (*hb_recv_cb_t)(uint32_t linux_seq);
int hb_init(hb_recv_cb_t cb);
int hb_send(uint32_t seq);
```

单 topic `heartbeat`（std_msgs/Int32）双向 pub/sub；DDS 语义保证发布者不收自己的
样本。P2 transport：ZVisor 共享内存 VM 通道（`MICROROS_TRANSPORT_ZVISOR_SHMEM`，
TX 走 HVC 0x86000000 token16 → Linux IRQ42，RX 走 virtual IRQ41）。

### 往返延迟干预机制（方案已定，待落地，见 TODO 12）

P1 桩与 P2 真 DDS 共用一套"Linux 侧往返延迟"目标参数 `{base_us, jitter_us}`
（jitter 取 base 的 10%）：

- P1：`hb_send` 内 `delay = base + LCG(jitter)`（当前固定 200+0~200µs）；
- P2：**扣减真实往返延迟**——zephyr 侧记录发送→收到回复时间，EMA 滑动估计真实
  链路延迟 R，插入 `delay = max(0, base + jitter − R)`，使控制点生效时刻的统计
  行为与 P1 一致（用户确认）；真实负载超过目标时插 0 尽力而为；
- 统一通过 shell 命令现场调节（`demo delay <base_us> [jitter_us]`），
  配合 dashboard 真实负载演示"延迟/抖动恶化"叙事。

## 5. 实施计划

### Phase 1：打桩 demo —— ✅ 已编码 + 构建通过，待真机验证
- 信号链 + dds_stub（200µs + 0~200µs LCG 抖动）+ rk_timer tick 已完成；
- 待办：上板验证（逻辑分析仪观测 3 GPIO 500Hz 方波 + 2 PWM 10kHz 波形）。

### Phase 2：真 DDS —— ✅ 已编码 + 构建通过，待联调
- `dds_microros.c`（ZVisor shmem transport）已通过构建；
- 待办：ZVisor 平台侧 micro-ROS Agent + Linux 心跳节点联调。

### 后续迭代（详见 TODO.md）
- ✅ PWM 频率提升（10kHz → 100kHz）+ 占空比 0.125/0.875（提交 eeb523a）；
- ✅ 心跳超时看门狗（纯软件，提交 d1ab26a）；
- ⬜ shell 往返延迟干预命令（方案已定：P2 扣减真实延迟 + 10% 抖动）；
- ⬜ 板内延迟自测（timer1 作时间戳源，min/avg/max/σ 统计）。

## 6. 验证方法

| 观测点 | 预期 |
| --- | --- |
| GPIO1（Pin 37） | **500Hz** 方波（1ms 间隔翻转，对齐 tick） |
| GPIO2（Pin 28） | 500Hz 方波，滞后 GPIO1 一个处理线程调度延迟（µs 级） |
| GPIO3（Pin 29） | 500Hz 方波，滞后 GPIO2 约 200~400µs（桩延迟） |
| PWM1（Pin 31） | 100kHz，占空比 12.5/87.5 交替（2ms 周期），控制点对齐 GPIO2 |
| PWM2（Pin 27） | 100kHz，占空比 12.5/87.5 交替，控制点对齐 GPIO3 |
| 串口日志 | 静默（ERR 级）——现场反馈走 shell（待加） |

## 7. 已确认决策（2026-08-24 讨论拍板）

1. GPIO 频率表述统一为 **500Hz 方波**（"1kHz"仅指每秒翻转 1000 次）。
2. PWM 当前 10kHz/25-75 已真机确认正确（同时实锤 PWM 时钟为 24MHz）；
   **已提升至 100kHz、占空比 0.125/0.875**（提交 eeb523a，待真机验证）。
3. 处理线程最高优先级（0）+ 全局日志压 ERR（已落地）。
4. 往返延迟干预做成 P1/P2 一套机制 + shell 现场调节（方案已定，待落地）。
5. 板内延迟自测（timer1 时间戳）——纳入 TODO，后续再做。
6. PWM 生效延迟的认知统一：**GPIO2 打点 → PWM 输出生效** 视为"PWM 控制延迟"，
   由 LOCK 机制决定（≤1 个 PWM 周期）；提高 PWM 频率即可更精确地观测该延迟。
   硬件不支持跨控制器同源同步触发，软件侧不做强制对齐（残余 µs 级相位差可接受）。
7. smp 亲和性：ISR 天然在核 0（GICv3 SPI 路由），处理线程 pin 核 0（已落地）。
8. 心跳超时看门狗：纯软件实现（不依赖 zephyr WDT 子系统），**已落地**（提交 d1ab26a）。
9. 两路 PWM "精确同相"不可行也无必要（各通道独立计数器、无同步机制），
   改为通过"打点→生效"延迟测量体现控制精度。
10. 演示边界：P1 讲机制与 RTOS 侧稳定性；"Linux 负载恶化"叙事需 P2 + dashboard。

## 8. 构建命令（备查）

```bash
# 环境
bash -c '. /com/zephyrproject/venv/bin/activate && ...'

# P1（默认打桩后端）
west build -b rock_5b_plus/rk3588/smp tasks/pwm-rk3588/demo -p -- \
  -DZEPHYR_SDK_INSTALL_DIR=/com/zephyrproject/sdk/v0.16.9

# P2（micro-ROS / ZVisor shmem 后端）
west build -b rock_5b_plus/rk3588/smp tasks/pwm-rk3588/demo -p -- \
  -DZEPHYR_SDK_INSTALL_DIR=/com/zephyrproject/sdk/v0.16.9 \
  -DEXTRA_CONF_FILE=prj_ros.conf
```

## 9. 参考材料

- 讲解稿：[讲解稿-路人30秒.md](讲解稿-路人30秒.md)、[讲解稿-技术人员2分钟.md](讲解稿-技术人员2分钟.md)
- 进度：[PROGRESS.md](PROGRESS.md)；待办：[TODO.md](TODO.md)
- 驱动：`drivers/counter/counter_rockchip_timer.c`（zephyr 树，提交 `99d1d9e6d90`）
- demo 提交：`c290b72`（rk_timer 接入）、`87c8e84`（优先级+日志）、`f22abb6`（pin 核）
