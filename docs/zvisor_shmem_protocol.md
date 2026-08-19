# ZVisor 共享内存传输协议（zephyr,zvisor-shmem）

本文档是 micro-ROS（Micro XRCE-DDS Client ↔ Agent）在 ZVisor hypervisor 双 VM
形态下、基于共享内存的传输协议规范。只覆盖 `zephyr,zvisor-shmem` 方案
（单槽帧 + HVC 门铃），不含 OpenAMP/RPMsg 方案。

两侧实现：

- Zephyr 端：`modules/libmicroros/microros_transports/zvisor_shmem/microros_transports.c`
- Linux Agent 端：Micro-XRCE-DDS-Agent fork 的
  `src/cpp/transport/zvisor/zvisor_slot.{c,h}` + `transport/shm/shm_backend.{c,h}`

## 1. 分层模型

```
ROS 2 / DDS 世界
      │  （Agent 侧标准 DDS-XRCE 桥接）
Micro XRCE-DDS 会话层（session/stream/reliable 确认）
      │  消息字节流
StreamFramingProtocol 组帧（stock 实现，两侧一致；
      │  client 侧 MICRO_ROS_FRAMING_REQUIRED=true）
      │  一组完整的帧字节（≤ 2048）
───────── 本协议 ─────────
单槽帧协议：[u32 len][u32 state][payload]
      │
共享内存窗口 + 门铃通知（HVC hypercall / 虚拟 IRQ）
```

组帧层读写是小块字节流（先读头部再读 payload），槽协议层是一整帧一整帧地收发，
因此两侧 read 都内置 stash/offset 保存未消费尾部，对外表现为字节流。

## 2. 内存布局

| 区域 | GPA/PA | 大小 | 方向 |
|---|---|---|---|
| request 窗口 | 0x30000000 | 8 MiB | Zephyr 写 → Linux 读 |
| response 窗口 | 0x30800000 | 8 MiB | Linux 写 → Zephyr 读 |

- 两个窗口是同一个 16 MiB ZVisor 共享内存区的两个等长半区，由设备树单节点
  描述（`zv_shm`，`reg = <0x30000000 DT_SIZE_M(16)>`），代码按 reg 大小对半切分。
- 真实硬件上 Linux VM 与 Zephyr VM 看到**同一物理地址**，Linux 侧直接
  `/dev/mem` mmap 即可。
- 协议实际只使用每个半区的前 2056 字节（8 字节帧头 + 2048 payload），
  两侧也只映射窗口首 4 KiB；8 MiB 是 ZVisor 分配粒度，不是协议需求。
- 窗口按 cache-none / device 内存访问：
  - 不能对它用 `memcpy`——device 内存上非 16 字节对齐的 ldp/stp 会触发
    alignment fault（payload 偏移 8 必然踩中）。Zephyr 侧用 4 字节粒度的
    `window_read/window_write`（窗口侧地址始终 4 字节对齐）。
  - 对端是另一个 VM，无硬件 cache 一致性问题，不需要也不允许 cache 维护操作。

## 3. 槽帧格式

每个方向一个固定消息槽，布局（偏移相对各自半区基址）：

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | length | u32 | payload 字节数，合法范围 (0, 2048] |
| 4 | state | u32 | 槽位归属：0=FREE，1=BUSY |
| 8 | payload | u8[] | 一整帧 XRCE 组帧数据，上限 2048 字节 |

状态语义：

- `SLOT_FREE`（0）：槽已被消费，写方可以写下一帧。**只有读者能置 FREE**。
- `SLOT_BUSY`（1）：帧待消费，由写方置入；读者看到 BUSY 才拷贝。

任意时刻每个方向只允许一条消息在途；无 ring buffer、无多生产者、无流控计数。

## 4. 传输时序

### 4.1 TX（写方）

1. 等槽位变 `FREE`；
2. 写 payload → 写 length → 写 `BUSY`；
3. 内存屏障（Zephyr 侧 `dsb sy`，agent 侧 `__sync_synchronize()`），
   保证帧对另一个 VM 可见后再通知；
4. 敲门铃（见第 5 节）。

等槽位的超时策略两侧**不对称**：

- Zephyr 端无限等（对端 agent 可能晚启动，与串口传输"阻塞等对端"语义一致）；
- Agent 端等 1 s（`ZVISOR_TX_SLOT_WAIT_TIMEOUT_MS`），超时丢帧并触发
  fini/init 自愈重连。

门铃失败时写方把槽置回 `FREE`，让下一次 write 可以重试（Zephyr 侧行为）。

### 4.2 RX（读方）

1. 阻塞等门铃（带超时；超时不是错误，视为兜底唤醒）；
2. 检查 `state == BUSY`，否则视为虚假/丢失的门铃直接返回 0；
3. 校验 `0 < length ≤ 2048`，非法则丢帧（打日志/计数）；
4. 把整帧拷进本地 stash；
5. **拷完后**才把槽置回 `FREE`；
6. 组帧层按小块从 stash 消费，消费完前不取新帧。

关键点：槽位只由读者线程归还，ISR/门铃回调里**绝不拷数据、绝不归还槽位**。
否则对端看到 FREE 立即写下一帧，会覆盖尚未被组帧层消费完的 stash
（压测实测过这个竞态：反向 500 条丢 487 条）。

## 5. 门铃与中断

门铃只负责通知，payload 永远在共享内存里。使用 RK3588 物理 mailbox 之外的
ZVisor 虚拟化通道：

| 方向 | 写方动作 | 收方效果 |
|---|---|---|
| Zephyr → Linux | HVC `0x86000000`，token 16 | hypervisor 向 Linux VM 注入 IRQ 42 |
| Linux → Zephyr | HVC `0x86000000`，token 32 | hypervisor 向 Zephyr VM 注入 IRQ 41 |

- SMCCC 门铃函数号 `0x86000000`，返回值 0 为成功。
- Zephyr 端：直接 `arm_smccc_hvc()`；收中断 IRQ 41（设备树
  `zephyr,doorbell-irq`），ISR 只做 `k_sem_give`，边沿触发（`IRQ_TYPE_EDGE`）。
- Linux 端：不直接碰 HVC/GIC，走 `zvisor_shmem` misc 内核驱动
  （`zvisor_shmem.c`，compatible `zephyr,zvisor-shmem`）：
  - kick = `ioctl(ZVISOR_SHMEM_IOCTL_NOTIFY)`，驱动代发 HVC（token 取自
    Linux 侧 DT 节点的 `zephyr,hypercall-token = <32>`）；
  - bell = `poll()` 等驱动 IRQ handler 唤醒 + `ioctl(CLEAR_IRQ_COUNT)` 清计数。
- QEMU 测试模式（无 hypervisor）：HVC 被 QEMU 拦截转成 unix socket 字节，
  两个方向各一个 chardev socket（`--kick-sock`/`--bell-sock`），线协议不变。

## 6. 初始化与重启

两侧 open/init 时都把**两个**槽的 length/state 清零置 `FREE` + 屏障，
丢弃对端或自己上一次运行的残留帧。因此任一侧重启后链路自动回到初始态；
agent 侧另有 `handle_error` → `fini() && init()` 的整链重连。

## 7. 约束汇总

| 项目 | 值 |
|---|---|
| payload 上限 | 2048 字节（XRCE transport MTU 不得超过槽容量） |
| 在途消息 | 每方向 1 条 |
| 字节序 | length/state 为原生 u32（两侧均 aarch64 LE） |
| state 取值 | 0=FREE / 1=BUSY，其余非法 |
| 窗口访问 | cache-none，4 字节粒度，禁 memcpy |
| 超时 | Zephyr TX 无限等；agent TX 1 s；RX 超时=兜底唤醒非错误 |

## 8. 设备树契约（`zephyr,zvisor-shmem`）

binding：Zephyr 树 `dts/bindings/zvisor/zephyr,zvisor-shmem.yaml`。

| 属性 | 必需 | 说明 |
|---|---|---|
| `reg` | 是 | 共享窗口 GPA 与大小（两半等长，前半 TX、后半 RX） |
| `zephyr,endpoint-name` | 是 | 端点名（Linux 驱动用它命名 misc 设备） |
| `zephyr,permissions` | 是 | Guest 读/写/DMA 权限位掩码 |
| `zephyr,hypercall-token` | 可选 | 门铃 hypercall token；与 `doorbell-irq` 同挂表示双向通道 |
| `zephyr,doorbell-irq` | 可选 | 本端接收门铃的虚拟 IRQ 号 |

改地址、token 或 IRQ 只动设备树（Zephyr 侧
`boards/rock_5b_plus_rk3588_smp.overlay` 的 `zv_shm` 节点），不改代码。
Linux 侧需要一个同 compatible 的节点（携带 token 32 与 IRQ 42），由
Linux Guest DT 提供。

## 9. 非目标

本协议不实现：ring buffer、多生产者、多客户端、历史队列、复杂重传与流控、
持久化、吞吐优化、生产级故障恢复。单槽协议下若组帧层中途放弃某帧，槽会一直
BUSY 导致该方向停摆，依赖上层的重连机制恢复。
