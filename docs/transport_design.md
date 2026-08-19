# micro-ROS 共享内存传输方案（RPMsg/OpenAMP 兼容）

## 1. 目标与拓扑

最终产品形态：自研 hypervisor 在 RK3588 上同时启动两个客户机——

- **Zephyr**：运行 micro-ROS client（rpmsg **master** 角色）；
- **Linux**：运行 micro-ROS agent（rpmsg **device** 角色）。

hypervisor 将同一片共享内存直接映射给两个客户机，mailbox MMIO 直通，guest 侧所有行为与裸机一致。wire 协议与现有 Zephyr OpenAMP 传输完全兼容（rpmsg + vring + mailbox doorbell），Zephyr 侧传输代码无需改动即可在生产环境复用。

```
生产:  Zephyr VM (client, rpmsg master) <--shm vring + mailbox 双向 doorbell--> Linux VM (agent, rpmsg device)
测试:  Zephyr (QEMU -kernel)           <--shm(文件后端) + kick/bell(unix socket)--> host docker 里的 agent
```

测试拓扑用 QEMU `rock-5b-plus` 机器在 host 上 1:1 复刻生产协议：共享内存由 host 文件后端提供，mailbox 双向 kick 各由一条 unix socket 承载（`kick`：agent→Zephyr 注入 IRQ；`bell`：Zephyr→agent 每字节一次门铃）。

内存布局（产品约定）：

| 区域 | 范围 | 用途 |
|---|---|---|
| shm | [0x30000000, 0x31000000) | RPMsg 共享内存，16 MiB |
| DRAM | [0x40000000, 0x60000000) | Zephyr guest RAM，512 MiB |

对端（Linux agent）由 hypervisor 负责启动，Zephyr 侧不做任何唤核操作（无 PSCI CPU_ON）。

## 2. 协议常量（两侧必须一致）

| 项目 | 值 |
|---|---|
| 共享内存窗口 | 0x30000000，16 MiB |
| virtio status 页 | shm + 0x000（0x400 字节，device 侧写 status） |
| buffer pool | shm + 0x400 起（master 管理） |
| master TX vring（device 的 RV） | shm + 0xFFF800 |
| master RX vring（device 的 SV） | shm + 0xFFFC00 |
| vring | 16 descriptor，align 4（vring 位置公式：base + 窗口大小 - 0x800 / - 0x400，随窗口大小变化） |
| rpmsg payload 上限 | 496 字节（512 pool buffer - 16 rpmsg header） |
| endpoint 名 | `microros`（device `rpmsg_create_ept` 触发 NS announcement 完成绑定） |
| mailbox MMIO | 0xFE380000；A2B_CMD +0x00（master→device kick，A2B_STAT +0x04 / A2B_CLR +0x08）；B2A_CMD +0x10（device→master kick，触发 Zephyr GIC SPI 219） |
| 门铃语义 | **双向**：master→device 写 A2B_CMD（生产由 hypervisor/硬件 IRQ 注入 Linux；QEMU 测试经 `bell` socket）；device→master 写 B2A_CMD（Zephyr 侧 GIC SPI 219；QEMU 测试经 `kick` socket 注入） |
| XRCE 组帧 | stream framing（HDLC），client 侧 `MICRO_ROS_FRAMING_REQUIRED=true` |

## 3. 各组成部分

### 3.1 Zephyr 侧（本仓库）

传输实现：`modules/libmicroros/microros_transports/openamp/microros_transports.c`

- master 角色（`CONFIG_RPMSG_SERVICE_MODE_MASTER=y`），SYS_INIT(POST_KERNEL, 47) 注册 endpoint `microros`；
- PRE_KERNEL_1 阶段 `arch_mem_map` 恒等映射 shm（提前映射，晚了会 data abort）；
- read = ring buffer + k_sem；write = `rpmsg_service_send`，-ENOMEM 重试；
- `zephyr_transport_open` 只等待 endpoint 绑定，**不做唤核**：对端（Linux agent VM）由 hypervisor 启动；
- 相关 Kconfig：
  - `MICROROS_OPENAMP_BIND_TIMEOUT_MS`（默认 5000）：等待 endpoint 绑定的超时，**0 = 无限等待**。生产场景 agent（Linux VM）启动远晚于 Zephyr，应设为 0。
- `tests/agent_test.conf`：对接 Linux agent 的配置片段（`BIND_TIMEOUT_MS=0`），用法：
  `west build -p always -b rock_5b_plus/rk3588/smp . -d build -- -DEXTRA_CONF_FILE=tests/agent_test.conf`
- `boards/rock_5b_plus_rk3588_smp.overlay`：dram 迁到 `[0x40000000, +512MiB)`，`shm0: memory@30000000`（16 MiB），`chosen { zephyr,ipc=&mbox0; zephyr,ipc_shm=&shm0; }`，`&mbox0 { status="okay"; }`。

### 3.2 QEMU 测试设施（fork：`/home/lhq/Workspace/qemu`）

1. 机器属性 `shm-file=<path>`（`hw/arm/rk3588.c`）：设置后在 0x30000000 建 16 MiB `memory-backend-file`（share=on）区域（位于 DRAM 之外的独立窗口），host 进程 mmap 同一文件即可与 guest 共享内存。不设置则行为不变（注意：`zephyr-ram=on` 场景必须设置，否则 shm 窗口是空洞）。
2. `zephyr-ram=on` 时直接内核 RAM 位于 0x40000000（`-m 512M`）。
3. mailbox 设备属性 `kick`（`hw/misc/rk3588_mailbox.c`）：可选 chardev；每收到 1 字节等价于 guest 写 B2A_CMD（置 pending + 按 B2A_INTEN 拉 IRQ）。接线：`-chardev socket,id=kick0,path=<sock>,server=on,wait=off -global rk3588-mailbox.kick=kick0`。
4. mailbox 设备属性 `bell`：可选 chardev；guest 每写一次 A2B_CMD（master→device kick）向该 chardev 发 1 字节，host 侧阻塞读即等价于收到中断。接线同上加 `-global rk3588-mailbox.bell=bell0`。
5. 机器属性 `hvc-doorbell=on`（zvisor 传输用）：拦截 guest 的 HVC 0x86000000（ZVisor 门铃 hypercall），等价为向 bell chardev 写 1 字节，x0 返回 0（SMCCC 成功）。实现照搬本 fork 已有的板级 SMC 钩子模式：`target/arm/internals.h` 声明 `arm_register_hvc_handler()`，`target/arm/psci-common.c` 存放钩子，`target/arm/helper.c` 的 `arm_cpu_do_interrupt()` 在 PSCI 判定前调用，默认 NULL 无行为变化；rk3588.c 的钩子函数把门铃转成 `rk3588_mailbox_ring_bell()`。
6. 机器属性 `kick-irq=<n>`（zvisor 传输用）：kick chardev 每收 1 字节，除原有 B2A 语义外，再脉冲 mailbox 第二路 IRQ 输出（边沿语义），rk3588.c 把它接到 GIC 外部中断 n。**编号是 SPI（gpio 索引），不是 INTID**：QEMU GICv3 的 `qdev_get_gpio_in(gic, i)` 对应 INTID i+32（见 `hw/intc/arm_gicv3.c` 的 `gicv3_set_irq`），所以 Zephyr IRQ 41（INTID 41）= kick-irq=9。Zephyr 侧门铃 IRQ 需配成边沿触发（`IRQ_TYPE_EDGE`）：TCG 下 pulse 的拉高/拉低发生在同一主循环回合内，电平触发的 pending 会随拉低立即消失，中断永远丢失。

### 3.3 Linux agent 侧：Micro-XRCE-DDS-Agent 一等 OpenAMP 传输

代码位于 fork `~/Workspace/Micro-XRCE-DDS-Agent`（v2.4.3），新增 `openamp` 一等传输（`OpenAMPServer` + rpmsg device 层 `rpmsg_device.{c,h}` + 共享后端 `transport/shm/shm_backend.{c,h}`），CLI：

```
micro_ros_agent openamp <--shm-file <path> --kick-sock <path> --bell-sock <path> | --shm-mem <phys> --kick-mmio> [--bind-timeout-ms <n>] [-v6]
```

agent 侧实现细节（子模块版本、rpmsg device 初始化序列、RESET 自愈、CMake、Dockerfile.microros-shm 镜像构建）见 **Agent 仓库 `docs/shmem_transports.md`**。

构建：

```bash
cd ~/Workspace/Micro-XRCE-DDS-Agent
git submodule update --init   # 若子模块尚未 checkout
docker build -t microros-shm-agent -f Dockerfile.microros-shm .
```

### 3.4 运行脚本（本仓库 `tests/scripts/`（已加入 .git/info/exclude））

- `run_qemu_agent.sh`：`-M rock-5b-plus,zephyr-ram=on,shm-file=/dev/shm/mros_shm -smp 4 -m 512M`、kick socket（/tmp/mros_kick）+ bell socket（/tmp/mros_bell），guest 上电即运行（不再有 `-S -s` 暂停/gdb 放行步骤；时序由 `BIND_TIMEOUT_MS=0` 兜底）。启动前 `rm -rf` 清掉旧 shm 文件与 socket。
- `run_agent_shm.sh`：docker 运行 `microros-shm-agent`（fork 镜像，entrypoint 为原版 `micro_ros_agent` 二进制），bind-mount shm 文件与两条门铃 socket，`--net=host --ipc=host`，参数为 `openamp --shm-file /shm/mros_shm --kick-sock /shm/mros_kick --bell-sock /shm/mros_bell`。启动前等待三个文件出现并校验类型（shm=普通文件、门铃=socket），因此**两个脚本启动顺序任意**；若路径是 docker 误建的 root 属主目录（docker 会把不存在的 bind 源自动建成目录）则报错提示 sudo 清理，不会死等。

## 4. 测试流程（QEMU 端到端）

```bash
# 1. 构建 Zephyr（agent 对接配置）
export ZEPHYR_BASE=/home/lhq/Workspace/infineon/zephyr
west build -p always -b rock_5b_plus/rk3588/smp . -d build -- -DEXTRA_CONF_FILE=tests/agent_test.conf

# 2. 构建 agent 镜像（Micro-XRCE-DDS-Agent fork，含 openamp 一等传输）
cd ~/Workspace/Micro-XRCE-DDS-Agent
git submodule update --init
docker build -t microros-shm-agent -f Dockerfile.microros-shm .
cd -

# 3. 运行（两个终端，顺序任意）
./tests/scripts/run_qemu_agent.sh      # guest 上电即运行
./tests/scripts/run_agent_shm.sh       # 若先启动会等 QEMU 建好 shm 文件与 socket

# 4. 验证（ros2 CLI 容器同样需要 --ipc=host，否则 FastDDS 的 SHM 传输
#    在容器私有 IPC 命名空间下失败，只能退化为不可靠的 UDP 回退）
docker run --rm --net=host --ipc=host --entrypoint bash microros/micro-ros-agent:jazzy -c \
    'source /opt/ros/jazzy/setup.bash; ros2 topic echo /zephyr_int32_publisher'
```

预期：guest 打印 `RPMsg endpoint bound, transport open`；agent 日志出现 `session established` 与 `datawriter created`；`ros2 topic echo` 收到递增 int32。

demo 同时带一个 subscriber（`/host_int32_publisher`，std_msgs/Int32），验证反向（host→Zephyr）数据：

```bash
docker run --rm --net=host --ipc=host --entrypoint bash microros/micro-ros-agent:jazzy -c \
    'source /opt/ros/jazzy/setup.bash; ros2 topic pub -t 4 /host_int32_publisher std_msgs/msg/Int32 "{data: 777}"'
# guest 控制台应打印 4 行：micro-ROS: received from host: 777
```

## 5. 已验证结果

- QEMU 冒烟：`shm-file` 创建 16 MiB 后端文件；kick socket 注入 1 字节后 B2A_STAT=1（等价 guest 写 B2A_CMD）；默认配置无行为变化。
- 共享内存端到端（新内存布局 shm 0x30000000 / DRAM 0x40000000，唤核代码已移除）：guest 打印 `RPMsg endpoint bound, transport open`；agent 日志出现 `doorbell from master, draining RX vring`（双向门铃工作），session/participant/topic/publisher/datawriter 全部建立；独立容器 `ros2 topic echo` 收到递增 int32（`--net=host --ipc=host`）。
- 注意：QEMU 侧改动已精简为 agent 路径必需的最小集（`zephyr-ram`、`shm-file`、mailbox `kick`/`bell` 双门铃）。`zephyr-ram=on` 时必须同时传 `shm-file`——0x30000000 不再有任何兜底映射，缺省会让 Zephyr 在 PRE_KERNEL_1 恒等映射 shm 时 data abort（控制台无任何输出）。

注意：FastDDS 同主机默认走 SHM 传输；docker 容器默认有私有 IPC 命名空间，跨容器订阅必须给两边都加 `--ipc=host`（`run_agent_shm.sh` 已内置），否则数据可能因 SHM 段不可见而丢失、只能靠 UDP 回退偶发成功。

## 6. 生产落地说明与限制

- hypervisor 需把 `[0x30000000, 0x31000000)` 这片物理内存映射给 Zephyr VM 与 Linux VM，并直通 mailbox MMIO；agent 使用默认参数（`openamp --shm-mem 0x30000000 --kick-mmio`，即 Dockerfile 默认 CMD）。
- 门铃是双向的：device→master 由 agent 写 B2A_CMD（mailbox MMIO）；master→device 在 QEMU 测试里经 `bell` socket 送达，生产 mmio 模式下 agent 以 1 ms 粒度轮询 A2B_STAT/A2B_CLR 兜底——如需真正的中断模式，需 hypervisor 将 mailbox IRQ 注入 Linux VM 后把 `bell_wait` 换成 UIO/事件等待。
- agent 的 rpmsg device 实现协议细节见第 2 节，两侧常量必须保持一致。
- 新 DDS 参与者的 topic 发现有数秒延迟（容器冷启动时 `ros2 topic echo` 前两次可能报未发现 topic），属正常 DDS 行为，非传输问题。

## 7. 传输三：ZVisor 共享内存 + HVC 门铃（hypervisor 方案）

最终产品形态：ZVisor hypervisor 启动一个 Zephyr VM 与一个 Linux VM，两者经 hypervisor 提供的共享内存窗口通信，门铃走 SMCCC HVC，由 hypervisor 转发为对端虚拟 IRQ。本传输与 openamp 并存，由 Kconfig 二选一（`CONFIG_MICROROS_TRANSPORT_ZVISOR_SHMEM`，prj.conf 默认即它；QEMU 演示用 `-DEXTRA_CONF_FILE=tests/agent_test.conf` 切回 openamp）。

### 7.1 拓扑与协议常量（两侧必须一致）

| 方向 | 共享窗口（GPA） | 写方门铃 | 收方中断 |
|---|---|---|---|
| Zephyr → Linux | `zv_shm` 前半 @ 0x30000000（8 MiB） | HVC 0x86000000，token 16 | Linux IRQ 42 |
| Linux → Zephyr | `zv_shm` 后半 @ 0x30800000（8 MiB） | HVC 0x86000000，token 32 | Zephyr IRQ 41 |

- 线协议：每窗口单槽，帧格式 `[u32 payload 长度][u32 state][payload]`（payload 上限 2048 字节，只用窗口首 4 KiB）。state 二值：0=FREE（已消费、写方可复用），1=BUSY（帧待消费）；任意时刻每方向只允许一条消息在途。
- TX：等槽位变 FREE（Zephyr 侧无限等——对端 agent 可能晚启动，与串口传输"阻塞等对端"语义一致；agent 侧 1s 超时丢帧并触发 fini/init 自愈）→ 写 payload/len → 置 BUSY → 内存屏障 → 门铃；RX：门铃中断只负责唤醒读者（ISR 里**不拷贝数据、不归还槽位**），读者线程检查 state==BUSY（否则视为虚假中断丢弃），把帧拷进 stash 后才把槽置回 FREE——槽位在读者消费完之前绝不归还，否则对端可能在上一帧还没读完时写入下一帧（压测抓到过这个竞态，反向 500 条丢 487 条）。
- 重启初始化：两侧 open/init 时都把两个槽的 len/state 清零置 FREE，丢弃对端或自己上一次运行的残留帧。
- 窗口映射为 cache-none（Zephyr 侧 device 内存）：**不能对它用 memcpy**——memcpy 可能生成 ldp/stp，device 内存上非 16 字节对齐的 ldp/stp 会触发 alignment fault（payload 偏移 8 必然踩中）；实现里用 4 字节粒度的 window_read/window_write。
- 帧与字节流的差异：每个槽是一整帧，但 StreamFramingProtocol 读写是**小块字节流**（先读头部再读 payload），两侧 read 都内置 stash/offset 保存未消费尾部，不能整帧取整帧丢（openamp 传输的 RX ring buffer 解决的是同一个问题）。

### 7.2 Zephyr 侧实现

- 传输代码：`modules/libmicroros/microros_transports/zvisor_shmem/microros_transports.c`；窗口 GPA/token/IRQ 全部从设备树节点读（binding：`zephyr,zvisor-shmem.yaml`，在 Zephyr 树 `dts/bindings/zvisor/`）。
- 端点节点见 `boards/rock_5b_plus_rk3588_smp.overlay` 的 `zv_shm`（单节点 16 MiB，代码按 reg 大小对半切出 req/rsp 两半，token/IRQ 同挂一个节点）；改地址、token 或 IRQ 只动 overlay，不改代码。
- `k_mem_map_phys_bare` 映射窗口首 4 KiB（`K_MEM_CACHE_NONE|K_MEM_PERM_RW`）；`IRQ_CONNECT` 接 `zv_shm` 的 `zephyr,doorbell-irq`（`IRQ_TYPE_EDGE`）。ISR 只做 `k_sem_give` 唤醒；帧拷贝与槽位归还都在读者线程（传输 read 回调）里完成。
- Linux 侧对等实现参考 Zephyr 树 `scripts/zvisor/rock5b/drivers/linux/zvisor_shmem.c`（内核模块，同一协议）。

### 7.3 Linux agent 侧实现（Micro-XRCE-DDS-Agent fork）

fork：`~/Workspace/Micro-XRCE-DDS-Agent`。新增一等传输 `zvisor-shm`（`UAGENT_ZVISOR_PROFILE`，Linux 默认 ON）：`src/cpp/transport/zvisor/zvisor_slot.{c,h}`（单槽协议，与 Zephyr 侧逐条对应）+ `ZvisorServerLinux.cpp/hpp`（`Server<CustomEndPoint>` 子类，结构同 OpenAMPServer，上层走 StreamFramingProtocol），复用共享后端 `transport/shm/shm_backend`。**实现细节、CLI、CMake 见 Agent 仓库 `docs/shmem_transports.md` 第 4 节**。

CLI：`micro_ros_agent zvisor-shm --shm-file ... --kick-sock ... --bell-sock ...`（QEMU 测试模式）或 `zvisor-shm --shm-mem 0x30000000 --kick-mmio`（/dev/mem + mailbox 模式）。真 ZVisor 硬件移植只剩门铃：`shm_backend` 门铃通道已是 ops 表，加一个 misc 驱动后端实例即可（kick=ioctl NOTIFY 触发 HVC token 32，bell=poll 等 IRQ 42），窗口仍 /dev/mem mmap（Linux VM 与 Zephyr 同物理地址），槽协议零改动。

测试脚本：`TRANSPORT=zvisor-shm ./tests/scripts/run_agent_shm.sh`（默认仍 openamp）。

### 7.4 构建

```bash
# 默认即 ZVisor 传输（prj.conf）
west build -p always -b rock_5b_plus/rk3588/smp . -d build
# 确认生效：grep MICROROS_TRANSPORT build/zephyr/.config 应为 ZVISOR_SHMEM=y
```

openamp 回归：`-DEXTRA_CONF_FILE=tests/agent_test.conf` pristine 重建（该片段会选回 openamp 并带上 RPMSG master 配置）。注意切换传输必须 `-p always`，增量构建的 `.config` 缓存不会切换 Kconfig choice。

### 7.5 验证状态

- QEMU 端到端已验证（`hvc-doorbell=on,kick-irq=9`，见 3.2 节 5/6 条；`run_qemu_agent.sh` + `TRANSPORT=zvisor-shm run_agent_shm.sh`，启动顺序任意）：
  - Zephyr→Agent→ROS 2：`ros2 topic echo /zephyr_int32_publisher` 收到 guest 递增数据；
  - ROS 2→Agent→Zephyr：`ros2 topic pub -t 4 /host_int32_publisher std_msgs/msg/Int32 "{data: 777}"` 后 guest 恰好打印 4 行 `received from host: 777`；
  - openamp 回归同流程通过（endpoint bound + datawriter created + echo + 4/4），证明两条传输可并存切换。
- 验证中踩过的坑（均已修，勿再踩）：①agent 侧 read 整帧取整帧丢导致 StreamFramingProtocol 字节流永不同步（需 RX stash，见 7.1）；②QEMU 侧 kick 脉冲配电平触发 GIC 中断必丢，Zephyr 侧门铃 IRQ 必须 `IRQ_TYPE_EDGE`（见 3.2 第 6 条）；③QEMU GIC gpio 索引 = SPI = INTID-32，IRQ 41 对应 `kick-irq=9`；④cache-none device 内存上 memcpy 的 ldp/stp 会 alignment fault，窗口访问走 4 字节粒度读写（见 7.1）；⑤ISR 里拷贝帧并提前归还槽位的竞态——对端可在上一帧未读完时写入下一帧，压测下反向 500 条丢 487 条；修复为 ISR 只唤醒、读者线程拷贝并归还槽位（见 7.6）。
- 待做：真 ZVisor 硬件验证（HVC token 16/32 与虚拟 IRQ 41/42 由 hypervisor 转发，QEMU 钩子语义已对齐；agent 侧 kick/bell 换 zvisor_shmem 内核驱动通道，槽协议不变）。
- 已知残留：QEMU 被 SIGTERM 拆掉时 agent 会记录一次 `transport error, reinit`（bell socket 随 QEMU 退出而 EOF），属正常拆链噪声，不影响运行期通信。

### 7.6 压力测试

设施（全部在 `tests/` 下，已 git exclude）：

- `tests/stress/`：独立 Zephyr 压测 app（构建产物在 `build/stress`，复用根 `boards/` overlay）。Guest 全速发布带序号的 `std_msgs/Int32`（topic `zephyr_stress`，best-effort），并订阅 `host_stress`（best-effort）校验序号连续性；独立线程每 2s 打印 `STRESS tx=… tx_rate=…/s rx=… rx_lost=… …`（含传输层调试计数器：写等待、ISR、坏帧、读超时）。
- `tests/scripts/stress_host.py`：host 端驱动（rclpy，容器内运行）。phase A 统计正向速率与序号空洞；phase B 以 `--pace` 限速发反向突发。接收必须 busy-pump（`spin_once` 一次只处理一个回调，睡眠式 spin 跟不上 5k/s 会让 DDS 读方历史溢出——那是测试假象不是链路丢包）。
- `tests/scripts/stress_test.sh`：一键编排（构建 → QEMU → agent → host driver → 汇总 guest STRESS 行与 agent transport 错误数）。`TRANSPORT=openamp` 可回归 openamp 路径；`DURATION/BURST/PACE/SKIP_BUILD` 可调。

实测结果（QEMU TCG，Int32，XRCE 帧约 30 B）：

| 场景 | 结果 |
|---|---|
| 正向 best-effort 稳态 | ~4600–5200 msg/s，预热后 0 丢失（预热期空洞来自 DDS 匹配建立，非链路） |
| 正向 reliable 稳态 | ~1350 msg/s，0 丢失（吞吐受 XRCE ACKNACK 往返节流） |
| 反向 200/s 限速 × 正向 4600/s 并发 | 400/400，0 丢失 |
| 反向 500/s 限速 × 正向满速并发 | 偶发 0–26% 丢失（三次运行：500/500、410+90lost、368+132lost） |
| 反向无限速瞬时突发 500 条 | 大部分丢失（DDS 各级 keep-last 历史深度溢出，与传输无关） |

结论与定位：

- 传输层（单槽 + 双门铃）双向无错误：所有运行中 agent `transport error` 为 0、guest 坏帧为 0、agent 发出的每一帧都被 guest 完整消费。
- 正向背压按设计工作：guest 写在槽位 BUSY 时阻塞等待（`wrwait` 计数），publisher 速率自动收敛到链路能力。
- 满负荷并发出现在 XRCE session/DDS 排队层的丢失（`RMW_UXRCE_STREAM_HISTORY=4` 的小历史缓冲在 5k/s 全双工下溢出），不是槽协议问题。使用建议：高吞吐数据通道用 best-effort；反向命令/控制通道限速 ≤200/s 或用小流量 reliable QoS。要更高反向速率需加深 XRCE stream history 或加应用层流控——属 PoC 非目标（环形队列、复杂流控）。

压测顺带修复的真实缺陷：Zephyr 侧 ISR 原先在中断里拷贝帧并立即置 FREE，读者线程尚未消费完时新帧即可覆盖 `rx_buf`（反向 500 丢 487）；现改为 ISR 只唤醒、读者线程拷贝并归还槽位（与 agent 侧单读者语义对齐）。
