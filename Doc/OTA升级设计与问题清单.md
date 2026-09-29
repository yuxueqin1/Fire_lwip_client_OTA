# OTA 远程升级 —— 设计说明 / 工作流程 / 问题清单

> 通信板型号：YH-F429（STM32F429IGT6，1MB Flash）
> 建档日期：2026-09-22
> 状态：代码修复完成、本机双镜像编译验证通过，**待上板实测**。进度见第 5 章，编译/测试步骤见第 7 章。

---

## 0. 固件体积常识（重要）

Keil 的 `Output/YH-F429.hex` 是 **ASCII 文本**，不是实际固件体积。
Intel HEX 每条记录 = `':' + LL(2) + AAAA(4) + TT(2) + 16字节数据(32) + CC(2) + CRLF(2)` = 45 字节。

> **换算公式：`.hex` 字节数 ÷ 2.8125 ≈ 实际镜像（`.bin`）字节数**

以 2026-09-22 加入 OTA 模块后的构建为例：

```
Output/ota/YH-F429_v1.0.0.1_A.bin  =  111420 字节  (108.8 KB)   ← OTA 实际传输的就是这个
Output/ota/YH-F429_v1.0.0.1_B.bin  =  111420 字节  (108.8 KB)
```

同一次构建的 `Output/YH-F429.hex` 约 313 KB —— `313000 / 111420 ≈ 2.81` ✓ 正好是 HEX 编码膨胀比。

**所以「hex 看着 300 多 KB」其实只有 109 KB。** 看体积一律以 `.bin` 为准。

### 0.1 固件大小 → 需擦扇区数

| 固件大小 | 需擦扇区数 | 擦除耗时（典型） |
|---|---|---|
| ≤128 KB（当前 108.8 KB，1 个扇区） | 1 | ~1.1 s |
| 128–256 KB | 2 | ~2.2 s |
| 256–384 KB（B 区上限） | 3 | ~3.3 s |

---

## 1. 角色与文件分工

| 端 | 文件 | 作用 |
|---|---|---|
| IAP / Bootloader | `ota升级需要的iap代码/lwip_client_Bootloader_RTOS_Final（完成版别动了）/User/main.c` | 上电读 boot_info，校验后跳转 APP |
| APP-OTA 客户端 | `User/ota/ota_client.c` | OTA 独立 TCP 任务、命令分发、状态机 |
| | `User/ota/ota_manager.c` | 版本比较、擦写调度、写 boot_info |
| | `User/ota/ota_net.c` | 自定义帧收发（0xAA … 0xBB） |
| | `User/ota/ota_flash.c` | 扇区擦除 / 字编程 / boot_info 读写 |
| | `User/ota/ota_crc32.c` | 标准 CRC32（与 Python zlib.crc32 兼容） |
| | `User/ota/ota_config.h` | Flash 分区表、命令字、状态枚举、固件版本 |
| 上位机 | `PC_Server/ota_gui.py` | **实际使用的升级工具**（tkinter GUI，服务器主动推送） |
| | `ota_server.py` | 旧命令行版本，协议相同（128B 分块 + 每块 sleep 1s），已过时 |

> 注：`PC_Server/board_server.py` 是板卡配置 / Modbus 模拟工具，**不含 OTA 代码**。

### 1.1 帧格式（两端一致）

```
[0xAA][CMD:1B][SEQ:1B][LEN:2B BE][OFFSET:4B BE][DATA:LEN][CHECKSUM:1B][0xBB]
CHECKSUM = (前 9+LEN 字节之和) & 0xFF
```

实现位置：`User/ota/ota_net.c:64` / `PC_Server/ota_gui.py:92`

### 1.2 命令字（`User/ota/ota_config.h:58`）

| 命令 | 值 | 方向 | 说明 |
|---|---|---|---|
| `HELLO` | 0x00 | 客户端→服务器 | 上报当前固件版本（4B BE），兼作 60s 心跳 |
| `HELLO_RESP` | 0x80 | 服务器→客户端 | 保留未用 |
| `PUSH_UPDATE` | 0x10 | 服务器→客户端 | 推送升级请求：版本 + 大小 + CRC32 |
| `CONFIRM_UPDATE` | 0x11 | 客户端→服务器 | 同意升级 |
| `REJECT_UPDATE` | 0x12 | 客户端→服务器 | 拒绝升级 |
| `ERROR_UPDATE` | 0x13 | 客户端→服务器 | 上报错误码 |
| `START_DOWNLOAD` | 0x03 | 服务器→客户端 | 开始推送固件 |
| `DATA` | 0x04 | 服务器→客户端 | 固件数据块（含 offset） |
| `END` | 0x05 | 服务器→客户端 | 传输结束 |
| `ACK` / `NAK` | 0x06 / 0x07 | — | 已定义，当前未使用 |

---

## 2. Flash 分区

### 2.1 硬件扇区表（STM32F429IGT6，1MB）

| 扇区 | 起始地址 | 大小 | 典型擦除时间 |
|---|---|---|---|
| S0 | 0x08000000 | 16 KB | ~0.25 s |
| S1 | 0x08004000 | 16 KB | ~0.25 s |
| S2 | 0x08008000 | 16 KB | ~0.25 s |
| S3 | 0x0800C000 | 16 KB | ~0.25 s |
| S4 | 0x08010000 | 64 KB | ~0.9 s |
| S5 | 0x08020000 | 128 KB | ~1.1 s |
| S6 | 0x08040000 | 128 KB | ~1.1 s |
| S7 | 0x08060000 | 128 KB | ~1.1 s |
| S8 | 0x08080000 | 128 KB | ~1.1 s |
| S9 | 0x080A0000 | 128 KB | ~1.1 s |
| S10 | 0x080C0000 | 128 KB | ~1.1 s |
| S11 | 0x080E0000 | 128 KB | ~1.1 s |

**结论：APP 区从 0x08020000 往上全是 128 KB，擦除粒度无法再小。**

### 2.2 旧分区方案（有问题，已废弃）

| 分区 | 起始 | 大小 | 结束 | 问题 |
|---|---|---|---|---|
| Bootloader | 0x08000000 | 32 KB | 0x08008000 | OK |
| APP1 | 0x08008000 | 512 KB | 0x08088000 | 尾部落在 S8 中间，与 APP2 共用 S8 |
| APP2 | 0x08088000 | 480 KB | 0x08100000 | 尾部包含 S11（boot_info 区） |
| BootInfo | 0x080E0000 | S11 | 0x08100000 | 与 APP2 重叠 128 KB |

### 2.3 新分区方案（与扇区严格对齐）

| 分区 | 起始地址 | 结束地址（不含） | 大小 | 占用扇区 |
|---|---|---|---|---|
| **Bootloader** | 0x08000000 | 0x08008000 | 32 KB | S0, S1 |
| **APP1（A 区）** | 0x08008000 | 0x08080000 | **480 KB** | S2, S3, S4, S5, S6, S7 |
| **APP2（B 区）** | 0x08080000 | 0x080E0000 | **384 KB** | S8, S9, S10 |
| **BootInfo** | 0x080E0000 | 0x08100000 | 128 KB（仅用 16 B） | S11 |

- 两侧固件镜像相同 ⇒ **实际单镜像上限 = 384 KB**（取较小者）。当前固件 88 KB，余量充足。
- 每个分区都是完整扇区边界，擦 A 区不会碰到 B 区，擦 B 区不会碰到 boot_info。

### 2.4 链接地址要求

| 镜像 | Keil `IROM1` Start | Size | 生成的 scatter |
|---|---|---|---|
| Bootloader | 0x08000000 | 0x8000 | `LR_IROM1 0x08000000 0x00008000` |
| APP（A 区版本） | 0x08008000 | 0x78000 | `LR_IROM1 0x08008000 0x00078000` |
| APP（B 区版本） | 0x08080000 | 0x60000 | `LR_IROM1 0x08080000 0x00060000` |

并且 APP 必须在 `SystemInit()` 之后设置 `SCB->VTOR = 本镜像链接地址`。

---

## 3. 完整升级时序（目标流程）

### 阶段 0 — 上电，Bootloader 决策

`Ota_Task`（`bootloader main.c:192`）读 `0x080E0000` 的

```c
typedef struct {
    uint32_t magic;      // 0x55AA55AA
    uint32_t boot_app;   // 1 = APP1, 2 = APP2
    uint32_t crc;        // 【待启用】目标分区固件 CRC32
    uint32_t size;       // 【待启用】目标分区固件大小
    uint32_t version;    // 【待启用】目标分区固件版本
    uint32_t reserved;
} boot_info_t;
```

- `magic` 不匹配 → 写默认 `boot_app = APP1`，跳 APP1
- `boot_app == 1` → 校验 APP1 → 跳 0x08008000
- `boot_app == 2` → 校验 APP2 → 跳 0x08080000
- 校验失败 / 其他值 → 回退到另一个分区（待实现，见 P9-5）

`JumpToApplication()`（`bootloader main.c:305`）：校验 `*(app_addr)` 栈指针 ∈ [0x20000000, 0x20030000] → 取 `*(app_addr+4)` 复位向量 → 关中断、清 SysTick / NVIC / PendSV、`HAL_DeInit` + `HAL_RCC_DeInit`、`__set_MSP`、`SCB->VTOR = app_addr`、跳转。

> ⚠️ **关键约定：`app_addr` 处必须是向量表（+0 栈指针，+4 复位向量）。**
> 因此 **禁止在分区起始写入任何自定义头**（旧代码的 `ota_app_header_t` 方案与此冲突，已废弃）。

### 阶段 1 — APP 启动，OTA 客户端接入

`main.c` 调 `ota_client_init()` → `sys_thread_new("ota_client", prio=4, 4096B)`。

`ota_client_task`（`ota_client.c:116`）：
1. `ota_manager_init()` → 读 boot_info，确定 `g_current_app_addr` / `g_download_addr`
2. 每 10 s 尝试连接 **192.168.1.99:5002**（该地址固定；升级时只需把上位机网卡配成 192.168.1.99 即可连通）
3. 连上后立即发 `HELLO`，载荷 = 当前固件版本 + 当前所在分区
4. 之后每 60 s 补发 `HELLO` 兼作心跳
5. 循环收帧并分发处理

### 阶段 2 — 服务器推送

`ota_gui.py` 收到 HELLO：若 `client_version < firmware_version` → 回 `PUSH_UPDATE`
载荷 = `>I 版本 + >I 大小 + >I CRC32`（`ota_gui.py:363`）

### 阶段 3 — 确认 + 擦除

1. `ota_handle_push_update_request()`：校验大小、比较版本 → 同意
2. 客户端发 `CONFIRM_UPDATE`
3. `ota_start_download()` → **按实际固件大小**擦除下载分区所需的扇区
   - 每擦完一个扇区立即 `WatchDog_Clear()`
   - 擦除期间由 TIM6 + DMA 硬件自动翻转 PF10 喂狗（见 P2）

### 阶段 4 — 数据传输

- 服务器：`START_DOWNLOAD` → 循环 `DATA`（1 KB/块，块间延迟可调）→ `END` → 关连接
- 客户端：**缓冲做 TCP 流重组**，一次 `recv` 可解析多帧；按块批量写 Flash；滚动更新 CRC32

### 阶段 5 — 校验与切换

1. `ota_finish_download()`：CRC32 比对，一致 → SUCCESS
2. `ota_prepare_update()`：
   - 把 `{magic, boot_app = 另一个分区, crc, size, version}` 写入 boot_info（擦 S11）
   - **不写任何分区内头**
3. 断连 → 延时 5 s → `NVIC_SystemReset()`

### 阶段 6 — 重启回到阶段 0，Bootloader 跳入新分区

---

## 4. 问题清单

| 编号 | 严重度 | 问题 | 状态 |
|---|---|---|---|
| P0-1 | 🔴 | `ota_client_init()` 在 `main.c` 被注释，OTA 任务从不创建 | ✅ 用户已改 |
| P0-2 | 🔴 | APP 链接地址为 0x08000000（非 OTA 布局），会盖掉 Bootloader | ✅ 用户已改 |
| P0-3 | 🔴 | APP 未设置 `SCB->VTOR`，跳到 0x08008000 后中断向量全错 | ✅ 用户已改 |
| P1 | 🔴 | 分区内 32 B 头与 IAP 跳转点冲突（IAP 要求分区起始即向量表） | ✅ 已修 |
| P2 | 🔴 | Flash 擦除期间 CPU 被 stall，SP706S（1.6 s）喂不上狗 → 复位 | ✅ 已修 |
| P3 | 🟠 | 分区表与扇区边界不对齐，APP1 / APP2 / boot_info 互相覆盖 | ✅ 已修 |
| P4 | 🟠 | `ota_flash_erase()` 按固定 64 KB 步进，与 128 KB 扇区不对齐，S3 漏擦 | ✅ 已修 |
| P5 | 🟠 | `ota_net_recv_frame()` 不做 TCP 流重组，一次 recv 只解析一帧，多余字节丢弃 | ✅ 已修 |
| P6 | 🟠 | 客户端每帧后 `vTaskDelay(1000)`，限速 1 帧/秒 | ✅ 已修 |
| P7 | 🟡 | 错误码返回值语义错误；失败后 state 永久卡死无法重试 | ✅ 已修 |
| P8 | 🟡 | `len % 4 != 0` 拒写；`HAL_FLASH_Unlock` 后不 Lock；擦除越界检查过宽 | ✅ 已修 |
| P9-1 | 🟡 | `ota_net_send_frame()` 2048 B 栈缓冲可能溢出 11 B | ✅ 已修 |
| P9-2 | 🟡 | OTA 任务栈 4096 B 偏紧 | ✅ 已修 |
| P9-3 | ⚪ | 服务器地址硬编码 192.168.1.99:5002 | ✅ 确认为有意设计，不改 |
| P9-4 | 🟡 | 服务器发完 END 立即关连接，无应用层确认闭环 | ✅ 已修 |
| P9-5 | 🟠 | Bootloader 不校验目标分区固件完整性，半截固件直接跳入 → 变砖无回滚 | ✅ 已修 |
| P10-1 | 🔴 | `LWIP_SO_SNDTIMEO` 未开 → `setsockopt(SO_SNDTIMEO)` 静默失效 → 对端窗口关闭时 `send()` 永久阻塞 → OTA 任务卡死不再 recv → 本板接收窗口归零，双方互等，现场表现为本板持续回 `TCP ZeroWindowProbeAck` 且**重启上位机也不恢复** | ✅ 已修 |
| P10-2 | 🔴 | OTA 任务不在 `App_TaskStart` 的看门狗表里 → 它卡死时板子照常喂 SP706S，永不复位 | ✅ 已修（复位码 31） |

### 各问题详细说明

#### P1 — 分区内头 vs IAP 跳转点（设计级冲突）

- IAP 从 `APP1_ADDR` / `APP2_ADDR` **起始处**取栈指针与复位向量。
- 旧 OTA 代码把 32 B `ota_app_header_t` 写在分区起始（`ota_manager.c:241`），固件数据写在 `+32`（`ota_manager.c:160`）。
- ⇒ 升级重启后 IAP 在 APP2 起始读到 `magic = 0x12345678` 当作栈指针，0x12345678 < 0x20000000，校验失败 → `while(1)` 死循环，变砖。
- 另外 `ota_get_current_version()` 永远只读 APP1（`ota_manager.c:100`），从 APP2 运行时版本上报永远不对。

**修复方案**：删除分区内头方案，裸固件直写分区起始；版本 / 大小 / CRC 存入 `boot_info_t`（S11），IAP 用它做校验与回滚。

#### P2 — 擦除期间喂不上硬件看门狗

- SP706S 外部看门狗，超时 **1.6 s**，靠翻转 PF10（WDI）喂狗（`User/board.c:647`）。
- 喂狗由 `App_TaskStart`（prio 8）每 500 ms 执行一次（`User/main.c:412`）。
- STM32F4 擦写 Flash 期间总线被占用，**从 Flash 取指的 CPU 会被 stall**，连 SysTick 中断都无法进入 ⇒ 单个 128 KB 扇区擦除（~1.1 s，最坏数秒）就可能触发复位。
- 旧代码一次擦 4 个扇区（~4~16 s），几乎必然复位。

**注意**：`main.c:19` 中"HAL_ETH_Init 忙等 5 s 也不会复位"的推理在此不适用 —— 那是**可抢占**的忙等循环，优先级 8 的任务能抢进来喂狗；Flash 擦除是**不可抢占的硬件 stall**。

**修复方案**（三管齐下）：
1. 按固件实际大小擦除，只擦必需的扇区（88 KB 固件只需 1 个 128 KB 扇区）
2. 每个扇区擦完立即 `WatchDog_Clear()`
3. 擦除前启用 **TIM6 + DMA 自动翻转 PF10**，完全不依赖 CPU，CPU 停多久都照喂

#### P3 — 分区表错位

见 2.2 / 2.3。

#### P4 — `ota_flash_erase()` 步进错误

`curr_addr += SECTOR_SIZE(64 KB)` 但只擦 `curr_addr` 所在扇区（`ota_flash.c:54`）：

- 目标 APP2：命中 S8, S9, S10, S11 —— 多擦了 S11（boot_info）
- 目标 APP1：命中 S2, S4, S5, S6, S7，**S3(0x0800C000-0x08010000) 永远不擦** → 旧数据残留，`HAL_FLASH_Program` 只能 1→0，写失败 → 第二次升级（回滚）必然失败

**修复方案**：按真实扇区表步进。

#### P5 — TCP 流重组缺失

`ota_net.c:97` 一次 `recv()` 后只解析一帧，多余字节直接丢弃：

```c
len = recv(sock, buffer, sizeof(buffer), 0);   // buffer[2048]
frame->len = (buffer[3]<<8) | buffer[4];
memcpy(frame->data, buffer + 9, frame->len);
```

`TCP_WND = 6*TCP_MSS = 8760`（`User/arch/lwipopts.h:110`），`buffer[2048]`，DATA 帧 = 9+1024+2 = 1035 B。
窗口填满后 `recv(2048)` 会一次取回 **1 整帧 + 下一帧的前 1013 字节**：第一帧校验刚好通过（`buffer[1033]==0xBB` ✓），**第二帧前 1013 字节被静默丢弃**，TCP 流从此错位，之后隔帧丢弃，`END` 时 CRC32 必然不匹配。
历史备份目录 `lwip_client_OTA-可以传输但卡在0%` 的症状与之吻合。

另外 `recv` 返回值未校验是否 ≥ `9+len+2`，会读到缓冲残留数据当校验和。

**修复方案**：累加缓冲 + 精确按 `LEN` 消费，支持一帧 / 多帧 / 半帧。

#### P6 — 限速

`ota_client_task` 每处理一帧都执行循环末尾的 `vTaskDelay(pdMS_TO_TICKS(1000))`（`ota_client.c:287`），88 KB / 1 KB = 88 帧 ⇒ 至少 88 s。

**修复方案**：一次 recv 连发多帧，批量写 Flash，仅在空闲时 delay。

#### P7 — 状态机与错误处理

- `ota_handle_push_update_request()` 超限时 `return g_ota_manager.state`（= `OTA_STATE_ERROR_SIZE` = **8**），调用方判断 `> 0` 即当作"同意升级"并回 `CONFIRM_UPDATE`（`ota_client.c:180`）。应返回 -1。
- `ota_start_download()` 要求 `state == OTA_STATE_IDLE`，而正常流程 state 已是 `REQUEST_RECEIVED`，靠调用点手工改 state 绕过（`ota_client.c:201`），逻辑脆弱。
- **任何一次失败（CRC 不符 / 写失败 / 发送失败）后 state 永久 ≠ IDLE，本次上电内再也无法重试**（`ota_manager_init()` 只在任务启动时调用一次）。

**修复方案**：修正返回值语义；引入失败自恢复与重试计数。

#### P8 — Flash 写入约束

- `ota_flash_write()` 要求 `len % 4 == 0`（`ota_flash.c:86`）。
  **原因**：STM32F4 内部 Flash 编程以 32 bit 字为最小单位，代码用 `FLASH_TYPEPROGRAM_WORD` 按字编程，每次必须提供完整 4 字节；长度非 4 的倍数时最后不足 4 字节无法按字写入（也可改用 `FLASH_TYPEPROGRAM_BYTE` 逐字节补，但两端需保持一致）。
  **修复方案**：上位机侧把固件**补齐到 4 字节后再计算 size 与 CRC32**，客户端保留该检查作为断言。补齐的字节落在镜像末尾，永不执行，无害。
- `ota_flash_init()` 调 `HAL_FLASH_Unlock()` 后从不 Lock（`ota_flash.c:19`）。
- `ota_flash_erase()` 越界检查 `addr < APP1_ADDR || end > APP2_ADDR+APP2_SIZE` 允许擦掉当前正在运行的 APP1 区。

#### P9-4 — 交互无闭环

`ota_gui.py` 发完 `END` 立即关连接，不等客户端任何确认；客户端在 END 处理里直接拔线重启。失败时服务器无法感知（`ERROR_UPDATE` / `ACK` 命令已定义但正常路径无人使用）。

#### P9-5 — Bootloader 无完整性校验

`boot_info.crc` 字段完全未使用，IAP 只校验栈指针范围。若目标分区为半截固件（掉电、CRC 逻辑失效），IAP 会照跳 → 死机且无回滚。

**修复方案**：`boot_info` 增加 `size` / `version` 字段，IAP 跳转前按 CRC32 校验目标分区，失败则回退另一分区。

#### P10-1 — 对端窗口关闭时 `send()` 永久阻塞（ZeroWindowProbeAck 的真凶）

现场现象：用 `ota_gui.py` 升级时，通信板持续向上位机发 **TCP ZeroWindowProbeAck**，
升级流程永久挂住，**连重启上位机程序都救不回来**。

链路推理（`lwip/lwip-2.1.2`）：

1. `ota_net_connect()` 设了 `SO_SNDTIMEO`，但本工程 `lwipopts.h` 里 **`LWIP_SO_SNDTIMEO` 没定义（=0）**
   ⇒ `setsockopt()` 里的 `case SO_SNDTIMEO` 被条件编译掉，**静默失败**（返回成功、实际什么都没设）。
2. 于是整帧发送的 `while (total < pos) send(...)` 里，单次 `send()` 没有超时。
3. 对端接收窗口为 0（对端应用不读）时，lwIP 的 `netconn_write` 停留在 `NETCONN_WRITE` 状态
   （`api_msg.c` 的 `lwip_netconn_do_writemore()` 要等 `TCP_SND_BUF` 腾出来才继续），**不会返回**。
4. OTA 任务卡在这条 `send()` 上 ⇒ 它再也不调 `recv()` ⇒ **本板的接收窗口也归零**。
5. 双方互等：本板对"对端窗口=0"发 ZeroWindowProbe，对端对"本板窗口=0"也发 probe
   ⇒ 界面上看到的就是本板一直回 `ZeroWindowProbeAck`，且**永远不收敛**。
6. 重启 `ota_gui.py` 也没用：本板这一侧的连接状态还停在 `NETCONN_WRITE`，
   新连接进来时旧 socket 仍占着任务，OTA 任务根本没回到 accept/connect 循环。

**为什么以前没暴露**：只在"对端应用短暂不读"（PC 端 GUI 卡一下、磁盘写日志、
`IDLE_TIMEOUT` 之前那段）时才触发，触发一次就永久卡住，属于低频必死。

**修复方案（两层，见下表第 1 / 3 层）**：
- `lwipopts.h` 打开 `LWIP_SO_SNDTIMEO = 1`，并在 `ota_net_connect()` 里对
  `setsockopt(SO_SNDTIMEO)` 的返回值做检查（失败打印告警）。
- `ota_net_send()` 增加**整帧总时限** `OTA_SEND_TOTAL_TIMEOUT_MS`(8 s)：
  到点即返回 -1，不再无限重试 → 上层断开重连 → 板子始终能自己走回来。

#### P10-2 — OTA 任务没有看门狗

`App_TaskStart`（`main.c`）靠"各任务别忘了清自己的 WDT 计数器"兜底，
复位码 21-30 全是这套机制。**OTA 任务不在其中** —— 因为它天然会长时间不返回
（Flash 擦除数秒、`connect` 几十秒），套不进"每 500 ms 清零"的模型。

⇒ 它一旦卡死（P10-1 的 `send()`、卡在阻塞 `printf`、被高优先级任务饿死），
板子照常喂 SP706S，**永不复位**。这就是"重启上位机也不行"的最后一块拼图。

**修复方案**：新增 `ota_task_wdt_*`（`User/ota/ota_wdt.c`），用 `HAL_GetTick()`
（与调度器无关）判活：

| 层 | 手段 | 恢复方式 |
|---|---|---|
| 1 | `send()`/`recv()` 都有超时，绝不永久阻塞 | 软：返回 -1 → 断开重连 |
| 2 | `DOWNLOADING` 期间 30 s 无新字节 ⇒ 会话假死 | 软：主动断开重连，不重启板子 |
| 3 | OTA 任务 30 s 不上报存活 ⇒ `WTD_RESET = 31` ⇒ 停喂狗 | 硬：SP706S 复位板子 |

长操作（Flash 擦/校验、TCP connect）用 `ota_task_wdt_busy(ms)` 声明窗口，
窗口内不判超时；内部每次喂狗（`ota_wdt_feed()`）顺带 `ota_task_wdt_progress()`
顺延截止时刻，所以"一个扇区一个扇区地擦"不算卡死，真卡在某个扇区里仍会判超时。

`ota_task_wdt_arm()` 之前一律不判超时：OTA 任务若因堆不足没创建起来，
板子不该陷入"复位 → 重启 → 再复位"的死循环。

---

## 5. 修复进度

| 步骤 | 内容 | 状态 |
|---|---|---|
| 0 | P0 三项前置阻塞（`ota_client_init` 解注、IROM1 改 0x08008000、设置 VTOR） | ✅ 用户已完成 |
| 1 | 对齐分区方案（P3）+ 同步 IAP / APP / `ota_config.h` | ✅ 完成 |
| 2 | 去掉分区内 32 B 头，版本 / CRC / 大小迁入 `boot_info`（P1） | ✅ 完成 |
| 3 | 重写 `ota_net_recv_frame()` 为流式重组（P5） | ✅ 完成 |
| 4 | 擦除优化 + 分级喂狗 + TIM7/DMA 硬件喂狗（P2） | ✅ 完成 |
| 5 | 扇区表步进擦除（P4）、去客户端 1s 限速（P6）、状态机自恢复（P7）、Flash 约束（P8） | ✅ 完成 |
| 6 | IAP 侧固件完整性校验（P9-5）、上位机 END 确认闭环（P9-4） | ✅ 完成 |
| 7 | 上位机协议层重写（流重组 / 小端 / 双 ACK / A-B 配对 / 预检） | ✅ 完成 |
| 8 | Keil 双 Target（LwIP_A / LwIP_B）+ `build_ota.bat` + `VECT_TAB_OFFSET` 归一 | ✅ 完成 |
| 9 | **本机编译验证**：双 Target 均编译通过（A 0 警告 / B 仅历史警告），两镜像复位向量、栈指针、scatter 逐一核对通过 | ✅ 完成（见 6.1、8.1–8.3） |
| 10 | 日志可读性：版本号改 `v1.0.0.1` 点分格式、槽位改 `A`/`B`；IAP 加启动决策日志、调试串口与 APP 统一为 USART6 | ✅ 完成（见 7.3 / 7.5 / 第 9 章） |
| 11 | **上板实测**（PC 配 192.168.1.99，跑 `ota_gui.py`） | ⏳ 待用户执行，步骤见第 7 章 |
| 12 | **上板实测首轮暴露**：烧 bootloader + A 镜像后 APP 卡死（LED_485_1 常亮 / LED_RUN 不闪 / 串口零输出）。根因：IAP `JumpToApplication()` 用 `__disable_irq()` 关了全局中断，跳转前**只恢复了 FAULTMASK、漏了 PRIMASK**；PRIMASK 不被跳转清零，APP 的 SysTick 中断进不来，`HAL_Delay()` 死等 `uwTick` | ✅ 已修复，跳转前补 `__enable_irq()`；反汇编确认 `CPSID i` … `CPSIE i` / `BLX` 顺序正确。详见 §8.4 |
| 13 | **上板实测二轮暴露**：OTA 传输走到 100%、字节数完全正确，但 `CRC32: got 0xEA122474, expected 0xFD10E5F5` → 校验失败。根因：`ota_net_next_frame()` 在 **return 之前**就 `memmove` 压缩了接收缓冲，而 `out->data` 指着缓冲内部、载荷区正好落在被覆盖的前缀里 —— 调用方读到的是**下一帧的数据** | ✅ 已修复，载荷先拷进独立的 `g_frame_data[1024]` 暂存区；反汇编确认 `out->data` 存的是 `g_frame_data` 地址。详见 §8.5 |
| 14 | **上板实测三轮暴露**：升级卡住，通信板持续回 `TCP ZeroWindowProbeAck` 且重启上位机也不恢复。根因：`LWIP_SO_SNDTIMEO=0` ⇒ `send()` 无超时 ⇒ OTA 任务永久卡在 `send()`，双方窗口同时归零互等；且 OTA 任务不在看门狗表里，板子永不复位 | ✅ 已修：开 `LWIP_SO_SNDTIMEO` + 发送总时限 8 s + 会话停滞 30 s 软重连 + OTA 任务看门狗（复位码 31）。详见 P10-1 / P10-2 |

### 5.1 本阶段新增 / 修改的文件

| 文件 | 说明 |
|---|---|
| `User/ota/ota_config.h` | 分区表、帧格式、错误码、状态枚举 |
| `User/ota/ota_flash.{h,c}` | 按扇区表步进擦除、4 字节对齐写、Unlock/Lock 成对 |
| `User/ota/ota_manager.{h,c}` | 状态机、版本比较、槽位选择、自恢复重试 |
| `User/ota/ota_wdt.{h,c}` | **新增** TIM7 + DMA1_Stream2 循环写 `GPIOF->BSRR` 喂 SP706S |
| `User/ota/ota_net.{h,c}` | 流式接收缓冲 + 反同步扫描（杂散 0xAA 不再堵住后续真帧） |
| `User/ota/ota_client.c` | 一次 recv 连发多帧、批量写 Flash、去掉每帧 1 s 延时 |
| `User/system_stm32f4xx.c` | `VECT_TAB_OFFSET` 由 `OTA_SLOT_SEL` 推导，与链接地址不可能不一致 |
| `Project/YH-F429.uvprojx` | 单 Target → **LwIP_A / LwIP_B 双 Target**，输出目录分离 |
| `Project/build_ota.bat` | **新增** 一键双 Target 编译，从 `ota_config.h` 读版本号命名产物 |
| `PC_Server/ota_gui.py` | 协议层重写：流重组、小端、双 ACK 握手、A/B 自动配对、镜像预检 |
| `PC_Server/tests/*.py` | **新增** 端侧 34 项 + 上位机侧重写算法 11 组自检，全部通过 |
| `ota升级需要的iap代码/.../User/main.c` | IAP：`boot_info` 保留字清零、跳转前完整性校验、致命错误不再死循环点灯；**新增启动决策日志** |
| `ota升级需要的iap代码/.../User/usart/bsp_debug_usart.{h,c}` | **改为 USART6 (PC6/PC7) + PC8 方向脚**，与 APP 统一；`fputc` 加防卡死保护 |
| `ota升级需要的iap代码/.../User/board.c` | 放开被注释掉的 `DEBUG_USART_Config()` |

---

## 6. 已定方案：A/B 双分区（ping-pong）

已确认采用 A/B 双镜像，v1 **不做试运行回滚**，先把链路跑通。

A/B 双分区要求 APP 有 **两个链接地址不同的镜像**（0x08008000 与 0x08080000），因为 Cortex-M 代码含绝对地址（向量表 + 地址字面量），同一个 bin 无法在两个地址都跑。因此同一份源码必须编出两个镜像：

| Keil Target | 链接地址 IROM1 | `OTA_SLOT_SEL` | `VECT_TAB_OFFSET` | 产物 |
|---|---|---|---|---|
| `LwIP_A` | 0x08008000 / 0x78000 | 1 | 0x8000 | `Output/ota/YH-F429_v<版本>_A.bin` |
| `LwIP_B` | 0x08080000 / 0x60000 | 2 | 0x80000 | `Output/ota/YH-F429_v<版本>_B.bin` |

升级时设备永远写"当前没在跑"的那个分区，运行中的分区自始至终不被触碰 —— 即使升级中途断电，设备重启后仍在旧分区正常跑，回滚是天然的。

⚠ 两个 Target 的 `OutputDirectory` 必须分开（`Output/A`、`Output/B`）。若共用一个目录，切 Target 时 Keil 会认为 `.o` 还是新的而不重编译，于是"用 A 的目标文件链出 B 的镜像"，而 `OTA_SLOT_SEL` / `VECT_TAB_OFFSET` 都是编译期常量，串了以后在下载和校验阶段完全看不出来，极难排查。

> **本章的坑真实发生过。** 生成双 Target 的迁移脚本把 `LwIP_A` 的起始地址写成了 `0x8080000`（少写一个 0）。`0x8080000` = `0x08080000`，那是 **APP2** 的地址 —— 也就是说 A 镜像按 APP2 链接，却被写进 APP1 分区，同时 `VECT_TAB_OFFSET=0x8000` 又和链接地址对不上，跳过去必然跑飞；而下载、长度、CRC 校验全部通过。
>
> **两个 `0x8` 开头、后面只有 6 位十六进制的数，肉眼几乎分不出来，必须用工具验。** 每次改动链接地址后都要跑下面的复位向量检查。

### 6.1 镜像自检（改完链接地址必做）

```python
import struct
SLOTS = {'A': (0x08008000, 0x0807FFFF, 'APP1'),
         'B': (0x08080000, 0x080DFFFF, 'APP2')}
for name, (lo, hi, slot) in SLOTS.items():
    d = open('Output/ota/YH-F429_v1.0.0.1_%s.bin' % name, 'rb').read()
    sp, rv = struct.unpack('<II', d[:8]); rv &= ~1
    print('%s: 大小=%d 栈=0x%08X 复位向量=0x%08X 应属%s -> %s'
          % (name, len(d), sp, rv, slot, 'OK' if lo <= rv <= hi else 'FAIL'))
```

同时确认 `Output/A/YH-F429_A.sct` 里写着 `LR_IROM1 0x08008000 0x00078000`、`Output/B/…` 写着 `LR_IROM1 0x08080000 0x00060000`。
2026-09-22 修复后实测：A = `0x0800828C`（APP1 ✓）、B = `0x0808028C`（APP2 ✓）、栈 `0x2002AF78`（SRAM ✓）。

---

## 7. 编译与上板测试步骤

### 7.1 编译：双击 `Project\build_ota.bat`

脚本自动完成五件事，**不需要打开 Keil**：

1. 在固定候选路径里找 `UV4.exe`（已含 `D:\Program Files\Keil_v5\UV4\UV4.exe`）；
2. 从 `User\ota\ota_config.h` 读 `FW_VERSION_*` 拼出版本号；
3. 依次编译 `LwIP_A`、`LwIP_B`，日志写 `Project\build_LwIP_A.log` / `build_LwIP_B.log`；
4. 把产物复制成带版本号的名字到 `Output\ota\`；
5. 调 `Project\merge_hex.py` 把 **Bootloader + A 镜像合成产线 hex**（详见 10.2 做法二）。

正常输出：

```
Using Keil: D:\Program Files\Keil_v5\UV4\UV4.exe
Firmware version: v1.0.0.5   from ..\User\ota\ota_config.h
[ OK ] target LwIP_A built with warnings, see build_LwIP_A.log
[ OK ] target LwIP_B built with warnings, see build_LwIP_B.log
  A image: YH-F429_v1.0.0.5_A.bin   size=111248 bytes   - to APP1
  B image: YH-F429_v1.0.0.5_B.bin   size=111240 bytes   - to APP2
  Production: YH-F429_v1.0.0.5_production.hex   size=378691 bytes   - factory, ONE file
```

`UV4` 返回码：`0`=无警告、`1`=有警告（**可以继续**，B 镜像首次全量编译出几十条警告属正常）、`≥2`=有错误（脚本判失败，不收集产物）。

⚠ **改版本号只改 `ota_config.h` 一处**，文件名、上位机自动识别的版本、固件自己上报的版本三者必然一致。

### 7.2 上板

| 步 | 操作 |
|---|---|
| 1 | PC 有线网卡配成 **192.168.1.99 / 255.255.255.0**（服务器地址已固定为 `192.168.1.99:5002`，不要改固件里的地址） |
| 2 | 网线直连通信板，`ping 192.168.1.99` 确认网卡起来了 |
| 3 | 板子上电，确认当前跑的是**旧版本**。固件只在收到 **比自己新** 的版本时才升级，否则回 `REJECT_UPDATE` 什么都不发生 |
| 4 | 运行 `PC_Server\ota_gui.py`，把 `Output\ota\` 里的 **A、B 两个 bin 放在同一个文件夹**，选中任意一个 —— 工具会自动配对 |
| 5 | 点"开始升级"，等进度到 100% 并出现 CRC 校验通过 |
| 6 | 板子自动重启 → Bootloader 读 `boot_info` → 跳到新分区 → 串口日志版本号变成 v1.0.0.1 |

### 7.3 判定升级真的成功

**只看串口日志，不要只看上位机显示 100%。** 上位机 100% 只代表数据传输 + CRC 通过；真正的判据是设备重启后运行的是新分区。

**调试串口（APP 与 IAP 现已统一为同一个口）**

| 项目 | 值 |
|---|---|
| 外设 | **USART6** |
| 引脚 | PC6 = TX, PC7 = RX, **PC8 = RS485 方向控制(DE)** |
| 参数 | 115200, 8-N-1 |
| 接线 | 因为是 RS485 半双工，需要 **RS485 转 USB** 线，普通 TTL 线不行 |

**设备重启后，串口会依次打出下面这些行**（IAP 一段，APP 一段）：

```
===== OTA Bootloader =====
[IAP] boot_info: boot_app=A, ver=v1.0.0.1, size=111596, crc=0x12345678
[IAP] boot_info asks for slot A (0x08008000, 480KB)
[IAP] slot A image check: OK (rc=0)
[IAP] jumping to slot A @ 0x08008000 ...

[OTA] running slot = A (source: VTOR), VTOR=0x08008000
[OTA] current ver v1.0.0.1, target slot B (0x08080000, 384KB)
[OTA] boot_info: boot_app=A ver=v1.0.0.1 size=111596 crc=0x12345678
```

**判据是这一行 —— `[OTA] current ver`：**

| 现象 | 判断 |
|---|---|
| `[OTA] current ver` = 新版本 | ✅ **升级成功**，新固件在运行 |
| `[OTA] current ver` = 旧版本 | ❌ 还在旧分区跑，升级没生效 |
| `[IAP] jumping to slot X` 之后就没了 | ⚠️ 跳过去跑飞了 |
| IAP 那几行都没有 | ⚠️ 串口没通/接线不对，不代表升级失败，但也就没法用日志判断了 |

`[OTA] current ver` 是 `FW_VERSION` 宏，是**正在执行的这份镜像**在编译时烧进去的常量 —— 它显示新版本，就是"新固件真的在跑"的直接证据。这行只在 APP 完整跑起来后才会打印，所以"跳转后就静默"本身就是有效信号。

**`[IAP]` 那几行告诉你 Bootloader 做了什么决定**：读到的 `boot_info` 是什么、要跳哪个槽、镜像校验过没过、以及是否回滚。有了它，"Bootloader 挑错槽"和"跳过去跑飞"就能区分开了 —— 在此之前 IAP 是全程静默的，只能靠串口死寂来猜。

IAP 侧还保留了 LED 提示（跳转前闪一下，500ms，要盯着看）：

| LED 现象 | 含义 |
|---|---|
| 全部 485 LED 同时亮 300ms（仅首次上电） | boot_info 无效，已写入默认 A |
| RUN 灯亮 500ms | 即将跳 **A** |
| 485_2 灯亮 500ms | 即将跳 **B** |
| 485_3 亮 1s / 灭 1s / 再亮 1s | 目标槽校验失败，**回滚**到另一个槽 |
| 全部 485 LED 轮流闪个不停 | 两个槽都不可用，死循环（只剩调试口能救） |

**建议顺手验一条回滚路径**：升级中途给板子断电，重新上电后应仍在**旧版本**正常跑（A/B 的天然回滚），且日志里能看到 `[IAP]` 走的是正常路径而非回滚路径。

### 7.5 版本号 / 槽位的日志约定

版本号在日志里一律是 **`v1.0.0.1` 这种点分格式**，槽位一律用 **`A`/`B`**，不再是 `0x01000001` 和 `1`/`2`。

底层存储仍是打包成 32 位整数用于比较：

```
FW_VERSION = (MAJOR<<24) | (MINOR<<16) | (REVISION<<8) | BUILD
v1.0.0.1  ->  0x01000001
v1.0.0.2  ->  0x01000002
v1.0.0.10 ->  0x0100000A
```

打印时用 `ota_config.h` 里的两个宏（IAP 侧在 `main.c` 里有一份等价定义，因为 IAP 不包含 `ota_config.h`）：

```c
#define FW_VER_FMT      "v%u.%u.%u.%u"
#define FW_VER_ARG(v)   (unsigned)(((uint32_t)(v) >> 24) & 0xFFU), \
                        (unsigned)(((uint32_t)(v) >> 16) & 0xFFU), \
                        (unsigned)(((uint32_t)(v) >>  8) & 0xFFU), \
                        (unsigned)( (uint32_t)(v)        & 0xFFU)

#define OTA_SLOT_CHAR(s)   (((s) == OTA_SLOT_APP2) ? 'B' : 'A')

/* 用法 */
printf("ver=" FW_VER_FMT ", slot=%c\r\n", FW_VER_ARG(ver), OTA_SLOT_CHAR(slot));
```

之所以做成两个宏，是因为 printf 的可变参数没法在运行时拆字段。

### 7.4 已知约束

- 单镜像上限 **384 KB**（APP2 只有 384KB，取两者较小值）。超了脚本会在编译结束时提示，而不是升级到一半才失败。
- 固件长度必须是 **4 的倍数** —— Flash 按字（32 位）编程。Keil 产出的 bin 天然对齐；若用别的工具改过 bin，需在**上位机侧补齐**（不要靠设备端容忍）。

---

## 8. 调试过程中定位的五个非显性陷阱（已修）

### 8.1 源文件编码：UTF-8 源码 + GBK 主机 = 字符串被吞

ARM Compiler 5.06 **没有** `--source-charset` / `--locale` / UTF-8 选项（`--help` 只有 30 行，搜不到任何 charset/locale/multibyte 相关项）。它按 Windows 主机区域解码源文件，中文系统即 **GBK**。GBK 把 `0x81..0xFE` 的字节连同**后一个字节**一起当汉字吃掉，于是 UTF-8 的三字节汉字会吞掉紧随其后的 ASCII 字符。

**症状**：`ota_manager.c(95): error #8: missing closing quote` —— 一个中文串的 UTF-8 字节把结束引号吃掉了；更坏的情况是编译通过但日志缺字/串行。

**试过但被推翻的假设**：曾以为"非 ASCII 连续段字节数为奇数才会吞下一个字节，凑成偶数即可"。用一个隔离的最小例（单独一行 `s = "取默认值";`，CJK 段 12 字节 = 偶数）做对照实验，**照样报 missing closing quote** —— 该奇偶模型不成立。

**采用方案（C）**：OTA 相关的 4 个 `.c` 文件里 **70 处含中文的字符串字面量全部改成英文**，中文只留在注释里（注释不被解码成字符串，安全）。改完 `_find_strings.py` 扫描结果 `total=0`，编译从 4 error 降到 **0 error / 0 warning**。

> 用户原有 40 个 UTF-8 源文件里仍有 132 处潜在"字节配对"隐患，目前能编译通过，**未改动**。

### 8.2 `.bat` 批处理必须纯 ASCII

cmd.exe 用**控制台代码页（中文系统 = GBK）**解析 `.bat`，而脚本存成 UTF-8 时解析器位置会错位，cmd 开始把行片段当命令执行，实测出现：

```
'0.0.2_B.bin' 不是内部或外部命令
'onfig.h' 不是内部或外部命令       ← ota_config.h 的片段
```

在文件里写 `chcp 65001` **无效**，那行本身要等到损坏之后才执行。`build_ota.bat` 已重写为**纯 ASCII 英文**（注释也是英文），并把 `echo` 文本里的括号去掉（块内括号是另一个解析陷阱）。

### 8.3 HAL 版本不匹配：`AutoReloadPreload` 不存在

工程用的是 **STM32F4xx_HAL v1.7.3**（`__STM32F4xx_HAL_VERSION_MAIN 0x01 / SUB1 0x07 / SUB2 0x03`），其 `TIM_Base_InitTypeDef` 只有 Prescaler / CounterMode / Period / ClockDivision / RepetitionCounter 五个成员 —— **没有 `AutoReloadPreload`**，`TIM_AUTORELOAD_PRELOAD_DISABLE` 这个宏也不存在。`ota_wdt.c` 里那行是从新版 HAL 的例程抄来的，删掉即可（`TIM_DMA_UPDATE` / `__HAL_TIM_ENABLE_DMA` / `__HAL_TIM_SET_AUTORELOAD` 都存在，不影响 TIM7+DMA 喂狗方案）。

**教训**：往本工程贴 HAL 代码前，先核对本工程 HAL 版本，不要照搬网上新版例程。

---

### 8.4 跳转不是复位：四类 CPU 状态会从 IAP 带进 APP ← 板上实测首轮的真凶

`JumpToApplication()` 只是设好 MSP/VTOR 后 `BX` 过去，**这不是复位**。Cortex-M 里有一批状态是内核寄存器，**只有真正的复位才清零**，跳转会让它们原样带进 APP：

| 状态 | 谁设的 | 带进 APP 的后果 |
|---|---|---|
| `PRIMASK` | `__disable_irq()` | **全局中断一直关着**：SysTick / PendSV 都进不来，`HAL_Delay()` 死等 `uwTick` 不返回，FreeRTOS 调度器一起摆停 |
| `FAULTMASK` | `__set_FAULTMASK()` | HardFault 被屏蔽，出错后直接锁死，连调试器都难看出 |
| `CONTROL` | `__set_CONTROL(0)` | 栈指针用错（MSP/PSP），APP 一进来栈就错乱 |
| `SCB->VTOR` | IAP 自己的向量表 | 中断跳到 IAP 的（或空的）向量表 |

**本次真凶是 `PRIMASK`**：IAP 的 `JumpToApplication()` 里 `__disable_irq()` 之后，只补了 `__set_FAULTMASK(0)`，**PRIMASK 从头到尾没放开**（全文件 `__enable_irq` 出现 0 次）。跳转本身是"成功"的 —— PC 确实到了 `0x08008000`，所以从 LED 上完全看得出"跳过去了"，但 APP 一执行到第一个 `HAL_Delay()` 就永久停住。

**现场特征**（三个症状互为印证，可直接对号入座）：

- **`LED_485_1` 常亮** —— APP 的 `Startup_LED_Running()` 流水灯**第一句**就是 `LED_485_1_ON; HAL_Delay(80);`，卡在这句上，灯就恰好停在第一步（这是最精确的指纹）
- **`LED_RUN` 不闪** —— RTOS 从没起来，`Test1_Task` 的 1 Hz 心跳从没跑
- **串口一个字节都没有** —— `DEBUG_USART_Config()` 排在流水灯**后面**，根本没执行到。注意这跟"串口发不出去"是两回事：是串口**压根没被初始化**

**修法**：跳转前把中断放开，且必须放在 `SCB->VTOR = app_address;` **之后** —— 这样万一有迟到中断，它取到的是 APP 的向量表。改完用反汇编复核：

```
0x080025b2:  CPSID  i        ← __disable_irq()
0x080025de:  MSR FAULTMASK   ← __set_FAULTMASK(0)
0x08002614:  STR r5,[r0,#0]  SCB->VTOR = 0x08008000
0x08002636:  CPSIE  i        ← __enable_irq()   ← 本次补上的
0x0800263a:  BLX  r7         ← JumpToApp()
```

**教训**：写引导程序时，把"跳到 APP"当成"半个复位"来对待 —— 凡是 `__disable_irq()` / `NVIC->ICER` / `HAL_DeInit()` / 时钟切换动过的东西，都要有一份**对称的恢复清单**，缺一项就是一类"LED 上看得出来、日志上完全看不出来"的故障。

---

### 8.5 "指针还指着它，它已经被搬走了" —— OTA 传输 CRC 全错的真凶

**症状**：传输进度一路走到 100%，`received 111576 / declared 111576 bytes` 字节数**一个不差**，紧接着 `CRC32: got 0xEA122474, expected 0xFD10E5F5` → 校验失败。

这个组合（长度全对、内容全错）把范围缩得很小：
- 帧重组、分片、Flash 写**都没有丢数据**（否则字节数会少）
- 设备端 CRC 算法是对的（`zlib.crc32` 兼容，自检值 `CRC32("123456789")=0xCBF43926` 正确）
- 上位机声明的值也是对的（拿磁盘上的 `YH-F429_v1.0.0.2_B.bin` 算一遍，`zlib.crc32` 正好 `= 0xFD10E5F5`）
- **设备算 CRC 用的是刚收到的 `data` 缓冲区，不是从 Flash 回读的** ⇒ Flash 写坏不可能是原因，问题在"收"这一侧

**根因**（`User/ota/ota_net.c`）：`parse_frame()` 把 `out->data` 指向**接收缓冲内部**：

```c
out->data = &p[OTA_FRAME_HEADER_SIZE];      /* p = &g_rx_buf[start]  ← 指向缓冲内部 */
```

而 `ota_net_next_frame()` 在 **返回 1 之前**就把缓冲尾部前移压缩了：

```c
uint32_t consumed = pos + (uint32_t)rc;              /* 帧长 1024 时 = 1035 */
if (consumed < g_rx_len) {
    memmove(g_rx_buf, &g_rx_buf[consumed], g_rx_len - consumed);   /* 搬到 [0, ...) */
}
g_rx_len -= consumed;
return 1;                                            /* 调用方这才去读 f->data */
```

载荷区 `g_rx_buf[start+9, start+9+len)` **正好落在 memmove 的目标前缀 `[0, consumed)` 里面** —— 帧被"取走"的同时，它的数据被**后面那帧**的字节覆盖了。

**为什么损坏形态每次都不一样**：覆盖多少取决于当时缓冲里积压了多少字节。

| 缓冲状态 | 结果 |
|---|---|
| 只有 1 帧（`g_rx_len == consumed`） | 不触发 memmove，**正常** |
| 2 整帧（`g_rx_len = 2070`） | 第一帧载荷被第二帧**整个**覆盖 |
| 第二帧只到齐一部分（`g_rx_len = 1500`） | 前 456 字节是第二帧的、后 568 字节才是自己的 —— **混合态** |

所以在 PC 上拿"整帧重复 / 整帧丢失 / 相邻互换 / 整体错位"去穷举，**一条都匹配不上** —— 真实损坏是混合态。10 MB/s 的推送下 TCP 必然粘连，于是每次 OTA 都错，且每次错得都不一样。

**还有一个更隐蔽的点**：`ota_net.h` 里原本写着

```c
 * ⚠ data 指向 ota_net.c 内部的接收缓冲, 只在本次处理期间有效,
 *   下一次 ota_net_recv() / ota_net_rx_reset() 之后内容即被覆盖。
```

**这句注释是错的** —— 实现是在 `ota_net_next_frame()` **返回之前**就覆盖了，比注释声明的失效点早得多。文档和实现不一致，把这类 bug 藏了很久。

**修法**：给载荷一个**不会被搬动**的落点，`out->data` 不再指向接收缓冲内部：

```c
static uint8_t g_frame_data[OTA_MAX_PAYLOAD];   /* 独立暂存区 */

/* parse_frame() 内: 先拷出来, 再交出去 */
memcpy(g_frame_data, &p[OTA_FRAME_HEADER_SIZE], len);
out->data = g_frame_data;
```

代价是每帧一次 1024 字节 memcpy（109 帧共 ~109 KB，可忽略），缓冲是 static 不占任务栈。反汇编复核：

```
0x080173d8:  BL   ___aeabi_memcpy8$move             memcpy(g_frame_data, &p[9], len)
0x080173de:  STR  r0,[r4,#8]                        out->data = g_frame_data   ★
```

**教训**：**别把"指向缓冲内部的指针"交给调用方，同时又在返回前搬动那块缓冲。** 要么给调用方一份拷贝，要么改成"读游标"式设计（只推进读下标、不搬数据），要么在文档里把失效点写清楚并严格遵守。这件事里三者都没做到 —— 而它表现为"长度校验全过、只有内容不对"，是 OTA 里最难查的一类症状。

---

## 9. IAP（引导程序）日志的取舍

### 9.1 改了什么

| 项 | 改前 | 改后 |
|---|---|---|
| 调试串口 | USART1 (PA9/PA10) | **USART6 (PC6/PC7) + PC8 方向脚** —— 与 APP 同一个口 |
| 串口初始化 | `DEBUG_USART_Config()` 被注释掉，**从不执行** | 放开 |
| 启动日志 | 无（全程静默） | 打 `boot_info` 内容、目标槽位、校验结果、是否回滚 |
| 发送超时 | 1000 ms/字节 | **10 ms/字节**，且失败一次就永久放弃 |

### 9.2 为什么 `fputc` 要加"防卡死"

`HAL_UART_Transmit()` 是**逐字节阻塞**发送，每个字节最多阻塞 `timeout` 毫秒。原来 fputc 写的是 1000 ms —— 万一串口没配好（接线不对、时钟没使能、波特率算错），一行 80 字符的日志最长能把启动卡住 **80 秒**。引导程序卡住 = 板子起不来，而它本来只是想说句话。

改成 10 ms 且第一次失败后置 `tx_dead` 标志彻底放弃后，最坏情况只耽误 10 ms 然后彻底闭嘴。**日志的价值必须远小于它可能造成的伤害。**

IAP 固件体积 23324 字节，引导区 32 KB，还剩约 9.4 KB —— 加日志的代价可以忽略，但这条余量以后要留意（Keil 的 IROM1 写的是 0x100000=1 MB，超了 32 KB **不会报错**，得自己盯）。

### 9.3 ⚠ IAP 侧源文件是 GBK，别用 UTF-8 工具改

| | APP 侧 `User/ota/*` | IAP 侧 `User/*` |
|---|---|---|
| 编码 | **UTF-8** | **GBK** |

两边的中文注释都是中文，但**字节编码不同**。用按 UTF-8 处理文本的编辑器/脚本去改 IAP 文件，会把里面的 GBK 中文注释写成乱码（虽然通常仍能编译，因为注释不影响语义，但注释就废了）。

本次改 IAP 用的是**字节级替换**：`open(p,'rb')` → `decode('latin-1')`（每个字节一个字符，完全无损）→ 替换 ASCII 片段、中文先 `encode('gbk')` 再嵌进去 → `encode('latin-1')` 写回。改完逐个文件 `decode('gbk')` 复验通过。

同理，`Project/build_ota.bat` 必须是**纯 ASCII**（见 8.2）。

---

## 10. 出厂烧写（产线 J-Link）→ OTA 升级 —— 完整操作流程

### 10.1 出厂状态：Flash 里应该有什么

| 区域 | 地址范围 | 大小 | 出厂内容 | 烧写来源 |
|---|---|---|---|---|
| Bootloader | `0x08000000`–`0x08007FFF` | 32 KB | ✅ **必须烧** | IAP 工程产物 |
| APP1（A 区） | `0x08008000`–`0x0807FFFF` | 480 KB | ✅ **必须烧** | APP 工程 `LwIP_A` 产物 |
| APP2（B 区） | `0x08080000`–`0x080DFFFF` | 384 KB | ⬜ **保持空白** | 第一次 OTA 才写 |
| BootInfo | `0x080E0000`–`0x080FFFFF` | 128 KB | ⬜ **保持空白** | 首次上电由 IAP 自动写默认值 |

**APP2 和 BootInfo 出厂不烧**：BootInfo 是 `0xFFFFFFFF`（全擦状态）时 magic 不匹配，IAP 会自动写入一份默认值（`boot_app=A`），这是设计好的首次上电流程，不需要产线干预。

### 10.2 产线 J-Link 烧写步骤

**前置检查（只需做一次）**

1. **用 `LwIP_A` 这个 Target**。工程里 J-Link 已经配好了，但**只配在 `LwIP_A` 上**；`LwIP_B` 的调试器选项是空的（`nTsel=-1`），选 B 点下载是烧不了的。产线一律用 A。
   - 配置位置：Options for Target → Debug → 右侧下拉应显示 `J-LINK / J-TRACE Cortex`
2. **⚠ 确认擦除方式是 "Erase Sectors"，不是 "Erase Full Chip"**
   - 位置：Options for Target → **Utilities** → Settings → **Download Function**
   - 为什么关键：APP 工程的下载地址范围写的是 `0x08000000` 起、1 MB 全片。如果选了 **Erase Full Chip**，单独烧 APP 会把**扇区 0–1 的 Bootloader 一起擦掉**，板子再也起不来。
   - 选 **Erase Sectors** 时，只有下载数据覆盖到的扇区会被擦（A 镜像从扇区 2 开始，Bootloader 在扇区 0–1，**安全**）。
   - ⚠ 工程文件里有个 `-FO15` 擦除参数，但它的具体含义**无法从文件可靠判断** —— 这一步请务必在 GUI 里亲眼确认，不要假定。
   - 顺手勾上 **Verify**。

**烧写（两种做法，任选）**

**做法一：全片擦除 + 依次烧两个 hex（推荐，产线最干净）**

产线本来就应该从干净片开始，所以直接全片擦除反而最省事、最确定：

| 步 | 操作 |
|---|---|
| 1 | J-Link 连上板子（SWD：SWCLK / SWDIO / GND / 3V3） |
| 2 | 全片擦除（Erase Full Chip） |
| 3 | 烧 Bootloader：打开 IAP 工程 → 下载 → 只写 `0x08000000`–`0x08007FFF` |
| 4 | 烧 A 镜像：打开 APP 工程、Target 选 `LwIP_A` → 下载 → 只写 `0x08008000` 起 |
| 5 | 上电，按 10.3 自检 |

**做法二：把两个 hex 合成一个，一次烧完（产线推荐）**

产线只烧**一个文件**，不用连线、不用切 Target、不用管烧到哪个地址：

```
Output\ota\YH-F429_v<版本>_production.hex
```

这个文件由 `Project\merge_hex.py` 生成，内容是 **Bootloader（`0x08000000`）+ A 镜像（`0x08008000`）**。
`build_ota.bat` 第 5 步会**自动**跑它，所以正常发版流程下你什么都不用做。

**什么时候需要手动合成**

只在"hex 已经编译好了，但不想再跑一遍完整编译"时（例如别人给了你一个 A 镜像的 hex，或者你只想重新合成一次）：

```bat
cd /d <仓库根目录>\Project
python merge_hex.py -o ..\Output\ota\YH-F429_v1.0.0.5_production.hex
```

- `-o` 是**相对当前目录**的，所以要么先 `cd` 进 `Project\`，要么给绝对路径。
- Bootloader 和 A 镜像的路径**不用传** —— 脚本里写好了默认值，而且是**相对脚本自己**算的，在哪个目录下执行都能找到。
- 只需要 Python 3（**只用标准库**），不需要装任何第三方包。

改输出文件名以外的参数（一般用不到）：

```bat
python merge_hex.py --boot <Bootloader.hex> --app <A镜像.hex> -o <输出.hex>
```

**脚本替你挡掉了什么**

| 检查 | 不过会怎样 |
|---|---|
| 两个镜像地址**重叠** | 报错退出。重叠意味着合并会静默改写内容，通常是 A 镜像的链接地址（`IROM1`）或 IAP 的 `APP1_ADDR` 配错了 |
| 数据**越出分区**（越过 `0x08080000`、或顶到 BootInfo 的 `0x080E0000`） | 报错退出 |
| 中断向量表（初始 MSP / 复位向量）不像话 | 只告警，不拦 —— 但烧下去就是块砖，看到就得查 |
| 写出后**回读逐字节比对** | 不一致就报错。这是防"写 hex 时跨 64KB 边界漏掉几个字节"那类静默错误 —— 那种镜像烧进去能跑，但会在某个想不到的时刻出错 |

**任何一项不过，脚本都不落盘** —— 宁可没有文件，也不要一个坏文件。

**合成对了长什么样**

```
[1/4] 读取
      Bootloader: ...\iap代码\...\Output\YH-F429.hex
                 0x08000000..0x08005B43  23364 字节 (含空洞)
     向量表: MSP=0x20009910  Reset=0x0800028D  (OK)
      APP       : ..\Output\A\YH-F429_A.hex
                 0x08008000..0x0802328F  111248 字节 (含空洞)
     向量表: MSP=0x2002B338  Reset=0x0800828D  (OK)
[2/4] 检查地址重叠
      Bootloader 0x08000000..0x08005B43  APP 0x08008000..0x0802328F  无重叠
[3/4] 合并
[4/4] 写出
      ..\Output\ota\YH-F429_v1.0.0.5_production.hex
      回读自检 = PASS  (134612 字节逐字节一致)
```

**判定合成成功的唯一依据是 `回读自检 = PASS`**，且括号里的字节数应等于 `Bootloader 字节数 + A 镜像字节数`（上例 `23364 + 111248 = 134612`）。**看不到 `PASS` 就别拿去烧。**

**⚠ 三个最容易踩的坑**

1. **改完源码必须先重编，否则合成的是旧镜像。**
   `merge_hex.py` 不会替你编译，它只是把磁盘上现成的 `Output\A\YH-F429_A.hex` 拿过来拼。
   如果你改了源码却直接跑 `merge_hex.py`，拿到的是**上一次编译**的程序 —— 而且文件名、版本号、CRC 全都自洽，**从文件上看不出任何问题**。
   所以顺序永远是：**改源码 → 跑 `build_ota.bat`（重编 + 自动合成）**；只有在确实不想重编时，才单独手动跑 `merge_hex.py`。
2. **文件名的版本号来自 `ota_config.h`，不是脚本里写的。**
   `build_ota.bat` 自动读 `FW_VERSION_*` 拼名字；手动跑 `merge_hex.py` 时版本号是**你自己在 `-o` 里敲的**，脚本不会校验它和固件里上报的版本是否一致 —— 敲错了不会有任何提示。
3. **产线烧的这个 hex 里的 A 镜像，必须和给上位机 OTA 的 `_A.bin` 是同一份。**
   否则出厂固件和 OTA 镜像内容不一致，A/B 版本比对会出岔子。同一轮 `build_ota.bat` 出来的两个文件天然一致；如果 A 镜像换过，两个都要换。

**为什么只烧 A、不烧 B 和 BootInfo**

见 10.1 的表格：B 区保持空白，第一次 OTA 才写；BootInfo 保持空白，首次上电由 IAP 自动写默认值（`boot_app=A`）并打印 `[IAP] boot_info missing, default slot A written`。

**烧写文件清单**

| 要烧的东西 | 文件 | 实际大小 |
|---|---|---|
| **合成版（产线用这个，只烧一个）** | `Output/ota/YH-F429_v<版本>_production.hex` | 131 KB |
| Bootloader（做法一分别烧时用） | `ota升级需要的iap代码/lwip_client_Bootloader_RTOS_Final（完成版别动了）/Output/YH-F429.hex` | 23 KB |
| A 镜像（做法一分别烧时用） | `Output/A/YH-F429_A.hex` | 109 KB |
| （参考）B 镜像 | `Output/B/YH-F429_B.hex` | 109 KB |

> 表里的"实际大小"是**真正烧进 Flash 的字节数**（也就是 `_A.bin` 的大小），不是 `.hex` 文件的磁盘大小。`.hex` 自身要大 3 倍左右 —— 它是 ASCII 文本，1 个字节写成 2 个十六进制字符，再加地址和校验字段。所以 `YH-F429_production.hex` 在资源管理器里显示 370 KB、而实占 Flash 只有 131 KB，是正常的，**不是固件变大了**。

> 用 `.hex` 而不是 `.bin`：hex 里带绝对地址，烧写工具不会放错位置。用 `.bin` 的话必须手工指定地址（A 镜像 → `0x08008000`）。

### 10.3 首次上电自检（预期日志）

```
===== OTA Bootloader =====
[IAP] boot_info invalid (magic=0xFFFFFFFF)
[IAP] boot_info missing, default slot A written
[IAP] boot_info asks for slot A (0x08008000, 480KB)
[IAP] slot A image check: OK (rc=0)
[IAP] jumping to slot A @ 0x08008000 ...

[OTA] running slot = A (source: VTOR), VTOR=0x08008000
[OTA] current ver v1.0.0.1, target slot B (0x08080000, 384KB)
[OTA] boot_info: boot_app=A ver=v0.0.0.0 size=0 crc=0x00000000
```

看到这 8 行，说明出厂状态正确，可以直接进 OTA 了。

> `ver=v0.0.0.0` 是**正常的** —— 那是 IAP 刚写的默认 BootInfo 里的 `version` 字段（默认 0）。**判据永远是 `[OTA] current ver`**，它反映的是正在运行的镜像的编译期版本。

**故障对照**

| 现象 | 原因 |
|---|---|
| `[IAP] slot A image check:` 后面不是 `OK`（如 `bad stack pointer` / `reset vector outside slot` / `size/CRC mismatch`） | A 镜像没烧、或烧错地址（必须烧到 `0x08008000`） |
| `[IAP] FATAL: neither slot bootable, blink forever` + 全部 485 LED 轮流闪 | 两个槽都没有效镜像 —— 回查第 3、4 步 |
| 完全没有任何日志 | 串口线/RS485 方向脚问题（见 7.3），或 Bootloader 没烧进去 |
| 跳到 A 之后就没日志了 | A 镜像链接地址不对 —— 用 6.1 的脚本验复位向量 |

### 10.4 ⚠ 单独更新 APP 时的禁忌（售后 / 开发机）

**已出厂的板子只单独重烧 APP 时，擦除方式必须是 "Erase Sectors"。**

这一条和第 10.2 的"全片擦除"不矛盾：
- **产线**：Bootloader 和 APP 一起烧，全片擦除没问题（两个都重写）
- **售后/开发**：板子上已经有 Bootloader 了，此时全片擦除会把它擦掉，板子变砖（只能重新连调试器救）

### 10.5 发一个新版本

| 步 | 操作 |
|---|---|
| 1 | 改 `User/ota/ota_config.h` 里的 `FW_VERSION_BUILD`（或 MAJOR/MINOR）——**只改这一处** |
| 2 | 双击 `Project/build_ota.bat` —— 一次得到**三个**文件 |
| 3 | 把 **`_A.bin` 和 `_B.bin` 两个文件**拷到上位机同一个文件夹 |
| 4 | `_production.hex` 是给**产线**的（空片 / 整机返修才用得到）；普通的 OTA 升级用不上它 |

`build_ota.bat` 第 2 步的三份产物，各给谁：

| 产物 | 给谁 | 干什么用 |
|---|---|---|
| `YH-F429_v<版本>_A.bin` | 上位机 `OTA_Upgrade_Tool.exe` | OTA 升级（A 槽镜像） |
| `YH-F429_v<版本>_B.bin` | 上位机 `OTA_Upgrade_Tool.exe` | OTA 升级（B 槽镜像） |
| `YH-F429_v<版本>_production.hex` | 产线 J-Link | 出厂一次烧完（Bootloader + A 镜像，见 10.2 做法二） |

⚠ 版本号**必须比板上正在跑的版本大**，否则设备回 `REJECT_UPDATE`，什么都不发生（设计如此）。

### 10.6 OTA 升级步骤

| 步 | 操作 |
|---|---|
| 1 | PC 有线网卡配 **192.168.1.99 / 255.255.255.0**（固件里地址固定死了，别改固件） |
| 2 | 网线直连通信板，`ping 192.168.1.99` 确认网卡起来了 |
| 3 | 板子上电，确认当前跑的是**旧版本**（看 `[OTA] current ver`） |
| 4 | 跑 `PC_Server\ota_gui.py`，把 A、B 两个 bin 放**同一个文件夹**，选中任意一个 |
| 5 | 点"开始升级"，等进度 100% + CRC 校验通过 |
| 6 | 板子自动重启 → IAP 读 `boot_info` → 跳到新分区 |

**第 4 步为什么选哪个都行**：上位机从设备发来的 HELLO 帧里读到设备**当前在哪个槽**，然后自动挑"另一个槽"对应的镜像推送过去。所以 A、B 两个文件必须在同一目录，工具会自动配对。

**升级过程中的串口日志**（关键几行）：

```
[OTA] PUSH_UPDATE: ver=v1.0.0.2 size=111596 crc=0x... (current v1.0.0.1, current slot A)
[OTA] accept upgrade: write slot B @0x08080000
[OTA] erase 0x08080000 +128KB (only needed sectors)...
[OTA] transfer end: received 111596 / declared 111596 bytes
[OTA] CRC32: got 0x..., expected 0x...
[OTA] verify OK
[OTA] BootInfo updated: next boot -> B slot, ver=v1.0.0.2, 111596 bytes
[OTA_CLIENT] ready, reboot in ...ms
```

重启后应看到 `[IAP] jumping to slot B` 和 `[OTA] current ver v1.0.0.2`。

### 10.7 A/B 乒乓是怎么翻面的

设备**永远写"当前没在跑"的那个槽**，运行中的分区自始至终不被触碰，所以回滚是天然的。

| 升级前运行 | 新固件写进 | 重启后运行 | 下次升级写进 |
|---|---|---|---|
| A（APP1） | **B（APP2）** | B | A |
| B（APP2） | **A（APP1）** | A | B |

**验证升级真的成功**：只看 `[OTA] current ver` 是不是新版本（完整判据见 7.3）。上位机的 100% 只代表传输 + CRC 通过，**不等于设备跑起来了**。

**顺手验一条回滚**：升级中途给板子断电，重新上电应仍在**旧版本**正常跑（走的是 10.3 那种正常路径，不是回滚路径）。
