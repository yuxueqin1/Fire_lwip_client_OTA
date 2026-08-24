# CommBoard_OutputModule 进程概述

## 一、整体定位

这是一个运行在 **RK3568 ARM64 Linux 通信板** 上的**输出控制与报警管理进程**。它位于“通信板”和“下位采集/输出模块”之间，充当 **Modbus 主站（Master）**，通过串口轮询和控制下位模块的 IO 状态；同时通过 **Unix 域套接字（IPC）** 与同一板卡上的另一个进程（文档中称为“进程A”）通信，接收报警状态和管理命令。

这是**铁路信号系统**（上海铁大）中的通信板软件组件，涉及切断开关、断接开关、蜂鸣器、报警灯等铁路信号领域的典型 IO 信号。

---

## 二、代码架构（7个模块，约1000行 C++17）

```
main.cpp          —— 入口：解析配置 → 初始化各组件 → 启动主循环
Config            —— INI 配置文件解析（支持多 Section）
SerialPort        —— Linux termios 串口封装（非阻塞 + select 超时）
ModbusMaster      —— Modbus RTU 主站协议（功能码 01/02/05/0F/10）
DataModel         —— 线程安全的数据模型（DI 输入 + 线圈输出）
BusinessLogic    —— 核心业务逻辑（轮询、报警判决、输出修正、IPC 收发）
SocketIPC         —— Unix 域套接字通信（支持服务端/客户端双模式）
crc_utils         —— CRC-16 校验工具
```

---

## 三、启动流程（main.cpp）

1. **注册信号处理**：`SIGINT`（Ctrl+C）和 `SIGTERM`（kill），安全退出
2. **加载配置文件**（`configs/comConfig.ini` → `[SerialSet]` 节）：

| 配置项 | 默认值 | 含义 |
|---|---|---|
| `serial_port` | `/dev/ttyEXT11` | 连接下位模块的串口 |
| `baudrate` | `9600` | 波特率 |
| `slave_address` | `4` | 下位模块的 Modbus 从站地址 |
| `di_start_addr` | `0` | 离散输入起始地址 |
| `di_quantity` | `4` | 离散输入数量 |
| `coil_start_addr` | `0` | 线圈起始地址 |
| `coil_quantity` | `4` | 线圈数量 |
| `poll_interval_ms` | `1000` | 轮询间隔（毫秒） |
| `ipc_client_path` | `/tmp/modbus_status.sock` | 上报状态的目标 socket |
| `ipc_server_path` | `/tmp/modbus_cmd.sock` | 接收命令的监听 socket |

3. **打开串口** → 创建 `ModbusMaster` → 创建 `DataModel` → 创建 `BusinessLogic`
4. **建立 IPC 连接**：作为客户端连接 `/tmp/modbus_status.sock`（上报），作为服务端监听 `/tmp/modbus_cmd.sock`（接收命令）
5. **进入主循环** `logic.runLoop()`

---

## 四、主循环逻辑（BusinessLogic::runLoop()）

每隔 `poll_interval_ms`（默认 1000ms）执行一轮：

```
┌──────────────────────────────────────────────────┐
│  1. 02 功能码：读取下位模块的离散输入（DI）       │
│     → 更新 DataModel 中的输入状态量              │
│                                                  │
│  2. 01 功能码：读取下位模块的线圈状态（DO）       │
│     → 更新 DataModel 中的输出状态量              │
│                                                  │
│  3. model_.printAll()  （打印当前所有 IO 状态）   │
│                                                  │
│  4. checkAndCorrectOutputs()                    │
│     → 根据业务逻辑计算"应有输出"                  │
│     → 与实际输出比对，不一致则用 05 功能码修正    │
│                                                  │
│  5. sleep(poll_interval_ms) ，分 100ms 小段睡    │
│     （便于快速响应退出信号）                      │
└──────────────────────────────────────────────────┘
```

---

## 五、核心业务逻辑：报警判决（computeDesiredOutputs()）

这是整个进程最核心的算法，对应报警逻辑真值表。

### 5.1 输入信号（DI）

| 地址 | 名称 | 含义 |
|---|---|---|
| DI0 | 切断开关（cut） | 切断按钮状态 |
| DI1 | 断接开关（break） | 断接开关状态 |

### 5.2 输入信号（来自 IPC）

| 来源 | 含义 |
|---|---|
| `alarm_status_` | 进程A通过 IPC 发来的“其他板卡报警”标志 |
| `last_status_time_` | 上次收到报警状态的时间戳 |

### 5.3 输出信号（DO）

| 地址 | 名称 | 控制逻辑 |
|---|---|---|
| DO0 | 工作灯（通信状态灯） | `time_ok`（15s 内有收到 IPC 状态更新就亮） |
| DO1 | 蜂鸣器 | `(alarm \|\| break) XOR cut` |
| DO2 | 报警灯 | `alarm \|\| break \|\| cut`（三选一即亮） |

### 5.4 关键设计

- **time_ok 机制**：如果超过 **15 秒**没有收到进程A发来的状态报警数据包，则认为通信超时，此时：
  - 工作灯熄灭（DO0 = OFF）
  - 报警标志被强制清零（超时不报警，避免误报）
- **蜂鸣器采用异或逻辑**：`(alarm \|\| break) XOR cut`，即切断开关可以抑制蜂鸣器

### 5.5 报警真值表

| alarm | break | cut | 蜂鸣器 | 报警灯 |
|-------|-------|-----|--------|--------|
| 0 | 0 | 0 | 0 | 0 |
| 0 | 0 | 1 | 1 | 1 |
| 0 | 1 | 0 | 1 | 1 |
| 0 | 1 | 1 | 0 | 1 |
| 1 | 0 | 0 | 1 | 1 |
| 1 | 0 | 1 | 0 | 1 |
| 1 | 1 | 0 | 1 | 1 |
| 1 | 1 | 1 | 0 | 1 |

### 5.6 输出修正

`checkAndCorrectOutputs()` 比较“应有状态”与“实际状态”，不一致时通过 Modbus **05 功能码（写单个线圈）** 下发修正命令给下位模块。

---

## 六、IPC 通信机制（SocketIPC）

使用 **Unix 域套接字（AF_UNIX + SOCK_STREAM）**，同时扮演两个角色：

| 角色 | 路径 | 方向 |
|---|---|---|
| IPC 客户端 | `/tmp/modbus_status.sock` | **上行**：向进程A上报 DI 输入状态 |
| IPC 服务端 | `/tmp/modbus_cmd.sock` | **下行**：接收进程A发来的命令 |

### 6.1 上行上报（sendInputStatus()）

- 上报格式：4字节长度（网络序）+ N 个字节（每个 DI 的值）
- 支持断线重连

### 6.2 下行命令接收（onReceiveCommand()）

反序列化 `IPCPacket`（instance_id + data_type + data 负载），根据 `data_type` 分发处理：

| data_type | 处理函数 | 用途 |
|---|---|---|
| `"modbus"` | `handleModbusCommand()` | 透传 Modbus 命令（目前标记为“暂未实现”） |
| `"io_output"` | `handleIoOutputCommand()` | IO 输出控制（解析 port+value） |
| `"status"` | `handleStatusCommand()` | **报警状态同步**——最核心的 IPC 功能 |

### 6.3 唤醒机制

使用 `pipe()` 创建的唤醒管道，在 `poll()` 阻塞等待时可以通过 `wakeup()` 方法写入一个字节来唤醒线程，实现安全退出。

---

## 七、报警状态协议（handleStatusCommand()）

这是 IPC 通信中最关键的数据包，由进程A发送：

```
┌─────────────┬──────────┬───────────────┬──────────┐
│ 帧头(4B)    │ 板卡数(1B)│ 板卡状态(N×8B) │ CRC-16(2B)│
│ 55 AA 55 AA │ count    │ 每板8字节     │          │
└─────────────┴──────────┴───────────────┴──────────┘
```

每块板卡的 8 字节状态：

| 字节 | 含义 |
|---|---|
| byte[0] | 板卡 ID |
| byte[1] | valid 标志（0x00=有效，非0=无效/不在位） |
| byte[2-3] | 协议1 的通信状态 + SPD状态 |
| byte[4-5] | 协议2 的通信状态 + SPD状态 |
| byte[6-7] | 协议3 的通信状态 + SPD状态 |

### 判决逻辑

- 对于每块 valid=0（有效在位）的板卡，检查 3 个协议的 `通信状态` 和 `SPD状态` 是否全为 0
- 任一协议的通信状态或 SPD 状态不为 0 → `any_alarm = true`
- 更新 `alarm_status_` 并刷新 `last_status_time_`
- 带有 **CRC-16 校验**保证数据完整性

---

## 八、Modbus 通信层（ModbusMaster）

完整实现了 Modbus RTU 主站的 5 个功能码：

| 功能码 | 方法 | 用途 |
|---|---|---|
| **01** | `readCoils()` | 读线圈（输出状态量 DO） |
| **02** | `readDiscreteInputs()` | 读离散输入（DI） |
| **05** | `writeSingleCoil()` | 写单个线圈 |
| **0F** | `writeMultipleCoils()` | 写多个线圈 |
| **10** | `writeMultipleRegisters()` | 写多个保持寄存器 |

### 通信保障措施

- 每个请求/响应都带有 **CRC-16 校验**
- 每次发送前 **清空串口缓冲区**（`tcflush`）
- `readFullResponse()` 循环接收直到收够完整帧或超时 500ms
- 响应的地址、功能码、起始地址、数据长度全部校验

---

## 九、串口层（SerialPort）

- 基于 Linux `termios` API
- **8N1**（8数据位、无校验、1停止位）
- **原始模式**（raw mode），无软件/硬件流控
- 使用 `select()` 实现带超时的非阻塞读取
- 支持 `readExact()` 精确读取指定字节数，支持 stop_flag 中断

---

## 十、编译部署（makefile）

| 目标 | 命令 | 说明 |
|---|---|---|
| 本地编译 | `make` | 输出 `build/local/ComBoardOutputModule`（x86_64） |
| 交叉编译 | `make cross` | 输出 `build/arm64/ComBoardOutputModule_arm64`（RK3568 Cortex-A55） |
| 静态编译 | `make static-cross` | 完全静态链接，适合嵌入式部署 |
| 清理 | `make clean` | 删除 build 目录 |

编译后自动 `strip` 精简二进制。

---

## 十一、总体数据流图

```
                    ┌──────────────────────────┐
                    │     进程A（另一个进程）    │
                    │   （主控/管理进程）        │
                    └─────┬──────────┬─────────┘
                          │          │
              上报 DI 状态 │          │ 下发报警状态/命令
            /tmp/modbus_   │          │ /tmp/modbus_
            status.sock    │          │ cmd.sock
                          │          │
                    ┌─────▼──────────▼─────────┐
                    │   CommBoard_OutputModule  │
                    │                           │
                    │  ┌─────────────────────┐  │
                    │  │ BusinessLogic       │  │
                    │  │ - 报警真值表判决    │  │
                    │  │ - 输出状态修正      │  │
                    │  │ - IPC 收发处理      │  │
                    │  └────────┬───────────┘  │
                    │           │               │
                    │  ┌────────▼───────────┐  │
                    │  │ ModbusMaster (RTU) │  │
                    │  │ 01/02/05/0F/10     │  │
                    │  └────────┬───────────┘  │
                    │           │               │
                    │  ┌────────▼───────────┐  │
                    │  │ SerialPort         │  │
                    │  │ /dev/ttyEXT11 @9600│  │
                    │  └────────┬───────────┘  │
                    └───────────┼───────────────┘
                                │ RS-485/232
                                │ Modbus RTU
                    ┌───────────▼───────────────┐
                    │   下位采集/输出模块         │
                    │   (从站地址=4)             │
                    │   DI: 切断开关、断接开关等  │
                    │   DO: 蜂鸣器、报警灯、工作灯│
                    └───────────────────────────┘
```

---

## 十二、总结

**CommBoard_OutputModule** 是铁路信号通信板上的一个**中间层进程**：

1. **向下**通过 Modbus RTU 串口轮询下位模块的 DI 状态（切断开关、断接开关），并根据业务逻辑自动控制 DO 输出（蜂鸣器、报警灯、工作灯）
2. **向上**通过 Unix 域套接字与同板的另一个管理进程通信，接收来自其他板卡的报警状态，参与多板卡联合报警判决
3. 核心是一个**闭环控制**：检测 → 判断 → 修正，确保输出状态始终符合业务逻辑预期
4. 15 秒 IPC 超时保护机制确保在进程间通信中断时不会产生误报警
