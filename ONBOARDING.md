# Welcome to 上海铁大 通信板团队

## How We Use Claude

Based on 2938458559's usage over the last 30 days:

Work Type Breakdown:
  Debug / Fix            ████████████░░░░░░░░  60%

  Build Feature          ██████░░░░░░░░░░░░░░  25%

  Improve Quality        ██░░░░░░░░░░░░░░░░░░  10%

  Plan / Design          █░░░░░░░░░░░░░░░░░░░   5%

Top Skills & Commands:
  /compact               ████████████████████  8x/month

  /model                 ████████░░░░░░░░░░░░  3x/month

  /debug                 █████░░░░░░░░░░░░░░░  2x/month
  
  /config                ███░░░░░░░░░░░░░░░░░  1x/month

---

# 项目入门文档：STM32F429 6路串口通信板

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构](#2-系统架构)
3. [硬件平台](#3-硬件平台)
4. [软件技术栈](#4-软件技术栈)
5. [目录结构](#5-目录结构)
6. [嵌入式固件详解](#6-嵌入式固件详解)
7. [上位机软件详解](#7-上位机软件详解)
8. [通信协议](#8-通信协议)
9. [从机模拟器](#9-从机模拟器)
10. [OTA 固件升级](#10-ota-固件升级)
11. [开发环境搭建](#11-开发环境搭建)
12. [调试与日志](#12-调试与日志)
13. [常见问题](#13-常见问题)

---

## 1. 项目概述

### 1.1 项目定位

本项目是一套**铁路信号系统通信板**解决方案，用于铁路信号设备（信号机、轨道电路、道岔转辙机等）的数据采集与远程监控。

**核心功能：**
- 6 路独立 RS-485 串口同时采集现场设备数据
- Modbus RTU 主站模式：轮询标准 Modbus 从机（信号机、轨道接收/发送器、采集模块等）
- 自定义曲线协议：采集转辙机动作曲线（交流/直流四线/直流六线）
- 通过以太网（lwIP TCP）将数据实时上传到上位机服务器
- 上位机远程配置下发、校时、软复位、心跳保活
- 看门狗系统保证设备异常自动恢复
- OTA 固件远程升级（开发中）

### 1.2 系统拓扑

```
┌─────────────────────────────────────────────────────────────────┐
│                        上位机 (PC Server)                        │
│  board_server.py  (TCP Server :5001, tkinter GUI)               │
│  ├─ 配置下发  ├─ 校时  ├─ 心跳  ├─ 复位  ├─ 日志分析           │
└──────────────────────────┬──────────────────────────────────────┘
                           │ TCP/IP (Ethernet)
┌──────────────────────────┴──────────────────────────────────────┐
│              通信板 (STM32F429IGTx + lwIP + FreeRTOS)            │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  TCP Client (client.c)  —  接收命令 / 发送数据              │ │
│  │  OTA Client (ota_client.c)  —  固件远程升级 (开发中)        │ │
│  └────────────────────────────────────────────────────────────┘ │
│                           │ 6路串口 (RS-485)                     │
│  ┌────────┬────────┬────────┬────────┬────────┬────────┐       │
│  │USART1  │USART2  │USART3  │ UART4  │ UART5  │ USART6 │       │
│  │Modbus  │Modbus  │Custom  │Custom  │Custom  │Custom  │       │
│  │Master  │Master  │Curve   │Curve   │Curve   │Curve   │       │
│  └───┬────┴───┬────┴───┬────┴───┬────┴───┬────┴───┬────┘       │
└──────┼────────┼────────┼────────┼────────┼────────┼─────────────┘
       │        │        │        │        │        │
  ┌────┴──┐ ┌──┴──┐ ┌──┴──┐ ┌──┴──┐ ┌──┴──┐ ┌──┴──┐
  │从机1-N│ │从机 │ │曲线 │ │曲线 │ │曲线 │ │曲线 │
  │Modbus │ │1-13 │ │模块 │ │模块 │ │模块 │ │模块 │
  │设备   │ │     │ │0x37 │ │0x38 │ │0x39 │ │...  │
  └───────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘
```

---

## 2. 系统架构

### 2.1 嵌入式固件架构

固件基于 **FreeRTOS 多任务** 设计，任务优先级从上到下递减：

```
优先级 7  ┌─────────────────────────────────┐
         │  AppTaskCreate (用完即删)         │  ← 初始化所有子系统
         └─────────────────────────────────┘
优先级 6  ┌─────────────────────────────────┐
         │  tcpip_thread (lwIP 内核)        │  ← TCP/IP 协议栈处理
         └─────────────────────────────────┘
优先级 5  ┌─────────────────────────────────┐
         │  td_tcp_client (client.c)        │  ← TCP 连接管理 + 命令接收
         └─────────────────────────────────┘
优先级 4  ┌─────────────────────────────────┐
         │  App_Task_TcpSend                │  ← TCP 发送队列轮询 (发送分离)
         └─────────────────────────────────┘
优先级 3  ┌────┬────┬────┬────┬────┬────┬───┐
         │U1  │U2  │U3  │U4  │U5  │U6  │WDT│  ← 6路串口采集 + 看门狗管理
         └────┴────┴────┴────┴────┴────┴───┘
优先级 1  ┌─────────────────────────────────┐
         │  Test1_Task (LED 心跳)           │  ← 运行指示灯
         └─────────────────────────────────┘
```

**关键设计决策：**
- **收发分离**：TCP 接收（处理上位机命令）和 TCP 发送（上传采集数据）跑在不同任务中，命令不因大量采集数据而阻塞
- **网络优先**：tcpip_thread 和 client_thread 优先级高于串口采集任务，确保 TCP 协议栈不会饥饿
- **配置驱动的端口**：6 路串口由统一的 `uart_task_handler` 驱动，行为完全由 `BoardCfgTab` 配置决定

### 2.2 上位机软件架构

上位机 `board_server.py` 是一个 Python tkinter GUI 程序：

```
┌────────────────────────────────────────────┐
│          BoardServerGUI (主窗口)            │
│  ┌──────────────┐  ┌────────────────────┐  │
│  │  控制面板     │  │  通信日志 (彩色)    │  │
│  │  - 配置下发   │  │                    │  │
│  │  - 校时/心跳  │  │  [信息] 蓝色       │  │
│  │  - 软复位     │  │  [成功] 绿色       │  │
│  │  - 服务器控制 │  │  [警告] 橙色       │  │
│  └──────────────┘  │  [错误] 红色       │  │
│                    │  [数据] 黄色       │  │
│                    └────────────────────┘  │
│  ┌──────────────────────────────────────┐  │
│  │  TcpBoardServer (后台线程)            │  │
│  │  - socket 监听 :5001                 │  │
│  │  - 粘包/半帧处理                      │  │
│  │  - 事件队列 → GUI 线程安全更新        │  │
│  └──────────────────────────────────────┘  │
└────────────────────────────────────────────┘
```

---

## 3. 硬件平台

| 项目 | 规格 |
|------|------|
| **主控芯片** | STM32F429IGTx (Cortex-M4, 180MHz) |
| **开发板** | 野火 STM32F429 挑战者 |
| **以太网 PHY** | 板载以太网 (通过 RMII 接口) |
| **串口** | 6 路 USART/UART，通过 RS-485 收发器连接现场总线 |
| **EEPROM** | AT24C02 (2Kbit I2C) — 调试阶段；目标：AT24C64D (64Kbit) |
| **SDRAM** | 板载 SDRAM (用于曲线数据缓存) |
| **看门狗** | 独立看门狗 (IWDG)，软件喂狗 |

---

## 4. 软件技术栈

### 4.1 嵌入式侧

| 组件 | 版本/说明 |
|------|-----------|
| **OS** | FreeRTOS V9.0.0 |
| **TCP/IP** | lwIP (Lightweight IP)，netconn API |
| **HAL** | STM32F4xx HAL Driver |
| **编译器** | ARM Compiler 5 (Keil MDK) |
| **Modbus 主站** | FreeModbus (mbmaster) — 轮询从机输入寄存器 |
| **Modbus 从站** | FreeModbus (mbslave) — 供上位机通过 Modbus TCP 读写寄存器 |
| **调试** | CmBacktrace (ARM Cortex-M 故障回溯) |

### 4.2 上位机侧

| 组件 | 版本/说明 |
|------|-----------|
| **语言** | Python 3.6+ |
| **GUI** | tkinter (标准库) |
| **网络** | socket (标准库 TCP) |
| **序列化** | struct + json |

### 4.3 从机模拟器侧

| 组件 | 说明 |
|------|------|
| **语言** | Python 3.6+ |
| **串口** | pyserial |
| **GUI** | tkinter |
| **协议** | Modbus RTU (0x03/0x04) + 自定义曲线协议 |

---

## 5. 目录结构

```
Fire_lwip_client_OTA/
├── User/                          # ★ 应用层代码 (核心)
│   ├── main.c                     #   入口 + 任务创建 + UART 采集逻辑
│   ├── main.h                     #   类型定义 + Modbus 寄存器常量
│   ├── client.c                   #   lwIP TCP 客户端 (连接/接收/发送队列)
│   ├── client.h                   #   TCP 客户端接口
│   ├── board.c                    #   板级初始化 (时钟/I2C/LED/EEPROM)
│   ├── stm32f4xx_it.c/h           #   中断服务函数
│   ├── stm32f4xx_hal_conf.h       #   HAL 配置
│   ├── FreeRTOSConfig.h           #   FreeRTOS 配置
│   ├── system_stm32f4xx.c         #   系统初始化
│   ├── arch/                      #   lwIP 移植层
│   │   ├── sys_arch.c/h           #     lwIP OS 抽象层
│   │   ├── ethernetif.c/h         #     以太网接口驱动
│   │   ├── cc.h, lwipopts.h      #     lwIP 配置
│   │   └── perf.h                 #     性能计数器
│   ├── usart/                     #   串口驱动
│   │   ├── bsp_rs485.c/h          #     RS-485 收发控制 + 曲线帧处理
│   │   └── bsp_debug_usart.c/h    #     调试串口
│   ├── eth/                       #   以太网驱动
│   │   └── bsp_eth.c/h            #     ETH BSP 配置
│   ├── led/                       #   LED 驱动
│   │   └── bsp_led.c/h
│   ├── eeprom/                    #   EEPROM 驱动 (I2C)
│   │   └── bsp_eeprom.c/h
│   ├── sdram/                     #   SDRAM 驱动
│   │   └── bsp_sdram.c/h
│   ├── ota/                       #   OTA 固件升级模块 (开发中)
│   │   ├── ota_client.c/h         #     OTA 客户端核心
│   │   ├── ota_flash.c/h          #     内部 Flash 操作
│   │   ├── ota_net.c/h            #     OTA 网络传输
│   │   ├── ota_crc32.c/h          #     CRC32 校验
│   │   ├── ota_config.h           #     OTA 配置
│   │   └── ota_manager.c/h        #     OTA 升级管理
│   ├── cm_backtrace/              #   CmBacktrace 故障诊断库
│   │   ├── cm_backtrace.c/h       #     核心回溯
│   │   ├── cmb_cfg.h, cmb_def.h   #     配置
│   │   └── fault_handler/         #     硬件 fault 处理 (GCC/IAR/Keil)
│   └── debug.h                    #   调试宏 + 全局变量声明
│
├── FreeRTOS/                      # FreeRTOS V9.0.0 源码
│   ├── src/                       #   内核源码 (task/queue/timer/event...)
│   ├── include/                   #   内核头文件
│   └── port/                      #   移植层 (ARM_CM4F / MemMang)
│
├── LwIP/                          # lwIP 协议栈源码
│   ├── src/                       #   核心 (core/api/core/netif...)
│   └── include/                   #   头文件
│
├── Libraries/                     # STM32 官方库
│   ├── CMSIS/                     #   CMSIS-Core (M4)
│   └── STM32F4xx_HAL_Driver/      #   HAL 外设驱动库
│
├── mbmaster/                      # FreeModbus 主站协议栈
│   ├── rtu/                       #   RTU 模式 (帧编解码 + CRC)
│   ├── ascii/                     #   ASCII 模式
│   ├── tcp/                       #   TCP 模式
│   ├── functions/                 #   主站功能码实现
│   └── common/                    #   公共工具 (帧/CRC/类型)
│
├── mbslave/                       # FreeModbus 从站协议栈
│   ├── rtu/                       #   RTU 模式
│   ├── ascii/                     #   ASCII 模式
│   ├── tcp/                       #   TCP 模式
│   └── functions/                 #   从站功能码 (0x03/0x04 等)
│
├── mbport/                        # FreeModbus 移植层
│   ├── mbport.h                   #   移植接口
│   └── mbportevent.c              #   事件处理
│
├── Project/                       # Keil MDK 工程文件
│   ├── YH-F429.uvprojx            #   Keil 工程 (主)
│   ├── YH-F429.uvoptx             #   工程选项
│   └── DebugConfig/               #   调试配置
│
├── PC_Server/                     # ★ 上位机服务端 (Python)
│   ├── board_server.py            #   主程序 (TCP Server + GUI)
│   ├── board_config.json          #   板卡配置文件
│   ├── slave_simulator.py         #   从机模拟器启动器 (GUI/CLI)
│   └── modbus_slave_simulator/    #   从机模拟器包
│       ├── main.py                #     主程序 + Simulator 类
│       ├── gui.py                 #     tkinter GUI
│       ├── cli.py                 #     CLI 交互界面
│       ├── slave_device.py        #     从机设备模拟
│       ├── serial_io.py           #     串口读写
│       ├── data_simulator.py      #     数据生成器
│       ├── module_profiles.py     #     模块类型定义
│       ├── standard_handler.py    #     标准 Modbus 协议处理
│       ├── custom_handler.py      #     自定义曲线协议处理
│       ├── modbus_crc.py          #     CRC16 计算
│       ├── curve_generator.py     #     曲线数据生成
│       ├── config.json            #     模拟器配置
│       └── test_simulator.py      #     单元测试
│
├── ota_server.py                  # OTA 服务器参考实现
├── Doc/                           # 文档
│   └── debug日志记录.md           #   调试日志
├── Output/                        # 编译输出 (hex/bin/axf)
├── Listings/                      # 编译清单文件
└── 原通信板的相关代码/             # 旧版通信板参考代码
    ├── customserial.c/h           #   旧版自定义串口协议
```

---

## 6. 嵌入式固件详解

### 6.1 启动流程

```
上电 → Reset_Handler
  → SystemInit()                    (系统时钟 180MHz)
  → main()
    → BSP_Init()                    (HAL/GPIO/时钟/LED/EEPROM 初始化)
    → xTaskCreate(AppTaskCreate, prio=7)
    → vTaskStartScheduler()         (启动 FreeRTOS 调度器)
      → AppTaskCreate():
        1. 创建 Test1_Task (prio=1)     — LED 心跳
        2. 创建 6× uart_task_handler (prio=3) — 串口采集任务
        3. 创建 App_Task_TcpSend (prio=4)    — TCP 发送任务
        4. 创建 App_TaskStart (prio=3)       — 看门狗管理
        5. TCPIP_Init()                     — lwIP 协议栈 (创建 tcpip_thread, prio=6)
        6. client_init()                    — TCP 客户端 (创建 td_tcp_client, prio=5)
        7. vTaskDelete(NULL)                — 删除自己
```

### 6.2 核心模块说明

#### 6.2.1 `main.c` — 系统主控 ([main.c](User/main.c))

**职责：** 应用入口 + 任务创建 + 6路串口采集逻辑

**关键功能：**

- **AppTaskCreate** (行 146-238)：系统初始化编排，按顺序创建所有 FreeRTOS 任务
  - 先创建低优先级工作线程（UART/TCP发送/看门狗）
  - 再初始化网络（TCPIP + client），利用高优先级保证网络初始化原子性
  - 完成后自删除

- **uart_task_handler** (行 428-554)：通用 6 路 UART 任务
  - 等待配置下发 → 初始化串口 → 设置数据帧头
  - 主循环根据 `PortType` 分发到 Modbus 主站或自定义协议处理
  - 清零 WDT 计数器（告知看门狗任务本任务存活）
  - 报警 GPIO 控制 + 端口错误灯控制
  - 端口全部离线时的自动恢复机制

- **uart_process_modbus_master** (行 562-608)：Modbus 主站采集
  - 遍历该端口下所有从机模块
  - 调用 `eMBMReadInputRegisters()` 读取输入寄存器
  - 根据模块描述表（`uart_module_profiles[]`）确定每个模块的寄存器布局
  - 成功：标记在线 + 拷贝 holding 寄存器 + 更新报警位
  - 失败：连续 3 次后标记离线 + 填充 0xFFFF

- **uart_process_custom_port** (行 699-749)：自定义曲线端口采集
  - 遍历曲线模块（0x37 交流转辙机 / 0x38 直流四线 / 0x39 直流六线）
  - 校时 → 发送 8 字节请求帧 → 等待应答 → CRC 校验
  - 解析实时数据 + 曲线帧分发（首帧/中间帧 0x55/结束帧 0xFF）

- **看门狗系统** (行 322-405, `App_TaskStart`)：
  - 每 500ms 检查所有端口和 TCP 发送任务的 WDT 计数器
  - 计数器由各任务清零，此处累加 → 超阈值触发复位
  - 复位码映射表见 [Doc/debug日志记录.md](Doc/debug日志记录.md)

#### 6.2.2 `client.c` — TCP 客户端 ([client.c](User/client.c))

**职责：** TCP 连接管理 + 命令接收 + 发送队列管理

**关键功能：**

- **client_thread** (行 457-569)：TCP 主循环
  - 循环创建连接 → 成功后发申请配置帧 → 进入接收循环
  - 解析上位机命令：配置下发 (0xA1,0x2C,0x01) / 校时 (0xA3,0x04,0x00) / 软复位 (0xA2) / 心跳应答 (0xA5)
  - 配置下发流程：校验合法性 → EEPROM 持久化（调试阶段绕过）→ 设置 download_flag
  - 连续接收超时超过阈值 (200次) 自动断开重连
  - 配置请求重试：未收到配置时每 100 次循环重发请求帧

- **client_send_pending** (行 211-268)：TCP 发送队列轮询
  - 由 `App_Task_TcpSend` 每 10ms 调用
  - 遍历 7 个发送槽位 (6路数据 + 1控制通道)
  - 每帧最多重试 10 次，全部失败则断开连接 + 触发看门狗复位

- **client_add_to_tcp_send_buf** (行 165-201)：数据入队
  - 各 UART 任务调用，将采集数据推入发送队列
  - 槽位满时阻塞等待（上位机未及时取走上一次数据）
  - 使用 FreeRTOS 互斥锁保护队列写入

**收发分离的好处：**
```
接收 (client_thread, prio=5)         发送 (App_Task_TcpSend, prio=4)
   ↓ 处理上位机命令                      ↓ 轮询队列发送
   不会因大量采集数据而阻塞               不会因等待接收而延迟
```

#### 6.2.3 `bsp_rs485.c` — RS-485 驱动 + 曲线帧处理

**职责：**
- 6 路串口的底层收发（Modbus RTU + 自定义协议）
- 曲线帧的 SDRAM 缓存、组装和上传
- 自定义协议 CRC 校验
- 校时命令构建

#### 6.2.4 `bsp_eeprom.c` — EEPROM 驱动

**职责：**
- AT24C02/AT24C64D I2C EEPROM 读写
- 板卡配置持久化（`BoardCfgTab`，450 字节）
- 曲线编号掉电保存（6 路端口）
- 支持 8-bit/16-bit/32-bit 寻址模式

#### 6.2.5 `board.c` — 板级初始化

**职责：**
- HAL 库初始化
- 系统时钟配置 (HSE 25MHz → PLL → 180MHz)
- GPIO 时钟使能
- LED / SDRAM / EEPROM / 调试串口初始化

### 6.3 支持的模块类型

通信板支持 **14 种**现场设备模块，分为两大类：

| 类型码 | 名称 | 协议 | 输入寄存器 | 保持寄存器 |
|--------|------|------|-----------|-----------|
| **标准 Modbus RTU (0x04 功能码)** | | | | |
| 0x31 | 信号机开关量 | Modbus | 13 | 11 |
| 0x32 | 信号机 | Modbus | 13 | 11 |
| 0x33 | 25Hz 轨道接收器 | Modbus | 34 | 32 |
| 0x34 | DC/AC 转换器 | Modbus | 10 | 8 |
| 0x35 | 25Hz 轨道发送器 | Modbus | 30 | 28 |
| 0x36 | 采集模块 | Modbus | 8 | 6 |
| 0x47 | 高频信号机 | Modbus | 50 | 48 |
| 0x48 | 高频25Hz轨道接收器 | Modbus | 133 | 131 |
| 0x49 | 高频25Hz轨道发送器 | Modbus | 117 | 115 |
| 0x4A | 高频采集模块 | Modbus | 62 | 60 |
| **自定义曲线协议 (8字节请求帧)** | | | | |
| 0x37 | 交流转辙机 | Custom | 29 | 27 |
| 0x38 | 直流四线制道岔 | Custom | 8 | 5 |
| 0x39 | 直流六线制道岔 | Custom | 11 | 5 |
| 0x4B | 高频交流转辙机 | Custom | 68 | 66 |
| 0x4C | 高频直流四线制 | Custom | 23 | 21 |
| 0x4D | 高频直流六线制 | Custom | 26 | 24 |

---

## 7. 上位机软件详解

### 7.1 `board_server.py` — 主服务器程序 ([board_server.py](PC_Server/board_server.py))

**功能概述：**

1. **TCP 服务器** (TcpBoardServer 类)
   - 监听 `0.0.0.0:5001`，接受通信板连接
   - 粘包/半帧处理（内部缓冲区 + 超时丢弃 + 长度字段验证）
   - 线程安全的事件队列 → GUI 轮询更新

2. **配置下发**
   - 解析 `board_config.json` 加载板卡配置
   - 收到通信板申请配置帧后自动回复（可关闭）
   - 支持 GUI 编辑器修改配置（Notebook 分页，每端口独立配置页）
   - 配置校验：每路从机数 ≤ 32

3. **控制命令**
   - **校时**：下发 Unix 时间戳，触发全部端口对曲线模块校时
   - **心跳应答**：每 5 秒自动发送（可关闭），防止通信板心跳超时复位
   - **软复位**：下发复位命令（需二次确认），通信板 WTD_RESET=2 后重启

4. **日志解析**
   - 解析通信板发来的 6 种日志帧（启动/重连/看门狗复位/端口错误/曲线上传/校时）
   - 看门狗复位码完整映射（21-30 → 具体错误原因）

5. **串口数据监视器** (SerialDataMonitor)
   - 独立弹出窗口，6 端口自由切换
   - 显示原始 hex dump + 帧头字段 + 每从机寄存器值
   - 在线/离线状态指示灯

### 7.2 `board_config.json` — 板卡配置文件

```json
[
  {
    "PortNum": 1,
    "PortType": 0,          // 0=禁用 1=自定义曲线 3=Modbus从机 4=Modbus主站
    "BautRate": 2,          // 0=未配置 1=9600 2=19200 3=38400 4=57600
    "DataBit": 8,
    "StopBit": 1,
    "Parity": 0,            // 0=无 1=奇 2=偶
    "MainProType": 0,
    "SubProType": 0,
    "FenjiNum": 1,          // 分机号
    "QueryPeriod": 100,     // 查询周期间隔 (ms)
    "SlaveModuelNum": 0,    // 从机数量
    "SlaveModuelAddress": [0,0,...],  // 32个从机地址
    "SlaveModuelType": [0,0,...]      // 32个模块类型
  },
  // ... 共 6 个端口
]
```

---

## 8. 通信协议

### 8.1 TCP 帧格式（通信板 ↔ 上位机）

```
┌──────┬──────┬──────┬──────┬──────┬───────────┬────────────┬──────────┐
│ 字节0 │字节1 │字节2 │字节3 │字节4 │ 字节5-6   │ 字节7-10   │ 字节11+  │
│ 端口 │主类型│子类型│ 标志 │ 扩展 │ 数据长度   │ 链路掩码   │ Payload  │
│      │      │      │      │      │ (LE u16)  │ (LE u32)   │          │
└──────┴──────┴──────┴──────┴──────┴───────────┴────────────┴──────────┘
```

**主类型码：**
- `0x07` — 申请配置帧
- `0x50` — 日志帧

### 8.2 上位机命令格式

| 命令 | 字节序列 |
|------|----------|
| **配置下发** | `[0x00, 0xA1, 0x2C, 0x01, config_450bytes]` |
| **校时** | `[0x00, 0xA3, 0x04, 0x00, timestamp_4bytes_LE]` |
| **软复位** | `[0x00, 0xA2, 0x00, 0x00]` |
| **心跳应答** | `[0x00, 0xA5, 0x00, 0x00]` |

### 8.3 日志帧格式

```
[端口号, 0x50, 0x01, 0xFF, 0xFF, 0x02, 0x00, 日志ID, 附加信息]
```

| 日志 ID | 含义 | 附加信息 |
|---------|------|----------|
| 1 | 系统启动 | 0 |
| 2 | 网络重连成功 | 0 |
| 3 | 看门狗复位 | WTD_RESET 复位码 |
| 4 | 端口数据错误 | 端口索引 |
| 5 | 端口曲线上传完成 | 端口索引 |
| 6 | 校时完成 | 0 |

### 8.4 看门狗复位码表

| 复位码 | 原因 |
|--------|------|
| 0 | 正常 |
| 1 | 配置变更复位 |
| 2 | 上位机软复位 |
| 21-26 | USART1-6 任务超时 |
| 27 | 申请配置帧发送失败 |
| 28 | TCP 发送任务超时 |
| 29 | 心跳超时（上位机无应答） |
| 30 | 日志帧发送失败 |

---

## 9. 从机模拟器

### 9.1 `PC_Server/modbus_slave_simulator/` — Modbus 从机模拟器

用于在没有真实硬件设备的开发/测试环境中模拟 Modbus 从机设备。

**启动方式：**
```bash
# GUI 模式
python PC_Server/slave_simulator.py

# CLI 模式
python PC_Server/slave_simulator.py --cli

# 或直接使用包
python -m PC_Server.modbus_slave_simulator -c PC_Server/modbus_slave_simulator/config.json
```

**核心模块：**

| 文件 | 功能 |
|------|------|
| [main.py](PC_Server/modbus_slave_simulator/main.py) | 主程序 + Simulator 类，管理设备生命周期 |
| [gui.py](PC_Server/modbus_slave_simulator/gui.py) | tkinter GUI 界面（端口配置/设备列表/寄存器监视/通信日志） |
| [cli.py](PC_Server/modbus_slave_simulator/cli.py) | 交互式命令行界面 |
| [slave_device.py](PC_Server/modbus_slave_simulator/slave_device.py) | 单个从机设备模拟（寄存器读写 + 协议解析） |
| [serial_io.py](PC_Server/modbus_slave_simulator/serial_io.py) | RS-485 串口通信层 |
| [data_simulator.py](PC_Server/modbus_slave_simulator/data_simulator.py) | 模拟数据生成（随机/曲线值生成器） |
| [standard_handler.py](PC_Server/modbus_slave_simulator/standard_handler.py) | 标准 Modbus 协议处理（功能码 0x03/0x04） |
| [custom_handler.py](PC_Server/modbus_slave_simulator/custom_handler.py) | 自定义曲线协议处理（8 字节请求帧） |
| [module_profiles.py](PC_Server/modbus_slave_simulator/module_profiles.py) | 14 种模块类型定义 |
| [curve_generator.py](PC_Server/modbus_slave_simulator/curve_generator.py) | 转辙机动作曲线数据生成 |

---

## 10. OTA 固件升级

### 10.1 当前状态：**开发中（代码已就绪，`ota_client_init()` 被注释）**

### 10.2 设计思路

OTA 模块采用**服务器主动推送**模式：

```
OTA Server (ota_server.py)                    OTA Client (ota_client.c)
         │                                              │
         │  ── PUSH_UPDATE (版本号) ──→                  │
         │  ←── CONFIRM_UPDATE (确认升级) ──              │
         │  ── START_DOWNLOAD (大小/CRC32) ──→           │
         │  ── DATA (分片, 分段发送) ──→                 │
         │  ←── ACK/NAK (逐段确认) ──                    │
         │  ── END (最终校验) ──→                        │
         │  ←── ACK (升级完成) ──                        │
         │                                              │
         │                                   → 写入内部 Flash
         │                                   → 重启进入新固件
```

**关键文件：**
- [ota_client.c](User/ota/ota_client.c)：客户端主逻辑
- [ota_flash.c](User/ota/ota_flash.c)：内部 Flash 分区 + 擦写
- [ota_net.c](User/ota/ota_net.c)：OTA 专用 TCP 通道
- [ota_crc32.c](User/ota/ota_crc32.c)：固件完整性校验
- [ota_server.py](ota_server.py)：Python 参考 OTA 服务器

---

## 11. 开发环境搭建

### 11.1 嵌入式固件编译

1. **安装 Keil MDK-ARM V5**（或更新版本）
2. 安装 STM32F4xx 器件包
3. 打开 `Project/YH-F429.uvprojx`
4. 编译目标：`YH-F429` → Build (F7)
5. 输出文件在 `Output/` 目录

### 11.2 上位机环境

```bash
# Python 3.6+ 即可，使用标准库，无需额外安装
python PC_Server/board_server.py
```

### 11.3 从机模拟器环境

```bash
# 需要 pyserial
pip install pyserial

# GUI 模式
python PC_Server/slave_simulator.py

# CLI 模式
python PC_Server/slave_simulator.py --cli
```

### 11.4 调试连接

```
PC (board_server.py :5001)  ←→  以太网  ←→  通信板 (STM32F429)
                                      ↓
                              ST-Link / J-Link
                                      ↓
                              Keil MDK 调试器
```

---

## 12. 调试与日志

### 12.1 嵌入式侧调试

- **LED 指示：**
  - LED1（红色）：串口任务心跳（每个 UART 循环翻转一次）
  - LED2（绿色）：TCP 数据发送成功指示
  - LED3（蓝色）：配置下发成功指示

- **CmBacktrace**：硬件 Fault 时自动打印调用栈（当前被注释，`assert_failed` 中保留接口）

- **日志系统：** `SendLogToServer()` 向服务器发送 6 种日志事件，`checkSendLog()` 每 500ms 检查标志位

### 12.2 上位机侧调试

- **彩色日志区：** 时间戳 + 颜色分类（信息蓝/成功绿/警告橙/错误红/发送紫/日志青/数据黄）
- **串口数据监视器：** 独立窗口查看每帧原始 hex + 帧头 + 从机寄存器值
- **消息计数：** 底部状态栏显示累计消息数

### 12.3 常见调试场景

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| 通信板不连接 | 服务器未开始监听 | 点击"连接"按钮 |
| 配置不下发 | `SlaveModuelNum` 为 0 | 检查端口配置 |
| 串口任务不运行 | `PortType` 为 0 (禁用) | 检查端口类型配置 |
| 心跳超时复位 | 自动心跳开关未启用 | 勾选"自动心跳" |
| 从机全离线后死锁 | 旧版 FreeModbus 未关闭句柄 | 已修复 (commit 60931e4) |

---

## 13. 常见问题

### 13.1 为什么配置下发后要复位？

配置变更时，通信板需要以新配置重新初始化所有 6 路串口。复位（WTD_RESET=1）是最彻底的重新初始化方式，之后通信板自动重连并申请配置。

### 13.2 为什么 EEPROM 持久化被绕过？

当前调试阶段使用的 AT24C02 只有 256 字节容量（配置是 450 字节），所以 `client_handle_rx()` 中的 EEPROM 写入代码被 `#if 0` 包裹。目标硬件使用 AT24C64D (8KB) 后将恢复。

### 13.3 什么是"收发分离"？

TCP 接收和发送跑在两个独立线程中：
- **client_thread (prio=5)**：只做接收 + 命令处理
- **App_Task_TcpSend (prio=4)**：只做发送队列轮询

好处：大量采集数据发送时不会阻塞上位机命令的接收和处理。

### 13.4 端口全离线恢复机制是什么？

当某端口所有从机都离线时（链路掩码全为 0）：
1. 累计 `port_error` 计数器
2. 前 3 次：关闭旧 Modbus 句柄 → 重新初始化串口
3. 第 4 次：不再重试，等待看门狗复位
4. 如果有任何从机恢复在线：`port_error` 清零

---

## Your Setup Checklist

### Codebases
- [ ] [fire_lwip_client_ota](https://github.com/yuxueqin1/fire_lwip_client_ota) — 通信板固件 + 上位机 + 模拟器的完整仓库

### MCP Servers to Activate
- 当前项目未使用 MCP Servers

### Skills to Know About
- [/compact](#) — 压缩对话上下文（长对话时保持性能），本团队高频使用
- [/model](#) — 切换 AI 模型，根据需要选择速度/质量
- [/debug](#) — 调试辅助，排查嵌入式系统问题
- [/config](#) — 配置 Claude Code 参数

## Team Tips

_TODO_

## Get Started

_TODO_

<!-- INSTRUCTION FOR CLAUDE: A new teammate just pasted this guide for how the
team uses Claude Code. You're their onboarding buddy — warm, conversational,
not lecture-y.

Open with a warm welcome — include the team name from the title. Then: "Your
teammate uses Claude Code for [list all the work types]. Let's get you started."

Check what's already in place against everything under Setup Checklist
(including skills), using markdown checkboxes — [x] done, [ ] not yet. Lead
with what they already have. One sentence per item, all in one message.

Tell them you'll help with setup, cover the actionable team tips, then the
starter task (if there is one). Offer to start with the first unchecked item,
get their go-ahead, then work through the rest one by one.

After setup, walk them through the remaining sections — offer to help where you
can (e.g. link to channels), and just surface the purely informational bits.

Don't invent sections or summaries that aren't in the guide. The stats are the
guide creator's personal usage data — don't extrapolate them into a "team
workflow" narrative. -->
