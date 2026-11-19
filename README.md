# upper_control — STM32G474RET6 CAN 收发验证固件

本工程用于验证 STM32G474 的 CAN1（FDCAN1）收发，并提供 RS01 的
**get-ID / 使能 / 失能** 三个最小命令；不发送位置、速度、运控、电流或参数写入帧。

- 板卡：STM32G474RET6
- CAN：FDCAN1，经典 CAN（非 CAN-FD），**1 Mbit/s**，29 位扩展帧
- 串口：USART1，**PB6 = TX，PB7 = RX**，115200 8N1
- 烧录器：**CMSIS-DAP / DAP-Link**（OpenOCD `interface/cmsis-dap.cfg`）
- 电机 ID：**0x09**
- 帧格式、波特率与 `rs01_motor_control` 保持一致

> **范围说明**：已实现通信类型 `0x00`（get-ID）、`0x03`（使能）、`0x04`（失能，
> 可带清错位）。**没有**实现运控、位置、速度、电流、设零点、参数读写等指令。
> 上电**不会**自动使能电机；只有串口按 `e` 才会发送使能帧。
>
> 使能后电机可能保持/执行之前的运动目标，首次测试请空载或固定电机，电源加限流。

---

## 1. 目录与文件

```text
upper_control/
├── main.c              # 时钟、串口、命令循环
├── main.h              # 板卡/引脚/波特率/测试帧配置
├── fdcan.c / fdcan.h   # FDCAN1 底层：1 Mbit/s、过滤器、中断收、发送、状态
├── can_comm.c / .h     # 应用层：按 a 发一帧、打印收到的每一帧、状态统计
├── rs01.c / rs01.h     # 仅扩展帧 ID 布局与 get-ID 帧构造（不含控制协议）
├── stm32g4xx_it.c      # SysTick、HardFault、FDCAN1_IT0 中断入口
├── syscalls.c          # printf 重定向到 USART1
├── stm32g4xx_hal_conf.h
├── STM32G474RETX_FLASH.ld
├── Makefile
└── Drivers/            # STM32G4 HAL + CMSIS（与 stm32/rs01_test 相同）
```

---

## 2. 硬件接线

| 信号 | STM32G474 | 说明 |
| --- | --- | --- |
| CAN1_RX | **PA11**（AF9） | 接 CAN 收发器的 RXD |
| CAN1_TX | **PA12**（AF9） | 接 CAN 收发器的 TXD |
| CAN1 端子 | **11A = CAN_H，12A = CAN_L** | 仅当板上已有收发器时；若 11A/12A 是 PA11/PA12 直接引出的排针，则必须外接收发器 |
| 串口 TX | **PB6**（AF7） | USART1_TX，接 DAP-Link 的 CDC RX |
| 串口 RX | **PB7**（AF7） | USART1_RX，接 DAP-Link 的 CDC TX |
| 烧录 | SWDIO / SWCLK / GND / 3V3 | DAP-Link |
| 终端电阻 | 120 Ω | 总线两端各一个 |
| 电机电源 | 24 V（按 RS01 要求） | 与主板共地 |

> **先确认 11A/12A 到底是什么：**
> - 如果 11A/12A 是板上 **CAN 收发器的输出**，它们才是 CAN_H/CAN_L；
> - 如果 11A/12A 只是 **PA11/PA12 直接引出的排针**，它们不是 CAN_H/CAN_L，
>   必须外接 TJA1050 / SN65HVD230 / MCP2551 等收发器，再把收发器的 H/L 接总线。
>
> PA11/PA12 是 FDCAN1 的 **RX/TX 单端信号**，不能直接接电机 CAN_H/CAN_L。
> 直接把 PA11/PA12 当差分线接到总线，典型现象就是 `lec=5(bit0)` + bus-off。

---

## 3. 波特率与帧格式（与 rs01_motor_control 一致）

### 3.1 位时序

| 项目 | 值 |
| --- | --- |
| 系统时钟 | HSI16 → PLL → 150 MHz |
| FDCAN 内核时钟 | PLLQ = **75 MHz** |
| 预分频 | 3 → 25 MHz tq（40 ns） |
| 位时间 | 1 + 21 + 3 = **25 tq = 1 µs** |
| 波特率 | **1 Mbit/s** |
| 采样点 | (1+21)/25 = **88 %** |

### 3.2 扩展帧 ID 布局

```text
bit 28..24 : 通信类型 comm
bit 23..8  : 附加数据 extra
bit 7..0   : 目标电机 ID

ID = (comm << 24) | (extra << 8) | motor_id
```

### 3.3 按 `a` 发送的帧

| 字段 | 值 |
| --- | --- |
| 帧类型 | 经典 CAN 数据帧，29 位扩展 ID |
| CAN ID | `(0x00 << 24) | (0xFF << 8) | 0x09` = **`0x0000FF09`** |
| 通信类型 | `0x00` = get device ID（获取设备 ID） |
| 附加数据 | `0x00FF`（主机 ID 0xFF） |
| 电机 ID | `0x09` |
| DLC | 8 |
| 数据 | `00 00 00 00 00 00 00 00` |

电机正常应答时，预期收到：

| 字段 | 值 |
| --- | --- |
| CAN ID | `(0x00 << 24) | (0x09 << 8) | 0xFE` = **`0x000009FE`** |
| 含义 | comm=0，应答电机 ID 在 bit15..8（0x09），低字节 0xFE 为广播应答标记 |
| DLC | 8 |

> `0x00` 只是获取设备 ID，不属于使能/运控/位置/速度/电流指令。如果总线上
> 没有电机，也可以接 USB-CAN 分析仪，按 `a` 后应能看到 `0x0000FF09`。

### 3.4 使能 / 失能帧（`e` / `d` / `D`）

RS01 没有单独的硬件 enable 线；**使能就是一条 CAN 命令**（通信类型 `0x03`）。

| 命令 | CAN ID | 数据区 | 说明 |
| --- | --- | --- | --- |
| `e` 使能 | `0x0300FF09` | `00 00 00 00 00 00 00 00` | 使能电机，电机回 type-2 反馈 |
| `d` 失能 | `0x0400FF09` | `00 00 00 00 00 00 00 00` | 正常失能 |
| `D` 失能+清错 | `0x0400FF09` | `01 00 00 00 00 00 00 00` | `Byte0=1` 清故障 |

使能/失能后，电机应回答**通信类型 2 的反馈帧**。反馈帧的扩展 ID 中：

```text
bit 15..14 : 模式状态  0=Reset, 1=Cali, 2=Motor(运行)
bit 13..8  : 6 bit 故障摘要
bit 7..0   : 电机 ID（0x09）
```

本固件会把反馈打印成：

```text
[RX] id=0x02xxxx09 ext dlc=8 comm=0x02(feedback) extra=0x8xxx motor=0x09 data=...
       -> feedback: mode=2(motor) fault=0x00
```

> `e` 只发送使能帧，不发送任何位置/速度/运控目标。但电机如果还保留着上一次的
> 运动目标，使能后仍可能动作；首次测试请空载或固定电机。

---

## 4. 编译、烧录、串口

### 4.1 进入工具链环境

```bash
cd /media/user/disk2/black_2b
source scripts/env.sh
```

### 4.2 编译

```bash
cd upper_control
make            # build/upper_control.elf / .bin / .hex
make size
make clean
```

当前构建结果（供参考）：

```text
FLASH: 24668 B / 512 KB
RAM  :  2968 B / 128 KB
0 warning
```

### 4.3 烧录（DAP-Link）

```bash
make probe      # 可选：检查 OpenOCD 能否打开 CMSIS-DAP
make flash      # 烧录 + verify + reset
make reset      # 只复位，重新打印 banner
```

默认参数：

```text
interface/cmsis-dap.cfg
transport swd
target/stm32g4x.cfg
```

如果使用 ST-Link：

```bash
make flash OOCD_INTERFACE=stlink.cfg
```

首次使用 DAP 需要 udev 规则（工作区已有脚本）：

```bash
cd /media/user/disk2/black_2b
sudo ./scripts/install_dap_driver.sh
```

### 4.4 打开串口

```bash
cd /media/user/disk2/black_2b
./scripts/serial_console.sh          # 自动找 /dev/ttyACM0 等，115200
# 或
picocom -b 115200 /dev/ttyACM0
```

串口必须接在 **PB6(TX) / PB7(RX)**，115200 8N1，无流控。

---

## 5. 使用方法

上电后应看到类似输出：

```text
==================================================
 upper_control - CAN bus send/receive test
 board    : STM32G474RET6
 CAN      : FDCAN1, classic CAN, 1000000 bit/s
 pins     : PA11=FDCAN1_RX, PA12=FDCAN1_TX (AF9)
 terminal : 11A=CAN_H, 12A=CAN_L
 console  : USART1 PB6=TX, PB7=RX, 115200 8N1
 motor id : 0x09
--------------------------------------------------
  'a' sends : id=0x0000FF09 (29-bit extended), DLC 8, payload 8 x 00
              comm=0x00(get-id) extra=0x00FF motor=0x09
--------------------------------------------------
 Only get-ID / enable / disable are implemented.
 No motion/position/velocity/current frame is sent.
 Press 'a' get-ID, 'e' enable, 'd' disable, 'h' help.
==================================================
```

### 命令（单键，无需回车）

| 键 | 作用 |
| --- | --- |
| `a` | 发送 get-ID 报文，等待 200 ms 看是否有应答 |
| `e` | 发送 RS01 **使能**帧（`0x03`），等待 type-2 反馈 |
| `d` | 发送 RS01 **失能**帧（`0x04`），等待 type-2 反馈 |
| `D` | 失能并清错（`0x04`，payload[0]=1） |
| `r` | 软件复位 MCU（板上没有 reset 键时用这个） |
| `h` | 帮助 |
| `x` | CAN 状态、错误计数器、收发统计、最近反馈 mode/fault |
| `l` | FDCAN 内部回环自测（只验证 MCU 侧，不经过总线） |
| `n` | 停止并重新初始化 FDCAN1（清除 bus-off） |

### 正常收发示例

```text
a

[TX] id=0x0000FF09 ext dlc=8 comm=0x00(get-id) extra=0x00FF motor=0x09 data=00 00 00 00 00 00 00 00
[RX] id=0x000009FE ext dlc=8 comm=0x00(get-id) extra=0x0009 motor=0xFE data=...
       -> get-id reply, motor answers with ID 0x09
[TX] done, waited 200 ms: 1 frame(s) received, get-id reply OK (motor 0x09)
```

若没有电机，只有分析仪：

```text
a

[TX] id=0x0000FF09 ext dlc=8 comm=0x00(get-id) extra=0x00FF motor=0x09 data=00 00 00 00 00 00 00 00
[TX] done, waited 200 ms: 0 frame(s) received, no reply
      -> check CAN_H/CAN_L, 120 ohm, transceiver, motor power,
         bitrate (1 Mbit/s) and motor ID (0x09)
```

> 没有其他节点 ACK 时，FDCAN 会自动重发并累计发送错误计数，最终可能进入
> bus-off。这是正常的“总线上只有自己”的表现。接好线或接上分析仪后按 `n`
> 重新初始化即可。

### 使能测试示例

```text
e

[TX] enable id=0x0300FF09 ext dlc=8 comm=0x03(enable) extra=0x00FF motor=0x09 data=00 00 00 00 00 00 00 00
[TX] enable sent; waiting 500 ms for type-2 feedback...
[RX] id=0x02xxxx09 ext dlc=8 comm=0x02(feedback) extra=0x8xxx motor=0x09 data=...
       -> feedback: mode=2(motor) fault=0x00
[TX] enable: feedback received, mode=2(motor) fault=0x00
      -> motor reports run mode; enable accepted
```

若没有反馈：

```text
[TX] enable: no feedback frame in 500 ms
      -> check motor 24 V, CAN wiring, motor ID 0x09
```

- `mode=2(motor)` 只表示电机进入运行模式；是否转动取决于它之前的运动目标。
- 想停止：按 `d`；如果电机报故障，按 `D` 失能并清错。
- 板上没有 reset 键时，按 `r` 软件复位，或执行 `make reset`。

---

## 6. 验证步骤建议

1. **先做 MCU 侧自测**：串口按 `l`。
   - `PASS`：FDCAN 外设、位时序、过滤器、收发 FIFO 都正常，问题在总线侧。
   - `FAIL`：先检查时钟配置/固件，不要怀疑接线。
2. **接 USB-CAN 分析仪**（1 Mbit/s，经典 CAN，扩展帧）：
   - 按 `a`，分析仪应收到 `0x0000FF09`，DLC 8，数据全 0。
   - 从分析仪发一帧扩展帧，串口应打印 `[RX] ...`。
3. **接 RS01 电机**（ID 0x09、24 V 供电、共地、120 Ω）：
   - 按 `a`，应收到 `0x000009FE` 的 get-ID 应答；应答的 8 字节是电机 64 位 MCU UID。
   - 若电机处于主动上报状态，串口也会打印 `0x02` 反馈帧；这同样证明 RX 正常。
   - 确认 get-ID 正常后，再按 `e` 做使能测试；按 `d`/`D` 失能。
4. 按 `x` 查看 `tx_err/rx_err/bus_off/lec`：

   | 状态 | 含义 | 优先检查 |
   | --- | --- | --- |
   | `lec=3(ack)`，`tx_err` 增长 | 帧发到了 ACK 位，但总线上没有节点应答 | 对端是否上电/接入、CAN_H/L 是否通、120 Ω、对端波特率 |
   | **`lec=5(bit0)`**，`tx_err` 增长 | 控制器发出显性位后，在自己的 RX 上读回隐性位 | **收发器是否在位/供电/使能、PA11/PA12 接线、TXD/RXD 是否交叉、H/L 是否接反** |
   | `lec=4(bit1)` | 发出隐性位却读回显性位 | 总线短路、多个节点同时发送、收发器故障 |
   | `lec=1/2/6(stuff/form/crc)` | 位流/格式/CRC 错误 | 波特率不匹配、线束干扰、终端电阻 |
   | `rx_err` 增长 | 接收方向出现位/格式错误 | 波特率、接线、终端电阻 |
   | `bus_off=1` | 发送错误计数超过 255，控制器退出总线 | 先修物理层，再按 `n` 重启 FDCAN1 |

   > **`lec=5(bit0)` 是最重要的线索**：它说明“本机连自己发出的显性位都收不到”，
   > 不是“对端不回 ACK”。如果只是总线上没有对端，通常会看到 `lec=3(ack)`。
   > 因此出现 `bit0` 时优先怀疑：没有收发器、收发器未供电/处于 standby、
   > PA11/PA12 直接当成了 CAN_H/CAN_L、TXD/RXD 接反、或 H/L 接反。

---

## 7. 常见问题

| 现象 | 处理 |
| --- | --- |
| 串口无 banner | 检查 DAP CDC 是否接 PB6/PB7、波特率 115200、TX/RX 是否交叉 |
| 串口乱码 | 波特率不对，或 TX/RX 接反 |
| `a` 后 `tx_err` 增长，`lec=3(ack)` | 总线上没有 ACK：电机/分析仪未上电或未接入、CAN_H/L 断、无 120 Ω、对端波特率不对 |
| `a` 后 `lec=5(bit0)`，bus-off | 本机收不到自己的显性位：**优先查收发器**（是否在位/供电/STB 使能）、PA11/PA12 是否直接当 H/L、TXD/RXD/H-L 是否接反 |
| `l` PASS 但 `a` 无应答 | MCU 内部回环正常，只说明 FDCAN 核/位时序正常；问题在 PA11/PA12 引脚、收发器或线束。用示波器看 PA12/PA11，或接分析仪抓 `0x0000FF09` |
| 收到帧但不是 get-ID 应答 | 电机可能在主动上报，或 ID 不是 0x09；用 `x` 看原始帧 |
| bus-off | 修好接线后按 `n`；或复位 |
| `make flash` 打不开 DAP | 安装 udev 规则、重新插拔 DAP、确认 `lsusb` 能看到探针 |
| `make flash` 报 transport 错误 | 默认已用 `cmsis-dap.cfg` + `swd`；不要误用 `stlink.cfg`（需 `hla_swd`） |

---

## 8. 配置修改位置

| 需求 | 修改 |
| --- | --- |
| 换电机 ID | `main.h` 的 `APP_MOTOR_ID` |
| 换主机 ID | `main.h` 的 `APP_HOST_ID`（默认 0xFF，与库一致） |
| 帧类型 | 固定提供 `0x00` get-ID、`0x03` 使能、`0x04` 失能；不提供运控/位置/速度/电流帧 |
| 换串口引脚 | `main.h` 的 `APP_CONSOLE_*` + `main.c` 的 `HAL_UART_MspInit` |
| 换 CAN 引脚 | `main.h` 的 `APP_CAN_*` + `fdcan.c` 的 `HAL_FDCAN_MspInit` |
| 换波特率 | `fdcan.c` 的位时序 + `main.h` 注释；保持 75 MHz 内核时钟时按 `25 tq` 计算 |

---

## 9. 后续扩展

本工程的 `rs01.c/h` 保留了扩展帧 ID 布局、get-ID 和 enable/disable 帧构造。
后续如果要接 RS01 运控/位置/速度/电流协议，可在 `rs01.c/h` 中按
`rs01_motor_control` 的帧定义继续加入；底层 `fdcan_send()` 已经可以直接发送
任意 29 位扩展帧，`can_comm.c` 的接收打印和反馈 mode/fault 解析也可以复用。
