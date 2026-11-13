# EndlessFlight H7Ultra 自定义 PX4/LQI 固件移植与修改记录

> **文档用途**：记录从 Ryan 原始自定义 PX4 固件向 EndlessFlight H7Ultra 移植时，源码究竟增加了什么、修改了什么，以及完成了哪些联调验证。适合纳入仓库 `Docs/` 或用于工作周报、项目交接。
>
> **比较基线**：`Ryan-5853/PX4-TeleaiTrans`，提交 `3fef8242687cf733b590ff21c5b36779943299d1`。
>
> **移植结果**：`Li-Yuhang7683/PX4-TeleaiTrans`，分支 `feature/endlessflight-h7ultra`，提交 `12d74dbb53`。
>
> **注意**：本文只比较上述两个固定版本，不包含 Ryan 在此基线之后的新控制算法及双机功能。描述的是**已提交源码差异及已提供的实际运行记录**，不将通信成功等同于飞行安全验证。

## 1. 项目背景与移植目标

原始固件基于 PX4 v1.16.2 的 Ryan 自定义控制框架，主要面向 MicoAir H743 V2 飞控，并集成共轴双电机三舵机 LQI 控制器、`nn_control`、`gs_link` 和 `airlink` 等功能。

本次目标是：**在不修改 Ryan 原有 LQI 控制算法和通信协议实现的前提下，使对应自定义固件能够在 EndlessFlight H7Ultra（STM32H743VIT6）上编译、启动、接受 ELRS/CRSF 遥控输入，并通过 ESP32-C5 与 Dock Ground Station 进行 GS-Link 通信。**

最终构建命令：

```bash
make endlessflight_h743ultra_nncontrol
```

本次主要成果可归纳为：

1. **硬件 BSP 移植**：引入 H7Ultra 的 NuttX、GPIO、UART、SPI/I2C、DMA、PWM/DShot、SDIO、ADC、USB、Bootloader 和固件打包配置。
2. **公共驱动扩展**：增加 QMC5883P 磁力计驱动及设备类型、构建系统注册。
3. **自定义固件构建**：新增 H7Ultra 的 `nncontrol` / `gslink` 等构建目标，沿用原控制框架。
4. **启动与参数适配**：预置 RC/CRSF/GPS/机架参数、执行器输出相关默认值及 LQI 台架安全门控。
5. **通信部署改动**：将 GS-Link 启动波特率由 4,000,000 调整为 5,000,000，并完成硬件双向通信验证。

## 2. 全仓库 Git 差异统计

从 `3fef824` 到 `12d74db`，经 Git 提交比较：

| 指标 | 数量 |
|---|---:|
| 差异文件总数 | **37** |
| 新增路径 | **34** |
| 修改已有路径 | **3** |
| 直接修改 LQI 核心代码 | **0** |
| 直接修改 GS-Link 协议实现 | **0** |
| 直接修改 AirLink 协议实现 | **0** |

按路径统计：

| 范围 | 文件数 | 用途 |
|---|---:|---|
| `boards/endlessflight/h743ultra/` | 26 | H7Ultra BSP、NuttX、固件目标、启动脚本、Bootloader |
| `src/drivers/` | 8 | QMC5883P 驱动及公共注册 |
| `Tools/h7ultra_port/` | 2 | BSP 导入工具和来源说明 |
| `ROMFS/px4fmu_common/init.d/rc.direct_control_apps` | 1 | GS-Link 启动波特率 |

这份统计对**整个移植分支**成立，并非只统计最后一次 Git 提交。

### 2.1 真正修改的原有三个文件

| 原有文件 | 变更内容 |
|---|---|
| `ROMFS/px4fmu_common/init.d/rc.direct_control_apps` | `gs_link` 启动参数由 `-b 4000000` 改为 `-b 5000000` |
| `src/drivers/drv_sensor.h` | 增加 QMC5883P 的磁力计设备类型标识 |
| `src/drivers/magnetometer/CMakeLists.txt` | 将 QMC5883P 纳入磁力计驱动构建 |

其余 34 个文件相对于原始 Ryan 基线都是**新增路径**。其中部分文件在移植过程中后来又进行了二次修改，但在全分支比较中依然计为新增。

## 3. 硬件 BSP 移植（主要工作）

### 3.1 新增板级文件与构建支持

新增目录：`boards/endlessflight/h743ultra/`。

主要内容包括：

| 类别 | 文件/目录 | 作用 |
|---|---|---|
| 固件目标 | `default.px4board`、`gslink.px4board`、`nncontrol.px4board`、`bootloader.px4board` | 定义编译进入固件的驱动、控制模块和工具 |
| 固件打包 | `firmware.prototype` | H7Ultra 固件元信息与镜像限制 |
| Bootloader 文件 | `extras/endlessflight_h743ultra_bootloader.bin` | 板级引导相关二进制文件；提交到源码不意味着此次已刷写 Bootloader |
| NuttX | `nuttx-config/bootloader/defconfig`、`nuttx-config/nsh/defconfig` | 系统和 Bootloader 编译配置 |
| 内存与 DMA | `nuttx-config/include/board.h`、`board_dma_map.h`、`scripts/*.ld` | 引脚、DMA、链接地址与内存布局 |
| 板级源码 | `src/board_config.h`、`hw_config.h`、`init.c`、`CMakeLists.txt` | MCU 外设定义与初始化 |
| I/O | `src/i2c.cpp`、`spi.cpp`、`timer_config.cpp` | 外设总线、执行器定时器和输出映射 |
| 其他板级功能 | `src/sdio.c`、`usb.c`、`led.c`、`bootloader_main.c` | SD 卡、USB、指示灯和引导入口 |

**来源说明**：这部分主要基于厂家 H7Ultra PX4 1.16.0 BSP 导入，再适配 Ryan 的 PX4 v1.16.2。不能将其全部表述为原创编写的底层驱动代码。

### 3.2 与 MicoAir 的主要硬件差异

| 子系统 | 移植后 H7Ultra | Ryan 原有 MicoAir 平台 |
|---|---|---|
| 板级路径 | `boards/endlessflight/h743ultra` | `boards/micoair/h743-v2` |
| IMU | SCH16T + 双 IIM42652 | BMI088 + BMI270 |
| 气压计 | SPA06 | SPL06 |
| 磁力计 | QMC5883P | QMC5883L |
| GS-Link 串口设备 | `/dev/ttyS0` | `/dev/ttyS0` |
| AirLink 串口设备 | `/dev/ttyS1` | `/dev/ttyS1` |
| 原生 RC 串口设备 | `/dev/ttyS4` | `/dev/ttyS5` |
| PWM/DShot | H7Ultra GPIO、定时器、DMA 映射 | MicoAir 对应的独立映射 |

**注意**：串口设备名相同不等于物理引脚相同。真正的 UART/GPIO/定时器关系由各自 BSP 定义。

## 4. BSP 之外的源码改动

### 4.1 新增 QMC5883P 磁力计驱动

原始 Ryan 基线没有供 H7Ultra 使用的这一套 QMC5883P 驱动，本次新增：

```text
src/drivers/magnetometer/qmc5883p/
├── CMakeLists.txt
├── Kconfig
├── QMC5883P.cpp
├── QMC5883P.hpp
├── QST_QMC5883P_registers.hpp
└── qmc5883p_main.cpp
```

另修改公共 `drv_sensor.h` 与磁力计目录 `CMakeLists.txt`，使新驱动可被构建系统识别。

H7Ultra 传感器启动脚本中的主要命令：

```sh
board_adc start
sch16t -s -b 1 -R 2 start
iim42652 -s -b 2 -R 4 start
iim42652 -s -b 3 -R 6 start
spa06 -I -a 0x76 start
qmc5883p -I -a 0x2C -R 6 start
```

驱动编译成功与飞控能够启动不代表全部传感器校准、安装方向及数据精度已经完成飞行级验证。

### 4.2 创建 H7Ultra 专用 `nncontrol` 编译目标

新增：`boards/endlessflight/h743ultra/nncontrol.px4board`。

保留核心自定义模块：

- `CONFIG_MODULES_NN_CONTROL=y`
- `CONFIG_MODULES_GS_LINK=y`
- `CONFIG_MODULES_AIRLINK=y`
- `CONFIG_MODULES_COMP_ATTITUDE=y`
- `CONFIG_NN_CONTROL_BACKEND_LQR=y`

同时关闭普通 PX4 多旋翼/固定翼飞控链路中的若干模块（例如 `MC_ATT_CONTROL`、`MC_RATE_CONTROL`、`MC_POS_CONTROL`、`CONTROL_ALLOCATOR`、`FLIGHT_MODE_MANAGER`），保留 Commander、MAVLink、遥控输入、EKF2、传感器、Logger 等必要功能。由于是精简目标，QGC 的部分标准功能页面可能引用未编入固件的参数（如 `CP_DIST`）。

**本项属于固件构建与软件模块集成，不是对 LQI 算法的修改。**

### 4.3 机架与 RC/GPS 出厂默认值

文件：`boards/endlessflight/h743ultra/init/rc.board_defaults`。

专用 `22001` 机架存在时，预置：

```sh
param set-default SYS_AUTOSTART 22001
param set-default RC_PORT_CONFIG 300
param set-default RC_SBUS_PRT_CFG 0
param set-default RC_CRSF_PRT_CFG 0
param set-default GPS_1_CONFIG 0
```

文件还具有公共预设：

```sh
param set-default RC_INPUT_PROTO -1
```

设计特点：

- **`param set-default`** 定义出厂默认值；QGC 中已保存的用户参数不应被无条件覆盖。
- 删除了旧的 `SYS_AUTOSTART != 22001` 时强制 `param set` 回 `22001` 的逻辑。
- 未打包 `22001` 机架的普通 H7Ultra 固件仍默认 `SYS_AUTOSTART=4001`。
- 用户如果已保存其他机架编号，默认值不会强制覆盖；但切换到未打包的机架可能导致启动配置不可用。

另外保留了仅用于专用固件台架调试的强制安全保护：

```sh
param set NN_LQI_OUTPUT_EN 0
```

这行**不是**“QGC 可修改且断电后仍保持使能”的普通默认值逻辑。它在相应启动分支执行时会重新把物理输出关闭，这是刻意保留的安全行为。

### 4.4 执行器和传感器脚本中的预设

文件：`boards/endlessflight/h743ultra/init/rc.board_sensors`。专用目标新增：

```sh
param set-default PWM_MAIN_TIM0 -4
param set-default PWM_MAIN_TIM1 400
param set-default PWM_MAIN_TIM2 400
param set-default DSHOT_BIDIR_EN 1
param set-default GPS_1_CONFIG 0
```

主要用于 DShot 电机、舵机 PWM 与双向 DShot 测速相关默认配置。

**需注意默认值的重复来源**：继承的公共机架 `22001_direct_actuator_controller` 中也包含 PWM 参数预设。因此当前文件结构里存在多处默认值配置；实际启动结果还受启动顺序、已有保存参数和硬件定时器分组约束影响，应以刷写后参数检查为准。

### 4.5 GS-Link 启动波特率固定为 5 Mbps

直接修改了 Ryan 原有启动脚本：

文件：`ROMFS/px4fmu_common/init.d/rc.direct_control_apps`

```diff
- gs_link start -d /dev/ttyS0 -b 4000000
+ gs_link start -d /dev/ttyS0 -b 5000000
```

该调整改变了**启动时串口驱动的波特率**，没有修改 GS-Link 报文定义、CRC、心跳或收发协议代码。

已通过实机：

```text
INFO [gs_link] port: /dev/ttyS0, baud: 5000000, open: 1
```

后续与 ESP32-C5 DroneBridge 串口配置应保持 5 Mbps 一致。

**遗留一致性问题**：继承的 `22001` 机架脚本里仍出现 `param set SER_TEL1_BAUD 4000000`，而自定义 `gs_link` 当前通过启动命令 `-b 5000000` 指定实际速率。两者数值不一致，后续整理时应注意不能错误地让 MAVLink 再次占用同一 UART。

### 4.6 工程复现工具与说明

新增：

```text
Tools/h7ultra_port/
├── README.md
└── import_vendor_bsp.py
```

作用是记录上游源码与厂家 BSP 的固定来源、导入流程及复现方法。现有 README 的部分早期描述（如“尚未编译验证”）已滞后于实际完成进度，后续应更新。

## 5. 明确保留、没有修改的 Ryan 核心算法

相对于 `3fef824`，下列关键实现文件没有源码差异：

| 模块 | 代表文件/范围 | 状态 |
|---|---|---|
| LQI 控制调度 | `src/modules/nn_control/NNControl.cpp` | 保留原实现 |
| LQI 核心运算 | `LqiCoaxialCollectiveCore.hpp` | 保留原实现 |
| LQI 模型 ABI | `LqiCoaxialCollectiveModel.hpp` | 保留原实现 |
| LQI 权重载荷 | `LqiCoaxialCollectivePayload.hpp` | 保留原实现 |
| GS-Link 协议 | `src/modules/gs_link/GsLinkProtocol.hpp`、`GsLink.cpp` | 保留原实现 |
| AirLink 协议与状态机 | `src/modules/airlink/` | 保留原实现 |
| EKF2 / Commander 核心 | `src/modules/ekf2/`、`src/modules/commander/` | 保留原实现 |

因此应准确表述为：**基于现有算法完成硬件移植与部署适配，而非重新开发 LQI 控制算法。**

当前实机报告仍采用 `airframe-1` 载荷：

```text
backend: lqi_coaxial_collective
airframe: airframe-1, expected id 1, configured id 1, mismatch 0
verification weights: 504 bytes, checksum: 0x78ae1111
```

新组装飞机 `airframe-3` 的机体辨识和专用载荷部署属于后续工作，不能混同为本次已完成事项。

## 6. 本次完成的编译、刷写与联调验证

以下是运行记录支持的验证结果（不等于飞行安全认证）：

| 项目 | 结果 | 依据/说明 |
|---|---|---|
| H7Ultra `nncontrol` 构建 | 已成功 | 生成 `endlessflight_h743ultra_nncontrol.px4` |
| 固件刷写与 QGC USB | 已成功 | 能进入 NSH 并查询参数与运行状态 |
| 机架 | `22001` | `param show SYS_AUTOSTART` |
| RC 端口 | `300` | `param show RC_PORT_CONFIG` |
| RC 协议 | 当前保存为 `6` | 固件默认 `-1`，已保存的 CRSF 值 `6` 优先生效 |
| SBUS / CRSF 额外端口 | `0 / 0` | 参数查询确认 |
| GPS1 配置 | `0` | 参数查询确认 |
| GS-Link 串口 | `5000000` / `open: 1` | 重新上电后的 `gs_link status` |
| GS-Link 端到端通信 | 成功验证过 | 曾出现 `heartbeat OK`、双向包数增加、CRC 错误为零 |
| `nn_control` 频率 | `500.0 Hz` | `nn_control status` |
| LQI 物理输出 | `inhibited` | `NN_LQI_OUTPUT_EN=0`，台架保护 |

GS-Link 联调链路：

```text
Dock Ground Station（Ryan 服务器）
    ↕  Wi-Fi / UDP
XIAO ESP32-C5（DroneBridge）
    ↕  UART1 / 5 Mbps
EndlessFlight H7Ultra（PX4 GS-Link + nn_control）
```

注意：有一次重启后查询时也出现过 `heartbeat stale` 和 `rx packets: 0`，这属于当次地面站通信未建立的状态，不应写成“任意时刻持续在线”。用户后续确认地面站链路可正常使用。

## 7. 尚未完成或需要后续复核的事项

1. **实际执行器台架验证**：确认电机/舵机通道、方向、限位、DShot RPM 与失效输出；带电测试必须保持拆桨及可靠安全隔离。
2. **机架启动参数一致性**：排查公共机架脚本、板级脚本对 PWM 参数的多处设置；核对 `SER_TEL1_BAUD` 与 GS-Link 实际固定波特率的区别。
3. **控制载荷匹配**：目前仍使用 `airframe-1`；未来需生成、验证并部署真正的 `airframe-3` 模型载荷。
4. **磁力计和 IMU 实测校准**：检查驱动读数、坐标变换、偏置与冗余传感器选择，而不仅是能启动。
5. **固件板型识别**：H7Ultra 与原 MicoAir 的 `firmware.prototype` 均出现 `board_id=1179`；需要进一步核查 Bootloader/板型匹配，避免刷错固件。
6. **README 与发布追溯**：更新导入文档中的“未编译”旧描述；保留带时间戳 `.px4`、SHA-256、编译环境、源码提交号。
7. **飞行安全验证**：通信和 500 Hz 运行成功不代表已经完成解锁、失控保护或首次飞行验证。在此之前继续保持输出禁用。

## 8. 完整改动文件清单（相对 Ryan 基线）

### 8.1 `boards/endlessflight/h743ultra/`（26 个）

```text
boards/endlessflight/h743ultra/bootloader.px4board
boards/endlessflight/h743ultra/default.px4board
boards/endlessflight/h743ultra/extras/endlessflight_h743ultra_bootloader.bin
boards/endlessflight/h743ultra/firmware.prototype
boards/endlessflight/h743ultra/gslink.px4board
boards/endlessflight/h743ultra/init/rc.board_defaults
boards/endlessflight/h743ultra/init/rc.board_extras
boards/endlessflight/h743ultra/init/rc.board_sensors
boards/endlessflight/h743ultra/nncontrol.px4board
boards/endlessflight/h743ultra/nuttx-config/bootloader/defconfig
boards/endlessflight/h743ultra/nuttx-config/include/board.h
boards/endlessflight/h743ultra/nuttx-config/include/board_dma_map.h
boards/endlessflight/h743ultra/nuttx-config/nsh/defconfig
boards/endlessflight/h743ultra/nuttx-config/scripts/bootloader_script.ld
boards/endlessflight/h743ultra/nuttx-config/scripts/script.ld
boards/endlessflight/h743ultra/src/CMakeLists.txt
boards/endlessflight/h743ultra/src/board_config.h
boards/endlessflight/h743ultra/src/bootloader_main.c
boards/endlessflight/h743ultra/src/hw_config.h
boards/endlessflight/h743ultra/src/i2c.cpp
boards/endlessflight/h743ultra/src/init.c
boards/endlessflight/h743ultra/src/led.c
boards/endlessflight/h743ultra/src/sdio.c
boards/endlessflight/h743ultra/src/spi.cpp
boards/endlessflight/h743ultra/src/timer_config.cpp
boards/endlessflight/h743ultra/src/usb.c
```

### 8.2 公共磁力计驱动及注册（8 个）

```text
src/drivers/drv_sensor.h
src/drivers/magnetometer/CMakeLists.txt
src/drivers/magnetometer/qmc5883p/CMakeLists.txt
src/drivers/magnetometer/qmc5883p/Kconfig
src/drivers/magnetometer/qmc5883p/QMC5883P.cpp
src/drivers/magnetometer/qmc5883p/QMC5883P.hpp
src/drivers/magnetometer/qmc5883p/QST_QMC5883P_registers.hpp
src/drivers/magnetometer/qmc5883p/qmc5883p_main.cpp
```

### 8.3 移植工具（2 个）

```text
Tools/h7ultra_port/README.md
Tools/h7ultra_port/import_vendor_bsp.py
```

### 8.4 公共启动脚本（1 个）

```text
ROMFS/px4fmu_common/init.d/rc.direct_control_apps
```

## 9. 提交记录与复现

- Ryan 基线：[3fef8242687c](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/3fef8242687cf733b590ff21c5b36779943299d1)
- 你的最终提交：[12d74dbb53](https://github.com/Li-Yuhang7683/PX4-TeleaiTrans/commit/12d74dbb53)
- [两版全仓库源码差异（37 个文件）](https://github.com/Li-Yuhang7683/PX4-TeleaiTrans/compare/3fef8242687cf733b590ff21c5b36779943299d1...12d74dbb53)

在本地 WSL2 PX4 工程中可复核：

```bash
cd ~/projects/PX4-H7Ultra-Ryan

git diff --stat 3fef8242687cf733b590ff21c5b36779943299d1 12d74dbb53
git diff --name-status 3fef8242687cf733b590ff21c5b36779943299d1 12d74dbb53
```

> **总结**：本次工作本质上是 **Ryan 自定义 PX4/LQI 固件的 H7Ultra BSP 移植 + QMC5883P 驱动集成 + `nncontrol` 构建目标适配 + 出厂参数及启动流程配置 + GS-Link 5 Mbps 通信联调**。核心 LQI 控制算法及原有 GS-Link/AirLink 协议实现没有修改。移植后的固件已完成构建、刷写和通信验证，但后续仍需针对新飞机进行控制载荷辨识及执行器/飞行安全验证。
