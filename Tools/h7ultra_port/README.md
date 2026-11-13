# EndlessFlight H7Ultra / Ryan PX4 自定义 LQI 固件移植

> 本文记录将 Ryan 的 PX4 自定义控制固件移植到 EndlessFlight H7Ultra 的源码来源、工程状态、编译归档方法和验证边界。该分支已经完成台架级软件与通信联调，**尚未完成真实飞行安全验证，不应视作可直接装桨试飞的合格版本**。

## 1. 固定版本与来源

| 项目 | 版本或信息 |
| --- | --- |
| Ryan 移植基线 | [`Ryan-5853/PX4-TeleaiTrans`](https://github.com/Ryan-5853/PX4-TeleaiTrans)，`teleai/v1.16.2` 在移植时的提交 `3fef8242687cf733b590ff21c5b36779943299d1` |
| 本次移植源码 | [`Li-Yuhang7683/PX4-TeleaiTrans`](https://github.com/Li-Yuhang7683/PX4-TeleaiTrans)，`feature/endlessflight-h7ultra`；首个完整移植记录提交 `12d74dbb53` |
| 飞控 | EndlessFlight H7Ultra，STM32H743VIT6，板级标识 `EndlessFlight_H7ultra` |
| 厂家 BSP | [EndlessFlight fc-h7-ultra](https://gitee.com/endlessflight/fc-h7-ultra)；用户提供 `fc-h7-ultra-master.zip`，内含 `PX4/PX4_1.16.0/h743ultra.zip`（24 个 BSP 文件） |
| 厂家 BSP 内层 ZIP 的 SHA-256 | `b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27` |
| QMC5883P 驱动来源 | PX4 提交 `ebf8b69c66fb30242bd9d6aa5f9ea9effcbe7e46`（含传感器量程比例修正） |
| 固件打包 | `board_id=1179`、最大镜像大小 `1835008` 字节；**原 MicoAir 固件也使用此 Board ID，不能据此认为两块硬件可互刷** |

本文的“Ryan 基线”特指上述固定提交，**不包含其后新增的新版 LQI Schema 5、双机组合控制等功能**。完整的逐文件改动说明见仓库根目录的 [《H7Ultra PX4/LQI 固件移植与修改记录》](../../H7Ultra_PX4_LQI_固件移植与修改记录.md)。

## 2. 已完成的源码移植

相对 `3fef824`，截至 `12d74dbb53` 共有 **37 个差异文件（34 个新增、3 个已有文件修改）**，集中在：

- `boards/endlessflight/h743ultra/`：加入完整 H7Ultra 板级文件、NuttX、GPIO、SPI、I2C、UART、ADC、DMA、PWM、SDIO、USB、链接脚本、Bootloader 支持文件及构建目标。
- `src/drivers/magnetometer/qmc5883p/`、`src/drivers/drv_sensor.h`、`src/drivers/magnetometer/CMakeLists.txt`：集成 QMC5883P 驱动和构建注册。
- `boards/endlessflight/h743ultra/default.px4board`：保留官方 H7Ultra 常规构建配置；针对 Ryan PX4 1.16.2 不存在的 Kconfig 符号 `CONFIG_COMMON_INS` 作兼容处理，实际 IMU 驱动仍启用。
- `boards/endlessflight/h743ultra/gslink.px4board`：单独的通信测试构建配置；与 `nncontrol` 目标不同，**不要混淆二者验证结果**。
- `boards/endlessflight/h743ultra/nncontrol.px4board`：启用 Ryan 原有 `nn_control`、`gs_link`、`airlink`、`comp_attitude`，保留所需 Commander、RC、传感器、估计器、日志、USB MAVLink；按专用直接执行器控制架构关闭传统 PX4 多旋翼控制器、Control Allocator 和 Flight Mode Manager。
- `ROMFS/px4fmu_common/init.d/rc.direct_control_apps`：将 GS-Link 的启动波特率从 **4,000,000** 改为 **5,000,000**；仍沿用原 GS-Link 协议实现。
- `boards/endlessflight/h743ultra/init/rc.board_defaults`、`rc.board_sensors`、`rc.board_extras`：设置专用机架、RC/CRSF/GPS、DShot/PWM 默认值和补充姿态估计器启动；对 `nncontrol` 固件在开机时执行 `param set NN_LQI_OUTPUT_EN 0`，禁止 LQI 物理输出以供安全台架调试。

**未直接修改** Ryan 基线中的 `NNControl.cpp`、`LqiCoaxialCollectiveCore.hpp`、`LqiCoaxialCollectiveModel.hpp`、`LqiCoaxialCollectivePayload.hpp`、`GsLink.cpp`、`GsLinkProtocol.hpp`、AirLink 及 EKF2 核心源码。当前使用的仍是 **airframe-1 / Schema 3** LQI 载荷，而不是重新辨识的 airframe-3 模型。

## 3. 当前硬件与通信映射

| 功能 | 飞控接口 | PX4 设备或配置 |
| --- | --- | --- |
| GS-Link → ESP32-C5 | UART1 / TELEM1 | `/dev/ttyS0`，`gs_link start -d /dev/ttyS0 -b 5000000` |
| AirLink | UART2 | `/dev/ttyS1`，原启动脚本配置为 6 Mbps |
| ELRS / CRSF | UART5 | `/dev/ttyS4`，`RC_PORT_CONFIG=300` |
| USB QGroundControl | USB CDC | 与 GS-Link 串口互不占用 |

`SER_TEL1_BAUD` 仍可能显示旧值 **4 Mbps**，但当前 GS-Link 实际启动速率由 `gs_link start` 的 `-b 5000000` 指定，应以运行中的 `gs_link status` 为准。

板级传感器启动配置包括 SCH16T、两路 IIM42652、SPA06 和 QMC5883P；**启动配置存在不等于已完成全部传感器校准及飞行可靠性验证**。

## 4. 已确认的台架联调状态

已在当前 H7Ultra `nncontrol` 工程中观察或验证：

- `make endlessflight_h743ultra_nncontrol` 完成 ARM 固件编译，已将专用 `.px4` 文件刷写到 H7Ultra 并正常启动。
- QGC 可通过 USB 连接，专用机架和主要 RC/CRSF/GPS 参数可在 NSH 检查；已观察到有效的手动 RC 输入。
- `nn_control status` 报告目标频率和实测频率均为 **500 Hz**；在已检查时 LQI 物理输出为 **inhibited**。
- `gs_link status` 报告 `/dev/ttyS0` 以 **5,000,000 baud** 打开；曾在 ESP32-C5 与 Dock Ground Station 联调中确认双向通信和有效心跳。重启后是否连通仍应重新检查。
- Git 源码已推送到 `feature/endlessflight-h7ultra`；带时间戳 `.px4` 文件按下面的办法保存在本地构建目录，**不会随 Git 提交自动上传**。

**未据此证明**：装桨飞行安全、全部 IMU/磁力计标定、所有 PWM/DShot 实际通道与电机/舵机对应关系、AirLink 实际双机控制，以及 airframe-3 的辨识和载荷部署。

## 5. WSL2 获取源码、编译与带时间戳归档

在 **Windows PowerShell** 中启动 Ubuntu（已在 WSL2 终端则跳过）：

```powershell
wsl -d Ubuntu-24.04
```

首次获取仓库（**已有仓库时不要重复克隆或覆盖**）：

```bash
mkdir -p ~/projects
cd ~/projects
git clone --branch feature/endlessflight-h7ultra --recurse-submodules \
  https://github.com/Li-Yuhang7683/PX4-TeleaiTrans.git PX4-H7Ultra-Ryan
cd ~/projects/PX4-H7Ultra-Ryan
```

已有本地仓库时，先检查分支和工作区；如需同步远程提交，在确认本地修改已妥善保存后再 `git pull --ff-only`：

```bash
cd ~/projects/PX4-H7Ultra-Ryan
git branch --show-current
git status --short
git log -1 --oneline
```

在已安装 PX4 ARM 交叉编译工具链、项目依赖齐备的 WSL2 环境中，使用下面命令进行**成功编译后才归档**：

```bash
cd ~/projects/PX4-H7Ultra-Ryan

if make endlessflight_h743ultra_nncontrol; then
    BUILD_DIR="build/endlessflight_h743ultra_nncontrol"
    SOURCE="$BUILD_DIR/endlessflight_h743ultra_nncontrol.px4"

    if [ ! -s "$SOURCE" ]; then
        echo "错误：找不到本次编译的固件：$SOURCE" >&2
    else
        TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
        ARCHIVE="$BUILD_DIR/endlessflight_h743ultra_nncontrol_${TIMESTAMP}.px4"
        cp -v "$SOURCE" "$ARCHIVE"
        sha256sum "$ARCHIVE" > "$ARCHIVE.sha256"
        sha256sum -c "$ARCHIVE.sha256"
        ls -lh "$SOURCE" "$ARCHIVE" "$ARCHIVE.sha256"
    fi
else
    echo "编译失败，本次不进行归档。" >&2
fi
```

归档目标：`build/endlessflight_h743ultra_nncontrol/`。原始 `.px4` 可以随重新编译而更新，带时间戳文件用于区分各次构建；但 **`build/` 是临时构建目录，执行清理或删除构建目录时可能丢失归档**，重要版本应额外备份。刷写时先检查文件名、Board ID 与目标硬件，**不要向 H7Ultra 刷写 MicoAir 固件或 Bootloader**。

## 6. 关于厂家 BSP 导入工具

`Tools/h7ultra_port/import_vendor_bsp.py` 是**初次移植的可追溯导入工具**，用于按固定 SHA-256 校验厂家 ZIP、检查 24 个板级文件并将缺失文件导入原始 Ryan 基线。它默认只预览（dry-run），`--apply` 才会写入缺失文件；不负责编译、提交 Git、刷机或编程 Bootloader。

**当前分支已提交完整 BSP，不需要再次运行此工具。** 另外，当前 `rc.board_defaults` / `rc.board_sensors` 已经过有意修改，导入器会比较厂家原始启动命令；在当前已适配分支上重新运行，可能因启动命令不同而拒绝执行。这是安全校验，并不代表当前源码损坏。若要重做初始移植，请另建从 `3fef824` 派生的独立工作分支或工作树，并先审查导入结果；不要在现有经过验证的分支直接运行 `--apply`。

## 7. 尚待完成的核验与风险

1. **飞行安全**：目前只确认构建、启动、控制循环及部分通信链路，尚未完成真实飞行条件下的控制与安全认证。LQI 输出当前保持禁止；严禁未经系统验证直接装桨试飞。
2. **传感器与执行器**：复核三路 IMU、气压计、磁力计的校准和方向；检查电机/舵机接线、失效动作、DShot/PWM 通道与频率。特别注意 `rc.board_sensors` 和机架脚本里可能存在重复的 PWM 默认值，最终以 `param show` 为准。
3. **控制器机体匹配**：当前为 airframe-1 基线载荷；airframe-3 需要单独完成辨识、模型生成、校验及台架验证。
4. **硬件识别**：H7Ultra 和原 MicoAir 的 `firmware.prototype` 均出现 `board_id=1179`；不要仅凭 Board ID 选择固件。
5. **状态文档**：每次新的实际飞控测试、载荷更新或安全策略调整后，更新本 README 的“已确认”与“尚待完成”部分，避免将预计功能写成已验证功能。

## 8. 源码差异与追溯

```bash
# 对比原始 Ryan 基线与本次已提交的完整移植版本
git diff --stat 3fef8242687cf733b590ff21c5b36779943299d1 12d74dbb53

git diff --name-status 3fef8242687cf733b590ff21c5b36779943299d1 12d74dbb53
```

对应的 [GitHub 固定版本差异页](https://github.com/Li-Yuhang7683/PX4-TeleaiTrans/compare/3fef8242687cf733b590ff21c5b36779943299d1...12d74dbb53)。
