# EndlessFlight H7Ultra / GS-Link port (Ryan PX4 v1.16.2)

**Status: source integration in progress; NOT compiled or qualified for flight.**

## Pinned provenance

- Ryan base: `Ryan-5853/PX4-TeleaiTrans` branch `teleai/v1.16.2`, commit `3fef8242687cf733b590ff21c5b36779943299d1`.
- Manufacturer: https://gitee.com/endlessflight/fc-h7-ultra . User-supplied `fc-h7-ultra-master.zip` contains `PX4/PX4_1.16.0/h743ultra.zip` (24 BSP files), SHA-256 `b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27`.
- Board identity: `EndlessFlight_H7ultra`, `board_id=1179`, max image size `1835008` bytes. Hardware: STM32H743VIT6, 2 × IIM42652, SPA06, QMC5883P, UART1 TELEM1 `/dev/ttyS0`.
- QMC5883P source: official PX4 commit `ebf8b69c66fb30242bd9d6aa5f9ea9effcbe7e46` (sensor scale correction). Exact six source files have been committed; API/real sensor calibration still require testing.

## What this branch already contains

- Original Ryan `gs_link`, `nn_control`, `airlink`, `comp_attitude` preserved without modifying their source.
- Six QMC5883P driver files; CMake directory registration and `DRV_MAG_DEVTYPE_QMC5883P 0x0F`.
- H7Ultra `default.px4board`, `firmware.prototype`, and manufacturer-equivalent `rc.board_defaults`, `rc.board_sensors`.
- Separate `gslink.px4board` with `CONFIG_MODULES_GS_LINK=y`, retaining PX4 multicopter controllers, Commander, RC, control allocator, EKF2, USB MAVLink.
- SHA-pinned `Tools/h7ultra_port/import_vendor_bsp.py` to extract the remaining NuttX, bootloader source, GPIO, SPI, I2C, PWM, DMA and linker files from the original vendor ZIP. Vendor bootloader *binary* is staged only in the local source tree, never flashed.
- One intentional board config adaptation: comment `CONFIG_COMMON_INS=y` because Ryan 1.16.2 has no corresponding Kconfig symbol. All actual IMU drivers and QMC5883P remain enabled.

**Important**: remaining vendor BSP files are not yet present in the remote branch. Run the importer on your local checkout before attempting compilation; review and commit those files separately. Do not manually substitute any other H743 board.

## Exact WSL2 workflow (does not touch hardware)

Open **Windows PowerShell / Windows Terminal**:
```powershell
wsl -d Ubuntu-24.04
```
Then run these commands **inside Ubuntu**:
```bash
mkdir -p ~/projects
cd ~/projects
git clone --branch feature/endlessflight-h7ultra --recurse-submodules \
  https://github.com/Li-Yuhang7683/PX4-TeleaiTrans.git PX4-H7Ultra-Ryan
cd ~/projects/PX4-H7Ultra-Ryan
git status --short
git log -1 --oneline
```
If a checkout already exists, do not overwrite it; first examine `git status --short` and use `git fetch origin` plus `git switch feature/endlessflight-h7ultra` only after protecting local work.

Place the supplied manufacturer ZIP where Ubuntu can read it. **Example only** for `D:\BaiduNetdiskDownload\fc-h7-ultra-master.zip`:
```bash
VENDOR_ZIP=/mnt/d/BaiduNetdiskDownload/fc-h7-ultra-master.zip
ls -lh "$VENDOR_ZIP"
python3 Tools/h7ultra_port/import_vendor_bsp.py \
  --vendor-zip "$VENDOR_ZIP" --px4-dir ~/projects/PX4-H7Ultra-Ryan
```
First invocation is **read-only dry-run**. Check the reported 24 files, Git base and conflicts. To install missing board files into the local source checkout:
```bash
python3 Tools/h7ultra_port/import_vendor_bsp.py \
  --vendor-zip "$VENDOR_ZIP" --px4-dir ~/projects/PX4-H7Ultra-Ryan --apply
git status --short
grep CONFIG_BOARD_SERIAL_TEL1 boards/endlessflight/h743ultra/default.px4board
grep QMC5883P boards/endlessflight/h743ultra/default.px4board
grep CONFIG_MODULES_GS_LINK boards/endlessflight/h743ultra/gslink.px4board
```

The importer **does not** commit, push, build, flash or alter firmware parameters.

## Hardware and source validation required

- This branch has not completed a real PX4 ARM build. Once toolchains and all original BSP files are present, attempt `make endlessflight_h743ultra_gslink` and capture any errors; the command is a **target to test**, not a verified build.
- Verify NuttX/CMake compatibility, QMC5883P actual device ID and correct ±2G scale, both IMUs and SPA06, flash usage and USB/RC/actuator safety before any test firmware is used.
- GS-Link is compiled, **not automatically started**, and does not grant remote motor control. Stock UART1 MAVLink is still configured until a separate propeller-free bench procedure changes serial ownership. USB QGC must remain available.
- Do not flash the MicoAir firmware/bootloader to H7Ultra. Board ID 1179 alone is not proof of a matching physical board.
- **NOT FLIGHT-READY.** The separate `nn_control`/LQI migration is not enabled or validated here.