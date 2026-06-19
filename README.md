# STMWheels — 基于 STM32 的智能小车

## 简介

基于 STM32F103C8T6 + FreeRTOS 的四轮智能小车，支持循迹、避障、遥控等功能。

## 硬件

| 模块 | 型号/说明 |
|------|-----------|
| 主控 | STM32F103C8T6 (Blue Pill) |
| 电机驱动 | RZ7899 ×4（四轮独立驱动） |
| 循迹 | 5 路红外对管（XJ_1 ~ XJ_5） |
| 超声波 | HC-SR04 |
| 显示 | OLED (I2C) |
| RTOS | FreeRTOS|
| 构建 | CMake + Ninja / STM32CubeIDE |

## 快速开始

### 前置条件

- **STM32CubeIDE**（推荐）或 **arm-none-eabi-gcc** 工具链
- CMake ≥ 3.22
- ST-Link 调试器


### 引脚分配

详见 [main.h](Core/Inc/main.h) 和 [STMWheels.ioc](STMWheels.ioc)（CubeMX 工程）。

| 功能 | 引脚 |
|------|------|
| 循迹 XJ_1~5 | PA0, PA1, PA3, PA4, PB15 |
| 电机 LQ | PB0/PB1 (TIM3_CH4/CH3) |
| 电机 RQ | PB4/PB5 (TIM3_CH1/CH2) |
| 电机 LH | PB3/PB6 (TIM2_CH2) |
| 电机 RH | PB7/PA11 (TIM4_CH2) |
| 红外 PWM | PB8 (TIM4_CH3) |
| 超声波 TRIG/ECHO | PA8/PB9 |
| I2C (OLED) | PB10/PB11 |
| UART | PA9/PA10 |

## 调参

编辑 [CarConfig.h](CarConfig.h)，修改后重新编译即可生效，无需改动业务代码。


## 注意事项

1. **电机方向**：首次上电前请悬空车轮，确认 `MOTOR_xx_INVERT` 与实际安装方向一致，否则可能导致控制逻辑反转。
2. **PWM 频率**：默认 TIM2/3/4 的 PWM 频率已针对 RZ7899 优化，修改定时器预分频会影响电机响应特性。
3. **FreeRTOS 堆栈**：默认堆大小 3072 字节，增加任务或调大栈空间时注意修改 `configTOTAL_HEAP_SIZE`。
4. **电源**：电机和 MCU 应分开供电，避免电机堵转导致 MCU 掉电复位。
5. **循迹传感器**：`IR_PWM_PULSE` 控制红外发射亮度，环境光变化较大时需要调整。
6. **CubeMX 重新生成**：修改 `.ioc` 后重新生成代码会覆盖 `Core/` 下的文件（`USER CODE` 区域内的代码会被保留）。

## 许可证

本项目（`BSP/`、`Core/` 中用户代码、`CarConfig.h`、`CMakeLists.txt` 等）采用 **Apache License 2.0**。

- ✅ 允许商用、修改、分发、专利授权
- ✅ 需保留原始版权声明和许可证
- ✅ 修改过的文件需标注变更

详见 [LICENSE](LICENSE)。

---

*Made with STM32CubeIDE & FreeRTOS*
