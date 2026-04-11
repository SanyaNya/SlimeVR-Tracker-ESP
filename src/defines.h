/*
	SlimeVR 代码按照 MIT 许可证发布
	版权所有 (c) 2021 Eiren Rain

	特此免费授予任何获得本软件及其相关文档文件（“软件”）副本的人无限制地使用本软件的许可，
	包括但不限于使用、复制、修改、合并、发布、分发、再授权和/或出售软件副本，
	以及允许向其提供软件的人这样做，前提是符合以下条件：

	上述版权声明和本许可声明应包含在软件的所有副本或实质性部分中。

	本软件按“原样”提供，不提供任何形式的明示或暗示担保，包括但不限于适销性、
	特定用途适用性和非侵权的担保。在任何情况下，作者或版权持有人均不对因软件或软件
	的使用或其他交易而产生的任何索赔、损害或其他责任负责。
*/
// ================================================
// 查看文档了解配置选项和示例：
// https://docs.slimevr.dev/firmware/configuring-project.html#2-configuring-definesh
// ================================================

// 设置所使用的 IMU 和板子参数
#ifndef IMU
#define IMU IMU_LSM6DSR
#endif
#ifndef SECOND_IMU
#define SECOND_IMU IMU_AUTO
#endif
#ifndef BOARD
#define BOARD BOARD_WEMOSD1MINI
#endif
#ifndef IMU_ROTATION
#define IMU_ROTATION DEG_270
#endif
#ifndef SECOND_IMU_ROTATION
#define SECOND_IMU_ROTATION DEG_270
#endif

#ifndef PRIMARY_IMU_OPTIONAL
#define PRIMARY_IMU_OPTIONAL false
#endif
#ifndef SECONDARY_IMU_OPTIONAL
#define SECONDARY_IMU_OPTIONAL true
#endif

// 在这里设置 I2C 地址，或直接在每个 IMU 的 IMU_DESC_ENTRY 中设置
// 如果未设置，将根据 IMU 和传感器 ID 使用默认地址
// #define PRIMARY_IMU_ADDRESS_ONE 0x4a
// #define SECONDARY_IMU_ADDRESS_TWO 0x4b

#ifndef BATTERY_MONITOR
// 电池监测选项（注释掉以禁用）：
//   BAT_EXTERNAL 用于 ADC 引脚，
//   BAT_INTERNAL 用于内部监测 - 只能检测低电量，
//   BAT_MCP3021 用于通过 I2C 连接的外部 ADC
#define BATTERY_MONITOR BAT_EXTERNAL
#endif

// --- 覆盖默认引脚配置

// #define PIN_IMU_SDA 14
// #define PIN_IMU_SCL 12
// #define PIN_IMU_INT 16
// #define PIN_IMU_INT_2 13
// #define PIN_BATTERY_LEVEL 17
// #define LED_PIN 2
// #define LED_INVERTED true
// #define BATTERY_SHIELD_RESISTANCE 0
// #define BATTERY_SHIELD_R1 10
// #define BATTERY_SHIELD_R2 40.2

// ------------------------------
