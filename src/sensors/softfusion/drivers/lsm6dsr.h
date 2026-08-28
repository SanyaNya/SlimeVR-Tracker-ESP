/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "lsm6ds-common.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 4g
// and gyroscope range at 1000dps
// Gyroscope ODR = 208Hz, accel ODR = 104Hz

struct LSM6DSR : LSM6DSOutputHandler {
	static constexpr uint8_t Address = 0x6a;
	static constexpr auto Name = "LSM6DSR";
	static constexpr auto Type = SensorTypeID::LSM6DSR;

	static constexpr float GyrFreq = 208;
	static constexpr float AccFreq = 208;
	static constexpr float MagFreq = 120;
	static constexpr float TempFreq = 52;

	static constexpr float GyrTs = 1.0 / GyrFreq;
	static constexpr float AccTs = 1.0 / AccFreq;
	static constexpr float MagTs = 1.0 / MagFreq;
	static constexpr float TempTs = 1.0 / TempFreq;

	static constexpr float GyroSensitivity = 1000 / 35.0f;
	static constexpr float AccelSensitivity = 1000 / 0.122f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 256.0f;

	static constexpr float TemperatureZROChange = 20.0f;

	static constexpr VQFParams SensorVQFParams{};

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x0f;
			static constexpr uint8_t value = 0x6b;
		};
		struct Ctrl1XL {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value = (0b01011000);  // XL at 208 Hz, 4g FS
		};
		struct Ctrl2GY {
			static constexpr uint8_t reg = 0x11;
			static constexpr uint8_t value = (0b01011000);  // GY at 208 Hz, 1000dps FS
		};
		struct Ctrl3C {
			static constexpr uint8_t reg = 0x12;
			static constexpr uint8_t valueSwReset = 1;
			static constexpr uint8_t value = (1 << 2);  // IF_INC = 1
		};

		struct Status
		{
			static constexpr uint8_t reg = 0x1E;
			static constexpr uint8_t GyrAccMask = 0b011;
			static constexpr uint8_t TempMask = 0b100;
		};

		struct TempGyrAcc
		{
			static constexpr uint8_t reg = 0x20;
			struct Data
			{
				int16_t temp;
				int16_t gyr[3];
				int16_t acc[3];
			};
			static_assert(sizeof(Data) == 14);
		};

		struct GyrAcc
		{
			static constexpr uint8_t reg = 0x22;
			struct Data
			{
				int16_t gyr[3];
				int16_t acc[3];
			};
			static_assert(sizeof(Data) == 12);
		};
	};



	LSM6DSR(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: LSM6DSOutputHandler(registerInterface, logger) {}

	bool initialize() {
		// perform initialization step
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
		delay(20);
		m_RegisterInterface.writeReg(Regs::Ctrl1XL::reg, Regs::Ctrl1XL::value);
		m_RegisterInterface.writeReg(Regs::Ctrl2GY::reg, Regs::Ctrl2GY::value);
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);

		return true;
	}

	bool bulkRead(DriverCallbacks<int16_t>&& callbacks)
	{
		const uint8_t status = m_RegisterInterface.readReg(Regs::Status::reg);
		const bool t = (status & Regs::Status::TempMask) == Regs::Status::TempMask;
		const bool ga = (status & Regs::Status::GyrAccMask) == Regs::Status::GyrAccMask;
		if(t && ga)
		{
			Regs::TempGyrAcc::Data data;
			m_RegisterInterface.readBytes(Regs::TempGyrAcc::reg, sizeof(data), reinterpret_cast<uint8_t*>(&data));
			callbacks.processGyroSample(data.gyr, GyrTs);
			callbacks.processAccelSample(data.acc, AccTs);
			callbacks.processTempSample(data.temp, TempTs);
		}
		else if(ga) [[likely]]
		{
			Regs::GyrAcc::Data data;
			m_RegisterInterface.readBytes(Regs::GyrAcc::reg, sizeof(data), reinterpret_cast<uint8_t*>(&data));
			callbacks.processGyroSample(data.gyr, GyrTs);
			callbacks.processAccelSample(data.acc, AccTs);
		}

		return false;
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
