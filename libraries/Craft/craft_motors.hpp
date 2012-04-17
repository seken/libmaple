#pragma once

#include <cstdint>

class MotorOutput {
	public:
		MotorOutput(const uint8_t *pins, const uint8_t &count);

		void set(const uint16_t &fl_pwr, const uint16_t &fr_pwr, const uint16_t &bl_pwr, const uint16_t &br_pwr) const;

	private:
		const uint8_t m_count;
		const uint8_t *const m_pins;
};

