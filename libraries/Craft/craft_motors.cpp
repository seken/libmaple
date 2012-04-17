#include "craft_motors.hpp"
#include <wirish.h>

MotorOutput::MotorOutput(const uint8_t *pins, const uint8_t &count) :
		m_count(count),
		m_pins(pins) {}

void MotorOutput::set(const uint16_t &fl_pwr, const uint16_t &fr_pwr, const uint16_t &bl_pwr, const uint16_t &br_pwr) const {
	pwmWrite(m_pins[0], fl_pwr);
	pwmWrite(m_pins[1], fr_pwr);
	pwmWrite(m_pins[2], bl_pwr);
	pwmWrite(m_pins[3], br_pwr);
}
