#include "craft.hpp"
#include <wirish.h>

Craft::Craft(const Config &config) :
		m_current(STOPPED),
		m_config(config),
		m_motors(m_config.motorPins, m_config.motorCount),
		m_attitude(&m_config),
		m_userDemand({0, 0, 0, 0}) {
    pinMode(m_config.ledPin , OUTPUT);

	// setup control
	switch(m_config.type) {
		case XQUAD:
			m_controlSchemes = (Control**) malloc(sizeof(Control*)*3);
			m_controlSchemes[0] = XQuadCopterNormalControl::get(&m_config, &m_motors, &m_userDemand, &m_attitude);
			// TODO rate based control
			break;

		default:
			while (true) {
				blink_fault();
			}
	}
}

Craft::~Craft() {
}

void Craft::loop() {
	m_attitude.update();
	checkBattery();

	uint32_t time = micros();
	if (time - m_telemetryLast > m_config.telemetryRate) {
		m_telemetryLast = time;
		SerialUSB.print(m_attitude.roll);
		SerialUSB.print(",");
		SerialUSB.print(m_attitude.pitch);
		SerialUSB.print(",");
		SerialUSB.print(m_attitude.yaw);
		SerialUSB.println(", 0, 0, 0, 0");
	}

	switch (m_config.type) {
		case PQUAD:
		case XQUAD:
		case PHEX:
		case XHEX:
		case TRI:
		case HELI:
			loop_vtol();
			break;
		
		case PLANE:
			loop_plane();
			break;

		case CAR:
		case BOAT:
			loop_gnd();
			break;

		default:
			blink_fault();
	}
}

void Craft::loop_vtol() {
	switch (m_current) {
		case STOPPED:
			m_attitude.calibrate();
			m_current = STOPPED_READY;

		case STOPPED_READY:
			blink_slow();

			if (m_userDemand.throttle >= 2000) {
				m_current = STOPPED_STARTUP;
			}
			break;

		case STOPPED_STARTUP:
			if (m_userDemand.throttle <= 48) {
				m_current = NORMAL;
			}
			break;

		case LAUNCH:
			break;

		case NORMAL:
			blink_fast();
			m_currentControl = m_controlSchemes[0];
			m_currentControl->update();

			// TODO check for switch to/from state
			break;

		case HOLD:
			blink_fast();
			m_currentControl = m_controlSchemes[1];
			m_currentControl->update();

			// TODO check for switch from/to state
			break;

		case ACRO:
			blink_fast();
			m_currentControl = m_controlSchemes[2];
			m_currentControl->update();

			// TODO
			break;

		case WAYPOINT:
			blink_fast();

			m_currentControl = m_controlSchemes[3];
			m_currentControl->update();

			// TODO
			break;

		case RETURN_HOME:
			blink_fast();

			// TODO set coords

			m_currentControl = m_controlSchemes[3];
			m_currentControl->update();

			// TODO
			break;

		case LOW_BATTERY:
			// TODO pulse motors, normal control
			break;

		case STOP:
			// TODO
			break;

		case REPLAY:
			// TODO implement these modes
			break;

		case FAULT:
		default:
			// TODO power off
			m_motors.set(0, 0, 0, 0);
			blink_fault();
			break;
	}
}

void Craft::loop_plane() {
}

void Craft::loop_gnd() {
}

void Craft::setDemand(const Demand &demand) {
	m_userDemand = demand;
}

void Craft::blink_mask(const uint16_t &mask) const {
	if ((millis() & mask) != 0) {
		digitalWrite(m_config.ledPin, HIGH);
	} else {
		digitalWrite(m_config.ledPin, LOW);
	}
}

void Craft::blink_slow() const {
	blink_mask(512);
}

void Craft::blink_fast() const {
	blink_mask(128);
}

void Craft::blink_fault() const {
	blink_mask(128 + 512);
}

void Craft::cal_GyroOffsets() {
	m_attitude.cal_GyroOffsets();
}

void Craft::cal_GyroUserOffsets() {
	m_attitude.cal_GyroUserOffsets();
}

void Craft::cal_AccelOffsets() {
	m_attitude.cal_AccelOffsets();
}

const Mode Craft::getMode() const {
	return m_current;
}

void Craft::checkBattery() {
}
