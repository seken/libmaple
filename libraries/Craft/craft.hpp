#pragma once

#include <cstdlib>
#include <craft_attitude.hpp>
#include <craft_control.hpp>
#include <craft_motors.hpp>
#include <craft_config.hpp>
#include <craft_mode.hpp>
#include <craft_demand.hpp>

class Craft {
	public:
		Craft(const Config &config);
		~Craft();

		void loop();
		void loop_vtol();
		void loop_plane();
		void loop_gnd();
		void setDemand(const Demand &demand);
		void checkBattery();

		const Mode getMode() const;

		void cal_GyroOffsets();
		void cal_GyroUserOffsets();
		void cal_AccelOffsets();

	protected:
		void blink_slow() const;
		void blink_fast() const;
		void blink_fault() const;
		void blink_mask(const uint16_t &mask) const;

		Mode m_current;
		Config m_config;
		MotorOutput m_motors;
		Control *m_currentControl;
		Control **m_controlSchemes;
		Attitude m_attitude;
		uint32_t m_telemetryLast;

		Demand m_userDemand;
};

