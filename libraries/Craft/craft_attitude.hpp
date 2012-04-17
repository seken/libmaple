#pragma once

#include <MS561101BA.h>
#include <MPU6050.h>
#include <HMC58x3.h>
#include <craft_config.hpp>

class Attitude {
	public:
		Attitude(Config *const config);
		~Attitude();

		void calibrate();
		
		void update();
		float getGroundHeight();

		void cal_GyroOffsets();
	
		float yaw;
		float pitch;
		float roll;

	private:
		Config *const m_config;
		MS561101BA m_baro;
		MPU6050 m_ags;
		HMC58X3 m_mag;
		float m_initialPressure;
		uint32_t m_startTime;
		int32_t m_counter;
};
