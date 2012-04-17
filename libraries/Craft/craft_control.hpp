#pragma once

#include <craft_demand.hpp>
#include <craft_motors.hpp>
#include <craft_pid.hpp>
#include <craft_attitude.hpp>

class Control {
	public:
		Control(MotorOutput *const output, Demand *const demand, Attitude *const attitude);
		virtual ~Control() {};

		virtual void reset() = 0;
		virtual void update() = 0;

	protected:
		MotorOutput *const m_output;
		Demand *const m_demand;
		Attitude *const m_attitude;
};

class XQuadCopterNormalControl : public Control {
	public:
		XQuadCopterNormalControl(Config *const cfg, MotorOutput *const output, Demand *const demand, Attitude *const attitude);
		~XQuadCopterNormalControl();

		static XQuadCopterNormalControl *const get(Config *const cfg=0, MotorOutput *const output=0, Demand *const demand=0, Attitude *const attitude=0);

		void reset();
		void update();

	protected:
		PID<int32_t> m_pitch;
		PID<int32_t> m_roll;
		PID<int32_t> m_yaw;
};
