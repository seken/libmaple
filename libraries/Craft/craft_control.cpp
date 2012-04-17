#include "craft_control.hpp"
#include "craft_util.hpp"

Control::Control(MotorOutput *const output, Demand *const demand, Attitude *const attitude) :
		m_output(output),
		m_demand(demand),
		m_attitude(attitude) {}

XQuadCopterNormalControl::XQuadCopterNormalControl(Config *const cfg, MotorOutput *const output, Demand *const demand, Attitude *const attitude) :
		Control(output, demand, attitude),
		m_pitch(cfg->control.normal.p, cfg->control.normal.i, cfg->control.normal.d, cfg->control.normal.iLimit),
		m_roll(cfg->control.normal.p, cfg->control.normal.i, cfg->control.normal.d, cfg->control.normal.iLimit),
		m_yaw(cfg->control.normal.p, cfg->control.normal.i, cfg->control.normal.d, cfg->control.normal.iLimit) {}

XQuadCopterNormalControl::~XQuadCopterNormalControl() {}

XQuadCopterNormalControl *const XQuadCopterNormalControl::get(Config *const cfg, MotorOutput *const output, Demand *const demand, Attitude *const attitude) {
	static XQuadCopterNormalControl singleton(cfg, output, demand, attitude);
	return &singleton;
}

void XQuadCopterNormalControl::reset() {
	m_pitch.reset();
	m_roll.reset();
	m_yaw.reset();
}

void XQuadCopterNormalControl::update() {
	int32_t pitchPower = m_pitch.calculate(m_demand->pitch, m_attitude->pitch*1000);
	int32_t rollPower = m_roll.calculate(m_demand->roll, m_attitude->roll*1000);
	int32_t yawPower = m_yaw.calculate(m_demand->yaw, m_attitude->yaw*1000);

	int32_t frontLeft = m_demand->throttle +pitchPower +rollPower +yawPower;
	int32_t frontRight = m_demand->throttle +pitchPower -rollPower -yawPower;
	int32_t backLeft = m_demand->throttle -pitchPower +rollPower -yawPower;
	int32_t backRight = m_demand->throttle -pitchPower -rollPower +yawPower;

	const int32_t max = MAX(frontLeft, frontRight, backLeft, backRight);
	if (max > UINT16_MAX) {
		const int32_t min = MIN(frontLeft, frontRight, backLeft, backRight);
		const int32_t diff = max - UINT16_MAX;
		if (min < diff) {
			// FIXME scale down
		} else {
			frontLeft -= diff;
			frontRight -= diff;
			backLeft -= diff;
			backRight -= diff;
		}
	}

	m_output->set(frontLeft, frontRight, backLeft, backRight);
}
