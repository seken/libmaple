#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "craft_attitude.hpp"
#include <MahonyAHRS.h>

Attitude::Attitude(Config *const config) :
		m_config(config),
		m_initialPressure(0),
		m_startTime(0),
		m_counter(0) {
	// Start barometer
	m_baro.init(MS561101BA_ADDR_CSB_LOW);
	
	// Start Accelerometer and Gyro package
	m_ags.initialize();
	m_ags.resetSensors();
	// Enable bypass for magnetometer
	m_ags.setI2CBypassEnabled(true);
	m_ags.setFullScaleAccelRange(2);
	m_ags.setFullScaleGyroRange(2);
	// Sample gyro at 1Khz
	m_ags.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	m_ags.setRate(7);

	// Apply precalculated gyro offsets
	m_ags.setXGyroOffsetUser(m_config->cal.dev_gyro_offset.x);
	m_ags.setYGyroOffsetUser(m_config->cal.dev_gyro_offset.y);
	m_ags.setZGyroOffsetUser(m_config->cal.dev_gyro_offset.z);

	// Start magnetometer
	m_mag.init(false);
}

Attitude::~Attitude() {}

void Attitude::calibrate() {
	for(int i = 0; i < 100; ++i) {
		float p = 0;
		while (p == 0) {
			p = m_baro.getPressure(MS561101BA_OSR_4096);
		}
		m_initialPressure += p;
	}
	m_initialPressure/= 100;

	m_mag.calibrate(1, 32);
	m_mag.setMode(0);
	delay(10);
	m_mag.setDOR(B110);
}

void Attitude::update() {
	float magx, magy, magz;
	m_mag.getValues(&magx, &magy, &magz);

	int16_t ax, ay, az, gx, gy, gz;
	m_ags.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	gx += m_config->cal.gyro_offset.x;
	gy += m_config->cal.gyro_offset.y;
	gz += m_config->cal.gyro_offset.z;
	
	const float fgx = float(gx)/1877.4681031;
	const float fgy = float(gy)/1877.4681031;
	const float fgz = float(gz)/1877.4681031;
	
	const float fax = ax + m_config->cal.acc_offset.x;
	const float fay = ay + m_config->cal.acc_offset.y;
	const float faz = az + m_config->cal.acc_offset.z;

	MahonyAHRSupdate(fgx, fgy, fgz, fax, fay, faz, magx, magy, magz);

	const float px = 2 * (q1*q3 - q0*q2);
	const float py = 2 * (q0*q1 + q2*q3);
	const float pz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	yaw = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0*q0 + 2 * q1 * q1 - 1) * 180/M_PI;
	pitch = atan2(px, sqrt(py*py + pz*pz))  * 180/M_PI;
	roll = atan2(py, sqrt(px*px + pz*pz))  * 180/M_PI;

	// Every 1/2 second update the sample rate for the gyro calculations
	++m_counter;
	const uint32_t end = micros();
	const uint32_t total = end - m_startTime;

	if (total > 100000) {
		sampleFreq = m_counter*10;
		m_counter = 0;
		m_startTime = end;
	}
}

float Attitude::getGroundHeight() {
	float pressure = 0;
	while (pressure == 0) {
		pressure = m_baro.getPressure(MS561101BA_OSR_4096);
	}
	return -(pressure-m_initialPressure) * 9.144;
}

void Attitude::cal_AccelOffsets() {
}

void Attitude::cal_GyroUserOffsets() {
	int64_t tx = 0;
	int64_t ty = 0;
	int64_t tz = 0;
	
	for(int i = 0; i < 5000; ++i) {
		int16_t ax, ay, az, gx, gy, gz;
		m_ags.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		tx += gx;
		ty += gy;
		tz += gz;
	}

	tx/=5000;
	ty/=5000;
	tz/=5000;

	SerialUSB.print("tx: ");
	SerialUSB.println(tx);
	SerialUSB.print("ty: ");
	SerialUSB.println(ty);
	SerialUSB.print("tz: ");
	SerialUSB.println(tz);

	SerialUSB.print("offx: ");
	SerialUSB.println(m_config->cal.dev_gyro_offset.x);
	SerialUSB.print("offy: ");
	SerialUSB.println(m_config->cal.dev_gyro_offset.y);
	SerialUSB.print("offz: ");
	SerialUSB.println(m_config->cal.dev_gyro_offset.z);

	if (tx > 0) {
		--m_config->cal.dev_gyro_offset.x;
	} else if (tx < 0) {
		++m_config->cal.dev_gyro_offset.x;
	}

	if (ty > 0) {
		--m_config->cal.dev_gyro_offset.y;
	} else if (ty < 0) {
		++m_config->cal.dev_gyro_offset.y;
	}

	if (tz > 0) {
		--m_config->cal.dev_gyro_offset.z;
	} else if (tz < 0) {
		++m_config->cal.dev_gyro_offset.z;
	}

	// Apply precalculated gyro offsets
	m_ags.setXGyroOffsetUser(m_config->cal.dev_gyro_offset.x);
	m_ags.setYGyroOffsetUser(m_config->cal.dev_gyro_offset.y);
	m_ags.setZGyroOffsetUser(m_config->cal.dev_gyro_offset.z);
}

void Attitude::cal_GyroOffsets() {
	int64_t tx = 0;
	int64_t ty = 0;
	int64_t tz = 0;
	
	for(int i = 0; i < 5000; ++i) {
		int16_t ax, ay, az, gx, gy, gz;
		m_ags.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		tx += gx + m_config->cal.gyro_offset.x;
		ty += gy + m_config->cal.gyro_offset.y;
		tz += gz + m_config->cal.gyro_offset.z;
	}

	tx/=5000;
	ty/=5000;
	tz/=5000;

	SerialUSB.print("tx: ");
	SerialUSB.println(tx);
	SerialUSB.print("ty: ");
	SerialUSB.println(ty);
	SerialUSB.print("tz: ");
	SerialUSB.println(tz);

	SerialUSB.print("offx: ");
	SerialUSB.println(m_config->cal.gyro_offset.x);
	SerialUSB.print("offy: ");
	SerialUSB.println(m_config->cal.gyro_offset.y);
	SerialUSB.print("offz: ");
	SerialUSB.println(m_config->cal.gyro_offset.z);

	if (tx > 0) {
		--m_config->cal.gyro_offset.x;
	} else if (tx < 0) {
		++m_config->cal.gyro_offset.x;
	}

	if (ty > 0) {
		--m_config->cal.gyro_offset.y;
	} else if (ty < 0) {
		++m_config->cal.gyro_offset.y;
	}

	if (tz > 0) {
		--m_config->cal.gyro_offset.z;
	} else if (tz < 0) {
		++m_config->cal.gyro_offset.z;
	}
}
