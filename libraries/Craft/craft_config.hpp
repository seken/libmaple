#pragma once

enum CraftType {
	PQUAD,
	XQUAD,

	PHEX,
	XHEX,
	
	PLANE,
	
	TRI,

	HELI,
	
	CAR,

	BOAT,
};

struct ControlConfig {
	uint32_t p;
	uint32_t i;
	uint32_t d;
	uint32_t iLimit;
};

template<class T>
struct xyz {
	T x;
	T y;
	T z;
};

struct Config {
	static const uint8_t MAX_PINS = 8;

	CraftType type;
	
	uint8_t motorCount;
	uint8_t motorPins[MAX_PINS];

	uint8_t ledPin;

	struct {
		ControlConfig normal;
		ControlConfig rate;
	} control;

	struct {
		bool sensors;
	} log;

	uint32_t telemetryRate;

	struct {
		xyz<int16_t> dev_gyro_offset;
		xyz<int16_t> gyro_offset;
		xyz<int16_t> acc_offset;
	} cal;

	union {
	} typeSpecific;
};
