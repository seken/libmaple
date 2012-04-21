#pragma once

#include <craft_demand.hpp>
#include <wirish.h>

class SpektrumDSMXSatelite {
	public:
		SpektrumDSMXSatelite(HardwareSerial *const input);
		Demand getDemand();

	private:
		Demand m_lastDemand;
		HardwareSerial *const m_input;
};

struct SpektrumChannel {
	unsigned value:11;
	unsigned id:4;
	unsigned frameLoss:1;
};

class SpektrumReciever {
	public:
		SpektrumReciever(HardwareSerial *const input);
		bool update();

		uint8_t holds;
		uint16_t frameLoss;
		uint16_t receiverFade;
		uint8_t unknownA[2];
		uint16_t remoteFade;
		uint8_t unknownB[6];
	
	private:
		HardwareSerial *const m_input;
};
