#include "craft_spektrum.hpp"

SpektrumDSMXSatelite::SpektrumDSMXSatelite(HardwareSerial *const input) :
		m_lastDemand({0, 0, 0, 0}),
		m_input(input) {}

#define SPEKTRUM_RECIEVER_THROTTLE 0
#define SPEKTRUM_RECIEVER_AILERON 1
#define SPEKTRUM_RECIEVER_ELEVATOR 2
#define SPEKTRUM_RECIEVER_RUDDER 3
#define SPEKTRUM_RECIEVER_GEAR 4
#define SPEKTRUM_RECIEVER_AUX1 5
#define SPEKTRUM_RECIEVER_AUX2 6
#define SPEKTRUM_RECIEVER_AUX3 7

Demand SpektrumDSMXSatelite::getDemand() {
	if (m_input->available() > 0) {
		// Read a frame

		m_input->read();
		m_input->read();

		for (uint8_t i = 0; i < 7; ++i) {
			SpektrumChannel channel;
			uint8_t *data = (uint8_t*) &channel;
			data[1] = m_input->read();
			data[0] = m_input->read();

			// TODO ranges, config, mapping
			switch (channel.id) {
				case SPEKTRUM_RECIEVER_THROTTLE:
					m_lastDemand.throttle = channel.value;
					break;

				default:
					break;
			}
		}
	}

	return m_lastDemand;
}

#define SPEKTRUM_RECIEVER_START 0x32
