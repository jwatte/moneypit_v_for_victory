#if !defined(RadioInput_h)
#define RadioInput_h

#include "PinPulseIn.h"

class RadioInput {
public:
	RadioInput();
	void init(uint32_t now);
	void step(uint32_t now, uint32_t dus);
	bool connected();
	bool driveMode();
	uint16_t throttle();
	uint16_t steer();

private:
	static PinPulseIn<PIN_PULSEIN_STATE>    pulseInState;
	static PinPulseIn<PIN_PULSEIN_THROTTLE> pulseInThrottle;
	static PinPulseIn<PIN_PULSEIN_STEER>    pulseInSteer;

	uint32_t lastMeasurement_;
	uint32_t lastStep_;
	int16_t pState;
	int16_t pThrottle;
	int16_t pSteer;
};

#endif	//	RadioInput_h

