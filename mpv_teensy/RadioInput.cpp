#include <Arduino.h>
#include "global.h"
#include "RadioInput.h"



PinPulseIn<PIN_PULSEIN_STATE>    RadioInput::pulseInState;
PinPulseIn<PIN_PULSEIN_THROTTLE> RadioInput::pulseInThrottle;
PinPulseIn<PIN_PULSEIN_STEER>    RadioInput::pulseInSteer;


RadioInput::RadioInput() {

}

void RadioInput::init(uint32_t now) {
	lastMeasurement_ = now;
	lastStep_ = now;
	pulseInState.init();
	pulseInThrottle.init();
	pulseInSteer.init();
}

void RadioInput::step(uint32_t now, uint32_t dus) {
	lastStep_ = now;
	if (pulseInState.hasValue() && pulseInThrottle.hasValue() && pulseInSteer.hasValue()) {
		pState = pulseInState.getValue();
		pThrottle = pulseInThrottle.getValue();
		pSteer = pulseInSteer.getValue();
		lastMeasurement_ = now;
	}
	else if ((int32_t)(lastStep_ - lastMeasurement_) > 1000000) {
		lastMeasurement_ = lastStep_ - 10000;
	}
	if (!connected()) {
		pState = pThrottle = pSteer = 0;
	}
}

bool RadioInput::connected() {
	return (int32_t)(lastStep_ - lastMeasurement_) < 100;
}

bool RadioInput::driveMode() {
	return pState > 1500;
}

uint16_t RadioInput::throttle() {
	return pThrottle;
}

uint16_t RadioInput::steer() {
	return pSteer;
}
