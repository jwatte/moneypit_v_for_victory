#include <Arduino.h>
#include <limits.h>
#include "Alarm.h"
#include "global.h"


Alarm *Alarm::alarm_;

Alarm::Alarm() {
	alarm_ = this;
}

void Alarm::init(uint32_t now) {
	blinkUntil_ = now;
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
}

void Alarm::step(uint32_t now, uint32_t dus) {
	int32_t timeLeft = (int32_t)(blinkUntil_ - now);
	if (timeLeft > 0) {
		digitalWrite(LED_PIN, ((now & 96) == 96) ? LOW : HIGH);
	}
	else {
		blinkUntil_ = now;
		digitalWrite(LED_PIN, LOW);
	}
	lastStep_ = now;
}

void Alarm::setLed(bool on) {
	blinkUntil_ = on ? LONG_MAX : alarm_->lastStep_;
	digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void Alarm::blink(uint32_t duration) {
	digitalWrite(LED_PIN, HIGH);
	int32_t left = (int32_t)(alarm_->blinkUntil_ - alarm_->lastStep_);
	if (left < (int32_t)duration) {
		alarm_->blinkUntil_ = alarm_->lastStep_ + duration;
	}
}
