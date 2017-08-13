#include <Arduino.h>
#include "PowerControl.h"
#include "global.h"
#include "Alarm.h"


bool PowerControl::externalEstopRequested_;
uint32_t PowerControl::pendingShutdown_;


PowerControl::PowerControl() {
}

void PowerControl::init(uint32_t now) {
	pinMode(PIN_VSENSE, INPUT);
	pinMode(PIN_A_VSENSE, INPUT);
	pinMode(PIN_ESTOP, INPUT_PULLUP);
	pinMode(PIN_RPI_5VCTL, OUTPUT);
	digitalWrite(PIN_RPI_5VCTL, LOW);
	pinMode(PIN_MOTCTL, OUTPUT);
	digitalWrite(PIN_MOTCTL, LOW);
	pinMode(PIN_PWR_ON, OUTPUT);
	digitalWrite(PIN_PWR_ON, LOW);
	lastBad_ = now;
	lastOk_ = now;
	lastNonEstop_ = now;
	voltage_ = 0;
	enableMain_ = false;
	enable5V_ = false;
	enableMotor_ = false;
	isEstop_ = false;
	externalEstopRequested_ = false;
	analogReadRes(12);
    pendingShutdown_ = 0;
}

void PowerControl::step(uint32_t now, uint32_t dus) {
    if (pendingShutdown_ == 1) {
        pendingShutdown_ = now + 8000;
    }
    if ((pendingShutdown_ > 1) && ((int32_t)(now - pendingShutdown_) > 0)) {
        externalEstopRequested_ = true;
    }
	readVoltage();
	if ((digitalRead(PIN_ESTOP) == HIGH) && !externalEstopRequested_) {
		lastNonEstop_ = now;
	}
	if ((int32_t)(now - lastNonEstop_) > 100) {
		if (!isEstop_) {
			SERIALUSB.println("ESTOP");
		}
		externalEstopRequested_ = false;
		isEstop_ = true;
		enableMain_ = false;
		enableMotor_ = false;
		enable5V_ = false;
		digitalWrite(PIN_MOTCTL, LOW);
		digitalWrite(PIN_RPI_5VCTL, LOW);
		digitalWrite(PIN_PWR_ON, LOW);
	}
	else if (voltage_ >= 9.8f) {
		lastOk_ = now;
		if ((int32_t)(now - lastBad_) > 1000) {
			enableMain_ = true;
			enableMotor_ = true;
			enable5V_ = true;
			digitalWrite(PIN_PWR_ON, HIGH);
			digitalWrite(PIN_MOTCTL, HIGH);
			digitalWrite(PIN_RPI_5VCTL, HIGH);
		}
	}
	else if (voltage_ < 9.8f) {
		lastBad_ = now;
		if (!isEstop_) {
			Alarm::blink(400);
		}
		if ((int32_t)(now - lastOk_) > 1000) {
			if (enableMotor_) {
				SERIALUSB.println("MOTOR OFF");
				enableMotor_ = false;
			}
			digitalWrite(PIN_MOTCTL, LOW);
		}
		if (voltage_ <= 9.6f) {
			if ((int32_t)(now - lastOk_) > 2000) {
				SERIALUSB.println("POWER OFF");
				enableMain_ = false;
				enable5V_ = false;
				digitalWrite(PIN_RPI_5VCTL, LOW);
				digitalWrite(PIN_PWR_ON, LOW);
			}
		}
	}
	if (isEstop_ && (digitalRead(PIN_ESTOP) == HIGH)) {
        SERIALUSB.println("reboot()");
        delay(400);
		reboot();
	}
}

void PowerControl::readVoltage() {
	voltage_ = analogRead(PIN_A_VSENSE) * 18.4f / 4096;
}

float PowerControl::getVoltage() {
	return voltage_;
}

bool PowerControl::voltageOk() {
	return (voltage_ >= 9.8f && voltage_ < 12.6f);
}

bool PowerControl::getMainEnabled() {
	return enableMain_;
}

bool PowerControl::get5VEnabled() {
	return enable5V_;
}

bool PowerControl::getMotorEnabled() {
	return enableMotor_;
}

void PowerControl::requestEstop() {
	if (!externalEstopRequested_) {
		SERIALUSB.println("XESTOP");
	}
	externalEstopRequested_ = true;
}

void PowerControl::requestShutdown() {
    if (pendingShutdown_ == 0) pendingShutdown_ = 1;
}

bool PowerControl::pendingShutdown() {
    return pendingShutdown_ != 0;
}


