#include <Arduino.h>
#include "RoboClawControl.h"
#include "RoboClawBus.h"
#include "global.h"
#include "Alarm.h"


#define STATE_RESET_ENCODERS 32
#define BOOT_STATE 0
#define INIT_STATE 1
#define FIRST_DRIVE_STATE 2
#define POLL_STATE 3
#define STATUS_STATE 4

RoboClawControl::RoboClawControl(RoboClawBus &bus) : roboclaw_(bus) {
}

void RoboClawControl::init(uint32_t now) {
    state_ = BOOT_STATE;
    freshState_ = true;
    lastStep_ = now;
    lastDrive_ = now;
	enable_ = false;
    shouldResetEncoders_ = false;
    memset(motor_, 0, sizeof(motor_));
    memset(pos_, 0, sizeof(pos_));
    memset(status_, 0, sizeof(status_));
}

void RoboClawControl::forceInit() {
	enable_ = true;
}

void RoboClawControl::step(uint32_t now, uint32_t dus) {

	lastStep_ = now;

	switch (state_) {
    default:
	case BOOT_STATE:
		if (roboclaw_.awaitTimeout(500)) {
			setState(INIT_STATE);
		}
		break;
	case INIT_STATE:
		stepInit();
		break;
	case FIRST_DRIVE_STATE:
		if (!enable_) {
			setState(BOOT_STATE);
		}
		else {
			stepDrive();
		}
		break;
	case POLL_STATE:
		if (!enable_) {
			setState(BOOT_STATE);
		}
		else {
			stepPoll();
		}
		break;
	case STATUS_STATE:
		if (!enable_) {
			setState(BOOT_STATE);
		}
		else {
			stepStatus();
		}
		break;
    case STATE_RESET_ENCODERS:
        stepReset();
        break;
	}
}


void RoboClawControl::stepInit() {

	uint8_t const *ptr = NULL;
	uint8_t len = 0;

	switch (phase_) {
	case 1:
		if (roboclaw_.hasResponse(ptr, len)) {
			//	todo: should I print the version number?
			phase_++;
		}
		else if (roboclaw_.awaitTimeout(500)) {
			//	timeout! -- try again
			SERIALUSB.print("RC T/O ");
            SERIALUSB.println(id_, HEX);
			Alarm::blink(150);
			phase_--;
		}
		break;
	case 3:
	case 5:
	case 7:
	case 9:
		if (roboclaw_.hasAck()) {
			phase_++;
		}
		else if (roboclaw_.awaitTimeout(50)) {
			//	timeout!
			phase_++;
		}
		break;
	case 0:
		roboclaw_.getVersion(id_);
		phase_++;
		break;
	case 2:
		roboclaw_.setPID(id_, 0);
		phase_++;
		break;
	case 4:
		roboclaw_.setPID(id_, 1);
		phase_++;
		break;
	case 6:
		roboclaw_.resetEncoders(id_);
		phase_++;
		break;
	case 8:
		roboclaw_.setVoltage(id_, 6.0f, 18.0f);
		phase_++;
		break;
	default:
		id_++;
		phase_ = 0;
		if (id_ == 0x83) {
            if (shouldResetEncoders_) {
                setState(STATE_RESET_ENCODERS);
            } else {
                setState(FIRST_DRIVE_STATE);
            }
		}
		break;
	}
}


void RoboClawControl::stepDrive() {

	bool towrite = freshState_;
    freshState_ = false;

	if (roboclaw_.hasAck()) {
		towrite = true;
	}
    else if (roboclaw_.awaitTimeout(20)) {
		towrite = true;
	}
	if (towrite) {
		if (id_ == 0x83) {
			setState(state_+1);
		}
		else {
			roboclaw_.driveM1M2(id_, motor_[(id_ - 0x80)*2], motor_[(id_ - 0x80)*2 + 1]);
			++id_;
		}
	}
}

void RoboClawControl::stepPoll() {

	bool towrite = freshState_;
    freshState_ = false;
	uint8_t const *ptr;
	uint8_t len;

	if (roboclaw_.hasResponse(ptr, len)) {
		towrite = true;
		//	decode position for id_-1
		ASSERT(id_ >= 0x81 && id_ <= 0x83);
		if (len >= 8) {
			pos_[id_ - 0x81][0] = GET4B(ptr);
			pos_[id_ - 0x81][1] = GET4B(ptr+4);
		}
	}
    else if (roboclaw_.awaitTimeout(20)) {
		towrite = true;
	}
	if (towrite) {
		if (id_ == 0x83) {
			setState(state_+1);
		}
		else {
			roboclaw_.getPosition(id_);
			++id_;
		}
	}
}

void RoboClawControl::stepStatus() {

	bool towrite = freshState_;
    freshState_ = false;
	uint8_t const *ptr;
	uint8_t len;

	if (roboclaw_.hasResponse(ptr, len)) {
		towrite = true;
		//	decode status for id_-1
		ASSERT(id_ >= 0x81 && id_ <= 0x83);
		if (len >= 2) {
			status_[id_ - 0x81] = GET2B(ptr);
		}
	}
    else if (roboclaw_.awaitTimeout(20)) {
        if (id_ > 0x80) {
            //  Timed out! -- assume it's not there and sound the alarm
            ASSERT(id_ >= 0x81 && id_ <= 0x83);
            status_[id_ - 0x81] = 0xffff;
        }
        towrite = true;
	}
	if (towrite) {
		if (id_ == 0x83) {
			setState(FIRST_DRIVE_STATE);
		}
		else {
			roboclaw_.getPosition(id_);
			++id_;
		}
	}
}

void RoboClawControl::stepReset() {
	bool towrite = freshState_;
    freshState_ = false;

	if (roboclaw_.hasAck()) {
		towrite = true;
	}
    else if (roboclaw_.awaitTimeout(20)) {
        id_ = 0x80;
		towrite = true;
	}
	if (towrite) {
		if (id_ == 0x83) {
            shouldResetEncoders_ = false;
			setState(FIRST_DRIVE_STATE);
		}
		else {
			roboclaw_.resetEncoders(id_);
			++id_;
		}
	}
}


void RoboClawControl::drive(int32_t const *p) {
    lastDrive_ = lastStep_;
	motor_[0] = p[0];
	motor_[1] = p[1];
	motor_[2] = p[2];
	motor_[3] = p[3];
	motor_[4] = p[4];
	motor_[5] = p[5];
}

void RoboClawControl::setEnable(bool en) {
	enable_ = en;
}

void RoboClawControl::resetEncoders() {
    shouldResetEncoders_ = true;
}

void RoboClawControl::getPos(int ix, uint32_t p[2]) {
	p[0] = pos_[ix][0];
	p[1] = pos_[ix][1];
}

void RoboClawControl::getStatus(uint16_t o[3]) {
    memcpy(o, status_, 6);
}

void RoboClawControl::setState(uint8_t st) {
	state_ = st;
	phase_ = 0;
	id_ = 0x80;
    freshState_ = true;
    roboclaw_.resetTimeout();
}

