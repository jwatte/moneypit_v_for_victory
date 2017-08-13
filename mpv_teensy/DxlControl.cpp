#include <Arduino.h>
#include "DxlControl.h"
#include "DxlBus.h"
#include "Alarm.h"

#define DXL_RETURN_DELAY_TIME 5
#define DXL_CW_ANGLE_LIMIT 6
#define DXL_CCW_ANGLE_LIMIT 8
#define DXL_MAX_TORQUE 14
#define DXL_STATUS_RETURN_LEVEL 16
#define DXL_ALARM_LED 17
#define DXL_ALARM_SHUTDOWN 18
#define DXL_TORQUE_ENABLE 24
#define DXL_LED 25
#define DXL_GOAL_POSITION 30
#define DXL_MOVING_SPEED 32
#define DXL_TORQUE_LIMIT 34
#define DXL_PRESENT_VOLTAGE 42
#define DXL_LOCK 47

#define DXL_CMD_READ 2
#define DXL_CMD_WRITE 3


const DxlControl::Regs DxlControl::regs_[12] = {
    //  set return level first, so that I actually get out of the loop
    //  of wait-for-acknowledge in the init phase.
    { DXL_STATUS_RETURN_LEVEL, { 2 }, 1 },
	{ DXL_LED, { 1 }, 1 },
	{ DXL_RETURN_DELAY_TIME, { 0 }, 1 },   //  see if the vampire pulse can be kicked
    { DXL_CW_ANGLE_LIMIT, { 0, 1 }, 2 },
    { DXL_CCW_ANGLE_LIMIT, { 0, 3 }, 2 },
    { DXL_MAX_TORQUE, { 0xff, 3 }, 2 },
    { DXL_ALARM_LED, { 0x7f, 0 }, 1 },
    { DXL_ALARM_SHUTDOWN, { 0x24, 0 }, 1 },
    { DXL_TORQUE_LIMIT, { 0xff, 3 }, 2 },
    { DXL_LOCK, { 1 }, 1 },
    { DXL_TORQUE_ENABLE, { 1, 0 }, 1 },
	{ DXL_LED, { 0 }, 1 },
};
const uint8_t DxlControl::ids_[4] = {
    2, 3, 4, 5
};

DxlControl::DxlControl(DxlBus &bus) : bus_(bus) {
}

void DxlControl::init(uint32_t now) {
    for (int i = 0; i != 4; ++i) {
        posVel_[i][0] = 0;
        posVel_[i][1] = 2;  //  position 512
        posVel_[i][2] = 0;
        posVel_[i][3] = 3;  //  velocity 768
    }
    state_ = 0;
    id_ = 0;
    reg_ = 0;
    phase_ = 0;
    lastStep_ = now;
	enable_ = false;
}

void DxlControl::forceInit(uint32_t now) {
	init(now);
	state_ = 2;
	enable_ = true;
}

void DxlControl::step(uint32_t ms, uint32_t dus) {

	lastStep_ = ms;
	if ((state_ > 0) && !enable_) {
		setState(0);
	}
    else switch (state_) {
        case 0:
            //  wait a second during startup
            if (bus_.awaitTimeout(1000)) {
                setState(1);
            }
            break;
        case 1:
            updateInit();
            break;
        case 2:
            updateRun();
            break;
    }
}

//	512 is approximately the max speed AX-12A can turn freely.
//	The AX-18 turns faster, if I were ever to upgrade to that.
#define STEER_SPEED 768

static inline void rToDxl(float r, uint16_t s, uint8_t o[4]) {
	if (r < M_PI * -0.5f) {
		r = M_PI * -0.5f;
	}
	else if (r > M_PI * 0.5f) {
		r = M_PI * 0.5f;
	}
	int16_t ret = (int16_t)(r * 195.4f + 511.5f);
	o[0] = uint8_t(ret);
	o[1] = uint8_t(ret >> 8);
	o[2] = uint8_t(STEER_SPEED);
	o[3] = uint8_t(STEER_SPEED >> 8);
}

void DxlControl::setTargetAngles(float leftRadians, float rightRadians) {
	rToDxl(leftRadians, STEER_SPEED, posVel_[0]);
	rToDxl(rightRadians, STEER_SPEED, posVel_[1]);
	rToDxl(-leftRadians, STEER_SPEED, posVel_[2]);
	rToDxl(-rightRadians, STEER_SPEED, posVel_[3]);
}

int16_t DxlControl::inspectAngle(uint8_t index) {
	return (int16_t)(posVel_[index][0] | ((uint16_t)posVel_[index][1] << 8));
}

void DxlControl::setState(uint8_t state) {
	if (state_ != state) {
		SERIALUSB.print("DXL state => ");
		SERIALUSB.println(state, DEC);
	}
    state_ = state;
    id_ = 0;
    reg_ = 0;
    phase_ = 0;
}

void DxlControl::updateInit() {

    uint8_t const *ptr = NULL;
    uint8_t len = 0;
    ResponseKind rk;

    switch (phase_) {
        case 0:
            writeReg(ids_[id_], regs_[reg_].reg, regs_[reg_].data, regs_[reg_].n);
            ++phase_;
            break;
        case 1:
            if ((rk = bus_.hasResponse(ids_[id_], ptr, len)) == ResponseReceived) {
                ++phase_;
            }
			else if (bus_.awaitTimeout(500)) {
				char msg[16] = "Timeout ID ";
				msg[11] = '0' + ids_[id_];
				SERIALUSB.println(msg);
				//	timeout!
				Alarm::blink(60);
                phase_ = 0;
			}
            else if (rk == ResponseCorrupt) {
                //  try sending again
                phase_ = 0;
            }
            break;
        case 2:
            ++reg_;
            phase_ = 0;
            if (reg_ == (uint8_t)(sizeof(regs_)/sizeof(regs_[0]))) {
                reg_ = 0;
                ++id_;
                if (id_ == (uint8_t)(sizeof(ids_)/sizeof(ids_[0]))) {
                    id_ = 0;
                    setState(2);
                }
            }
            break;
    }
}

void DxlControl::updateRun() {
    uint8_t const *ptr = NULL;
    uint8_t len = 0;
    ResponseKind rk;

	if (id_ == 0 && phase_ == 0) {
		if (!bus_.awaitTimeout(15)) {
			return;
		}
	}
    switch (phase_) {
        default:
            phase_ = 0;
            ++id_;
            if (id_ == (uint8_t)(sizeof(ids_)/sizeof(ids_[0]))) {
                id_ = 0;
            }
        case 0:
            writeReg(ids_[id_], DXL_GOAL_POSITION, &posVel_[id_][0], 4);
            ++phase_;
            break;
        case 2:
            readReg(ids_[id_], DXL_PRESENT_VOLTAGE, 2);
			++phase_;
            break;
        case 1:
        case 3:
            if ((rk = bus_.hasResponse(ids_[id_], ptr, len)) == ResponseReceived) {
                parseResponse(ptr, len);
                ++phase_;
            }
            else if (bus_.awaitTimeout(200)) {
                servoTimeout();
                ++phase_;
            }
            else if (rk == ResponseCorrupt) {
                //  try again!
                --phase_;
            }
            break;
    }
}

void DxlControl::parseResponse(uint8_t const *data, uint8_t len) {
    switch (phase_) {
        case 1: //  write command
            bus_.status().error[id_] = data[0];
            break;
        case 3: //  read
            bus_.status().error[id_] = data[0];
            bus_.status().voltage[id_] = data[1];
            bus_.status().temperature[id_] = data[2];
            break;
    }
}

void DxlControl::servoTimeout() {
    bus_.status().error[id_] |= 0x80;
}

void DxlControl::writeReg(uint8_t id, uint8_t reg, uint8_t const *data, uint8_t n) {
    buf_[0] = 0xff;
    buf_[1] = 0xff;
    buf_[2] = id;
    buf_[3] = n + 3;
    buf_[4] = DXL_CMD_WRITE;
    buf_[5] = reg;
    for (uint8_t i = 0; i != n; ++i) {
        buf_[6 + i] = data[i];
    }
    buf_[6 + n] = DxlBus::checksum(&buf_[2], n + 4);
    bus_.write(buf_, 7+n);
}

void DxlControl::readReg(uint8_t id, uint8_t reg, uint8_t len) {
    buf_[0] = 0xff;
    buf_[1] = 0xff;
    buf_[2] = id;
	buf_[3] = 4;
    buf_[4] = DXL_CMD_READ;
    buf_[5] = reg;
    buf_[6] = len;
    buf_[7] = DxlBus::checksum(&buf_[2], 5);
    bus_.write(buf_, 8);
}

void DxlControl::setEnable(bool en) {
	if (enable_ != en) {
		SERIALUSB.println(en ? "DXL Enable" : "DXL Disable");
	}
	enable_ = en;
}