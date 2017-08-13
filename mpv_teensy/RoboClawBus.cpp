#include <Arduino.h>
#include "global.h"
#include "RoboClawBus.h"
#include "Alarm.h"


#define RCPORT serial_
#define BE4(d) \
	(uint8_t)((d) >> 24), (uint8_t)((d) >> 16), (uint8_t)((d) >> 8), (uint8_t)(d)


/* These constants are application specific, and shouldn't be in here
 * if I think of RoboClawBus as a re-usable component.
 */
#define PID_P 0x100000
#define PID_I 0x004000
#define PID_D 0x002000
#define QPPS (MOTOR_GEAR*MOTOR_RPM*MOTOR_CPR/60)
#define ACCEL (QPPS*2)

#define RC_BAUD_RATE 115200


RoboClawBus::RoboClawBus(HardwareSerial &serial) : serial_(serial) {
}

void RoboClawBus::init(uint32_t now) {
	RCPORT.begin(RC_BAUD_RATE);
	inPtr_ = 0;
	lastStep_ = now;
	lastWrite_ = now;
	ack_ = false;
	responseType_ = 0xff;
}

void RoboClawBus::forceInit(uint32_t now) {
	init(now);
	//	no particular additional work
}

void RoboClawBus::step(uint32_t ms, uint32_t dus) {

	lastStep_ = ms;

	while (RCPORT.available()) {
		if (inPtr_ == sizeof(inBuf_)) {
			inPtr_ = 0;
		}
		inBuf_[inPtr_] = RCPORT.read();
		if ((responseType_ == 0xff) && (inBuf_[inPtr_] == 0xff)) {
			ack_ = true;
		}
		++inPtr_;
	}
}

bool RoboClawBus::awaitTimeout(int32_t ms) {
	return (int32_t)(lastStep_ - lastWrite_) > ms;
}

void RoboClawBus::resetTimeout() {
    lastWrite_ = lastStep_;
}

void RoboClawBus::write(uint8_t const *ptr, uint8_t len) {
	lastWrite_ = lastStep_;
	ack_ = false;
	inPtr_ = 0;
	RCPORT.write(ptr, len);
	responseType_ = 0xff;
}

void RoboClawBus::getVersion(uint8_t id) {
	/*
	Send: [Address, 21]
	Receive: [Version..., 0, CRC(2)]
	*/
	uint8_t buf[] = {
		id, 21
	};
	crc_.clear();
	crc_.update(buf, sizeof(buf));
	write(buf, sizeof(buf));
	responseType_ = 0;
}

void RoboClawBus::getPosition(uint8_t id) {
	/*
	Send: [Address, 78]
	Receive: [ENC1(4), ENC2(4), CRC(2)]
	*/
	uint8_t buf[] = {
		id, 78
	};
	crc_.clear();
	crc_.update(buf, sizeof(buf));
	write(buf, sizeof(buf));
	responseType_ = 8;
}

void RoboClawBus::getStatus(uint8_t id) {
    /*
     Send: [Address, 90]
     Receive: [Status, CRC(2 bytes)]
     */
	uint8_t buf[] = {
		id, 90
	};
	crc_.clear();
	crc_.update(buf, sizeof(buf));
	write(buf, sizeof(buf));
	responseType_ = 2;
}

void RoboClawBus::driveM1M2(uint8_t id, int32_t m1, int32_t m2) {
	/*
	Send: [Address, 51, AccelM1(4 Bytes), SpeedM1(4 Bytes), DistanceM1(4 Bytes),
	AccelM2(4 Bytes), SpeedM2(4 bytes), DistanceM2(4 Bytes), Buffer, CRC(2 bytes)]
	Receive: [0xFF]
	*/
	uint8_t buf[] = {
		id, 51,
		BE4(ACCEL), BE4(m1), BE4(m1 / 2),
		BE4(ACCEL), BE4(m2), BE4(m2 / 2),
		1
	};
	crc_.clear();
	crc_.update(buf, sizeof(buf));
	write(buf, sizeof(buf));
	writeCrc();
}

void RoboClawBus::setPID(uint8_t id, uint8_t m1m2) {
	/*
	Send: [Address, 28, D(4 bytes), P(4 bytes), I(4 bytes), QPPS(4 byte), CRC(2 bytes)]
	Receive: [0xFF]
	*/
	uint8_t buf[] = {
		id, (uint8_t)((m1m2 & 1)+28), BE4(PID_D), BE4(PID_P), BE4(PID_I), BE4(QPPS)
	};
	crc_.clear();
	crc_.update(buf, sizeof(buf));
	write(buf, sizeof(buf));
	writeCrc();
}

void RoboClawBus::resetEncoders(uint8_t id) {
	/*
	Send: [Address, 20, CRC(2 bytes)]
	Receive : [0xFF]
	*/
	uint8_t buf[] = {
		id, 20
	};
	crc_.clear();
	crc_.update(buf, sizeof(buf));
	write(buf, sizeof(buf));
	writeCrc();
}

void RoboClawBus::setVoltage(uint8_t id, float low, float high) {
	/*
	Send: [Address, 57, Min(2 bytes), Max(2 bytes), CRC(2 bytes)]
	Receive: [0xFF]
	*/
	uint16_t lo = (uint16_t)(low * 10);
	uint16_t hi = (uint16_t)(high * 10);
	uint8_t buf[] = {
		id, 57, (uint8_t)(lo >> 8), (uint8_t)lo, (uint8_t)(hi >> 8), (uint8_t)hi
	};
	crc_.clear();
	crc_.update(buf, sizeof(buf));
	write(buf, sizeof(buf));
	writeCrc();
}

void RoboClawBus::writeCrc() {
	lastWrite_ = lastStep_;
	uint16_t crc = crc_.get();
	uint8_t cb[2] = { (uint8_t)(crc >> 8), (uint8_t)crc };
	RCPORT.write(cb, 2);
	crc_.clear();
	ack_ = false;
}



bool RoboClawBus::hasResponse(uint8_t const *&ptr, uint8_t &len) {
	if (inPtr_ < 3) {
		return false;
	}
	//	If there are more bytes on the bus after the CRC, this will fail.
	if (((responseType_ == 0) && (inBuf_[inPtr_ - 3] == 0))
			|| ((responseType_ != 0) && (inPtr_ == responseType_ + 2))) {
		uint16_t sav = crc_.get();
		crc_.update(inBuf_, inPtr_ - 2);
		uint16_t bc = ((uint16_t)inBuf_[inPtr_ - 2] << 8) | inBuf_[inPtr_ - 1];
		if (crc_.get() == bc) {
			inPtr_ = 0;
			ptr = inBuf_;
			len = inPtr_ - 3;
			return true;
		}
		else {
			SERIALUSB.print("CRC ");
			SERIALUSB.print(crc_.get(), HEX);
			SERIALUSB.print(" got ");
			SERIALUSB.println(bc, HEX);
			Alarm::blink(50);
		}
		crc_ = sav;
	}
    return false;
}

bool RoboClawBus::hasAck() {
	return ack_;
}

