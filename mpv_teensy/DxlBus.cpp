#include <Arduino.h>
#include "global.h"
#include "DxlBus.h"
#include "Alarm.h"

#define DXLPORT serial_


DxlBus::DxlBus(HardwareSerial &serial) : serial_(serial) {
    clear();
    lastStep_ = lastWrite_ = 0;
}

void DxlBus::init(uint32_t now) {
    lastStep_ = lastWrite_ = now;
    pinMode(PIN_DIRECTION, OUTPUT);
    digitalWrite(PIN_DIRECTION, LOW);
    DXLPORT.begin(1000000);
    DXLPORT.transmitterEnable(PIN_DIRECTION);
//	DXLPORT.write((uint8_t const *)"Hello, World!\n", 14);
    memset(&status_, 0, sizeof(status_));
}

void DxlBus::forceInit() {
	init(0);
	//	no particular extra work
}

void DxlBus::step(uint32_t nowms, uint32_t dus) {
    lastStep_ = nowms;
    while (DXLPORT.available()) {
        if (inPtr_ == sizeof(inBuf_)) {
            //  overflow of buffer!
            inPtr_ = 0;
        }
        inBuf_[inPtr_] = DXLPORT.read();
        if (inPtr_ || (inBuf_[0] == 0xff)) {
			//	received an FF in position 2? that's not right
			if (inPtr_ == 2 && inBuf_[2] == 0xff) {
				//	do I already have FF FF or is this a first FF? (0 is always FF)
				if (inBuf_[1] != 0xff) {
					inPtr_ = 1;
				}	//	else stay as 2
			}
			else {
				++inPtr_;
			}
        }
    }
}

void DxlBus::write(uint8_t const *wrdata, uint8_t wrsize) {
    DXLPORT.write(wrdata, wrsize);
	clear();
}

ResponseKind DxlBus::hasResponse(uint8_t id, uint8_t const *&ptr, uint8_t &len) {
    if (inPtr_ < 5) {
        return ResponseWaiting;
    }
    unsigned char plen = inBuf_[3] + 4;
    if (inPtr_ < plen) {
        return ResponseWaiting;
    }
    ASSERT(id >= 2 && id < 6);
    uint8_t cks = checksum(&inBuf_[2], inBuf_[3] + 1);
    if ((inBuf_[2] == id) && (cks == inBuf_[plen-1])) {
        status_.error[id-2] = inBuf_[4];
        len = inBuf_[3] - 1;
        ptr = &inBuf_[4];
		clear();
		return ResponseReceived;
    }
    SERIALUSB.print(cks, HEX);
    SERIALUSB.print(" != ");
    for (int i = 0; i != inBuf_[3]+4; ++i) {
        SERIALUSB.print(" ");
        SERIALUSB.print(inBuf_[i], HEX);
    }
    SERIALUSB.println("!");
	Alarm::blink(40);
    //  bad checksum
    status_.error[id - 2] |= DXL_STATUS_CHECKSUM_ERROR;
    inPtr_ = 0;
    return ResponseCorrupt;
}

bool DxlBus::awaitTimeout(int32_t diff) {
    if ((int32_t)(lastStep_ - lastWrite_) >= diff) {
        clear();
        return true;
    }
    return false;
}

void DxlBus::clear() {
    inPtr_ = 0;
    lastWrite_ = lastStep_;
}

uint8_t DxlBus::checksum(uint8_t const *ptr, uint8_t len) {
  uint8_t cs = 0;
  while (len-- > 0) {
    cs += *(ptr++);
  }
  return ~cs;
}

DxlStatus &DxlBus::status() {
    return status_;
}
