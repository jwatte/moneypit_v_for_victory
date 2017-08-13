#include <stdint.h>
#include "CRC.h"
#include "global.h"


CRC16::CRC16() {
	crc_ = 0;
}

CRC16::CRC16(void const *b, size_t sz) {
	crc_ = 0;
	ASSERT(sz <= 255);
	update((uint8_t const *)b, (uint8_t)sz);
}

CRC16 &CRC16::operator=(uint16_t e) {
	crc_ = e;
	return *this;
}

void CRC16::update(uint8_t const * src, uint8_t len) {
	uint16_t crc = crc_;
	for (uint8_t i = 0; i != len; ++i) {
		uint8_t data = src[i];
		crc = crc ^ ((uint16_t)data << 8);
		//	todo: turn this into a table
		for (int i = 0; i < 8; i++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			}
			else {
				crc <<= 1;
			}
		}
	}
	crc_ = crc;
}

uint16_t CRC16::get() {
	return crc_;
}

void CRC16::clear() {
	crc_ = 0;
}
