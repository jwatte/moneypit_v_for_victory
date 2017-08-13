#include "global.h"
#include "Packets.h"



Decode::Decode(uint8_t const *ptr, uint8_t len) {
	ptr_ = ptr;
	len_ = len;
	ok_ = true;
}

void Decode::get(uint8_t &u8) {
	get(&u8, 1);
}

void Decode::get(uint16_t &u16) {
	get(&u16, 2);
}

void Decode::get(int16_t &s16) {
	get(&s16, 2);
}

void Decode::get(uint32_t &u32) {
	get(&u32, 4);
}

void Decode::get(int32_t &s32) {
	get(&s32, 4);
}

bool Decode::ok() {
	return ok_;
}

void Decode::get(void *b, uint8_t n) {
	if (!ok_) return;
	if (n > len_) {
		len_ = 0;
		ok_ = false;
		return;
	}
	//	assert this host is little endian
	for (uint8_t t = n; t != 0; --t) {
		((uint8_t *)b)[n - t] = ptr_[t - 1];
	}
	len_ -= n;
	ptr_ += n;
}




Encode::Encode(uint8_t *ptr, uint8_t size) {
	ptr_ = ptr;
	size_ = size;
	len_ = 0;
	ok_ = true;
}

void Encode::put(uint8_t const &u8) {
	put(&u8, 1);
}

void Encode::put(uint16_t const &u16) {
	put(&u16, 2);
}

void Encode::put(int16_t const &s16) {
	put(&s16, 2);
}

void Encode::put(uint32_t const &u32) {
	put(&u32, 4);
}

void Encode::put(int32_t const &s32) {
	put(&s32, 4);
}

bool Encode::ok() {
	return ok_;
}

uint8_t Encode::len() {
	return len_;
}

void Encode::put(void const *b, uint8_t n) {
	if (!ok_) return;
	if (n > size_) {
		size_ = 0;
		len_ = 0;
		ok_ = false;
		return;
	}
	//	assert this host is little endian
	for (uint8_t t = n; t != 0; --t) {
		ptr_[t - 1] = ((uint8_t const *)b)[n - t];
	}
	size_ -= n;
	len_ += n;
	ptr_ += n;
}

