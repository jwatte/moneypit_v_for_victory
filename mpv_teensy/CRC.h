#if !defined(CRC_h)
#define CRC_h

#include <stdlib.h>
#include <stdint.h>

class CRC16 {
public:
	CRC16();
	CRC16(void const *buf, size_t sz);
	CRC16 &operator=(uint16_t e);
	void update(uint8_t const * src, uint8_t len);
	uint16_t get();
	void clear();

	uint16_t crc_;
};

#endif	//	CRC_h
