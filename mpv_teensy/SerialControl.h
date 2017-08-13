#if !defined(SerialControl_h)
#define SerialControl_h

#include "global.h"
#include "Packets.h"

/*
protocol:
 0xff    1
 cmd     1
 length  1
 data    length
 CRC-16  high, low (including ff ... end data)
 */

#define CTRLPORT SERIALUSB

class SerialControl {
public:
	SerialControl();
	void init(uint32_t now);
	void step(uint32_t now, uint32_t dus);
	T2H_State &update();
	H2T_State const &get();
	bool hasInState();
    bool testClearResetEncoders();
private:
	uint32_t lastStep_;
	uint32_t lastIn_;
	uint32_t lastOut_;
	uint8_t inBuf_[32];
	uint8_t inPtr_;
    uint8_t numGood_;
    uint8_t numTotal_;
    uint8_t numInstate_;
    uint8_t numUnknown_;
    bool gotResetEncoders_;

	T2H_State outState_;
	H2T_State inState_;

	void slideBuf();
	void parsePacket();
	void unknown();
	void textStatus();
};

#endif	//	SerialControl_h
