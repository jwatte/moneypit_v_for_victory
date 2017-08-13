#if !defined(Packets_h)
#define Packets_h

#include <stdint.h>

/* Packet format of the protocol is:
 * 0xff <cmd> <len> <data:len bytes> <crchi> <crclo>
 * Packets are numbered 0x11 and up for Pi -> Teensy
 * and 0x91 and up for Teensy -> Pi.
 */

#define PACKET_SETINSTATE 0x11
#define HOST_MODE_RADIODRIVE 0
#define HOST_MODE_HOSTDRIVE 1
struct H2T_State {
	uint8_t mode;
	uint8_t version;
	uint16_t turn;
	uint16_t drive;

#define t(x) _t(x, #x)
	template<typename T> T &visit(T &_t) {
		t(mode);
		t(version);
		t(turn);
		t(drive);
		return _t;
	}
#undef t
};

#define PACKET_RESETENCODERS 0x12
struct H2T_ResetEncoders {

#define t(x) _t(x, #x)
	template<typename T> T &visit(T &_t) {
		return _t;
	}
#undef t
};

#define FLAG_CONNECTED  0x1
#define FLAG_DRIVEMODE  0x2
#define FLAG_LEARNMODE  0x4
#define FLAG_RACEMODE   0x8
#define FLAG_HOSTDRIVE 0x10
#define FLAG_SHUTDOWN  0x80

#define RESPONSE_SETOUTSTATE 0x91
struct T2H_State {
	uint32_t m1;
	uint32_t m2;
    uint16_t rstatus[3];

	uint16_t flags;

	uint8_t volt;
	uint8_t version;

	uint8_t dtemp[4];
	uint8_t dstatus[4];

#define t(x) _t(x, #x)
	template<typename T> T &visit(T &_t) {
		t(m1);
		t(m2);
        t(rstatus[0]);
        t(rstatus[1]);
        t(rstatus[2]);
		t(flags);
		t(volt);
		t(version);
		t(dtemp[0]);
		t(dtemp[1]);
		t(dtemp[2]);
		t(dtemp[3]);
		t(dstatus[0]);
		t(dstatus[1]);
		t(dstatus[2]);
		t(dstatus[3]);
		return _t;
	}
#undef t
};

#define RESPONSE_LINKSTATS 0x92
struct T2H_LinkStats {
    uint8_t numTotal;
    uint8_t numGood;
    uint8_t numInstate;
    uint8_t numUnknown;
#define t(x) _t(x, #x)
	template<typename T> T &visit(T &_t) {
        t(numTotal);
        t(numGood);
        t(numInstate);
        t(numUnknown);
		return _t;
	}
#undef t
};



class Decode {
public:
	Decode(uint8_t const *ptr, uint8_t len);
	void get(uint8_t &u8);
	void get(uint16_t &u16);
	void get(int16_t &s16);
	void get(uint32_t &u32);
	void get(int32_t &s32);
	bool ok();

	template<typename T> void operator()(T &t, char const *name) {
		get(t);
	}
private:
	uint8_t const *ptr_;
	uint8_t len_;
	bool ok_;
	void get(void *b, uint8_t n);
};

class Encode {
public:
	Encode(uint8_t *ptr, uint8_t size);
	void put(uint8_t const &u8);
	void put(uint16_t const &u16);
	void put(int16_t const &s16);
	void put(uint32_t const &u32);
	void put(int32_t const &s32);
	bool ok();
	uint8_t len();

	template<typename T> void operator()(T const &t, char const *name) {
		put(t);
	}
private:
	uint8_t *ptr_;
	uint8_t size_;
	uint8_t len_;
	bool ok_;
	void put(void const *b, uint8_t n);
};

#endif	//	Packets_h

