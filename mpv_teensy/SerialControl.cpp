#include <Arduino.h>
#include "global.h"
#include "SerialControl.h"
#include "CRC.h"
#include "Alarm.h"


SerialControl::SerialControl() {
	
}

void SerialControl::init(uint32_t now) {
	lastStep_ = now;
	lastIn_ = now;
	lastOut_ = now;
	inPtr_ = 0;
    gotResetEncoders_ = false;
	memset(&inState_, 0, sizeof(inState_));
	memset(&outState_, 0, sizeof(outState_));
}

//	This is more complicated than it ought to be, because it 
//	wants to robustly recover from de-sync with the host. This means 
//	that every sub-packet that starts with a 0xff needs to be investigated
//	until a sequence that has a good length and valid CRC is found.
//	Some phantom sub-sequences may be longer than their "contained" sub-
//	sequences, so I need to re-check length and packet each time I slide 
//	the input window down.
//	When I actually have sync, I will just step through a few if statements 
//	and otherwise not do any unnecessary memcopies, which would hurt 
//	performance on embedded chips with no cache line merging.
void SerialControl::step(uint32_t now, uint32_t dus) {
	lastStep_ = now;
	while (CTRLPORT.available() > 0) {
		if (inPtr_ == sizeof(inBuf_)) {
			//	overflow -- discard
			inPtr_ = 0;
		}
		inBuf_[inPtr_] = CTRLPORT.read();
        ++numTotal_;
		if (inPtr_ || (inBuf_[0] == 0xff)) {
			++inPtr_;
		}
		else if (inPtr_ == 0 && inBuf_[0] == '?') {
			textStatus();
		}
		if (inPtr_ == 3) {
	test_again:
			if ((inPtr_ >= 3) && (inBuf_[2] > (sizeof(inBuf_) - 5))) {
				slideBuf();
				goto test_again;
			}
		}
		if (inPtr_ >= (inBuf_[2] + 5)) {
			CRC16 dcrc(inBuf_, inBuf_[2] + 3);
			uint16_t pcrc = GET2B(&inBuf_[inBuf_[2] + 3]);
			if (dcrc.get() == pcrc) {
                numGood_ += inBuf_[2] + 5;
				parsePacket();
				uint8_t left = inPtr_ - (inBuf_[2] + 5);
				if (left > 0) {
					memmove(inBuf_, &inBuf_[inPtr_ - left], left);
					inPtr_ = left;
					if (inBuf_[0] != 0xff) {
						slideBuf();
					}
					goto test_again;
				}
				else {
					inPtr_ = 0;
				}
			}
			else {
				slideBuf();
				goto test_again;
			}
		}
	}

    if ((numGood_ > 100) || (numTotal_ > 200)) {
		uint8_t w[32] = { 0xff, RESPONSE_LINKSTATS, 0 };
		Encode enc(&w[3], sizeof(w)-3);
        T2H_LinkStats linkStats = { 0 };
        linkStats.numTotal = numTotal_;
        linkStats.numGood = numGood_;
        linkStats.numInstate = numInstate_;
        linkStats.numUnknown = numUnknown_;
        linkStats.visit(enc);
		w[2] = enc.len();
		CRC16 crc(w, 3 + w[2]);
		enc.put(crc.crc_);
		ASSERT(enc.ok());
		CTRLPORT.write(w, enc.len() + 3);
        numGood_ = 0;
        numTotal_ = 0;
        numInstate_ = 0;
        numUnknown_ = 0;
     }

	if (hasInState() && ((int32_t)(lastStep_ - lastOut_) > 50)) {
        lastOut_ = lastStep_;
		uint8_t w[32] = { 0xff, RESPONSE_SETOUTSTATE, 0 };
		Encode enc(&w[3], sizeof(w)-3);
		outState_.version++;
		outState_.visit(enc);
		w[2] = enc.len();
		CRC16 crc(w, 3 + w[2]);
		enc.put(crc.crc_);
		ASSERT(enc.ok());
		CTRLPORT.write(w, enc.len() + 3);
	}
}

void SerialControl::slideBuf() {
	for (uint8_t i = 1; i < inPtr_; ++i) {
		if (inBuf_[i] == 0xff) {
			memmove(&inBuf_[0], &inBuf_[i], inPtr_ - i);
			inPtr_ -= i;
			return;
		}
	}
	inPtr_ = 0;
}

T2H_State &SerialControl::update() {
	//	todo: mark changed?
	return outState_;
}

H2T_State const &SerialControl::get() {
	return inState_;
}

bool SerialControl::hasInState() {
	return ((int32_t)lastStep_ - lastIn_) < 100;
}

bool SerialControl::testClearResetEncoders() {
    bool ret = gotResetEncoders_;
    gotResetEncoders_ = false;
    return ret;
}


void SerialControl::parsePacket() {
	//	inBuf_[0] == 0xff
	//	inBuf_[1] == cmd
	//	inBuf_[2] == len
	Decode dec(&inBuf_[3], inBuf_[2]);
	switch (inBuf_[1]) {
	case PACKET_SETINSTATE:
		inState_.visit(dec);
		ASSERT(dec.ok());
		lastIn_ = lastStep_;
        ++numInstate_;
		break;
    case PACKET_RESETENCODERS:
        {
            H2T_ResetEncoders res;
            res.visit(dec);
            ASSERT(dec.ok());
            gotResetEncoders_ = true;
        }
        break;
	default:
		//	what is this shit?
		unknown();
        ++numUnknown_;
		break;
	}
}

void SerialControl::unknown() {
	SERIALUSB.println("SER U/K");
	Alarm::blink(200);
}

class SerialPrinter {
	public:
		SerialPrinter() {
			beg_ = true;
		}
		template<typename T> void operator()(T t, char const *name) {
			if (!beg_) {
				SERIALUSB.print(" ");
			} else {
				beg_ = false;
			}
			SERIALUSB.print(name);
			SERIALUSB.print("=");
			SERIALUSB.print(t);
		}
		void done() {
			SERIALUSB.print("\r\n");
			beg_ = true;
		}

		bool beg_;	//	at beginning of line
};

static SerialPrinter printer;

void SerialControl::textStatus() {
    SERIALUSB.println(__DATE__ " " __TIME__);
	outState_.visit(printer);
	printer.done();
}
