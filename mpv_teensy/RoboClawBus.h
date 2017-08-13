#if !defined(RoboClawBus_h)
#define RoboClawBus_h

#include "CRC.h"

class RoboClawBus {
    public:
        RoboClawBus(HardwareSerial &serial);
        void init(uint32_t now);
		void forceInit(uint32_t now);
        void step(uint32_t nowms, uint32_t dus);
		bool awaitTimeout(int32_t ms);
        void resetTimeout();

		void getVersion(uint8_t id);
		void getPosition(uint8_t id);
        void getStatus(uint8_t id);
		void setPID(uint8_t id, uint8_t m1m2);
		void driveM1M2(uint8_t id, int32_t m1, int32_t m2);
		void resetEncoders(uint8_t id);
		void setVoltage(uint8_t id, float low, float high);
		bool hasResponse(uint8_t const *&ptr, uint8_t &len);
		bool hasAck();

		void write(uint8_t const *wrdata, uint8_t wrsize);
        void writeCrc();
    private:
		HardwareSerial &serial_;
        uint8_t inBuf_[36];
        uint8_t inPtr_;
		uint8_t responseType_;
		bool ack_;
		CRC16 crc_;
		uint32_t lastStep_;
        uint32_t lastWrite_;
};

#endif  //  RoboClawBus_h
