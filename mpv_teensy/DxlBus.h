#if !defined(DxlBus_h)
#define DxlBus_h

#include "global.h"

#define DXL_STATUS_INSTRUCTION_ERROR 0x40
#define DXL_STATUS_OVERLOAD_ERROR 0x20
#define DXL_STATUS_CHECKSUM_ERROR 0x10
#define DXL_STATUS_RANGE_ERROR 0x08
#define DXL_STATUS_OVERHEATING_ERROR 0x04
#define DXL_STATUS_ANGLE_LIMIT_ERROR 0x02
#define DXL_STATUS_INPUT_VOLTAGE_ERROR 0x01


struct DxlStatus {
    uint8_t error[4];
    uint8_t voltage[4];
    uint8_t temperature[4];
};

enum ResponseKind {
    ResponseWaiting = 0,
    ResponseReceived = 1,
    ResponseCorrupt = 2
};

class DxlBus {
    public:
        DxlBus(HardwareSerial &serial);
        void init(uint32_t now);
		void forceInit();
        void step(uint32_t nowms, uint32_t dus);
        void write(uint8_t const *wrdata, uint8_t wrsize);
        ResponseKind hasResponse(uint8_t id, uint8_t const *&ptr, uint8_t &len);
        bool awaitTimeout(int32_t diff);
        void clear();
        DxlStatus &status();
		static uint8_t checksum(uint8_t const *ptr, uint8_t len);
	private:
		HardwareSerial &serial_;
        uint8_t inBuf_[32];
        uint8_t inPtr_;
        uint32_t lastStep_;
        uint32_t lastWrite_;
        DxlStatus status_;
};

#endif  //  DxlBus_h
