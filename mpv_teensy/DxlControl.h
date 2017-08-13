#if !defined(DxlControl_h)
#define DxlControl_h

class DxlBus;

class DxlControl {
    public:
        DxlControl(DxlBus &bus);
        void init(uint32_t now);
		void forceInit(uint32_t now);
        void step(uint32_t ms, uint32_t dus);
        void setTargetAngles(float leftRadians, float rightRadians);
		void setEnable(bool en);
        struct Regs {
            uint8_t reg;
            uint8_t data[2];
            uint8_t n;
        };
		int16_t inspectAngle(uint8_t index);
    private:
        DxlBus &bus_;
		uint32_t lastWrite_;
		uint32_t lastStep_;
		uint8_t buf_[16];
        uint8_t posVel_[4][4];
        uint8_t pvRead_[2];
        uint8_t state_;
        uint8_t id_;
        uint8_t reg_;
        uint8_t phase_;
		bool enable_;

        void setState(uint8_t state);
        bool awaitTimeout(int32_t n);
        void updateInit();
        void updateRun();
        void writeReg(uint8_t id, uint8_t reg, uint8_t const *data, uint8_t n);
        void readReg(uint8_t id, uint8_t reg, uint8_t len);
		void parseResponse(uint8_t const *ptr, uint8_t len);
		void servoTimeout();

        static const Regs regs_[12];
        static const uint8_t ids_[4];
};

#endif  //  DxlControl_h
