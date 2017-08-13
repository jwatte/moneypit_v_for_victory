#if !defined(StatusTFT_h)
#define StatusTFT_h

struct StatusData {
    uint32_t wheelDistance[2];
    int32_t posX;
    int32_t posY;
    int16_t heading;
    uint16_t fps;
    uint16_t maxloop;
    uint16_t clawFlags[3];
    uint8_t servoFlags[4];
    uint8_t voltage;
    uint8_t flags;
    float hostDrive;
    float hostTurn;
    float maxT;
};

enum StatusMode {
    SM_Idle = 0,
    SM_Learn = 1,
    SM_Race = 2
};

class StatusTFT {
    public:
        StatusTFT();
        void init(uint32_t now);
        void step(uint32_t ms, uint32_t dus);
        StatusMode mode();
        void force_text(char const *text);

        StatusData status;
        static StatusTFT instance;
        static void setMode(StatusMode m);
    private:
        StatusData prevStatus;
        uint32_t onMs;
        uint32_t lastOnTime;
        uint32_t lastSvoTime;
        StatusMode mode_;
};

#endif // StatusTFT_h
