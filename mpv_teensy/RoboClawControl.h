#if !defined(RoboClawContol_h)
#define RoboClawControl_h

class RoboClawBus;

class RoboClawControl {
public:
    RoboClawControl(RoboClawBus &bus);
    void init(uint32_t now);
	void forceInit();
    void step(uint32_t now, uint32_t dus);
    void drive(int32_t const *p);
	void setEnable(bool en);
    void resetEncoders();
	//	ix is 0, 1, or 2
	void getPos(int ix, uint32_t p[2]);
    void getStatus(uint16_t o[3]);
private:
    RoboClawBus &roboclaw_;
    uint32_t lastStep_;
    uint32_t lastDrive_;
	bool enable_;
    bool freshState_;
    bool shouldResetEncoders_;
	uint8_t state_;
	uint8_t phase_;
	uint8_t id_;
    int32_t motor_[6];
	uint32_t pos_[3][2];
    uint16_t status_[3];
	void setState(uint8_t st);
	void stepInit();
	void stepDrive();
	void stepPoll();
    void stepStatus();
    void stepReset();
};

#endif  //  RoboClawControl_h
