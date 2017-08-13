#if !defined(Steer_h)
#define Steer_h

class RoboClawControl;
class DxlControl;

class Steer {
public:
    Steer(RoboClawControl &roboclaw, DxlControl &dxl);
    void init(uint32_t now);
    void step(uint32_t now, uint32_t dus);
    void driveSteer(float driveMetersPerSecond, float steerRadiansPerSecond);
private:
    RoboClawControl &roboclaw_;
    DxlControl &dxl_;
    float driveM;
    float turnR;
	bool gotDrive_;
    uint32_t lastStep_;
    uint32_t lastDrive_;
	uint32_t lastLockout_;
	int32_t wheelSpeeds_[6];
	float wheelAngles_[2];
    float leftFlip_;
    float rightFlip_;

	void updateControls();
};

#endif  //  Steer_h
