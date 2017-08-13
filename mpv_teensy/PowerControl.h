#if !defined(PowerControl_h)
#define PowerControl_h

class PowerControl {
public:
	PowerControl();
	void init(uint32_t now);
	void step(uint32_t now, uint32_t dus);
	float getVoltage();
	void readVoltage();
	bool voltageOk();
	bool getMainEnabled();
	bool get5VEnabled();
	bool getMotorEnabled();
	static void requestEstop();
    static void requestShutdown();
    static bool pendingShutdown();
private:
	float voltage_;
	uint32_t lastBad_;
	uint32_t lastOk_;
	uint32_t lastNonEstop_;
	bool enableMain_;
	bool enable5V_;
	bool enableMotor_;
	bool isEstop_;
    static uint32_t pendingShutdown_;
	static bool externalEstopRequested_;
};
#endif	//	PowerControl_h
