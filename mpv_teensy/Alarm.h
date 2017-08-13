#if !defined(Alarm_h)
#define Alarm_h

class Alarm {
public:
	Alarm();
	void init(uint32_t now);
	void step(uint32_t now, uint32_t dus);
	static void blink(uint32_t duration);
	void setLed(bool on);
private:
	uint32_t blinkUntil_;
	uint32_t lastStep_;
	static Alarm *alarm_;
};

#endif	//	Alarm_h
