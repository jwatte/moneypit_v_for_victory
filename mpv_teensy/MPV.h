#if !defined(MPV_h)
#define MPV_h

class MPV {
public:
    void init();
    void step();
private:
    void late_init();
	void read_input(uint32_t now);
	void applyDriveSteer(float dr, float st, uint32_t now);
	uint32_t lastDriven_;
};

#endif  //  MPV_h
