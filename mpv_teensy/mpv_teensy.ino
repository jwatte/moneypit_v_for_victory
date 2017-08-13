#include "MPV.h"

#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

void reboot() {
#if defined(MSV_VER)
	__debugbreak();
#else
	WRITE_RESTART(0x5FA0004);
#endif
}

MPV mpv;

void setup() {
  // put your setup code here, to run once:
  mpv.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  mpv.step();
}


