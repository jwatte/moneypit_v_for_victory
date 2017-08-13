#if !defined(global_h)
#define global_h

#include <stdint.h>


#define PIN_UTIL 4
#define PIN_B5 23
#define PIN_B4 22
#define PIN_B3 21
#define PIN_B2 17
#define PIN_B1 16

#define PIN_CLAW_TX 10
#define PIN_CLAW_RX 9

#define LED_PIN PIN_B1
#define PIN_ESTOP PIN_B2
//#define PIN_BUMP_SENSOR PIN_B3
#define PIN_VSENSE 20
#define PIN_A_VSENSE A6
#define PIN_RPI_5VCTL 2
#define PIN_MOTCTL 3
#define PIN_PWR_ON 6
#define PIN_DIRECTION 5
#define PIN_PULSEIN_STATE PIN_B4
#define PIN_PULSEIN_THROTTLE PIN_B5
#define PIN_PULSEIN_STEER PIN_UTIL


#if !defined(M_PI)
#define M_PI 3.1415927f
#endif

//	Physical measurements of rover
#define MOTOR_RPM 160       //  If motor can't reach RPM, I term will slip after stop
#define MOTOR_CPR 64
#define MOTOR_GEAR 50
#define WHEEL_DIAMETER 175
#define SUSPENSION_WHEELBASE 500
#define SUSPENSION_TRACK 500
#define MAX_SPEED_MM (MOTOR_RPM*WHEEL_DIAMETER*float(M_PI)/60.0f)
#define MAX_SPEED_M (MAX_SPEED_MM*0.001f)



template<typename T> T iabs(T a) { return (a < 0) ? -a : a; }
void test_assert(bool b, char const *expr);
#define ASSERT(x) test_assert(x, #x)
#define SERIALUSB SerialUSB

#define GET4B(p) \
	( ((uint32_t)((uint8_t const *)(p))[0] << 24) \
	| ((uint32_t)((uint8_t const *)(p))[1] << 16) \
	| ((uint32_t)((uint8_t const *)(p))[2] << 8) \
	| ((uint32_t)((uint8_t const *)(p))[3]) \
	)
#define GET2B(p) \
	( ((uint16_t)((uint8_t const *)(p))[0] << 8) \
	| ((uint16_t)((uint8_t const *)(p))[1]) \
	)

extern void reboot();

#endif  //  global_h
