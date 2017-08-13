#include <Arduino.h>
#include "MPV.h"
#include "global.h"
#include "DxlBus.h"
#include "RoboClawBus.h"
#include "DxlControl.h"
#include "RoboClawControl.h"
#include "Steer.h"
#include "PowerControl.h"
#include "Alarm.h"
#include "RadioInput.h"
#include "SerialControl.h"
#include "StatusTFT.h"

#if !defined(WIN32)
#include <Audio.h>

// GUItool: begin automatically generated code
#if 1 //    Teensy 3.2
AudioInputUSB            usb1;           //xy=70,242
AudioMixer4              mixer1;         //xy=226,256
AudioOutputAnalog        dac1;           //xy=355,256
AudioConnection          patchCord1(usb1, 0, mixer1, 0);
AudioConnection          patchCord2(usb1, 1, mixer1, 1);
AudioConnection          patchCord3(mixer1, dac1);
#else //    Teensy 3.5
AudioInputUSB            usb1;           //xy=70,242
AudioOutputAnalogStereo  dacs1;          //xy=291,211
AudioConnection          patchCord1(usb1, 0, dacs1, 0);
AudioConnection          patchCord2(usb1, 1, dacs1, 1);
#endif
// GUItool: end automatically generated code

#endif

bool seenGoodPower = false;
bool beenInitialized = false;

//  These don't go into MPV.h because I don't want to recompile everything 
//  each time I change anything.
static uint32_t                         lastUs;
static uint32_t                         last_power;
static DxlBus                           dxl(Serial1);
static RoboClawBus                      roboclaw(Serial2);
static DxlControl                       dxlCtl(dxl);
static RoboClawControl                  roboClawCtl(roboclaw);
static Steer                            steer(roboClawCtl, dxlCtl);
static PowerControl						power;
static Alarm							alarm;
static RadioInput						radio;
static SerialControl					serial;

uint32_t volatile a = 10;
uint32_t volatile b = 5;

void test_assert(bool b, char const *expr) {
	if (!b) {
		pinMode(PIN_MOTCTL, OUTPUT);
		digitalWrite(PIN_MOTCTL, LOW);
		pinMode(LED_PIN, OUTPUT);
		pinMode(PIN_ESTOP, INPUT_PULLUP);
		int n = 0;
#if defined(_MSC_VER)
		__debugbreak();
#endif
		while (true) {
			digitalWrite(LED_PIN, HIGH);
			delay(10);
			digitalWrite(LED_PIN, LOW);
			delay(30);
			if (digitalRead(PIN_ESTOP) == LOW) {
				SERIALUSB.println(expr);
				break;
			}
			n = (n + 1) & 0x7f;
			if (!n) {
				SERIALUSB.println(expr);
			}
		}
	}
}

void test_math() {
	int32_t volatile c = (int32_t)(a - b);
	ASSERT(c == 5);
	int32_t volatile d = (int32_t)(b - a);
	ASSERT(d == -5);
}

void wait_for_power() {
}

void MPV::init() {

	test_math();
#if !defined(WIN32)
    AudioMemory(6);
#endif
	power.init(0);
	alarm.init(0);
	alarm.setLed(true);
    StatusTFT::instance.init(0);
}

void MPV::late_init() {

	uint32_t now = millis();
	alarm.init(now);
	power.init(now);
	radio.init(now);
	StatusTFT::instance.init(now);
	serial.init(now);
	dxl.init(now);
    roboclaw.init(now);
	dxlCtl.init(now);
	roboClawCtl.init(now);
	steer.init(now);
}

void MPV::applyDriveSteer(float dr, float st, uint32_t now) {
	steer.driveSteer(dr, st);
	lastDriven_ = now;
}

void MPV::read_input(uint32_t now) {
	if (radio.connected()) {
        auto const &h2t(serial.get());

        float maxT = (radio.throttle() - 1550) / 350.0f;
        if (maxT < 0) maxT = 0;
        if (maxT > 1) maxT = 1;
        float hostDrive = ((float)h2t.drive - 32768) / 10000.0f;
        float hostSteer = ((float)h2t.turn - 32768) / 10000.0f;
        //  don't apply very small jittery inputs
        if (fabsf(hostSteer) < 0.02f) {
            hostSteer = 0;
        }
        if (fabsf(hostDrive) < 0.02f) {
            hostDrive = 0;
            hostSteer = 0;
        }
        //  enforce a minimum turning radius
        if (fabsf(hostSteer) > fabsf(hostDrive) * 2) {
            hostSteer *= fabsf(hostDrive) * 2 / fabsf(hostSteer);
        }

        StatusTFT::instance.status.hostDrive = hostDrive;
        StatusTFT::instance.status.hostTurn = hostSteer;
        StatusTFT::instance.status.maxT = maxT;

        switch (h2t.mode) {
        case HOST_MODE_RADIODRIVE:
            if (radio.driveMode()) {
                float driveM = ((radio.throttle() > 1550) ? (radio.throttle() - 1550) :
                    (radio.throttle() < 1450) ? (radio.throttle() - 1450) : 0) * 0.005f;
                float turnR = ((radio.steer() > 1550) ? (radio.steer() - 1550) :
                    (radio.steer() < 1450) ? (radio.steer() - 1450) : 0) * 0.005f;
                applyDriveSteer(driveM, turnR * (1.0f + fabsf(driveM)), now);
            }
            break;
        case HOST_MODE_HOSTDRIVE:
            if (radio.driveMode()) {
                applyDriveSteer(
                        hostDrive * maxT,
                        hostSteer * maxT,
                        now);
            }
            break;
        default:
            //  unknown!
            Alarm::blink(300);
            applyDriveSteer(0, 0, now);
            break;
        }
   	}
	if ((int32_t)(now - lastDriven_) >= 1500) {
        applyDriveSteer(0, 0, now);
		//Alarm::blink(300);
	}
}

uint32_t mdus = 0;
uint32_t last_fps = 0;
uint32_t fr = 0;

void MPV::step() {
    if (!seenGoodPower) {
		power.readVoltage();
        if (power.voltageOk()) {
            seenGoodPower = true;
        } else {
            uint32_t now = millis();
            if (now - last_power >= 8000) {
                char buf[20];
                sprintf(buf, "Low Volt: %.1f", power.getVoltage());
                SERIALUSB.println(buf);
                StatusTFT::instance.force_text(buf);
                last_power = now;
                ++fr;
                if (fr == 6) {  //  40 seconds? reboot
                    fr = 0;
                    SERIALUSB.println("reboot()");
                    delay(500);
                    reboot();
                }
            }
        }
	} else if (!beenInitialized) {
        late_init();
        alarm.setLed(false);
        SERIALUSB.println("initialize");
        beenInitialized = true;
    } else {
        ++fr;
        uint32_t now_ms = millis();
        uint32_t now_us = micros();
        uint32_t dus = now_us - lastUs;
        if (dus > mdus) {
            mdus = dus;
        }
        lastUs = now_us;
        radio.step(now_ms, dus);
        serial.step(now_ms, dus);
        read_input(now_ms);
        power.step(now_ms, dus);
        roboClawCtl.setEnable(power.getMotorEnabled());
        dxlCtl.setEnable(power.getMotorEnabled());
        steer.step(now_ms, dus);
        roboClawCtl.step(now_ms, dus);
        dxlCtl.step(now_ms, dus);
        roboclaw.step(now_ms, dus);
        dxl.step(now_ms, dus);
        alarm.step(now_ms, dus);

        if (now_ms - last_fps >= 2000) {
            StatusTFT::instance.status.maxloop = mdus > 65535 ? 65535 : (uint16_t)mdus;
            StatusTFT::instance.status.fps = fr * 1000 / (now_ms - last_fps);
            fr = 0;
            mdus = 0;
            last_fps = now_ms;
        }
        auto &status = serial.update();

        if (serial.testClearResetEncoders()) {
            roboClawCtl.resetEncoders();
        }

        roboClawCtl.getPos(0x81, &status.m1);
        //  TODO: RoboClaw error bits
        for (int i = 0; i != 4; ++i) {
            status.dtemp[i] = dxl.status().temperature[i];
            status.dstatus[i] = dxl.status().error[i];
            StatusTFT::instance.status.servoFlags[i] |= status.dstatus[i];
        }

        uint16_t cs[3] = { 0 };
        roboClawCtl.getStatus(cs);
        for (int i = 0; i != 3; ++i) {
            StatusTFT::instance.status.clawFlags[i] |= cs[i];
        }

        status.flags = 0;
        if (radio.connected()) {
            status.flags |= FLAG_CONNECTED;
            if (radio.driveMode()) {
                status.flags |= FLAG_DRIVEMODE;
                if (serial.get().mode == HOST_MODE_HOSTDRIVE) {
                    status.flags |= FLAG_HOSTDRIVE;
                }
            }
        }
        switch (StatusTFT::instance.mode()) {
            case SM_Learn:
                status.flags |= FLAG_LEARNMODE;
                break;
            case SM_Race:
                status.flags |= FLAG_RACEMODE;
                break;
            default:
                // nothing
                break;
        }
        if (PowerControl::pendingShutdown()) {
            status.flags |= FLAG_SHUTDOWN;
        }

        status.volt = (uint8_t)(power.getVoltage() * 10 + 0.5f);
        StatusTFT::instance.status.flags = status.flags;
        StatusTFT::instance.status.voltage = status.volt;
        StatusTFT::instance.status.wheelDistance[0] = status.m1;
        StatusTFT::instance.status.wheelDistance[1] = status.m2;
        StatusTFT::instance.step(now_ms, dus);

        uint32_t nus = micros() - lastUs;
        if (nus < 200) {
            delayMicroseconds(200 - nus);
        }
    }
}

