#include <Arduino.h>
#include "Steer.h"
#include "RoboClawControl.h"
#include "DxlControl.h"
#include "global.h"
#include <algorithm>		//	for swap

static float const TRAVEL_MM_PER_TICK = (WHEEL_DIAMETER * M_PI) / (MOTOR_CPR * MOTOR_GEAR);
static float const RADIUS = (SUSPENSION_TRACK * 0.5f);
static float const MAX_SPEED_CPS = (MAX_SPEED_MM) / TRAVEL_MM_PER_TICK;
static float const HALF_WHEELBASE = (SUSPENSION_WHEELBASE * 0.5f);


Steer::Steer(RoboClawControl &claw, DxlControl &dxl) : roboclaw_(claw), dxl_(dxl) {
}

void Steer::init(uint32_t now) {
    lastDrive_ = now;
    lastStep_ = now;
	lastLockout_ = now;
    driveM = 0;
    turnR = 0;
	gotDrive_ = false;
	wheelAngles_[0] = wheelAngles_[1] = 0;
	leftFlip_ = 0;
	rightFlip_ = 0;
	memset(wheelSpeeds_, 0, sizeof(wheelSpeeds_));
}

void Steer::step(uint32_t now, uint32_t dus) {
	lastStep_ = now;
	if ((int32_t)(now - lastDrive_) > 500) {
        //  failsafe
        driveSteer(0, 0);
		lastLockout_ = now;
    }
	//	received a command since the previous step
    if (gotDrive_) {
		//	now > lockout: not locked out
		if ((int32_t)(now - lastLockout_) >= 0) {
			gotDrive_ = false;
			updateControls();
			lastLockout_ = now;
		}
    }
}

#define STEER_SLEW_RATE 1.0f
#define STEER_MILLISECONDS_PER_RADIAN 200
#define MAX_SPIN 2.0f
#define MAX_SPIN_PER_VEL 2.0f

void Steer::driveSteer(float speed, float spin) {
	if (speed > MAX_SPEED_M) {
		speed = MAX_SPEED_M;
	}
	else if (speed < -MAX_SPEED_M) {
		speed = -MAX_SPEED_M;
	}
	float steerLimitSpeed = fabsf(speed);
	float spinLimit = MAX_SPIN + steerLimitSpeed * MAX_SPIN_PER_VEL;
	float limitSpin = fabsf(spin);
	if (limitSpin > spinLimit) {
		spin = (spin > 0) ? spinLimit : -spinLimit;
	}
    driveM = speed;
    turnR = spin;
    lastDrive_ = lastStep_;
	gotDrive_ = true;
}

void Steer::updateControls() {
	//	MOTOR_CPR * MOTOR_GEAR counts per shaft revolution
	//	WHEEL_DIAMETER * M_PI millimeters of travel per revolution
	//	1000 mm per meter
	float averageCPS = driveM * MOTOR_CPR * MOTOR_GEAR * 1000 / (WHEEL_DIAMETER * M_PI);
	//	one tick moves left side forward and right side backward
	//	amount turned in radians is (travel per tick)/(radius)
	//	travel per tick is (wheel circumference)/(ticks per rotation)
	//	tsps = cps*tpt/r
	//	tsps*r/tpt = cps

	float deltaCPS = turnR * RADIUS / TRAVEL_MM_PER_TICK;
	float leftCPS = averageCPS + deltaCPS;
	float rightCPS = averageCPS - deltaCPS;
	if (fabsf(leftCPS) > MAX_SPEED_CPS) {
		rightCPS *= (MAX_SPEED_CPS / fabsf(leftCPS));
		leftCPS = copysign(MAX_SPEED_CPS, leftCPS);
	}
	if (fabsf(rightCPS) > MAX_SPEED_CPS) {
		leftCPS *= (MAX_SPEED_CPS / fabsf(rightCPS));
		rightCPS = copysign(MAX_SPEED_CPS, rightCPS);
	}

	float leftAngle = 0.0f;
	float rightAngle = 0.0f;
	float leftRadius = 1.0f;
	float rightRadius = 1.0f;

	if (leftCPS != rightCPS) {	//	actually turning

		//	First, calculate the radius of the center wheel circles
		//	distance from left to right wheel is the track, which is the difference between the radii
		//	equivalent radians traveled on different-radius circles gives:
		//	Define R as distance-to-right of center
		//
		//	for leftCPS != 0:
		//	(R + RADIUS) / leftCPS = (R - RADIUS) / rightCPS
		//	(R + RADIUS) / leftCPS * rightCPS + RADIUS = R
		//	RADIUS / leftCPS * rightCPS + RADIUS = R * (1 - rightCPS / leftCPS)
		//	((RADIUS * rightCPS / leftCPS) + RADIUS) / (1 - rightCPS / leftCPS) = R
		//
		//	for rightCPS != 0:
		//	R = (R - RADIUS) * leftCPS / rightCPS - RADIUS
		//	R * (1 - leftCPS / rightCPS) = -RADIUS * leftCPS / rightCPS - RADIUS
		//	R = ((-RADIUS * leftCPS / rightCPS) - RADIUS) / (1 - leftCPS / rightCPS)
		float R;
		if (leftCPS != 0) {
			float divr = float(rightCPS) / float(leftCPS);
			R = ((RADIUS * divr) + RADIUS) / (1 - divr);
		}
		else {
			float divl = float (leftCPS) / float(rightCPS);
			R = ((-RADIUS * divl) - RADIUS) / (1 - divl);
		}
		//	Find the Ackerman angles that makes each corner wheel turn at the appropriate rate.
		//	Trigonometry to calculate steer angles
		float leftRadius = R + RADIUS;
		leftAngle = atan2f(HALF_WHEELBASE, leftRadius);
		float rightRadius = R - RADIUS;
		rightAngle = atan2f(HALF_WHEELBASE, rightRadius);
		//	Right triangles gives us the other wheel radii
		float leftHypothenuse = sqrtf(leftRadius*leftRadius + HALF_WHEELBASE*HALF_WHEELBASE);
		float rightHypothenuse = sqrtf(rightRadius*rightRadius + HALF_WHEELBASE*HALF_WHEELBASE);
		int32_t leftCornerSpeed;
		if (fabsf(leftRadius) < 1.0f) {
			leftCornerSpeed = (int32_t)(rightCPS * leftHypothenuse / (SUSPENSION_TRACK - leftRadius));
		} else {
			leftCornerSpeed = (int32_t)(leftCPS * leftHypothenuse / leftRadius);
		}
		int32_t rightCornerSpeed;
		if (fabsf(rightRadius) < 1.0f) {
			rightCornerSpeed = (int32_t)(leftCPS * rightHypothenuse / (SUSPENSION_TRACK - rightRadius));
		} else {
			rightCornerSpeed = (int32_t)(rightCPS * rightHypothenuse / rightRadius);
		}
		wheelSpeeds_[0] = leftCornerSpeed;
		wheelSpeeds_[1] = rightCornerSpeed;
		wheelSpeeds_[2] = (int32_t)leftCPS;
		wheelSpeeds_[3] = (int32_t)rightCPS;
		wheelSpeeds_[4] = wheelSpeeds_[0];
		wheelSpeeds_[5] = wheelSpeeds_[1];
	}
	else {
		//	straight wheels
		//	all six wheels have the same speed
		wheelSpeeds_[0] = (int32_t)averageCPS;
		wheelSpeeds_[1] = (int32_t)averageCPS;
		wheelSpeeds_[2] = (int32_t)averageCPS;
		wheelSpeeds_[3] = (int32_t)averageCPS;
		wheelSpeeds_[4] = (int32_t)averageCPS;
		wheelSpeeds_[5] = (int32_t)averageCPS;
	}

	if (leftAngle > 1.7f) {
		leftFlip_ = -M_PI;
	} else if (leftAngle < -1.7f) {
		leftFlip_ = M_PI;
	}
	if (rightAngle > 1.7f) {
		rightFlip_ = -M_PI;
	} else if (rightAngle < -1.7f) {
		rightFlip_ = M_PI;
	}
	if (fabsf(leftAngle) < 1.5f) {
		leftFlip_ = 0;
	}
	if (fabsf(rightAngle) < 1.5f) {
		rightFlip_ = 0;
	}
	if (leftFlip_ != 0.0f) {
		leftAngle += leftFlip_;
		wheelSpeeds_[0] *= -1;
		wheelSpeeds_[4] *= -1;
	}
	if (rightFlip_ != 0.0f) {
		rightAngle += rightFlip_;
		wheelSpeeds_[1] *= -1;
		wheelSpeeds_[5] *= -1;
	}

	wheelAngles_[0] = leftAngle;
	wheelAngles_[1] = rightAngle;

	dxl_.setTargetAngles(wheelAngles_[0], wheelAngles_[1]);
	roboclaw_.drive(wheelSpeeds_);
}
