#include <Arduino.h>
#include <L298NMotor.hpp>
#include <EDFMotor.hpp>
#include <WallClimbingRobot.hpp>
#include <Bounce2.h>



void setup() {
	L298NMotor::L298NMotor dmotor_l {1, 2, 3};
	L298NMotor::L298NMotor dmotor_r {4, 5, 6};
	EDFMotor::EDFMotor edf_grp_f {9};
	EDFMotor::EDFMotor edf_grp_m {10};
	EDFMotor::EDFMotor edf_grp_b {11};
	WallClimbingRobot::WallClimbingRobot wcr {dmotor_l, dmotor_r,
		edf_grp_f, edf_grp_m, edf_grp_b};
}

void loop() {
	/* I don't want to do anything inside here */
}
