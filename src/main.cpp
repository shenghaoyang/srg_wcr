#include <Arduino.h>
#include <L298NMotor.hpp>
#include <EDFMotor.hpp>
#include <ToggleSwitch.hpp>
#include <vl53l0x.hpp>
#include <StateMachineFunctions.hpp>
#include <RobotConstants.hpp>

void setup() {
	using namespace StateMachine;

	/* Initialize motor drive handles */
	EDFMotor::EDFMotor edfs[3] {
			{ RobotConstants::EDFS_CTRL_F },
			{ RobotConstants::EDFS_CTRL_M },
			{ RobotConstants::EDFS_CTRL_B },
	};

	L298NMotor::L298NMotor drive_motors[2] {
			{
			  RobotConstants::DM_LEFT_IN0,
			  RobotConstants::DM_LEFT_IN1,
			  RobotConstants::DM_LEFT_EN
			},
			{
			  RobotConstants::DM_RIGHT_IN0,
			  RobotConstants::DM_RIGHT_IN1,
			  RobotConstants::DM_RIGHT_EN
			},
	};

	/* Initialize sensor handles */
	ToggleSwitch::ToggleSwitch shutoff {
		RobotConstants::SAFES_IN,
		RobotConstants::SAFES_REF,
		OUTPUT,
		50
	};
	/* Initialize sensor buses: Tof[0, 2] on Wire, Tof[1, 3] on Wire1 */
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
	Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_22_23, I2C_PULLUP_EXT, 400000);

	TeensyVL53L0X::TeensyVL53L0X range_sensors[4] {
		{ Wire,  RobotConstants::TOF0_XSHUT },
		{ Wire1, RobotConstants::TOF1_XSHUT },
		{ Wire,  RobotConstants::TOF2_XSHUT },
		{ Wire1, RobotConstants::TOF3_XSHUT },
	};

	/* Initialize one sensor first */
	range_sensors[RobotConstants::RS_FRONT_LEFT].poweron();
	delay(100);
	range_sensors[RobotConstants::RS_FRONT_LEFT].init();
	range_sensors[RobotConstants::RS_FRONT_LEFT].startContinuous();
	shutoff.wait_edge(ToggleSwitch::EdgeType::EDGE_RISING);
	for (uint8_t i = 1; i <= 10; i++) {
		shutoff.update();
		if (!shutoff)
			break;
		digitalWrite(13, i % 2);
		delay(1000);
	}


	/* Setup variables associated with the state machine */
	StateFunction curstate { StatesTest::start };
	StateFunction newstate;
	StateStatus status;
	RobotState rstate {
		{ 8192, 8192, 8192, 8192 },
		{ 0, 0, 0},
		{ 0, 0 }
	};
	bool first_run { true };
	Serial.printf("Main thread: starting state machine");
	while (!shutoff) {
		/*
		 * Run the state machine, resetting the first_run variable if
		 * we have completed our first run
		 */
		status = curstate(rstate, first_run);
		first_run = first_run ? false : false;

		/*
		 * Lookup a new state, then execute it. Set first_run if the new
		 * state is not the current state
		 */
		if(!(newstate
				= lookup_transition(curstate, status, StatesTest::StateTable)))
			/* No state looked up, commit to exit state */
			break;
		first_run = (newstate == curstate) ? false : true;
		curstate = newstate;

		/* Commit the changes to robot state */
		for (uint8_t i = 0; i < 3; i++) {
			edfs[i].set_power(
					constrain(rstate.edf_speeds[i], 0.f, 100.0f));
		}
		for (uint8_t i = 0; i < 2; i++) {
			drive_motors[i].set_power(
					constrain(rstate.motor_speeds[i], -100.f, 100.f));
		}

		/* Update robot state information */
		shutoff.update();
		int8_t poll_status = range_sensors[RobotConstants::RS_FRONT_LEFT].range_available(1000);
		if (poll_status < 0) {
			/* I2C error, break and exit to safe state */
			break;
		} else {
			if (poll_status > 0) {
				/* range available */
				int32_t range_mm = range_sensors[RobotConstants::RS_FRONT_LEFT].get_and_clear_range(1000);
				if (range_mm < 0)
					break; // I2C error
				for (uint8_t i=0; i < 4; i++) {
					rstate.range_readings[i] = range_mm;
				}
			}
		}
	}
	Serial.printf("Main thread: state machine destroyed\n");

	/* Put system into safe state */
	for (uint8_t i = 0; i < 3; i++) {
		edfs[i].flt();
	}
	for (uint8_t i = 0; i < 2; i++) {
		drive_motors[i].brake();
	}
	digitalWrite(13, HIGH);
	while (1);
}

void loop() {}
