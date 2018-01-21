/*
 * StateMachineFunctions.cpp
 *
 * Function representing the wall climbing robot's states.
 *
 *  Created on: 20 Jan 2018
 *      Author: shenghao
 */

#include <StateMachine.hpp>
#include <RobotConstants.hpp>
#include <Arduino.h>

namespace StatesTest {
	using namespace RobotConstants;
	using namespace StateMachine;

	StateStatus state_A(RobotState& rstate, bool first_run) {
		if (first_run) {
			Serial.printf("State A: first run\n");
			return StateStatus::REPEAT;
		} else {
			Serial.printf("State A: non-first run\n");
			Serial.printf("State A: transitioning now to state B\n");
			return StateStatus::NEXT;
		}
		return StateStatus::ABORT;
	}

	StateStatus state_B(RobotState& rstate, bool first_run) {
		static elapsedMillis state_time {};
		if (first_run) {
			Serial.printf("State B: first run\n");
			state_time = elapsedMillis {};
			return StateStatus::REPEAT;
		} else {
			if (state_time < 10000) {
				Serial.printf("State B: sensor readings: %u mm\n", rstate.range_readings[0]);
				delay(20);
			} else {
				Serial.printf("State B: transitioning now to state C via a ABORT jump");
				return StateStatus::ABORT;
			}
		}
		return StateStatus::ABORT;
	}

	StateStatus state_C(RobotState& rstate, bool first_run) {
		if (first_run) {
			Serial.printf("State C: first run\n");
			Serial.printf("State C: a state machine break message should be seen next\n");
			Serial.printf("State C: because the state table doesn't have an entry for this state");
			return StateStatus::REPEAT;
		} else {
			Serial.printf("State C: second run\n");
			Serial.printf("State C: YOU SHOULD NOT BE SEEING THIS!?!@#1111");
			return StateStatus::ABORT;
		}
	}


}


namespace States {
	using namespace RobotConstants;
	using namespace StateMachine;
	StateStatus A_to_ABtrans(RobotState& rstate, bool first_run) {
		if (first_run) {
			//\todo PID stuff
		}

		float average_distance_m {
			(static_cast<float>(rstate.range_readings[RS_FRONT_LEFT] +
			rstate.range_readings[RS_FRONT_RIGHT]) / (2.0f / 1000.0f))
		};


		if (average_distance_m < 0.06) {
			/*
			 * If we are not close enough to the wall, drive forward at a speed
			 * of 40 for both motors, ask supervisor to repeat this state.
			 */
			rstate.motor_speeds[DM_LEFT] = 40;
			rstate.motor_speeds[DM_RIGHT] = 40;
			rstate.edf_speeds[EDFS_FRONT] = 0;
			rstate.edf_speeds[EDFS_MIDDLE] = 0;
			rstate.edf_speeds[EDFS_REAR] = 0;
			return StateStatus::REPEAT;
		} else {
			/*
			 * Otherwise, signal escape from this state and move on towards
			 * the next state
			 */
			return StateStatus::NEXT;
		}
	}


	StateStatus ABtrans_to_B(RobotState& rstate, bool first_run) {
		if (first_run) {
			//\todo some init stuff here, for pid, etc.
		}

		float average_distance_m {
			(static_cast<float>(rstate.range_readings[RS_FRONT_LEFT] +
			 rstate.range_readings[RS_FRONT_RIGHT]) / (2.0f / 1000.0f))
		};


		if (average_distance_m < 0.08) {
			/*
			 * While we are making our way up the transition, wait till we are
			 * significantly vertical before turning on the top and middle EDFs.
			 *
			 * The middle EDFs are turned on to ensure sufficient grip for the
			 * rear wheels as they push the robot up the transition point.
			 *
			 * The drive motors are always driven at speed 100 to ensure
			 * sufficient wall climbing capability.
			 *
			 */
			rstate.motor_speeds[DM_LEFT] = 100;
			rstate.motor_speeds[DM_RIGHT] = 100;

			rstate.edf_speeds[EDFS_FRONT] = 0;
			rstate.edf_speeds[EDFS_MIDDLE] = 0;
			rstate.edf_speeds[EDFS_REAR] = 100;

			return StateStatus::REPEAT;
		} else {
			/*
			 * Otherwise, signal escape from this state and allow the
			 * next state to take over.
			 */
			return StateStatus::NEXT;
		}
	}


	StateStatus B_to_BCtrans(RobotState& rstate, bool first_run) {
		static elapsedMillis time_in_state {};
		if (first_run) {
			//\todo some init stuff here, for pid, etc.
			time_in_state = elapsedMillis {};
		}


		if (time_in_state < 2500) {
			/*
			 * Only drive vertically for 2500ms. Terminate afterwards.
			 */
			rstate.motor_speeds[DM_LEFT] = 100;
			rstate.motor_speeds[DM_RIGHT] = 100;

			rstate.edf_speeds[EDFS_FRONT] = 100;
			rstate.edf_speeds[EDFS_MIDDLE] = 100;
			rstate.edf_speeds[EDFS_REAR] = 100;

			return StateStatus::REPEAT;
		} else {
			/*
			 * Otherwise, signal escape from this state and allow the next
			 * state to take over
			 */
			return StateStatus::NEXT;
		}
	}
};



