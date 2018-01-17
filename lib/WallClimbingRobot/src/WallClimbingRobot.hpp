/*
 * WallClimbingRobot.hpp
 *
 * Class controlling the wall climbing robot.
 *
 *  Created on: 12 Jan 2018
 *      Author: shenghao
 */

#ifndef LIB_WALLCLIMBINGROBOT_SRC_WALLCLIMBINGROBOT_HPP_
#define LIB_WALLCLIMBINGROBOT_SRC_WALLCLIMBINGROBOT_HPP_

#include <PWMMotor.hpp>
#include <UniPWMMotor.hpp>

namespace WallClimbingRobot {


/*!
 * \brief States that the wall climbing robot can be in
 */
enum class WallClimbingState {
	ASCENDING_AREA_A, //!< Robot is process of ascending the wall, in area A
	ASCENDING_AREA_B, //!< Robot is in process of ascending the wall, in area B
	ASCENDING_AREA_C, //!< Robot is in process of ascending the wall, in area C
	DESCENDING_AREA_C,//!< Robot is in process of descending the wall, in area C
	DESCENDING_AREA_B,//!< Robot is in process of descending the wall, in area B
	DESCENDING_AREA_A,//!< Robot is in process of descending the wall, in area C
};

class WallClimbingRobot {
private:
	PWMMotor* drive_motor_groups[2];	///< Array of drive motor handles [L->R]
	UniPWMMotor* edf_motor_groups[3];	///< Array of EDF motor handles [T->B]
	WallClimbingState state { WallClimbingState::ASCENDING_AREA_A };

public:
	/*!
	 * \brief Constructor for the wall climbing robot coordinator class.'
	 *
	 * The constructor performs these following functions:
	 *
	 * - All motors are stopped.
	 *
	 * @param drive_motor_l left drive motor handle
	 * @param drive_motor_r right drive motor handle
	 * @param edf_top top EDF motor group handle
	 * @param edf_mid middle EDF motor group handle
	 * @param edf_btm bottom EDF motor group handle
	 */
	WallClimbingRobot(PWMMotor& drive_motor_l, PWMMotor& drive_motor_r,
		UniPWMMotor& edf_top, UniPWMMotor& edf_mid, UniPWMMotor& edf_btm):
			drive_motor_groups { &drive_motor_l, &drive_motor_r},
			edf_motor_groups { &edf_top, &edf_mid, &edf_btm} {
		/* Setup the motors */
		for(uint8_t i=0; i < sizeof(drive_motor_groups) / sizeof(PWMMotor*);
				i++) {
			drive_motor_groups[i]->set_power(0.00);
		}
		for(uint8_t i=0; i < sizeof(edf_motor_groups) / sizeof(UniPWMMotor*);
				i++) {
			edf_motor_groups[i]->set_power(0.00);
		}
	}

	/*!
	 * \brief Commands the robot to drive its two drive motors to specified
	 * power levels, without altering the EDF motor power.
	 *
	 * @param pl power level for the left drive motors in the range [-100, 100]
	 * @param pr power level for the right drive motors in the range [-100, 100]
	 */
	void drive(float pl, float pr) {
		drive_motor_groups[0]->set_power(pl);
		drive_motor_groups[1]->set_power(pr);
	}

	/*!
	 * \brief Commands the robot to run the EDFs at their maximum rate and
	 * attempt to climb forward by running the drive motors at a specified
	 * power level.
	 *
	 * @param pow power level for the drive motors in the range [-100, 100]
	 */
	void climb(float pow) {

	}
};

}







#endif /* LIB_WALLCLIMBINGROBOT_SRC_WALLCLIMBINGROBOT_HPP_ */
