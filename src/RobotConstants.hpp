/*
 * RobotConstants.hpp
 *
 * Constants for the wall climbing robot.
 *
 *  Created on: 20 Jan 2018
 *      Author: shenghao
 */

#ifndef ROBOTCONSTANTS_HPP_
#define ROBOTCONSTANTS_HPP_

namespace RobotConstants {

/*!
 * \brief Enumerator matching sensor indices to textual representations
 * of the ToF sensor positions.
 */
enum RangeSensorPositions {
	RS_FRONT_LEFT = 0,	///< Index of front left sensor
	RS_FRONT_RIGHT,		///< Index of front right sensor
	RS_REAR_LEFT,		///< Index of rear left sensor
	RS_REAR_RIGHT		///< Index of rear right sensor
};

/*!
 * \brief Enumerator matching motor indices to textual representations of the
 * drive motor positions.
 */
enum DriveMotorPositions {
	DM_LEFT = 0, ///< Index of left drive motor group
	DM_RIGHT, 	 ///< Index of right drive motor group
};

/*!
 * \brief Enumerator matching motor H-bridge control pins to textual
 * representations
 */
enum DriveMotorControlPins {
	DM_LEFT_EN = 3,  ///< Enable pin of H-bridge controlling left motors
	DM_LEFT_IN0 = 5, ///< IN0 pin of H-bridge controlling left motors
	DM_LEFT_IN1 = 6, ///< IN1 pin of H-bridge controlling left motors

	DM_RIGHT_EN = 4, ///< Enable pin of H-bridge controlling right motors
	DM_RIGHT_IN0 = 7,///< IN0 pin of H-bridge controlling right motors
	DM_RIGHT_IN1 = 8,///< IN1 pin of H-bridge controlling right motors
};

/*!
 * \brief Enumerator matching EDF group indices to textual representations
 * of the EDF positions.
 */
enum EDFPositions {
	EDFS_FRONT = 0, ///< Index of front EDFs (EDFs closest to micro-controller)
	EDFS_MIDDLE,    ///< Index of middle EDFs (EDFs closest to battery)
	EDFS_REAR,	    ///< Index of rear EDFs (EDFs closest to drive H-bridge)
};

/*!
 * \brief Enumerator matching EDF control pins to textual representations.
 */
enum EDFControlPins {
	EDFS_CTRL_F = 9,	///< Control pin of front EDFs
	EDFS_CTRL_M = 10,	///< Control pin of middle EDFs
	EDFS_CTRL_B = 16,	///< Control pin of rear EDFs
};


/*!
 * \brief Enumerator mapping safety switch pins to textual representations.
 */
enum SafetySwitchPins {
	SAFES_IN = 20, 	   ///< Pin used to read the safety switch's state.
	SAFES_REF = 21,    ///< Pin used to provide the safety switch reference.
};



}

#endif /* ROBOTCONSTANTS_HPP_ */
