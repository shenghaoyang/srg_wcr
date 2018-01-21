/*
 * StateMachine.hpp
 *
 * State machine implementation for the wall climbing robot.
 *
 *  Created on: 20 Jan 2018
 *      Author: shenghao
 */

#ifndef LIB_STATEMACHINE_SRC_STATEMACHINE_HPP_
#define LIB_STATEMACHINE_SRC_STATEMACHINE_HPP_

#include <stdint.h>

namespace StateMachine {

/*!
 * \brief Possible return values for a state machine function
 */
enum class StateStatus {
	REPEAT = 0,  ///< Continue to execute the same function
	NEXT,  	 	 ///< Jump to the defined next state
	ABORT,   	 ///< Jump to the defined abort state
};

/*!
 * \brief Structure holding information that will be passed into the function
 * executed when the state machine is in a particular state.
 */
struct RobotState {
	/*!
	 * \brief Range readings, in millimetres, for each sensor.
	 *
	 * Array index | Sensor position
	 * :---------: | :-------------:
	 * 0		   | Front left
	 * 1 		   | Front right
	 * 2 		   | Rear left
	 * 3		   | Rear right
	 *
	 * \warning Read-only values, the function representing a particular
	 * state of the state machine must \b NOT modify them.
	 */
	uint16_t range_readings[4];

	/*!
	 * \brief EDF speed settings, in the range [0, 100], for each EDF
	 *
	 * Array index | EDF position
	 * :---------: | :----------:
	 * 0		   | Front
	 * 1 		   | Middle
	 * 2		   | Rear
	 *
	 * \warning If the EDF speeds are set outside the range [0, 100], unexpected
	 * behavior may manifest. The state machine supervisor is \b NOT guaranteed
	 * to correct the mistakes made by the state machine functions.
	 */
	float edf_speeds[3];

	/*!
	 * \brief Motor speed settings, in the range [-100, 100] for each motor
	 * pair
	 *
	 * Array index | Motor group position
	 * :---------: | :------------------:
	 * 0		   | Left
	 * 1 		   | Right
	 *
	 * \warning If the motor speeds are set outside the range [-100, 100],
	 * unexpected
	 * behavior may manifest. The state machine supervisor is \b NOT guaranteed
	 * to correct the mistakes made by the state machine functions.
	 */
	float motor_speeds[2];
};

/*!
 * \brief Type alias for pointer to a function that will be
 * executed for a particular state of the state machine
 */
using StateFunction = StateStatus(*)(RobotState& rstate, bool first_run);

/*!
 * \brief Structure holding the state transition targets for a
 * particular state machine state.
 *
 * \c nullptr can be assigned to any of these pointers. The state
 * machine supervisor will automatically perform an emergency abort.
 *
 */
struct StateTransitionTargets {
	const StateFunction main_target;	///< State this transition target structure is defined for
	const StateFunction next_target;	///< State to jump to when request for transition to next state is made
	const StateFunction abort_target;	///< State to jump to when request for transition to abort state is made
};

/*!
 * \brief Lookup the next function to execute given a status of a previously
 * executed state machine function.
 *
 * \param orig_state previously executed state machine function
 * \param status status of the executed function
 * \param table state machine lookup table to use
 *
 * \return next function to execute
 * \retval nullptr if there is no next function to execute (state machine exit)
 */
StateFunction lookup_transition(StateFunction orig_state, StateStatus status,
		const StateTransitionTargets* table);
}




#endif /* LIB_STATEMACHINE_SRC_STATEMACHINE_HPP_ */
