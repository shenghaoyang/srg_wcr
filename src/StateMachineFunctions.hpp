/*
 * StateMachineFunctions.hpp
 *
 * Declarations for the state machine functions
 *
 *  Created on: 21 Jan 2018
 *      Author: shenghao
 */

#ifndef STATEMACHINEFUNCTIONS_HPP_
#define STATEMACHINEFUNCTIONS_HPP_

#include <StateMachine.hpp>

namespace StatesTest {
	using namespace StateMachine;

	StateStatus state_A(RobotState& rstate, bool first_run);

	StateStatus state_B(RobotState& rstate, bool first_run);

	StateStatus state_C(RobotState& rstate, bool first_run);

	constexpr StateFunction start = state_A;
	constexpr StateTransitionTargets StateTable[] = {
			{ state_A, state_B, nullptr },
			{ state_B, nullptr, state_C },
			{ nullptr, nullptr, nullptr }
	};
}

namespace States {
	using namespace StateMachine;
	/*!
	 * \brief Function executed while the robot is in the state when it is
	 * moving towards the transition point between segments A and B,
	 * from segment A.
	 *
	 * @param rstate Robot state, passed in by state machine supervisor.
	 * @param first_run True if this state was jumped to.
	 * @return state status value for the state machine supervisor.
	 *
	 * \todo Abnormal exit state
	 * \todo PID control for orientation control and motor spin up
	 */
	StateStatus A_to_ABtrans(RobotState& rstate, bool first_run);

	/*!
	 * \brief Function executed while the robot is in the state when it is
	 * at the transition point from segment A to segment B
	 *
	 * \param rstate Robot state, passed in by the state machine supervisor.
	 * \param first_run True if this state was jumped to.
	 * \return state status value for the state machine supervisor
	 *
	 * \todo Abnormal exit state
	 * \todo PID control for orientation control.
	 */
	StateStatus ABtrans_to_B(RobotState& rstate, bool first_run);


	/*!
	 * \brief Function executed while the robot is in the state when it is
	 * climbing segment B towards the transition point between segment
	 * B and segment C.
	 *
	 * \param rstate Robot state, passed in by the state machine supervisor.
	 * \param first_run True if this state was jumped to.
	 * \return state status value for the state machine supervisor
	 *
	 * \todo Abnormal exit state
	 * \todo PID control for orientation control.
	 */
	StateStatus B_to_BCtrans(RobotState& rstate, bool first_run);

	/*!
	 * \brief State table designating all the possible state transitions
	 */
	constexpr StateTransitionTargets StateTable[] = {
		{ A_to_ABtrans, ABtrans_to_B, nullptr },
		{ ABtrans_to_B, B_to_BCtrans, nullptr },
		{ B_to_BCtrans, nullptr		, nullptr },
		{ nullptr	  , nullptr		, nullptr }
	};
	/*!
	 * \brief State machine root function
	 */
	constexpr StateFunction start = A_to_ABtrans;
};





#endif /* STATEMACHINEFUNCTIONS_HPP_ */
