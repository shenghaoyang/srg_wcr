/*
 * StateMachine.cpp
 *
 * Definitions for various state machine helper functions
 *
 *  Created on: 21 Jan 2018
 *      Author: shenghao
 */

#include <StateMachine.hpp>

namespace StateMachine {

StateFunction lookup_transition(StateFunction orig_state, StateStatus status,
		const StateTransitionTargets* table) {
	for (const StateTransitionTargets* target_list = table;
			target_list->main_target; target_list++) {
		if (target_list->main_target == orig_state) {
			switch (status) {
			case StateStatus::REPEAT:
				return orig_state;
			case StateStatus::ABORT:
				return target_list->abort_target;
			case StateStatus::NEXT:
				return target_list->next_target;
			default:
				/* We should not end up here */
				return nullptr; // Grumble grumble
			}
		}
	}
	/* We should not end up here, state table is not well defined, then. */
	return nullptr; // Grumble grumble
}

}


