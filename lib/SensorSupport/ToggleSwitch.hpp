/*
 * ToggleSwitch.hpp
 *
 * Toggle switch debouncing class,
 *
 *  Created on: 17 Jan 2018
 *      Author: shenghao
 */

#include <Bounce2.h>
#include <Arduino.h>
#include <stdint.h>

#ifndef LIB_SENSORSUPPORT_TOGGLESWITCH_HPP_
#define LIB_SENSORSUPPORT_TOGGLESWITCH_HPP_

namespace ToggleSwitch {

/*!
 * \brief Types of edges detected.
 */
enum class EdgeType {
	EDGE_FALLING,///< EDGE: FALLING
	EDGE_NONE,   ///< EDGE: NONE
	EDGE_RISING, ///< EDGE: RISING
};

class ToggleSwitch: private Bounce {
public:
	/*!
	 * \brief Sets up a new debouncer for a toggle switch
	 *
	 * @param[in] pin pin the toggle switch signal is to be received at
	 * @param[in] pinmode pin mode the toggle switch input pin should be in
	 * @param[in] interval debounce time constant in milliseconds.
	 */
	ToggleSwitch(uint8_t pin, uint8_t pinmode, uint16_t interval=50):
		Bounce {} {
		pinMode(pin, pinmode);
		Bounce::interval(interval);
    delay(1);
		Bounce::attach(pin);
	}

  /*!
   * \brief Sets up a new debouncer for a toggle switch, using another pin
   * to provide the reference signal for this switch.
   *
   * \param[in] pin pin the toggle switch input is connected to
   * \param[in] ref_pin pin the other lead of the toggle switch is connected to
   * \param[in] ref_state the state the reference pin of the toggle switch
   * should be in.
   * \param[in] interval debounce time constant in milliseconds
   */
  ToggleSwitch(uint8_t pin, uint8_t ref_pin, uint8_t ref_state,
               uint16_t interval):
    Bounce {} {
    pinMode(pin, INPUT_PULLDOWN);
    pinMode(ref_pin, ref_state);
    digitalWrite(ref_pin, 1);
    Bounce::interval(interval);
    delay(10);
    Bounce::attach(pin);
  }

	/*!
	 * \brief Updates the state machine handling the debouncing of the toggle
	 * switch.
	 *
	 * \warning This update function must be called regularly in order to ensure
	 * timely detection of state changes.
	 *
	 * @retval true if the toggle switch has changed state
	 * @retval false if the toggle switch has not changed state
	 */
	bool update() {
		return Bounce::update();
	}

	/*!
	 * \brief Returns the edge transition that caused the last state change of
	 * the toggle switch, detected by the \ref update() method.
	 *
	 * \note The \ref update() method \b MUST have returned previously with a
	 * value of \c true for this method to return with time-relevant results.
	 *
	 * \return type of edge transition that caused the last state change.
	 */
	EdgeType edge() {
		if (fell())
			return EdgeType::EDGE_FALLING;
		if (rose())
			return EdgeType::EDGE_RISING;
		return EdgeType::EDGE_NONE;
	}

	void wait_edge(EdgeType typ) {
		do {
			update();
		} while (edge() != typ);
	}

	/*!
	 * \brief Returns the state of the switch, as processed by the debounce
	 * algorithm.
	 *
	 * \note The \ref update() method \b MUST have returned previously with a
	 * value of \c true for this method to return with time-relevant results.
	 *
	 * \retval true the switch input state is high
	 * \retval false the switch input state is low
	 */
	operator bool() {
		return Bounce::read();
	}

};

}


#endif /* LIB_SENSORSUPPORT_TOGGLESWITCH_HPP_ */
