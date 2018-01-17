/*
 * BasePWMMotor.hpp
 *
 *  Created on: 10 Jan 2018
 *      Author: shenghao
 */

#ifndef LIB_MOTORSUPPORT_SRC_BASEPWMMOTOR_HPP_
#define LIB_MOTORSUPPORT_SRC_BASEPWMMOTOR_HPP_

#include <BaseMotor.hpp>
#include <stdint.h>

class BasePWMMotor {
public:
	/*!
	 * \brief Set the frequency of the PWM waveform used to drive
	 * the motor
	 * \param freq frequency of the PWM waveform in Hz.
	 */
	virtual void set_frequency(float freq) = 0;

	/*!
	 * \brief Obtain the maximum PWM frequency supported by PWM controlled motor
	 * \return maximum PWM frequency supported by this motor
	 */
	virtual float max_frequency(void) const = 0;

	/*!
	 * \brief Obtain the minimum PWM frequency supported by PWM controlled motor
	 * \return minimum PWM frequency supported by this motor
	 */
	virtual float min_frequency(void) const = 0;

	virtual ~BasePWMMotor(void) {

	}
};




#endif /* LIB_MOTORSUPPORT_SRC_BASEPWMMOTOR_HPP_ */
