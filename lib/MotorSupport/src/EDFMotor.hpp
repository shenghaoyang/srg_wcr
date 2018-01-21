/*
 * EDFMotor.hpp
 *
 * EDF Motor class for the Wall climbing robot.
 *
 *  Created on: 10 Jan 2018
 *      Author: shenghao
 */

#ifndef LIB_MOTORSUPPORT_SRC_EDFMOTOR_HPP_
#define LIB_MOTORSUPPORT_SRC_EDFMOTOR_HPP_

#include <UniPWMMotor.hpp>
#include <Arduino.h>

namespace EDFMotor {

class EDFMotor: public UniPWMMotor {
private:
	uint8_t ctrl_0;		 ///< Motor control pin
	float cur_pow {0.f}; ///< Current motor power
	static constexpr uint16_t max_duty {65535}; ///< Maximum duty cycle for \ref analogWrite()
public:
	/*!
	 * \brief Constructor for the EDFMotor motor class.
	 *
	 * This constructor performs these functions:
	 *
	 * - Sets the motor PWM frequency to the minimum allowed
	 * - Sets the motor to COAST mode with high impedance across its terminals.
	 *
	 * @param ctrl pin connected to the motor's control pin, i.e. the gate
	 * pin of the MOSFET controlling power to this motor.
	 */
	EDFMotor(uint8_t ctrl): ctrl_0 {ctrl} {
		/* Setup output modes */
		pinMode(ctrl, OUTPUT);
		/*
		 * Setup virtual resolution
		 */
		analogWriteResolution(16); // MUST MATCH THAT USED BY OTHER CLASSES.
		/* Setup PWM frequency to the minimum allowed */
		set_frequency(min_frequency());
		/* Setup initial state, COAST with no power applied */
		set_power(cur_pow);
	}

	/* Implementing BasePWMMotor virtual functions */
	virtual void set_frequency(float freq) override final {
		analogWriteFrequency(ctrl_0, freq);
	}

	virtual float max_frequency(void) const override final {
		return 1000.00f;
	}

	virtual float min_frequency(void) const override final {
		return 732.4218f;
	}

	/* Implementing BaseUniMotor virtual functions */
	virtual void set_power(float pow) override {
		constrain(pow, 0.0f, 100.0f);
		cur_pow = pow;
		uint16_t mapped_duty = round(
				(fabs(pow) / 100.0f) * static_cast<float>(max_duty));
		analogWrite(ctrl_0, mapped_duty);
	}

	virtual float power(void) const override {
		return cur_pow;
	}

	virtual void flt(void) override {
		analogWrite(ctrl_0, 0);
	}

	/*!
	 * \brief Destructor for the EDFMotor class.
	 *
	 * Simply stops the motor and puts it in COAST mode.
	 */
	~EDFMotor() {
		flt();
	}
};

}



#endif /* LIB_MOTORSUPPORT_SRC_EDFMOTOR_HPP_ */
