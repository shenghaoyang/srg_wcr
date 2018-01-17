/*
 * L298NMotor.hpp
 *
 * L298N H-bridge motor driver class for the Wall climbing robot.
 *
 *  Created on: 10 Jan 2018
 *      Author: shenghao
 */

#ifndef LIB_MOTORSUPPORT_SRC_L298NMOTOR_HPP_
#define LIB_MOTORSUPPORT_SRC_L298NMOTOR_HPP_

#include <PWMMotor.hpp>
#include <Arduino.h>

namespace L298NMotor {

/*!
 * \brief Output modes of the L298N
 *
 * The MSB corresponds to the state of IN1, while the LSB corresponds to the
 * state of IN2, that is required to achieve a particular mode.
 */
enum L298NMotorModes {
	FLOAT = 0b00,  ///< Float output mode, motor not being driven.
	REVERSE = 0b01,///< Reverse output mode, motor driven in reverse direction.
	FORWARD = 0b10,///< Forward output mode, motor driven in forward direction.
	BRAKE = 0b11   ///< Brake output mode, motor undergoing active braking.
};

class L298NMotor: public PWMMotor {
private:
	uint8_t in1;		///< L298N Control Pin 1
	uint8_t in2;		///< L298N Control Pin 2
	uint8_t en;			///< L298N Enable Pin
	float cur_pow {0.f};///< Current motor power

	static constexpr uint16_t max_duty {65535}; ///< Maximum duty cycle for \ref analogWrite

	void enable(void) {
		digitalWrite(en, 1);
	}

	void disable(void) {
		digitalWrite(en, 0);
	}

	void set_mode(L298NMotorModes mode) {
		digitalWrite(in1, mode >> 0x01);
		digitalWrite(in2, mode & 0x01);
	}
public:
	/*!
	 * \brief Constructor for the L298N Motor class.
	 *
	 * The constructor performs these functions:
	 *
	 * - Sets the PWM frequency to the minimum allowed.
	 * - Sets the motor to BRAKE mode with no rotation commanded.
	 *
	 * @param ctrl_1 pin connected to the IN1 pin on the motor driver.
	 * @param ctrl_2 pin connected to the IN2 pin on the motor driver.
	 * @param enable pin connected to the enable pin on the motor driver.
	 */
	L298NMotor(uint8_t ctrl_1, uint8_t ctrl_2, uint8_t enable):
		in1 {ctrl_1}, in2 {ctrl_2}, en {enable} {
		/* Setup output modes */
		pinMode(in1, OUTPUT);
		pinMode(in2, OUTPUT);
		pinMode(en, OUTPUT);
		/*
		 * Setup virtual resolution - Note that this is NOT real resolution.
		 * Real resolution depends on the actual PWM frequency, but we
		 * abstract this away.
		 */
		analogWriteResolution(16);
		/* Setup PWM frequency to the minimum allowed */
		set_frequency(min_frequency());
		/* Setup initial state, brake with power of 0.00 */
		set_power(cur_pow);
	}

	/* Implementing BasePWMMotor virtual functions */

	virtual void set_frequency(float freq) override final {
		analogWriteFrequency(en, freq);
	}

	virtual float max_frequency(void) const override final {
		return 22000.00f;
	}

	virtual float min_frequency(void) const override final {
		return 732.4218f;
	}

	/* Implementing BaseMotor virtual functions */
	/*!
	 * \brief Set the motor power by adjusting the PWM duty cycle fed
	 * to the motor
	 * @param pow motor power in the range [-100, 100]
	 *
	 * \note A motor power of 0.00 puts the motor into BRAKE mode.
	 */
	virtual void set_power(float p) override {
		constrain(p, -100.0f, 100.0f);
		cur_pow = p;
		if (p != 0) {
			uint16_t mapped_duty = round(
					(fabs(p) / 100.0f) * static_cast<float>(max_duty));
			if (p > 0)
				set_mode(FORWARD);
			else
				set_mode(REVERSE);
			analogWrite(en, mapped_duty);
		} else {
			brake();
		}
	}

	virtual float power(void) const override {
		return cur_pow;
	}

	virtual void brake(void) override {
		enable();
		set_mode(BRAKE);
	}

	virtual void flt(void) override {
		enable();
		set_mode(FLOAT);
	}

	virtual void halt(void) override {
		brake();
	}

	/*!
	 * \brief Destructor for the L298N motor driver handle.
	 *
	 * Simply disables the motor.
	 */
	virtual ~L298NMotor(void) {
		disable();
	}
};
}


#endif /* LIB_MOTORSUPPORT_SRC_L298NMOTOR_HPP_ */
