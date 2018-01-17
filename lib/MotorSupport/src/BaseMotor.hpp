/*
 * BaseMotor.hpp
 *
 *  Created on: 10 Jan 2018
 *      Author: shenghao
 */

#ifndef LIB_MOTORSUPPORT_SRC_BASEMOTOR_HPP_
#define LIB_MOTORSUPPORT_SRC_BASEMOTOR_HPP_

class BaseMotor {
public:
	/*!
	 * \brief Sets the motor power.
	 * \param pow motor power in the range [-100, 100]. Negative motor powers
	 * correspond to a motor reversing and positive motor powers correspond
	 * to forward motor rotation.
	 *
	 * So long as higher motor power levels result in faster motor rotation,
	 * implementations are free to use whatever sort of motor power control
	 * they wish. There needs to be no explicit mathematical transformation
	 * from motor power levels into motor rotation speed.
	 */
	virtual void set_power(float pow) = 0;
	/*!
	 * \brief Obtain the current motor power.
	 * @return current motor power in the range [-100, 100].
	 */
	virtual float power(void) const = 0;

	/*!
	 * \brief Stop the motor and place the motor in brake mode so that
	 * motor shaft rotation is resisted by the motor.
	 */
	virtual void brake(void) = 0;

	/*!
	 * \brief Stop the motor and place the motor in coast mode so that
	 * it can still turn with little resistance.
	 */
	virtual void flt(void) = 0;

	/*!
	 * \brief Halt the motor using an implementation defined method. This halt
	 * method should be the same as calling \ref set_power() with an argument
	 * of 0.00
	 */
	virtual void halt(void) = 0;

	virtual ~BaseMotor(void) {

	}
};



#endif /* LIB_MOTORSUPPORT_SRC_BASEMOTOR_HPP_ */
