/*
 * BaseUniMotor.hpp
 *
 *  Created on: 10 Jan 2018
 *      Author: shenghao
 */

#ifndef LIB_MOTORSUPPORT_SRC_BASEUNIMOTOR_HPP_
#define LIB_MOTORSUPPORT_SRC_BASEUNIMOTOR_HPP_

class BaseUniMotor {
public:
	/*!
	 * \brief Sets the motor power
	 * @param pow motor power in the range [0, 100].
	 *
	 * So long as higher motor power levels result in faster motor rotation,
	 * implementations are free to use whatever sort of motor power control
	 * they wish. There needs to be no explicit mathematical transformation
	 * from motor power levels into motor rotation speed.
	 *
	 * The motor is expected to enter coast mode with a power level of
	 * 0.0 commanded.
	 */
	virtual void set_power(float pow) = 0;

	/*!
	 * \brief Obtain the current motor power.
	 * @return current motor power in the range [-100, 100]
	 */
	virtual float power(void) const = 0;

	/*!
	 * \brief Stop the motor and place the motor in coast mode so that
	 * it can still turn with little resistance.
	 */
	virtual void flt(void) = 0;

	virtual ~BaseUniMotor() {}
};



#endif /* LIB_MOTORSUPPORT_SRC_BASEUNIMOTOR_HPP_ */
