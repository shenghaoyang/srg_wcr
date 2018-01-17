/*
 * BaseSensor.hpp
 *
 *  Created on: 10 Jan 2018
 *      Author: shenghao
 */

#ifndef LIB_SENSORSUPPORT_BASESENSOR_HPP_
#define LIB_SENSORSUPPORT_BASESENSOR_HPP_

#include <stddef.h>
#include <stdint.h>

template<typename ResultType>
class BaseSensor {
public:
	/*!
	 * \brief Obtain the latest sensor reading.
	 * \return Sensor reading (type and unit implementation defined)
	 */
	virtual ResultType read() = 0;

	virtual ~BaseSensor() {}
};




#endif /* LIB_SENSORSUPPORT_BASESENSOR_HPP_ */
