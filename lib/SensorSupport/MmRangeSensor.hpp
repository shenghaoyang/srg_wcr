/*
 * MmRangeSensor.hpp
 *
 *  Created on: 10 Jan 2018
 *      Author: shenghao
 */

#ifndef LIB_SENSORSUPPORT_MMRANGESENSOR_HPP_
#define LIB_SENSORSUPPORT_MMRANGESENSOR_HPP_

#include <BaseSensor.hpp>

class MmRangeSensor: BaseSensor<uint32_t> {
private:

public:
	virtual uint32_t read() override {
	}
};



#endif /* LIB_SENSORSUPPORT_MMRANGESENSOR_HPP_ */
