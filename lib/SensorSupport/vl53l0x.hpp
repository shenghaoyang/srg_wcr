/*
 * vl53l0x.hpp
 *
 *  Created on: 17 Jan 2018
 *      Author: shenghao
 */

#include <VL53L0X.h>
#include <i2c_t3.h>

#ifndef LIB_SENSORSUPPORT_VL53L0X_HPP_
#define LIB_SENSORSUPPORT_VL53L0X_HPP_

class TeensyVL53L0X: private VL53L0X {
	TeensyVL53L0X() {
		VL53L0X::init();
	}

};




#endif /* LIB_SENSORSUPPORT_VL53L0X_HPP_ */
