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

namespace TeensyVL53L0X {
class TeensyVL53L0X: public VL53L0X {
private:
	i2c_t3* wire;
	virtual void writeReg(uint8_t reg, uint8_t value) override {
		wire->beginTransmission(address);
		wire->write(reg);
		wire->write(value);
		last_status = Wire.endTransmission();
	}

	virtual void writeReg16Bit(uint8_t reg, uint16_t value) override {
		wire->beginTransmission(address);
		wire->write(reg);
		wire->write((value >> 8) & 0xff);
		wire->write(value & 0xff);
		last_status = wire->endTransmission();
	}

	virtual void writeReg32Bit(uint8_t reg, uint32_t value) override {
		wire->beginTransmission(address);
		wire->write(reg);
		wire->write((value >> 24) & 0xff);
		wire->write((value >> 16) & 0xff);
		wire->write((value >> 8) & 0xff);
		wire->write(value & 0xff);
		last_status = wire->endTransmission();
	}

	virtual uint8_t readReg(uint8_t reg) override {
		uint8_t value;
		wire->beginTransmission(address);
		wire->write(reg);
		last_status = wire->endTransmission();

		wire->requestFrom(address, 0x01);
		value = wire->read();

		return value;
	}

	// Read a 16-bit register
	virtual uint16_t readReg16Bit(uint8_t reg) override {
	  uint16_t value;

	  wire->beginTransmission(address);
	  wire->write(reg);
	  last_status = wire->endTransmission();

	  wire->requestFrom(address, (uint8_t)2);
	  value  = (uint16_t)wire->read() << 8; // value high byte
	  value |=           wire->read();      // value low byte

	  return value;
	}

	// Read a 32-bit register
	virtual uint32_t readReg32Bit(uint8_t reg) override {
	  uint32_t value;

	  wire->beginTransmission(address);
	  wire->write(reg);
	  last_status = wire->endTransmission();

	  wire->requestFrom(address, (uint8_t)4);
	  value  = (uint32_t)wire->read() << 24; // value highest byte
	  value |= (uint32_t)wire->read() << 16;
	  value |= (uint16_t)wire->read() <<  8;
	  value |=           wire->read();       // value lowest byte

	  return value;
	}

	// Write an arbitrary number of bytes from the given array to the sensor,
	// starting at the given register
	virtual void writeMulti(uint8_t reg, const uint8_t * src, uint8_t count)
    override {
	  wire->beginTransmission(address);
	  wire->write(reg);
		wire->write(src, count);

	  last_status = wire->endTransmission();
	}

	// Read an arbitrary number of bytes from the sensor, starting at the given
	// register, into the given array
	virtual void readMulti(uint8_t reg, uint8_t * dst, uint8_t count) override {
	  wire->beginTransmission(address);
	  wire->write(reg);
	  last_status = wire->endTransmission();

	  wire->requestFrom(address, count);
		wire->read(dst, count);
	}

public:
	TeensyVL53L0X(i2c_t3& i2chandle): VL53L0X {}, wire {&i2chandle} {
		init();
	}
};
}




#endif /* LIB_SENSORSUPPORT_VL53L0X_HPP_ */
