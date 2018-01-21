/*
 * vl53l0x.hpp
 *
 *  Created on: 17 Jan 2018
 *      Author: shenghao
 */

#include <VL53L0X.h>
#include <i2c_t3.h>
#include <Arduino.h>

#ifndef LIB_SENSORSUPPORT_VL53L0X_HPP_
#define LIB_SENSORSUPPORT_VL53L0X_HPP_

namespace TeensyVL53L0X {

class TeensyVL53L0X: public VL53L0X {
private:
	i2c_t3* wire;		///< Pointer to \c i2c_t3 bus handle object
	uint8_t powerctrl;	///< VL53L0X XSHUT pin

	virtual void writeReg(uint8_t reg, uint8_t value, uint32_t timeout=0) override {
		wire->beginTransmission(address);
		wire->write(reg);
		wire->write(value);
		last_status = Wire.endTransmission(I2C_STOP, timeout);
	}

	virtual void writeReg16Bit(uint8_t reg, uint16_t value, uint32_t timeout=0) override {
		wire->beginTransmission(address);
		wire->write(reg);
		wire->write((value >> 8) & 0xff);
		wire->write(value & 0xff);
		last_status = wire->endTransmission(I2C_STOP, timeout);
	}

	virtual void writeReg32Bit(uint8_t reg, uint32_t value, uint32_t timeout=0) override {
		wire->beginTransmission(address);
		wire->write(reg);
		wire->write((value >> 24) & 0xff);
		wire->write((value >> 16) & 0xff);
		wire->write((value >> 8) & 0xff);
		wire->write(value & 0xff);
		last_status = wire->endTransmission(I2C_STOP, timeout);
	}

	virtual uint8_t readReg(uint8_t reg, uint32_t timeout=0) override {
		uint8_t value;
		wire->beginTransmission(address);
		wire->write(reg);
		last_status = wire->endTransmission(I2C_STOP, timeout);

		wire->requestFrom(address, static_cast<uint8_t>(0x01),
				I2C_STOP, timeout);
		value = wire->read();

		return value;
	}

	// Read a 16-bit register
	virtual uint16_t readReg16Bit(uint8_t reg, uint32_t timeout=0) override {
	  uint16_t value;

	  wire->beginTransmission(address);
	  wire->write(reg);
	  last_status = wire->endTransmission(I2C_STOP, timeout);

	  wire->requestFrom(address, (uint8_t)2, I2C_STOP, timeout);
	  value  = (uint16_t)wire->read() << 8; // value high byte
	  value |=           wire->read();      // value low byte

	  return value;
	}

	// Read a 32-bit register
	virtual uint32_t readReg32Bit(uint8_t reg, uint32_t timeout=0) override {
	  uint32_t value;

	  wire->beginTransmission(address);
	  wire->write(reg);
	  last_status = wire->endTransmission();

	  wire->requestFrom(address, (uint8_t)4, I2C_STOP, timeout);
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
	/*!
	 * \brief Sets up a new VL53L0X sensor handle
	 *
	 * @param i2chandle \c i2c_t3 object used to communicate with the sensor
	 * @param xshut pin connected to the XSHUT pin of the VL53L0X
	 * \note By default, the sensor is powered down. Call the \ref poweron()
	 * function to turn it on so that \ref init() can be called.
	 */
	TeensyVL53L0X(i2c_t3& i2chandle, uint8_t xshut):
		VL53L0X {}, wire {&i2chandle}, powerctrl {xshut} {
		powerdown();
	}

	/*!
	 * \brief Sets the I2C address of this sensor
	 *
	 * \pre the sensor has been powered on and initialized
	 * \param new_addr new I2C address in the range [0x00, 0x7f]
	 * \param timeout for the operation in milliseconds
	 * \post the sensor has a new i2c address as specified
	 *
	 * \retval true i2c address has been successfully set
	 * \retval false i2c address has not been successfully set (bus error)
	 * \sa init()
	 * \sa poweron()
	 */
	bool setAddress(uint8_t new_addr, uint32_t timeout=0)
	{
	  writeReg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F, timeout);
	  address = new_addr;
	  if (last_status) {
		  return false;
	  }
	  return true;
	}


	/*!
	 * \brief Sets the sensor into power-down mode by pulling down the
	 * XSHUT pin
	 *
	 * \post the sensor is in powerdown mode
	 * \sa poweron()
	 */
	void powerdown() {
		pinMode(powerctrl, OUTPUT);
		digitalWrite(powerctrl, 0);
	}

	/*!
	 * \brief Sets the sensor into power-on mode by putting the XSHUT pin into
	 * high-z mode.
	 *
	 * \post the sensor has been powered on, after spending time in
	 * hardware standby.
	 * \sa powerdown()
	 */
	void poweron() {
		pinMode(powerctrl, INPUT);
	}


	/*!
	 * \brief Polls the sensor to check if a new ranging result is available
	 *
	 * \pre The sensor must have been initialized through calling
	 * \ref init() and in continuous ranging mode,
	 * through calling \ref startContinuous(), and not powered down.
	 * \param timeout i2c bus timeout in microseconds [0 = inf]
	 * \retval 0 no range result is available
	 * \retval 1 a range result is available
	 * \retval -1 an i2c bus error was encountered
	 * \sa init()
	 * \sa startContinuous()
	 * \sa poweron()
	 */
	int8_t range_available(uint32_t timeout=0) {
		uint8_t result = (readReg(RESULT_INTERRUPT_STATUS, timeout) & 0x07);
		int8_t retval;
		if (last_status) {
			/* I2C bus error */
			retval = -1;
		} else {
			retval = result ? 1 : 0;
		}
		return retval;
	}

	/*!
	 * \brief Obtains the range result from the sensor, clearing the
	 * interrupt flag as well, so the sensor performs another ranging
	 * cycle.
	 *
	 * \pre a new ranging result has been confirmed to be available
	 * through calling \ref range_available()
	 * \param timeout i2c bus timeout in microseconds [0 = inf]
	 * \return range measurement result in millimeters
	 * \retval -1 an i2c bus error was encountered
	 * \post the range available interrupt flag would have been cleared,
	 * so the sensor can range again.
	 * \sa range_available()
	 */
	 int32_t get_and_clear_range(uint32_t timeout=0) {
		 uint16_t range = readReg16Bit(RESULT_RANGE_STATUS + 10, timeout);
		 if (last_status)
			 return -1;
		 writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01, timeout);
		 if (last_status)
			 return -1;
		 return range;
	 }
};
}




#endif /* LIB_SENSORSUPPORT_VL53L0X_HPP_ */
