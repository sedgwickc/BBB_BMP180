/***************************************************************************
  This library is a port of Adafruit's BMP180 library for Arduino to the Beagle
  Bone Black.

  This port is written and maintained by Charles Sedgwick. 
  This port retains the licence of the software it is based off of which is
  described below.
 ***************************************************************************
  This is a library for the BMP180 pressure sensor

  Designed specifically to work with the Adafruit BMP180 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603
 
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include "Adafruit_BMP180.hpp"
#include "mraa.h"
#include <math.h>
#include <limits.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace std;

namespace rover {

#define BMP180_USE_DATASHEET_VALS (0) /* Set to 1 for sanity check */

class Adafruit_BMP180{

public:
	bmp180_calib_data _bmp180_coeffs;   // Last read accelerometer data will be available here
	uint8_t           _bmp180Mode;

	int32_t computeB5(int32_t ut);

	short combineRegisters(unsigned char msb, unsigned char lsb);

	Adafruit_BMP180(unsigned int I2CBus, unsigned int I2CAddress);
	uint8_t readRegister(unsigned int reg);
	void writeRegister(unsigned int reg, unsigned char value);
	void writeCommand(unsigned int reg, unsigned char value);
	void read8(unsigned int reg, uint8_t *value);
	void read16(unsigned int reg, uint16_t *value);
	void readS16(unsigned int reg, int16_t *value);
	void readCoefficients(void);
	void readRawTemperature(int32_t *temperature);
	void readRawPressure(int32_t *pressure);
	bool begin(bmp180_mode_t mode);
	void getPressure(float *pressure);
	void getTemperature(float *temp);
	float pressureToAltitude(float seaLevel, float atmospheric);
	float pressureToAltitude(float seaLevel, float atmospheric, float temp);
	float seaLevelForAltitude(float altitude, float atmospheric);
	float seaLevelForAltitude(float altitude, float atmospheric, float temp);
	} //BMP180 class
} // bmp180 namespace
