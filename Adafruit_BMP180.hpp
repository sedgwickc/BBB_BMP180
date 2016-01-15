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

#include "mraa.hpp"
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
#define BMP180_DEBUG 0

#define BMP180_I2CADDR 0x77

#define BMP180_ULTRALOWPOWER 0
#define BMP180_STANDARD      1
#define BMP180_HIGHRES       2
#define BMP180_ULTRAHIGHRES  3
#define BMP180_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP180_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP180_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP180_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP180_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP180_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP180_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP180_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP180_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP180_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP180_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP180_CONTROL           0xF4 
#define BMP180_TEMPDATA          0xF6
#define BMP180_PRESSUREDATA      0xF6
#define BMP180_READTEMPCMD          0x2E
#define BMP180_READPRESSURECMD            0x34

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BMP180_ADDRESS                (0x77)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BMP180_REGISTER_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
      BMP180_REGISTER_CHIPID             = 0xD0,
      BMP180_REGISTER_VERSION            = 0xD1,
      BMP180_REGISTER_SOFTRESET          = 0xE0,
      BMP180_REGISTER_CONTROL            = 0xF4,
      BMP180_REGISTER_TEMPDATA           = 0xF6,
      BMP180_REGISTER_PRESSUREDATA       = 0xF6,
      BMP180_REGISTER_READTEMPCMD        = 0x2E,
      BMP180_REGISTER_READPRESSURECMD    = 0x34
    };
/*=========================================================================*/

/*=========================================================================
    MODE SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      BMP180_MODE_ULTRALOWPOWER          = 0,
      BMP180_MODE_STANDARD               = 1,
      BMP180_MODE_HIGHRES                = 2,
      BMP180_MODE_ULTRAHIGHRES           = 3
    } bmp180_mode_t;
/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      int16_t  ac1;
      int16_t  ac2;
      int16_t  ac3;
      uint16_t ac4;
      uint16_t ac5;
      uint16_t ac6;
      int16_t  b1;
      int16_t  b2;
      int16_t  mb;
      int16_t  mc;
      int16_t  md;
    } bmp180_calib_data;
/*=========================================================================*/

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
private:
	mraa::I2c *i2c_bmp180;
	unsigned int I2CAddress;
	}; //BMP180 class
} // rover namespace
