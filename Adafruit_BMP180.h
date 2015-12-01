/***************************************************************************
  This is a library for the BMP180 pressure sensor

  Designed specifically to work with the Adafruit BMP180 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __BMP180_H__
#define __BMP180_H__

#include <stdint.h>
#include "../../I2CDevice.h"

namespace rover {
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BMP180_ADDRESS                (0x77)
/*=========================================================================*/


class Adafruit_BMP180:protected I2CDevice
{
  public:
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
private:
	unsigned int I2CBus, I2CAddress;
	unsigned char *registers;

public:
    Adafruit_BMP180(unsigned int I2CBus, unsigned int I2CAddress = 0x77);
	virtual void writeCommand(unsigned int reg, unsigned char value);
	virtual void read8(unsigned int, uint8_t*);
	virtual void read16(unsigned int, uint16_t*);
	virtual void readS16(unsigned int, int16_t*);
	virtual void readCoefficients();
	virtual void readRawTemperature(int32_t*);
	virtual void readRawPressure(int32_t*);
	virtual short combineRegisters(unsigned char, unsigned char);

    virtual bool  begin(bmp180_mode_t mode = BMP180_MODE_ULTRAHIGHRES);
    virtual void  getTemperature(float *temp);
    virtual void  getPressure(float *pressure);
    virtual float pressureToAltitude(float seaLvel, float atmospheric);
    virtual float seaLevelForAltitude(float altitude, float atmospheric);
    // Note that the next two functions are just for compatibility with old
    // code that passed the temperature as a third parameter.  A newer
    // calculation is used which does not need temperature.
    virtual float pressureToAltitude(float seaLevel, float atmospheric, float temp);
    virtual float seaLevelForAltitude(float altitude, float atmospheric, float temp);
    virtual ~Adafruit_BMP180();

  private:
    virtual int32_t computeB5(int32_t ut);
};

} // bmp180 namespace
#endif
