# BBB_BMP180
A C++ BMP180 sensor driver written for the Beagle Bone Black.

This driver is developed to work with the BMP180 sensor and the BeagleBone Black. It is a port of the Adafruit-BMP180-Library for the Arduino. This port replaces the Arduino Wire.h library with Derek Molloy's I2CDevice library for the BeagleBone Black. 

# Requirements
- Derek Molloy's I2CDevice library for the BeagleBone Black (https://github.com/derekmolloy/exploringBB/tree/master/chp08/i2c/cpp)

# Version History
02/12/2015: Fix read16() and readS16() so that getTemperature() and getPressure() return expected values.
