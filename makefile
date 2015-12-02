GPP=g++
CFLAGS=
DEBUG=-Wall -g -DDEBUG

all: BMP

BMP:
	$(GPP) $(CFLAGS) application_BMP180.cpp ../../I2CDevice.cpp Adafruit_BMP180.cpp -o BMP180

BMP_debug:
	$(GPP) $(CFLAGS) $(DEBUG) application_BMP180.cpp ../../I2CDevice.cpp Adafruit_BMP180.cpp -o BMP180

clean:
	rm *.o BMP180 *.log
