GPP=g++
CFLAGS=
DEBUG=-Wall -g -DDEBUG
LINK= -lmraa

all: BMP

BMP:
	$(GPP) $(CFLAGS) application_BMP180.cpp Adafruit_BMP180.cpp -o BMP180 \
	$(LINK)

BMP_debug:
	$(GPP) $(CFLAGS) $(DEBUG) application_BMP180.cpp Adafruit_BMP180.cpp -o
	BMP180 $(LINK)

clean:
	rm *.o BMP180 *.log
