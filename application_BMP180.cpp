/* application_bmp180.cpp
 * Charles Sedgwick
 * This is a small program used for testing the Adafruit_BMP180 driver for the
 * Beaglebone black.
 */

#include <iostream>
#include "Adafruit_BMP180.hpp"
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace rover;

int main() {

	Adafruit_BMP180 BMP180(1,0x77);
	if( BMP180.begin() == false)
	{
		std::cout<<"Could not find a valid BMP180!\n"<<std::flush;
		return 0;
	}

	float* temp = (float*)calloc(1,sizeof(float));
	float* pressure = (float*)calloc(1, sizeof(float));
	std::cout<<"Getting Temps"<<endl;
	BMP180.getTemperature(temp);
	BMP180.getPressure(pressure);

	printf("Temp: %f\n", *temp);
	printf("Pressure: %f\n", *pressure);

	return 0;
}
