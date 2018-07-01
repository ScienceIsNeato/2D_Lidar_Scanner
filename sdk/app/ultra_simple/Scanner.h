#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "../include/rplidar.h" //RPLIDAR standard sdk, all-in-one header

#define NUM_SAMPLE_POINTS 8192


using namespace rp::standalone::rplidar;

class Scanner
{
public:
	Scanner();
	~Scanner();

	bool CheckRPLIDARHealth(RPlidarDriver * drv);
	void Close(RPlidarDriver * drv);
	void Initialize(RPlidarDriver * drv);
	void Calibrate(RPlidarDriver * drv, int num_samples, double (&calibration_results) [NUM_SAMPLE_POINTS], double scale_factor);

};
