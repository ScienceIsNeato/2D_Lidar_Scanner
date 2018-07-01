#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "../include/rplidar.h" //RPLIDAR standard sdk, all-in-one header

#define NUM_SAMPLE_POINTS 8192
#define DEFAULT_CALIBRATION_VALUE 15000.0

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


using namespace rp::standalone::rplidar;

struct ScanResult
{
	double closest_angle;
	double closest_distance;
	int closest_index;
	bool valid;
};

class Scanner
{
public:
	Scanner();
	~Scanner();

	bool CheckRPLIDARHealth(RPlidarDriver * drv);
	void Close(RPlidarDriver * drv);
	void Initialize(RPlidarDriver * drv);
	void Calibrate(RPlidarDriver * drv, int num_samples, double (&calibration_results) [NUM_SAMPLE_POINTS], double scale_factor);
	ScanResult Scan(RPlidarDriver * drv, double(calibration_values)[NUM_SAMPLE_POINTS]);

};

