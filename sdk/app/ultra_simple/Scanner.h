#pragma once
#include "../include/rplidar.h" //RPLIDAR standard sdk, all-in-one header


using namespace rp::standalone::rplidar;

class Scanner
{
public:
	Scanner();
	~Scanner();

	bool checkRPLIDARHealth(RPlidarDriver * drv);

};

