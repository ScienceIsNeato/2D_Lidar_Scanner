/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2018 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "Scanner.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;



#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

void on_finished(RPlidarDriver * drv)
{
	RPlidarDriver::DisposeDriver(drv);
	drv = NULL;
	exit(0);
}

int main(int argc, const char * argv[]) {
	const char * opt_com_path = NULL;
	_u32         baudrateArray[2] = { 115200, 256000 };
	_u32         opt_com_baudrate = 0;
	u_result     op_result;

	Scanner *scanner = new Scanner();

	bool useArgcBaudrate = false;

	printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
		"Version: 1.6.1\n");

	// read serial port from the command line...
	if (argc > 1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

	// read baud rate from the command line if specified...
	if (argc > 2)
	{
		opt_com_baudrate = strtoul(argv[2], NULL, 10);
		useArgcBaudrate = true;
	}

	if (!opt_com_path) {
#ifdef _WIN32
		// use default com port
		opt_com_path = "\\\\.\\com3";
#else
		opt_com_path = "/dev/ttyUSB0";
#endif
	}

	// create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}

	rplidar_response_device_info_t devinfo;
	bool connectSuccess = false;
	// make connection...
	if (useArgcBaudrate)
	{
		if (!drv)
			drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
		if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
		{
			op_result = drv->getDeviceInfo(devinfo);

			if (IS_OK(op_result))
			{
				connectSuccess = true;
			}
			else
			{
				delete drv;
				drv = NULL;
			}
		}
	}
	else
	{
		size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
		for (size_t i = 0; i < baudRateArraySize; ++i)
		{
			if (!drv)
				drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
			if (IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
			{
				op_result = drv->getDeviceInfo(devinfo);

				if (IS_OK(op_result))
				{
					connectSuccess = true;
					break;
				}
				else
				{
					delete drv;
					drv = NULL;
				}
			}
		}
	}
	if (!connectSuccess) {

		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
			, opt_com_path);
		on_finished(drv);
	}

	// print out the device serial number, firmware and hardware version number..
	printf("RPLIDAR S/N: ");
	for (int pos = 0; pos < 16; ++pos) {
		printf("%02X", devinfo.serialnum[pos]);
	}

	printf("\n"
		"Firmware Ver: %d.%02d\n"
		"Hardware Rev: %d\n"
		, devinfo.firmware_version >> 8
		, devinfo.firmware_version & 0xFF
		, (int)devinfo.hardware_version);



	// check health...
	if (!scanner->checkRPLIDARHealth(drv)) {
		on_finished(drv);
	}

	signal(SIGINT, ctrlc);

	drv->startMotor();
	// start scan...
	drv->startScan(0, 1);
	int calibration_counter = 0;
	const static int CALIBRATION_PNTS = 50;
	//double calibration_seeds[8192] = {{ 0.0 }}; // seed sum, num samples
	double calibration_seeds[8192];
	double calibration_values[8192];

	for (int i = 0; i < 8192; i++)
	{
		calibration_seeds[i] = 15000.0;
		calibration_values[i] = 15000.0;
	}
	int bad_counter = 0;

	for (int i = 100; i > 0; i--)
	{
		std::cout << "Calibrating in " << i << " ...\n";
		//std::this_thread::sleep_for(std::chrono::seconds(1)); This is broken
	}

	std::cout << "CALIBRATION COMMENCING!\n";
	std::cout << "Calibration countdown! -- " << CALIBRATION_PNTS << std::endl;



    // fetech result and print it out...
    while (1) {
        rplidar_response_measurement_node_t nodes[8192];

        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);
		double shortest_distance = 1000000;
		double shortest_angle = 0;
		int shortest_index = 0;

		if (calibration_counter == CALIBRATION_PNTS)
		{

			for (int i = 0; i < 8192; i++)
			{
					calibration_values[i] = calibration_seeds[i]*0.9;
					//std::cout << "\nseed sum: " << calibration_seeds[i][0] << "count: " << calibration_seeds[i][1] << "result: " << calibration_values[i] << std::endl;
					//std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			}
			std::cout << "CALIBRATION COMPLETE! only " << bad_counter << " bad samples out of 8192\n";
		}



        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
			if (calibration_counter < CALIBRATION_PNTS)
			{
				std::cout << CALIBRATION_PNTS - calibration_counter << ", ";
			}
            for (int pos = 0; pos < (int)count ; ++pos) {

				double dist = nodes[pos].distance_q2 / 4.0f;
				if (calibration_counter < CALIBRATION_PNTS)
				{
					if (dist > 0)
					{
						if (dist < calibration_seeds[pos])
						{
							calibration_seeds[pos] = dist;
						}
					}
				}
				else
				{ 
					if ((dist > 0) &&
						(dist < calibration_values[pos]) &&
						nodes[pos].sync_quality > 40)
					{
						shortest_distance = dist;
						shortest_angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
						shortest_index = pos;
						//std::cout << "accepting b/c " << dist << " is less than " << calibration_values[pos] << " for " << shortest_angle << std::endl;
					}
				}


                //printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                //    (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                //    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                //    nodes[pos].distance_q2/4.0f,
                //    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
			calibration_counter++;
			//std::cout << "c counter " << calibration_counter << "!\n";


			if (shortest_distance < calibration_values[shortest_index])
			{
				printf("shortest theta: %03.2f shortest Dist: %08.2f calibration Dist: %08.2f\n",
					shortest_angle,
					shortest_distance,
					calibration_values[shortest_index]
				);
				
				//for (auto val : calibration_seeds[shortest_index])
				//{
				//	std::cout  << val << ", ";
				//}
				//std::cout << "/n";
			}
			else
			{
				std::cout << std::endl;
				//printf("INVALID shortest theta: %03.2f shortest Dist: %08.2f , seed value: %03.2f\n", shortest_angle, shortest_distance, calibration_values[shortest_index] );
			}
        }

        if (ctrl_c_pressed){ 
			int dummy;
			std::cin >> dummy;
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    // done!
	on_finished(drv);    
    return 0;
}

