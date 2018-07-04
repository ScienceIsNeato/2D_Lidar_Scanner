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

void on_finished(RPlidarDriver * drv, Scanner *scanner)
{
	int tmp;
	std::cin >> tmp;
	scanner->Close(drv);
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
		on_finished(drv, scanner);
	}

	// print out the device serial number, firmware and hardware version number..
	printf("RPLIDAR S/N: ");
	for (int pos = 0; pos < 16; ++pos)
	{
		printf("%02X", devinfo.serialnum[pos]);
	}

	printf("\n"
		"Firmware Ver: %d.%02d\n"
		"Hardware Rev: %d\n"
		, devinfo.firmware_version >> 8
		, devinfo.firmware_version & 0xFF
		, (int)devinfo.hardware_version);

	// check health...
	if (!scanner->CheckRPLIDARHealth(drv)) {
		on_finished(drv, scanner);
	}

	signal(SIGINT, ctrlc);

	drv->startMotor();
	// start scan...
	drv->startScan(0, 1);
	const static int CALIBRATION_PNTS = 50;
	double CALIBRATION_SCALE_FACTOR = 0.98;
	double calibration_values[NUM_SAMPLE_POINTS];
	double smoothed_cal_vals[NUM_SAMPLE_POINTS];

	for (int i = 0; i < NUM_SAMPLE_POINTS; i++)
	{
		calibration_values[i] = DEFAULT_CALIBRATION_VALUE;
		smoothed_cal_vals[i] = 0;
	}

	for (int i = 100; i > 0; i--)
	{
		std::cout << "Calibrating in " << i << " ...\n";
		//std::this_thread::sleep_for(std::chrono::seconds(1)); This is broken
	}

	std::cout << "CALIBRATION COMMENCING!\n";
	std::cout << "Calibration countdown! -- " << CALIBRATION_PNTS << std::endl;

	scanner->Calibrate(drv, CALIBRATION_PNTS, calibration_values);
	scanner->SmoothCalibrationResults(calibration_values, smoothed_cal_vals, CALIBRATION_SCALE_FACTOR);

	ScanResult res;
	while (!ctrl_c_pressed)
	{
		res = scanner->Scan(drv, smoothed_cal_vals);
		if (res.valid && res.closest_distance < smoothed_cal_vals[res.closest_index])
		{
			printf("\nshortest theta: %03.2f shortest Dist: %08.2f calibration Dist: %08.2f",
				res.closest_angle,
				res.closest_distance,
				smoothed_cal_vals[res.closest_index]
			);
		}
		else
		{
			std::cout << "." << std::flush;
		}
	}

    drv->stop();
    drv->stopMotor();

    // done!
	on_finished(drv, scanner);    
    return 0;
}

