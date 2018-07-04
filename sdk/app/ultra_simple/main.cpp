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
#include <signal.h>

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

bool ctrl_c_pressed;

// Local helpers
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

RPlidarDriver* CreateDriver()
{
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}
	return drv;
}

void on_finished(RPlidarDriver * drv, Scanner *scanner)
{
	scanner->Stop(drv);
	int tmp;
	std::cin >> tmp;
	scanner->Close(drv);
	exit(0);
}


// Program entr point
int main(int argc, const char * argv[]) 
{
	double calibration_values[NUM_SAMPLE_POINTS];
	signal(SIGINT, ctrlc); // set signal handler for control c
	Scanner *scanner = new Scanner();
	RPlidarDriver * drv = CreateDriver(); // create the driver instance - for some reason creating it in the Scanner class crashes the program

	if (!(scanner->Start(drv, argc, argv)))
	{
		on_finished(drv, scanner);
	}

	scanner->Calibrate(drv, CALIBRATION_PNTS, calibration_values);

	ScanResult res;
	while (!ctrl_c_pressed)
	{
		res = scanner->Scan(drv, calibration_values);
		if (res.valid && res.closest_distance < calibration_values[res.closest_index])
		{
			printf("\nshortest theta: %03.2f shortest Dist: %08.2f calibration Dist: %08.2f",
				res.closest_angle,
				res.closest_distance,
				calibration_values[res.closest_index]
			);
		}
		else
		{
			std::cout << "." << std::flush;
		}
	}

	on_finished(drv, scanner);    
    return 0;
}

