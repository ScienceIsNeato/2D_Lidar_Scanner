#include "Scanner.h"



Scanner::Scanner()
{
}


Scanner::~Scanner()
{
}

bool Scanner::CheckRPLIDARHealth(RPlidarDriver * drv)
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;


	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// drv->reset();
			return false;
		}
		else {
			return true;
		}

	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}

void Scanner::Close(RPlidarDriver * drv)
{
	RPlidarDriver::DisposeDriver(drv);
	drv = NULL;
}

void Scanner::Calibrate(RPlidarDriver * drv, int num_samples, double (&calibration_results) [NUM_SAMPLE_POINTS], double scale_factor)
{
	for(int calibration_counter = 0; calibration_counter <= num_samples; calibration_counter++) 
	{
		rplidar_response_measurement_node_t nodes[NUM_SAMPLE_POINTS];
		size_t   count = _countof(nodes);
		u_result op_result = drv->grabScanData(nodes, count);

		std::cout << num_samples - calibration_counter << " (ok) , ";


		if (IS_OK(op_result)) 
		{
			drv->ascendScanData(nodes, count);
			std::cout << num_samples - calibration_counter << " (NOT OK), ";
			
			for (int pos = 0; pos < (int)count; ++pos)
			{
				double dist = nodes[pos].distance_q2 / 4.0f;
				if (dist > 0)
				{
					if (dist < calibration_results[pos])
					{
						calibration_results[pos] = dist;
					}
				}
			}
		}
	}

	// Multiply the results by the scale factor
	for (int i = 0; i < NUM_SAMPLE_POINTS; i++)
	{
		calibration_results[i] = calibration_results[i] * 0.9;
		//std::cout << "\nseed sum: " << calibration_seeds[i][0] << "count: " << calibration_seeds[i][1] << "result: " << calibration_values[i] << std::endl;
		//std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}
}

