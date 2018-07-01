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
	int max_attempts = 500;
	int good_samples = 0;
	int attempts = 0;
	while((attempts < max_attempts) && (good_samples < num_samples))
	{
		rplidar_response_measurement_node_t nodes[NUM_SAMPLE_POINTS];
		size_t   count = _countof(nodes);
		u_result op_result = drv->grabScanData(nodes, count);

		if (IS_OK(op_result)) 
		{
			drv->ascendScanData(nodes, count);
			std::cout << num_samples - good_samples << "\n" << std::flush;
			
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
			good_samples++;
		}
		else
		{
			std::cout << "attempt " << attempts << " Failed.\n" << std::flush;
		}
		attempts++;
	}

	std::cout << "Calibration gathered " << good_samples << " good samples out of " << attempts << "attempts.\n" << std::flush;

	// Multiply the results by the scale factor
	int bad_samples = 0;
	for (int i = 0; i < NUM_SAMPLE_POINTS; i++)
	{
		if (calibration_results[i] == DEFAULT_CALIBRATION_VALUE)
		{
			bad_samples++;
		}
		calibration_results[i] = calibration_results[i] * 0.9;
	}

	std::cout << "Calibration found " << NUM_SAMPLE_POINTS - bad_samples << " valid samples out of " << NUM_SAMPLE_POINTS << " total collected.\n" << std::flush;
}

ScanResult Scanner::Scan(RPlidarDriver * drv, double(calibration_values)[NUM_SAMPLE_POINTS])
{
	u_result     op_result;
	ScanResult ret_val;
	ret_val.valid = false;

	// fetech result and print it out...
	rplidar_response_measurement_node_t nodes[NUM_SAMPLE_POINTS];
	size_t count = _countof(nodes);
	op_result = drv->grabScanData(nodes, count);
	double shortest_distance = 1000000;
	double shortest_angle = 0;
	int shortest_index = 0;

	if (IS_OK(op_result))
	{
		drv->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count; ++pos)
		{
			double dist = nodes[pos].distance_q2 / 4.0f;
			if ((dist > 0) &&
				(dist < calibration_values[pos]) &&
				nodes[pos].sync_quality > 40)
			{
				ret_val.closest_distance = dist;
				ret_val.closest_angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
				ret_val.closest_index = pos;
				ret_val.valid = true;
				//std::cout << "accepting b/c " << dist << " is less than " << calibration_values[pos] << " for " << shortest_angle << std::endl;
			}

			//printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
			//    (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
			//    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
			//    nodes[pos].distance_q2/4.0f,
			//    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
		}
	}	
	return ret_val;
}

