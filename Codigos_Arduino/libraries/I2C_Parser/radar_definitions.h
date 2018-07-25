#ifndef __radDef_H__
#define __radDef_H_

#define NSENSORS 5 
#define SENSORANGLE 60 // Angulo entre sensores
#define MAX_DISTANCE_SENSOR 300
#define MEASUREMENT_BUFFER 10


enum scanner_mode {MANUAL_RAD, AUTOMATIC_RAD, CONTROLLED_RAD};
enum turret_position{TRAVELLING, GOING_MIN, GOING_MAX, CENTER,UNKNOWN};

/*typedef struct sensor_data{
	int16_t distance[NSENSORS]; // mm
	double center_angle; // Relative to the centero of the radar
	double separation_angle;
};

typedef struct sensor_data_bytes{
	uint8_t perbyte[sizeof(sensor_data)];
};

typedef union sensor_splitter{
	sensor_data data_formatted;
	sensor_data_bytes data_splitted;
};*/

#endif