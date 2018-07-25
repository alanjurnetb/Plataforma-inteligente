
#include "GP2Y0A02YK0F.h"
#include "servo_driver.h"
#include "radar_definitions.h"
#include "I2C_parser.h"

class redgrey_radar{
private:
	unsigned int n_sensors;
	double angle_sensors;
	double center_angle;
	double scan_angle;
	double min_angle;
	double max_angle;
	double max_distance;
	scanner_mode scan_mode;
	int speed; // porcentaje en velocidad (-1 es maximo)

	sensor_data last_measure;


//	unsigned char prob_map;
	GP2Y0A02YK0F *sensors[NSENSORS];
	servo_controller *turret;
	turret_position t_status;


//	unsigned char * localMap;
//	unsigned int map_size;

	void push_measure(sensor_data measure);

	void set_angle(double angle);
public:
	//redgrey_radar(unsigned char * mapdir, unsigned int mapSize);
	redgrey_radar();
	void configuration(double angleSensors, double centerAngle, double scannerAngle, scanner_mode scanMode, double maxDistance,int speed);
//	unsigned char * get_map();
	double get_distance(unsigned int sensor_number);
	void move_to(double angle);
	void update();
	void update_measures_measures();
	void change_mode(scanner_mode new_mode);
	double get_radar_angle(); //relative to the cero setting
	void update_controller();
	sensor_data get_sensor_data();
//	void update_map();

};

