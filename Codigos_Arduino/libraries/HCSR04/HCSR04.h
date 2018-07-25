#include  "Arduino.h"
#define SPEED_SOUND 343
#define DELTA_BEWTWEEN_MEASURES 300

enum sensor_mode {IDLE, TRIGGERED_S, MEASURING_ECHO};

class ultraSensor{
private:
	float distance;
	char trigger_pin;
	char echo_pin;
	double trigger_time;
	double reading_time;
	double measured_time;
	bool triggered=false;
	sensor_mode sensor_status;
	double last_measure;
public:
	ultraSensor::ultraSensor(char t_pin, char e_pin);
	float ultraSensor::get_distance(void);
	void ultraSensor::trigger_sensor(void);
	void ultraSensor::update_measure();
	void ultraSensor::sense();
	void ultraSensor::start_measure(void);
};
