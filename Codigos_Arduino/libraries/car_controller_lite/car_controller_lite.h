
#include "Arduino.h"

#define GOAL_POINTS 3
#define GOAL_POSITION_ERROR 1 // en metros
#define MAX_SPEED 150
#define MAX_BACK_SPEED -200
#define MIN_STEERING_ANGLE -45
#define MAX_STEERING_ANGLE 45
#define TURN_FACTOR 1.384f
#define NSENSORS 5
#define SENSOR_RANGE 3000
#define SENSOR_ANGLE_SEPEARATION 45
#define WHEEL_DIAMETER 80 //mm
#define SPEED_KP 200
#define STEERING_KP 1
#define VFH_SIZE 25
#define VFH_WINDOW 15

///

#define NEAR_THRESHOLD 1000
#define ANGLE_WINDOW 10
#define MAX_VALLEY_SIZE 0 // EN GRADOS

enum control_mode{POINT, POSE};

typedef struct coordinates{
	float x;
	float y;
	float theta;
};

typedef struct commands{
	float speed;
	float steering;
	int turret_angle;
};

typedef struct vhf_model{
	int histogram[VFH_SIZE];
	int angles[VFH_SIZE];
};

typedef struct valley{
	float min_angle;
	float max_angle;
	unsigned char size;
};



class control_system{
	private:
		control_mode mode;
		coordinates goal_coordinates[GOAL_POINTS];
		coordinates next_goal;
		unsigned char goals_left;
		coordinates real_time_position;
		coordinates position_offset;
		coordinates new_position;
		commands new_commands;
		unsigned char push_point_index;
		unsigned char pull_point_index;
		vhf_model vf_histogram;	
		vhf_model old_vf_histogram;	
		bool control_active=false;
		coordinates force_field_vector;
		float distance[NSENSORS];
		float old_distance[NSENSORS];
		float goal_distance;
		int turret_angle;
		bool colition_course;
		

	public:
		control_system::control_system();
		void control_system::update_control(float shaftTurns, float theta);
		void control_system::update_avoider(float sensor_angle, float distance0,float distance1,float distance2,float distance3,float distance4);
		void control_system::set_new_point(coordinates);
		void control_system::set_new_point(float x, float y, float theta);
		coordinates control_system::get_next_point();
		void control_system::clean_points();
		void control_system::start_control();
		void control_system::stop_control();	
		commands control_system::get_commands();
		bool control_system::is_active();
		void control_system::reset_car_position();
		double control_system::get_vhf_steering(double goal_steering_angle);
		int control_system::get_vfh_min_angle();

};