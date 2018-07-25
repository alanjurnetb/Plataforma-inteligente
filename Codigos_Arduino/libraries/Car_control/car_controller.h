
#include "Arduino.h"

#define GOAL_POINTS 3
#define GOAL_POSITION_ERROR 1 // en metros
#define MAX_SPEED 100
#define MIN_STEERING_ANGLE -45
#define MAX_STEERING_ANGLE 45
#define TURN_FACTOR 1.384f
#define W_GRID 25
#define W_CELL_SIZE 300 //mm 
#define H_CELL_SIZE 300 //mm
#define H_GRID 25
#define CAR_CENTER_X_GRID 12
#define CAR_CENTER_Y_GRID 12
#define NSENSORS 5
#define SENSOR_RANGE 3000
#define SENSOR_ANGLE_SEPEARATION 45
#define SENSOR_SENS_HALF_ANGLE 2
#define MAX_CELL_VISITED 100
#define OCCUPIED_THRESHOLD 180
#define FREE_THRESHOLD -50
#define OCCUPANCY_WEIGTH 0.51
#define FREE_WEIGTH 0.49
#define DELTA_ANGLE_VFH 45
#define VHF_ZONES 5
#define NEAR_THRESHOLD 400
#define WHEEL_DIAMETER 80 //mm
#define SPEED_KP 800
#define STEERING_KP 1

enum control_mode{POINT, POSE};
enum line_task{UPDATE_FREE_GRID,UPDATE_UNKNOWN_GRID, UPDATE_VFH};
enum cellType{FREE, OCCUPIED, UNKNOWN};

typedef struct coordinates{
	float x;
	float y;
	float theta;
};

typedef struct commands{
	float speed;
	float steering;
};

typedef struct grid_coordinates
{
	unsigned char x;
	unsigned char y;
};

typedef struct vector_histrogram_zone
{
	unsigned int weigth;
	char angle;
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

		bool control_active=false;
		//unsigned char grid[W_GRID*H_GRID];
		char grid[W_GRID*H_GRID];
		vector_histrogram_zone vfh[VHF_ZONES];
		char vhf_index;
		bool danger;
		float front_distance;
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
		void control_system::reset_grid();
		void control_system::reset_car_position();
		double control_system::fill_cell_probability(cellType cell_type, grid_coordinates coord);
		void control_system::Bresenham_plotLineLow(grid_coordinates p0, grid_coordinates p1, line_task task);
		void control_system::Bresenham_plotLineHigh(grid_coordinates p0, grid_coordinates p1, line_task task);
		void control_system::Bresenham_plotLine(grid_coordinates p0, grid_coordinates p1, line_task task);
		void control_system::print_grid(void); 
		void control_system::update_histogram(void);
		void control_system::update_delta_angle(grid_coordinates coord);
		void control_system::print_histogram();
		void control_system::clear_histogram();
		double control_system::get_vhf_steering(double goal_steering_angle);
};