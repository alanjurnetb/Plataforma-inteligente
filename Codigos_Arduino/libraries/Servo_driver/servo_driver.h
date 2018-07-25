#ifndef __servoDriver_H__
#define __servoDriver_H_


#include <PID_v1.h>
#include <Servo.h>              // Add library  (Kütüphaneyi Ekleyin)


#define SERVO 6
#define SERVO_SENSOR A0
#define SERVO_SENSOR_SAMPLES 10

#define VECTORSAMPLES 4
#define CONTROL_DELTA_TIME 50 //ms

#define MIN_ANGLE 15  // Servo comando a ojo
#define CERO_ANGLE 55
#define MAX_ANGLE 100


class servo_controller{

private:
	
	int max_angle;
	int min_angle;

	int center_angle;

	int servo_pin;
	int servo_sensor_pin;

	int max_angle_limit;
	int min_angle_limit;
	int center_angle_limit;

	double Input, Output, Setpoint;
	double last_setpoint;
	Servo * direccion;
	PID * servoPID;
	bool pid_active=false;
	bool arrived;
	bool servo_sensor_status=true;
	// Factores de coversion entre sensor y angulo
	double angle_factor=1;
	double angle_offset=0;
	
	
	// Constantes de control
	double consKp=0.05, consKi=5, consKd=0.001;
	long setPoint_time_stamp;
	
	double get_raw_angle(); // positivo para la izquierda

public:
	void startSequence();
	servo_controller (int pin, int sensor_pin);
	void set_angle_limits(unsigned int min_angle_lim,unsigned int cero_angle_lim,unsigned int max_angle_lim);
	
	void set_pid(double Kp, double Ki, double Kd );
	void enable_pid();
	void disable_pid();
	
	void update();
	
	void set_angle(double angle);
	bool arrived_to_setpoint();
	
	double get_angle(); // positivo para la izquierda
	double get_angle_rads(); // positivo para la izquierda
	double get_angle_absolute();
	double get_relative_angle_rads();

};

#endif