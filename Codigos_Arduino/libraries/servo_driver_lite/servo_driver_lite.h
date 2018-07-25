#ifndef __servo_driver_lite_H__
#define __servo_driver_lite_H_

#include <Servo.h>              // Add library  (Kütüphaneyi Ekleyin)
#include "Arduino.h"


#define SERVO 6

#define MIN_ANGLE 16  // Servo comando a ojo
#define CERO_ANGLE 55
#define MAX_ANGLE 94


class servo_controller_lite{

private:
	
	unsigned char max_angle;
	unsigned char min_angle;
	unsigned char center_angle;
	double angle_setpoint;
	Servo * direccion;
	void start_sequence(void);
public:
	servo_controller_lite (int pin);
	void set_angle_limits(unsigned char min_angle_lim,unsigned char cero_angle_lim,unsigned char max_angle_lim); // en coordenadas de servo
	void set_angle(double angle); // El cero esta en el medio, positivo para la izquierda
	double get_angle(void); // positivo para la izquierda
};

#endif