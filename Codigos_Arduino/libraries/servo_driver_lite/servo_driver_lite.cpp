#include "servo_driver_lite.h"


servo_controller_lite::servo_controller_lite(int pin){
	Serial.begin(250000);
	
	direccion= new Servo();  	
	pinMode(pin,OUTPUT);

	max_angle=MAX_ANGLE;
	min_angle=MIN_ANGLE;
	center_angle=CERO_ANGLE;

	direccion->attach(pin); 
	start_sequence();


}

void servo_controller_lite::set_angle_limits(unsigned char min_angle_lim,unsigned char cero_angle_lim,unsigned char max_angle_lim){
	max_angle=max_angle_lim;
	min_angle=min_angle_lim;
	center_angle=cero_angle_lim;
}


void servo_controller_lite::set_angle(double angle){
	angle_setpoint=angle;

	if (angle_setpoint+center_angle <= max_angle+1 && angle_setpoint+center_angle>=min_angle-1){
		direccion->write ((int)(angle_setpoint+center_angle)) ;
	}else if(angle_setpoint+center_angle > max_angle+1){
		direccion->write ((int)max_angle);
	}else if(angle_setpoint+center_angle < min_angle-1){
		direccion->write ((int)min_angle);
	}
}

double servo_controller_lite::get_angle(void){ // positivo para la izquierda
	return(angle_setpoint);
}

void servo_controller_lite::start_sequence(void){
	 
	set_angle(min_angle-center_angle);
	delay(500);
	set_angle(center_angle=-center_angle);
	delay(500);
	set_angle(max_angle-center_angle);
	delay(500);
	set_angle(center_angle-center_angle);
	delay(500);
}