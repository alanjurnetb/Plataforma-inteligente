#include "HCSR04.h"

ultraSensor::ultraSensor(char t_pin, char e_pin){
	trigger_pin=t_pin;
	echo_pin=e_pin;
	sensor_status=IDLE;
	pinMode(t_pin, OUTPUT); /*activación del pin 9 como salida: para el pulso ultrasónico*/
 	pinMode(e_pin, INPUT); /*activación del pin 8 como entrada: tiempo del rebote del ultrasonido*/
	
};

void ultraSensor::sense(){
	digitalWrite(trigger_pin,LOW); /* Por cuestión de estabilización del sensor*/
	delayMicroseconds(5);
	digitalWrite(trigger_pin, HIGH); /* envío del pulso ultrasónico*/
	delayMicroseconds(10);		
	measured_time=pulseIn(echo_pin, HIGH); /* Función para medir la longitud del pulso entrante. Mide el tiempo que transcurrido entre el envío
  	del pulso ultrasónico y cuando el sensor recibe el rebote, es decir: desde que el pin 12 empieza a recibir el rebote, HIGH, hasta que
  	deja de hacerlo, LOW, la longitud del pulso entrante*/
  	distance= int(0.017*measured_time); 
}
void ultraSensor::start_measure(void){
	if(micros()-last_measure>DELTA_BEWTWEEN_MEASURES){
		ultraSensor::trigger_sensor();
	}
}

void ultraSensor::trigger_sensor(void){
	if(sensor_status==IDLE){
		digitalWrite(trigger_pin,LOW); /* Por cuestión de estabilización del sensor*/
		delayMicroseconds(5);
		digitalWrite(trigger_pin, HIGH); /* envío del pulso ultrasónico*/
		delayMicroseconds(10);		
		sensor_status=TRIGGERED_S;
	}
};

void ultraSensor::update_measure(){
	if (digitalRead(echo_pin)==1 && sensor_status==TRIGGERED_S){
		sensor_status=MEASURING_ECHO;
		trigger_time=micros();
	}else if(digitalRead(echo_pin)== 0 && sensor_status==MEASURING_ECHO){
		measured_time=reading_time-trigger_time;		
		distance= int(0.017*measured_time);
		sensor_status=IDLE;
		last_measure=micros();
	}
	reading_time=micros();
};

float ultraSensor::get_distance(void){
	return(distance);
};