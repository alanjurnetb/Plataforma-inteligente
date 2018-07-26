#ifndef __brushdriver_H__
#define __brushdriver_H_

#include "Arduino.h"
#include <PID_v1.h>
#include <Servo.h> 

#define BRUSHLESS 5
#define SPEEDSENSOR 3
#define SPEEDSENSOR_VCC 2
#define STARTING_TIME 400

#define MIN_MOVING_TORQUE_FORWARD 1565
#define MIN_MOVING_TORQUE_BACKWARD 1435
#define GEAR_DIFFERENTIAL_RELATION 0.3142

#define MAX_RPM 500
#define MIN_RPM (-100)

#define MAX_SIGNAL 2000//(2000-250)
#define CERO_SIGNAL 1500
#define MIN_SIGNAL 1000//(1000+250)

#define CONTROL_DELTA_TIME 100 //ms

#define ENCODER_TIME_SAMPLES 30
#define SPEED_SAMPLES_AVG 20

#define TICKS_LAP 10

enum motor_status{UNKNOWN_MOTORSTATUS,STALL,STARTING, MOVING};

class brushless_motor{

private:

  double Input, Output=CERO_SIGNAL, Setpoint,Output_buffer;
  Servo * motor;
  PID * speedPID;

  float speed_history[SPEED_SAMPLES_AVG];

  float speed_rpm; //rpm
  float prev_speed_rpm; //rpm
  long last_Sense_update;

  float shaf_turns;
  long setpoint_turns;
  // Constantes de control
  double consKp=0.1, consKi=0.1, consKd=0.001;
  double ultraconsKp=0.01, ultraconsKi=0.01, ultraconsKd=0.001;
  double powerKp=1, powerKi=1, powerKd=0.001;
  double power_back_Kp=0.2, power_back_Ki=0.2, power_back_Kd=0.003;
  long last_tick_time;
  long last_starting_time;

  bool braking=false;
  motor_status car_status;


  
//  float get_speed(); // positivo para la izquierda
//  float get_raw_speed(); // positivo para la izquierda
public:
  brushless_motor ();
  void set_pid(float Kp, float Ki, float Kd );
  void update_brushless();
  void set_speed(float angle);
  float read_speed();
  void encoder_callback();
  float get_turns();
  void setup(uint8_t irq_pin, void (*ISR_callback)(void), int value);
  void brake();
  void brushless_motor::clear_turns();

};


#endif