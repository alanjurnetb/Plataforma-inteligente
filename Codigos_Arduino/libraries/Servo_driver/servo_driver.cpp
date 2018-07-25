#include "servo_driver.h"
#include "Arduino.h"

#define SERVO 6
#define SERVO_SENSOR A0
#define SERVO_SENSOR_SAMPLES 10

#define VECTORSAMPLES 4
#define CONTROL_DELTA_TIME 10 //ms

#define MIN_ANGLE 15  // Servo comando a ojo
#define CERO_ANGLE 55
#define MAX_ANGLE 100
#define MAX_ANGLE_RANGE_SERVO 200

#define MAX_ERROR 3



void servo_controller::update(){
  Input=get_angle_absolute();
  double out_pwm;
//  ////Serial.println(Input);
	if (pid_active){
    servoPID->Compute();    
    out_pwm=Output*MAX_ANGLE_RANGE_SERVO*1.0/100;
  }else{
    out_pwm=Setpoint;
    if(arrived==false && millis()-setPoint_time_stamp >2000){
      arrived=true;
    }
  }

  
  direccion->write (out_pwm);
  //////Serial.print("Servo enviado a:");
  //////Serial.println(out_pwm);
  if(servo_sensor_status){
    if (sqrt(pow((Input-Setpoint),2))<MAX_ERROR){
          arrived=true;
    }else{
       arrived=false;
    }
  }
 
}

void servo_controller::enable_pid(){
  pid_active=true;
}

void servo_controller::disable_pid(){
  pid_active=false;
}




void servo_controller::set_angle(double angle){
 // if (angle <= max_angle_limit+1 && angle>=min_angle_limit-1){
    Setpoint=angle+90;
//  }
 	
  setPoint_time_stamp=millis();
}

void servo_controller::set_angle_limits(unsigned int min_angle_lim,unsigned int cero_angle_lim,unsigned int max_angle_lim){
  max_angle_limit=max_angle_lim;
  min_angle_limit=min_angle_lim;
  center_angle_limit=cero_angle_lim;
  ////Serial.println("configuration servo:");
   ////Serial.print ("min_angle_limit:");
    ////Serial.print (min_angle_limit); 
    ////Serial.print ("  max_angle_limit:");
    ////Serial.print (max_angle_limit);
    ////Serial.print ("  center_angle_limit:");
    ////Serial.println (center_angle_limit);
  
}



servo_controller::servo_controller(int pin, int sensor_pin){
	////Serial.println("New Servo controller.");  
  servo_pin=pin;
  arrived=false;  
  servo_sensor_pin=sensor_pin;
	servoPID = new PID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
	direccion= new Servo();  	
	pinMode(servo_pin,OUTPUT);
  
  max_angle_limit=MAX_ANGLE;
  min_angle_limit=MIN_ANGLE;
  center_angle_limit=CERO_ANGLE;
  Setpoint=center_angle;
	direccion->attach(servo_pin); 
	
	servoPID->SetSampleTime(CONTROL_DELTA_TIME);
	servoPID->SetOutputLimits(min_angle_limit, max_angle_limit,center_angle_limit);
	servoPID->SetMode(AUTOMATIC);
}


double  servo_controller::get_raw_angle(){
  static double servo_angle;
  static unsigned int last_angle;
  static double oldSamples_angle[VECTORSAMPLES];

    //servo_angle=10;
    //return(servo_angle);
    for (int i=0;i<SERVO_SENSOR_SAMPLES-1;i++){
      //////Serial.println(analogRead(servo_sensor_pin));
      //servo_angle+=10;
      servo_angle+=analogRead(servo_sensor_pin);
      delay(1);
    }
    
      servo_angle=servo_angle/SERVO_SENSOR_SAMPLES;
    for(int j=0;j<VECTORSAMPLES-1;j++){
      oldSamples_angle[VECTORSAMPLES-1-j]=oldSamples_angle[VECTORSAMPLES-2-j];
    }
    oldSamples_angle[0]=servo_angle;
    servo_angle=6*servo_angle;
    for(int j=0;j<VECTORSAMPLES;j++){
      servo_angle+=oldSamples_angle[j];
    }
    servo_angle=servo_angle/(6+VECTORSAMPLES);
    return(servo_angle);

}
double servo_controller::get_angle(){
  if (servo_sensor_status==true){
   return (get_raw_angle()*1.0*angle_factor+angle_offset);
  }else{

    return(Setpoint);
  }
}

double servo_controller::get_angle_absolute(){
   return (get_angle()+center_angle_limit);
}

double servo_controller::get_angle_rads(){
  return(get_angle()*3.1415926/180);
}

void servo_controller::startSequence(void){
  //Serial.println("Start sequence.");
  direccion->write (min_angle_limit) ;
  //Serial.println(min_angle_limit);
  //Serial.print("Min angle: ");
  long now_time=millis();
  long seqTime=2000;
  while (millis()-now_time<seqTime){
    min_angle=get_raw_angle();
     
  } 
  
  //Serial.print(min_angle);


//Serial.print(" center angle: ");
  direccion->write (center_angle_limit ) ;
  now_time=millis();
  while (millis()-now_time<seqTime){
    center_angle=get_raw_angle();
    
  }
  
  //Serial.print(center_angle);

//Serial.print(" Max angle: ");
  direccion->write (max_angle_limit) ;
  now_time=millis();
  while (millis()-now_time<seqTime){
    max_angle=get_raw_angle();
  }
  
  //Serial.println(max_angle);


  direccion->write (center_angle_limit ) ;
  
  if (sqrt(pow((min_angle-max_angle),2))<2 || max_angle<min_angle || center_angle<min_angle || center_angle>max_angle || servo_sensor_status==false){
    servo_sensor_status=false;
    //Serial.println("El sensor del servo no esta funcionando o esta mal conectado.");
    //Serial.println("El sistema funcioanara sin feedback");
    center_angle=center_angle_limit;
    max_angle=max_angle_limit;
    min_angle=min_angle_limit;
    Setpoint=center_angle;
  }else{
    servo_sensor_status=true;
    //Serial.println("Configuracion correcta");
  }
  angle_factor=((center_angle_limit-min_angle_limit)*1.0/(center_angle-min_angle)+(max_angle_limit-center_angle_limit)*1.0/(max_angle-center_angle))/2;
  angle_offset=((center_angle_limit*1.0-angle_factor*1.0*center_angle)+(max_angle_limit-angle_factor*max_angle))/2-center_angle_limit;
  

  delay(1000);


}

 void servo_controller::set_pid(double Kp, double Ki, double Kd ){
  consKp=Kp;
  consKi=Ki; 
  consKd=Kd;
 }


 bool servo_controller::arrived_to_setpoint(){

    return arrived;
 }
 