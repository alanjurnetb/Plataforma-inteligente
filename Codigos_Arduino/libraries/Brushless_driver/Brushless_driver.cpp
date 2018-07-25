#include "Brushless_driver.h"
#include "Arduino.h"
/*
void brushless_motor::update_brushless(){
  if(micros()-last_tick_time>500000){
    speed_rpm=0;
  }
  if (Setpoint<0){
    Input=-speed_rpm;  
  }else if (Setpoint>0){
    Input=speed_rpm;    
  }

  if (Setpoint==0){
    Output=CERO_SIGNAL;
    Setpoint=0;
    speedPID->Reset();
  }
       
  speedPID->Compute();

  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print("\tInput: ");
  Serial.print(Input);
  Serial.print("\tSpeed RPM: ");
  Serial.print(speed_rpm);    
  Serial.print("\tOutput: ");
  Serial.println(Output);
  

  motor->writeMicroseconds((int)Output);  

}*/

void brushless_motor::update_brushless(){	
	//Input=get_speed();
  if(micros()-last_tick_time>500000){
    speed_rpm=0;
  }
  if (Setpoint<0){
    Input=-speed_rpm;  
  }else if (Setpoint>0){
    Input=speed_rpm;    
  }
  
  if (speed_rpm==0 && (car_status==UNKNOWN_MOTORSTATUS ||car_status==MOVING)){
    car_status=STALL;     
 //   Serial.println("Stall");
  }else if (car_status==STALL && (Setpoint>0 ||Setpoint<0)){
    last_starting_time=millis();
    car_status=STARTING;
 //   Serial.println("Starting");
  }else  if (car_status==STARTING){
      if (millis()-last_starting_time<STARTING_TIME ){
        if( Setpoint>0){
          speedPID->SetTunings(powerKp, powerKi, powerKd);  
        }else{
          speedPID->SetTunings(power_back_Kp, power_back_Ki, power_back_Kd);
        }
        
        Serial.println("Power");
        Input=0;  
      }else{  
        if(Input==0){
          Serial.println("Error");
          Output=CERO_SIGNAL;
          Setpoint=0;
          speedPID->Reset();
          car_status=UNKNOWN_MOTORSTATUS;
        }
        car_status=MOVING;
      }
      
  }else if (car_status==MOVING){
    if(Setpoint<1000 && Setpoint>-1000 && Setpoint!=0){
      Serial.println("Ultra Conservador");
      speedPID->SetTunings(ultraconsKp, ultraconsKp, ultraconsKp);     
    }else if (Setpoint>1000 ||Setpoint<-1000){
      Serial.println("Conservador");
      speedPID->SetTunings(consKp, consKi, consKd);        
    }    
  }

    speedPID->Compute();

  //if (!braking){
    if (Setpoint==0){
      Output=CERO_SIGNAL;
      Setpoint=0;
      speedPID->Reset();
    }/*else if (Setpoint <-200){
      Setpoint=0;
      motor->writeMicroseconds(1000);  
      delay(10);
      motor->writeMicroseconds(CERO_SIGNAL);  
      
    }*/else{
      if (car_status==MOVING && Output <MIN_MOVING_TORQUE_FORWARD && Setpoint>0){
        Output=MIN_MOVING_TORQUE_FORWARD;
      }else if (car_status==MOVING && Output >MIN_MOVING_TORQUE_BACKWARD && Setpoint<0){
        Serial.print("Output torque: ");
        Serial.println(Output);
        Output=MIN_MOVING_TORQUE_BACKWARD;
      }
    }
 /*}else{
    if(speed_rpm>100){
      Output=1000;  
      Setpoint=0;      
    }else{
      Output=CERO_SIGNAL;  
      Setpoint=0;
      braking=false;
    }
    if (Output == 1000 && prev_speed_rpm<=speed_rpm){
      Output=CERO_SIGNAL;  
      Setpoint=0;
      braking=false; 
    }
  }*/
   /* Serial.print("Setpoint: ");
    Serial.print(Setpoint);
    Serial.print("\tInput: ");
    Serial.print(Input);
    Serial.print("\tSpeed RPM: ");
    Serial.print(speed_rpm);    
    Serial.print("\tOutput: ");
    Serial.println(Output);*/
    prev_speed_rpm=speed_rpm;
    motor->writeMicroseconds((int)Output);  
}

float brushless_motor::get_turns(){  
  float turns=shaf_turns*1.0*GEAR_DIFFERENTIAL_RELATION;
  return(turns);
}

void brushless_motor::setup(uint8_t irq_pin, void (*ISR_callback)(void), int value)
{
  attachInterrupt(digitalPinToInterrupt(irq_pin), ISR_callback, value);
  speedPID->SetTunings(consKp, consKi, consKd);
}

void brushless_motor::set_speed(float speed){
  		Setpoint=speed;
      Output_buffer=Output-CERO_SIGNAL;
}


brushless_motor::brushless_motor(){

  pinMode(SPEEDSENSOR,INPUT);
  pinMode(SPEEDSENSOR_VCC,OUTPUT);
  motor = new Servo();
  
  motor->attach(BRUSHLESS);   
  
  digitalWrite(SPEEDSENSOR_VCC,HIGH);

	speedPID=new PID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
	
  last_starting_time=0;
	speedPID->SetSampleTime(CONTROL_DELTA_TIME);
	speedPID->SetOutputLimits(MIN_SIGNAL, MAX_SIGNAL, CERO_SIGNAL);
	speedPID->SetMode(AUTOMATIC);
	Setpoint=0;
  shaf_turns=0;
  last_tick_time=0;
  car_status=UNKNOWN_MOTORSTATUS;
  Output=CERO_SIGNAL;
}


void brushless_motor::brake(){
  braking=true;

}

void brushless_motor::encoder_callback(){
  long now=micros();
  static unsigned int ticks=0;
  static bool new_speed_meas=false;
  long new_delta=0;
  
  if (now-last_tick_time>6000){ // corresponde a 75.4 km/h    
    if(ticks<2){
      ticks++;          
    }else{
      ticks=0;
      new_speed_meas=true;
    }    
  }

  //Serial.println(ticks);
  if(new_speed_meas){
    shaf_turns=shaf_turns+1.0f/TICKS_LAP;    
    //speed_rpm+=20/((now-last_tick_time)*2.0f/1000000.0f);
    //speed_rpm/=2;
    
    new_delta=(now-last_tick_time);
    speed_rpm=(60*1000000.0)/(TICKS_LAP*new_delta);
    speed_rpm=speed_rpm/GEAR_DIFFERENTIAL_RELATION;
   // Serial.println(speed_rpm);
    
    last_tick_time=now;
    new_speed_meas=false;
  }
  //Serial.println(shaf_turns*1.0*GEAR_DIFFERENTIAL_RELATION);
}

 void brushless_motor::set_pid(float Kp, float Ki, float Kd ){
  consKp=Kp;
  consKi=Ki; 
  consKd=Kd;
 }

 float brushless_motor::read_speed(){
  return(speed_rpm);
 }

 void brushless_motor::clear_turns(){
  shaf_turns=0;
 }