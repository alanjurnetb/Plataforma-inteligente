

#include <SPI.h>
#include "CComands.h"
#include "rf_parser.h"

#include "serialParser.h"


#define XAXIS A2
#define YAXIS A1
#define SWITCH 2

serialManager *manager_serial;
rf_communication *rf24;
long lastdatatime;
car_data real_data;
bool remote_control=false;
bool connected_rf=false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);

  manager_serial = new serialManager();
 Serial.println("New rfManager");
  rf24=new rf_communication(0,MASTER,9,10);
  rf24->add_callback(show_turns,GET_TURNS);
  rf24->add_callback(show_motor_speed,GET_MOTOR_SPEED);
  rf24->add_callback(show_battery_status,BATTERY_STATUS);
  rf24->add_car_data_callback(get_car_data);
  rf24->add_callback(set_turret_manual_mode,SET_TURRET_MANUAL_MODE);
  rf24->add_callback(set_turret_auto_mode,SET_TURRET_AUTO_MODE);
  rf24->add_callback(reset_controller,RESET_CONTROLLER);
 
  Serial.println("Starting rfManager");
  rf24->start();

  manager_serial->add_callback(set_motor_speed,SET_MOTOR_SPEED);
  manager_serial->add_callback(turn_steering,TURN_STEERING);
  manager_serial->add_callback(get_motor_speed,GET_MOTOR_SPEED);
  manager_serial->add_callback(get_turns,GET_TURNS);
  manager_serial->add_callback(get_battery_status,BATTERY_STATUS);
  manager_serial->add_callback(set_turret_manual_mode,SET_TURRET_MANUAL_MODE);
  manager_serial->add_callback(set_turret_auto_mode,SET_TURRET_AUTO_MODE);
  manager_serial->add_callback(get_radar_measures,GET_RADAR_MEASURES);
  manager_serial->add_callback(reset_controller,RESET_CONTROLLER);

  //settings
  manager_serial->add_callback(send_new_x,SEND_CAR_NEW_X);
  manager_serial->add_callback(send_new_y,SEND_CAR_NEW_Y);
  manager_serial->add_callback(send_new_theta,SEND_CAR_NEW_THETA);
  manager_serial->add_callback(send_fil_x,SEND_CAR_X_FILTERED);
  manager_serial->add_callback(send_fil_y,SEND_CAR_Y_FILTERED);
  manager_serial->add_callback(send_fil_theta,SEND_CAR_THETA_FILTERED);
  manager_serial->add_callback(control_mode,CONTROL_MODE);
  manager_serial->add_callback(show_car_data,SHOW_ALL_DATA);

  manager_serial->add_callback(push_new_set_point,NEW_SET_POINT);
  manager_serial->add_callback(clear_goals,CLEAR_GOALS);
  manager_serial->add_callback(start_control,START_CONTROL);
  manager_serial->add_callback(stop_control,STOP_CONTROL);


  lastdatatime=millis();
  Serial.println("Ready");  
  pinMode(XAXIS,INPUT);
  pinMode(YAXIS,INPUT);
  pinMode(SWITCH,INPUT);
  digitalWrite(SWITCH, HIGH);
  attachInterrupt(digitalPinToInterrupt(SWITCH),change_control_mode,RISING );
}

void change_control_mode(){
  
  static long last_change=0;
  long now=millis();
  if(now-last_change>1000){    
  if (remote_control){
      remote_control=false;
      Serial.println("Control automatico.");
    }else{
      remote_control=true;
      Serial.println("Control manual.");
    }
    last_change=now;
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  manager_serial->update();
  rf24->update();
  long now=millis();
  if (now-lastdatatime>50){
 //   Serial.println("get_radar");
    
    lastdatatime=now;
    if (remote_control){
      int set_speed=-((analogRead(YAXIS)-1024.0/2)*200.0*2/1024);
      int set_angle=-((analogRead(XAXIS)-1024.0/2)*50.0*2/1024);
      Serial.println(set_speed);
      Serial.println(set_angle);
      set_motor_speed(set_speed);
      turn_steering(set_angle);
    }else{
      get_radar_measures(1);
    }
  }

  
}


void push_new_set_point(float data){
  Serial.println("Pushing new point");
  
  rf24->send_message(NEW_SET_POINT,data);
  Serial.println("Ready");  
}
void clear_goals(float data){
  Serial.println("Clear_goals");
  
  rf24->send_message(CLEAR_GOALS,data);
  Serial.println("Ready");  
}
void start_control(float data){
  Serial.println("start_control");   
  rf24->send_message(START_CONTROL,data);
  Serial.println("Ready"); 
}
void stop_control(float data){
  Serial.println("stop_control");
  rf24->send_message(STOP_CONTROL,data);
  Serial.println("Ready");  
}

void send_new_x(float data){
  Serial.print("Setear new x: ");
  Serial.print(data);
  Serial.println(" m");  
  
  rf24->send_message(SEND_CAR_NEW_X,data);
  Serial.println("Ready");  
}

void send_new_y(float data){
  Serial.print("Setear new y: ");
  Serial.print(data);
  Serial.println(" m");  
  
  rf24->send_message(SEND_CAR_NEW_Y,data);
  Serial.println("Ready");  
}

void send_new_theta(float data){
  //Serial.print("Setear new theta: ");
  //Serial.print(data);
  //Serial.println(" m");   
  rf24->send_message(SEND_CAR_NEW_THETA,data);
}


void send_fil_x(float data){
  //Serial.print("Setear filtered x: ");
  //Serial.print(data);
  //Serial.println(" m");  
  rf24->send_message(SEND_CAR_X_FILTERED,data);
}

void send_fil_y(float data){
  //Serial.print("Setear filtered y: ");
  //Serial.print(data);
  //Serial.println(" m");    
  rf24->send_message(SEND_CAR_Y_FILTERED,data);
}

void send_fil_theta(float data){
  //Serial.print("Setear filtered theta: ");
  //Serial.print(data);
  //Serial.println(" grads");   
  rf24->send_message(SEND_CAR_THETA_FILTERED,data);
}
void control_mode(float data){
  //Serial.print("Setear modo de control: ");
  //Serial.print(data);
    //Serial.println(" n modo"); 
  rf24->send_message(CONTROL_MODE,data);
}


void set_motor_speed(float speed){
  //Serial.print("Sending speed");
  //Serial.println(speed);
  rf24->send_message(SET_MOTOR_SPEED,speed);
  
}

void turn_steering(float angle){
  rf24->send_message(TURN_STEERING,angle);
}

void get_turns(float data){
  rf24->send_message(GET_TURNS,data);
}

void get_motor_speed(float data){
  rf24->send_message(GET_MOTOR_SPEED,data);
}

void get_battery_status(float data){
  rf24->send_message(BATTERY_STATUS,data);
}

void set_turret_manual_mode(float data){
  //Serial.print("Moviendo torre a: ");
  //Serial.println(data);
  rf24->send_message(SET_TURRET_MANUAL_MODE,data);
}
void reset_controller(float data){
  //Serial.println("Reseteando datos");
  rf24->send_message(RESET_CONTROLLER,data);
  Serial.println("Ready");  
}

void set_turret_auto_mode(float data){
 
 rf24->send_message(SET_TURRET_AUTO_MODE,data);
}


void show_turns(float data){
  /*Serial.print("Vueltas: ");
  Serial.println(data);*/
}

void show_motor_speed(float data){
  /*Serial.print("Velocidad: ");
  Serial.print(data);
  Serial.println(" RPM");*/
}

void show_battery_status(float data){
 /* Serial.print("Bateria: ");
  Serial.print(data);
  Serial.println(" V");*/
}

void get_radar_measures(float data){
  rf24->send_message(GET_RADAR_MEASURES,data);
  
}

/*void send_carsettings(car_data_imposed_serial data){
  Serial.println("Enviando nuevos settings");  
  Serial.print("Angulo nuevo: ");
  Serial.print(data.sensor_angle);
  Serial.print("\tXf: ");
  
  Serial.print(data.x_position_filtered);
  Serial.print("\tYf");
  Serial.print(data.x_position_filtered);
  Serial.print("\tTf");
  Serial.print(data.theta_heading_filtered);
  Serial.print("\tPose active");
  Serial.print(data.pose_control);
  Serial.print("\tXg");
  Serial.print(data.x_position_goal);
  Serial.print("\tYg");
  Serial.print(data.y_position_goal);
  Serial.print("\tTg");
  Serial.println(data.theta_heading_goal);
  car_data_imposed temp;
  
  temp.sensor_angle =data.sensor_angle ;
  temp.x_position_filtered = data.x_position_filtered ;
  temp.y_position_filtered =data.y_position_filtered ;
  temp.theta_heading_filtered =data.theta_heading_filtered ;
  temp.pose_control = data.pose_control;
  temp.x_position_goal = data.x_position_goal ;
  temp.y_position_goal =data.y_position_goal ;
  temp.theta_heading_goal =data.theta_heading_goal;
 
  rf24->send_car_settings(temp);
}*/

void get_car_data(car_data data){
  real_data=data;
  if(!connected_rf){
    Serial.println("OK");  
    connected_rf=true;
  }
  //show_car_data(data);
  
  //Serial.println(data.x_position);
}

void show_car_data(car_data data){
  //Serial.println("---------- Auto --------------");
//  Serial.print("Angulo de giro: ");
  //Serial.println("Car:\t");
  Serial.println("C0:");
  Serial.println(real_data.x_position);
  //Serial.print(" \t");
  //Serial.print(" \tVueltas de eje: ");
  Serial.println("C1:");
  Serial.println(real_data.y_position);
  //Serial.print(" \t");
  Serial.println("C2:");
  Serial.println(real_data.theta_heading);
  //Serial.println("Radar:\t");
/*  Serial.print(data.time_stamp);  

  Serial.print(" \t");
  Serial.print(data.sensor_angle,4);
  Serial.print(" \t");
/*
 //// Serial.print("Angulo de torreta: ");
 // Serial.println(data.center_angle);
 // Serial.println("------- Sensores --------");*/
  for (int i=0;i<5;i++){
    Serial.print("R");
    Serial.print(i);
    Serial.println(":");
    Serial.println(real_data.distance[i]);
    //Serial.print("\t");
  }
  //Serial.println("");  
}

