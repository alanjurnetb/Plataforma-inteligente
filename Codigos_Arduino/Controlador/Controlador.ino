#include "I2C_parser.h"
#include "car_controller_lite.h"
#include <MemoryFree.h>


I2C_communication *controller_com;
control_system *goal_controller;
I2C_command_controller_receive cardata;
bool new_data=false;
coordinates new_goal;

void setup() {
  Serial.begin(250000); 
  Serial.println("Setting up communication");
  controller_com = new I2C_communication(0,CONTROLLER_ID,SLAVE_I2C);
  
  controller_com->add_callback(set_new_x,SEND_CAR_NEW_X); 
  controller_com->add_callback(set_new_y,SEND_CAR_NEW_Y); 
  controller_com->add_callback(set_new_theta,SEND_CAR_NEW_THETA);    
  controller_com->add_callback(push_new_set_point,NEW_SET_POINT); 
  controller_com->add_callback(clear_goals,CLEAR_GOALS);    
  controller_com->add_callback(start_control,START_CONTROL); 
  controller_com->add_callback(stop_control,STOP_CONTROL); 
  controller_com->add_callback(reset_controller,RESET_CONTROLLER); 

  controller_com->add_car_data_callback(update_car_data);
  Wire.onRequest(send_command_data); // register event
  Wire.onReceive(get_commands);

  Serial.println("Setting up controller");
  goal_controller= new control_system(); 
    
  Serial.print("freeMemory()=");
  Serial.println(freeMemory());
}

void loop() {
  // put your main code here, to run repeatedly:
  static long last_update=0;
  long now=millis();
  
  //if (new_data && now-last_update>1){
    goal_controller->update_avoider(cardata.sensor_angle,cardata.distance[0],cardata.distance[1],cardata.distance[2],cardata.distance[3],cardata.distance[4]);
    goal_controller->update_control(cardata.shaft_turns,cardata.heading);
    new_data=false;
    last_update=now;
  //}
}

void get_commands(){
  controller_com->read_callback();
}

void send_command_data(){
  I2C_controller_request commanders;
    commands calc_commands=goal_controller->get_commands();
    commanders.data_formatted.speed=calc_commands.speed;
    commanders.data_formatted.steering=calc_commands.steering;
    commanders.data_formatted.turret_angle=calc_commands.turret_angle;
    controller_com->update_I2C_controllerdata(commanders);
    controller_com->request_callback();
 //   Serial.println("Data sent");
}

void update_car_data(I2C_command_controller_receive data){  
  cardata=data;
  new_data=true;
  
//  Serial.print(data.command);
//  Serial.print("\t");
//  Serial.print(data.sensor_angle);
//  Serial.print("\t");
//  for (int k=0;k<5;k++){
//    Serial.print(data.distance[k]);
//    Serial.print("\t");
//  }
//  Serial.print(data.speed);
//  Serial.print("\t");
//  Serial.print(data.heading);
//  Serial.print("\t");
//  Serial.print(data.shaft_turns);
//  Serial.print("\t");
//  Serial.print(data.steering);
//  Serial.println("");
//  update_control(data.heading,data.shaft_turns);
}





///////////////////////////////////////////////////////////////////////////////////////////// CALLBACKS

void set_new_x(float data){ 
  Serial.print("Setting new x: ");
  Serial.println(data);
  new_goal.x=data;
}
void set_new_y(float data){
  Serial.print("Setting new y: ");
  Serial.println(data);
  new_goal.y=data;
}
void set_new_theta(float data){
  Serial.print("Setting new theta: ");
  Serial.println(data);
  new_goal.theta=data;
}

void push_new_set_point(float data){
  Serial.println("Setting new point");
  goal_controller->set_new_point(new_goal.x,new_goal.y,new_goal.theta);
}
void clear_goals(float data){
  Serial.println("Clear goals");
  goal_controller->clean_points();
}
void start_control(float data){
  Serial.println("Start controller");
  goal_controller->start_control();
}

void stop_control(float data){    
  goal_controller->stop_control();
  Serial.println("Stop controller");
}
void reset_controller(float data){  
  goal_controller->reset_car_position();
  Serial.println("Reset controller");
}




