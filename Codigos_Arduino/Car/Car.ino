#include <SPI.h>
#include <Wire.h>
#include <MemoryFree.h>
#include "rf_parser.h"
#include "servo_driver_lite.h"
#include "Brushless_driver.h"
#include "radar_definitions.h"
#include "I2C_parser.h"
#include "Math.h" 
#include "imu_driver_lite.h"


#define WHEEL_DIAMETER 80
#define AXIS_DISTANCE 0.34f
#define BATTERY_SENSOR_PIN A7
#define RF_TIMEOUT 1000 

servo_controller_lite* direccion;//(SERVO);
brushless_motor* motor;
mpu_9250_lite* imu_sensor;
rf_communication* rf24;//(1,SLAVE,9,10);
I2C_communication* radar_com;//(0,8,MASTER_I2C);
I2C_communication* control_com;//(0,8,MASTER_I2C);

typedef struct coordinates{
  float x;
  float y;
  float theta;
};

typedef struct frame{
  float x_pos;
  float y_pos;
  float theta;
  float velocity;
};

frame car; 
sensor_data sensor_measures;
controller_data control_orders;
float bateria;
bool activated_controller=false;

coordinates new_point;
bool brake=false;
imu_si_data imu_real_data;


void setup() {
  Serial.begin(250000);
  car.x_pos=0;
  car.y_pos=0;
  car.theta=0;
  

 
//  Serial.print("freeMemory()=");
//  Serial.println(freeMemory());

//  Serial.println("Configurando comunicacion RF"); 
  rf24= new rf_communication(1,SLAVE,9,10); 
//  Serial.print("freeMemory()=");
//  Serial.println(freeMemory());

  rf24->add_callback(set_motor_speed,SET_MOTOR_SPEED);
  rf24->add_callback(turn_steering,TURN_STEERING);
  rf24->add_callback(get_turns,GET_TURNS);
  rf24->add_callback(get_motor_speed,GET_MOTOR_SPEED);
  rf24->add_callback(get_battery_status,BATTERY_STATUS);
  rf24->add_callback(get_radar_measures,GET_RADAR_MEASURES);
  rf24->add_callback(set_turret_manual_mode,SET_TURRET_MANUAL_MODE);
  rf24->add_callback(set_turret_auto_mode,SET_TURRET_AUTO_MODE);
  rf24->add_callback(reset_controller,RESET_CONTROLLER);
  rf24->add_callback(set_new_x,SEND_CAR_NEW_X);
  rf24->add_callback(set_new_y,SEND_CAR_NEW_Y);
  rf24->add_callback(set_new_theta,SEND_CAR_NEW_THETA);
  rf24->add_callback(set_fil_x,SEND_CAR_X_FILTERED);
  rf24->add_callback(set_fil_y,SEND_CAR_Y_FILTERED);
  rf24->add_callback(set_fil_theta,SEND_CAR_THETA_FILTERED);
  rf24->add_callback(set_mode,CONTROL_MODE);
  rf24->add_callback(push_new_set_point,NEW_SET_POINT);
  
  rf24->add_callback(clear_goals,CLEAR_GOALS);
  rf24->add_callback(start_control,START_CONTROL);
  rf24->add_callback(stop_control,STOP_CONTROL);

  rf24->start();

//  Serial.println("Configurando direccion");
  direccion=new servo_controller_lite(SERVO);
//   Serial.print("freeMemory()=");
//    Serial.println(freeMemory());

  direccion->set_angle_limits(16,55,94);
  direccion->set_angle(0);
  
//  Serial.println("Configurando motor");
  motor=new brushless_motor();
//   Serial.print("freeMemory()=");
//    Serial.println(freeMemory());

  motor->setup(SPEEDSENSOR, []{motor->encoder_callback();}, RISING);
  motor->set_speed(0);

//  Serial.println("Configurando radar");
  radar_com= new I2C_communication(0,RADAR_ID,MASTER_I2C);
//   Serial.print("freeMemory()=");
//    Serial.println(freeMemory());

//  Serial.println("Configurando control");
  control_com= new I2C_communication(0,CONTROLLER_ID,MASTER_I2C);
//   Serial.print("freeMemory()=");
//    Serial.println(freeMemory());

 // Serial.println("Configurando imu");
  imu_sensor= new mpu_9250_lite();

  
 /* Serial.println("Configurando controlador");
  controller= new control_system(); 
   Serial.print("freeMemory()=");
    Serial.println(freeMemory());

  controller->clean_points();*/

  Serial.println("Listo.");
     Serial.print("freeMemory()=");
    Serial.println(freeMemory());

}


void loop() {
  static long last_update=0;
  // Leo el IMU
 // Serial.println("Leo IMU");
  imu_real_data=imu_sensor->get_imu_SI();
  
  //Actualizo posicion estimada del auto
//  Serial.println("Actualizo posicion");
  update_car_estimated_position();
  
  // Obtengo datos del radar  
 // Serial.println("Obtengo datos radar");
  

  // Obtengo datos del controlador  
  //Serial.println("Obtengo datos del controlador");
  long now=millis();
  if (now-last_update>10){
    sensor_measures=radar_com->get_sensor_data(); 
    delay(5);
    control_com->send_car_data(SEND_CAR_DATA,sensor_measures,motor->read_speed(),car.theta,motor->get_turns(),direccion->get_angle());
    delay(5);
    control_orders=control_com->get_controller_data(); 
    last_update=now;
  }
//  Serial.print(motor->read_speed());
//  Serial.print("\t");
//  Serial.print(control_orders.steering);
//  Serial.println("\t");
//  for (int k=0;k<5;k++){
//    Serial.print(sensor_measures.distance[k]);
//    Serial.print("\t");
//  }
//  Serial.print(sensor_measures.sensor_angle);
//  Serial.print("\t");
//  Serial.print(sensor_measures.time_stamp);
//  Serial.println("");
  
  //Serial.print("freeMemory()=");
  //Serial.println(freeMemory());

  //Actualizo el controlador
  coordinates carpos;
  carpos.x=(float)car.x_pos;
  carpos.y=(float)car.y_pos;
  carpos.theta=car.theta;
  //controller->update_control(carpos);
  //Serial.println(control_orders.turret_angle);
  if (activated_controller){
 //   commands control_commands=controller->get_commands();
    Serial.println(control_orders.speed);
    if( control_orders.speed>-500 && control_orders.speed< 500){
      motor->set_speed(control_orders.speed);
    }
    Serial.println(control_orders.steering);
    if(control_orders.steering>-50 && control_orders.steering< 50){
      direccion->set_angle(control_orders.steering);    
      set_turret_manual_mode(control_orders.turret_angle);
    }
  }

  // Verifico que no se este chocando?mm esto se borra
  //Serial.println("Verifico choque");
  /*if (sensor_measures.distance[2]<300 && brake==false){
    Serial.println("Freno");
    motor->brake();    
    brake=true;
  }else if(sensor_measures.distance[2]>400 && brake==true){
    Serial.println("Suelto freno");
    brake=false;
  }
*/
  // Actualizo el controlador del motor  
  //Serial.println("Actualizo controlador motor");
  motor->update_brushless();
  // Actualizo el manager de RF
  //Serial.println("Actualizo el manager de RF");

  rf24->update();  

  // Leo el estado de la bateria// Esto hay que modificarlo
  //Serial.println("Leo bateria");
  bateria=analogRead(BATTERY_SENSOR_PIN);
  bateria=bateria*10.0/1024;
}

void update_car_estimated_position(){
  static float last_turns=0;
  static long last_car_update=0;

  float turns=motor->get_turns();
  long now=millis();
  
  float deltaTime=(now-last_car_update)*1.0f/1000;
  car.velocity=motor->read_speed()*WHEEL_DIAMETER*0.10472/1000;
  if (activated_controller&&car.velocity>0){
    car.theta=car.theta+imu_real_data.gz*deltaTime;
  }

  if (last_turns!=turns){
    car.x_pos=turns*cos(car.theta*PI/180)*WHEEL_DIAMETER*PI;
    car.y_pos=turns*sin(car.theta*PI/180)*WHEEL_DIAMETER*PI;
    last_turns=turns;
  }

  last_car_update=millis();
  last_turns=turns;
}



////////////////////////////////////////////////////////// CALLBACKS DE RF ////////////////////////////////////////////////
void get_turns(float vueltas){
  double vueltas_eje=motor->get_turns();
  rf24->send_message(GET_TURNS,vueltas_eje);
}

void set_motor_speed(float speed){
  activated_controller=false;
  motor->set_speed(speed);
}

void get_battery_status(float data){
  rf24->send_message(BATTERY_STATUS,bateria);  
}

void get_motor_speed(float data){
  int  velocidad=motor->read_speed();
  rf24->send_message(GET_MOTOR_SPEED,velocidad);
}

void set_turret_manual_mode(float data){
  radar_com->send_message(RADAR_ID,SET_TURRET_MANUAL_MODE,data);
  Serial.print("Move radar to: ");
  Serial.println(data);
  
}

void set_turret_auto_mode(float data){
  radar_com->send_message(RADAR_ID,SET_TURRET_AUTO_MODE,data);
  Serial.print("Auto radar con agulo: ");
  Serial.println(data);
}


void turn_steering(float angle){
  activated_controller=false;
  direccion->set_angle(angle);
}
void set_new_x(float data){ 
  Serial.println("New X");
  control_com->send_message(CONTROLLER_ID,SEND_CAR_NEW_X,data);
}
void set_new_y(float data){
 control_com->send_message(CONTROLLER_ID,SEND_CAR_NEW_Y,data);
}
void set_new_theta(float data){
 control_com->send_message(CONTROLLER_ID,SEND_CAR_NEW_THETA,data);
}

void push_new_set_point(float data){
  control_com->send_message(CONTROLLER_ID,NEW_SET_POINT,0);
  //controller->set_new_point(new_point.x,new_point.y,new_point.theta  );
}

void reset_controller(float data){
  control_com->send_message(CONTROLLER_ID,RESET_CONTROLLER,0);
  motor->clear_turns();
  reset_car_data(1);
   activated_controller=false;
  Serial.println("Clear goals");
}
void clear_goals(float data){
  //controller->clean_points();
  control_com->send_message(CONTROLLER_ID,CLEAR_GOALS,0);
  
  Serial.println("Clear goals");
}
void start_control(float data){
  activated_controller=true;
//  controller->start_control();
  control_com->send_message(CONTROLLER_ID,START_CONTROL,0);
  Serial.println("Start controller");
}

void stop_control(float data){  
//  controller->stop_control();
control_com->send_message(CONTROLLER_ID,STOP_CONTROL,0);
Serial.println("Stop controller");
}



void set_fil_x(float data){
}
void set_fil_y(float data){
}
void set_fil_theta(float data){
}

void set_mode(float data){
}

void get_radar_measures(){
  rf24->send_car_data(sensor_measures,car.x_pos,car.y_pos,car.theta);
} 

void reset_car_data(float value){
  car.theta=0;
  car.x_pos=0;
  car.y_pos=0;
}


