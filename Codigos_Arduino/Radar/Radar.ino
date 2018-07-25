#include "radar.h"
//#include "serialParser.h"
#include "Wire.h"
#include "CComands.h"
  

#define NSENSORS 5
#define ADDRESS 8

redgrey_radar *radar;
//serialManager *manager_serial;

I2C_sensor_request sensor_measures;
I2C_communication *radar_com;

bool radar_mode=false; // false es manual
bool radar_new_mode=false; 
float new_angle;


void setup() {
    Serial.begin(250000);
  // put your setup code here, to run once:
  
Serial.println("Radar");
   radar = new redgrey_radar();  
   
  radar->configuration(60,90,60,MANUAL_RAD,300,-1);  
  radar->move_to(0);
  Serial.println("New I2C manager");
  radar_com =new I2C_communication(0,RADAR_ID,SLAVE_I2C);  
  radar_com->add_callback(manual_mode_setting,SET_TURRET_MANUAL_MODE); 
  radar_com->add_callback(get_distances,GET_RADAR_MEASURES); 
  radar_com->add_callback(automatic_mode_setting,SET_TURRET_AUTO_MODE);  
  
  Wire.onRequest(send_sensor_data); // register event
  Wire.onReceive(get_commands);  
  Serial.println("Ready i2c_com");
    
  /*Serial.println("New Serial manager");
  manager_serial = new serialManager();
  manager_serial->add_callback(manual,SET_TURRET_MANUAL_MODE);
  manager_serial->add_callback(automatic_mode,SET_TURRET_AUTO_MODE);  
  manager_serial->add_callback(get_distances,GET_RADAR_MEASURES); 
  Serial.println("Ready");*/
  
  
}
 
void loop() {
  // put your main code here, to run repeatedly:
 radar->update();
 get_measures();
 radar_com->update_I2C_sensordata(sensor_measures);

// manager_serial->update();
// get_distances();  
 if (radar_mode==false && radar_new_mode==true){
  automatic_mode(new_angle);
  radar_mode=true;
 }else if(radar_mode==true && radar_new_mode==false){
  radar_mode=false;
 }
}

void get_measures(){ 
  sensor_measures.data_formatted=radar->get_sensor_data();
}
void automatic_mode(float numero){
  Serial.println(numero);
  radar->configuration(60,90,(int)numero,AUTOMATIC_RAD,300,-1);    
}
void automatic_mode_setting(float angulo){
  new_angle=angulo;
  radar_new_mode=true;
}

void manual_mode_setting(float angulo){
  new_angle=angulo; 
  radar_new_mode=false;
    manual(new_angle);
}
void manual (float numero){
    radar->move_to(numero);
    Serial.print("Moviendo torre a: ");
    Serial.println(numero);

}


void send_sensor_data(){
  radar_com->request_callback();
}
void get_commands(){
  radar_com->read_callback();
}

void get_distances(){
  Serial.println (radar->get_radar_angle());
  Serial.println("\t"); 
  for(int i=0;i<NSENSORS;i++){
    Serial.print(sensor_measures.data_formatted.distance[i]);
    Serial.print("\t");
  }
  Serial.println("\t");
}

