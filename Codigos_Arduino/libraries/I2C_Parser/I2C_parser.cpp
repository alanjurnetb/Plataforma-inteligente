#include "I2C_parser.h"


bool I2C_communication::add_callback(void (*triggerFunction)(float), unsigned char triggerCommand){

  for (int i=0;i<commands_configured;i++){
    if (trigger_commands[i]==triggerCommand){
      return (false);
    }
  }
  if (commands_configured + 1 >=MAX_SUPPORTED_I2C_COMMANDS){
    return (false);
  }
  trigger_commands[commands_configured]=triggerCommand;
  trigger_functions[commands_configured]=triggerFunction;
  commands_configured++;  
  //Serial.print("Comanddo agregado: ");
  //Serial.println(triggerCommand);
  return(true);
}

void I2C_communication::add_car_data_callback (void (*triggerCarDataFunction)(I2C_command_controller_receive)){
  trigger_car_data_function=triggerCarDataFunction;
}

I2C_communication::I2C_communication(unsigned char masterID,unsigned char slave_ID,I2C_mode mode){
  //Serial.println("ENTREADO AL CONSTRUCTOR");
  module_mode=mode;
  

  if (module_mode==MASTER_I2C){
    myID=masterID;
    Wire.begin();
    slaveID=slave_ID;
  }else{
    myID=slave_ID;
    slaveID=slave_ID;
     Wire.begin(myID);
     masterID=slaveID;
     //Serial.print("Wire iniciado como slave con el id: ")   ;
     //Serial.println(myID);
  }
  Wire.setClock(400000);
}

sensor_data I2C_communication::get_sensor_data(){
  return(get_sensor_data(RADAR_ID));
}

/*paragolpe_data I2C_communication::get_paragolpe_data(){
  return(get_paragolpe_data(PARAGOLPE_ID));
}*/

sensor_data I2C_communication::get_sensor_data(unsigned char ID){
  Wire.requestFrom(ID,sizeof(I2C_sensor_request));
  for (int i=0;i<sizeof(I2C_sensor_request);i++){
      incoming.data_splitted.perbyte[i]=Wire.read();     
  }
  sensor_data retrieved_data=incoming.data_formatted;
  return(retrieved_data);
}
/*
paragolpe_data I2C_communication::get_paragolpe_data(unsigned char ID){
  Wire.requestFrom(ID,sizeof(I2C_paragolpe_request));
  for (int i=0;i<sizeof(I2C_paragolpe_request);i++){
      incoming_paragolpe.data_splitted.perbyte[i]=Wire.read();     
  }
  paragolpe_data retrieved_data=incoming_paragolpe.data_formatted;
  return(retrieved_data);
}*/

controller_data I2C_communication::get_controller_data(){
  return(get_controller_data(CONTROLLER_ID));
}

controller_data  I2C_communication::get_controller_data(unsigned char ID){
  Wire.requestFrom(ID,sizeof(I2C_controller_request));
  for (int i=0;i<sizeof(I2C_controller_request);i++){
      incoming_controller.data_splitted.perbyte[i]=Wire.read();     
  }
  controller_data retrieved_data=incoming_controller.data_formatted;
  return(retrieved_data);
}


/*
void I2C_communication::update_I2C_paragolpedata(I2C_paragolpe_request data){
  outgoing_paragolpe=data;  
}*/


void I2C_communication::update_I2C_sensordata(I2C_sensor_request data){
  outgoing=data;  
}

void I2C_communication::update_I2C_controllerdata(I2C_controller_request data){
  outgoing_controller=data;
}


void I2C_communication::read_callback(void){

  if (myID==RADAR_ID){
    for (int j=0;j<sizeof(I2C_command_sensor_packet_receive);j++){
      out_com.I2C_command_sensor_bytes[j] = Wire.read(); // receive byte as a character  
      Serial.println(out_com.I2C_command_sensor_bytes[j]);
    }
    while (0 < Wire.available()) {
       byte a=Wire.read(); // receive byte as a character  
    }
 
    for (int i=0;i<commands_configured;i++){
      if (trigger_commands[i]==out_com.message.command){           
        /*Serial.print("Indice: ");
        Serial.print(i);
        Serial.print(" Comando: ");
        Serial.print(out_com.message.command);
        Serial.print(" Data: ");
        Serial.println(out_com.message.data);*/
        trigger_functions[i](out_com.message.data);
      }
    }
  }else if(myID==CONTROLLER_ID){
    for (int j=0;j<sizeof(I2C_command_controller_packet_receive);j++){
      out_com_controller.I2C_command_controller_bytes[j] = Wire.read(); // receive byte as a character  
//      Serial.println(out_com_controller.I2C_command_controller_bytes[j]);
    }
    while (0 < Wire.available()) {
       byte a=Wire.read(); // receive byte as a character  
    }
    if(out_com_controller.com_message.command==SEND_CAR_DATA){
      trigger_car_data_function(out_com_controller.message);
    }else{
      for (int i=0;i<commands_configured;i++){
        if (trigger_commands[i]==out_com_controller.com_message.command){           
          /*Serial.print("Indice: ");
          Serial.print(i);
          Serial.print(" Comando: ");
          Serial.print(out_com_controller.com_message.command);
          Serial.print(" Data: ");
          Serial.println(out_com_controller.com_message.data);*/
          trigger_functions[i](out_com_controller.com_message.data);
        }
      }
    }

  }/*else if(myID==PARAGOLPE_ID){
    for (int j=0;j<sizeof(I2C_command_paragolpe_packet_receive);j++){
      out_com_paragolpe.I2C_command_paragolpe_bytes[j] = Wire.read(); // receive byte as a character  
      Serial.println(out_com_paragolpe.I2C_command_paragolpe_bytes[j]);
    }
    while (0 < Wire.available()) {
       byte a=Wire.read(); // receive byte as a character  
    }
 
    for (int i=0;i<commands_configured;i++){
      if (trigger_commands[i]==out_com_paragolpe.message.command){           
        /*Serial.print("Indice: ");
        Serial.print(i);
        Serial.print(" Comando: ");
        Serial.print(out_com.message.command);
        Serial.print(" Data: ");
        Serial.println(out_com.message.data);
        trigger_functions[i](out_com_paragolpe.message.data);
      }
    }
  }*/
  new_com=true;

}

void I2C_communication::request_callback(void){
  if (myID==RADAR_ID){
    Wire.write(outgoing.data_splitted.perbyte,sizeof(I2C_sensor_request));
  }else if(myID==CONTROLLER_ID){
    Wire.write(outgoing_controller.data_splitted.perbyte,sizeof(I2C_controller_request));
  }/*else if(myID==PARAGOLPE_ID){
    Wire.write(outgoing_paragolpe.data_splitted.perbyte,sizeof(I2C_paragolpe_request));
  }*/
}


void I2C_communication::send_message(unsigned char ID, unsigned char command, float data){
  
  if (module_mode == SLAVE_I2C){
    return;
  }
  if (ID == RADAR_ID){
    out_com.message.command = command;
    out_com.message.data = data;

    Wire.beginTransmission(RADAR_ID); // transmit to device #8
    Wire.write(out_com.I2C_command_sensor_bytes,sizeof(I2C_command_sensor_receive));        // sends five bytes  
    Wire.endTransmission();    // stop transmitting

    
  }else if (ID == CONTROLLER_ID){
    Serial.println("Sending command to CONTROLLER");
    out_com_controller.com_message.command = command;
    Serial.println(out_com_controller.com_message.command);
    out_com_controller.com_message.data = data;
    Serial.println(out_com_controller.com_message.data);

    Wire.beginTransmission(CONTROLLER_ID); // transmit to device #2
    Wire.write(out_com_controller.I2C_command_controller_bytes,sizeof(I2C_command_controller_packet_receive));        // sends five bytes  
    Wire.endTransmission();    // stop transmitting

  }/*else if (ID == PARAGOLPE_ID){
    out_com_paragolpe.message.command = command;
    out_com_paragolpe.message.data = data;

    Wire.beginTransmission(PARAGOLPE_ID); // transmit to device #8
    Wire.write(out_com_paragolpe.I2C_command_paragolpe_bytes,sizeof(I2C_command_paragolpe_receive));        // sends five bytes  
    Wire.endTransmission();    // stop transmitting

  }*/
  //delay(100);
}

void I2C_communication::send_car_data(unsigned char command, sensor_data sensorData,  double speed,  double heading, double shaft_turns, double steering ){
  if (module_mode == SLAVE_I2C){
    return;
  }


  out_car_data.message.command = command;
  for (int i=0;i<5;i++){
    out_car_data.message.distance[i] = sensorData.distance[i];  
  }
  out_car_data.message.sensor_angle = sensorData.sensor_angle;
  out_car_data.message.speed =  speed;
  out_car_data.message.heading = heading;
  out_car_data.message.shaft_turns = shaft_turns;
  out_car_data.message.steering = steering;

  
  //Serial.print("Enviando: ");
  //Serial.print(out_com.I2C_packet_bytes[0]);
  //Serial.print(" -- ");

  Wire.beginTransmission(slaveID); // transmit to device #8
  Wire.write(out_car_data.I2C_command_controller_bytes,sizeof(I2C_command_controller_receive));        // sends five bytes  
  Wire.endTransmission();    // stop transmitting

  //Serial.print(out_com.message.command);
  //Serial.print(" data: ");
  //Serial.println(out_com.message.data);

  //Serial.print("mensaje Enviando parseado: ");
  //for (int j=0;j<sizeof(I2C_payload_receive);j++){
      //Serial.print(out_com.I2C_packet_bytes[j]);
      //Serial.print(" - ");
  //  }
//Serial.println("");
  //delay(100);
}

