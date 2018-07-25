#include "rf_parser.h"



bool rf_communication::add_callback(void (*triggerFunction)(float), unsigned char triggerCommand){

  for (int i=0;i<commands_configured;i++){
    if (trigger_commands[i]==triggerCommand){
      return (false);
    }
  }
  if (commands_configured + 1 >=MAX_SUPPORTED_RF_COMMANDS){
    return (false);
  }
  trigger_commands[commands_configured]=triggerCommand;
  trigger_functions[commands_configured]=triggerFunction;
  commands_configured++;  
  return(true);
}

bool rf_communication::add_car_data_callback(void (*triggerFunctionLong)(car_data)){
  trigger_function_long=triggerFunctionLong;
}


rf_communication::rf_communication(unsigned char ID,rf_mode mode, unsigned char CE, unsigned char CSN){
    //Serial.println("ENTREADO AL CONSTRUCTOR");
  module_mode=mode;
  myID=ID;
  radio_module= new RF24(CE,CSN);
 // Serial.println("Retries");
 // radio_module->setRetries(1,1);
 // Serial.println("Dynamic payloads");
  //radio_module->enableDynamicPayloads();
 // Serial.println("Auto Ack");
 // radio_module->setAutoAck(false);
  //Serial.println("PA Level");
 // radio_module->setPALevel(RF24_PA_HIGH);


  radio_module->begin();
  if(radio_module->setDataRate(RF24_2MBPS)){
  //  Serial.println("RF DATA RATE CHANGED TO 2MBPS");
  }else{
  //  Serial.println("RF DATA RATE UNABLE TO CHANGE");
  }

}

void rf_communication::start(){

  if (module_mode == SLAVE){
    //Serial.println("Sarting RF as SLAVE");
    //Serial.println("openWritingPipe"); 
    delay(100);
    radio_module->openWritingPipe(pipes[myID]);
    //Serial.println("openReadingPipe");
    delay(100); 
    radio_module->openReadingPipe(1, pipes[0]);
    //Serial.println("startListening"); 
    delay(100);
    radio_module->startListening();
    //Serial.println("Listening");   
  }else{
    //Serial.println("Sarting RF as MASTER");
    //Serial.println("openWritingPipe"); 
    delay(100);
    radio_module->openWritingPipe(pipes[0]);
    //Serial.println("openReadingPipe");
    delay(100); 
 //   for (int i=1;i<6;i++){
      radio_module->openReadingPipe(1, pipes[myID]);  
 //   }  
    //Serial.println("startListening"); 
    delay(100);
    radio_module->startListening();
    //Serial.println("Listening");   
  }
  

}

void rf_communication::update(void){
  if(radio_module->available()) {
    radio_module->read(&incoming, sizeof(RF_packet));
    //Serial.print("Nuevo mensaje: ");
    if (incoming.short_message.data_size<=sizeof(command_payload)+1){
      //Serial.println(incoming.short_message.command);    
      for (int i=0;i<commands_configured;i++){
            if (trigger_commands[i]==incoming.short_message.command){           
    /*          Serial.print("Indice: ");
              Serial.print(i);
              Serial.print(" Comando: ");
              Serial.print(incoming.short_message.command);
              Serial.print(" Data: ");
              Serial.println(incoming.short_message.data);
              */
              trigger_functions[i](incoming.short_message.data);
            }
          }
      new_com=true;
    }else{
      trigger_function_long(incoming.long_message);
    }
  }
}

bool rf_communication::get_connection(void){
  if (new_com){
    new_com=false;
    return true;
  }else{
    return false;
  }
}

void rf_communication::send_message(unsigned char command, float data){
  radio_module->stopListening();
  if (module_mode == SLAVE){
    radio_module->openWritingPipe(pipes[0]);
  }else{
    radio_module->openWritingPipe(pipes[myID]);
  }
  delay(10);
  //Serial.print("Enviando: ");
  outgoing.short_message.command = command;
  outgoing.short_message.data = data;
  outgoing.short_message.data_size = sizeof(command_payload);
  bool ok=radio_module->write(&outgoing,sizeof(command_payload)+1);
  if (ok){
    //Serial.println("Se enviaron los datos");
  }else{
    //Serial.println("No se enviaron los datos");
  }
  //Serial.print(outgoing.short_message.command);
  //Serial.print(" data: ");
  //Serial.print(outgoing.short_message.data);
  delay(10);
  radio_module->startListening();
}

void rf_communication::send_car_data(sensor_data sensorData,float xpos, float ypos, float theta){
  radio_module->stopListening();
  if (module_mode == SLAVE){
    radio_module->openWritingPipe(pipes[0]);
  }else{
    radio_module->openWritingPipe(pipes[myID]);
  }
  delay(10);
  //Serial.print("Enviando: ");


  for (int i=0;i<5;i++){
     outgoing.long_message.distance[i]=sensorData.distance[i];
  }
  outgoing.long_message.sensor_angle =sensorData.sensor_angle ;
  outgoing.long_message.time_stamp =sensorData.time_stamp;
  outgoing.long_message.x_position = xpos;
  outgoing.long_message.y_position =ypos;
  outgoing.long_message.theta_heading =theta;
  outgoing.long_message.data_size = sizeof(RF_packet)+1;
  bool ok=radio_module->writeFast(&outgoing,sizeof(RF_packet)+1,false);
 // bool ok=radio_module->write(&outgoing,sizeof(RF_packet)+1);
  if (ok){
    //Serial.println("Se enviaron los datos");
  }else{
    //Serial.println("No se enviaron los datos");
  }
  delay(10);
  radio_module->startListening();
}