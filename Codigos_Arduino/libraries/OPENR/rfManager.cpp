#include "rfManager.h"

typedef struct RF_payload_settings
{
  unsigned char command;
  unsigned char speed_command;
  int RF_new_speed;
  unsigned char direction_command;
  int RF_new_angle;
};

typedef union RF_settings_packet{
  RF_payload_settings settings;
  byte RF_packet_bytes[sizeof(RF_payload_settings)];
};

typedef struct RF_payload_status
{
  int real_speed;
  int real_angle;
  float battery_load;
};

typedef union RF_status_packet{
  RF_payload_status status;
  byte RF_packet_bytes[sizeof(RF_payload_status)];
};

RF24 radio(RF_CE,RF_CSN);


long pipes[7] = {0xF0F0F0F000,0xF0F0F0F001,0xF0F0F0F002,0xF0F0F0F003,0xF0F0F0F004,0xF0F0F0F005};
byte myID = 0x01; // Definition de l'id de l'Arduino 

RF_settings_packet incoming;
RF_status_packet outgoing;

RF_status_packet actual_status;
RF_settings_packet new_settings;

bool new_com = false;

enum {SET,GET,IDLE};

int RF_mstatus =IDLE;



void sendStatus(void);

void startRF(void){
  radio.begin();
  Serial.println("Sarting RF");
  Serial.println("openWritingPipe"); 
  delay(100);
  radio.openWritingPipe(pipes[1]);
  Serial.println("openReadingPipe");
  delay(100); 
  radio.openReadingPipe(1, pipes[0]);
  Serial.println("startListening"); 
  delay(100);
  radio.startListening();
  Serial.println("Listening"); 

}

void updateRF(void){
  if(radio.available()) {
    radio.read(&incoming, sizeof(RF_settings_packet));
    Serial.println("Command:");
    if (incoming.settings.command == 'G'&& RF_mstatus==IDLE){
      Serial.println("Get");
      RF_mstatus=GET;
    }else if (incoming.settings.command == 'S' && RF_mstatus==IDLE){
      RF_mstatus=SET;
      Serial.println("Set");
    }
    new_com=true;
  }
  if (RF_mstatus==GET){
    outgoing=actual_status;
    sendStatus();
    RF_mstatus=IDLE;
  }else if(RF_mstatus==SET){
    new_settings=incoming;
    RF_mstatus=IDLE;
  }
}

bool get_RF_connection(void){
  if (new_com){
    new_com=false;
    return true;
  }else{
    return false;
  }
}

RF_settings get_RF_settings(void){
  RF_settings get;
  get.mspeed=new_settings.settings.RF_new_speed;
  unsigned char direction_command;
  int RF_new_angle;
  
  get.angle=new_settings.settings.RF_new_angle;
  return(get);
}

void set_RF_status(RF_status){
  RF_status set;
  actual_status.status.real_speed=set.mspeed;
  actual_status.status.real_angle=set.angle;
  actual_status.status.battery_load=set.battery;
  
}

void sendStatus(void){
  radio.stopListening();
  radio.openWritingPipe(pipes[1]);
  delay(10);
  radio.write(&outgoing,sizeof(RF_status_packet)+1);
  delay(10);
  radio.startListening();
}