
#ifndef __RFparser_H__
#define __RFparser_H_

#include <SPI.h>

#include "nRF24L01.h"
#include "RF24.h"
#include "CComands.h"
#include "I2C_parser.h"


#define MAX_SUPPORTED_RF_COMMANDS 25
#define RF_CE 7
#define RF_CSN 8

enum rf_mode{MASTER, SLAVE};

typedef struct command_payload
{
  unsigned char data_size;
  unsigned char command;
  float data;
};


typedef struct car_data{
  unsigned char data_size;
  int16_t distance[5]; //mm
  double sensor_angle; // 90 grados es en el centro del auto
  long time_stamp;
  float x_position;
  float y_position;  
  float theta_heading;
};

typedef union RF_packet{
  command_payload short_message;
  car_data long_message;
};


class rf_communication{
private:

    RF24 *radio_module;

    long pipes[7] = {0xF0F0F0F000,0xF0F0F0F001,0xF0F0F0F002,0xF0F0F0F003,0xF0F0F0F004,0xF0F0F0F005};

    RF_packet incoming;
    RF_packet outgoing;

    bool new_com = false;
    
    rf_mode module_mode;
    unsigned char myID;
    
    void (*trigger_functions[MAX_SUPPORTED_RF_COMMANDS])(float);
    void (*trigger_function_long)(car_data);
    unsigned char trigger_commands[MAX_SUPPORTED_RF_COMMANDS];
    unsigned char commands_configured=0;

  public:
    rf_communication(unsigned char ID,rf_mode mode, unsigned char CE, unsigned char CSN); //para el master se usa ID 0
    bool add_callback (void (*triggerFunction)(float), unsigned char triggerCommand);
    bool add_car_data_callback (void (*triggerFunctionLong)(car_data));
    void rf_communication::start();
    void rf_communication::update(void);
    bool rf_communication::get_connection(void);
    void rf_communication::send_message(unsigned char command, float data);
    void rf_communication::send_car_data(sensor_data sensorData,float axisTurns, float motorSpeed, float anguloDireccion);

};

#endif