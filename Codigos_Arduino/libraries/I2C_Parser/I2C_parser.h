
#ifndef __I2Cparser_H_
#define __I2Cparser_H_

#include <Wire.h>
#include "CComands.h"
#include <Arduino.h>

#define MAX_SUPPORTED_I2C_COMMANDS 10
#define RADAR_ID 8
//#define PARAGOLPE_ID 9
#define CONTROLLER_ID 2

enum I2C_mode{MASTER_I2C, SLAVE_I2C};

////////// Radar

typedef struct I2C_command_sensor_receive
{
  unsigned char command;
  float data;
};

typedef union I2C_command_sensor_packet_receive{
  I2C_command_sensor_receive message;
  uint8_t I2C_command_sensor_bytes[sizeof(I2C_command_sensor_receive)];
};  

/*///////// Paragolpe

typedef struct I2C_command_paragolpe_receive
{
  unsigned char command;
  float data;
};

typedef union I2C_command_paragolpe_packet_receive{
  I2C_command_paragolpe_receive message;
  uint8_t I2C_command_paragolpe_bytes[sizeof(I2C_command_paragolpe_receive)];
};  */

///////// Controlador

typedef struct I2C_command_controller_receive{
  unsigned char command;
  int16_t distance[5]; // mm
  double sensor_angle;
  double speed;
  double heading;
  double shaft_turns;
  double steering;
};

typedef struct I2C_command_controller_command_receive{
  unsigned char command;
  float data;
};

typedef union I2C_command_controller_packet_receive{
  I2C_command_controller_receive message;
  I2C_command_controller_command_receive com_message;
  uint8_t I2C_command_controller_bytes[sizeof(I2C_command_controller_receive)];
};  

/////////// radar
typedef struct sensor_data{
  int16_t distance[5]; // mm
  double time_stamp; // Relative to the centero of the radar
  double sensor_angle;
};

typedef struct sensor_data_bytes{
  uint8_t perbyte[sizeof(sensor_data)];
};

typedef union I2C_sensor_request{
  sensor_data data_formatted;
  sensor_data_bytes data_splitted;
};

/*////////// paragolpe
typedef struct paragolpe_data{
  int16_t distance[3]; // mm
  bool bumper[2];
};

typedef struct paragolpe_data_bytes{
  uint8_t perbyte[sizeof(paragolpe_data)];
};

typedef union I2C_paragolpe_request{
  paragolpe_data data_formatted;
  paragolpe_data_bytes data_splitted;
};
*/

//////////controlador
typedef struct controller_data{
  float speed;
  float steering;
  int turret_angle;
};

typedef struct controller_data_bytes{
  uint8_t perbyte[sizeof(controller_data)];
};

typedef union I2C_controller_request{
  controller_data data_formatted;
  controller_data_bytes data_splitted;
};

typedef I2C_sensor_request I2C_sensor_receive;
//typedef I2C_paragolpe_request I2C_paragolpe_receive;
typedef I2C_controller_request I2C_controller_receive;


class I2C_communication{
private:

    I2C_sensor_receive incoming;  // es el que recibe el auto con los datos de los sensores
    I2C_command_sensor_packet_receive out_com;      // es el paquete que corresponde a los comandos que el auto le envia al radar
    I2C_sensor_request outgoing;  // es la que se envia del radar al auto con los datos de los sensores

    //I2C_paragolpe_receive incoming_paragolpe;  // es el que recibe el auto con los datos de los sensores
    //I2C_command_paragolpe_packet_receive out_com_paragolpe;      // es el paquete que corresponde a los comandos que el auto le envia al radar
    //I2C_paragolpe_request outgoing_paragolpe;  // es la que se envia del radar al auto con los datos de los sensores

    I2C_controller_receive incoming_controller;  // es el que recibe el auto con las ordenes del controlador
    I2C_command_controller_packet_receive out_car_data;      // es el paquete que corresponde a los datos que el auto le envia al controlador
    I2C_command_controller_packet_receive out_com_controller;      // es el paquete que corresponde a los datos que el auto le envia al controlador con los goals
    I2C_controller_request outgoing_controller;  // es el paquete que envia el controlador al auto con los datos de velocidad y giro


    long last_read;
    bool new_com = false;
    
    I2C_mode module_mode;
    unsigned char myID;
    unsigned char slaveID;
    
    void (*trigger_functions[MAX_SUPPORTED_I2C_COMMANDS])(float);
    void (*trigger_car_data_function)(I2C_command_controller_receive);
    unsigned char trigger_commands[MAX_SUPPORTED_I2C_COMMANDS];
    unsigned char commands_configured=0;

  public:
    I2C_communication(unsigned char masterID,unsigned char slave_ID,I2C_mode mode); //para el master se usa ID 0
    bool add_callback (void (*triggerFunction)(float), unsigned char triggerCommand);
    void add_car_data_callback (void (*triggerCarDataFunction)(I2C_command_controller_receive)); // el comando es "SEND_CAR_DATA"
    
    sensor_data I2C_communication::get_sensor_data(unsigned char ID);
    sensor_data I2C_communication::get_sensor_data();

    //paragolpe_data I2C_communication::get_paragolpe_data(unsigned char ID);
    //paragolpe_data I2C_communication::get_paragolpe_data();

    controller_data I2C_communication::get_controller_data(unsigned char ID);
    controller_data I2C_communication::get_controller_data();

    void I2C_communication::update_I2C_sensordata(I2C_sensor_request data);
    //void I2C_communication::update_I2C_paragolpedata(I2C_paragolpe_request data);
    void I2C_communication::update_I2C_controllerdata(I2C_controller_request data);

    void I2C_communication::read_callback(void);
    void I2C_communication::request_callback(void);
    void I2C_communication::update(void);
    void I2C_communication::send_message(unsigned char ID,unsigned char command, float data);
    void I2C_communication::send_car_data(unsigned char command, sensor_data sensorData,  double speed,  double heading, double shaft_turns, double steering );
};

#endif