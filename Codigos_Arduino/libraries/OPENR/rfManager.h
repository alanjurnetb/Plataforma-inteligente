#ifndef rfManager_h
#define rfManager_h



#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define RF_CE 9
#define RF_CSN 10

typedef struct RF_settings{
  int mspeed;
  int angle;
};

typedef struct RF_status{
  int mspeed;
  int angle;
  float battery;
};

void startRF(void);
void updateRF(void);

RF_settings get_RF_settings(void);
void set_RF_status(RF_status);
bool get_RF_connection(void);



#endif