/*
 *	Sharp GP2Y0A02YK0F IR Distance Sensor 20-150 cm 
 *	(Parallax Part Number: #28997) 
 *	IR Distance Sensor Library
 */


#include <GP2Y0A02YK0F.h>

#define MAX_FILTER_SAMPLES 10
#define MAX_MEASURE_DISTANCE 300 // cm
// Constructor

GP2Y0A02YK0F::GP2Y0A02YK0F() {
//	Serial.println("New Sensor.");

}

// Default Begin method: sensorPin = A0.

void GP2Y0A02YK0F::begin() {
	begin (A0);
}

// Begin method - assign sensorPin as the analog sensor input
// When you use begin() without variables the default input A0 is assumed.

void GP2Y0A02YK0F::begin(int sensorPin) {
  	pinMode(sensorPin, INPUT);
	_sensorPin = sensorPin;
}

// getDistanceRaw() Method: Returns the distance as a raw value: ADC output: 0 -> 1023

int GP2Y0A02YK0F::getDistanceRaw() {
		return (analogRead(_sensorPin));
}

// getDistanceCentimeter() Method: Returns the distance in centimeters

double GP2Y0A02YK0F::getDistanceCentimeter() {
	double sensorValue = analogRead(_sensorPin);
  	//double cm = 10650.08 * pow(sensorValue,-0.935) - 10;
  	double cm = 65.6993 *pow(map(sensorValue, 0, 1023, 0, 5000)/1000.0, -0.813);
  	return (cm);
}


double GP2Y0A02YK0F::getDistanceCentimeterFiltered (int reading_samples) {
	double distance=0;
	double buffer=0;
	if (reading_samples<=0) reading_samples=1;
	

	for(int j=0;j<reading_samples;j++){
    buffer= getDistanceCentimeter();
      distance =distance +buffer;

     }     
   distance=distance/reading_samples;
   if (distance <= 0) distance=MAX_MEASURE_DISTANCE;
   if (distance >300) distance=MAX_MEASURE_DISTANCE;
   return (distance);
}