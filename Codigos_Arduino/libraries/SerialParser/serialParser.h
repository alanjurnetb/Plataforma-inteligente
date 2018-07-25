

#ifndef __Serial_parser_H__
#define __Serial_parser_H__

#include "Arduino.h"
#include "CComands.h"


#define MAX_SUPPORTED_COMMANDS 30
#define SEND_BUFFER_SIZE 10


typedef struct outgoingCommand{
	unsigned char instruction;
	float data;	
};

class serialManager{
	private:
	int baudrate;
	void (*trigger_functions[MAX_SUPPORTED_COMMANDS])(float);
	unsigned char trigger_commands[MAX_SUPPORTED_COMMANDS];
	unsigned char commands_configured=0;


	public:
	serialManager();	
	bool add_callback (void (*triggerFunction)(float), unsigned char triggerCommand);
	void update();
	void sendCommand(unsigned char command, float data);

};


#endif