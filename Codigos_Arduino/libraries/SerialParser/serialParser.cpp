#include "serialParser.h"

serialManager::serialManager(){
	//Serial.println("Serial Manager active.");
};


bool serialManager::add_callback(void (*triggerFunction)(float), unsigned char triggerCommand){

	for (int i=0;i<commands_configured;i++){
		if (trigger_commands[i]==triggerCommand){
			return (false);
		}
	}
	if (commands_configured + 1 >=MAX_SUPPORTED_COMMANDS){
		return (false);
	}
	trigger_commands[commands_configured]=triggerCommand;
	trigger_functions[commands_configured]=triggerFunction;
	commands_configured++;	
	return(true);
}


void serialManager::update(){

	char incomingByte;
	float incomingData;
	if (Serial.available() > 0) {
	// read the incoming byte:
		incomingByte = Serial.read();
		for (int i=0;i<commands_configured;i++){
			if (trigger_commands[i]==incomingByte){
				//if(Serial.peek()=='-'){
//					incomingData=-Serial.parseFloat();
				//}else{
					incomingData=Serial.parseFloat();
				//}
				//Serial.print("Indice: ");
				//Serial.print(i);
				
				//Serial.print(" Comando: ");
				//Serial.print(incomingByte);
				//Serial.print(" Data: ");
				//Serial.println(incomingData);
				
				trigger_functions[i](incomingData);
			}
		}
	}


}

void sendCommand(unsigned char command, float data){
	Serial.print(command);
	Serial.println(data,3);
}