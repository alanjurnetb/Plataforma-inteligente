#include "radar.h"

//redgrey_radar::redgrey_radar(unsigned char * mapdir, unsigned int mapSize){
redgrey_radar::redgrey_radar(){
	//Serial.println("New radar.");
//	localMap=mapdir;
//	map_size=mapSize;
	//center_map_x=1.0*map_size/2;
	//center_map_y=1.0*map_size/2;
	n_sensors=NSENSORS;
	angle_sensors=SENSORANGLE;
	for (int i=0; i<n_sensors;i++){
		sensors[i]=new GP2Y0A02YK0F();	
	}		
	sensors[0]->begin(A0);
	sensors[1]->begin(A1);
	sensors[2]->begin(A2);
	sensors[3]->begin(A3);
	sensors[4]->begin(A6);

	turret= new servo_controller(3,A7);	
	turret->set_pid(0.01,0.01,0.01);

	t_status=UNKNOWN;

/*	//Serial.println("Clearing map");
	delay(1);
    //Serial.println("LocalMap");

	for (int i = 0; i < map_size; i++)
	{
		for (int j = 0; j < map_size; j++)
		{
			localMap[i*map_size+j]=0;
		}
	}*/
	//Serial.println("Constructor ended.");
	configuration(60,90,60,MANUAL_RAD,150,-1);
}

void redgrey_radar::configuration(double angleSensors, double centerAngle, double scannerAngle, scanner_mode scanMode, double maxDistance, int speed){
	 
	angle_sensors=angleSensors;
	center_angle=centerAngle;
	scan_angle=scannerAngle;
	scan_mode=scanMode;
	min_angle=(center_angle-scan_angle*1.0/2);
	max_angle=(center_angle+scan_angle*1.0/2);
	max_distance=maxDistance;

	turret->set_angle_limits(min_angle,center_angle,max_angle);

	//Serial.println("configuration complete.");
	//Serial.print ("min_angle:");
  	//Serial.print (min_angle);	
  	//Serial.print ("  max_angle:");
  	//Serial.print (max_angle);
  	//Serial.print ("  center_angle:");
  	//Serial.println (center_angle);
	turret->startSequence();
	Serial.print("Modo automatico activado, angulo de barrido: ");
}

void redgrey_radar::update(){
	if (scan_mode == AUTOMATIC_RAD){
		turret->disable_pid();								
		////Serial.println("Auto");
		if (t_status==UNKNOWN){
			turret->set_angle((double)min_angle);	
		//	//Serial.println("GOING_MIN");
			////Serial.println((double)min_angle);
			t_status=GOING_MIN;
		}else if (t_status==GOING_MIN && turret->arrived_to_setpoint()==true){
			turret->set_angle((double)max_angle);	
			t_status=GOING_MAX;
			////Serial.println("GOING_MAX");
		}else if (t_status==GOING_MAX && turret->arrived_to_setpoint()==true){
			turret->set_angle((double)min_angle);	
			t_status=GOING_MIN;
			////Serial.println("GOING_MIN");
		}	
	}else if(scan_mode ==CONTROLLED_RAD){	
		turret->enable_pid();								
		if (t_status==UNKNOWN){
			turret->set_angle((double)min_angle);	
		//	//Serial.println("GOING_MIN");
			////Serial.println((double)min_angle);
			t_status=GOING_MIN;
		}else if (t_status==GOING_MIN && turret->arrived_to_setpoint()==true){
			turret->set_angle((double)max_angle);	
			t_status=GOING_MAX;
			////Serial.println("GOING_MAX");
		}else if (t_status==GOING_MAX && turret->arrived_to_setpoint()==true){
			turret->set_angle((double)min_angle);	
			t_status=GOING_MIN;
			////Serial.println("GOING_MIN");
		}	
		
	}else if(scan_mode ==MANUAL_RAD){
		turret->disable_pid();
	}else{
		////Serial.println("Otro modo");
	}
	update_measures_measures();
	update_controller();
	
}


void redgrey_radar::move_to(double angle){
	set_angle((double)angle);
	scan_mode=MANUAL_RAD;
}

void redgrey_radar::change_mode(scanner_mode new_mode){
	scan_mode=new_mode;
}


void redgrey_radar::set_angle(double angle){	
	turret->set_angle(angle);
}

void redgrey_radar::update_measures_measures(){
	double turret_angle=turret->get_angle_rads();
	last_measure.time_stamp=millis();
	last_measure.sensor_angle=turret->get_angle_rads();	
//	Serial.println(last_measure.sensor_angle);
	for (int i=0;i<n_sensors;i++){
		//last_measures[i]=sensors[i]->getDistanceCentimeterFiltered(9,1);				
		last_measure.distance[i]=sensors[i]->getDistanceCentimeterFiltered(30)*10.0;		
		//Serial.print(last_measure.distance[i]);

   		//Serial.print("\t");
	}	
	//Serial.println("\t");
}

sensor_data redgrey_radar::get_sensor_data(){
	return(last_measure);
	//return(pop_measure());
}
/*
void redgrey_radar::push_measure(sensor_data measure){
	measurements[measurements_write_index]=measure;
	measurements_write_index++;
	if(measurements_write_index>=MEASUREMENT_BUFFER){
		measurements_write_index=0;
	}	
	measures_qty++;
}

sensor_data redgrey_radar::pop_measure(){
	sensor_data returned;
	returned = measurements[measurements_read_index];
	if (measures_qty>0){
		measurements_read_index++;
		if (measurements_read_index>=MEASUREMENT_BUFFER){
			measurements_read_index=0;
		}
	}
	measures_qty--;	
	return(returned);
}*/

 void redgrey_radar::update_controller(){
	turret->update();
 }


double redgrey_radar::get_radar_angle(){
	return(turret->get_angle_rads());
} 