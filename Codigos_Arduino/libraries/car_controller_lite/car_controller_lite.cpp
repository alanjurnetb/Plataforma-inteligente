#include "Math.h"
#include "car_controller_lite.h"
#include "Arduino.h"

#define PI 3.1415926535

void control_system::clean_points(){
	for (int i=0;i<GOAL_POINTS;i++){
		goal_coordinates[i].x=0;
		goal_coordinates[i].y=0;
		goal_coordinates[i].theta=0;
	}
	push_point_index=0;
	pull_point_index=0;	
	goals_left=0;
	control_active=false;
}

void control_system::set_new_point(coordinates newPosition){
	goal_coordinates[push_point_index]=newPosition;
	if (push_point_index>=GOAL_POINTS-1){
		return;
	}else{
		push_point_index++;		
		goals_left++;
	}
}

void control_system::set_new_point(float x, float y, float theta){
	coordinates temp;
	temp.x=x;
	temp.y=y;
	temp.theta=theta;
	goal_coordinates[push_point_index]=temp;
	Serial.println("Nuevo set point agregado");
	Serial.print("goal_total_steps: ");
	Serial.println(push_point_index);
	Serial.print("X: ");
	Serial.println(goal_coordinates[push_point_index].x);
	Serial.print("Y: ");
	Serial.println(goal_coordinates[push_point_index].y);
	Serial.print("THETA: ");
	Serial.println(goal_coordinates[push_point_index].theta);

	
	if (push_point_index>=GOAL_POINTS-1){
		Serial.println("Nect will be added on top of last.");
		return;
	}else{
		push_point_index++;		
	}
	
}

coordinates control_system::get_next_point(){
	coordinates to_return=goal_coordinates[pull_point_index];
	coordinates buffer;
	for (int i=0;i<=push_point_index-1;i++){
		goal_coordinates[i]=goal_coordinates[i+1];
	}
	push_point_index=push_point_index-1;
	return(to_return);

}

bool control_system::is_active(){
	return (control_active);
}

void control_system::start_control(){
	control_active=true;
	next_goal=get_next_point();
	Serial.println("Control active.");
}

void control_system::stop_control(){
	new_commands.speed=0;
	new_commands.steering=0;
	control_active=false;	
	control_system::clean_points();
}

control_system::control_system(){
	mode=POINT;	
	control_active=false;
	push_point_index=0;
	pull_point_index=0;
	goals_left=0;
	position_offset.x=0;
	position_offset.y=0;
	position_offset.theta=0;
	real_time_position.x=0;
	real_time_position.y=0;
	real_time_position.theta=0;
	for (int i=0;i<VFH_SIZE;i++){
		vf_histogram.histogram[i]=SENSOR_RANGE;
		old_vf_histogram.histogram[i]=SENSOR_RANGE;
	}
	new_commands.turret_angle=0;

}

commands control_system::get_commands(){
	return(new_commands);
}

void control_system::update_control(float shaftTurns, float theta){
	static float last_turns=0;

	if (last_turns!=shaftTurns){
		real_time_position.x=shaftTurns*cos(theta*PI/180)*WHEEL_DIAMETER*PI/1000;
		real_time_position.y=shaftTurns*sin(theta*PI/180)*WHEEL_DIAMETER*PI/1000;
		real_time_position.theta=theta*PI/180-position_offset.theta;
		last_turns=shaftTurns;
	}


	if (control_active==true){
		
		goal_distance=	sqrt(pow(next_goal.x-real_time_position.x,2)+pow(next_goal.y-real_time_position.y,2));
		if(goal_distance<GOAL_POSITION_ERROR){
			if (goals_left==0){
				stop_control();
				Serial.println("Control stopped");

				return;
			}else{
				next_goal=get_next_point();				
				Serial.println("Next Goal");
				Serial.println(goals_left);
			}	
			goals_left--;	
		}
		
		if (mode==POINT){
			
			new_commands.speed=SPEED_KP*sqrt(pow(next_goal.x-real_time_position.x,2)+pow(next_goal.y-real_time_position.y,2));	
			
			
			if (new_commands.speed>MAX_SPEED){
				new_commands.speed=MAX_SPEED;
			}
			double desired_steering=(atan2((next_goal.y-real_time_position.y),(next_goal.x-real_time_position.x)))-real_time_position.theta;
			Serial.println("desired_steering: ");
			Serial.println(desired_steering*180/PI);
			double vfh_steering =get_vhf_steering(desired_steering*180/PI)*PI/180;

/*			// COntrol propocional de giro.
			double error_steering=((vfh_steering-real_time_position.theta*3.1415/180)*TURN_FACTOR*180.0f/3.1415);
			static double sum_error=0;
			sum_error+=error_steering;
			Serial.println("error_steering: ");
			Serial.println(error_steering);

			double buffer=new_commands.steering+error_steering*STEERING_KP+sum_error*STEERING_KI;// considerar que el comando para el servo es negativo*/
			double buffer=vfh_steering*180/PI;
			//double buffer=(desired_steering*180/PI);			
			if (buffer>=MIN_STEERING_ANGLE &&buffer<=MAX_STEERING_ANGLE){
				new_commands.steering=buffer;
			}else{
				if (buffer<MIN_STEERING_ANGLE){
					new_commands.steering=MIN_STEERING_ANGLE;
				}else if(buffer<=MAX_STEERING_ANGLE){
					new_commands.steering=MAX_STEERING_ANGLE;
				}
			}

			

			new_commands.turret_angle=0;//((int)(new_commands.steering)/VFH_WINDOW)*VFH_WINDOW;

			Serial.print("Goal x: ");
			Serial.print(next_goal.x);
			Serial.print("Goal y: ");
			Serial.print(next_goal.y);
			Serial.print("\tX: ");
			Serial.print(real_time_position.x);			
			Serial.print("\tY: ");
			Serial.print(real_time_position.y);			
			Serial.print("\tSpeed: ");
			Serial.print(new_commands.speed);
			Serial.print("\tD_steering: ");
			Serial.print(desired_steering*180/PI);
			Serial.print("\tF steering");
			Serial.print(vfh_steering*180/PI);
			Serial.print("\tSteering: ");
        	Serial.println(new_commands.steering); 
		}
	}else{
		new_commands.speed=0;
		new_commands.steering=0;
		new_commands.turret_angle=0;
	}
}


void control_system::reset_car_position(){
	real_time_position.x-=real_time_position.x;
	real_time_position.y-=real_time_position.y;
	real_time_position.theta-=real_time_position.theta;
}

int control_system::get_vfh_min_angle(){
	int max=0;
	int index;
	for(int i=0;i<NSENSORS;i++){	
		if (distance[i]>max){
			max=distance[i];
			index=i;
		}
	}
	return(turret_angle+(index-2)*SENSOR_ANGLE_SEPEARATION);
}

void control_system::update_avoider(float sensor_angle, float distance0,float distance1,float distance2,float distance3,float distance4){
		distance[4]=distance0;
		distance[3]=distance1;
		distance[2]=distance2;
		distance[1]=distance3;
		distance[0]=distance4;

		turret_angle=((int)(sensor_angle*180.0/PI+new_commands.turret_angle)/2/VFH_WINDOW)*VFH_WINDOW;
		turret_angle=0;
		Serial.println("turret_angle");
		Serial.println(turret_angle);
		force_field_vector.x=0;
		force_field_vector.y=0;
		for (int i=0;i<VFH_SIZE;i++){
			vf_histogram.angles[i]=180-i*VFH_WINDOW;
		}

		for(int i=0;i<VFH_SIZE;i++){		
			for (int j=0;j<NSENSORS;j++){		
				if (vf_histogram.angles[i]==turret_angle+(j-2)*SENSOR_ANGLE_SEPEARATION){
					vf_histogram.histogram[i]=(SENSOR_RANGE-distance[j]);	
					//vf_histogram.histogram[i]=0.3*vf_histogram.histogram[i]+0.7*(SENSOR_RANGE-distance[j]);	
									
				}		
			}
			if ((vf_histogram.angles[i]<turret_angle+(-2)*SENSOR_ANGLE_SEPEARATION)||(vf_histogram.angles[i]>turret_angle+(2)*SENSOR_ANGLE_SEPEARATION)){
				vf_histogram.histogram[i]=SENSOR_RANGE;
			}
			Serial.print(vf_histogram.histogram[i]);
			Serial.print("\t");
		}
		Serial.println("");


		for(int i=0;i<VFH_SIZE;i++){		
			for (int j=1;j<NSENSORS;j++){
				if (vf_histogram.angles[i]==turret_angle+(j-2)*SENSOR_ANGLE_SEPEARATION){
					int counter=0;
					
					for (int k=i;k<i+SENSOR_ANGLE_SEPEARATION/VFH_WINDOW;k++){
						
						if(vf_histogram.histogram[i]>vf_histogram.histogram[i+SENSOR_ANGLE_SEPEARATION/VFH_WINDOW]){					
							vf_histogram.histogram[k]=((vf_histogram.histogram[i]-counter*(vf_histogram.histogram[i]-vf_histogram.histogram[i+SENSOR_ANGLE_SEPEARATION/VFH_WINDOW])*1.0/((SENSOR_ANGLE_SEPEARATION*1.0/VFH_WINDOW))));
							//vf_histogram.histogram[k]=0.1*vf_histogram.histogram[k]+0.9*((vf_histogram.histogram[i]-counter*(vf_histogram.histogram[i]-vf_histogram.histogram[i+SENSOR_ANGLE_SEPEARATION/VFH_WINDOW])*1.0/((SENSOR_ANGLE_SEPEARATION*1.0/VFH_WINDOW))));
						}else{
							vf_histogram.histogram[k]=((vf_histogram.histogram[i]+counter*(-vf_histogram.histogram[i]+vf_histogram.histogram[i+SENSOR_ANGLE_SEPEARATION/VFH_WINDOW])*1.0/((SENSOR_ANGLE_SEPEARATION*1.0/VFH_WINDOW))));
							//vf_histogram.histogram[k]=0.1*vf_histogram.histogram[k]+0.9*((vf_histogram.histogram[i]+counter*(-vf_histogram.histogram[i]+vf_histogram.histogram[i+SENSOR_ANGLE_SEPEARATION/VFH_WINDOW])*1.0/((SENSOR_ANGLE_SEPEARATION*1.0/VFH_WINDOW))));
						}
						counter++;
					}
				}
			}
			//Serial.print(vf_histogram.angles[i]);
			//Serial.print("\t");
		}
		//Serial.println("");
		//Serial.println("Update");

}


double control_system::get_vhf_steering(double goal_steering_angle){

	valley valley_detected[VFH_SIZE];
	unsigned char index=0;
	valley nearer_valley;
	bool new_valley=false;
	for (int i=0;i<VFH_SIZE;i++){
		if (vf_histogram.histogram[i]<NEAR_THRESHOLD  && new_valley==false){
			valley_detected[index].max_angle=vf_histogram.angles[i];
			valley_detected[index].size=1;
			new_valley=true;

		}else if(new_valley && vf_histogram.histogram[i]<NEAR_THRESHOLD){
			valley_detected[index].min_angle=vf_histogram.angles[i];
			valley_detected[index].size++;


			if(i==VFH_SIZE-1){
				index++;
			}
		}else if(new_valley && vf_histogram.histogram[i]>=NEAR_THRESHOLD){
			if(valley_detected[index].size==1){
				valley_detected[index].min_angle=valley_detected[index].max_angle;
			}
			new_valley=false;
			//Serial.print("Min_angle: ");
			//Serial.println(valley_detected[index].min_angle);
			//Serial.print("Max_angle: ");
			//Serial.println(valley_detected[index].max_angle);
			index++;
		}
		if (vf_histogram.histogram[i]<NEAR_THRESHOLD){
			Serial.print(1);
		}else{
			Serial.print(0);
		}
		Serial.print("\t");
	}
	Serial.print("\n");

	for (int i=0;i<VFH_SIZE-1;i++){
		if(goal_steering_angle>=vf_histogram.angles[i]&&goal_steering_angle<=vf_histogram.angles[i+1]){
			if(vf_histogram.histogram[i]-SENSOR_RANGE>goal_distance){

				return(goal_steering_angle);			
			}
		}
		Serial.print(vf_histogram.angles[i]);
		Serial.print("\t");
	}
	Serial.print("\n");

	Serial.print("Zonas: ");
	Serial.println(index);

	// Busco el valley mas grande
	int biggest_valley=0;
	int valley_size_buffer=0;
	for (int i=0;i<index;i++){
		if (valley_detected[i].size>valley_size_buffer){
			valley_size_buffer=valley_detected[i].size;
			biggest_valley=i;
		}
	}
	Serial.print("biggest_valley: ");
	Serial.println(biggest_valley);

	// Pondero los valores del valle y elijo ese promedio
	int stated=0;
	double ponde=0;
	double sumweigth=0;
	for (int i=0;i<VFH_SIZE-1;i++){
		if (vf_histogram.angles[i]==valley_detected[biggest_valley].max_angle && stated==0){
			stated=1;
		
		}
		if(stated==1 && valley_detected[biggest_valley].min_angle <=vf_histogram.angles[i]) {
			ponde=ponde+vf_histogram.angles[i]*(SENSOR_RANGE-vf_histogram.histogram[i]);
			sumweigth=sumweigth+SENSOR_RANGE-vf_histogram.histogram[i];
		}else if(stated==1 &&valley_detected[biggest_valley].min_angle >vf_histogram.angles[i]){
			ponde=ponde/sumweigth;
			stated=0;
		}
	}
/*
	Serial.print("Ponde: ");
	Serial.println(ponde);
	if (valley_detected[biggest_valley].min_angle<=goal_steering_angle && valley_detected[biggest_valley].max_angle>=goal_steering_angle ){
		return(goal_steering_angle);
	}else{
		return(ponde);
	}*/


	for (int i=0;i<index;i++){
		Serial.println("Min angle:");
		Serial.println(valley_detected[i].min_angle);
		Serial.println("Max angle:");
		Serial.println(valley_detected[i].max_angle);
		Serial.println("Goal angle:");
		Serial.println(goal_steering_angle);
		if (valley_detected[i].min_angle<=goal_steering_angle && valley_detected[i].max_angle>=goal_steering_angle ){
			Serial.println("El goal esta dentro del valley");
			if((valley_detected[i].max_angle-valley_detected[i].min_angle)>=MAX_VALLEY_SIZE){
				return(goal_steering_angle);
			}else{
				return((valley_detected[i].min_angle+valley_detected[i].max_angle)/2);
			}
		}
	}
	float last_min=10000;

	for (int i=0;i<index;i++){
		float d1=sqrt(pow(valley_detected[i].min_angle-goal_steering_angle,2));
		float d2=sqrt(pow(valley_detected[i].max_angle-goal_steering_angle,2));
		float min;
		if (d1>d2){
			min=d2;
		}else{
			min=d1;
		}

		if (min < last_min && valley_detected[i].size>=1){
			nearer_valley=valley_detected[i];	
			last_min=min;
		}
	}
	Serial.print("Nearer valley: ");
	Serial.print("\tmin_angle: ");
	Serial.print(nearer_valley.min_angle);
	Serial.print("\tmax_angle: ");
	Serial.println(nearer_valley.max_angle);

	//Serial.println("Not even one valley");

	//if (index ==0){
		//return((valley_detected[index].min_angle+valley_detected[index].max_angle)/2);		
	//}else{
		return((nearer_valley.min_angle+nearer_valley.max_angle)/2);		
//	}
}