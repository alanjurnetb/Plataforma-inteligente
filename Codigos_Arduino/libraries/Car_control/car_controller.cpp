#include "Math.h"
#include "car_controller.h"
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
	reset_grid();
	vhf_index=0;
}

commands control_system::get_commands(){

	return(new_commands);
}

void control_system::update_control(float shaftTurns, float theta){
	static float last_turns=0;
	//Serial.print(theta);
	//Serial.print("\t");
	//Serial.println(real_time_position.x);
	if (last_turns!=shaftTurns){
		real_time_position.x=shaftTurns*cos(theta*PI/180)*WHEEL_DIAMETER*PI/1000;
		real_time_position.y=shaftTurns*sin(theta*PI/180)*WHEEL_DIAMETER*PI/1000;
		real_time_position.theta=theta*PI/180-position_offset.theta;
		last_turns=shaftTurns;
	}


	if (control_active==true){
	
			
		if(sqrt(pow(next_goal.x-real_time_position.x,2)+pow(next_goal.y-real_time_position.y,2))<GOAL_POSITION_ERROR){
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
			if (!danger){
				new_commands.speed=SPEED_KP*sqrt(pow(next_goal.x-real_time_position.x,2)+pow(next_goal.y-real_time_position.y,2));	
			}else{
				if (front_distance<300){
					new_commands.speed=0;
				}
			}
			
			
			if (new_commands.speed>MAX_SPEED){
				new_commands.speed=MAX_SPEED;
			}
			double desired_steering=(atan2((next_goal.y-real_time_position.y),(next_goal.x-real_time_position.x)));
			double vfh_steering = get_vhf_steering(desired_steering*180/PI)*PI/180;




			new_commands.steering=((vfh_steering-real_time_position.theta*3.1415/180)*TURN_FACTOR*180.0f/3.1415)*STEERING_KP;// considerar que el comando para el servo es negativo
			if ((new_commands.steering/TURN_FACTOR)<MIN_STEERING_ANGLE){
				new_commands.steering=MIN_STEERING_ANGLE;
			}else if((new_commands.steering/TURN_FACTOR)>MAX_STEERING_ANGLE){
				new_commands.steering=MAX_STEERING_ANGLE;
			}
			
			/*Serial.print("Goal x: ");
			Serial.print(next_goal.x);
			Serial.print("Goal y: ");
			Serial.print(next_goal.y);
			Serial.print("\tX: ");
			Serial.print(real_time_position.x);			
			Serial.print("\tY: ");
			Serial.print(real_time_position.y);			
			Serial.print("\tSpeed: ");
			Serial.print(new_commands.speed);*/
			Serial.print("\tD_steering: ");
			Serial.print(desired_steering*180/PI);
			Serial.print("\tVFH steering");
			Serial.print(vfh_steering*180/PI);
			Serial.print("\tSteering: ");
        	Serial.println(new_commands.steering);  
		}
	}else{
		new_commands.speed=0;
		new_commands.steering=0;

	}
	
	
}

void control_system::reset_grid(){
  for (int i=0;i<W_GRID;i++){
    for (int j=0;j<H_GRID;j++){
      grid[i*W_GRID+j]=0;
    }
  }
}

void control_system::reset_car_position(){
	real_time_position.x-=real_time_position.x;
	real_time_position.y-=real_time_position.y;
	real_time_position.theta-=real_time_position.theta;
}

void control_system::update_avoider(float sensor_angle, float distance0,float distance1,float distance2,float distance3,float distance4){
	float distance[NSENSORS];
	distance[0]=distance0;
	distance[1]=distance1;
	distance[2]=distance2;
	distance[3]=distance3;
	distance[4]=distance4;
	
	front_distance	=distance[2];
	
	grid_coordinates cell;
	grid_coordinates center_cell;
	grid_coordinates last_cell;
	center_cell.x=CAR_CENTER_X_GRID;
	center_cell.y=CAR_CENTER_Y_GRID;
	bool new_cell=false;

	int m=0;
	//print_grid();
	int l =2;
	for (int l=0;l<NSENSORS;l++){

		for (int m=-SENSOR_SENS_HALF_ANGLE; m<SENSOR_SENS_HALF_ANGLE;m++){
			cell.x=(unsigned char)(CAR_CENTER_X_GRID+distance[l]/W_CELL_SIZE*cos(sensor_angle+(m+((-2+l)*SENSOR_ANGLE_SEPEARATION))*PI/180));
			cell.y=(unsigned char)(CAR_CENTER_Y_GRID-distance[l]/H_CELL_SIZE*sin(sensor_angle+(m+((-2+l)*SENSOR_ANGLE_SEPEARATION))*PI/180));				
			last_cell.x=(unsigned char)(CAR_CENTER_X_GRID+(SENSOR_RANGE+W_CELL_SIZE)/W_CELL_SIZE*cos(sensor_angle+(m+((-2+l)*SENSOR_ANGLE_SEPEARATION))*PI/180));
			last_cell.y=(unsigned char)(CAR_CENTER_Y_GRID-(SENSOR_RANGE+H_CELL_SIZE)/W_CELL_SIZE*sin(sensor_angle+(m+((-2+l)*SENSOR_ANGLE_SEPEARATION))*PI/180));
			Bresenham_plotLine(center_cell,cell,UPDATE_FREE_GRID);
			Bresenham_plotLine(cell,last_cell,UPDATE_UNKNOWN_GRID);
			fill_cell_probability(OCCUPIED,cell);
/*
			Serial.print("real_time_position: ");
			Serial.print(real_time_position.theta);
			Serial.print("Angulo: ");
			//Serial.print(sensor_angle+real_time_position.theta+(m+((-2+l)*SENSOR_ANGLE_SEPEARATION))*PI/180);
			Serial.print(sensor_angle);
			Serial.print("\tCell x mod:");
			Serial.print(distance[l]/W_CELL_SIZE*cos(sensor_angle+real_time_position.theta+(m+((-2+l)*SENSOR_ANGLE_SEPEARATION))*PI/180));
			Serial.print("\tCell y mod:");
			Serial.print(distance[l]/W_CELL_SIZE*sin(sensor_angle+real_time_position.theta+(m+((-2+l)*SENSOR_ANGLE_SEPEARATION))*PI/180));
			
			Serial.print("\tCell x:");
			Serial.print(cell.x);
			Serial.print("\tCell y:");
			Serial.println(cell.y);*/

		}
	}
	clear_histogram();
	update_histogram();
	print_histogram();
}

double control_system::get_vhf_steering(double goal_steering_angle){
	valley valley_detected[VHF_ZONES];
	unsigned char index=0;
	valley nearer_valley;
	bool new_valley=false;
	for (int i=0;i<VHF_ZONES;i++){
		if (vfh[i].weigth<NEAR_THRESHOLD  && new_valley==false){
			valley_detected[index].min_angle=vfh[i].angle;
			valley_detected[index].size=1;
			new_valley=true;
//			Serial.print("Min_angle: ");
//			Serial.println(valley_detected[index].min_angle);
			
		}else if(new_valley && vfh[i].weigth<NEAR_THRESHOLD){
			valley_detected[index].max_angle=vfh[i].angle;
			valley_detected[index].size++;

//			Serial.print("Max_angle: ");
//			Serial.println(valley_detected[index].max_angle);
			if(i==VHF_ZONES-1){
				index++;
			}
		}else if(new_valley && vfh[i].weigth>=NEAR_THRESHOLD){
			if(valley_detected[index].size==1){
				valley_detected[index].max_angle=valley_detected[index].min_angle;
			}
			new_valley=false;

			index++;
		}

		/*if (vfh[i].weigth<min.weigth){
			min.weigth=vfh[i].weigth;
			min.angle=vfh[i].angle;
		}*/
	}
	Serial.print("Zonas: ");
	Serial.println(index);
	for (int i=0;i<index;i++){
		if (valley_detected[i].min_angle>goal_steering_angle && valley_detected[i].max_angle<goal_steering_angle){
			return(((valley_detected[i].min_angle+valley_detected[i].max_angle)/2+4*goal_steering_angle)/5);
		}
	}
	float last_min=10000;

	for (int i=0;i<index;i++){
		float d1=sqrt(pow(valley_detected[i].min_angle-goal_steering_angle,2));
		float d2=sqrt(pow(valley_detected[i].max_angle-goal_steering_angle,2));
		float min;
		if (d1<d2){
			min=d1;
		}else{
			min=d2;
		}
		if (min < last_min){
			nearer_valley=valley_detected[i];	
			last_min=min;
		}
	}
	/*Serial.print("Nearer valley: ");
	Serial.print("\tmin_angle: ");
	Serial.print(nearer_valley.min_angle);
	Serial.print("\tmax_angle: ");
	Serial.println(nearer_valley.max_angle);*/
	if (index==0){
		danger=true;
	}else{
		danger=false;
	}
	return((nearer_valley.min_angle+nearer_valley.max_angle)/2);

}
void control_system::update_histogram(){
	grid_coordinates cell;

	grid_coordinates center_cell;
	center_cell.x=CAR_CENTER_X_GRID;
	center_cell.y=CAR_CENTER_Y_GRID;
	for(int i=-90;i<=90;i=i+DELTA_ANGLE_VFH){
		//Serial.print("Angle: ");
		
		//vfh[vhf_index].angle=i+(char)(DELTA_ANGLE_VFH*1.0/2);
		vfh[vhf_index].angle=i;
	//	Serial.println(vfh[vhf_index].angle,DEC);
		//for(int j=i;j<i+DELTA_ANGLE_VFH;j++){
			cell.x=(unsigned char)(CAR_CENTER_X_GRID+SENSOR_RANGE/W_CELL_SIZE*cos(vfh[vhf_index].angle*PI/180));
			cell.y=(unsigned char)(CAR_CENTER_Y_GRID+SENSOR_RANGE/H_CELL_SIZE*sin(vfh[vhf_index].angle*PI/180));				
			Bresenham_plotLine(center_cell,cell,UPDATE_VFH);
	//	}
		if (vhf_index<VHF_ZONES-1){
			vhf_index++;	
		}else{
			vhf_index=0;
		}
		
	}

}

void control_system::print_histogram(){
	for (int i=0;i<VHF_ZONES;i++){
		Serial.print(vfh[i].weigth);
		Serial.print("\t");
	}
	Serial.println("");
	for (int i=0;i<VHF_ZONES;i++){
		Serial.print(vfh[i].angle,DEC);
		Serial.print("\t");
	}
	Serial.println("");
}

void control_system::clear_histogram(){
	for (int i=0;i<VHF_ZONES;i++){
		vfh[i].weigth=0;
		vfh[i].angle=0;
	}
	
}

void control_system::update_delta_angle(grid_coordinates coord){
	//if (grid[coord.y*W_GRID+coord.x]>0){
		if (vfh[vhf_index].weigth+(grid[coord.y*W_GRID+coord.x]+127)<60000){
			vfh[vhf_index].weigth=vfh[vhf_index].weigth+(grid[coord.y*W_GRID+coord.x]+127);
		}else{
			vfh[vhf_index].weigth=60000;
		}
	//}
	
}

double control_system::fill_cell_probability(cellType cell_type, grid_coordinates coord){
	if (cell_type==OCCUPIED){
		if ((grid[coord.y*W_GRID+coord.x])+0.9*10<127){
			grid[coord.y*W_GRID+coord.x]=(grid[coord.y*W_GRID+coord.x])+0.9*10;
		}
	}else if (cell_type==FREE){
		if ((grid[coord.y*W_GRID+coord.x])-0.7*10>-127){
			grid[coord.y*W_GRID+coord.x]=(grid[coord.y*W_GRID+coord.x])-0.7*10;
		}
	}else if (cell_type==UNKNOWN){
		if (grid[coord.y*W_GRID+coord.x]>1){
			grid[coord.y*W_GRID+coord.x]=(grid[coord.y*W_GRID+coord.x])-0.1*10;	
		}else if(grid[coord.y*W_GRID+coord.x]<-1){
			grid[coord.y*W_GRID+coord.x]=(grid[coord.y*W_GRID+coord.x])+0.1*10;	
		}else{
			grid[coord.y*W_GRID+coord.x]=0;
		}
	}

}

// Bresenham line

void control_system::Bresenham_plotLineLow(grid_coordinates p0, grid_coordinates p1, line_task task){
	grid_coordinates cell;
	char dx = p1.x - p0.x;
	char dy = p1.y - p0.y;
	char yi = 1;
	char D,x,y;
	if (dy < 0){
		yi = -1;
		dy = -dy;
	}
  	D = 2*dy - dx;
  	y = p0.y;
	
	for (x=p0.x;x<p1.x;x++){
	    cell.x=x;
	    cell.y=y;
	    if(task==UPDATE_FREE_GRID){
	    	fill_cell_probability(FREE,cell);	
	    }else if (task==UPDATE_UNKNOWN_GRID){
	    	fill_cell_probability(UNKNOWN,cell);	
	    }else if(task==UPDATE_VFH){
	    	update_delta_angle(cell);
	    }
	    
	    if (D > 0){
	       y = y + yi;
	       D = D - 2*dx;
	    }
	    D = D + 2*dy;
	}
}


void control_system::Bresenham_plotLineHigh(grid_coordinates p0, grid_coordinates p1, line_task task){
	grid_coordinates cell;
  	char dx = p1.x - p0.x;
	char dy = p1.y - p0.y;
  	char xi = 1;
  	char D,x,y;
  	if (dx < 0){
	    xi = -1;
	    dx = -dx;
	}

  	D = 2*dx - dy;
  	x = p0.x;
  	for (y=p0.y;y<p1.y;y++){
    	cell.x=x;
	    cell.y=y;
	    if(task==UPDATE_FREE_GRID){
	    	fill_cell_probability(FREE,cell);	
	    }else if (task==UPDATE_UNKNOWN_GRID){
	    	fill_cell_probability(UNKNOWN,cell);	
	    }else if(task==UPDATE_VFH){
	    	update_delta_angle(cell);
	    }
	    
    	if (D > 0){
	       x = x + xi;
	       D = D - 2*dy;
	   	}    
 	   D = D + 2*dx;
	}
}

void control_system::Bresenham_plotLine(grid_coordinates p0, grid_coordinates p1, line_task task){
  if (abs(p1.y - p0.y) < abs(p1.x - p0.x)){
    if (p0.x > p1.x){
      Bresenham_plotLineLow(p1, p0,task);
    }else{
      Bresenham_plotLineLow(p0, p1,task);
    }
  }else{
    if (p0.y > p1.y){
      Bresenham_plotLineHigh(p1, p0,task);
    }else{
      Bresenham_plotLineHigh(p0, p1,task);
    }
  }
}

void control_system::print_grid(void){
	Serial.println("///////////////////////////////////////////////////////////////////////////////////////////////");
	for (int i=0;i<W_GRID;i++){
	    for (int j=0;j<H_GRID;j++){
	    	
	    	/*if (grid[i*W_GRID+j]>OCCUPIED_THRESHOLD){
	    		Serial.print(1);
	    	}else{
	    		
    			Serial.print(0);	
	    		
	    	}*/
	    	if (i==CAR_CENTER_Y_GRID&&j==CAR_CENTER_X_GRID){
    			Serial.print("X");	
    			Serial.print(grid[i*W_GRID+j],DEC);	
    		}else{
    			Serial.print(grid[i*W_GRID+j],DEC);	
    		}
	    	Serial.print("  ");
    	}
    	Serial.println("");
  	}  	
  	Serial.println("///////////////////////////////////////////////////////////////////////////////////////////////");
}