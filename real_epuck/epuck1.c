/********************************************************************************/
/* File:     	epuck1.c                                             			*/
/* Version:  	1.0                                                     		*/
/* Date:     	06-Dec-18                                               		*/
/* Description:  Reynolds flocking with relative positions for real E-Pucks   	*/
/*                                                                       		*/
/* Author:      Group 10			  											*/
/* Last revision: 06-Dec-18 Hugo Grall Lucas   			  					 	*/
/********************************************************************************/

			/*Librairies*/
			
#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <stdio.h>
#include <ircom/ircom.h>
#include <btcom/btcom.h>
#include <math.h>

#include "stdio.h"
#include "string.h"

#include "./a_d/advance_ad_scan/e_prox.h"
#include "./a_d/advance_ad_scan/e_ad_conv.h"

			/*To Change */
#define ROBOT_ID = 0;
#define ROBOT_GROUP = 0;

			/*Constantes*/
#define NB_SENSORS    	8   	// Number of distance sensors
#define MIN_SENS      	350 	// Minimum sensibility value
#define MAX_SENS      	4096	// Maximum sensibility value
#define MAX_SPEED     	800 	// Maximum speed
#define FLOCK_SIZE     	4   	// Size of flock

#define AXLE_LENGTH      0.052   		// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS  0.00628   		// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS     0.0205    		// Wheel radius (meters)
#define DELTA_T   	   	 0.064   	 	// Timestep (seconds)

#define RULE1_THRESHOLD 	0.1  	   		// Threshold to activate aggregation rule. default 0.1
#define RULE2_THRESHOLD 	0.12  			// Threshold to activate dispersion rule. default 0.12
#define RULE1_WEIGHT 	0.5
#define RULE2_WEIGHT 	0.95
#define RULE3_WEIGHT 	0.1
float migr[2] = {1.,0.};

#define INTIALISATION_STEPS  		30000  		//30 secondes at the beginning to form flock and align with migration (before infinite loop)
#define REYN_MIGR_RATIO  			0.5 		// TO TUNE: ratio of weights between Reynolds and Migration urge  	 
#define BRAITENBERG_LOWER_THRESH 	150 		// below this value, no avoidance
#define BRAITENBERG_SPEED_BIAS		300

// Possible mode
#define MODE_BRAI	0
#define MODE_REYN	1
#define MODE_MIGR	2

//Active comunication via Bluetooth
#defien VERBOSE 1

// Macro
#define ABS(x) ((x>=0)?(x):-(x))

/*Try other*/
#define BETA_MIGRATION 1        				// how much of migr_diff to implement during each step 

int e_puck_matrix[16] =   {16,9,6,3, -1,-5,-6,-8,   -12,-8,-5,6, 2,4,5,9}; // for obstacle avoidance giving priority to a side
float relative_pos[FLOCK_SIZE][3];    // relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];    // Previous relative  X, Z, Theta values
float prev_global_pos[FLOCK_SIZE][2];    // Previous global  X, Z values

float my_position[3];			 // X, Z, Theta of the current robot
float prev_my_position[3]; 		 // X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];   	 // Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];    // Speeds calculated with Reynold's rules
char buffer[80];
 
// tracks which are my flockmates. Myself excluded.
// 1 : is my flockmate and 0 : isn't my flockmate
bool flockmates[FLOCK_SIZE] = {0};
float migr_diff = 0;

// Flags
bool flag_brai = false;
bool flag_reyn = false;
bool flag_migr = false;

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}
/*
 * Reset the robot's devices and get its ID
 */
static int reset()
{
	// Initialize system and sensors
	e_init_port();
	e_init_uart1();
	e_init_motors();
	e_init_ad_scan();
    e_led_clear();	
    e_start_agendas_processing();
	
	// Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR = 0;
		__asm__ volatile ("reset");
	}
	
	// Send message via Bluetooth
	if(VERBOSE){
	sprintf(buffer, "[Robot id: %d] [Robot Group: %d]<< Reset Start >>\r\n",ROBOT_ID, ROBOT_GROUP);
	e_send_uart1_char(buffer, strlen(buffer));
	}
	
	e_calibrate_ir(); 

    // initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();
	
    // rely on selector to define the role
    int selector = getselector();
	
	return(selector);
}

void set_mode(int mode){
	switch(mode){
		case: MODE_BRAI
			flag_brai = true;
			if(VERBOSE){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode Brai>>\r\n",ROBOT_ID, ROBOT_GROUP);
			e_send_uart1_char(buffer, strlen(buffer));
			}
		break;
		case: MODE_REYN
			flag_reyn = true;
			if(VERBOSE){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode Reyn>>\r\n",ROBOT_ID, ROBOT_GROUP);
			e_send_uart1_char(buffer, strlen(buffer));
			}
		break;
		case: MODE_MIGR
			flag_migr = true;
			if(VERBOSE){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode Migr>>\r\n",ROBOT_ID, ROBOT_GROUP);
			e_send_uart1_char(buffer, strlen(buffer));
			}
		break;
		case: MODE_ALL
			flag_brai = true;
			flag_reyn = true;
			flag_migr = true;
			if(VERBOSE){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode All>>\r\n",ROBOT_ID, ROBOT_GROUP);
			e_send_uart1_char(buffer, strlen(buffer));
			}
		break;
	}
}

void update_self_motion(int msl, int msr) {

    float theta = my_position[2];
 
    // Compute deltas of the robot
    float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    float du = (dr + dl)/2.0;
    float dtheta = (dr - dl)/AXLE_LENGTH;
    
    // Compute deltas in the environment
    float dx = du * cosf(theta+dtheta/2);
	float dy = du * sinf(theta+dtheta/2);
    
    // Update position
    my_position[0] += dx;
    my_position[1] += dy;
    my_position[2] += dtheta;
     	 
	// keep orientation between [0,2*pi]
	my_position[2] = (my_position[2]<0? my_position[2]+2*M_PI:my_position[2]);
	my_position[2] = (my_position[2]>2*M_PI? my_position[2]-2*M_PI:my_position[2]);
}

void normalize_speed(int *msl, int *msr) {
    if(ABS(*msl) > MAX_SPEED || ABS(*msr) > MAX_SPEED){
      float max = (ABS(*msl)>ABS(*msr)?ABS(*msl):ABS(*msr));
      *msl = *msl/max * MAX_SPEED;
      *msr = *msr/max * MAX_SPEED;
    }
}

void compute_wheel_speeds(int *msl, int *msr)
{
	float x = speed[ROBOT_ID][0];
	float z = speed[ROBOT_ID][1];
   	     
    float Ku = 0.02;   // Forward control coefficient
    float Kw = 0.08;  // Rotational control coefficient
    float range = sqrtf(x*x + z*z);      // Distance to the wanted position
    float bearing = atan2(z, x);      // Orientation of the wanted position
    // Compute forward control
    float u = Ku*range;//*cosf(bearing);
    // Compute rotational control
    float w = Kw*bearing;

    // Convert to wheel speeds!
    *msl += (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    *msr += (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    
    // Avoid saturation
    normalize_speed(msl, msr);
}



void global2rel(float* v_global, float* v_ref){
    v_ref[0] =   cos(my_position[2])*(v_global[0]) + sin(my_position[2])*(v_global[1]);
    v_ref[1] = - sin(my_position[2])*(v_global[0]) + cos(my_position[2])*(v_global[1]);
}

void rel2global(float* v_ref, float* v_global){
	v_global[0] =   cos(-my_position[2])*(v_ref[0]) + sin(-my_position[2])*(v_ref[1]) ;
	v_global[1] = - sin(-my_position[2])*(v_ref[0]) + cos(-my_position[2])*(v_ref[1]) ;
}



void migration_urge(void) {
    global2rel(migr,speed[ROBOT_ID]);  
}
void reynolds_rules() {
    int j, k;   					 // Loop counters
    float rel_avg_loc[2] = {0,0};    // Flock average positions
    float rel_avg_speed[2] = {0,0};  // Flock average speeds
    float cohesion[2] = {0,0};
    float dispersion[2] = {0,0};
    float consistency[2] = {0,0};
    
    int n_flockmates = 0;
    float dist = 0.0;
   	 
    //Added by Pauline
    speed[ROBOT_ID][0] = 0;
    speed[ROBOT_ID][1] = 0;
    
    // Compute averages over the whole flock
    for(k=0;k<FLOCK_SIZE;k++){
   	 if(k == ROBOT_ID){
   		 continue;
   	 }

   	 if(flockmates[k]){
   		 for (j=0;j<2;j++) {
   			 rel_avg_speed[j] += relative_speed[k][j];
   			 rel_avg_loc[j] += relative_pos[k][j];
   		 }
   		 n_flockmates++;
   	 }
    }
    
    if(n_flockmates>0){
   	 for(j=0;j<2;j++){
   		 rel_avg_speed[j] /= n_flockmates;
   		 rel_avg_loc[j] /= n_flockmates;
   	 }
    }
    else{ 
   	 return; // no flockmates, no reynolds to be done
    }
    
    dist = sqrt(rel_avg_loc[0]*rel_avg_loc[0] + rel_avg_loc[1]*rel_avg_loc[1]);
    if(dist > RULE1_THRESHOLD){
   	 for (j=0;j<2;j++){    
   		 cohesion[j] = rel_avg_loc[j];
   	 }
    }
    for(k=0;k<FLOCK_SIZE;k++){
   	 if(k == ROBOT_ID){
   		 continue;
   	 }

   	 dist = sqrt(relative_pos[k][0]*relative_pos[k][0] + relative_pos[k][1]*relative_pos[k][1]);

   	 if(dist < RULE2_THRESHOLD && flockmates[k]){
   		 for (j=0;j<2;j++) {
   			 dispersion[j] -= (relative_pos[k][j]);
   		 }
   	 }
    }    
 
    // Rule 3 - Consistency/Alignment: match the speeds of flockmates
    for (j=0;j<2;j++){
   	 consistency[j] = rel_avg_speed[j];
    }

    //aggregation of all behaviors with relative influence determined by weights
    for (j=0;j<2;j++){
   	 speed[ROBOT_ID][j] +=  cohesion[j]    * RULE1_WEIGHT;
   	 speed[ROBOT_ID][j] +=  dispersion[j]  * RULE2_WEIGHT;
   	 speed[ROBOT_ID][j] +=  consistency[j] * RULE3_WEIGHT;
    }
}

void epuck_send_message(void)  
{
	long int message;
	message = (ROBOT_ID + (ROBOT_GROUP<<3));
	
	ircomSend(message);	    
	while (ircomSendDone() == 0);
}
void epuck_receive_message(void){
	
	bool read_buffer = true;
         static int loop_time;
	while (read_buffer &&	loop_time< FLOCK_SIZE-1) // Read all the buffer
	{
	    IrcomMessage imsg;
	    ircomPopMessage(&imsg);
	    if (imsg.error == 0)
	    {
			// get the message
			long int message_value = imsg.value;
			// get the distance
			float range = imsg.distance;
			// get the orientation
			float theta = ismg.direction;
			// get the sensor from witch we get the message
			int id_sensor = ismg.receivingSensor;
			// get the robot emitter id
			int other_robot_id = 0x0007 & temp;
			int other_robot_group = temp>>3;
			// check if message is my own (caused by reflection) and if it belongs to my group
			if((other_robot_id != ROBOT_ID) && (other_robot_group == ROBOT_GROUP)){
				
			global2rel(prev_global_pos[other_robot_id], prev_relative_pos[other_robot_id]);
				 
			// Set current relative location
			relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
			relative_pos[other_robot_id][1] = range*sin(theta);   // relative y pos
			relative_pos[other_robot_id][2] = theta;
									  
			// Compute speed in relative coordinates
			relative_speed[other_robot_id][0] = (relative_pos[other_robot_id][0] - prev_relative_pos[other_robot_id][0])/DELTA_T;
			relative_speed[other_robot_id][1] = (relative_pos[other_robot_id][1] - prev_relative_pos[other_robot_id][1])/DELTA_T;

			// Set relative_pos in global coordinates of this time step (x and y only)
			/* prev_global_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
			prev_global_pos[other_robot_id][1] = relative_pos[other_robot_id][1];*/

			rel2global(relative_pos[other_robot_id],  prev_global_pos[other_robot_id]);

			flockmates[other_robot_id] = 1;
	    }
	    else if (imsg.error > 0)
	    {
		// error in transmission	
	    }
		else
		{
			// else imsg.error == -1 -> no message available in the queue
			read_buffer = false;
		}
	}
	loop_time++;
}

/*Added by Pauline*/
float set_final_speed(int b_speed, int r_speed, int m_speed, int max_sens) {  //weights for each of the speed (wb: Braitenberg, wm: Migration, wr: Reynolds)
    static float wb, wm, wr;
    static float final_speed;
    static bool f_computeNot = 1;
 
    if(f_computeNot){
     	 //wb prop to max_sens	and	wr = REYN_MIGR_RATIO * wm	and	wr + wb + wm = 1
  
     	 if (log(max_sens) > log(BRAITENBERG_LOWER_THRESH)){
     		 wb = log(max_sens) / log(BRAITENBERG_UPPER_THRESH); // Above upper threshold, do only avoidance;
     		 wb = wb > 1.0 ? 1.0 : wb;
     		 flag_obstacle = 1;
     	 }
     	 else{
     		 wb = 0.0;
     		 flag_obstacle = 0;
     	 }
   	 wm = (1 - wb) / (float)(1 + REYN_MIGR_RATIO);
   	 wr = REYN_MIGR_RATIO *  wm;
     	 
    }
	
	final_speed = wb * b_speed + wm * m_speed + wr * r_speed;
    f_computeNot = ~f_computeNot;
 
  return (int) final_speed;
}



// the main function
int main(){
    int i;   						 // Loop counter
    int msl, msr;   				 // Wheel speeds
    int rmsl, rmsr;
    int mmsl, mmsr;
    int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
    int distances[NB_SENSORS];   	 // Array for the distance sensor readings
    int max_sens;   				 // Store highest sensor value
	int mode;
	
    mode = reset();   				// Reset of the robot
	set_mode(mode);
	
    msl = 0; msr = 0;
    
    
    int count=0;
    // Forever
    for(;;){
    
   	 // reset flockmates list. It's populated by sim_receive_message
   	 for(i=0;i<FLOCK_SIZE;i++){flockmates[i] = 0;}
   	 rmsl = 0; rmsr = 0;
   	 bmsl = 0; bmsr = 0;
   	 mmsl = 0; mmsr = 0;

   	 sum_sensors = 0;
   	 max_sens = 0;
   			 
   	 if (flag_brai){
   		 
   		 /* Braitenberg */
   		 for(i=0;i<NB_SENSORS;i++)
   		 {
			 distances[i] = e_get_prox(i);
   			 sum_sensors += distances[i]; // Add up sensor values
   			 max_sens = max_sens > distances[i] ? max_sens : distances[i]; // Check if new highest sensor value

   			 // Weighted sum of distance sensor values for Braitenburg vehicle
   			 bmsr += e_puck_matrix[i] * distances[i];
   			 bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
  		 }
  		 normalize_speed(&bmsr, &bmsl);
    	}    

   	 // Send and get information
   	 epuck_send_message(); // sending a ping to other robot, so they can measure their distance to this robot

   	 /// Compute self position
   	 prev_my_position[0] = my_position[0];
   	 prev_my_position[1] = my_position[1];
   			 
   	 epuck_receive_message();
    
   	 if(flag_reyn){
   		 // Reynold's rules with all previous info (updates the speed[][] table)
   		 reynolds_rules();   			 
   		 // Compute wheels speed from reynold's speed
   		 compute_wheel_speeds(&rmsl, &rmsr);
   	 }

   	 if(flag_migr){
               //attention on a remis à 0 dans reynolds et braitenberg!!!!!!!!!!!!!!!!!!!!!
   		migration_urge();
   		if(!flag_obstacle){
   		 mmsl -= BETA_MIGRATION * migr_diff;
		mmsr += BETA_MIGRATION * migr_diff;}
   		 compute_wheel_speeds(&mmsl, &mmsr);
   	 }   	 
   	 
    	/* Added by Pauline*/
   	 // Set final speed
   	 if(count < 100) {
               msl = rmsl;
               msr = rmsr;
   	 } else {
              msl = set_final_speed(bmsl,  rmsl,  mmsl,  max_sens);
              msr = set_final_speed(bmsr,  rmsr,  mmsr, max_sens);
   	 }


          migr_diff += msl -msr;
   	 
	 // A Changer Hugo
	 // Set speed
   	 
    e_set_speed_left(msl);
    e_set_speed_right(msr);

   	 update_self_motion(msl,msr);
   	 
   	 count++;
    }
}   
 


