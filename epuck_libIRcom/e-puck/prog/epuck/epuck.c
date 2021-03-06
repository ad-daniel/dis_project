/********************************************************************************/
/* File:     	epuck1.c                                             			*/
/* Version:  	1.0                                                     		*/
/* Date:     	06-Dec-18                                               		*/
/* Description:  Reynolds flocking with relative positions for real E-Pucks   	*/
/*                                                                       		*/
/* Author:      Group 10			  											*/
/* Last revision: 06-Dec-18 Hugo Grall Lucas   			  					 	*/
/********************************************************************************/

//-------------------------------------------------------------------------------------------------//
		               	   //Libraries//
//-------------------------------------------------------------------------------------------------//

			
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
typedef enum { false = 0, true = !false } bool;


			/*To Change for each robot */
static const int robot_id = 1;
static const int robot_group = 0;

//-------------------------------------------------------------------------------------------------//
			//Variables, parameters & macro//
//-------------------------------------------------------------------------------------------------//


			
#define NB_SENSORS    	8   	// Number of distance sensors
#define NB_LEDS		8
#define MIN_SENS      	350 	// Minimum sensibility value
#define MAX_SENS      	4096	// Maximum sensibility value

#define MAX_TIME_ALONE 10
#define JOIN_SPEED 50         // Speed added in join when join mode (retunrning to migr_diff = 0)
#define MAX_SPEED_BMR 400      // Maximum speed
#define NO_REYN_BIAS 50      // Difference in saturation speed if robot is alone
#define JOIN_BIAS          0.5

#define FLOCK_SIZE     	3  	// Size of flock			
#define M_PI 3.14159265358979323846

#define AXLE_LENGTH      0.052   		// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS  0.00628   		// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS     0.0205    		// Wheel radius (meters)

#define RULE1_THRESHOLD 	0.1  	   		// Threshold to activate aggregation rule.
#define RULE2_THRESHOLD 	0.12  			// Threshold to activate dispersion rule.
#define RULE1_WEIGHT 	0.5
#define RULE2_WEIGHT 	0.95
#define RULE3_WEIGHT 	0.1
static float migr[2] = {1.,0.};

#define REYN_MIGR_RATIO  			0.5 		// TO TUNE: ratio of weights between Reynolds and Migration urge  	 
#define BRAITENBERG_UPPER_THRESH	2000
#define BRAITENBERG_LOWER_THRESH 	150 		// below this value, no avoidance
#define BRAITENBERG_SPEED_BIAS		700
#define WAIT_FOR_SENDING			1			// Avoid to send a to large rate of message -> Problem: Large number of receiver failed... 
#define WAIT_FOR_SENDING_RESET		1			// Avoid to send a to large rate of message -> Problem: Large number of receiver failed... 
#define WAIT_FOR_READING			1			// 


static const int TIME_RESET = 100;
static const int TIME_READING = 800;
static const int TIME_SENDING = 800;
static const int TIME_STEP = 10;
//static long int time = 0;	// current time in [ms]

// Possible mode
#define MODE_PAUSE 		0
#define MODE_NO_SPEED 	        1
#define MODE_BRAI		2
#define MODE_REYN		3
#define MODE_MIGR		4
#define MODE_JOIN		5
#define MODE_ALL		6

// Macro
#define ABS(x) ((x>=0)?(x):-(x))

/*Try other*/
#define BETA_MIGRATION 1        				// how much of migr_diff to implement during each step 

static const int e_puck_matrix[16] =   {16,9,6,3, -1,-5,-6,-8,   -12,-8,-5,6, 2,4,5,9}; // for obstacle avoidance giving priority to a side
static float relative_pos[FLOCK_SIZE][3];    // relative X, Z, Theta of all robots
static float prev_relative_pos[FLOCK_SIZE][3];    // Previous relative  X, Z, Theta values
static float prev_global_pos[FLOCK_SIZE][2];    // Previous global  X, Z values

static float my_position[3];			 // X, Z, Theta of the current robot
static float prev_my_position[3]; 		 // X, Z, Theta of the current robot in the previous time step
static float speed[FLOCK_SIZE][2];   	      // Speeds calculated with Reynold's rules
static float relative_speed[FLOCK_SIZE][2];    // Speeds calculated with Reynold's rules
static long int time = 0;					// Loop counter
static char buffer[80];
// tracks which are my flockmates. Myself excluded.
// 1 : is my flockmate and 0 : isn't my flockmate
static bool flockmates[FLOCK_SIZE] = {0};          // Table of flag: 1- is flockmate ; 0- is not flockmate
static float migr_diff = 0;                        // Integrator of the wheels difference during the obstacle avoidance state
int n_flockmates = 0;                                // Number of flockmate
bool no_reynolds = 0;                                // flag to deactivate reynolds: 1- reynold deactivated ; 0- reynold activated

int pow_id;                               //CHECK IS STILL NEEDED

int join_speed = JOIN_SPEED;     // Speed added in join when join mode (retunrning to migr_diff = 0)
int max_speed_bmr = MAX_SPEED_BMR;      // Maximum speed

// Flags
static bool flag_verbose = true;
static bool flag_brai = false;
static bool flag_reyn = false;
static bool flag_migr = false;
static bool flag_join = false;
static bool flag_speed = true;
static bool flag_time_reading = true;
static bool flag_time_sending = true;
static bool flag_time_reset = true;
static bool flag_verbose_timer = false;
static bool flag_verbose_speed = false;


int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}
/*
 * Reset the robot's devices and get its ID
 */
 void _current_time(void){
	 time += TIME_STEP;
 }
 void _reset_time(void){
	if(flag_verbose_timer){
	sprintf(buffer,"[Robot id: %d] [Robot Group: %d]<< RESET INTERUPT >>\r\n",robot_id,robot_group);
	e_send_uart1_char(buffer, strlen(buffer));
	}
	flag_time_reset = false;
}
void _sending_time(void){
	if(flag_verbose_timer){
	sprintf(buffer,"[Robot id: %d] [Robot Group: %d]<< SENDING INTERUPT >>\r\n",robot_id,robot_group);
	e_send_uart1_char(buffer, strlen(buffer));
	}
	flag_time_sending = false;
	time += TIME_SENDING/(float)(pow_id);
}
void _reading_time(void){
	if(flag_verbose_timer){
	sprintf(buffer,"[Robot id: %d] [Robot Group: %d]<< READING INTERUPT >>\r\n",robot_id,robot_group);
	e_send_uart1_char(buffer, strlen(buffer));
	}
	flag_time_reading = false;
	time += TIME_READING/(float)(pow_id);
}

int reset() {
/*
 * Reset()
 * Initialisation of the robots and of communication.
 */
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
	if(flag_verbose){
	sprintf(buffer,"[Robot id: %d] [Robot Group: %d]<< Reset Start >>\r\n",robot_id,robot_group);
	e_send_uart1_char(buffer, strlen(buffer));
	}
	
	e_calibrate_ir(); 

    // initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();
	
    // rely on selector to define the role
    int selector = getselector();
	int time = TIME_RESET * (robot_id + 1);
    e_activate_agenda(_reset_time, time);
    flag_time_reset = true;
	do{
		asm("nop");
	}while(WAIT_FOR_SENDING_RESET && flag_time_reset);
	e_destroy_agenda(_reset_time);

	int i = 0;
	pow_id = 1;
	for(i=1; i<=robot_id; i++) {
		pow_id*= 2;                  //Set prescaler to change between receive and send 
	}
	return(selector);
}

void set_mode(int mode){
/*
 * set_mode()
 * Set mode for testing and debugging
 */
	switch(mode){
		case MODE_PAUSE:
			flag_verbose = false;
			flag_speed = false;
		break;
		case MODE_NO_SPEED:
			flag_brai = false;
			flag_reyn = false;
			flag_migr = false;
			flag_speed = false;
			if(flag_verbose){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode No_SPEED>>\r\n",robot_id, robot_group);
			btcomSendString(buffer);	
			}
		break;
		case MODE_BRAI:
			flag_brai = true;
			if(flag_verbose){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode Brai>>\r\n",robot_id, robot_group);
			btcomSendString(buffer);	
			}
		break;
		case MODE_REYN:
			flag_reyn = true;
			if(flag_verbose){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode Reyn>>\r\n",robot_id, robot_group);
			btcomSendString(buffer);	
			}
		break;
		case MODE_MIGR:
			flag_migr = true;
			if(flag_verbose){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode Migr>>\r\n",robot_id, robot_group);
			btcomSendString(buffer);	
			}
		break;
		case MODE_JOIN:
			flag_brai = true;
			flag_migr = true;
			flag_join = true;
			if(flag_verbose){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode Join>>\r\n",robot_id, robot_group);
			btcomSendString(buffer);	
			}
		break;
		case MODE_ALL:
			flag_brai = true;
			flag_reyn = true;
			flag_migr = true;
			if(flag_verbose){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode All>>\r\n",robot_id, robot_group);
			btcomSendString(buffer);	
			}
		break;
		default:
		if(flag_verbose){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Mode Unknown>>\r\n",robot_id, robot_group);
			btcomSendString(buffer);	
			}
			int i;
		for(i=0; i<NB_LEDS; i++){
			e_set_led(i, 1); // set on all the leds
		}	
		break;
	}
	flag_verbose = false;
}

void update_self_motion(int msl, int msr) {
/*
 * update_self_motion()
 * Update the robot's states thanks to Odometry.
 */
    float theta = my_position[2];
 
// Compute deltas of the robot
    float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * time;
    float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * time;
    float du = (dr + dl)/2.0;
    float dtheta = (dr - dl)/AXLE_LENGTH;
        
// Compute deltas x and y according to the motion
    float dx = du * cosf(theta+dtheta/2);
    float dy = du * sinf(theta+dtheta/2);
	
// Update the previous position with the current one
    prev_my_position[0] = my_position[0]; 	
    prev_my_position[1] = my_position[1];
    
// Update positions (robot's states)
    my_position[0] += dx;
    my_position[1] += dy;
    my_position[2] += dtheta;
     	 
// Keep orientation between [0,2*pi]
    my_position[2] = (my_position[2] < 0      ? my_position[2]+2*M_PI : my_position[2]); // case: bellow 0 rad
    my_position[2] = (my_position[2] > 2*M_PI ? my_position[2]-2*M_PI : my_position[2]); // case: above 2*pi rad 
}



void normalize_speed(int *msl, int *msr, int max_speed) {
/*
 * normalize_speed()
 * Keep the velocity bellow max_speed and keep also the angle of the speed vector.
 */
    if(ABS(*msl) > max_speed || ABS(*msr) > max_speed){
      float max = (ABS(*msl) > ABS(*msr) ? ABS(*msl):ABS(*msr));
      *msl = *msl/max * max_speed;
      *msr = *msr/max * max_speed;
    }
}

void compute_wheel_speeds(int *msl, int *msr) {
/*
 * compute_wheel_speeds()
 * Transforme the desire speed into the robot's motor speeds with 2 proportional control laws.
 */
    static float Ku = 0.02;   // Forward control proportional gain
    static float Kw = 0.08;   // Rotational control proportional gain
    
    float x = speed[robot_id][0];
    float y = speed[robot_id][1];
    float range = sqrtf(x*x + y*y);      // Distance to the wanted position
    float bearing = atan2(y, x);         // Orientation of the wanted position
// Forward control law
    float u = Ku*range;
// Rotational control Law
    float w = Kw*bearing;
// Set the robot's motor speeds according to the forward and rotational motion
    *msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    *msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
// Keep the speed in the allowed range and take care to avoid saturation
    normalize_speed(msl, msr, max_speed_bmr);
    
}

void global2rel(float* v_global, float* v_ref) { 
/*
 * global2rel()
 * Transforme a vector in the global ref into one in the relative one (at the current time step).
 */
    v_ref[0] =   cos(my_position[2])*(v_global[0]) + sin(my_position[2])*(v_global[1]);
    v_ref[1] = - sin(my_position[2])*(v_global[0]) + cos(my_position[2])*(v_global[1]);
}

void rel2global(float* v_ref, float* v_global) {
/*
 * rel2global()
 * Transforme a vector in the relative ref (at the current time step) into one in the global one.
 */
    v_global[0] =   cos(-my_position[2])*(v_ref[0]) + sin(-my_position[2])*(v_ref[1]) ;
    v_global[1] = - sin(-my_position[2])*(v_ref[0]) + cos(-my_position[2])*(v_ref[1]) ;
}



void migration_urge(void) {
/*
 * migration_urge()
 * Transforme the migration urge vector from the global referentiel to the local one and set the result in the desired speed vector.
 */
    global2rel(migr,speed[robot_id]);  
}

void reynolds_rules() {
/*
 * reynolds_rules()
 * Compute the 3 terms of the reynolds rules, do a weighted sum and set the result into the desired speed vector.
 */
// Initialisation of the different parameters
    int j, k;   	               // Loop counters
    float rel_avg_loc[2] = {0,0};    // Flock average positions
    float rel_avg_speed[2] = {0,0};  // Flock average speeds
    float cohesion[2] = {0,0};       // Cohesion speed
    float dispersion[2] = {0,0};     // Dispersion speed
    float consistency[2] = {0,0};    // Consistency speed
    n_flockmates = 0;                // Current number of flockmates    
    float dist = 0.0;                // Distance between the robots
    speed[robot_id][0] = 0;          // Reset the desired speed
    speed[robot_id][1] = 0;
    
// Compute averages over the whole flock
    for(k=0;k<FLOCK_SIZE;k++){
   	 if(k == robot_id){ 
   		 continue; // Does not take into account itself for the averages
   	 }

   	 if(flockmates[k]){
   		 for (j=0;j<2;j++) {
   			 rel_avg_speed[j] += relative_speed[k][j];
   			 rel_avg_loc[j] += relative_pos[k][j];
   		 }
   		 n_flockmates++; // Increase the current number of flockmates
   	 }
    }
    
    if(n_flockmates>0){
   	 for(j=0;j<2;j++){
   		 rel_avg_speed[j] /= n_flockmates;
   		 rel_avg_loc[j] /= n_flockmates;
   	 }
    }
    else{ 
   	 return;  // No flockmates, no reynolds to be done
    }

// Rule 1 - Aggregation/Cohesion: move towards the center of mass
    dist = sqrt(rel_avg_loc[0]*rel_avg_loc[0] + rel_avg_loc[1]*rel_avg_loc[1]);
    if(dist > RULE1_THRESHOLD){
   	 for (j=0;j<2;j++){    
   		 cohesion[j] = rel_avg_loc[j];
   	 }
    }

// Rule 2 - Dispersion/Separation: keep far enough from flockmates
    for(k=0;k<FLOCK_SIZE;k++){
   	 if(k == robot_id){
   		 continue;
   	 }

   	 dist = sqrt(relative_pos[k][0]*relative_pos[k][0] + relative_pos[k][1]*relative_pos[k][1]);

   	 if(dist < RULE2_THRESHOLD && flockmates[k]){
   		 for (j=0;j<2;j++) {
   			 dispersion[j] -= (relative_pos[k][j]); // Small change from the initiale reynolds rule
   		 }
   	 }
    }    
 
// Rule 3 - Consistency/Alignment: match the speeds of flockmates
    for (j=0;j<2;j++){
   	 consistency[j] = rel_avg_speed[j];
    }

// Weighted sum of all the 3 rules
    for (j=0;j<2;j++){
   	 speed[robot_id][j] +=  cohesion[j]    * RULE1_WEIGHT;
   	 speed[robot_id][j] +=  dispersion[j]  * RULE2_WEIGHT;
   	 speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
    }
}

void epuck_send_message(void)  
{
	long int message;
	message = (robot_id + (robot_group<<3));
	if(flag_verbose){
		sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<epuck_send_message: Begin>>\r\n",robot_id, robot_group);
		btcomSendString(buffer);	
	}
	ircomSend(message);	  

	do{
		while (ircomSendDone() == 0);
		// takes ~15knops for a 32window, avoid putting messages too close...
	    //for(j = 0; j < 10; j++)	asm("nop");
	}while(flag_time_sending);
		
}
void epuck_receive_message(void){
	
	bool read_buffer = true;
    if(flag_verbose){
		sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<epuck_receive_message: Begin>>\r\n",robot_id, robot_group);
		btcomSendString(buffer);	
	}
	while (read_buffer || flag_time_reading) // Read all the buffer
	{
	    IrcomMessage imsg;
	    ircomPopMessage(&imsg);
	    if (imsg.error == 0)
	    {
			if(flag_verbose){
				sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<epuck_receive_message: imsg.error = 0>>\r\n",robot_id, robot_group);
				btcomSendString(buffer);	
			}
			// get the message
			long int message_value = imsg.value;
			// get the distance
			float range = imsg.distance;
			// get the orientation
			float theta = imsg.direction;
			// get the sensor from witch we get the message
			int id_sensor = imsg.receivingSensor;
			
			// get the robot emitter id
			int other_robot_id = 0x0007 & message_value;
			int other_robot_group = message_value>>3;
			// check if message is my own (caused by reflection) and if it belongs to my group
			if((other_robot_id != robot_id) && (other_robot_group == robot_group)){
				
				global2rel(prev_global_pos[other_robot_id], prev_relative_pos[other_robot_id]);
					 
				// Set current relative location
				relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
				relative_pos[other_robot_id][1] = range*sin(theta);   // relative y pos
				relative_pos[other_robot_id][2] = theta;
				
				if(flag_verbose){
					sprintf(buffer, "Receive successful : [%d]  - distance=%f \t direction=%f \r\n", other_robot_id, (double)range,(double)theta);
					btcomSendString(buffer);
				}
										  
				// Compute speed in relative coordinates
				relative_speed[other_robot_id][0] = (relative_pos[other_robot_id][0] - prev_relative_pos[other_robot_id][0])/time;
				relative_speed[other_robot_id][1] = (relative_pos[other_robot_id][1] - prev_relative_pos[other_robot_id][1])/time;

				// Set relative_pos in global coordinates of this time step (x and y only)
				/* prev_global_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
				prev_global_pos[other_robot_id][1] = relative_pos[other_robot_id][1];*/

				rel2global(relative_pos[other_robot_id],  prev_global_pos[other_robot_id]);

				flockmates[other_robot_id] = true;
			}
		}
	    else if (imsg.error > 0)
	    {
		// error in transmission
			if(flag_verbose){
				sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<epuck_receive_message: imsg.error > 0>>\r\n",robot_id, robot_group);
				btcomSendString(buffer);	
			}	
	    }
		else
		{
			// else imsg.error == -1 -> no message available in the queue
			if(flag_verbose){
				sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<epuck_receive_message: imsg.error = -1>>\r\n",robot_id, robot_group);
				btcomSendString(buffer);	
			}
			read_buffer = false;
		}
	}
	if(flag_verbose || flag_verbose_timer){
		int j;
		int sum = 0;
		for(j=0;j<FLOCK_SIZE;j++){sum +=flockmates[j];}
		sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<epuck_receive_message: End >>\r\n",robot_id, robot_group);
		sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<the robot has %d flockmate>>\r\n",robot_id, robot_group,sum);
		btcomSendString(buffer);	
	}
}

void set_weights(int max_sens,float * wb,float * wm,float * wr,float * wj) {
/*
 * set_weights()
 * set the weight depending on environment for following final speed computation
*/
  // If higher sensing value than noise, obstacle avoidance   	 
  if (log(max_sens) > log(BRAITENBERG_LOWER_THRESH)){            
	 *wb = log(max_sens) / log(BRAITENBERG_UPPER_THRESH); // Above upper threshold, do only avoidance;
	 *wb = *wb > 1.0 ? 1.0 : *wb;                         // Weight can not be above 1 otherwise too high speed value
	 *wj = 0;                                             // Ignore join
   // If no obstacle 
   } else {
	 *wb = 0.0;                                           // Ignore noisy sensors
          	 *wj = 1 - n_flockmates/(FLOCK_SIZE - 1);             // Join weights decreases with number of flockmates
   }
   
   // If the robot is alone for more  than MAX_TIME_ALONE continous time step (see check_if_alone())
   if(no_reynolds) {
            *wm = 1 - *wb;                              // Migration weight is more important     
            *wr = 0;                                    // Do not implement Reynold
   // If it has flockmates
   }else {                                              // Implement solution of wb + wr + wm = 1 and wr = REYN_MIGR_RATIO * wm
        *wm = (1 - *wb) / (float)(1 + REYN_MIGR_RATIO); 
        *wr = REYN_MIGR_RATIO *  (*wm); 
   }
return;
}



//Condition pour moins faire reynolds si on avance pas
int check_if_alone(int alone) {
/*
 * check_if_alone()
 * Check if the robot has been alone more than MAX_TIME_ALONE time step.
 * If it is the case, the flag no_reynold is set (affects set_weights() computation)
 * The flag is removed as soon as the robot has a flockmate
*/
   // If no flockmate, increment counter
   if( n_flockmates == 0) { 
       alone += 1;
   }

   // If robot alone for more than MAX_TIME_ALONE time step
   if(alone > MAX_TIME_ALONE) {
      alone = 0;
      no_reynolds = true;   // Set no_reynold flag (Reynold will be ignored)
      max_speed_bmr = MAX_SPEED_BMR - NO_REYN_BIAS;  // Decrease speed of Braitenberg and Migration
      join_speed = JOIN_SPEED + NO_REYN_BIAS;     // Increase speed of Join 
   }
   
   // If there is a flockmate, set flag to 0 and reset counter
   if(n_flockmates > 0) {
     alone = 0;
     no_reynolds = false; 
     max_speed_bmr = MAX_SPEED_BMR;
     join_speed = JOIN_SPEED;
   }
   
   return alone;
}



// the main function
int main(){

    int msl = 0; int msr = 0;   	                      // Wheel speeds:
    int rmsl, rmsr;				// 	r: for Reynold
    int mmsl, mmsr;				// 	m: for migratory urge
    int jmsl, jmsr;				// 	j: for joint
    int bmsl, bmsr;				// 	b: for braitenberg
    int distances[NB_SENSORS];   	 // Array for the distance sensor readings
	int mode;

    int sum_sensors; 				// Sum of the sensors value
    int max_sens;   				// Store highest sensor value
    float wb, wm, wr, wj;                              // Weights for the different speeds
    int alone = 0;       	                      // Sum of the not_moving time step due to REYN_MIGR_RATIO	
	
// Step 0 : RESET and set mode
    mode = reset();   				// Reset of the robot
	set_mode(mode);
	    
    
    int count=0;
// Step 1: INFINITY LOOP
    for(;;){
// Loop step 0 : INITIALISATION		

		time = 0; // time of loop (TIME_STEP of webots);
		// Start current time
		e_activate_agenda(_current_time, TIME_STEP);

		int i;   				 // Loop counter		
		for(i=0;i<FLOCK_SIZE;i++){flockmates[i] = 0;} // reset flockmates list. It's populated by sim_receive_message
		rmsl = 0; rmsr = 0;
		bmsl = 0; bmsr = 0;
		mmsl = 0; mmsr = 0;
   	 	jmsl = 0; jmsr = 0;

		sum_sensors = 0;
		max_sens = 0;

		// Loop step 1 : BRAITENBERG
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
			normalize_speed(&bmsr, &bmsl, max_speed_bmr);
			}    
		
		e_destroy_agenda(_current_time);
		
		// Send and get information
		int j = 0;
		while(j<pow_id){
			// Loop step 2 : SENDING MESSAGE
			flag_time_sending = true;
			e_activate_agenda(_sending_time, TIME_SENDING/(float)(pow_id));
			epuck_send_message(); // sending a ping to other robot, so they can measure their distance to this robot
			e_destroy_agenda(_sending_time);
			
			/// Compute self position
			prev_my_position[0] = my_position[0];
			prev_my_position[1] = my_position[1];
			
			// Loop step 3 : RECEIVE MESSAGE
			flag_time_reading = true;
			e_activate_agenda(_reading_time, TIME_READING/(float)(pow_id));
			epuck_receive_message();
			e_destroy_agenda(_reading_time);

			j++;
		}
		// Verbose: number of flockmates
		if(true){
			int sum = 0;
			for(j=0;j<FLOCK_SIZE;j++){sum +=flockmates[j];}
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] %d \r\n",robot_id, robot_group,sum);
			btcomSendString(buffer);	
		}
		
		// Get current time
		e_activate_agenda(_current_time, TIME_STEP);
		
		// Loop step 4 : REYNOLDS
		if(flag_reyn){
			// Reynold's rules with all previous info (updates the speed[][] table)
			reynolds_rules();   			 
			// Compute wheels speed from reynold's speed
			compute_wheel_speeds(&rmsl, &rmsr);
		}
		
		// Loop step 5 : MIGRATORY_URGE
		if(flag_migr){
			//attention on a remis à 0 dans reynolds et braitenberg!!!!!!!!!!!!!!!!!!!!!
			migration_urge();
			compute_wheel_speeds(&mmsl, &mmsr);
		}   

		// Loop step 6 : JOIN	 
   	    	if(flag_join){
			jmsl =  -migr_diff/2 + JOIN_BIAS;
			jmsr =   migr_diff/2 + JOIN_BIAS;
			normalize_speed(&jmsr, &jmsl, join_speed);   	
   	 	}
	
		// Loop step 7 : COMPUTE THE FINAL SPEED
		set_weights(max_sens,&wb,&wm,&wr,&wj); 
        	msl = wb * bmsl + wm * mmsl + wr * rmsl + wj * jmsl; 
            	msr = wb * bmsr + wm * mmsr + wr * rmsr + wj * jmsr; 

		alone = check_if_alone(alone);
		migr_diff += msl - msr;


		// A Changer Hugo
		// Loop step 8 : SET THE FINAL SPEED ON THE MOTORS
		if(!flag_speed){
			msl = 0;
			msr = 0;
		}
		switch(mode){
		case MODE_REYN:
			msl = rmsl;
			msr = rmsr;
		break;	
		case MODE_BRAI:
			msl = bmsl;
			msr = bmsr;
		break;
		case MODE_MIGR:
			msl = mmsl;
			msr = mmsr;
		break;
		}
		
		if(flag_verbose_speed){
			sprintf(buffer, "[Robot id: %d]<< msr = %d | msl = %d >>\r\n",robot_id,msr, msl );
			btcomSendString(buffer);
		}
		
		e_set_speed_left(msl);
		e_set_speed_right(msr);
		
		// Loop step 9 : UPDATE ROBOT STATES	
		update_self_motion(msl,msr);		// Update the robot states according to the odometry

		count++;


		if(flag_verbose_timer){
			sprintf(buffer, "[Robot id: %d] [Robot Group: %d] <<Time Loop = %ld >>\r\n",robot_id, robot_group,time);
			btcomSendString(buffer);	
		}
	}
}
