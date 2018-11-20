/*****************************************************************************/
/* File:         raynolds2.c                                                 */
/* Version:      2.0                                                         */
/* Date:         00-Nov-18                                                   */
/* Description:  Reynolds flocking with relative positions	          */
/*               Run several simulations with differents parameters          */
/* Author: 	 00-Nov-18 by .....				     */
/* Last revision:15-Nov-18 by Mathilde Bensimhon 		           */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>


/**************************************/
/*Webots 2018b*/
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/compass.h>
/**************************************/
/*Webots 2018b*/
#define MAX_SPEED_WEB      		6.28    // Maximum speed webots
#define MAX_COMMUNICATION_DIST 	0.25    // maximum communication distance in [cm]
/**************************************/

#define NB_SENSORS	 	 	8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed

//<----------TO CHANGE---------->
//Scenario A 
#define FLOCK_SIZE	  		5	  // Size of flock
//Scenario B
//#define FLOCK_SIZE	  		10	  // Size of flock

#define TIME_STEP	  		64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052		// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628		// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205		// Wheel radius (meters)
#define DELTA_T				0.064		// Timestep (seconds)
#define MASQUE_ROBOT_ID		0xE0000000	// Keep 3 bits for the Robot_id
/**************************************/
/*Reynolds' rules*/
/*1) Aggregation/Cohesion*/
#define RULE1_THRESHOLD     0.20  		// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.6/10)	// Weight of aggregation rule. default 0.6/10
/*2) Dispersion/Separation*/
#define RULE2_THRESHOLD     0.015   		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	// Weight of dispersion rule. default 0.02/10
/*3) Consistency/Alignment*/
#define RULE3_WEIGHT        (1.0/10)   	// Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT_X    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATION_WEIGHT_Y    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

/*FLAGS*/
#define MIGRATORY_URGE 1 			// Disable/Enable the migration urge
#define OBSTACLE_AVOIDANCE 1 		// Disable/Enable the obstacle avoidance
#define REYNOLDS 1                  // Disable/Enable the Reynolds'rules
#define LOCAL_COMMUNICATION 1		// Disable/Enable local communication between the epucks

#define WRITE_REL_POS 0				// Disable/Enable the writing of relative position of the robot1's neighborhood

/* STATES OF FINITE STATE MACHINE */
#define ALIGN_2_MIGRATION            0
#define OBSTACLE_AVOIDANCE_STATE     1  
#define REYNOLDS_STATE               2

#define SWITCH_TO_AVOIDANCE_STATE  500  //to change - took value from slides of DISAL on internet.
#define SWITCH_TO_REYNOLDS         300  //not sure, juste choose to put an hysteresis compared to other state

/**************************************/
/* Macros */
#define ABS(x) ((x>=0)?(x):-(x))
/**************************************/


#define EPS      1e-10
#define VERBOSE  1
/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Braitenberg weights


WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node
WbDeviceTag compass;

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
char* robot_name;

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values

float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];      // X, Z, Theta of the current robot in the previous time step

float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules

int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {0,-30};	        // Migration vector

bool  neighborhood[FLOCK_SIZE] = {0};				// Table of flag: 1 means in the neighborhood of the robot. Added by Hugo (15.11.18) 
bool  prev_neighborhood[FLOCK_SIZE] = {0};

float theta_robots[FLOCK_SIZE];

int distances[NB_SENSORS];      // Array for the distance sensor readings
float to_keep[NB_SENSORS]; 		//flag of sensor to consider for computation

int state; // current state of the robot in the Finite State Machine

/*
 * Reset the robot's devices and get its ID
 */
static void reset() 
{
	wb_robot_init();

	compass = wb_robot_get_device("compass");
	wb_compass_enable(compass, TIME_STEP);

	receiver2 = wb_robot_get_device("receiver2");
	emitter2 = wb_robot_get_device("emitter2");
	
	/*Webots 2018b*/
	//get motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    /*Webots 2018b*/
	
	char s[4]="ps0";
	for(int i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;			                // increases the device number
	}

	robot_name=(char*) wb_robot_get_name(); 

	for(int i=0;i<NB_SENSORS;i++) {
	    wb_distance_sensor_enable(ds[i],64);
	}

	wb_receiver_enable(receiver2,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
  
	for(int i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}
	
  	printf("Reset: robot %d\n",robot_id_u);
	printf("Robot [%d] using urge: [%f][%f]\n", robot_id, migr[0], migr[1]);
}


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit) 
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) { 
	float theta = my_position[2];
  
	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
  
	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);
  
	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;
  
	// printf("%f %f\n", my_position[0], my_position[1]);

	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 1;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);

	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}

/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules() { 
	int j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};
	int n_flockmates[2] = {0};

	
	// Compute averages over the local flock 
	for (k=0;k<FLOCK_SIZE;k++) {
		if(k == robot_id){
			continue; // dont consider yourself in the average
		}
		
		if(neighborhood[k]){
			rel_avg_loc[0] += relative_pos[k][0];
			rel_avg_loc[1] += relative_pos[k][1];
			n_flockmates[0]++;
		}
		if(prev_neighborhood[k]){
			rel_avg_speed[0] += relative_speed[k][0];
			rel_avg_speed[1] += relative_speed[k][1];
			n_flockmates[1]++;
		}
	}
      
    if(robot_id == 0 && VERBOSE) printf("# flockmates of R0: %d\n", n_flockmates[0]);

    for(j=0;j<2;j++){
    	if(n_flockmates[0] > 0){
			rel_avg_loc[j]   /= n_flockmates[0];
			rel_avg_speed[j] /= n_flockmates[1];
		}
    	else{
			rel_avg_loc[j]   = relative_pos[robot_id][j];
			rel_avg_speed[j] = relative_speed[robot_id][j];
    	}
	}
	
	if(robot_id == 0 && VERBOSE) printf("rel_loc/rel_speed [%1.7f][%1.7f] : [%1.7f][%1.7f]\n", rel_avg_loc[0], rel_avg_loc[1], rel_avg_speed[0], rel_avg_speed[1]);

	// Rule 1 - Aggregation/Cohesion: move towards the center of mass   
    for (j=0;j<2;j++) 
	{	
		//if (sqrt(pow(relative_pos[robot_id][0]-rel_avg_loc[0],2.0)+pow(relative_pos[robot_id][1]-rel_avg_loc[1],2.0)) > RULE1_THRESHOLD){
            cohesion[j] = rel_avg_loc[j];
    	// }
	}

	if(robot_id == 0 && VERBOSE) printf("   cohesion [%1.7f][%1.7f]\n", cohesion[0], cohesion[1]);

	// Rule 2 - Dispersion/Separation: keep far enough from flockmates
	for (k=0; k<FLOCK_SIZE;k++){
		// dont consider yourself
		if(k != robot_id){
			// if flockmate is too close
			if(sqrt(pow(relative_pos[k][0], 2.0) + pow(relative_pos[k][1],2.0)) < RULE2_THRESHOLD){
				for (j=0;j<2;j++){
					//relative distance to kth flockmate
					if(ABS(relative_pos[robot_id][j]) < EPS)
						dispersion[j] -= 0.0;
					else
						dispersion[j] += 1/(relative_pos[robot_id][j]);
				}
			}
		}
  	}

	if(robot_id == 0 && VERBOSE){ printf("   dispersion [%1.7f][%1.7f]\n", dispersion[0], dispersion[1]); }

	// Rule 3 - Consistency/Alignment: match the speeds of flockmates
	for (j=0;j<2;j++) {
		// align with flock speed
		consistency[j] = rel_avg_speed[j];		
    }

	if(robot_id == 0 && VERBOSE){ printf("   consistency [%1.4f][%1.4f]\n", consistency[0], consistency[1]);}

    //aggregation of all behaviors with relative influence determined by weights
    for (j=0;j<2;j++) {
        speed[robot_id][j]  =  cohesion[j]    * RULE1_WEIGHT;
        speed[robot_id][j] +=  dispersion[j]  * RULE2_WEIGHT;
        speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
    }
    
    speed[robot_id][1] *= -1; //y axis of webots is inverted

	if(robot_id == 0 && VERBOSE){ printf("  > before migr: %1.7f %1.7f\n", speed[robot_id][0], speed[robot_id][1]);}

	if(MIGRATORY_URGE == 0){
		speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
		speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
	}
	else {
		speed[robot_id][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT_Y;
		speed[robot_id][1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT_X; //y axis of webots is inverted
	
		if(robot_id == 0 && VERBOSE){ printf("  > after  migr: %1.7f %1.7f\n", speed[robot_id][0], speed[robot_id][1]);}
	}	
}


/*	sim_send_message(void) // added by Hugo (16.11.18)
 *
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction.
 *
*/
void sim_send_message(void)  
{
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter2,out,strlen(out)+1);  
}

void sim_receive_message(void)
{
    const double *message_direction;
    double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
    int other_robot_id;

	for(int i=0; i<FLOCK_SIZE;i++){
		// Store the previous neighborhood -> goal:needed to compute the speed
		prev_neighborhood[i] = neighborhood[i];
		// Clear the neighborhood table
		neighborhood[i] = 0;
	}


	while (wb_receiver_get_queue_length(receiver2) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver2);
		message_direction = wb_receiver_get_emitter_direction(receiver2);
		message_rssi = wb_receiver_get_signal_strength(receiver2);
		double y = message_direction[2];
		double x = message_direction[1];

        theta =	-atan2(y,x);
        theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));
		

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
		
		// Get position update
		//theta += dtheta_g[other_robot_id];
		//theta_robots[other_robot_id] = 0.8*theta_robots[other_robot_id] + 0.2*theta;

		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);
		if(range <= MAX_COMMUNICATION_DIST){
			relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
			relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);		

			// Set Flag
			neighborhood[other_robot_id] = 1;
			// Update the prev position
			prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
			prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];
			prev_relative_pos[other_robot_id][2] = relative_pos[other_robot_id][2];
			// Set relative location
			relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
			relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos
			relative_pos[other_robot_id][2] = theta;
						  
			if(prev_neighborhood[other_robot_id]){ // Take only in account the previouse neighborhood
				// Set relative speed
				relative_speed[other_robot_id][0] = (relative_pos[other_robot_id][0] - prev_relative_pos[other_robot_id][0])/DELTA_T;
				relative_speed[other_robot_id][1] = (relative_pos[other_robot_id][1] - prev_relative_pos[other_robot_id][1])/DELTA_T;
			}
		}

		wb_receiver_next_packet(receiver2);
	}
}


void sensor_to_keep() {
	static float lutAngle[] = {5.986479, 5.410521, 4.712389, 3.665191, 2.617994, 1.570796, 0.8726646, 0.296706}; //radian of middle of position of sensors

	// Initialization of to_keep at 1
	for(int i=0; i<NB_SENSORS;i++){to_keep[i] = 1;}
	
    // to_keep: get the two closest sensors around the orientation of the other robot and put 0 to not consider them afterwards
    for(int j=0;j<FLOCK_SIZE; j++) {
    	if(neighborhood[j] == 1) {
			
			if(relative_pos[j][2] > lutAngle[0] || relative_pos[j][2] < lutAngle[7]) {
				to_keep[0] = 0; 
				to_keep[7] = 0;
			}       	
			else if(relative_pos[j][2] > lutAngle[1]) {
				to_keep[0] = 0;
				to_keep[1] = 0;
			} 	
			else if(relative_pos[j][2] > lutAngle[2]) {
				to_keep[1] = 0;
				to_keep[2] = 0;
			}  	
			else if(relative_pos[j][2] > lutAngle[3]) {
				to_keep[2] = 0;
				to_keep[3] = 0;
			}
			else if(relative_pos[j][2] > lutAngle[4]) {
				to_keep[3] = 0;
				to_keep[4] = 0;
			}  
			else if(relative_pos[j][2] > lutAngle[5]) {
				to_keep[4] = 0;
				to_keep[5] = 0;
			} 	
			else if(relative_pos[j][2] > lutAngle[6]) {
				to_keep[5] = 0;
				to_keep[6] = 0;
			}  	
			else if(relative_pos[j][2] > lutAngle[7]) {
				to_keep[6] = 0;
				to_keep[7] = 0;
			} 
		}
    }
}

void obstacle_avoidance(int* bmsl, int* bmsr, int* max_sens, int* sum_sensors)
{

	//sensor_to_keep(); // ALREADY CALLED ON MAIN LOOP

	/* Braitenberg */
	for(int i=0;i<NB_SENSORS;i++) 
	{
		// !! state_switcher already populates distances[i]
		//distances[i] = wb_distance_sensor_get_value(ds[i]);           //Read sensor values

        *sum_sensors += distances[i] * to_keep[i]; 
        *max_sens = *max_sens > distances[i] ? *max_sens:distances[i]; // Check if new highest sensor value

        // Weighted sum of distance sensor values for Braitenburg vehicle
        *bmsr += e_puck_matrix[i] * distances[i] * to_keep[i];             // Add up sensor values ***IF*** in sensor to consider
        *bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i] * to_keep[i];
    }

	// Adapt Braitenberg values (empirical tests)
    *bmsl/=MIN_SENS + 66;
    *bmsr/=MIN_SENS + 72;
    
}

// Finite State Machine
/*
* Sample FSM structure given below: 
*        REYNOLDS_STATE is reynolds: flock speed set with reynolds rules.
*        OBSTACLE_AVOIDANCE_STATE is obstacle avoidance
*
*        If in REYNOLDS_STATE, check if you are close to an obstacle (using distance sensors)
*        If yes, change to OBSTACLE_AVOIDANCE_STATE, otherwise stay in REYNOLDS_STATE
*
*        If in OBSTACLE_AVOIDANCE_STATE, check if you are far enough from obstacles
*        If yes, change to REYNOLDS_STATE, otherwise stay in OBSTACLE_AVOIDANCE_STATE
*/
void state_switcher(void){
	int maxDist = 0;

	if(robot_id == 0 && VERBOSE) {printf("distances[i] :: ");}
	
	for (int i = 0; i < NB_SENSORS; i++)
	{
	  	distances[i] = wb_distance_sensor_get_value(ds[i]) * to_keep[i];  //Read sensor values
	  	
	  	if(robot_id == 0 && VERBOSE) {printf("[%d]",distances[i]);}
		
		if (maxDist < distances[i]){
			maxDist = distances[i];
		}
	}
	
	if(robot_id == 0 && VERBOSE) {printf("\n");}

	// Check state change, if the condition is met (e.g. no obstacles in sight) change state
    if(maxDist < SWITCH_TO_REYNOLDS && state == OBSTACLE_AVOIDANCE_STATE){ // hysteresis
      state = REYNOLDS_STATE;
      if(robot_id == 0 && VERBOSE) {printf("STATE = REYNOLDS\n");}
    }
    if(maxDist > SWITCH_TO_AVOIDANCE_STATE && state == REYNOLDS_STATE){
    	state = OBSTACLE_AVOIDANCE_STATE;
    	if(robot_id == 0) {printf("STATE = OBST AVOID\n");}
    }
}

// the main function
int main(){ 
	int msl=0; 
	int msr=0;			// Wheel speeds
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int max_sens= 0;			// Store highest sensor value

	/*******************************************************************/


 	reset();			// Resetting the robot
 	
 	state = REYNOLDS_STATE; 

	// Forever
	for(;;) {

		bmsl = 0; 
		bmsr = 0;
        sum_sensors = 0;
		max_sens = 0;

		//sensor_to_keep();
		for(int i=0; i<NB_SENSORS;i++){to_keep[i] = 1;}


	    state_switcher();

		if(robot_id == 0 && VERBOSE){ printf("\n 0) [msl][msr] [%d][%d] :: [bmsl][bmsr] [%d][%d]\n", msl, msr, bmsl, bmsr);}	

		if(state == OBSTACLE_AVOIDANCE_STATE){
			if(OBSTACLE_AVOIDANCE){
				obstacle_avoidance(&bmsl, &bmsr, &max_sens, &sum_sensors);
				if(robot_id == 0 && VERBOSE) printf(" 1) [bmsl][bmsr] [%d][%d]", bmsl, bmsr);
			}
		}

		if(LOCAL_COMMUNICATION){
			sim_send_message();  		// sending a message to other robot in the neighborhood.
		}
		
		// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		
		update_self_motion(msl,msr);
		
		if(robot_id == 0 && VERBOSE) printf(" 2) [msl][msr] [%d][%d]\n", msl, msr);

		sim_receive_message();

		if(robot_id == 0 && VERBOSE){printf("R0 neighborhood :: ");}
		for(int p = 0; p<FLOCK_SIZE; p++){
			if(robot_id == 0 && VERBOSE && neighborhood[p]){ printf("[%d]", p);}
		}
		if(robot_id == 0 && VERBOSE){printf("\n");}

		speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
		speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);


		if(state == REYNOLDS_STATE){
			reynolds_rules();
		}

		// Compute wheels speed from speed table
		compute_wheel_speeds(&msl, &msr);

		if(robot_id == 0 && VERBOSE){ printf(" 3) [msl][msr] [%d][%d]\n", msl, msr);}

		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}

		if(robot_id == 0 && VERBOSE){ printf(" 4) [msl][msr] [%d][%d]\n", msl, msr);}

		if(state == OBSTACLE_AVOIDANCE_STATE){
			// Add Braitenberg
			msl += bmsl;
			msr += bmsr;

			if(robot_id == 0 && VERBOSE){ printf("\n 5) [msl][msr] [%d][%d] :: [bmsl][bmsr] [%d][%d]\n", msl, msr, bmsl, bmsr);}	
		}

		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;

		if(robot_id == 0 && VERBOSE){ printf(" 6) [msl_w][msr_w] [%f][%f]\n", msl_w, msr_w);}

		wb_motor_set_velocity(left_motor, msl_w);
        wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/
    
		// Continue one step
		wb_robot_step(TIME_STEP);
	}	 
}
