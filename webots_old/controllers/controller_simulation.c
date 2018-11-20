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
#define RULE1_WEIGHT        (0.3/10)	// Weight of aggregation rule. default 0.6/10
/*2) Dispersion/Separation*/
#define RULE2_THRESHOLD     0.15   		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	// Weight of dispersion rule. default 0.02/10
/*3) Consistency/Alignment*/
#define RULE3_WEIGHT        (1.0/10)   	// Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT_X    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATION_WEIGHT_Y    (0.01*10)   // Wheight of attraction towards the common goal. default 0.01/10

/*FLAGS*/
#define MIGRATORY_URGE 1 			// Disable/Enable the migration urge
#define OBSTACLE_AVOIDANCE 0 		// Disable/Enable the obstacle avoidance
#define REYNOLDS 1                  // Disable/Enable the Reynolds'rules
#define LOCAL_COMMUNICATION 1		// Disable/Enable local communication between the epucks

#define WRITE_REL_POS 1				// Disable/Enable the writing of relative position of the robot1's neighborhood
/**************************************/
/* Macros */
#define ABS(x) ((x>=0)?(x):-(x))
/**************************************/

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Braitenberg weights


WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
char* robot_name;

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values

float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];      // X, Z, Theta of the current robot in the previous time step

float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules

int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {25,-25};	        // Migration vector

bool  neighborhood[FLOCK_SIZE] = {0};				// Table of flag: 1 means in the neighborhood of the robot. Added by Hugo (15.11.18) 
bool  prev_neighborhood[FLOCK_SIZE] = {0};

float theta_robots[FLOCK_SIZE];

/*
 * Reset the robot's devices and get its ID
 */
static void reset() 
{
	wb_robot_init();

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
//	printf("id = %d, x = %f, y = %f\n", robot_id, x, z);

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
//	printf("bearing = %f, u = %f, w = %f, msl = %f, msr = %f\n", bearing, u, w, msl, msr);

	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}


/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules() {  //DANIEL TO ADD
	int i, j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};
	
	/* Compute averages over the whole flock */
	for(i=0; i<FLOCK_SIZE; i++){
        if(i==robot_id) continue; 
            for (j=0;j<2;j++) {
                rel_avg_speed[j] += relative_speed[i][j];
                rel_avg_loc[j] += relative_pos[i][j] ;
            }
    }
           
    for (j=0;j<2;j++) {
        rel_avg_speed[j]/=FLOCK_SIZE-1;
        rel_avg_loc[j] /= FLOCK_SIZE-1; 
    }
	
	
	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */  
    for (j=0;j<2;j++) 
	{	
            cohesion[j] = rel_avg_loc[j];
            //printf("cohesion:%f\n", cohesion[j]);
	}

    /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
    for (k=0;k<FLOCK_SIZE;k++) {
        if (k != robot_id){ // Loop on flockmates only
            // If neighbor k is too close (Euclidean distance)
            if(pow(relative_pos[k][0],2)+pow(relative_pos[k][1],2) < RULE2_THRESHOLD){
            	for (j=0;j<2;j++) {
                	dispersion[j] -= 1/relative_pos[k][j]; // Relative distance to k
                //printf("dispersion:%f, ", dispersion[j]);
              	}
            }
        }
    }
          
	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
    for (j=0;j<2;j++) {
		consistency[j] = rel_avg_speed[j];
		//printf("consistency\n");
    }

    //aggregation of all behaviors with relative influence determined by weights
    for (j=0;j<2;j++) {
        speed[robot_id][j]  =  cohesion[j]    * RULE1_WEIGHT;
        speed[robot_id][j] +=  dispersion[j]  * RULE2_WEIGHT;
        speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
    }
    speed[robot_id][1] *= -1; //y axis of webots is inverted
        
    //move the robot according to some migration rule
    if(MIGRATORY_URGE == 0){
        speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
        speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
    }
    else {
        speed[robot_id][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT_X;
        speed[robot_id][1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT_Y; //y axis of webots is inverted
    }
}



/*	write_relpos(int other_robots, int loop_time) // added by Hugo (16.11.18)
 *
 *  write the relative_pos of the robot1's neighborhood in a file. -> Test if send_ping() works properly
 * 	Note: Need to be removed on real E-puck
*/
void sim_write_relpos(int other_robot_id,int loop_time){
	FILE* f;
	char fname[] = "../../../Simulation/robot_1.txt";			            // file name can be changed

	if(robot_id == 1){ // Write only for the robot1
  	printf("begin writing file\n");
	
	f = fopen(fname,"a");
	printf("writing file 1\n");

  	// Datas
    fprintf(f,"%d\t %d\t %f\t %f\t %f\t %f\n",loop_time,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],relative_pos[other_robot_id][2],(relative_pos[other_robot_id][2]/M_PI *180));

	printf("writing file 2\n"); 
	fclose(f);
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



/* update_neighborhood_states(float range, float theta, int other_robot_id, int loop_time) // added by Hugo in (16.11.18)
 * compute the relative position, speed and orientation of each neighbors.
 * update the previous position, orientation.
 * update the flag table of neighborhood.
 * write the relative distance of neighbors of robot 1. -> Need to set the flag WRITE_REL_POS = 1
*/
void update_neighborhood_states(float range, float theta, int other_robot_id, int loop_time){
	// Set Flag
	neighborhood[other_robot_id] = 1;
	// Update the prev position
	prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
	prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];
	prev_relative_pos[other_robot_id][2] = relative_pos[other_robot_id][2];
	// Set relative location
	relative_pos[other_robot_id][0] =  range * cosf(theta);
    relative_pos[other_robot_id][1] =  range * sinf(theta);
	relative_pos[other_robot_id][2] = theta;
	
	// writing function
	if(WRITE_REL_POS){
		sim_write_relpos(other_robot_id,loop_time);
	}
	  
	if(prev_neighborhood[other_robot_id]){ // Take only in account the previouse neighborhood
		// Set relative speed
		relative_speed[other_robot_id][0] = (relative_pos[other_robot_id][0] - prev_relative_pos[other_robot_id][0])/DELTA_T;
		relative_speed[other_robot_id][1] = (relative_pos[other_robot_id][1] - prev_relative_pos[other_robot_id][1])/DELTA_T;
	}
}

/* sim_receive_message(void) // added by Hugo (16.11.18)
 *
 * inspired by the process_received_ping_messages() function.
 * read the buffer and compute the relative state.
 * call update_neighborhood_states(range, theta, other_robot_id) to store the state
 * 
*/
void sim_receive_message(void)
{
    const double *message_direction;
    double message_rssi; 	// Received Signal Strength indicator
	double range;
	char* inbuffer;			// Buffer for the receiver node
    int other_robot_id;
    static int loop_time = 0;
	
	if(!loop_time && robot_id ==1){
	FILE* f;
	char fname[] = "../../../Simulation/robot_1.txt";			            // file name can be changed
	f = fopen(fname,"w"); // erase previous file
  	// File header
  	fprintf(f,"time_step\t neighbors_id\t x\t y\t theta\t deg_theta\t\n");
  	fclose(f);
  	}

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

		// since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
		other_robot_id = (int)(inbuffer[5]-'0'); 
    	range = sqrt((1/message_rssi));

		/*//Check all the angles
		double x = message_direction[0];
		double y = message_direction[1];
		double z = message_direction[2];
		
        double theta1 =	atan2(y,x)/M_PI *180;
        double theta2 =	atan2(z,x)/M_PI *180;
        double theta3 =	atan2(y,z)/M_PI *180;
                  
        if(robot_id ==1){
            printf("id: %d, x: %f, y: %f, z: %f, thetaXY: %f, thetaXZ: %f,thetaZY: %f\n",other_robot_id,x,y,z,theta1, theta2, theta3);
        }*/
        double x = message_direction[1];
		double y = message_direction[2];
        double theta = atan2(x,y);

        theta = (theta<0?2*M_PI+theta:theta); // theta between [0;2*PI];

		
		//Check if in IR physical range
		if(range<=MAX_COMMUNICATION_DIST){
			update_neighborhood_states(range, theta, other_robot_id, loop_time);
			// Increment the number of neighbors
		}
		
		wb_receiver_next_packet(receiver2);
	}
	
	if(robot_id == 1){loop_time++;}
	
}


void obstacle_avoidance(int* bmsl, int* bmsr, int* max_sens, int* sum_sensors)
{
    int distances[NB_SENSORS];      // Array for the distance sensor readings
	float to_keep[NB_SENSORS]; 		//flag of sensor to consider for computation
	
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



	/* Braitenberg */
	for(int i=0;i<NB_SENSORS;i++) 
	{

		distances[i] = wb_distance_sensor_get_value(ds[i]);           //Read sensor values

        *sum_sensors += distances[i] * to_keep[i] ;                  
        *max_sens = *max_sens > distances[i] ? *max_sens:distances[i]; // Check if new highest sensor value

        // Weighted sum of distance sensor values for Braitenburg vehicle
        *bmsr += e_puck_matrix[i] * distances[i] * to_keep[i];             // Add up sensor values ***IF*** in sensor to consider
        *bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i] * to_keep[i];
    }

	// Adapt Braitenberg values (empirical tests)
    *bmsl/=MIN_SENS + 66;
    *bmsr/=MIN_SENS + 72;
    
}

// the main function
int main(){ 
	int msl=0; 
	int msr=0;			// Wheel speeds
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens= 0;			// Store highest sensor value
	
	int t=0; //Mathilde
	char *message; //Mathilde

	/*******************************************************************/


 	reset();			// Resetting the robot

	
	// Forever
	for(;;){

		bmsl = 0; 
		bmsr = 0;
        sum_sensors = 0;
		max_sens = 0;

		if(OBSTACLE_AVOIDANCE){     
			obstacle_avoidance(&bmsl, &bmsr, &max_sens, &sum_sensors);
		}

		if(LOCAL_COMMUNICATION){
		/* Send and get information */
			sim_send_message();  		// sending a message to other robot in the neighborhood.
		}              

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		
		//update value of my_position
		update_self_motion(msl,msr);
		
		if(LOCAL_COMMUNICATION){
        	sim_receive_message();		// receiving a messages from other robots  in the neighborhood + update their local relative states in the current robot memory.
		}

		//absolute speed of robot
		speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
		speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);
    
		if(REYNOLDS){
			// Reynold's rules with all previous info (updates the speed[][] table)
			reynolds_rules();
		}
    
		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);
    
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
    
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;
                  
		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
        wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/
    
    	t += TIME_STEP; // Mathilde
		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}  
  
