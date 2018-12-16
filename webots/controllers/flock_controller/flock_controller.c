/*****************************************************************************/
/* File:     	flock_controller.c                                      */
/* Version:  	1.0                                                     */
/* Date:     	15-Dec-18                                               */
/* Description:  Reynolds flocking with relative local positions obtain via   */
/*               Ir comunication. This code is inspired from Reynolds.c of the Lab 4   	  	                      */
/*                                                                       	*/
/* Author:      Group 10 : DIS mini-Project  			*/
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>
/*Webots 2018b*/
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

//-------------------------------------------------------------------------------------------------//
			//Variables, parameters & macro//
//-------------------------------------------------------------------------------------------------//

#define NB_SENSORS    	8   	// Number of distance sensors
#define MIN_SENS      	350 	// Minimum sensibility value
#define MAX_SENS      	4096	// Maximum sensibility value
#define MAX_SPEED_BMR     	700 	// Maximum speed
#define JOIN_SPEED    100        // Speed added in join when join mode (returning to migr_diff = 0)
#define NO_REYN_BIAS 200      // Difference in saturation speed if robot is alone
#define JOIN_BIAS      0.5
#define FLOCK_SIZE     5   	// Size of flock

#define TIME_STEP             64        // [ms] Duration of a time step
#define AXLE_LENGTH            0.052    // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS        0.00628  // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS           0.0205   // Wheel radius (meters)
#define DELTA_T   	         0.064    // Timestep (seconds)
#define MAX_SPEED_WEB 	         6.28     // Maximum speed webots
#define MAX_COMMUNICATION_DIST 0.25     // Maximum communication range (meters)
// Flags for the different part of the controller
#define MIGRATORY_URGE 1   // Tells the robots if they should just go forward or move towards a specific migratory direction
#define REYNOLDS 1         // Tells the robots to use the 3 Reynolds rules for forming a flock
#define BRAITENBERG 1      // Tells the robots to use a Braitenberg architecture for obstacle avoidance
#define JOIN 1             // Tells the robots to add a speed to join the flock after an obstacle avoidance
// Macros
#define ABS(x) ((x>=0)?(x):-(x))
// Debug
#define ROBOT_DEBUG  3      // which robot's information to filter out

// Reynolds threshold and weights
float RULE1_THRESHOLD = 0.1;  	// Threshold to activate aggregation rule
float RULE2_THRESHOLD = 0.12;  	// Threshold to activate dispersion rule
float RULE1_WEIGHT; float RULE2_WEIGHT; float RULE3_WEIGHT; float REYN_MIGR_RATIO; float WEIGHT_JOIN; 

// Braitenberg parameters
#define BRAITENBERG_LOWER_THRESH 350  // Below this value, no avoidance
#define BRAITENBERG_UPPER_THRESH 2000 // Normalisation factor
int e_puck_matrix[16] =   {16,9,6,3, -1,-5,-6,-8,   -12,-8,-5,6, 2,4,5,9}; // for obstacle avoidance giving more weigth (asymetry) for the right side

// 
#define MAX_TIME_ALONE 10     // Nomber maximum of step alone, then deactivate reynold
int join_speed = JOIN_SPEED;         // Speed allocated for the join mode
int max_speed_bmr = MAX_SPEED_BMR;      // Speed allocated for the braitenberg + migration + reynold mode

/*Webots 2018b*/
WbDeviceTag left_motor;     // Handle for left wheel of the robot
WbDeviceTag right_motor;    // Handle for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS]; // Handle for the infrared distance sensors
WbDeviceTag receiver2;      // Handle for the receiver node 2 (Communication between the E-pucks)
WbDeviceTag emitter2;       // Handle for the emitter node 2 (Communication between the E-pucks)
WbDeviceTag receiver3;      // Handle for the emitter node 3 (Communication between an E-pucks and a Supervisor for the optimization)

// Robot id, group and name
char* robot_name;
int robot_id_u, robot_id;    // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
int robot_group;

// Robot's states
float my_position[3];			 // X, Z, Theta of the current robot
float prev_my_position[3]; 		 // X, Z, Theta of the current robot in the previous time step
float relative_pos[FLOCK_SIZE][3];           // relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];      // Previous relative  X, Z, Theta values
float prev_global_pos[FLOCK_SIZE][2];        // Previous global  X, Z values
float speed[FLOCK_SIZE][2];   	            // Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];         // Speeds calculated with Reynold's rules

// Other
bool flockmates[FLOCK_SIZE] = {0};           // Table of flag: 1- is flockmate ; 0- is not flockmate
int n_flockmates = 0;                        // Number of flockmate
bool no_reynolds = 0;                        // flag to deactivate reynolds: 1- reynold deactivated ; 0- reynold activated
float migr_diff = 0;                         // Integrator of the wheels difference during the obstacle avoidance state
float migr[2];                               // Migration Urge vector

//-------------------------------------------------------------------------------------------------//
			       //Functions//
//-------------------------------------------------------------------------------------------------//


static void reset() {
/*
 * Reset()
 * Initialisation of the robots and get the different weigths from the supervisor.
 */
    int i;
    char *inbuffer;
// Robot initialisation in Webots
    wb_robot_init();
// Initialisation for the communication
    receiver2 = wb_robot_get_device("receiver2"); // for the local Ir communication
    wb_receiver_enable(receiver2, 64);            // enable the receiver 2
    emitter2 = wb_robot_get_device("emitter2");   // for the local Ir communication
    receiver3 = wb_robot_get_device("receiver3"); // for the communication with the supervisor, used to set the different weights
                                                  // at the initialisation and to test different weight during the optimization step
    wb_receiver_enable(receiver3, 64);            // enable the receiver 3
// Initialisation of the motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
// Initiali	sation of the sensors 
    char s[4]="ps0";
    for(i=0; i<NB_SENSORS;i++) {
   	 ds[i] = wb_robot_get_device(s);      // the device name is specified in the world file
   	 wb_distance_sensor_enable(ds[i],64); // enable the sensor devices
   	 s[2]++;   			     // increases the device number
    }
// Initialisation of the robot id and group
    robot_name = (char*) wb_robot_get_name();    // Get the robot name
    sscanf(robot_name,"epuck%d",&robot_id_u);    // read robot id from the robot's name !Only work with id := [0;9]!
    robot_id = robot_id_u % FLOCK_SIZE;          // normalize between 0 and FLOCK_SIZE-1
    robot_group = (robot_id_u / FLOCK_SIZE);     // group ID needed for scenario 2
// Get the weights from the supervisor
    while(wb_receiver_get_queue_length(receiver3) == 0){wb_robot_step(TIME_STEP);}
    while(wb_receiver_get_queue_length(receiver3) > 0){ 
       inbuffer = (char*) wb_receiver_get_data(receiver3);
       sscanf(inbuffer,"%f#%f##%f#%f#%f#%f#%f\n", &migr[0], &migr[1], &RULE1_WEIGHT,&RULE2_WEIGHT,&RULE3_WEIGHT,&REYN_MIGR_RATIO,&WEIGHT_JOIN);
       // Print the weigths in the console
       if(robot_id == ROBOT_DEBUG){ 
           printf("Received from supervisor [%.3f][%.3f][%.3f][%.3f] and migr [%.3f][%.3f] and weight join [%.3f]\n",RULE1_WEIGHT,RULE2_WEIGHT,RULE3_WEIGHT,REYN_MIGR_RATIO, migr[0], migr[1],WEIGHT_JOIN); 
       }
       wb_receiver_next_packet(receiver3);
    }
// Set the reynold to migration ratio according to the robot id
    REYN_MIGR_RATIO += (robot_id+1.)*0.4; // Adding heterogenity (pseudo leader based)

}

void update_self_motion(int msl, int msr) {
/*
 * update_self_motion()
 * Update the robot's states thanks to Odometry.
 */
    float theta = my_position[2];
 
// Compute deltas of the robot
    float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
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
    float bearing = range == 0 ? 0.0 : atan2(y, x);         // Orientation of the wanted position
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

void sim_send_message(void)  {
/*
 *  sim_send_message()
 *  Each robot sends a ping message, so the other robots can measure the relative range and bearing to the sender.
 *  The message contains the robot's id and group.
*/
    char message[10];
    sprintf(message,"%d#%d", robot_group, robot_id);
    wb_emitter_send(emitter2, message, strlen(message)+1);
}

void sim_receive_message(void)
{
/*
 * sim_receive_message()
 * Processing all the received ping messages, and calculate range and bearing to the other robots.
 * Extract the robot's id and group from the received message.
*/
    const double *message_direction;
    double message_rssi;   // Received Signal Strength indicator
    double theta;
    double range;
    char *inbuffer;        // Buffer for the receiver node
    int other_robot_id;    // Id of the robot that has sent the message
    int other_robot_group; // Group of the robot that has sent the message

    while (wb_receiver_get_queue_length(receiver2) > 0) {
   	 inbuffer = (char*) wb_receiver_get_data(receiver2);
   	 message_direction = wb_receiver_get_emitter_direction(receiver2);
   	 message_rssi = wb_receiver_get_signal_strength(receiver2);
   	 // Parse message
   	 sscanf(inbuffer,"%d#%d", &other_robot_group, &other_robot_id);
   	 // Check if message is my own (caused by reflection) and if it belongs to my group
   	 if((other_robot_id != robot_id) && (other_robot_group == robot_group)){
           	    double x = - message_direction[2]; // Extract the direction of the message and store it with respect to the axis of the robot
   	    double y = - message_direction[1];
   	 // Compute the angle
               theta = atan2(y,x);                
            // Keep the angle between [0,2*pi]
               theta = (theta < 0      ? theta + 2*M_PI : theta); // case bellow 0 Rad
               theta = (theta > 2*M_PI ? theta - 2*M_PI : theta); // case above 2*PI Rad
            // Compute the distance
   	  range = sqrt((1/message_rssi));
   	 // Transforme the previous selative position in global ref. into the relative one
             global2rel(prev_global_pos[other_robot_id], prev_relative_pos[other_robot_id]);   		   		 
            // Set current relative location and orientation
   	  relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
   	  relative_pos[other_robot_id][1] = range*sin(theta);  // relative y pos
   	  relative_pos[other_robot_id][2] = theta;             // relative orientation  		    	      	      
            // Compute speed in relative coordinates
   	  relative_speed[other_robot_id][0] = (relative_pos[other_robot_id][0] - prev_relative_pos[other_robot_id][0])/DELTA_T;
             relative_speed[other_robot_id][1] = (relative_pos[other_robot_id][1] - prev_relative_pos[other_robot_id][1])/DELTA_T;
            // Transforme the current position in the global referentiel and store it in the previous global vector.
             rel2global(relative_pos[other_robot_id],  prev_global_pos[other_robot_id]);	
   	  // if range < 0.25 m , consider it as part of my flock
   	 if(range < MAX_COMMUNICATION_DIST){
   	   flockmates[other_robot_id % FLOCK_SIZE] = 1;
   	 }
   	}
   	wb_receiver_next_packet(receiver2);
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
	   *wb = 0.0;    
    if(n_flockmates > 0){                                       // Ignore noisy sensors
      //*wj = 0.8 - 0.8*(n_flockmates/(FLOCK_SIZE - 1));             // Join weights decreases with number of flockmates
      *wj = 0.6;
    }
    else{
      *wj = 1;
    }
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
int check_if_alone(int msl, int msr, int alone) {
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
//-------------------------------------------------------------------------------------------------//
				//Main//
//-------------------------------------------------------------------------------------------------//
 
int main(){
	// Loop parameters
    int msl = 0; int msr = 0;   	                      // Wheel speeds:
    int rmsl, rmsr;				// 	r: for Reynold
    int mmsl, mmsr;				// 	m: for migratory urge
    int jmsl, jmsr;				// 	j: for joint
    int bmsl, bmsr;				// 	b: for braitenberg
    float msl_w, msr_w;			//	w: webots

    int distances[NB_SENSORS];   	                      // Array for the distance sensor readings
    int sum_sensors; 				// Sum of the sensors value
    int max_sens;   				// Store highest sensor value
    float wb, wm, wr, wj;                              // Weights for the different speeds.

    int alone = 0;       	                      // Sum of the not_moving time step due to REYN_MIGR_RATIO	
	
// Step 0 : RESET
    reset();   				// Resetting the robot
    
// Step 1: INFINITY LOOP
    for(;;){
// Loop step 0 : INITIALISATION
   	 for(int i=0;i<FLOCK_SIZE;i++){flockmates[i] = 0;}    	 // Reset flockmates list. It's populated by sim_receive_message
   	 rmsl = 0; rmsr = 0;
   	 bmsl = 0; bmsr = 0;
   	 mmsl = 0; mmsr = 0;
   	 jmsl = 0; jmsr = 0;

   	 sum_sensors = 0;
   	 max_sens = 0;
	    
	// Loop step 1 : BRAITENBERG
   	 if (BRAITENBERG){
   		 
   		 /* Braitenberg */
   		 for(int i=0;i<NB_SENSORS;i++)
   		 {
   			 distances[i] = wb_distance_sensor_get_value(ds[i]); 			//Read sensor values
   			 sum_sensors += distances[i]; 									// Add up sensor values
   			 max_sens = (max_sens > distances[i] ? max_sens : distances[i]); 	// Check if new highest sensor value
			 
   			 // Weighted sum of distance sensor values for Braitenburg vehicle
   			 bmsr += e_puck_matrix[i] * distances[i];
   			 bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
  		 }
  		 normalize_speed(&bmsr, &bmsl,max_speed_bmr); // Set the wheel_speed on bmsl and bmsr
    	}    

// Loop step 2 : SENDING MESSAGE
   	// Send and get information
   	sim_send_message(); // sending a ping to other robot, so they can measure their distance to this robot
   	
// Loop step 3 : RECEIVE MESSAGE
   	 sim_receive_message(); // reading messages from other robot and update their position
 
// Loop step 4 : REYNOLDS
   	 if(REYNOLDS){
   		 // Reynold's rules with all previous info (updates the speed[][] table)
   		 reynolds_rules();   			 
   		 // Compute wheels speed from reynold's speed
   		 compute_wheel_speeds(&rmsl, &rmsr); // Set the wheel_speed on rmsl and rmsr
   	 }

// Loop step 5 : MIGRATORY_URGE
   	 if(MIGRATORY_URGE){
   		migration_urge(); // Set the speed direction to the migratory urge, from the local ref to the relative at time t
   		compute_wheel_speeds(&mmsl, &mmsr); // Set the wheel_speed on mmsl and mmsr
   	 }
	 
// Loop step 6 : JOIN
   	 if(JOIN) {
		//jmsl =  -migr_diff/2;
		//jmsr =   migr_diff/2;
		if((my_position[2]<M_PI * 0.85/2 && my_position[2] > 0.0) || (my_position[2] < 2*M_PI && my_position[2] > 31*M_PI/20)){
            		jmsl = my_position[1]*WEIGHT_JOIN/2; 
            		jmsr = -my_position[1]*WEIGHT_JOIN/2; 
		}
		else{
            		jmsr = 0; 
            		jmsl = 0; 
		}	
		normalize_speed(&jmsr, &jmsl, JOIN_SPEED);
   	 }   	 
   	 
// Loop step 7 : COMPUTE THE FINAL SPEED
            set_weights(max_sens,&wb,&wm,&wr,&wj); 
            msl = wb * bmsl + wm * mmsl + wr * rmsl + wj * jmsl; 
            msr = wb * bmsr + wm * mmsr + wr * rmsr + wj * jmsr; 
	 //msl = set_final_speed(bmsl,  rmsl,  mmsl, jmsl, max_sens);     
	 //msr = set_final_speed(bmsr,  rmsr,  mmsr, jmsr, max_sens);
          	 alone = check_if_alone(msl, msr, alone);

// Loop step 8 : SET THE FINAL SPEED ON THE MOTORS
   	 msl_w = msl*MAX_SPEED_WEB/1000;	// Transforme the wheel speed for the real E-puck in webots ones
   	 msr_w = msr*MAX_SPEED_WEB/1000;
   	
   	 wb_motor_set_velocity(left_motor,  msl_w); // Apply the speed on webots
   	 wb_motor_set_velocity(right_motor, msr_w);
	
// Loop step 9 : UPDATE ROBOT STATES	
   	 update_self_motion(msl,msr); 			// Update the robot states according to the odometry
	
	 migr_diff += msl - msr;				// Update the difference of velocity between the wheels

// Next simulation step
   	 wb_robot_step(TIME_STEP);
   	 
    }
}   
 


