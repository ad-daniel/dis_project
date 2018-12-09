/*****************************************************************************/
/* File:     	raynolds2.c                                             	*/
/* Version:  	2.0                                                     	*/
/* Date:     	06-Oct-15                                               	*/
/* Description:  Reynolds flocking with relative positions   	  	*/
/*                                                                       	*/
/* Author:      06-Oct-15 by Ali Marjovi   			  	*/
/* Last revision:12-Oct-15 by Florian Maushart   			  	*/
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/compass.h>

#define NB_SENSORS    	8   	   // Number of distance sensors
#define MIN_SENS      	350 	// Minimum sensibility value
#define MAX_SENS      	4096	// Maximum sensibility value
#define MAX_SPEED     	800 	// Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB 	6.28	// Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE     5   	   // Size of flock
#define TIME_STEP      64      // [ms] Length of time step

#define AXLE_LENGTH      0.052   	 // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS  0.00628    // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS     0.0205    // Wheel radius (meters)
#define DELTA_T   	   0.064   	 // Timestep (seconds)

#define MAX_COMMUNICATION_DIST 0.25

float RULE1_THRESHOLD = 0.1;  	   // Threshold to activate aggregation rule. default 0.201
//float RULE1_WEIGHT	= 5;  // Weight of aggregation rule. default 0.6/10

float RULE2_THRESHOLD = 0.12;  	// Threshold to activate dispersion rule. default 0.15
//float RULE2_WEIGHT	= 5.1; // Weight of dispersion rule. default 0.02/10

//float RULE3_WEIGHT	= 0;  // Weight of consistency rule. default 1.0/10
float RULE1_WEIGHT; float RULE2_WEIGHT; float RULE3_WEIGHT; float weightX; float weightY;

#define MIGRATORY_URGE 1    		   // Tells the robots if they should just go forward or move towards a specific migratory direction
#define REYNOLDS 1
#define BRAITENBERG 1

#define ABS(x) ((x>=0)?(x):-(x))

#define VERBOSE    	 0
#define VERBOSE_2  	 0
#define VERBOSE_3  	 0
#define VERBOSE_4  	 1
#define VERBOSE_M      1
#define ROBOT_DEBUG  0 // which robot's information to filter out
/*Added by Pauline for weights*/
#define INTIALISATION_STEPS  3000  //30 secondes at the beginning to form flock and align with migration (before infinite loop)
#define REYN_MIGR_RATIO  	0.5 // TO TUNE: ratio of weights between Reynolds and Migration urge  	 
#define BRAITENBERG_LOWER_THRESH 350 // below this value, no avoidance
#define BRAITENBERG_UPPER_THRESH 2000
#define BRAITENBERG_SPEED_BIAS 300

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

//int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,   -72,-58,-36,8,10,36,28,18}; // for obstacle avoidance
// Changed again in much higher weights
//int e_puck_matrix[16] =   {130,90,60,22,8,-48,-56,-76,   -112,-85,-70,6,20,36,38,28}; // for obstacle avoidance giving priority to a side
int e_puck_matrix[16] =   {16,9,6,3, -1,-5,-6,-8,   -12,-8,-5,6, 2,4,5,9}; // for obstacle avoidance giving priority to a side

WbDeviceTag ds[NB_SENSORS];    // Handle for the infrared distance sensors
WbDeviceTag receiver2;   	 // Handle for the receiver node
WbDeviceTag emitter2;   	 // Handle for the emitter node
WbDeviceTag compass;
WbDeviceTag receiver3;

int robot_id_u, robot_id;    // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
int robot_group;

float relative_pos[FLOCK_SIZE][3];    // relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];    // Previous relative  X, Z, Theta values
float prev_global_pos[FLOCK_SIZE][2];    // Previous global  X, Z values

float my_position[3];			 // X, Z, Theta of the current robot
float prev_my_position[3]; 		 // X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];   	 // Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];    // Speeds calculated with Reynold's rules

float migr[2]; // !!! CHANGE IT IN THE SUPER, NOT HERE !!!

char* robot_name;
// tracks which are my flockmates. Myself excluded.
// 1 : is my flockmate and 0 : isn't my flockmate
bool flockmates[FLOCK_SIZE] = {0};


/*
 * Reset the robot's devices and get its ID
 */
static void reset()
{
    int i;

    char *inbuffer; //Mathilde

    wb_robot_init();

    compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, 64);

    receiver2 = wb_robot_get_device("receiver2");
    emitter2 = wb_robot_get_device("emitter2");
    receiver3 = wb_robot_get_device("receiver3"); //Mathilde
    
    //get motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
   	 
    char s[4]="ps0";
    for(i=0; i<NB_SENSORS;i++) {
   	 ds[i] = wb_robot_get_device(s);    // the device name is specified in the world file
   	 s[2]++;   			 // increases the device number
    }
    robot_name = (char*) wb_robot_get_name();

    for(i=0;i<NB_SENSORS;i++){
   	   wb_distance_sensor_enable(ds[i],64);
    }

    wb_receiver_enable(receiver2, 64);
    //Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
    sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
    robot_id = robot_id_u % FLOCK_SIZE;       // normalize between 0 and FLOCK_SIZE-1
    robot_group = (robot_id_u / FLOCK_SIZE);  // group ID needed for scenario 2

    wb_receiver_enable(receiver3, 64); //Mathilde

	float weightX; float weightY; 

	while(wb_receiver_get_queue_length(receiver3) == 0){wb_robot_step(TIME_STEP);}

	while(wb_receiver_get_queue_length(receiver3) > 0){ 
	 inbuffer = (char*) wb_receiver_get_data(receiver3);
	 sscanf(inbuffer,"%f#%f##%f#%f#%f#%f#%f\n", &migr[0], &migr[1], &RULE1_WEIGHT,&RULE2_WEIGHT,&RULE3_WEIGHT,&weightX,&weightY);
	 
	 if(VERBOSE_M && robot_id == ROBOT_DEBUG){ printf("Received from supervisor [%.3f][%.3f][%.3f][%.3f][%.3f] and migr [%.3f][%.3f]\n",RULE1_WEIGHT,RULE2_WEIGHT,RULE3_WEIGHT,weightX,weightY, migr[0], migr[1]); }
	 //printf("initializing\n");
	 wb_receiver_next_packet(receiver3);
	}
	//printf("Received from supervisor [%.3f][%.3f][%.3f][%.3f][%.3f] and migr [%.3f][%.3f]\n",RULE1_WEIGHT,RULE2_WEIGHT,RULE3_WEIGHT,weightX,weightY, migr[0], migr[1]);


    
    if(VERBOSE_4 && robot_id == ROBOT_DEBUG){printf("Reset robot %d :: [id: %d][group: %d]\n",robot_id_u, robot_id, robot_group);}
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
/*
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
    if (my_position[2] > 2*M_PI){
   	 my_position[2] -= 2.0*M_PI;
    }
    if (my_position[2] < 0){
   	 my_position[2] += 2.0*M_PI;
    }
}
*/
void update_self_motion(int msl, int msr) {

    float theta = my_position[2];
 
    // Compute deltas of the robot
    float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    float du = (dr + dl)/2.0;
    float dtheta = (dr - dl)/AXLE_LENGTH;
	if(robot_id == 0 && VERBOSE){printf("dr %f dl %f du %f dtheta %f\n",dr,dl,du,dtheta);}
    
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

/*
 * Computes wheel speed given a certain X,Z speed
 */
/*
void compute_wheel_speeds(int *msl, int *msr)
{
    float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
    float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates

    float Ku = 0.2;   // Forward control coefficient
    float Kw = 1;  // Rotational control coefficient
    float range = sqrtf(x*x + z*z);      // Distance to the wanted position
    float bearing = -atan2(x, z);      // Orientation of the wanted position
    
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
*/

void normalize_speed(int *msl, int *msr) {
    if(ABS(*msl) > MAX_SPEED || ABS(*msr) > MAX_SPEED){
      float max = (ABS(*msl)>ABS(*msr)?ABS(*msl):ABS(*msr));
      *msl = *msl/max * MAX_SPEED;
      *msr = *msr/max * MAX_SPEED;
    }
}

void compute_wheel_speeds(int *msl, int *msr)
{
    // Compute wanted position from Reynold's speed and current location
    //float x = speed[robot_id][0]*cosf(-my_position[2]) - speed[robot_id][1]*sinf(-my_position[2]); // x in robot coordinates
    //float z = speed[robot_id][0]*sinf(-my_position[2]) + speed[robot_id][1]*cosf(-my_position[2]); // z in robot coordinates
    	float x = speed[robot_id][0];
    	float z = speed[robot_id][1];
   	 
	if(robot_id == 0 && VERBOSE){printf("[x] : %f, [y] : %f [theta] : %f\n",x,z,my_position[2]);}
    
    float Ku = 0.02;   // Forward control coefficient
    float Kw = 0.08;  // Rotational control coefficient
    float range = sqrtf(x*x + z*z);      // Distance to the wanted position
    float bearing = atan2(z, x);      // Orientation of the wanted position
    // Compute forward control
    float u = Ku*range;//*cosf(bearing);
    // Compute rotational control
    float w = Kw*bearing;
    if(robot_id == 0 && VERBOSE){printf("[u] : %f, [w] : %f [range] : %f\n",u,w,range);}

    // Convert to wheel speeds!
    *msl += (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    *msr += (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    if(robot_id == 0 && VERBOSE){printf("[msr] : %d, [msl] : %d \n",*msr,*msl);}
    
    // Avoid saturation
    
    normalize_speed(msl, msr);
    
    if(robot_id == 0 && VERBOSE){printf("[msr] : %d, [msl] : %d \n",*msr,*msl);}

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
    //float migr_rel[2] = {0};
    global2rel(migr,speed[robot_id]);  
}


/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules() {
    int j, k;   					 // Loop counters
    float rel_avg_loc[2] = {0,0};    // Flock average positions
    float rel_avg_speed[2] = {0,0};    // Flock average speeds
    float cohesion[2] = {0,0};
    float dispersion[2] = {0,0};
    float consistency[2] = {0,0};
    
    int n_flockmates = 0;
    float dist = 0.0;
   	 
    //Added by Pauline
    speed[robot_id][0] = 0;
    speed[robot_id][1] = 0;
    
    // Compute averages over the whole flock
    for(k=0;k<FLOCK_SIZE;k++){
   	 if(k == robot_id){
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
    
    if(VERBOSE && ROBOT_DEBUG == robot_id){ printf("R0 rel_avg_loc [%f][%f] rel_avg_speed [%f][%f]\n", rel_avg_loc[0], rel_avg_loc[1], rel_avg_speed[0], rel_avg_speed[1]); }


    // Rule 1 - Aggregation/Cohesion: move towards the center of mass
    dist = sqrt(rel_avg_loc[0]*rel_avg_loc[0] + rel_avg_loc[1]*rel_avg_loc[1]);
    if(dist > RULE1_THRESHOLD){
   	 for (j=0;j<2;j++){    
   		 cohesion[j] = rel_avg_loc[j];
   	 }
    }
    if(VERBOSE && ROBOT_DEBUG == robot_id){ printf("R1 dist [%f] :: coehsion [%f][%f]\n", dist, cohesion[0], cohesion[1]); }

    // Rule 2 - Dispersion/Separation: keep far enough from flockmates
    if(VERBOSE && ROBOT_DEBUG == robot_id){ printf("R2 distances :: "); }

    for(k=0;k<FLOCK_SIZE;k++){
   	 if(k == robot_id){
   		 continue;
   	 }

   	 dist = sqrt(relative_pos[k][0]*relative_pos[k][0] + relative_pos[k][1]*relative_pos[k][1]);
   	 if(VERBOSE && ROBOT_DEBUG == robot_id){ printf("[%d : %f]", k, dist); }

   	 if(dist < RULE2_THRESHOLD && flockmates[k]){
   		 for (j=0;j<2;j++) {
   			 dispersion[j] -= (relative_pos[k][j]);
   		 }
   	 }
    }    
    if(VERBOSE && ROBOT_DEBUG == robot_id){ printf("\nR2 dispersion [%f][%f]\n", dispersion[0], dispersion[1]); }
 
    // Rule 3 - Consistency/Alignment: match the speeds of flockmates
    for (j=0;j<2;j++){
   	 consistency[j] = rel_avg_speed[j];
   	if(VERBOSE && ROBOT_DEBUG == robot_id){ printf("R3 consistency [%f][%f]\n", consistency[0], consistency[1]); }
    }
    /*
    if(VERBOSE_2 && robot_id == 0) { 
          printf("Consistency of robot 0 is [%f][%f]\n", consistency[0], consistency[1]); 
    }
    if(VERBOSE_2 && robot_id == 1) { 
          printf("Consistency of robot 1 is [%f][%f]\n", consistency[0], consistency[1]); 
    }*/

    //aggregation of all behaviors with relative influence determined by weights
    for (j=0;j<2;j++){
   	 speed[robot_id][j] +=  cohesion[j]    * RULE1_WEIGHT;
   	 speed[robot_id][j] +=  dispersion[j]  * RULE2_WEIGHT;
   	 speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
   	 if(VERBOSE && ROBOT_DEBUG == robot_id){ printf("ACC  [%1.7f][%1.7f][%1.7f]\n", cohesion[j] * RULE1_WEIGHT, dispersion[j] * RULE2_WEIGHT, consistency[j] * RULE3_WEIGHT); }

    }
}

/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void sim_send_message(void)  
{
    char message[10];
    sprintf(message,"%d#%d", robot_group, robot_id);
    wb_emitter_send(emitter2, message, strlen(message)+1);
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void sim_receive_message(void)
{
    const double *message_direction;
    double message_rssi; // Received Signal Strength indicator
    double theta;
    double range;
    char *inbuffer;    // Buffer for the receiver node
    int other_robot_id;
    int other_robot_group;

    while (wb_receiver_get_queue_length(receiver2) > 0) {
   	 inbuffer = (char*) wb_receiver_get_data(receiver2);
   	 message_direction = wb_receiver_get_emitter_direction(receiver2);
   	 message_rssi = wb_receiver_get_signal_strength(receiver2);
   	 // parse message
   	 sscanf(inbuffer,"%d#%d", &other_robot_group, &other_robot_id);

   	 // check if message is my own (caused by reflection) and if it belongs to my group
   	 if((other_robot_id != robot_id) && (other_robot_group == robot_group)){
   	  double x = - message_direction[2]; // Changed by Hugo (20.11.18)
   	  double y = - message_direction[1]; // Changed by Hugo (20.11.18)

      theta = atan2(y,x); // changed by Hugo (20.11.18)
      theta =(theta<0? theta+2*M_PI:theta);// theta between [0,2*pi]
      theta = (theta>2*M_PI? theta-2*M_PI:theta);
             	 
   	  range = sqrt((1/message_rssi));
   	 //if(robot_id == 0 && VERBOSE_2){printf("[id]: %d [x]: %f [y]: %f [theta]: %f\n",other_robot_id,range*cos(theta),range*sin(theta),theta/M_PI*180);}
   		 
   	            // Previous in local 
      global2rel(prev_global_pos[other_robot_id], prev_relative_pos[robot_id]);
   		   		 
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
   		
   		
   		
   	  // if range < 0.25, consider it as part of my flock
   	  if(range < MAX_COMMUNICATION_DIST){
   	   flockmates[other_robot_id % FLOCK_SIZE] = 1;
   	  }
   	}

   	 wb_receiver_next_packet(receiver2);
    }

    if(VERBOSE && (ROBOT_DEBUG == robot_id)){
   	 printf("[%d] of ID [%d] has flockmates ", robot_id, robot_group);
   	 for(int i = 0;i<FLOCK_SIZE;i++){
   		 if(flockmates[i]){
   			 printf("[%d]", i);
   		 }
   	 }
   	 printf("\n");
    }
}

double get_bearing(WbDeviceTag tag) {
    const double *north = wb_compass_get_values(tag);
    double bearing = atan2(north[1], north[2]);

    if (bearing < 0.0){
   	 bearing = bearing + 2*M_PI;
    }
    
    return bearing;
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
     	 }
     	 else{
     		 wb = 0.0;
     	 }
     	 

   	 wm = (1 - wb) / (float)(1 + REYN_MIGR_RATIO);
   	 wr = REYN_MIGR_RATIO *  wm;
     	 
    }
    if(VERBOSE_3) {     
   	 printf("[w_b_s]: %f [w_m_s]: %f [w_r_s]: %f\n", wb * b_speed, wm * m_speed,wr * r_speed);
     }
    if(ROBOT_DEBUG == robot_id && VERBOSE_3){printf("[wb] [%f] :: [wm] [%f] :: [wr] [%f]\n", wb, wm, wr);}

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
    float msl_w, msr_w;
    int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
    int distances[NB_SENSORS];   	 // Array for the distance sensor readings
    int max_sens;   				 // Store highest sensor value
    //float wb, wm, wr;              	 
    reset();   					 // Resetting the robot

    msl = 0; msr = 0;
    
    /* Added by Pauline
    //First, e-puck form a Flock: implement when main is already done
    
    for(int i=0; i<INTIALISATION_STEPS; i++) {
          	//do as in loop of main with only reynolds and migration
          	//just set  max_sens = 0 in set_final_speed 	//don't care about obstacles, try to get a flock aligned with migration
    }
    */
    
    int count=0;
    // Forever
    for(;;){
    
    
   	 if(VERBOSE && ROBOT_DEBUG == robot_id){ printf("\n"); }

   	 // reset flockmates list. It's populated by sim_receive_message
   	 for(i=0;i<FLOCK_SIZE;i++){flockmates[i] = 0;}
   	 rmsl = 0; rmsr = 0;
   	 bmsl = 0; bmsr = 0;
   	 mmsl = 0; mmsr = 0;

   	 sum_sensors = 0;
   	 max_sens = 0;
   			 
   	 if (BRAITENBERG){
   		 
   		 /* Braitenberg */
   		 for(i=0;i<NB_SENSORS;i++)
   		 {
   			 distances[i] = wb_distance_sensor_get_value(ds[i]); //Read sensor values
   			 sum_sensors += distances[i]; // Add up sensor values
   			 max_sens = max_sens > distances[i] ? max_sens : distances[i]; // Check if new highest sensor value

   			 // Weighted sum of distance sensor values for Braitenburg vehicle
   			 bmsr += e_puck_matrix[i] * distances[i];
   			 bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
  		 }
  		 normalize_speed(&bmsr, &bmsl);
  		 //Il faudrait prendre la différence des 2 et normaliser 
                      
   		 // Adapt Braitenberg values (empirical tests)
   		 //bmsl/=MIN_SENS; bmsr/=MIN_SENS;
   		 //bmsl+=66; bmsr+=72;

   		 // add speed bias to ensure we are moving rather fast during avoidance
   		 //bmsl+=400; bmsr +=400;

   		 //bmsl = bmsl + BRAITENBERG_SPEED_BIAS < MAX_SPEED ? bmsl + BRAITENBERG_SPEED_BIAS : bmsl;
   		 //bmsr = bmsr + BRAITENBERG_SPEED_BIAS < MAX_SPEED ? bmsr + BRAITENBERG_SPEED_BIAS : bmsr;

    	}    

   	 // Send and get information
   	 sim_send_message(); // sending a ping to other robot, so they can measure their distance to this robot

   	 /// Compute self position
   	 prev_my_position[0] = my_position[0];
   	 prev_my_position[1] = my_position[1];
   			 
   	 sim_receive_message();

   	 //speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
   	 //speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);
    
   	 if(REYNOLDS){
   		 // Reynold's rules with all previous info (updates the speed[][] table)
   		 reynolds_rules();   			 
   		 // Compute wheels speed from reynold's speed
   		 compute_wheel_speeds(&rmsl, &rmsr);
   	 }

   	 if(MIGRATORY_URGE){
               //attention on a remis à 0 dans reynolds et braitenberg!!!!!!!!!!!!!!!!!!!!!
   		 migration_urge();
   		 if(robot_id == 4){
   		 float angle = atan2(speed[robot_id][1],speed[robot_id][0]); 
     		 if(VERBOSE){printf("angle = %f\n",angle*180/(M_PI));}
   		 }
   		 compute_wheel_speeds(&mmsl, &mmsr);

   	 }   	 
   	 
    	/* Added by Pauline*/
   	 // Set final speed
   	 if(count < 100) {
   	 //printf("ONLY REYNOLDS");
               msl = rmsl;
               msr = rmsr;
   	 } else {
             //printf("ALL IS ON");
              msl = set_final_speed(bmsl,  rmsl,  mmsl,  max_sens);
              msr = set_final_speed(bmsr,  rmsr,  mmsr, max_sens);
   	 }
   	 /*
   	 if(robot_id == 0) {
   	       msr = 0; msl = 0;
   	 }
   	 */

   	 if(VERBOSE) {
   	 printf("-----------------------------------------------\n");
   	 printf("[bmsl]: %d [mmsl]: %d [rmsl]: %d\n",bmsl,mmsl,rmsl);      
   	 printf("[bmsr]: %d [mmsr]: %d [rmsr]: %d\n",bmsr,mmsr,rmsr);
   	 printf("<<<<<<<<<<<>>>>>>>>>>>>>>\n");
       		 printf("[msl]: %d [msr] %d\n",msl,msr);
            }
   	 //printf("-----------------------------------------------\n");

   	 // Set speed
   	 msl_w = msl*MAX_SPEED_WEB/1000;
   	 msr_w = msr*MAX_SPEED_WEB/1000;
   	 
   	 
   	 wb_motor_set_velocity(left_motor, msl_w);
   	 wb_motor_set_velocity(right_motor, msr_w);

   	 update_self_motion(msl,msr);

    
   	 // Continue one step
   	 wb_robot_step(TIME_STEP);
   	 
   	 count++;
    }
}   
 


