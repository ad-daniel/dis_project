/*****************************************************************************/
/* File:         performance_estimation.c                                    */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  estimating the performance of a formation 		     */
/*                                                                           */
/* Author: 	 10-Oct-14 by Ali marjovi				     */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	5 		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step
#define VMAX        0.1287

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag receiver;

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

#define RULE1_THRESHOLD 0.2
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0


int offset;				// Offset of robots number
float migrx = 3;
float migrz = 0;			// Migration vector
float orient_migr; 			// Migration orientation
int t;


/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();


	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i+offset);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}

	printf("Reset supervisor\n");

}




/*
 * Compute performance metric.
 */
void compute_performance (float* fit_o, float* fit_c, float * fit_v){
	float real = 0.0;   float imm = 0.0;
	float avg_loc[2] = {0,0};
	static float avg_loc_old[2] = {0,0};
	float sum_dist = 0.0;
	float projection;

	// Compute averages position
	for(int i=0; i<FLOCK_SIZE; i++) {
	 for (int j=0;j<2;j++) {
	   avg_loc[j] += loc[i][j];
	   }
	}
	for (int j = 0; j<2; j++){
	avg_loc[j] /= FLOCK_SIZE;
	}

	// compute alignment between the robots
	for(int i = 0; i < FLOCK_SIZE; i++){
	real += cos(loc[i][2]);
	imm += sin(loc[i][2]); 
	}

	* fit_o = sqrt(real * real + imm * imm) / FLOCK_SIZE;

	// compute dispersion of robots
	for (int i = 0; i < FLOCK_SIZE; i++){
	sum_dist += sqrt( pow(loc[i][0]-avg_loc[0],2)+pow(loc[i][1]-avg_loc[1],2));
	}

	* fit_c = 1 / (1 + (sum_dist/FLOCK_SIZE));

	// average displacement velocity along direction of migratory urge
	float speed[2] = { avg_loc[0] - avg_loc_old[0], avg_loc[1] - avg_loc_old[1] };
	projection = (speed[0]*migrx + speed[1]*migrz) / sqrt(migrx*migrx + migrz*migrz);

	* fit_v = projection > 0.0 ? projection/VMAX : 0.0; 

	// update location info for next timestep
	avg_loc_old[0] = avg_loc[0];
	avg_loc_old[1] = avg_loc[1];
}



/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
	int i;			// Index
  
	reset();

	printf("Supervisor using urge: [%f][%f]\n", migrx, migrz);
	
	// Compute reference fitness values
	
	float fit_cohesion;			// Performance metric for cohesion
	float fit_orientation;			// Performance metric for orientation
	float fit_velocity;
	float performance;	
		
	for(;;) {
		wb_robot_step(TIME_STEP);
		
		if (t % 50 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA				
    		}

			//Compute and normalize fitness values
			compute_performance(&fit_orientation, &fit_cohesion, & fit_velocity);
	  		performance = fit_orientation * fit_cohesion * fit_velocity;
			printf("[time %8d] :: orient: %1.6f :: cohes : %1.6f :: veloc : %1.6f ::: performance %1.6f\n", t, fit_orientation, fit_cohesion, fit_velocity, performance);			
			
		}
		
		t += TIME_STEP;
	}

}
