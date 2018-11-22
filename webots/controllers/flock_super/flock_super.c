/*****************************************************************************/
/* File:         performance_estimation.c                                    */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  estimating the performance of a formation 		     */
/*                                                                           */
/* Author: 	 10-Oct-14 by Ali marjovi				     */
/*****************************************************************************/

#define _GNU_SOURCE
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	5 		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step
#define VMAX        0.1287

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag receiver;
WbDeviceTag emitter;

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

int offset;				// Offset of robots number
float migrx = 0;
float migrz = -3;			// Migration vector
float orient_migr; 			// Migration orientation
int t;

double **data_glob;//Mathilde
double **data_line;


/*
* MATHILDE
* Read csv file and update information in a table, so the parameters can be accessed and send to robots controllers
*/
void read_csv(int row, int col, char *filename, double **data){

  FILE *file;
  file = fopen(filename, "r");

  int i = 0;
  char line[4098];
  while (fgets(line, 4098, file) && (i < row))
  {
    char* tmp = strdup(line);
    int j = 0;
    const char* tok;
    
    for (tok = strtok(line, ","); tok && *tok; j++, tok = strtok(NULL, ",")){
      data[i][j] = atof(tok);
      //printf("%.3f\t\t", data[i][j]);
      //printf("%s", tok);
    }
    free(tmp);
    i++;
  }
  fclose(file);
}


/*
* MATHILDE
* Create, read and rewrite line file to avoid changing all the params file at every simul just for 1 line change
*/
void handle_line_csv(FILE *line_to_read, int action, double line, double **data_l){
  int read = 0; 
  int create = 1; 
  int rewrite = 2; 
  char *name = "Line_to_read.csv";
  
  if(action == read){
    line_to_read = fopen(name,"r");
    //printf("Step2\n");
    read_csv(1,1,name, data_l);
  }else if(action==create){
  printf("line number init %f\n", line );
    line_to_read = fopen(name,"w");
    data_line[0][0] = line; 
    //fprintf(line_to_read, "%f,\n", line);
    read_csv(1,1,name, data_l);
  }else if(action==rewrite){
    line_to_read = fopen(name,"w");
    fprintf(line_to_read, "%f\n", line);
  }  
  fclose(line_to_read);  
}


/*
* MATHILDE
* Write in a file the sets of parameters to be tested, allow to change params at every world reload
*/
void test_param(FILE *params, FILE *line_to_read){                              
  /*Create the .cvs file to test differents parameters, and upload it at every new reload*/
  float WEIGHT1 = 0.7; float WEIGHT2 = (0.02/10); float WEIGHT3 = (1.0/10); 
  float WEIGHTX =(0.01/10); float WEIGHTY =(0.01/10);
  float weights[5] = {WEIGHT1, WEIGHT2, WEIGHT3, WEIGHTX, WEIGHTY};
  int line = 1; 
  
  char *file_name;
  char str[60];
  char message[255]; 
    
  /* Indice param, init, resol and final to be changed manually to give the parameter and its range */  
  int indice_param = 1; //1 for WEIGHT1; 2 for WEIGHT2 etc...
  float init = 1; float resol = 1; float final = 10;
  int row = 1 + (final-init)/resol + 1;
  int col = 5;
  
  //Create a .csv file with the different information about the simulation tested
  asprintf (&file_name, "Test_parameters_WEIGHT%d_[%.3f,%.3f,%.3f].csv", indice_param, init, resol, final);
  
  //Allocate space to store usefull param for the current simulation
  data_glob = (double **)malloc(row * sizeof(double *));
  for (int i = 0; i < row; ++i){
    data_glob[i] = (double *)malloc(col * sizeof(double));
  }
  
  data_line = (double **)malloc(1 * sizeof(double *));
  for (int i = 0; i < 1; ++i){
    data_line[i] = (double *)malloc(1 * sizeof(double));
  }
  
  //Create or read file that contains the line we need to read next
  if((line_to_read = fopen("Line_to_read.csv", "r"))){
    printf("Ready to read line file\n");
    int read = 0; 
    handle_line_csv(line_to_read, read, 0, data_line);
    //printf("data line value is : %f\n", data_line[0][0]);
    fclose(line_to_read);
  }else{
    int create = 1; 
    printf("Creating line file\n"); 
    handle_line_csv(line_to_read, create, (double) line, data_line);
    //printf("data line value after init is : %f\n", data_line[0][0]);
    fclose(line_to_read);
  }
  
  //Depending on the existence of the file, read or create + read it, and extract parameters usefull for the current simulation  
  if((params = fopen(file_name, "r"))){
    printf("File already existing\n");	
    
    if(params == NULL) {
      perror("No line found to read");
      //printf("No line found to read\n");
     }
     
     if( fgets (str, 60, params)!=NULL ) {
       //printf("Should be printing\n");
       read_csv(row, col, file_name, data_glob);
    }
    fclose(params);   
  }else{
    printf("File creation\n"); 
    params = fopen(file_name, "w");
    fprintf(params, "%d,%d\n", row, col);
    //fprintf(params, "File created\n"); 
    float i = init; 
    for(i=init; i<=final; i += resol){
      weights[indice_param-1] = i;
      fprintf(params, "%.3f,%.3f,%.3f,%.3f,%.3f\n", weights[0], weights[1],weights[2], weights[3], weights[4]);
    }
    read_csv(row, col, file_name, data_glob);
    fclose(params);
  }   
  
  //Send to the robot controller
  int lp = (int)data_line[0][0];
  printf("Param to be sent %.3f,%.3f,%.3f,%.3f,%.3f\n", data_glob[lp][0], data_glob[lp][1], data_glob[lp][2], data_glob[lp][3], data_glob[lp][4]);
  sprintf(message, "%f#%f#%f#%f#%f\n", data_glob[lp][0], data_glob[lp][1], data_glob[lp][2], data_glob[lp][3], data_glob[lp][4]);
  //printf("%s\n", message);
  wb_emitter_send(emitter, message, strlen(message) + 1);
  printf("actual line %f\n", data_line[0][0]);
  
}



/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	emitter = wb_robot_get_device("emitter"); //Mathilde


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

	//Mathilde	
	FILE *fp = NULL;
	FILE *fparam = NULL; 
	FILE *line_to_read = NULL; 
	int nb_repetition = 0; //find a way to count nb of simulation with the same parameters
	
	test_param(fparam, line_to_read);

		
	for(;;) {
		wb_robot_step(TIME_STEP);
		
		if (t % 5 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA				
    		}

			//Compute and normalize fitness values
			compute_performance(&fit_orientation, &fit_cohesion, & fit_velocity);
	  		performance = fit_orientation * fit_cohesion * fit_velocity;
			//printf("[time %8d] :: orient: %1.6f :: cohes : %1.6f :: veloc : %1.6f ::: performance %1.6f\n", t, fit_orientation, fit_cohesion, fit_velocity, performance);			
			
			//Mathilde
			nb_repetition += 1; 
			fp = fopen("Reynolds_performance 2.csv" ,"a");
			fprintf(fp, "%d,%d,%f,%f,%f,%f\n",nb_repetition,t, fit_orientation, fit_cohesion, fit_velocity, performance);
            fclose(fp);			
		}

		if(t==40000){
            printf("Exit condition\n");
	        if(data_line[0][0] <= data_glob[0][0]-1){
        	    data_line[0][0] += 1;
    	    	printf("next line number is %f\n", data_line[0][0]);
            	int action = 2; 
            	handle_line_csv(line_to_read, action, data_line[0][0], data_line);
            	wb_supervisor_world_reload();
        	}
       		else{
        		//wb_supervisor_simulation_quit();
            	printf("In pause mode\n");
            	wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
        		}
        }
                      
        t += TIME_STEP;
                      
	}
}
