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

#define FLOCK_SIZE  5     // Number of robots in flock
#define TIME_STEP   64    // [ms] Length of time step
#define VMAX        0.1287

#define OPTIMIZE    0
#define READ        0
#define CREATE      1 
#define REWRITE     2 

#define MAX_T       100000

WbDeviceTag emitter;


struct States {
   WbNodeRef robs[FLOCK_SIZE];           // Robots nodes
   WbFieldRef robs_trans[FLOCK_SIZE];    // Robots translation fields
   WbFieldRef robs_rotation[FLOCK_SIZE]; // Robots rotation fields
   
   double loc[FLOCK_SIZE][3];
   double fit_o;
   double fit_c;
   double fit_v;
   double performance;
};

#define SCENARIO    0  // 0 is obstacles, 1 is crossing

#if SCENARIO == 0
  #define GROUPS 1 // obstacles
  struct States state[1];
#else
  #define GROUPS 2 // crossing
  struct States state[2];
#endif

double migrx = 1.0;
double migrz = 0.0;			// Migration vector
int t;

double **data_glob;//Mathilde
double **data_line;
// good results ...
double default_weight[5] = { 0.6, 0.8, 0.2, 0.01, 0.01 };
  

/*
* Daniel
* Send default parameters if no optimization going on
*/
void send_default_params(){           
  char message[255]; 

  // send default parameters
  
  // Also good results ...
  // double data_glob[5] = { }

  printf("Sending migr [%.3f][%.3f] and default parameters [%.3f][%.3f][%.3f][%.3f][%.3f] \n", migrx, migrz, default_weight[0], default_weight[1], default_weight[2], default_weight[3], default_weight[4]);
  sprintf(message, "%f#%f##%f#%f#%f#%f#%f\n", migrx, migrz, default_weight[0], default_weight[1], default_weight[2], default_weight[3], default_weight[4]);

  wb_emitter_send(emitter, message, strlen(message) + 1);
}



/*.....................................................Optimisation...............................................*/
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
  char *name = "Line_to_read.csv";
  
  if(action == READ){
    line_to_read = fopen(name,"r");
    //printf("Step2\n");
    read_csv(1,1,name, data_l);
  }else if(action==CREATE){
  printf("line number init %f\n", line );
    line_to_read = fopen(name,"w");
    data_line[0][0] = line; 
    //fprintf(line_to_read, "%f,\n", line);
    read_csv(1,1,name, data_l);
  }else if(action==REWRITE){
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
  double WEIGHT1 = 0.7; double WEIGHT2 = (0.02/10); double WEIGHT3 = (1.0/10); 
  double WEIGHTX =(0.01/10); double WEIGHTY =(0.01/10);
  double weights[5] = {WEIGHT1, WEIGHT2, WEIGHT3, WEIGHTX, WEIGHTY};
  int line = 1; 
  
  char *file_name;
  char str[60];
  char message[255]; 
    
  /* Indice param, init, resol and final to be changed manually to give the parameter and its range */  
  double init = 1; double resol = 0.5; double final = 10; int nb_param_tuned = 2; 
  int row = 1 + pow((floor(final-init)/resol + 1),nb_param_tuned);
  printf("nb lines %d\n", row);
  int col = 5;
  
  //Create a .csv file with the different information about the simulation tested
  asprintf (&file_name, "Test_parameters_WEIGHT 1 and 2 Reynolds_[%.3f,%.3f,%.3f].csv", init, resol, final);

  
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
    handle_line_csv(line_to_read, READ, 0, data_line);
    //printf("data line value is : %f\n", data_line[0][0]);
    fclose(line_to_read);
  }else{
    printf("Creating line file\n"); 
    handle_line_csv(line_to_read, CREATE, (double) line, data_line);
    //printf("data line value after init is : %f\n", data_line[0][0]);
    fclose(line_to_read);
  }
  
  //Depending on the existence of the parameter file, read or create + read it, and extract parameters usefull for the current simulation  
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
    double i = init; double j = init; double k = init;  
    for(i=init; i<=final; i += resol){
      for(j=init; j<=final; j += resol){
        //for(k=init; k<=final; k+=resol){
        //  weights[2] = i/10;
          weights[1] = i/10;
          weights[0] = j/10;
          fprintf(params, "%.3f,%.3f,%.3f,%.3f,%.3f\n", weights[0], weights[1],weights[2], weights[3], weights[4]);
          //printf("%.3f,%.3f,%.3f,%.3f,%.3f\n", weights[0], weights[1],weights[2], weights[3], weights[4]);
        //}
      }
    }
    read_csv(row, col, file_name, data_glob);
    fclose(params);
    //data_line[0][0] = line; 
  }   
  
  //Send to the robot controller
  int lp = (int)data_line[0][0];
  printf("Param to be sent [%.3f][%.3f][%.3f][%.3f][%.3f] and migr [%.3f][%.3f]\n", data_glob[lp][0], data_glob[lp][1], data_glob[lp][2], data_glob[lp][3], data_glob[lp][4], migrx, migrz);
  sprintf(message, "%f#%f##%f#%f#%f#%f#%f\n", migrx, migrz, data_glob[lp][0], data_glob[lp][1], data_glob[lp][2], data_glob[lp][3], data_glob[lp][4]);
  //printf("%s\n", message);
  if(wb_emitter_send(emitter, message, strlen(message) + 1)){printf("Send\n");}
  printf("actual line %f\n", data_line[0][0]);
}
/*.............................................End of Optimisation...............................................*/



/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	emitter = wb_robot_get_device("emitter"); //Mathilde

  int i, group_id;

  for(group_id=0; group_id<GROUPS;group_id++){
    char rob[7] = "epuck0";

    for (i=0;i<FLOCK_SIZE;i++){
      sprintf(rob,"epuck%d",i+group_id*FLOCK_SIZE);
      state[group_id].robs[i] = wb_supervisor_node_get_from_def(rob);
      state[group_id].robs_trans[i] = wb_supervisor_node_get_field(state[group_id].robs[i],"translation");
      state[group_id].robs_rotation[i] = wb_supervisor_node_get_field(state[group_id].robs[i],"rotation");
    }
  }
	
  printf("Supervisor reset\n");
}




/*
 * Compute performance metric.
 */
void compute_performance (int group_id){
	double real = 0.0;   double imm = 0.0;
	double avg_loc[2] = {0,0};
	static double avg_loc_old[2] = {0,0};
	double sum_dist = 0.0;
	double projection;

	// Compute averages position
	for(int i=0; i<FLOCK_SIZE; i++) {
	  for (int j=0;j<2;j++) {
	    avg_loc[j] += state[group_id].loc[i][j];
	  }
	}
	
  for (int j=0; j<2; j++){
	  avg_loc[j] /= FLOCK_SIZE;
	}

	// compute alignment between the robots
	for(int i=0; i < FLOCK_SIZE; i++){
	  real += cos(state[group_id].loc[i][2]);
	  imm  += sin(state[group_id].loc[i][2]); 
	}

	state[group_id].fit_o = sqrt(real * real + imm * imm) / FLOCK_SIZE;

	// compute dispersion of robots
	for (int i = 0; i < FLOCK_SIZE; i++){
	  sum_dist += sqrt( pow(state[group_id].loc[i][0]-avg_loc[0],2)+pow(state[group_id].loc[i][1]-avg_loc[1],2));
	}

	state[group_id].fit_c = 1 / (1 + (sum_dist/FLOCK_SIZE));

	// average displacement velocity along direction of migratory urge
	double speed[2] = { avg_loc[0] - avg_loc_old[0], avg_loc[1] - avg_loc_old[1] };
	projection = (speed[0]*migrx + speed[1]*migrz) / sqrt(migrx*migrx + migrz*migrz);

	state[group_id].fit_v = projection > 0.0 ? projection/VMAX : 0.0; 

  state[group_id].performance = state[group_id].fit_o * state[group_id].fit_c * state[group_id].fit_v;

	// update location info for next timestep
	avg_loc_old[0] = avg_loc[0];
	avg_loc_old[1] = avg_loc[1];
}


bool in_arena_limit(void){
  bool inArena = 1;

  for(int group_id=0; group_id<GROUPS;group_id++){
    for(int i=0;i<FLOCK_SIZE;i++){
      if(wb_supervisor_field_get_sf_vec3f(state[group_id].robs_trans[i])[0] < -3.00 ||
         wb_supervisor_field_get_sf_vec3f(state[group_id].robs_trans[i])[0] >  2.85 ||
         wb_supervisor_field_get_sf_vec3f(state[group_id].robs_trans[i])[2] < -1.85 ||
         wb_supervisor_field_get_sf_vec3f(state[group_id].robs_trans[i])[2] >  1.85){

        inArena = 0;
      }
    }
  }
  return inArena;
}


/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
	int i;			// Index
  int group_id;
	reset();

	// Compute reference fitness values
	
	//double fit_cohesion;			// Performance metric for cohesion
	//double fit_orientation;			// Performance metric for orientation
	//double fit_velocity;
	//double performance;	

	//Mathilde	
	FILE *fp = NULL;
	FILE *fparam = NULL; 
	FILE *line_to_read = NULL; 
	int nb_repetition = 0; //find a way to count nb of simulation with the same parameters
	
	if(OPTIMIZE){ 
	  printf("OPTIMIZATION: ACTIVE\n");
		test_param(fparam, line_to_read);
	}
	else{
		printf("OPTIMIZATION: INACTIVE\n");
		send_default_params();
	}
		
	for(;;) {
		wb_robot_step(TIME_STEP);
		
		if (t % 50 == 0) {
     // gather data from each group
      for(group_id=0;group_id<GROUPS;group_id++){
			  for (i=0;i<FLOCK_SIZE;i++) {
			    // Get data
			    state[group_id].loc[i][0] = wb_supervisor_field_get_sf_vec3f(state[group_id].robs_trans[i])[0]; // X
			    state[group_id].loc[i][1] = wb_supervisor_field_get_sf_vec3f(state[group_id].robs_trans[i])[2]; // Z
			    state[group_id].loc[i][2] = wb_supervisor_field_get_sf_rotation(state[group_id].robs_rotation[i])[3]; // THETA				
    	  }
			  
        // Compute and normalize and fill struct accordingly
			  compute_performance(group_id);
      }
			
		  //Mathilde
		  if(OPTIMIZE && (t != 0)){
        nb_repetition += 1; 

        if(SCENARIO == 0){ 
          // obstacles scenario
          fp = fopen("Reynolds_performance.csv" ,"a");
          fprintf(fp, "%d,%d,%f,%f,%f,%f\n", nb_repetition, t, state[0].fit_o, state[0].fit_c, state[0].fit_v, state[0].performance);
          fclose(fp);
          printf("%d,%d,%f,%f,%f,%f\n", nb_repetition, t, state[0].fit_o, state[0].fit_c, state[0].fit_v, state[0].performance);
        }
        else{ 
          // crossing scenario
          for(group_id=0;group_id<GROUPS;group_id++){
            // name file according to group
            char CSV_filename[50] = "Reynolds_performance_G0.csv";
            sprintf(CSV_filename,"Reynolds_performance_G%d.csv",group_id);

            fp = fopen(CSV_filename ,"a");
            fprintf(fp, "%d,%d,%f,%f,%f,%f\n",nb_repetition, t, state[group_id].fit_o, state[group_id].fit_c, state[group_id].fit_v, state[group_id].performance);
            fclose(fp);
            printf("G%d :: %d,%d,%f,%f,%f,%f\n", group_id, nb_repetition, t, state[group_id].fit_o, state[group_id].fit_c, state[group_id].fit_v, state[group_id].performance);
          }
        }                        	
			
      	if(t > MAX_T || !in_arena_limit()){
          printf("Exit condition\n");
                          
          if(data_line[0][0] <= data_glob[0][0]-1){
            data_line[0][0] += 1;
            printf("next line number is %f\n", data_line[0][0]);
            handle_line_csv(line_to_read, REWRITE, data_line[0][0], data_line);
            wb_supervisor_world_reload();
          }
          else{
            //wb_supervisor_simulation_quit();
            printf("In pause mode\n");
            wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
          }
        }
                      
        
      }
    }

    t += TIME_STEP;
  }
}
