/*****************************************************************************/
/* File:         performance_estimation.c                                    */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  estimating the performance of a formation         */
/*                                                                           */
/* Author:   10-Oct-14 by Ali marjovi            */
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


#define T_MAX       100000  // max simulation time
#define T_RES       10      // frequency of fitness computation

#define TIME_STEP   64      // [ms] Length of time step
#define VMAX        0.1287

#define READ        0
#define CREATE      1 
#define REWRITE     2 

#define VERBOSE_M   1

#define CROSSING    1
#define OPTIMIZE    1
#define FLOCK_SIZE  5     // Number of robots in flock

#if CROSSING == 1
  #define NB_ROBOTS  2*FLOCK_SIZE
#else
  #define NB_ROBOTS  FLOCK_SIZE
#endif

WbNodeRef robs[NB_ROBOTS];   // Robots nodes
WbFieldRef robs_trans[NB_ROBOTS];  // Robots translation fields
WbFieldRef robs_rotation[NB_ROBOTS]; // Robots rotation fields
float loc[NB_ROBOTS][3];   // Location of everybody in the flock


WbDeviceTag receiver;
WbDeviceTag emitter;

float migrx = 1; // Migration vector component X
float migrz = 0; // Migration vector component Z

int t;

double **data_glob;
double **data_line;
// good results ...
float default_weight[3] = {10., 3.1, 0.5};
float default_RATIO = 1.;
  

/*
* Daniel
* Send default parameters if no optimization going on
*/
void send_default_params(){           
  char message[255]; 

  // send default parameters
  printf("Sending default parameters [%.3f][%.3f][%.3f][%.3f] and migr [%.3f][%.3f]\n", default_weight[0], default_weight[1], default_weight[2], default_RATIO, migrx, migrz);
  sprintf(message, "%f#%f##%f#%f#%f#%f\n", migrx, migrz, default_weight[0], default_weight[1], default_weight[2], default_RATIO);

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
    if(VERBOSE_M){ printf("line number init %f\n", line ); }
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
  float WEIGHT1 = 0.7; float WEIGHT2 = (0.02/10); float WEIGHT3 = (1.0/10); 
  float RATIO = 1.0; 
  float weights[4] = {WEIGHT1, WEIGHT2, WEIGHT3, RATIO};
  int line = 1; 
  
  char *file_name;
  char str[60];
  char message[255]; 
    
  /* Indice param, init, resol and final to be changed manually to give the parameter and its range */  
  float init = 1.0; float resol = 1.0; float final = 10; int nb_param_tuned = 3; 
  int row = 1 + pow((floor(final-init)/resol + 1),nb_param_tuned);
  if(VERBOSE_M){printf("nb lines %d\n", row);}
  int col = 5;
  
  //Create a .csv file with the different information about the simulation tested
  asprintf (&file_name, "Test_parameters_WEIGHT_Reynolds_[%.3f,%.3f,%.3f][%.3f].csv", init, resol, final,RATIO);

  
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
    if(VERBOSE_M){printf("Ready to read line file\n");}
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
      if(VERBOSE_M){printf("File already existing\n");}	
    
      if(params == NULL) {
          perror("No line found to read");
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
    float i = init; float j = init; float k = init; float l = init;
    for(i=init; i<=final; i += resol){
      for(j=init; j<=final; j += resol){
        for(k=init; k<=final; k += resol){
          //for(l=init; l<=final; l += resol){
            weights[2] = i/10;
            weights[1] = j/10;
            weights[0] = k/10;
            //RATIO = (l - init) / (final - init); // scale back to [0, 1] range
            fprintf(params, "%.3f,%.3f,%.3f,%.3f\n", weights[0], weights[1],weights[2],RATIO);
            //if(VERBOSE_M){printf("%.3f,%.3f,%.3f,%.3f\n", weights[0], weights[1],weights[2], RATIO);}
          //}
        }
      }
    }
    read_csv(row, col, file_name, data_glob);
    fclose(params);
    //data_line[0][0] = line; 
  }   
  
  //Send to the robot controller
  int lp = (int)data_line[0][0];
  sprintf(message, "%f#%f##%f#%f#%f#%f\n", migrx, migrz, data_glob[lp][0], data_glob[lp][1], data_glob[lp][2], data_glob[lp][3]);
  //printf("%s\n", message);
  if(wb_emitter_send(emitter, message, strlen(message) + 1)){printf("Send\n");}
  printf("actual line %f\n", data_line[0][0]);
}
/*.............................................End of Optimisation...............................................*/





/*
 * Initialize flock position and devices
 */
void reset(void) {
  int i;

  wb_robot_init();

  emitter = wb_robot_get_device("emitter");
  
  char rob[8] = "epuck0";

  for (i=0;i<NB_ROBOTS;i++){
    sprintf(rob,"epuck%d", i);
    robs[i] = wb_supervisor_node_get_from_def(rob);
    robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
    robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
  }
  
  if(VERBOSE_M) {printf("Reset supervisor\n"); }

}


/*
 * Compute performance metric.
 */
void compute_performance (float* fit_o, float* fit_c, float * fit_v, int offset){
  int i, j;
  float real = 0.0; float imm = 0.0;
  float avg_loc[2] = {0,0};
  float sum_dist = 0.0;
  float projection;
  float speed[2] = {0.0};


  // Compute averages position
  for(i=offset; i<offset+FLOCK_SIZE; i++) {
    for (j=0;j<2;j++) {
      avg_loc[j] += loc[i][j];
    }
  }

  for (j = 0; j<2; j++){
    avg_loc[j] /= FLOCK_SIZE;
  }


  // !!! delay compute_performance until both avg_loc_old_G1 and G2 are populated once (cant use init 0.0 as it affects fitness_velocity)
  static float avg_loc_old_G1[2] = {0.0};
  static float avg_loc_old_G2[2] = {0.0};

  static bool f_first_run = 1;

  if(CROSSING){
    if(f_first_run && offset == 0){ avg_loc_old_G1[0] = avg_loc[0]; avg_loc_old_G1[1] = avg_loc[1]; return; }
    if(f_first_run && offset == FLOCK_SIZE){ avg_loc_old_G2[0] = avg_loc[0]; avg_loc_old_G2[1] = avg_loc[1]; f_first_run = 0; return; }
  }
  else{
    if(f_first_run && offset == 0){ avg_loc_old_G1[0] = avg_loc[0]; avg_loc_old_G1[1] = avg_loc[1]; f_first_run = 0; return; }
  }

  // compute alignment between the robots
  for(i=offset; i<offset+FLOCK_SIZE; i++){
    real += cos(loc[i][2]);
    imm  += sin(loc[i][2]); 
  }

  * fit_o = sqrt(real * real + imm * imm) / FLOCK_SIZE;

  // compute dispersion of robots
  for (i=offset; i<offset+FLOCK_SIZE; i++){
    sum_dist += sqrt( pow(loc[i][0]-avg_loc[0], 2) + pow(loc[i][1]-avg_loc[1], 2) );
  }

  * fit_c = 1 / (1 + (sum_dist/FLOCK_SIZE));

  if(CROSSING){
    if(offset == 0){
      // average displacement velocity along direction of migratory urge
      speed[0] = avg_loc[0] - avg_loc_old_G1[0];
      speed[1] = avg_loc[1] - avg_loc_old_G1[1];      

      //printf("[%d]   [%f --> %f][%f --> %f]  == [%f][%f]\n", offset, avg_loc_old_G1[0], avg_loc[0], avg_loc_old_G1[1], avg_loc[1], speed[0], speed[1]);

      // update location info for next timestep
      avg_loc_old_G1[0] = avg_loc[0];
      avg_loc_old_G1[1] = avg_loc[1];      
      // adapt migration for crossing scenario ( -1.0 for the group going leftward)
      projection = (speed[0]*(-migrx) + speed[1]*migrz) / sqrt(migrx*migrx + migrz*migrz);
    }
    else{
      // average displacement velocity along direction of migratory urge
      speed[0] = avg_loc[0] - avg_loc_old_G2[0];
      speed[1] = avg_loc[1] - avg_loc_old_G2[1];  

      //printf("[%d]   [%f --> %f][%f --> %f]  == [%f][%f]\n", offset, avg_loc_old_G2[0], avg_loc[0], avg_loc_old_G2[1], avg_loc[1], speed[0], speed[1]);

      // update location info for next timestep
      avg_loc_old_G2[0] = avg_loc[0];
      avg_loc_old_G2[1] = avg_loc[1]; 
    }

    projection = (speed[0]*migrx + speed[1]*migrz) / sqrt(migrx*migrx + migrz*migrz);
  }
  else{
    // average displacement velocity along direction of migratory urge
    speed[0] = avg_loc[0] - avg_loc_old_G1[0];
    speed[1] = avg_loc[1] - avg_loc_old_G1[1];      
    // update location info for next timestep
    avg_loc_old_G1[0] = avg_loc[0];
    avg_loc_old_G1[1] = avg_loc[1];  

    projection = (speed[0]*migrx + speed[1]*migrz) / sqrt(migrx*migrx + migrz*migrz);
  }

  * fit_v = projection > 0.0 ? projection/VMAX : 0.0; 
}


bool in_allowed_area(void){
  int i;
  bool inArena = 1;

  for(i=0;i<NB_ROBOTS;i++){
    if(CROSSING){
      if(loc[i][0] < -1.93 || loc[i][0] >  -0.07 || loc[i][1] < -1.85 || loc[i][1] >  1.85){
        inArena = 0;
      }               
    }     
    else{
      if(loc[i][0] < -3.00 || loc[i][0] >  2.85 || loc[i][1] < -1.85 || loc[i][1] >  1.85){
        inArena = 0;
      }
    }
  }

  if(!inArena){
    printf("Robot out of bounds. Aborting simulation run.\n");
  }
  
  return inArena;
}



/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
  int i;                  // Index
  
  reset();

  float fit_cohesion    = 0;     // Performance metric for cohesion
  float fit_orientation = 0;  // Performance metric for orientation
  float fit_velocity    = 0;
  float performance_instant;  
  float performance_overall; float sum_perf = 0; 

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

    for (i=0;i<NB_ROBOTS;i++){            
      loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
      loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
      loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA  
    }
  
    bool f_safe_zone = in_allowed_area();

    if ((t % T_RES == 0) && (t > 0)){ // avoid division by 0
      //Compute and normalize fitness values
      //printf("\n");
      for(i=0; i< (int)(NB_ROBOTS/FLOCK_SIZE); i++){
        compute_performance(&fit_orientation, &fit_cohesion, &fit_velocity, i*FLOCK_SIZE);
        performance_instant = fit_orientation * fit_cohesion * fit_velocity;
        sum_perf += performance_instant; // NB: for crossing, the fitness considered is the combined fitness of both groups
        performance_overall = sum_perf / t; 
        
        //printf("[GROUP %d][time %8d] :: orient: %1.6f :: cohes : %1.6f :: veloc : %1.6f ::: perf_instant %1.6f ::: perf_overall %1.6f\n", i, t, fit_orientation, fit_cohesion, fit_velocity, performance_instant, performance_overall);      
      }

      //Mathilde
      if(OPTIMIZE && (t != 0)){
        nb_repetition += 1; 
        fp = fopen("Reynolds_performance.csv" ,"a");
        fprintf(fp, "%d,%d,%f,%f,%f,%f,%f\n",nb_repetition,t, fit_orientation, fit_cohesion, fit_velocity, performance_instant, performance_overall);
        fclose(fp);

  
        if((t > T_MAX) || !f_safe_zone){
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
