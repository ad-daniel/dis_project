/*****************************************************************************/
/*						hugo.c
/* Last revision:15-Nov-18 by Mathilde Bensimhon 		           */
/*****************************************************************************/

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
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	float x = speed[robot_id][0]*cosf(-my_position[2]) - speed[robot_id][1]*sinf(-my_position[2]); // x in robot coordinates
	float z = speed[robot_id][0]*sinf(-my_position[2]) + speed[robot_id][1]*cosf(-my_position[2]); // z in robot coordinates
    if(robot_id == 0 && VERBOSE){printf("[x] : %f, [y] : %f [theta] : %f\n",x,z,my_position[2]);}
	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.8;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = atan2(z, x);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	if(robot_id == 0 && VERBOSE){printf("[u] : %f, [w] : %f [range] : %f\n",u,w,range);}

	// Convert to wheel speeds!
	*msl += (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr += (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	if(robot_id == 0 && VERBOSE){printf("[msr] : %d, [msl] : %d \n",*msr,*msl);}

  	if(ABS(*msl) > MAX_SPEED || ABS(*msr) > MAX_SPEED){
	float max = (ABS(*msl)>ABS(*msr)?ABS(*msl):ABS(*msr));
	*msl = *msl/max * MAX_SPEED;
	*msr = *msr/max * MAX_SPEED;
	}
	if(robot_id == 0 && VERBOSE){printf("[msr] : %d, [msl] : %d \n",*msr,*msl);}

}

void global2rel(float* v_global, float* v_ref){
	v_ref[0] = cos(my_position[2])*(v_global[0]) - sin(my_position[2])*(v_global[1]);
	v_ref[1] = sin(my_position[2])*(v_global[0]) + cos(my_position[2])*(v_global[1]);
}
void rel2global(float* v_global, float* v_ref){
	v_ref[0] = cos(-my_position[2])*(v_global[0]) - sin(-my_position[2])*(v_global[1]);
	v_ref[1] = sin(-my_position[2])*(v_global[0]) + cos(-my_position[2])*(v_global[1]);
}

void migration_urge(int* mmsl, int*mmsr)
{
	//float migr_rel[2] = {0};
	
	//global2rel(migr,migr_rel);
	
	compute_wheel_speeds(mmsl, mmsr);
	
}

void main(){
for(;;){

int mmsl = 0;
int mmsr = 0;
if(MIGRATORY_URGE){
speed[robot_id][0] = 0;
speed[robot_id][1] = 1;

migration_urge(&mmsl,&mmsr);}

}
}
