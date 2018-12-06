#ifndef _ACC_DEFS
#define _ACC_DEFS


#define CST_RADIAN		(180.0/3.1415)	// used to convert radian in degrees
#define ANGLE_ERROR		666.0			// value returned if an angle can't be defined
#define FILTER_SIZE		5				// define the size of the averaging filter

// ID of the different captor in their respective array

#define ACCX_BUFFER	0
#define ACCY_BUFFER	1
#define ACCZ_BUFFER	2


/* struct to store the acceleration vector
   in spherical coord. */
typedef struct
{
	// lenght of the acceleration vector
	// = intensity of the acceleration
	float acceleration;

	// orientation of the acceleration vector 
	// in the horizontal plan, zero facing front
	//   0° = inclination to the front 
	//        (front part lower than rear part)
	//  90° = inclination to the left  
	//        (left part lower than right part)
	// 180° = inclination to the rear  
	//        (rear part lower than front part)
	// 270° = inclination to the right 
	//        (right part lower than left part)
	float orientation;

	// inclination angle with the horizontal plan
	//   0° = e-puck horizontal
	//  90° = e-puck vertical
	// 180° = e-puck horizontal but up-side-down
	float inclination;

} TypeAccSpheric;

/* struct to store the acceleration raw data
   in carthesian coord. */
typedef struct
{
	int acc_x;
	int acc_y;
	int acc_z;
} TypeAccRaw;


/**********************************************************************
 * ---------------------- Functions from acc.c ------------------------
 **********************************************************************/
int e_get_acc(unsigned int captor);
int e_get_acc_filtered(unsigned int captor, unsigned int filter_size);
TypeAccSpheric e_read_acc_spheric(void);
float e_read_orientation(void);
float e_read_inclination(void);
float e_read_acc(void);

TypeAccRaw e_read_acc_xyz(void);
int e_read_acc_x(void);
int e_read_acc_y(void);
int e_read_acc_z(void);

void e_acc_calibr(void);
void e_display_angle(void);
#endif
