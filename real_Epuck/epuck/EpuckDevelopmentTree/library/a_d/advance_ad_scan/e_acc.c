//START_HEADER
/*
; --------------------------------------------------------------------------
; file name : acc.c
; Version : 0.0a 
; Date :07/11/05
; Conceptor : Borter Jean-Joël
; Made for : dsPIC30F6014
; --------------------------------------------------------------------------
; File fonction and description
; --------------------------------------------------------------------------
;
; This file is made to read the values from the accelerometre
; and return with 
;
;	calculate_acc_raw() :
;		the X,Y, values centred and filtred in a pointed struct
;	calculate_acc_spherical() :
;		the filtered composite values (direction, intensity) in a pointed struct
;	read_acc() :
;		the acceleration absolut value in float
;	read_angle() :
;		the angle value in int
;	acc_calibre() :	
;		it reads two time the average_size to avoid edge effects
;		then it reads 100 values and average them to initiat 
;		the "zero" value of the accelerometre
;		
; --------------------------------------------------------------------------
; --------------------------------------------------------------------------
*/
//END_HEADER

#include "math.h"
#include "e_acc.h"
#include "e_ad_conv.h"
#include "../../motor_led/e_epuck_ports.h"
#include "../../motor_led/e_led.h"
#include <stdlib.h>

extern int e_acc_scan[3][ACC_SAMP_NB];
extern unsigned int e_last_acc_scan_id;
/*****************************************************
 * internal variables                                *
 *****************************************************/
static int angle_mem = 0;			//used in the display_angle function

static int centre_x = 0;			//zero value for x axe
static int centre_y = 0;			//zero value for y axe
static int centre_z = 2000;		//zero value for z axe

static int acc_x, acc_y, acc_z;	//raw acceleration values in carthesian coord.
static float acceleration, orientation, inclination;	//spherical coord. of the accel vector

/*****************************************************
 * internal function                                 *
 *****************************************************/

/**
 * Read the last value of a given accelerator axes.
 *
 * @param captor		IN ID of the AD channel to read (must be 0 to 2)
 * @return value		filtered channel's value
 */
int e_get_acc(unsigned int captor)
{
	if (captor < 3)
	{
		return (e_acc_scan[captor][e_last_acc_scan_id]);
	}
	else
	{
		return((int)ANGLE_ERROR);	
	}
}
/**
 * Read the value of a channel, filtered by an averaging filter
 *
 * @param captor		IN ID of the AD channel to read (must be 0 to 2)
 * @param filter_size	IN size of the filter (must be between 1 and SAMPLE_NUMBER)
 * @return value		filtered channel's value
 */

int e_get_acc_filtered(unsigned int captor, unsigned int filter_size)
{
	long temp = 0;
	int i,j;

	// channel ID must be between 0 to 13 and 
	// filter_size must be between 1 to SAMPLE_NUMBER
	if ((captor < 3) && 
		(filter_size > 0) && (filter_size <= ACC_SAMP_NB))
	{
		for (i=0, j=(ACC_SAMP_NB-(filter_size-(e_last_acc_scan_id+1)))%ACC_SAMP_NB ; i<filter_size ; i++, j=(j+1)%ACC_SAMP_NB)
		{
			temp += e_acc_scan[captor][j];
		}
	}
	return ((int)(temp/filter_size));
}


/**
 * read the x, y, z values, applie an averaging filter
 * and finaly subbstract the centre values.
 * @param  void
 * @return void
 */
void calculate_acc_raw(void)
{
	acc_x = e_get_acc_filtered(ACCX_BUFFER, FILTER_SIZE) - centre_x;	// generates positive  
	acc_y = e_get_acc_filtered(ACCY_BUFFER, FILTER_SIZE) - centre_y;	// and negative value
	acc_z = e_get_acc_filtered(ACCZ_BUFFER, FILTER_SIZE) - centre_z;	// to make the use easy
	return;
}

/**
 * calculate the intensity of the acceleration vector,
 * and the Euler's angles
 * @param  void
 * @return void
 */
void calculate_acc_spherical(void)
{
	calculate_acc_raw();

// calculat the absolute acceleration value
	acceleration = sqrtf((float)(((long)acc_x * (long)acc_x) + ((long)acc_y * (long)acc_y) + ((long)acc_z * (long)acc_z)));
	
	inclination =  90.0 - atan2f((float)(acc_z), sqrtf( (float)(((long)acc_x * (long)acc_x) + ((long)acc_y * (long)acc_y) ))) * CST_RADIAN;
	
	if (inclination<5)
		orientation=0;
	else 
		orientation = (atan2f((float)(acc_x), (float)(acc_y)) * CST_RADIAN) + 180.0;		// 180 is added to have 0 to 360° range

	return;
}



/**
 * user called functions
 */

/*
 * initialize de ad converter and calculate the zero
 * values.
 * @param  void
 * @return void
 */
void e_acc_calibr(void)
{
	centre_x = e_get_acc_filtered(ACCX_BUFFER, 50);
	centre_y = e_get_acc_filtered(ACCY_BUFFER, 50);
}

/**
 * calculate and return the accel. in spherical coord.
 * @param  void
 * @return OUT (TypeAccSpheric) accel. in spherical coord.
 */
TypeAccSpheric e_read_acc_spheric(void)
{
	TypeAccSpheric result;

	calculate_acc_spherical();
	result.acceleration = acceleration;
	result.orientation  = orientation;
	result.inclination  = inclination;
	
	return(result);
}

/**
 * calculate and return the inclination angle
 * @param  void
 * @return OUT (int) inclination angle of the robot
 */
float e_read_inclination(void)
{
	calculate_acc_spherical();
	
	return(inclination);
}

/**
 * calculate and return the orientation angle
 * @param  void
 * @return OUT (int) orientation of the accel vector
 */
float e_read_orientation(void)
{
	calculate_acc_spherical();
	
	return(orientation);
}

/**
 * calculate and return the intensity of the acceleration
 * @param  void
 * @return OUT (int) intensity of the acceleration vector
 */
float e_read_acc(void)
{ 
	calculate_acc_spherical();	//calculate intesnsity and euler's angle
	
	return(acceleration);
}

/**
 * calculate and return acceleration on the x,y,z axis
 * @param  void
 * @return OUT (TypeAccRaw) acceleration on the x,y,z axis
 */
TypeAccRaw e_read_acc_xyz(void)
{
	TypeAccRaw result;

	calculate_acc_raw();
	result.acc_x = acc_x;
	result.acc_y = acc_y;
	result.acc_z = acc_z;

	return(result);
}

/**
 * calculate and return acceleration on the x axis
 * @param  void
 * @return OUT (int) acceleration on the x axis
 */
int e_read_acc_x(void)
{
	calculate_acc_raw();

	return(acc_x);
}

/**
 * calculate and return acceleration on the y axis
 * @param  void
 * @return OUT (int) acceleration on the y axis
 */
int e_read_acc_y(void)
{
	calculate_acc_raw();

	return(acc_y);
}

/**
 * calculate and return acceleration on the z axis
 * @param  void
 * @return OUT (int) acceleration on the z axis
 */
int e_read_acc_z(void)
{
	calculate_acc_raw();

	return(acc_z);
}

/**
 * light the led according to the orientation angle
 * @param  void
 * @return void
 */
void e_display_angle(void)
{
	float angle = 0.0;

// 	To avoid oscillation the limite of variation is limited at 
//  a fix value	wich is 1/9 of the LED resolution
	angle = e_read_orientation();
	if ( abs(angle_mem - angle) > 5.0)
	{
		e_led_clear();
			// table of selection
		if ( 	  (angle > (360.0 - 22.5)) |  (angle <= (0.0   + 22.5)) && angle != ANGLE_ERROR)
			LED0 = 1;
		else if ( angle > (45.0 - 22.5)  && angle <= (45.0  + 22.5))
			LED7 = 1;
		else if ( angle > (90.0 - 22.5)  && angle <= (90.0  + 22.5))
			LED6 = 1;
		else if ( angle > (135.0 - 22.5) && angle <= (135.0 + 22.5))
			LED5 = 1;
		else if ( angle > (180.0 - 22.5) && angle <= (180.0 + 22.5))
			LED4 = 1;
		else if ( angle > (225.0 - 22.5) && angle <= (225.0 + 22.5))
			LED3 = 1;
		else if ( angle > (270.0 - 22.5) && angle <= (270.0 + 22.5))
			LED2 = 1;
		else if ( angle > (315.0 - 22.5) && angle <= (315.0 + 22.5))
			LED1 = 1;
		else				// if angle undefined
			e_led_clear();
		angle_mem = angle;	// value to compare with the next one
	}
}
