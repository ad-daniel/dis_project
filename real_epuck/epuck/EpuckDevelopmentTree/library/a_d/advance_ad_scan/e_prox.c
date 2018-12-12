/* fonctions to get values from the proximity sensors   */
/* december 2004: first version                         */
/* Basic examples from Lucas Meier                      */
/* Adaptation, formatting and test by Francesco Mondada */

#include "e_ad_conv.h"
#include "../../motor_led/e_epuck_ports.h"
#include "e_prox.h"

extern int e_ambient_ir[8];						// ambient light measurement
extern int e_ambient_and_reflected_ir[8];			// light when led is on

/**
 * Read one proximetre
 * 
 * @param  sensor_number	IN : number of the sensor to get value
 * @return value			OUT: intensity of the reflected pulse IR
 */
int e_get_prox(unsigned int sensor_number)
{
	if (sensor_number > 7)
		return 0;
	else
		return e_ambient_ir[sensor_number] - e_ambient_and_reflected_ir[sensor_number];
}


/**
 * Get the ambiant IR light on one captor
 * 
 * @param  sensor_number	IN : number of the sensor to get value
 * @return value			OUT: ambient IR light
 */
int e_get_ambient_light(unsigned int sensor_number)
{
	if (sensor_number > 7)
		return 0;
	else
		return e_ambient_ir[sensor_number];
}


