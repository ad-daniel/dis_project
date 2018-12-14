/********************************************************************************

			Control motor of e-puck	with timer 4 and 5
			December 2004: first version							
			Lucas Meier & Francesco Mondada  						
			Version 1.0 november 2005								
			Michael Bonani
			Additions by Alexander Bahr (February 2010)


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005-2007 Michael Bonani, Francesco Mondada, Lucas Meier
(c) 2010 Alexander Bahr

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Distributed intelligent systems and algorithms laboratory http://disal.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup motor_LED
 * \brief Manage the motors (with timer 4 and 5).
 *
 * This module manage the motors with two timers: timer4 (motor left)
 * and timer5 (motor right).
 * \warning You can't use this module to control the motors if you are
 * using the camera, because the camera's module also use timer4 and
 * timer5.
 *
 * A little exemple for the motors (e-puck turn on himself)
 * \code
 * #include <p30f6014A.h>
 * #include <motor_led/e_epuck_ports.h>
 * #include <motor_led/e_init_port.h>
 * #include <motor_led/e_motors.h>
 * 
 * int main(void)
 * {
 * 	e_init_motors();
 * 	e_set_speed_left(500); //go forward on half speed
 * 	e_set_speed_right(-500); //go backward on half speed
 * 	while(1) {}
 * }
 * \endcode
 * \author Code: Michael Bonani, Francesco Mondada, Lucas Meier, Alexander Bahr \n Doc: Jonathan Besuchet
 */

#ifndef _MOTORS
#define _MOTORS

// robot geometry // added by bahr
#define STEPS_PER_M 4443 			// in [steps per meters] added by bahr NOTE: this is NOT the correct parameter
#define WHEEL_DISTANCE 0.060	// in [meters] added by bahr NOTE: this is NOT the correct parameter
#define DR_NOISE 0						// noise level in percent
#define DR_NOISE_BIAS 0 			// noise bias in percent
#define DR_BIAS 0 						// dead-reckoning bias in percent

/* functions */

void e_init_motors(void); // init to be done before using the other calls

void e_set_speed_left(int motor_speed); 	// motor speed in steps/s
void e_set_speed_right(int motor_speed);  	// motor speed in steps/s
long int e_get_steps_left(void);					// motors steps done left // modified by bahr
long int e_get_steps_right(void);				// motors steps done right // modified by bahr
void e_set_steps_left(long int set_steps);		// set motor steps counter // modified by bahr
void e_set_steps_right(long int set_steps);		// set motor steps counter // modified by bahr

// open loop motor control
int e_get_ol_ctrl_status(void); // added by bahr
void e_set_distance(long int set_distance, int motor_speed); // added by bahr
void e_set_turn(double turn_angle, int motor_speed); // added by bahr

// dead reckoning
void e_do_dr(void); // added by bahr
void e_set_dr_position(double set_dr_x,double set_dr_y,double set_dr_theta); // added by bahr
void e_get_dr_position(double *get_dr_x,double *get_dr_y,double *get_dr_theta); // added by bahr

// auxilary routines to convert from a compass heading (0 degrees is y-axis) in degrees to a
// mathematical angle 0 radians is x-axis
double e_deg2rad(double angle_deg); // added by bahr
double e_rad2deg(double angle_rad); // added by bahr

#endif
