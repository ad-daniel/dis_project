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

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h> // added by bahr
#include "e_epuck_ports.h"
#include "e_init_port.h"
#include "e_motors.h"

#define PI 3.14159265358979 // added by bahr

/* internal variables */

static int left_speed = 0;
static int right_speed = 0;
static long int nbr_pas_left = 0; // modified by bahr
static long int nbr_pas_right = 0; // modified by bahr

// indicates the status of the open loop control
// 0 = open loop control is not active
// 1 = open loop control is active, but the target has not been reached
// 2 = open loop control is active and the target has been reached on left motor only
// 3 = open loop control is active and the target has been reached on right motor only
// 4 = open loop control is active and the target has been reached on both motors
static int ol_ctrl_status = 0; // added by bahr

// maintains status and the target motor step count for both motors for the open loop control
static long int nbr_pas_ol_ctrl = 0; // added by bahr
static long int nbr_pas_ol_ctrl_left = 0; // added by bahr
static long int nbr_pas_ol_ctrl_right = 0; // added by bahr

// dead-reckoned x,y,theta position
double dr_x = 0; // added by bahr
double dr_y = 0; // added by bahr
double dr_theta = 0; // added by bahr

static long int nbr_pas_left_old = 0; // added by bahr
static long int nbr_pas_right_old = 0; // added by bahr

/* internal calls */
 
void __attribute__((interrupt, auto_psv, shadow))
 _T5Interrupt(void) // interrupt for motor 1 (of two) = left motor
{
  static int motor_phase=0;		 // phase can be 0 to 3

  IFS1bits.T5IF = 0;             // clear interrupt flag

  // increment or decrement phase depending on direction

  if (left_speed > 0) // inverted for the two motors
    {
   	nbr_pas_left++;
		motor_phase--;
    if (motor_phase < 0) motor_phase = 3;
    }
  else 
   	{
    nbr_pas_left--;
    motor_phase++;
    if (motor_phase > 3) motor_phase = 0;
    }

	// check if open loop control is active // added by bahr
	if ((ol_ctrl_status != 0) && (ol_ctrl_status != 4))
	{	
		// check if the open loop control target for the left motor has been reached // added by bahr
		if (nbr_pas_ol_ctrl_left == nbr_pas_left)
		{
			// stop left motor
			T5CONbits.TON = 0;            // stop Timer5
    	MOTOR1_PHA = 0;
    	MOTOR1_PHB = 0;
    	MOTOR1_PHC = 0;
    	MOTOR1_PHD = 0;

			// updates status // added by bahr
			if (ol_ctrl_status == 3)
			{
				// set status to "target reached"
				ol_ctrl_status = 4;
			}
			else
			{
				// set status to "left motor done, right motor running"
				ol_ctrl_status = 2;
			}
		
		}
	}
  
  // set the phase on the port pins

  switch (motor_phase)
  {
    case 0:
    {
      MOTOR1_PHA = 0;
      MOTOR1_PHB = 1;
      MOTOR1_PHC = 0;
      MOTOR1_PHD = 1;
      break;
    }
    case 1:
    {
      MOTOR1_PHA = 0;
      MOTOR1_PHB = 1;
      MOTOR1_PHC = 1;
      MOTOR1_PHD = 0;
      break;
    }
    case 2:
    {
      MOTOR1_PHA = 1;
      MOTOR1_PHB = 0;
      MOTOR1_PHC = 1;
      MOTOR1_PHD = 0;
      break;
    }
    case 3:
    {
      MOTOR1_PHA = 1;
      MOTOR1_PHB = 0;
      MOTOR1_PHC = 0;
      MOTOR1_PHD = 1;
      break;
    }
  }
}

void __attribute__((interrupt, auto_psv, shadow))
 _T4Interrupt(void)  // interrupt for motor 2 (of two) = right motor
{
  static int motor_phase=0;	// phase can be 0 to 3

  IFS1bits.T4IF = 0;             // clear interrupt flag

  // increment or decrement phase depending on direction

  if (right_speed < 0)
    {
    nbr_pas_right--;
    motor_phase--;
    if (motor_phase < 0) motor_phase = 3;
    }
  else 
    {
    nbr_pas_right++;
    motor_phase++;
    if (motor_phase > 3) motor_phase = 0;
    }
	
	// check if open loop control is active // added by bahr
	if ((ol_ctrl_status != 0) && (ol_ctrl_status != 4))
	{
		// check if the open loop control target for the right motor has been reached // added by bahr
		if (nbr_pas_ol_ctrl_right == nbr_pas_right)
		{
			// stop right motor // added by bahr
			T4CONbits.TON = 0;            // stop Timer4
    	MOTOR1_PHA = 0;
    	MOTOR1_PHB = 0;
    	MOTOR1_PHC = 0;
    	MOTOR1_PHD = 0;

			// updates status // added by bahr
			if (ol_ctrl_status == 2)
			{
				// set status to "target reached"
				ol_ctrl_status = 4;
			}
			else
			{
				// set status to "right motor done, left motor running"
				ol_ctrl_status = 3;
			}
		}
	}
  
  // set the phase on the port pins

  switch (motor_phase)
  {
    case 0:
    {
      MOTOR2_PHA = 0;
      MOTOR2_PHB = 1;
      MOTOR2_PHC = 0;
      MOTOR2_PHD = 1;
      break;
    }
    case 1:
    {
      MOTOR2_PHA = 0;
      MOTOR2_PHB = 1;
      MOTOR2_PHC = 1;
      MOTOR2_PHD = 0;
      break;
    }
    case 2:
    {
      MOTOR2_PHA = 1;
      MOTOR2_PHB = 0;
      MOTOR2_PHC = 1;
      MOTOR2_PHD = 0;
      break;
    }
    case 3:
    {
      MOTOR2_PHA = 1;
      MOTOR2_PHB = 0;
      MOTOR2_PHC = 0;
      MOTOR2_PHD = 1;
      break;
    }
  }
}


/* ---- user calls ---- */

/*! \brief Initialize the motors's ports
 *
 * This function initialize the ports used by the motors.
 * In fact it call "the e_init_port()" function.
 * \sa e_init_port
 */
void e_init_motors(void)
{
  e_init_port();				// init general ports
}

/*! \brief Manage the left speed
 *
 * This function manage the left motor speed by changing the MOTOR1
 * phases. The changing phases frequency (=> speed) is controled by
 * the timer5.
 * \param motor_speed from -1000 to 1000 give the motor speed in steps/s,
 * positive value to go forward and negative to go backward.
 */
void e_set_speed_left(int motor_speed)  // motor speed in steps/s
{
  if (motor_speed == 0)
  {
    T5CONbits.TON = 0;            // stop Timer5
    MOTOR1_PHA = 0;
    MOTOR1_PHB = 0;
    MOTOR1_PHC = 0;
    MOTOR1_PHD = 0;
  }
  else
  {
    T5CONbits.TON = 0;            // stop Timer5

    left_speed = motor_speed;

    T5CON = 0;                    // 
    T5CONbits.TCKPS=3;            // prescsaler = 256
    TMR5 = 0;                     // clear timer 5
    PR5 = (FCY/256)/abs(motor_speed);  // interrupt every Xms with 256 prescaler
    IFS1bits.T5IF = 0;            // clear interrupt flag
    IEC1bits.T5IE = 1;            // set interrupt enable bit
    T5CONbits.TON = 1;            // start Timer5
  }
}

/*! \brief Manage the right speed
 *
 * This function manage the right motor speed by changing the MOTOR2
 * phases. The changing phases frequency (=> speed) is controled by
 * the timer4.
 * \param motor_speed from -1000 to 1000 give the motor speed in steps/s,
 * positive value to go forward and negative to go backward.
 */
void e_set_speed_right(int motor_speed)
{
  if (motor_speed == 0)
  {
    T4CONbits.TON = 0;            // stop Timer4
    MOTOR2_PHA = 0;
    MOTOR2_PHB = 0;
    MOTOR2_PHC = 0;
    MOTOR2_PHD = 0;
  }
  else
  {
    T4CONbits.TON = 0;            // stop Timer4

    right_speed = motor_speed;

    T4CON = 0;                    // 
    T4CONbits.TCKPS=3;            // prescsaler = 256
    TMR4 = 0;                     // clear timer 4
    PR4 = (FCY/256)/abs(motor_speed);  // interrupt every Xms with 256 prescaler
    IFS1bits.T4IF = 0;            // clear interrupt flag
    IEC1bits.T4IE = 1;            // set interrupt enable bit
    T4CONbits.TON = 1;            // start Timer4
  }
}

/*! \brief Give the number of left motor steps
 * \return The number of phases steps made since the left motor
 * is running.
 */
long int e_get_steps_left(void) // modified by bahr
{
  return nbr_pas_left;
}

/*! \brief Set the number of left motor steps
 * \param set_steps The number of changed phases that you want set.
 */
void e_set_steps_left(long int set_steps) // modified by bahr
{
  INTERRUPT_OFF();
  nbr_pas_left = set_steps;
  INTERRUPT_ON();
}

/*! \brief Give the number of right motor steps right motor
 * is running.
 */
long int e_get_steps_right(void) // modified by bahr
{
  return nbr_pas_right;
}

/*! \brief Set the number of right motor steps
 * \param set_steps The number of changed phases that you want set.
 */
void e_set_steps_right(long int set_steps) // modified by bahr
{
  INTERRUPT_OFF();
  nbr_pas_right = set_steps;
  INTERRUPT_ON();
}

/*! \brief Get the status of the open loop control
*/
int e_get_ol_ctrl_status(void) // added by bahr
{
	return ol_ctrl_status;
}

/*! \brief Travel a preset distance in [mm] forward or backward
 * \param set_distance The distance in [mm] which is to be traveled 
 * forward (+) or backward (-)
 * \param motor_speed The speed at which the robot travels. The speed is always positive.
 */
void e_set_distance(long int set_distance, int motor_speed) // added by bahr
{
	INTERRUPT_OFF();
  
	nbr_pas_left = 0;
	nbr_pas_right = 0;

	// set open loop control status to active
	ol_ctrl_status = 1;

	// computebthe number of steps for each motor
	nbr_pas_ol_ctrl_left = (long int)(((long int)STEPS_PER_M*set_distance)/1000);
	nbr_pas_ol_ctrl_right = (long int)(((long int)STEPS_PER_M*set_distance)/1000);

	// go backwards
	if (set_distance < 0)
	{
		motor_speed=-abs(motor_speed);
	}

	// go forward
	if (set_distance > 0)
	{
		motor_speed=abs(motor_speed);
	}

	// stop
	if (set_distance == 0)
	{
		ol_ctrl_status = 4;
		motor_speed = 0;
	}

	// set motor speed
	e_set_speed_left(motor_speed);
	e_set_speed_right(motor_speed);
  
	INTERRUPT_ON();
}

/*! \brief Turn a preset angle in [degrees] clockwise or counter-clockwise
 * \param turn_angle The angle in [degrees] which is to be traveled 
 * CW (-) or CCW (+)
 * \param motor_speed The speed at which the robot turns. The speed is always positive.
 */
void e_set_turn(double turn_angle, int motor_speed) // added by bahr
{
	INTERRUPT_OFF();
  
	double delta_s;

	nbr_pas_left = 0;
	nbr_pas_right = 0;

	// set open loop control status to active
	ol_ctrl_status = 1;

	// compute the distance which each wheels has to travel (in opposite direction)
	delta_s=((double)WHEEL_DISTANCE*(fabs(turn_angle)*(PI/(double)180.0)))/(double)2.0;

	// compute the number of necessary steps for the turn
	nbr_pas_ol_ctrl = (long int)(((long int)STEPS_PER_M*delta_s));

	// ensure that the motor speed is positive
	motor_speed=abs(motor_speed);
		
	// turn clock-wise
	if (turn_angle < 0)
	{
		nbr_pas_ol_ctrl_left = nbr_pas_ol_ctrl;
		nbr_pas_ol_ctrl_right = -nbr_pas_ol_ctrl;
		e_set_speed_left(motor_speed);
		e_set_speed_right(-motor_speed);
	}

	// turn counter clock-wise
	if (turn_angle > 0)
	{
		nbr_pas_ol_ctrl_left = -nbr_pas_ol_ctrl;
		nbr_pas_ol_ctrl_right = nbr_pas_ol_ctrl;
		e_set_speed_left(-motor_speed);
		e_set_speed_right(motor_speed);
	}

	// stop
	if (turn_angle == 0)
	{
		ol_ctrl_status = 4;
		e_set_speed_left(0);
		e_set_speed_right(0);
	}

	INTERRUPT_ON();
}

void e_do_dr() // added by bahr
{
	double delta_l = 0; // distance traveled by left wheel in [mm]
	double delta_r = 0; // distance traveled by right wheel in [mm]
	double delta_s = 0; // distance traveled by body center in [mm]
	double delta_theta = 0;

	double noise;

	// compute distance traveled for both wheels since last dead-reckoning step in [mm]
	delta_l = ((double)nbr_pas_left - (double)nbr_pas_left_old)*(((double)1000.0)/((double)STEPS_PER_M));
	delta_r = ((double)nbr_pas_right - (double)nbr_pas_right_old)*(((double)1000.0)/((double)STEPS_PER_M));

	// compute forward distance
	delta_s = (delta_l + delta_r)/((double)2.0);

	// compute turn radius
	delta_theta = (delta_r - delta_l)/((double)WHEEL_DISTANCE*(double)1000);

	// generate noise
	//noise_int=(rand()%200);
	noise =(double)(rand()%200);
	noise -= 100+DR_NOISE_BIAS;

	// add noise to dr position if required, otherwise, comment it out
	delta_s += delta_s*((double)DR_NOISE/(double)10000.0)*(noise);
	//delta_theta += delta_theta*((double)DR_NOISE/(double)10000.0)*(noise);
	
	// add a bias to the dead reckoned position
	delta_s += delta_s*((double)DR_BIAS/(double)100.0);
	delta_theta += delta_theta*((double)DR_BIAS/(double)100.0);
	

	// update dr position
	dr_x += delta_s * cos(dr_theta + (delta_theta/(double)2.0));
	dr_y += delta_s * sin(dr_theta + (delta_theta/(double)2.0));
	dr_theta += delta_theta;

	// check for angle 2*PI roll-over
	if(fabs(dr_theta)>((double)2.0*(double)PI))
	{
		if(dr_theta>0) dr_theta = fmod((fabs(dr_theta)),((double)2.0*(double)PI));
		else dr_theta = -fmod((fabs(dr_theta)),((double)2.0*(double)PI));
	}

	// check for angle PI roll-over
	if(fabs(dr_theta)>((double)PI))
	{
		if(dr_theta>0) 
		{
			dr_theta -= (double)2.0*(double)PI;
		}
		else 
		{
			dr_theta += (double)2.0*(double)PI;
		}
	}
	
	// store number of steps up to this point
	nbr_pas_left_old=nbr_pas_left;
	nbr_pas_right_old=nbr_pas_right;
}

void e_set_dr_position(double set_dr_x,double set_dr_y,double set_dr_heading) // added by bahr
{
	// set the dead-reckoned position, angle is povided in degrees
	dr_x = set_dr_x;
	dr_y = set_dr_y;
	dr_theta = e_deg2rad(set_dr_heading);

	// reset step counter
	nbr_pas_left_old = nbr_pas_left;
	nbr_pas_right_old = nbr_pas_right;

	// initialize the random seed
	srand(time(NULL));

}

void e_get_dr_position(double *get_dr_x,double *get_dr_y,double *get_dr_heading) // added by bahr
{
	*get_dr_x = dr_x;
	*get_dr_y = dr_y;
	*get_dr_heading = e_rad2deg(dr_theta);
}

double e_deg2rad(double angle_deg)
{
	// convert an angle in degrees, with 0 degrees along the y-axis
	// to an angle in radians with 0 rad along the x-axis
	
	double angle_rad;

	if((angle_deg>270) || (angle_deg<90))
	{
		if(angle_deg<90)
		{
			angle_rad = (90-angle_deg)*((double)PI/(double)180.0);
		}
		else
		{
			angle_rad = 90+(360-angle_deg)*((double)PI/(double)180.0);
		}
	}
	else
	{
			angle_rad = -(angle_deg-90)*((double)PI/(double)180.0);
	}

	return angle_rad;
}

double e_rad2deg(double angle_rad)
{
	// convert an angle in radians with 0 rad along the x-axis
	// to an angle in degrees, with 0 degrees along the y-axis

	double angle_deg;
	
	if(angle_rad>0)
	{
		if(angle_rad<((double)PI/(double)2.0))
		{
			angle_deg = 90-(angle_rad*((double)180.0/(double)PI));
		}
		else
		{
			angle_deg = 360-((angle_rad*((double)180.0/(double)PI))-90);
		}
	}
	else
	{
			angle_deg = (-(angle_rad*((double)180.0/(double)PI)))+90;
	}

	return angle_deg;
}
