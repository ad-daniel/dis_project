/* fonctions to control motor speed                     */
/* december 2004: first version                         */
/* Basic examples from Lucas Meier & Francesco Mondada  */
/* Adaptation, formatting and test by Francesco Mondada */

#include "../e_epuck_ports.h"
#include "e_agenda.h"
#include <stdlib.h>

/* If powersave is enabled, the motor library will not leave 
 * the motor's phase powered when running at low speed, thus reducing the heat
 * dissipation and power consumption
 * 
 * TRESHV == the "virtual" speed when powersave is enabled, the phase will 
 * be keept on 1/TRESHV seconds.
 *
 * Powersave will only be enabled for speed < MAXV
 */
#define POWERSAVE
#define TRESHV 650
#define MAXV 601

#if TRESHV <= MAXV
#error TRESHV must be higher than MAXV
#endif

/* internal variables */
static int left_speed = 0;
static int right_speed = 0;

static int left_motor_phase=0;	    // phase can be 0 to 3
static int right_motor_phase=0;    // phase can be 0 to 3

static int nbr_steps_left=0;
static int nbr_steps_right=0;

/* internal calls */
/**
 * Change left motor phase according to the left_speed signe
 *
 * @param	void
 * @return	void
 */
 void run_left_motor(void)  // interrupt for motor 1 (of two) = left motor
{
  // increment or decrement phase depending on direction

#ifdef POWERSAVE
  static int phase_on = 0;
  if(phase_on && abs(left_speed) < MAXV) {
    MOTOR1_PHA = 0;
    MOTOR1_PHB = 0;
	MOTOR1_PHC = 0;
	MOTOR1_PHD = 0;
	phase_on = 0;
	e_set_agenda_cycle(run_left_motor, 10000/abs(left_speed) - 10000/TRESHV);
	return;
  }
#endif

  if (left_speed > 0) // inverted for the two motors
  {
    nbr_steps_left++;
    left_motor_phase--;
    if (left_motor_phase < 0) left_motor_phase = 3;
  }
  else 
  {
    nbr_steps_left--;
    left_motor_phase++;
    if (left_motor_phase > 3) left_motor_phase = 0;
  }
  
  // set the phase on the port pins

  switch (left_motor_phase)
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
#ifdef POWERSAVE
  if(abs(left_speed) < MAXV) {
    phase_on = 1;
    e_set_agenda_cycle(run_left_motor,10000/TRESHV);
  }
#endif
}

/**
 * Change right motor phase according to the right_speed signe
 *
 * @param	void
 * @return	void
 */
void run_right_motor(void)  // interrupt for motor 2 (of two) = right motor
{
  // increment or decrement phase depending on direction
#ifdef POWERSAVE
  static int phase_on = 0;
  if(phase_on && abs(right_speed) < MAXV) {
    MOTOR2_PHA = 0;
    MOTOR2_PHB = 0;
	MOTOR2_PHC = 0;
	MOTOR2_PHD = 0;
	phase_on = 0;
	e_set_agenda_cycle(run_right_motor, 10000/abs(right_speed) - 10000/TRESHV);
	return;
  }
#endif
  if (right_speed < 0)
  {
	  nbr_steps_right++;
    right_motor_phase--;
    if (right_motor_phase < 0) right_motor_phase = 3;
  }
  else 
  {
    nbr_steps_right--;
    right_motor_phase++;
    if (right_motor_phase > 3) right_motor_phase = 0;
  }
  
  // set the phase on the port pins

  switch (right_motor_phase)
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
#ifdef POWERSAVE
  if(abs(right_speed) < MAXV) {
    phase_on = 1;
    e_set_agenda_cycle(run_right_motor,10000/TRESHV);
  }
#endif
}

/* ---- user calls ---- */

/**
 * Start the left and right motor's agenda*
 *
 * @param	void
 * @return	void
 */
void e_init_motors(void)
{
  e_activate_agenda(run_left_motor, 0);
  e_activate_agenda(run_right_motor, 0);
}


/**
 * Set the speed of the left motor by changing the 
 * frequency of the left motor's agenda
 *
 * @param motor_speed		IN  speed of the motor
 * @return void				OUT not used
 */
void e_set_speed_left(int motor_speed)  // motor speed in percent
{
  // speed null
  if (motor_speed == 0)
  {
    left_speed = 0;
    e_set_agenda_cycle(run_left_motor, 0);
    MOTOR1_PHA = 0;
    MOTOR1_PHB = 0;
    MOTOR1_PHC = 0;
    MOTOR1_PHD = 0;
  }
  // speed inferior to the minimum value
  else if(motor_speed < -1000)
  {
    left_speed = -1000;
    e_set_agenda_cycle(run_left_motor, (int)-10000/left_speed);
  }
  // speed superior to the maximum value
  else if(motor_speed > 1000)
  {
    left_speed = 1000;
    e_set_agenda_cycle(run_left_motor, (int) 10000/left_speed);
  }
  else
  {
    left_speed = motor_speed;
  // negative speed
	if(motor_speed < 0)
		e_set_agenda_cycle(run_left_motor, (int)-10000/motor_speed);
  // positive speed
	else
		e_set_agenda_cycle(run_left_motor, (int) 10000/motor_speed);

  }
}

/**
 * Set the speed of the right motor by changing the 
 * frequency of the right motor's agenda
 *
 * @param motor_speed		IN  speed of the motor
 * @return void				OUT not used
 */
void e_set_speed_right(int motor_speed)  // motor speed in percent
{
  // speed null
  if (motor_speed == 0)
  {
    right_speed = 0;
    e_set_agenda_cycle(run_right_motor, 0);
    MOTOR2_PHA = 0;
    MOTOR2_PHB = 0;
    MOTOR2_PHC = 0;
    MOTOR2_PHD = 0;
  }
  // speed inferior to the minimum value
  else if (motor_speed < -1000)
  {
    right_speed = -1000;
    e_set_agenda_cycle(run_right_motor, (int)-10000/right_speed);
  }
  // speed superior to the maximum value
  else if (motor_speed > 1000)
  {
    right_speed = 1000;
    e_set_agenda_cycle(run_right_motor, (int) 10000/right_speed);
  }
  else 
  {
    right_speed = motor_speed;

  // negative speed
	if(motor_speed < 0)
		e_set_agenda_cycle(run_right_motor, (int)-10000/motor_speed);
  // positive speed
	else
		e_set_agenda_cycle(run_right_motor, (int) 10000/motor_speed);

  }
}


/**
 * Set the speed of the motors according to the 
 * desired linear and angular speed.
 *
 * @param linear_speed		IN  linear speed
 * @param angular_speed		IN  angular speed (trigonometric)
 * @return void				OUT not used
 */
void e_set_speed(int linear_speed, int angular_speed)
{
	if(abs(linear_speed) + abs(angular_speed) > 1000)
		return;
	else
	{
		e_set_speed_left (linear_speed - angular_speed);
		e_set_speed_right(linear_speed + angular_speed);
	}
}

/**
 * Set a counter on the left motor for the number of steps done by the robots
 *
 * @param steps_left	IN	number of steps
 * @return void			OUT	not used
 */
void e_set_steps_left(int steps_left)
{
	nbr_steps_left=steps_left;
}

/**
 * Set a counter on the right motor for the number of steps done by the robots
 *
 * @param steps_right	IN	number of steps
 * @return void			OUT	not used
 */
void e_set_steps_right(int steps_right)
{
	nbr_steps_right=steps_right;	
}

/**
 * Return status of the counter of steps on the left motor
 *
 * @return int			OUT	the number of steps remaining
 */
int e_get_steps_left()
{
	return nbr_steps_left;
}

/**
 * Return status of the counter of steps on the right motor
 *
 * @return int			OUT	the number of steps remaining
 */
int e_get_steps_right()
{
	return nbr_steps_right;
}
