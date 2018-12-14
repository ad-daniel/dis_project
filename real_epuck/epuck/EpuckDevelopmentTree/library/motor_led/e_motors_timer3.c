/****************************************************************
* 			Control motor of e-puck	with timer 3	 		    *
*			December 2004: first version                        *
*			Lucas Meier & Francesco Mondada                 	*
*			Version 1.0 november 2005							*
*			Michael Bonani										*
*			Version 1.1 january 2006							*
*			Xavier Raemy										*
*																*
****************************************************************/

#include <stdlib.h>
#include "e_epuck_ports.h"
#include "e_init_port.h"

/* internal variables */

static int left_speed = 0;
static int right_speed = 0;
static int nbr_pas_left = 0;
static int nbr_pas_right = 0;

static int motor_counter_left =0;
static int motor_counter_right=0;
static int motor_counter_left_init =0;
static int motor_counter_right_init=0;

/* internal calls */
 
void _ISRFAST _T3Interrupt(void) // interrupt for motor
{
  static int motor_phase_left=0;		 // phase can be 0 to 3
  static int motor_phase_right=0;

  IFS0bits.T3IF = 0;             // clear interrupt flag


  if (left_speed != 0)
  	motor_counter_left--;
  else
  {
    MOTOR1_PHA = 0;
    MOTOR1_PHB = 0;
    MOTOR1_PHC = 0;
    MOTOR1_PHD = 0;
  }
  if (right_speed != 0)
  	motor_counter_right--;
  else
  {
    MOTOR2_PHA = 0;
    MOTOR2_PHB = 0;
    MOTOR2_PHC = 0;
    MOTOR2_PHD = 0;
  }

  if (motor_counter_left <= 0) 
  {
	motor_counter_left=motor_counter_left_init;  

  	// increment or decrement phase depending on direction

  	if (left_speed > 0) // inverted for the two motors
    {
    	nbr_pas_left++;
		motor_phase_left--;
    	if (motor_phase_left < 0) motor_phase_left = 3;
    }
  	else 
    {
    	nbr_pas_left--;
    	motor_phase_left++;
    if (motor_phase_left > 3) motor_phase_left = 0;
    }
  
  	// set the phase on the port pins

 	switch (motor_phase_left)
  	{
    case 0:
      MOTOR1_PHA = 0;
      MOTOR1_PHB = 1;
      MOTOR1_PHC = 0;
      MOTOR1_PHD = 1;
      break;
    case 1:
      MOTOR1_PHA = 0;
      MOTOR1_PHB = 1;
      MOTOR1_PHC = 1;
      MOTOR1_PHD = 0;
      break;
    case 2:
      MOTOR1_PHA = 1;
      MOTOR1_PHB = 0;
      MOTOR1_PHC = 1;
      MOTOR1_PHD = 0;
      break;
    case 3:
      MOTOR1_PHA = 1;
      MOTOR1_PHB = 0;
      MOTOR1_PHC = 0;
      MOTOR1_PHD = 1;
      break;
  	}
  }

	
  if (motor_counter_right <= 0) 
  {
	motor_counter_right=motor_counter_right_init; 
  	if (right_speed < 0)
    {
      nbr_pas_right--;
      motor_phase_right--;
      if (motor_phase_right < 0) motor_phase_right = 3;
    }
    else 
    {
      nbr_pas_right++;
      motor_phase_right++;
      if (motor_phase_right > 3) motor_phase_right = 0;
    }
  
  	// set the phase on the port pins

  	switch (motor_phase_right)
  	{
    case 0:
      MOTOR2_PHA = 0;
      MOTOR2_PHB = 1;
      MOTOR2_PHC = 0;
      MOTOR2_PHD = 1;
      break;
    case 1:
      MOTOR2_PHA = 0;
      MOTOR2_PHB = 1;
      MOTOR2_PHC = 1;
      MOTOR2_PHD = 0;
      break;
    case 2:
      MOTOR2_PHA = 1;
      MOTOR2_PHB = 0;
      MOTOR2_PHC = 1;
      MOTOR2_PHD = 0;
      break;
    case 3:
      MOTOR2_PHA = 1;
      MOTOR2_PHB = 0;
      MOTOR2_PHC = 0;
      MOTOR2_PHD = 1;
      break;
  	}
  }


}



/* ---- user calls ---- */

int e_get_steps_left(void)
{
  return nbr_pas_left;
}

void e_set_steps_left(int set_steps)
{
  INTERRUPT_OFF();
  nbr_pas_left = set_steps;
  INTERRUPT_ON();
}

int e_get_steps_right(void)
{
  return nbr_pas_right;
}

void e_set_steps_right(int set_steps)
{
  INTERRUPT_OFF();
  nbr_pas_right = set_steps;
  INTERRUPT_ON();
}

void e_set_speed_left(int motor_speed)  // motor speed in steps/s
{
	INTERRUPT_OFF();
	if (motor_speed != 0)
	{
		if (motor_speed > 999)
			motor_speed = 999;
		if (motor_speed <= -999)
			motor_speed = -999; 
		motor_counter_left=abs(10000/motor_speed);
	}
	else 
		motor_counter_left=9999;	
	motor_counter_left_init = motor_counter_left;
	left_speed = motor_speed;
	INTERRUPT_ON();	
}


void e_set_speed_right(int motor_speed)  // motor speed in steps/s
{
	INTERRUPT_OFF();
	if (motor_speed != 0)
	{
		if (motor_speed > 999)
			motor_speed = 999;
		if (motor_speed < -999)
			motor_speed = -999;
		motor_counter_right=abs(10000/motor_speed);
	}
	else 
		motor_counter_right=9999;	
	motor_counter_right_init = motor_counter_right;	
	right_speed = motor_speed;
	INTERRUPT_ON();
}


void e_init_motors(void)
{
    T3CONbits.TON = 0;            // stop Timer5
  	InitPort();					  // init general ports
    left_speed = 0;
	right_speed = 0;
	motor_counter_left_init=9999;
	motor_counter_right_init=9999;
	motor_counter_left=9999;
	motor_counter_right=9999;	

    T3CON = 0;                    // 
    T3CONbits.TCKPS=3;            // prescsaler = 256
    TMR3 = 0;                     // clear timer 3
    PR3 = (FCY/256)/10000;  	  // interrupt every 0.1ms with 256 prescaler
    IFS0bits.T3IF = 0;            // clear interrupt flag
    IEC0bits.T3IE = 1;            // set interrupt enable bit
    T3CONbits.TON = 1;            // start Timer5

}

