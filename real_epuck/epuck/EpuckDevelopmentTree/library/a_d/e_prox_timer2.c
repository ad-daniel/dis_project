/****************************************************************
* 			Control proximity sensor of e-puck	with timer 1	*	 		*
*			December 2004: first version                        *
*			Lucas Meier & Francesco Mondada  					*
*			Version 1.0 november 2005							*
*			Michael Bonani										*
*			Version 1.1 January 2006							*
*			Xavier Raemy										*
*																*
****************************************************************/


#include "e_ad_conv.h"
#include "../motor_led/e_epuck_ports.h"
#include "e_prox.h"

/* internal variables for prox */
static int ambient_ir[8];				 // ambient light measurement
static int ambient_and_reflected_ir[8]; // light when led is on
static int reflected_ir[8];			 // variation of light


/* internal calls for prox */
void init_tmr2(void)
{
  T2CON = 0;                    // 
  T2CONbits.TCKPS = 1;          // prescsaler = 8
  TMR2 = 0;                     // clear timer 2
  PR2 = (350.0*MICROSEC)/8.0;   // first interrupt after 350us with 8 prescaler
  IFS0bits.T2IF = 0;            // clear interrupt flag
  IEC0bits.T2IE = 1;            // set interrupt enable bit
  T2CONbits.TON = 1;            // start Timer2
}

void _ISRFAST _T2Interrupt(void)
{

// read ambient light and switch on leds in a first phase
// wait 350 us to let the phototransistor react
// read reflected light and switch off the leds in a second phase
// wait 3 ms before stating again
// repeat these two steps for the four couples of prox sensors

  static int ir_phase=0;	// phase can be 0 (ambient) or 1 (reflected)	
  static int ir_number=0;	// number goes from 0 to 3 (4 couples of sensors)	

  IFS0bits.T2IF = 0;            // clear interrupt flag

  switch (ir_number)
  {
    case 0:		// ir sensors 0 and 4
    {
      if (ir_phase == 0)
      {
        PR2 = (350.0*MICROSEC)/8.0;		// next interrupt in 350 us
		ambient_ir[0] = e_read_ad(IR0);
        ambient_ir[4] = e_read_ad(IR4);
        PULSE_IR0 = 1;			// led on for next measurement
        ir_phase = 1;			// next phase
      }
      else
      {
        PR2 = (2100.0*MICROSEC)/8.0;		// next interrupt in 3 ms
        ambient_and_reflected_ir[0] = e_read_ad(IR0);
        ambient_and_reflected_ir[4] = e_read_ad(IR4);
		reflected_ir[0] = ambient_ir[0] - ambient_and_reflected_ir[0];
		reflected_ir[4] = ambient_ir[4] - ambient_and_reflected_ir[4];
        PULSE_IR0 = 0;			// led off
        ir_phase = 0;			// reset phase
        ir_number = 1;			// next two sensors
      }
      break;
    }
    case 1:		// ir sensors 1 and 5
    {
      if (ir_phase == 0)
      {
        PR2 = (350.0*MICROSEC)/8.0;		// next interrupt in 350 us
        ambient_ir[1] = e_read_ad(IR1);
        ambient_ir[5] = e_read_ad(IR5);
        PULSE_IR1 = 1;			// led on for next measurement
        ir_phase = 1;			// next phase
      }
      else
      {
        PR2 = (2100.0*MICROSEC)/8.0;		// next interrupt in 3 ms
        ambient_and_reflected_ir[1] = e_read_ad(IR1);
        ambient_and_reflected_ir[5] = e_read_ad(IR5);
		reflected_ir[1] = ambient_ir[1] - ambient_and_reflected_ir[1];
		reflected_ir[5] = ambient_ir[5] - ambient_and_reflected_ir[5];
        PULSE_IR1 = 0;			// led off
        ir_phase = 0;			// reset phase
        ir_number = 2;			// next two sensors
      }
      break;
    }
    case 2:		// ir sensors 2 and 6
    {
      if (ir_phase == 0)
      {
        PR2 = (350.0*MICROSEC)/8.0;		// next interrupt in 350 us
        ambient_ir[2] = e_read_ad(IR2);
        ambient_ir[6] = e_read_ad(IR6);
        PULSE_IR2 = 1;			// led on for next measurement
        ir_phase = 1;			// next phase
      }
      else
      {
        PR2 = (2100.0*MICROSEC)/8.0;		// next interrupt in 3 ms
        ambient_and_reflected_ir[2] = e_read_ad(IR2);
        ambient_and_reflected_ir[6] = e_read_ad(IR6);
		reflected_ir[2] = ambient_ir[2] - ambient_and_reflected_ir[2];
		reflected_ir[6] = ambient_ir[6] - ambient_and_reflected_ir[6];
        PULSE_IR2 = 0;			// led off
        ir_phase = 0;			// reset phase
        ir_number = 3;			// next sensor
      }
      break;
    }
    case 3:		// ir sensors 3 and 7
    {
      if (ir_phase == 0)
      {
        PR2 = (350.0*MICROSEC)/8.0;		// next interrupt in 350 us
        ambient_ir[3] = e_read_ad(IR3);
        ambient_ir[7] = e_read_ad(IR7);
        PULSE_IR3 = 1;			// led on for next measurement
        ir_phase = 1;			// next phase
      }
      else
      {
        PR2 = (2100.0*MICROSEC)/8.0;		// next interrupt in 3 ms
        ambient_and_reflected_ir[3] = e_read_ad(IR3);
        ambient_and_reflected_ir[7] = e_read_ad(IR7);
		reflected_ir[3] = ambient_ir[3] - ambient_and_reflected_ir[3];
		reflected_ir[7] = ambient_ir[7] - ambient_and_reflected_ir[7];
        PULSE_IR3 = 0;			// led off
        ir_phase = 0;			// reset phase
        ir_number = 0;			// next sensor (back to beginning)
      }
      break;
    }
  }
  
}

/* ---- user calls ---- */

int e_get_prox(unsigned int sensor_number)
{
	if (sensor_number > 7)
		return 0;
	else
		return reflected_ir[sensor_number];
}

int e_get_ambient_light(unsigned int sensor_number)
{
	if (sensor_number > 7)
		return 0;
	else
		return ambient_ir[sensor_number];
}

void e_init_prox(void)
{
	e_init_ad();				// init AD converter module
	init_tmr2();				// init timer 1 for ir processing
}

