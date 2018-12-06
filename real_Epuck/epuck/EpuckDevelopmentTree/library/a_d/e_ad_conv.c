/****************************************************************
* 			Functions to get values from the ADC module	 		*
*			December 2004: first version						*
*			Basic examples from Microchip                       *
*			Lucas Meier & Francesco Mondada  					*
*			Version 1.0 november 2005							*
*			Michael Bonani										*
*																*
****************************************************************/


//#include "p30f6014a.h"
#include "../motor_led/e_epuck_ports.h"

void e_init_ad(void)
{
  ADCON1 = ADCON2 = ADCON3 = 0;
  // ADPCFGbits.PCFGx 
  // = 0 for Analog input mode, 
  // = 1 for digital input mode (default)
  ADPCFGbits.PCFG0 = 1; // Debugger 
  ADPCFGbits.PCFG1 = 1; // Debugger 
  ADPCFGbits.PCFG2 = 0; // mic1
  ADPCFGbits.PCFG3 = 0; // mic2
  ADPCFGbits.PCFG4 = 0; // mic3
  ADPCFGbits.PCFG5 = 0;	// axe x acc.
  ADPCFGbits.PCFG6 = 0; // axe y acc.
  ADPCFGbits.PCFG7 = 0; // axe z acc.
  ADPCFGbits.PCFG8 = 0; // ir0
  ADPCFGbits.PCFG9 = 0; // ir1
  ADPCFGbits.PCFG10 = 0;  // ir2
  ADPCFGbits.PCFG11 = 0;  // ir3
  ADPCFGbits.PCFG12 = 0;  // ir4
  ADPCFGbits.PCFG13 = 0;  // ir5
  ADPCFGbits.PCFG14 = 0;  // ir6
  ADPCFGbits.PCFG15 = 0;  // ir7
  ADCON3 = (2*667/TCY_PIC)-1; //ADCS sampling time Tad minimum 667ns    
  ADCHS =0x0007;
  ADCON1bits.ADON = 1;
}
// Function to sample an AD channel
int e_read_ad(unsigned int channel)
{
  int delay;
  if(channel > 0x000F) return(0);
  ADCHS = channel;
  ADCON1bits.SAMP = 1;
  for(delay = 0; delay < 40; delay++);
  IFS0bits.ADIF = 0;
  ADCON1bits.SAMP = 0;
  while(!ADCON1bits.DONE);
  return(ADCBUF0);
}
 
// Example of use:
//
//  unsigned int AD_Value1;
//
//  AD_Value1 = read_ad(7);
 
