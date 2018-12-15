/**********************************************************************/
/* Fonctions to get values from the ADC module                        */
/*                                                                    */
/* Author: Borter Jean-Jo�l                                           */
/* december 2005: first version						  */
/* april 2006: debug and optimisation Michael Bonani                  */
/*                                                                    */
/* Purpose:	The AD converter module is set to operate by itself.	*/
/*          It will scan the AD channels and at the end an iterrupt   */
/*          will be triggered to unload the buffer register in an     */
/*          array. This arrray should only be accessed by functions   */
/*          like:     get_ad                                          */
/*                    get_ad_filtered.                                */
/**********************************************************************/

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_led.h"
#include "e_ad_conv.h"
#include "e_range_bearing.h"
#include "e_uart_char.h"

#include <stdio.h>
#include <string.h>

int received_signal[NB_IR_SENSORS][SAMPLING_WINDOW];
int sampling_signal[NB_IR_SENSORS][SAMPLING_WINDOW];

static char array_filled = 0;
unsigned int e_last_ir_scan_id = 0;	//ID of the last scan in the mic array (must be int else probleme of overchange)

/**
 * Set up the different ADC register to process the AD conversion
 * by scanning the used AD channels. Each value of the channels will
 * be stored in a different AD buffer register and an inturrupt will
 * occure at the end of the scan.
 *
 * @param  void
 * @return void
 */


void e_init_ad_scan(void)
{
	ADCON1 = 0;		//reset to default value
	ADCON2 = 0;		//reset to default value
	ADCON3 = 0;		//reset to default value
	ADCHS = 0;		//reset to default value

	// ADPCFGbits.PCFGx 
	// = 0 for Analog input mode, 
	// = 1 for digital input mode (default)
	ADPCFGbits.PCFG0 = 1;   // Debugger 
	ADPCFGbits.PCFG1 = 1;   // Debugger 
	ADPCFGbits.PCFG2 = 0;   // micro 0
	ADPCFGbits.PCFG3 = 0;   // micro 1
	ADPCFGbits.PCFG4 = 0;   // micro 2
	ADPCFGbits.PCFG5 = 0;   // axe x acc
	ADPCFGbits.PCFG6 = 0;   // axe y acc
	ADPCFGbits.PCFG7 = 0;   // axe z acc
	ADPCFGbits.PCFG8 = 0;   // ir0
	ADPCFGbits.PCFG9 = 0;   // ir1
	ADPCFGbits.PCFG10 = 0;  // ir2
	ADPCFGbits.PCFG11 = 0;  // ir3
	ADPCFGbits.PCFG12 = 0;  // ir4
	ADPCFGbits.PCFG13 = 0;  // ir5
	ADPCFGbits.PCFG14 = 0;  // ir6
	ADPCFGbits.PCFG15 = 0;  // ir7

	//specifie the channels to be scanned
	ADCSSLbits.CSSL0 = 0;   // Debugger
	ADCSSLbits.CSSL1 = 0;   // Debugger
	ADCSSLbits.CSSL2 = 0;   // micro 0
	ADCSSLbits.CSSL3 = 0;   // micro 1
	ADCSSLbits.CSSL4 = 0;   // micro 2
	ADCSSLbits.CSSL5 = 0;   // axe x acc
	ADCSSLbits.CSSL6 = 0;   // axe y acc
	ADCSSLbits.CSSL7 = 0;   // axe z acc
	ADCSSLbits.CSSL8 = 1;   // ir0
	ADCSSLbits.CSSL9 = 1;   // ir1
	ADCSSLbits.CSSL10 = 1;  // ir2
	ADCSSLbits.CSSL11 = 1;  // ir3
	ADCSSLbits.CSSL12 = 1;  // ir4
	ADCSSLbits.CSSL13 = 1;  // ir5
	ADCSSLbits.CSSL14 = 1;  // ir6
	ADCSSLbits.CSSL15 = 1;  // ir7

	ADCON1bits.FORM = 0;	//output = unsigned int
	ADCON1bits.ASAM = 1;	//automatic sampling on
	ADCON1bits.SSRC = 7;	//automatic convertion mode

	ADCON2bits.SMPI = 8-1;	//interupt on 8th sample
	ADCON2bits.CSCNA = 1;	//scan channel input mode on
	
	ADCON3bits.SAMC = 1;	//number of cycle between acquisition and conversion (need 2 for the prox)
	ADCON3bits.ADCS = 19;	//Tad = (ADCS + SAMC) * Tcy/2 = 2170[ns], 
				//WARNING: Tad min must be 667 [ns]
				//WARNING: MAX 63 !!

	IFS0bits.ADIF = 0;	//Clear the A/D interrupt flag bit
	IEC0bits.ADIE = 1;	//Set the A/D interrupt enable bit

	ADCON1bits.ADON = 1;	//enable AD conversion
}

void e_ad_scan_on(void)
{
	ADCON1bits.ADON = 1;	//enable AD conversion
}

void e_ad_scan_off(void)
{
	ADCON1bits.ADON = 0;	//enable AD conversion
}

static void inline switch_signals()
{
	int i = 0, j = 0;
	for (i=0; i<NB_IR_SENSORS; i++)
		for(j=0; j<SAMPLING_WINDOW; j++)
			received_signal[i][j] = sampling_signal[i][j];
}


/**
 * Save the AD buffer registers in differents arrays
 *
 * @param  void
 * @return void
 */
void __attribute__((__interrupt__)) _ADCInterrupt(void)
{
	volatile unsigned int * adc_ptr;
	static int first_sample = 1;
	
	//Clear the A/D Interrupt flag bit or else the CPU will
	//keep vectoring back to the ISR
	IFS0bits.ADIF = 0;

	//////////////////////////////////////
	//  Copy of the buffer regs in the  //
	//        approprieted array        //
	//////////////////////////////////////
	adc_ptr = &ADCBUF0;

	sampling_signal[0][e_last_ir_scan_id] = *adc_ptr++;
	sampling_signal[1][e_last_ir_scan_id] = *adc_ptr++;
	sampling_signal[2][e_last_ir_scan_id] = *adc_ptr++;
	sampling_signal[3][e_last_ir_scan_id] = *adc_ptr++;
	sampling_signal[4][e_last_ir_scan_id] = *adc_ptr++;
	sampling_signal[5][e_last_ir_scan_id] = *adc_ptr++;
	sampling_signal[6][e_last_ir_scan_id] = *adc_ptr++;
	sampling_signal[7][e_last_ir_scan_id] = *adc_ptr++;

	// The first sample of the AD is a bit strange... Don't take it
	if(!first_sample)
		e_last_ir_scan_id++;
	
	first_sample = 0;

	// Array full ?
	if(e_last_ir_scan_id>= SAMPLING_WINDOW) {
		int i=0;
		e_last_ir_scan_id = 0;
		array_filled = 1;
		switch_signals();
		for (i=0; i<NB_IR_SENSORS; i++)
			print_signal(i);
		first_sample = 1;
	}
}

char e_ad_is_array_filled(void)
{
	char result;
	// Mutex array_filled to prevent unwanted changes
	INTERRUPT_OFF();
	result = array_filled;
	array_filled = 0;
	INTERRUPT_ON();
	NOP();
	NOP();
	NOP();
	NOP();
	NOP();
	return result;
}
