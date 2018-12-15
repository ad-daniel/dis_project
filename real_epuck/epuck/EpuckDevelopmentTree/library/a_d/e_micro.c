/****************************************************************
* 			Microphone sensor of e-puck							*
*			Version 1.0 January 2006							*
*			Michael Bonani										*
*																*
****************************************************************/

#include "e_ad_conv.h"
#include "./motor_led/e_epuck_ports.h"
#include "e_micro.h"

void e_init_micro(void)
{
	e_init_ad();	// init AD converter module
}


void e_get_micro(int *m1, int *m2,int *m3)
{
	T1CONbits.TON = 0;//cut of proximity timer
	*m1=e_read_ad(MIC1);
	*m2=e_read_ad(MIC2);
	*m3=e_read_ad(MIC3);
	T1CONbits.TON = 1;
}
				
