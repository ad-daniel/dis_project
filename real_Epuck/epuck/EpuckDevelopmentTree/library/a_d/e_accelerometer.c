/****************************************************************
* 			Accelerometer sensor of e-puck						*
*			Version 1.0 november 2005							*
*			Michael Bonani										*
*																*
****************************************************************/

#include "e_ad_conv.h"
#include "../motor_led/e_epuck_ports.h"
#include "e_accelerometer.h"


void e_init_acc(void)
{
	e_init_ad();	// init AD converter module
}


void e_get_acc(int *x, int *y,int *z)
{
	T1CONbits.TON = 0;//cut of proximity timer
	*x=e_read_ad(ACCX);
	*y=e_read_ad(ACCY);
	*z=e_read_ad(ACCZ);
	T1CONbits.TON = 1;
}
				
