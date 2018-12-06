#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"

#include "./motor_led/e_init_port.h"
#include "./motor_led/e_led.h"
#include <math.h>
#define PI 3.14159265358979

int main() {
	int i;

	//system initialization 
	e_init_port();

	//Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR=0;
		__asm__ volatile ("reset");
	}

	while(1) {
		for (i=0;i<30000;i++); //wait 30000 cycles
		e_set_led(0,2);	// make LED #0 toggle
	}

	return 0;
}

