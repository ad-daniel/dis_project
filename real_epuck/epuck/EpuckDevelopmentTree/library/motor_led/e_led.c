/****************************************************************
* 			fonctions for simple LED manipulation      		 	*	*
*			december 2004: first example for microinfo  		*
*			Version 1.0 november 2005							*
*			Michael Bonani, Francesco Mondada					*
*																*
****************************************************************/ 
 
#include "e_epuck_ports.h"
#include "e_led.h"

int e_get_led(unsigned int led_number)
{
	switch(led_number)
	{
		case 0: 
            return LED0;
			break;
		case 1: 
            return LED1;
			break;
		case 2: 
            return LED2;
			break;
		case 3: 
            return LED3;
			break;
		case 4: 
            return LED4;
			break;
		case 5: 
            return LED5;
			break;
		case 6: 
            return LED6;
			break;
		case 7: 
            return LED7;
			break;
		default: 
            return 0;
	}
}

void e_set_led(unsigned int led_number, unsigned int value)
// led_number between 0 and 7, value 0 (off) or 1 (on)
// if led_number other than 0-7, all leds are set to value
{
	switch(led_number)
	{
		case 0: 
			{
			if(value>1)
				LED0 = LED0^1; // "^" exclusif OR bit to bit
			else
				LED0 = value;
			break;
			}
		case 1: 
			{
			if(value>1)
				LED1 = LED1^1;
			else
				LED1 = value;
			break;
			}
		case 2: 
			{
			if(value>1)
				LED2 = LED2^1;
			else
				LED2 = value;
			break;
			}
		case 3: 
			{
			if(value>1)
				LED3 = LED3^1;
			else
				LED3 = value;
			break;
			}
		case 4: 
			{
			if(value>1)
				LED4 = LED4^1;
			else
				LED4 = value;
			break;
			}
		case 5: 
			{
			if(value>1)
				LED5 = LED5^1;
			else
				LED5 = value;
			break;
			}
		case 6: 
			{
			if(value>1)
				LED6 = LED6^1;
			else
				LED6 = value;
			break;
			}
		case 7: 
			{
			if(value>1)
				LED7 = LED7^1;
			else
				LED7 = value;
			break;
			}
		default: 
			LED0 = LED1 = LED2 = LED3 = LED4 = LED5 = LED6 = LED7 = value;
	}
}


void e_set_body_led(unsigned int value)
{
	if(value>1)
		BODY_LED = BODY_LED^1;
	else
		BODY_LED = value;
}

void e_set_front_led(unsigned int value)
{
	if(value>1)
		FRONT_LED = FRONT_LED^1;
	else
	FRONT_LED = value;
}

void e_led_clear(void)
{
	LED0 = 0;
	LED1 = 0;
	LED2 = 0;
	LED3 = 0;
	LED4 = 0;
	LED5 = 0;
	LED6 = 0;
	LED7 = 0;
}
