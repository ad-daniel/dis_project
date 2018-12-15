#ifndef _LED
#define _LED

/* functions */
int e_get_led(unsigned int led_number);
void e_set_led(unsigned int led_number, unsigned int value); // set led_number (0-7) to value (0=off 1=on higher=inverse) 
void e_led_clear(void);

void e_set_body_led(unsigned int value); // value (0=off 1=on higher=inverse) 
void e_set_front_led(unsigned int value); //value (0=off 1=on higher=inverse) 

#endif

