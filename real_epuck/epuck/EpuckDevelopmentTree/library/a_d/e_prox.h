#ifndef _PROX
#define _PROX


/* functions */

void e_init_prox(void);   // to be called before starting using prox
void e_stop_prox(); //Stop the timer and put pusle to 0
int e_get_prox(unsigned int sensor_number); // to get a prox value
int e_get_ambient_light(unsigned int sensor_number); // to get ambient light value


#endif
