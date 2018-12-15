#ifndef _RANGE_BEARING
#define _RANGE_BEARING

typedef enum
{
	READING = 0,
	EMIT = 1
} state_t;

/* functions */
void e_init_range_bearing(state_t state);   // to be called before starting using prox
void e_stop_range_bearing(); //Stop the timer and put pusle to 0
void print_signal(int sensor);


#endif
