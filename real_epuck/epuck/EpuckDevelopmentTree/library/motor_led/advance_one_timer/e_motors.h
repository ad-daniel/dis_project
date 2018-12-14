#ifndef _MOTORS
#define _MOTORS

/* internal functions */
//void run_left_motor(void);
//void run_right_motor(void);

/* user called function */
void e_init_motors(void); 				// init to be done before using the other calls

void e_set_speed_left(int motor_speed);  	// motor speed in percent
void e_set_speed_right(int motor_speed);	// motor speed in percent
void e_set_speed(int linear_speed, int angular_speed);

void e_set_steps_left(int steps_left);
void e_set_steps_right(int steps_right);

int e_get_steps_left();
int e_get_steps_right();

#endif
