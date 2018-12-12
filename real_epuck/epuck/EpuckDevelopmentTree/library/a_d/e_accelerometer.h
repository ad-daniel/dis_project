#ifndef _ACCELEROMETER
#define _ACCELEROMETER


/* functions */

void e_init_acc(void);   // to be called before starting using accelerometer
void e_get_acc(int *x, int *y,int *z); // to get analog value of accelerometer
#endif
