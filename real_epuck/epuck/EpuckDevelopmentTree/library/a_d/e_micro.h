#ifndef _MICROPHONE
#define _MICROPHONE


/* functions */

void e_init_micro(void);   // to be called before starting using microphone
void e_get_micro(int *m1, int *m2,int *m3); // to get analog value of microphone
#endif
