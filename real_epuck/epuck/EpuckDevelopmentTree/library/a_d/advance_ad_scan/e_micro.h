#ifndef _MICRO
#define _MICRO


// ID of the different captor in their respective array

#define MIC0_BUFFER	0
#define MIC1_BUFFER	1
#define MIC2_BUFFER	2

/***********************************************************************
 * -------------------- Functions from micro.c -----------------------
 **********************************************************************/
int e_get_micro(unsigned int micro_id);
int e_get_micro_average(unsigned int micro_id, unsigned int filter_size);
int e_get_micro_volume (unsigned int micro_id);

#endif /*_MICRO*/

/* End of File : ad_conv.h */
