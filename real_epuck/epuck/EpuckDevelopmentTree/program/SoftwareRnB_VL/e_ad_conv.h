#ifndef _AD_CONV
#define _AD_CONV

#define SAMPLING_WINDOW 30
#define NB_IR_SENSORS 8

/***********************************************************************
 * -------------------- Functions from ad_conv.c -----------------------
 **********************************************************************/
void e_init_ad_scan(void);
void e_ad_scan_on(void);
void e_ad_scan_off(void);
char e_ad_is_array_filled(void);

#endif /*_AD_CONV*/

/* End of File : ad_conv.h */
