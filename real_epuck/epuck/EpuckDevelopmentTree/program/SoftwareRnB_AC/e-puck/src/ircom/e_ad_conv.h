
#ifndef _AD_CONV
#define _AD_CONV


/***********************************************************************
 * -------------------- Functions from ad_conv.c -----------------------
 **********************************************************************/
void e_init_ad_scan(void);
void e_ad_scan_on(void);
inline void e_ad_scan_reset(void);
inline void e_ad_skip_samples (int samples_count);
void e_ad_scan_off(void);
void e_ad_ircom_interrupt();
void e_ad_proximity_interrupt();
inline char e_ad_is_array_filled(void);
inline int e_get_prox(int s);
inline int e_get_calibrated_prox(int s);
inline int e_get_ambient_light(int s);
void e_calibrate_ir();

#define SAMPLING_WINDOW 32
#define NB_IR_SENSORS 8
#define ADCS_SETTING 19

extern int* ad_sampling;
extern int* ad_received;
extern volatile unsigned int e_last_ir_scan_id;

extern volatile int ad_activate_proximity;

extern volatile int ad_disable_proximity;
extern volatile int ad_disable_ircom;

extern volatile int e_ambient_ir[NB_IR_SENSORS]; // ambient light measurement
extern volatile int e_ambient_and_reflected_ir[NB_IR_SENSORS]; // light when led is on
extern volatile int e_reflected_ir[NB_IR_SENSORS]; // variation of light
extern volatile int e_init_value_ir[NB_IR_SENSORS];


#endif /*_AD_CONV*/

/* End of File : ad_conv.h */
