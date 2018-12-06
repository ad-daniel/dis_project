#ifndef _IR_REMOTE_CONTROLE
#define _IR_REMOTE_CONTROLE

/* defines for the keys on the remote controler
 * the numbers 0 to 9 are not defined since they are the same */
#define BOTTOMR		10 	// -/-- key
#define BOTTOMl		11 	// p<p key
#define	STANDBY		12 
#define MUTE		13
#define VOL_UP		16
#define VOL_DOWN	17
#define CHAN_UP		32
#define CHAN_DOWN	33
#define I_II		35
#define OUT_AUX_1	56



/* functions */
void e_init_remote_control(void);
void e_read_remote_control(void);

unsigned char e_get_check(void);
unsigned char e_get_address(void);
unsigned char e_get_data(void);

#endif
