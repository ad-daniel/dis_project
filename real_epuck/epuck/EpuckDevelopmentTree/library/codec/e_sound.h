#ifndef _SOUND
#define _SOUND


/* functions */
void e_init_sound(void);
void e_play_sound(int sound_offset, int sound_length);
void e_close_sound(void);


/* DCI */
void e_init_dci_master(void);
void e_init_codec_slave(void);
void e_sub_dci_kickoff(int,int);



#endif

