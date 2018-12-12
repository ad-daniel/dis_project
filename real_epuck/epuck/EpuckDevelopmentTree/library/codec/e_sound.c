/* fonctions to set sound*/
/* august 2005: first version                        
 ---- user calls ---- */
#include "../motor_led/e_epuck_ports.h"
#include "e_sound.h"

void e_init_sound(void)  // init sound module using si3000 codec
{
	int i;
	AUDIO_ON = 1;
	for (i=0;i<10000;i++);
	e_init_dci_master();
	e_init_codec_slave();	
}


void e_play_sound(int sound_nbr, int sound_length)
{
	__asm__ volatile ("setm    _e_dci_unavailable"); // set DCI as used
	__asm__ volatile ("clr	   _e_stop_flag");


	e_sub_dci_kickoff(sound_nbr,sound_length);
}

void e_close_sound(void)  // init sound module using si3000 codec
{	
	AUDIO_ON = 0;
}
