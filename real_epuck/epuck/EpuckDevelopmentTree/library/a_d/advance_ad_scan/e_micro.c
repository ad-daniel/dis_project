//START_HEADER
/*
; --------------------------------------------------------------------------
; file name : micro.c
; Version : 0.0a 
; Date :07/11/05
; Conceptor : Borter Jean-Joël
; Made for : dsPIC30F6014
; --------------------------------------------------------------------------
; File fonction and description
; --------------------------------------------------------------------------
;
;		
; --------------------------------------------------------------------------
; --------------------------------------------------------------------------
*/
//END_HEADER

#include "p30f6014a.h"
#include "e_ad_conv.h"
#include "../../motor_led/e_epuck_ports.h"

/*****************************************************
 * external variables                                *
 *****************************************************/
extern int e_mic_scan[3][MIC_SAMP_NB];
extern unsigned int e_last_mic_scan_id;


/*****************************************************
 * user function                                     *
 *****************************************************/

/**
 * Get the last value of a given micro.
 *
 * @param micro_id		IN	micro's ID (0, 1, or 2)
 *							(use MICR0, MICR1, MICR2 defined in ad_conv.h)
 * @return result		OUT last value of the micro
 */
int e_get_micro (unsigned int micro_id)
{
	return e_mic_scan[micro_id][e_last_mic_scan_id];
}


/**
 * Get the average on a given number of sample from a micro
 *
 * @param micro_id		IN	micro's ID (0, 1, or 2)
 *							(use MICR0, MICR1, MICR2 defined in ad_conv.h)
 * @param filter_size	IN	number of sample to average
 * @return result		OUT last value of the micro
 */
int e_get_micro_average(unsigned int micro_id, unsigned int filter_size)
{
	long temp = 0;
	int i,j;

	// channel ID must be between 0 to 2 and 
	// filter_size must be between 1 to SAMPLE_NUMBER
	if ((micro_id < 3) && 
		(filter_size > 0) && (filter_size <= MIC_SAMP_NB))
	{
		for (i=0, j=(MIC_SAMP_NB-(filter_size-(e_last_mic_scan_id+1)))%MIC_SAMP_NB ; i<filter_size ; i++, j=(j+1)%MIC_SAMP_NB)
		{
			temp += e_mic_scan[micro_id][j];
		}
	}
	return ((int)(temp/filter_size));
}


/**
 * Get the difference between the highest and lowest sample.
 *
 * @param micro_id		IN	micro's ID (0, 1, or 2)
 *							(use MICR0, MICR1, MICR2 defined in ad_conv.h)
 * @return result		OUT volume
 */
int e_get_micro_volume (unsigned int micro_id)
{
	int i, highest = 0, lowest = 4999;

	for (i=0; i<MIC_SAMP_NB; i++)
	{
		if (e_mic_scan[micro_id][i] > highest)
		{
			highest = e_mic_scan[micro_id][i];
		}
		else if (e_mic_scan[micro_id][i] < lowest)
		{
			lowest = e_mic_scan[micro_id][i];
		}
	}
	return (highest - lowest);
}


/**
 * Write to a given array, the last values of one micro. The values are 
 * stored with the last one first, and the older one at the end of the array.
 * [ t ][ t-1 ][ t-2 ][ t-3 ]...[ t-(samples_nb-1) ][ t-samples_nb ]
 *
 * @param micro_id		IN	micro's ID (0, 1, or 2)
 *							(use MICR0, MICR1, MICR2 defined in ad_conv.h)
 * @param *result		IN	pointer on the result array
 * @param samples_nb	IN	size of the result array
 *							(must be between 1 and MIC_SAMP_NB)
 * @return void			OUT not used
 */

void e_get_micro_last_values (int micro_id, int * result, unsigned samples_nb)
{
	int i;
	if (samples_nb > 0 && samples_nb <= MIC_SAMP_NB)
	{
		for (i=0 ; i<samples_nb ; i++)
		{
			result[samples_nb-1 - i] = e_mic_scan[micro_id][(e_last_mic_scan_id + i) % MIC_SAMP_NB];
		}
	}
}
