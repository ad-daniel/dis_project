#ifndef _AD_CONV
#define _AD_CONV

	/////////////////////////////////////////////////////////////
	// WARNING:  This file is to be used with dsPIC30F6014A    //
	//                     with 8x PLL                         //
	/////////////////////////////////////////////////////////////

/**********************************************************************
 * ---------------------------- Defines -------------------------------
 **********************************************************************/
// sampling frequency for the microphons
#define MIC_SAMP_FREQ 16384.0	// WARNING: must be the highest one. This is the base to calculate ad_cycle
								//16384.0 is max could also use 12288
									

// sampling frequency for the accelerometres and proximetres
#define ACC_PROX_SAMP_FREQ 256.0	// WARNING: should be a fraction of MIC_SAMP_FREQ
									//			to ensure a good timing precision
// lenght of the IR pulse in seconds
#define PULSE_LENGHT 0.0003		

//Calculation of the periodes in ad_cycle
//ad_cycle = 1/MIC_SAMP_FREQ
#define ACC_PROX_PERIOD (int)(MIC_SAMP_FREQ/ACC_PROX_SAMP_FREQ)	// 60 acc and prox periode in [ad_cycle]	
#define PULSE_PERIOD (int)(PULSE_LENGHT*MIC_SAMP_FREQ)			//  pulse length in [ad_cycle]			

//ADCS calculation to allways have the same time for an AD conversion scan  
//with a different numbers of channels
#define ADCS_3_CHAN	(int)(2.0*FCY/(MIC_SAMP_FREQ*(14+1)*3)-1)	// SAMPLETIME 3TAD
#define ADCS_5_CHAN	(int)(2.0*FCY/(MIC_SAMP_FREQ*(14+1)*5)-1)	// 
#define ADCS_6_CHAN	(int)(2.0*FCY/(MIC_SAMP_FREQ*(14+1)*6)-1)	// 

#define MIC_SAMP_NB 100	// number of microphon samples to store
#define ACC_SAMP_NB  50	// number of accelerometre samples to store


/***********************************************************************
 * -------------------- Functions from ad_conv.c -----------------------
 **********************************************************************/
void e_init_ad_scan(void);

#endif /*_AD_CONV*/

/* End of File : ad_conv.h */
