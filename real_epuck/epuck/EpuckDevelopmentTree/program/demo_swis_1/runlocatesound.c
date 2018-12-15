
#include "runlocatesound.h"

#include "p30f6014a.h"
#include <math.h>
#include "../../../library/motor_led/e_motors.h"

#define PI 3.1415

/* defines used in main.c                          */
#define MEAN_MAX 100		// Length of the mean_table
#define MEAN_MAX_INV 0.01	// Inversed value of MEAN_MAX
#define PERCENT 0.1			// Defines range to distinguish between noise an signal

/* defines used in ad_conv_int.c					*/
#define MIC_SAMP_NB 600		// number of microphon samples to store

/* defines used in find_direction					*/
//maximum_delta_t = 
//sampling_frequency[Hz] * distance_between_microphones[m] / speed_of_sound[m/s];
#define MAXIMUM_DELTA_T1 6.
#define MAXIMUM_DELTA_T2 5.

/* defines used in find_delta_t.c					*/
#define TAU_RANGE 14		// Needs to be a pair number

/* defines used in turn_to_direction.c */
#define TURN_SPEED 1000
#define STEPS_FOR_2PI 1300.




int mic_scan[3][MIC_SAMP_NB];		//Array to store the mic values
unsigned int last_mic_scan_ID = 0;	//ID of the last scan in the mic array
int new_sample;
float mean_table[3][MEAN_MAX];
int mean_nb;
float mean[3];
float signal_max[3], signal_min[3];


void calculate_average(int *);
void init_sound(void);
void record_sound(void);
int check_for_event(void);
void filter_signal(void);
void calculate_average(int *current_sample);
int get_micro (unsigned int micro_ID);
int get_micro_average(unsigned int micro_ID, unsigned int filter_size);
float calculate_direction(void);
int find_delta_t(int mic1_nb,int mic2_nb);
void show_led(float angle);
void turn_to_direction(float direction);



/**********************************************************************
 * Set up the different ADC register to process the AD conversion
 * by scanning the used AD channels. Each value of the channels will
 * be stored in a different AD buffer register and an inturrupt will
 * occure at the end of the scan.
 *
 * @param  void
 * @return void
 **********************************************************************/
void init_ad_scan(void)
{
	ADCON1 = 0;						//reset to default value
	ADCON2 = 0;						//reset to default value
	ADCON3 = 0;						//reset to default value
	ADCHS = 0;						//reset to default value

	// ADPCFGbits.PCFGx 
	// = 0 for Analog input mode, 
	// = 1 for digital input mode (default)
	ADPCFGbits.PCFG0 = 1;   // Debugger 
	ADPCFGbits.PCFG1 = 1;   // Debugger 
	ADPCFGbits.PCFG2 = 0;   // micro 0
	ADPCFGbits.PCFG3 = 0;   // micro 1
	ADPCFGbits.PCFG4 = 0;   // micro 2
	ADPCFGbits.PCFG5 = 0;   // axe x acc
	ADPCFGbits.PCFG6 = 0;   // axe y acc
	ADPCFGbits.PCFG7 = 0;   // axe z acc
	ADPCFGbits.PCFG8 = 0;   // ir0
	ADPCFGbits.PCFG9 = 0;   // ir1
	ADPCFGbits.PCFG10 = 0;  // ir2
	ADPCFGbits.PCFG11 = 0;  // ir3
	ADPCFGbits.PCFG12 = 0;  // ir4
	ADPCFGbits.PCFG13 = 0;  // ir5
	ADPCFGbits.PCFG14 = 0;  // ir6
	ADPCFGbits.PCFG15 = 0;  // ir7

	//specifie the channels to be scanned
	ADCSSLbits.CSSL0 = 0;   // Debugger
	ADCSSLbits.CSSL1 = 0;   // Debugger
	ADCSSLbits.CSSL2 = 1;   // micro 0
	ADCSSLbits.CSSL3 = 1;   // micro 1
	ADCSSLbits.CSSL4 = 1;   // micro 2
	ADCSSLbits.CSSL5 = 0;   // axe x acc
	ADCSSLbits.CSSL6 = 0;   // axe y acc
	ADCSSLbits.CSSL7 = 0;   // axe z acc
	ADCSSLbits.CSSL8 = 0;   // ir0
	ADCSSLbits.CSSL9 = 0;   // ir1
	ADCSSLbits.CSSL10 = 0;  // ir2
	ADCSSLbits.CSSL11 = 0;  // ir3
	ADCSSLbits.CSSL12 = 0;  // ir4
	ADCSSLbits.CSSL13 = 0;  // ir5
	ADCSSLbits.CSSL14 = 0;  // ir6
	ADCSSLbits.CSSL15 = 0;  // ir7

	ADCON1bits.FORM = 0;	//output = unsigned int
	ADCON1bits.ASAM = 1;	//automatic sampling on
	ADCON1bits.SSRC = 7;	//automatic convertion mode

	ADCON2bits.SMPI = 3-1;	//interupt on sample
	ADCON2bits.CSCNA = 1;	//scan channel input mode on
	
	ADCON3bits.SAMC = 1;	//number of cycle between acquisition and conversion
	ADCON3bits.ADCS = 19;	//Tad = (ADCS + 1) * Tcy/2 = 678[ns], (samp_freq = 98'304 Hz) 
							//WARNING: Tad min must be 667 [ns]

	IFS0bits.ADIF = 0;		//Clear the A/D interrupt flag bit
	IEC0bits.ADIE = 1;		//Set the A/D interrupt enable bit

	ADCON1bits.ADON = 1;	//enable AD conversion
}


/**********************************************************************
 * Save the AD buffer registers in differents arrays
 *
 * @param  void
 * @return void
 **********************************************************************/
void __attribute__((__interrupt__)) _ADCInterrupt(void)
{
	
	unsigned char i = 0;
	volatile unsigned int * adcPtr;
	static unsigned int j = 0;	// ID of the next mic scan
  
	//Clear the A/D Interrupt flag bit or else the CPU will
	//keep vectoring back to the ISR
	IFS0bits.ADIF = 0;

	//////////////////////////////////////
	//  Copy of the buffer regs in the  //
	//  approprieted array              //
	//////////////////////////////////////
	adcPtr = &ADCBUF0;

	//mic channels are always copied
	
for (i=0;i<3;i++)
	{
		mic_scan[i][j] = *adcPtr++;
	}
	
	last_mic_scan_ID = j;
	if (++j >= MIC_SAMP_NB)
		j=0;

	new_sample = 0; // indicate a new sample taken
}




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		init sound
//
// Initialize everything necessary to record sound
//
// in:  void
// out: void
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void init_sound(void)  
{
	int j, k;
	
	mean_nb = 0;			// start to fill up the table from position 0
	new_sample = 1;			// no sample has been taken
	
	while(new_sample); 		// wait until a new sample has been taken
	new_sample = 1;    		// reset the value to 1


	// initialize the table of average values
	// save signal level
	for (k=0; k<3; k++)
	{
		mean[k]=(float)get_micro((unsigned) k);     // start at a given average value
	}
	
    for( k=0; k<3; k++)
    {
		for (j=0; j<MEAN_MAX; j++)      // fill the mean_table with predefined values
		{
			mean_table[k][j]= mean[k] * MEAN_MAX_INV;
		}

	signal_min[k] = mean[k] - PERCENT * mean[k];	// predefine level for eventdetecting
	signal_max[k] = mean[k] + PERCENT * mean[k];
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		record sound
//
// Fills up the memory with the recorded sound from all
// three microphones at a sampling rate of about 25kHz
// 
// in:  void
// out: void
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void record_sound(void)
{
	last_mic_scan_ID = 0;   // reset the scan_ID to 0 so that we have the full table with the sound
	while ( last_mic_scan_ID <= (MIC_SAMP_NB - 2) ); //wait until the table is full
	
	// if table is filled up, disable conversion and start calculation
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		check for event
//
// Checks if an event has happened. This function dynamically 
// takes the average of the soundstream and then checks, whether
// it is tresspassing a predefined treshold.
// 
// in:  void
// out: int (1: no event has occured)
//          (0: an event has occured!)
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int check_for_event(void)
{
	int not_event;  
	int current_sample[3];
	
	// get one single sample for all 3 microphones
	current_sample[0]=get_micro((unsigned) 0);
	current_sample[1]=get_micro((unsigned) 1);
	current_sample[2]=get_micro((unsigned) 2);

    // Detect event on any of the 3 microphones
	not_event = (  ((current_sample[0]<signal_max[0]) && (current_sample[0]>signal_min[0])) ||
				   ((current_sample[1]<signal_max[1]) && (current_sample[1]>signal_min[1])) ||
				   ((current_sample[2]<signal_max[2]) && (current_sample[2]>signal_min[2])) ); 
					
	calculate_average(current_sample);		// dynamically calculates the new average value of the noise

	return not_event;   					// if no event, return 1 
	                    					// if event, return 0
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		filter signal
//
// Filters the signal, so that the detect_direction module 
// has an optimum signal.
// This includes shifting the signal around zero and then
// takes the absolute value.
// 
// in:  void
// out: void
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void filter_signal(void)
{
	int i, k;

	for (i=0; i<3; i++)  							// for all three mics
	{
		for (k = 0; k < MIC_SAMP_NB; k++)			// for the whole signal
		{
			mic_scan[i][k] -= mean[i];				// shift the signal down to around 0
			if (mic_scan[i][k] < 0)			        // take the absolute value 
				mic_scan[i][k] = -mic_scan[i][k];   // --> gives better values in the cross-correlation	
		}
	}
}	



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//		Private functions
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		calculate average
//
// Dynamically calculates the average of the sound stream. 
// This is necessary because the average signal coming from the 
// ADC is not always the same for all 3 microphones and tests
// have shown that it may shift during the exexution of the program
// 
// in:  pointer to the current sample
// out: void
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void calculate_average(int *current_sample)
{
	int k;
	
	while(new_sample); 				// wait until a new sample has been taken
	new_sample = 1;    				// reset the value to 1
	
	for( k=0; k<3; k++)				// for all three mics calculate average
    {
		mean[k] = mean[k] - mean_table[k][mean_nb];
		mean_table[k][mean_nb] = MEAN_MAX_INV * (float)current_sample[k];
		mean[k] += mean_table[k][mean_nb];
				
		// adapt treshold to detect an event
		signal_min[k] = mean[k] - PERCENT * mean[k];
		signal_max[k] = mean[k] + PERCENT * mean[k];
	}
	
	// ensure a circular memory usage
	if (mean_nb<MEAN_MAX-1)
		mean_nb++;
	else
		mean_nb = 0;
}




/**********************************************************************
 * Get the last value of a given micro.
 *
 * @param micro_ID		IN	micro's ID (0, 1, or 2)
 *							(use MICR0, MICR1, MICR2 defined in ad_conv_int.h)
 * @return result		OUT last value of the micro
 **********************************************************************/
int get_micro (unsigned int micro_ID)
{
	return mic_scan[micro_ID][last_mic_scan_ID];
}


/**********************************************************************
 * Get the average on a given number of sample from a micro
 *
 * @param micro_ID		IN	micro's ID (0, 1, or 2)
 *							(use MICR0, MICR1, MICR2 defined in ad_conv_int.h)
 * @param filter_size	IN	number of sample to average
 * @return result		OUT last value of the micro
 **********************************************************************/
int get_micro_average(unsigned int micro_ID, unsigned int filter_size)
{
	long temp = 0;
	int i,j;

	// channel ID must be between 0 to 2 and 
	// filter_size must be between 1 to SAMPLE_NUMBER
	if ((micro_ID < 3) && 
		(filter_size > 0) && (filter_size <= MIC_SAMP_NB))
	{
		for (i=0, j=(MIC_SAMP_NB-(filter_size-(last_mic_scan_ID+1)))%MIC_SAMP_NB ; i<filter_size ; i++, j=(j+1)%MIC_SAMP_NB)
		{
			temp += mic_scan[micro_ID][j];
		}
	}
	return ((int)(temp/filter_size));
}



float calculate_direction(void)
{
	int delta_t1, delta_t2;
	float direction, angle1, angle2;	
	
	// first get the phase-shift between the right and the left microphone
	delta_t1 = find_delta_t(0,1);
	
	// calculate the angle (between -90° and +90° where the sound is coming from)
	if (delta_t1 >= MAXIMUM_DELTA_T1)
			angle1 = PI * 0.5; 			// to avoid NaN of asin
	else if (delta_t1 <= -MAXIMUM_DELTA_T1)
			angle1 = -PI * 0.5; 		// to avoid NaN of asin
	else 
		angle1 = asin( (float)(delta_t1)/MAXIMUM_DELTA_T1 );
	
	
	// now if the signal is coming from the right, we check the phase-shift between the
	// left and the rear microphone in order to find out if the direction is 
	// angle1 or (180° - angle1)
	// if thes signal coming from the left, we make the same test with the right and the 
	// rear microphone
	
	if(angle1 > 0)
	{
		delta_t2 = find_delta_t(2,1);   // phase shift between left and rear microphone

		if (delta_t2 >= MAXIMUM_DELTA_T2)
			angle2 = PI * 0.5; 			// to avoid NAN of asin
		else if (delta_t2 <= -MAXIMUM_DELTA_T2)
			angle2 =-PI * 0.5; 			// to avoid NAN of asin
		else 
			angle2 = asin( (float)delta_t2/MAXIMUM_DELTA_T2 );
		
		if(angle2 > PI/6.)  			// if the second angle is bigger than +30°
			direction = PI - angle1;   	// the direction = 90°-angle1
		else direction = angle1;	
	}
	else
	{
		delta_t2 = find_delta_t(0,2); 	// phase shift between right and rear microphone
		
		if (delta_t2 >= MAXIMUM_DELTA_T2)
			angle2 = PI * 0.5; 			// to avoid NAN of asin
		else if (delta_t2 <= -MAXIMUM_DELTA_T2)
			angle2 = -PI * 0.5; 		// to avoid NAN of asin
		else 
			angle2 = asin( (float)delta_t2/MAXIMUM_DELTA_T2 );
		
		if(angle2 < -PI/6.)				// if the second angle is smaller than -30°
			direction = PI - angle1;	// the direction = 90°-angle1
		else direction = angle1;
	}
	
	// We want an angle strictly between [0,2*PI]
	if (direction < 0)
		direction = 2*PI + direction;
	
	return direction;
}




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		find delta t
//
// Finds the phase-shift between two signal.
// Basically, this function finds the maximum of the
// cross-correlation between two signals. 
//
// in:  int (microphone number of signal 1)
//      int (microphone number of signal 2)
// out: int (time expressed as number of samples taken)
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int find_delta_t(int mic1_nb,int mic2_nb)
{
	int delta_t, tau, k; 
	long int correlation, max;
	extern int mic_scan[3][MIC_SAMP_NB];
	
	int tau_min = -TAU_RANGE / 2;
	int tau_max = TAU_RANGE / 2;

	int save_sound_start = TAU_RANGE / 2 + 1;
	int save_sound_end = MIC_SAMP_NB - TAU_RANGE / 2 - 1;
	
	
	max = 0;
	
	for (tau = tau_min; tau < tau_max; tau++)
	{
		//reset the the correlation value
		correlation = 0;
	
	    // For each tau calculate the correlation between the two signals
	    for (k = save_sound_start; k < save_sound_end; k++)
	    {
		    correlation += (long int)(mic_scan[mic1_nb][k]) * (long int)(mic_scan[mic2_nb][k+tau] );
		}
		
		// find out if this correlation is the biggest one so far. --> If yes,
    	// save the value of tau --> This gives us the phaseshift between the
	    // signals
    
        if (correlation > max)
        {
	        max = correlation;
	        delta_t = tau;
	    }

	}
	
	return delta_t;	
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		show led
//
// Lights up the LED in the appropriate direction
//
// in:  float (angle between 0 and 2PI clockwise)
// out: void
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void show_led(float angle)
{
	int led_nb = 6;
	long int i;
	
	// led_nb corresponds to the appropriate bit number in the LATA register
	if ( angle > (PI/8) )
		led_nb = 7;
	if ( angle > (3*PI/8) )
		led_nb = 9;
	if ( angle > (5*PI/8) )
		led_nb = 12;
	if ( angle > (7*PI/8) )
		led_nb = 10;
	if ( angle > (9*PI/8) )
		led_nb = 13;
	if ( angle > (11*PI/8) )
		led_nb = 14;
	if ( angle > (13*PI/8) )
		led_nb = 15;
	if ( angle > (15*PI/8) )
		led_nb = 6;
	
	// set the bit on PortA to illuminate the led
	LATA = 1 << led_nb;
	
	for (i=0;i<500000;i++);   // Wait to indicate the direction
	
	LATA = 0;	// turn all LEDs off
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		turn to direction
//
// Turns the robot to the appropriate direction
//
// in:  float (angle between 0 and 2PI clockwise)
// out: void
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void turn_to_direction(float direction)
{
	int end_turn;
		
		if (direction < PI)     // turn clockwise
		{
			e_set_steps_left(0);
			e_set_speed_left(TURN_SPEED);  // motor speed in steps/s
			e_set_speed_right(-TURN_SPEED);  // motor speed in steps/s
			
			// calculate how many steps the robot needs to turn
			end_turn = (int)(STEPS_FOR_2PI*(direction/(2*PI)));   
			while(e_get_steps_left() < end_turn);   // turn until done 
		}
		else 					// turn counterclockwise
		{
			e_set_steps_right(0);
			e_set_speed_left(-TURN_SPEED);  // motor speed in steps/s
			e_set_speed_right(TURN_SPEED);  // motor speed in steps/s
			
			// calculate how many steps the robot needs to turn
			end_turn = (int)(STEPS_FOR_2PI*((2*PI-direction)/(2*PI)));
			while(e_get_steps_right() < end_turn);   // turn until done
		}

		// stop motors
		e_set_speed_left(0);  // motor speed in steps/s
		e_set_speed_right(0);  // motor speed in steps/s
}





void run_locatesound() {
	int not_event;
	float direction;
	e_init_motors();
	init_ad_scan();    			// initialize and start ad_conversion
	init_sound();
	while(1) {
		init_sound();
		not_event = 1;
		while (not_event)   	// do this loop until an event has occured
		{
			not_event = check_for_event();
		}
								// if there is an event then:
		record_sound();         // fill up the memory with the claping sound
					
		ADCON1bits.ADON = 0;	// disable AD conversion (avoid interferes with the calculations)
					
		filter_signal();        // filters the signals
					
		direction = calculate_direction();   // do all the calculations where the sound is coming from
							
		ADCON1bits.ADON = 1;	// enable AD conversion
							
		show_led(direction);    // indicate where the sound is coming from
							
		turn_to_direction(direction); // turn the robot to the direction the sound is coming from
	}								

}
