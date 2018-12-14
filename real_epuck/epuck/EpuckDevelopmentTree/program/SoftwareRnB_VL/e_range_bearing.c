/****************************************************************
*	Range and bearing measurement				*
*	between e-pucks with IR proximity sensors		*
*	uses timer1						*
*	June 2007						*
*	Valentin Longchamp		 			*
*								*
****************************************************************/


#include "e_ad_conv.h"
#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_led.h"
#include "e_range_bearing.h"
#include "e_uart_char.h"

#include <stdio.h>
#include <string.h>

#define MARK_LIMIT 10
#define SPACE_LIMIT 8
#define MARK_HPERIOD (250.0*MICROSEC)/8.0
#define SPACE_HPERIOD (312.0*MICROSEC)/8.0

#define READ_PERIOD (500.0*MICROSEC)/8.0

#define LOW_SIGNAL_TRSH 3

typedef enum
{
	NO_SIGNAL = 0,
	SIGNAL = 1
} signal_t;


/* current state */
state_t cur_state;
int emitted_value = 0;

extern int received_signal[NB_IR_SENSORS][SAMPLING_WINDOW];
int min_signal[NB_IR_SENSORS];
int max_signal[NB_IR_SENSORS];
int max_str_sensor;


/* internal calls for prox */
void init_tmr_emit(int value)
{
	T1CON = 0;			// 
	T1CONbits.TCKPS = 1;		// prescsaler = 8
	TMR1 = 0;			// clear timer 1
	if (value)
		PR1 = MARK_HPERIOD;	// emit @ 600 Hz (1.66 ms period) with 8 prescaler
	else
		PR1 = SPACE_HPERIOD;	// emit @ 400 Hz (2.5 ms period) with 8 prescaler
	IFS0bits.T1IF = 0;		// clear interrupt flag
	IEC0bits.T1IE = 1;		// set interrupt enable bit
	T1CONbits.TON = 1;		// start Timer1
}

void inline emit_interrupt()
{
	static int emit_count = 0;
	int thres = emitted_value ? MARK_LIMIT : SPACE_LIMIT;
	
	if (emit_count >= thres)
	{
		if (SELECTOR0)
			emitted_value = 1;
		else
			emitted_value = 0;
		
		e_set_led(4, emitted_value);
		init_tmr_emit(emitted_value);
	}
	else
	{
		emit_count++;
	}
	
	PULSE_IR3 = PULSE_IR2 = PULSE_IR1 = PULSE_IR0 = PULSE_IR0^1;
}

void init_tmr_read()
{
	T1CON = 0;			// 
	T1CONbits.TCKPS = 1;		// prescsaler = 8
	TMR1 = 0;			// clear timer 1
	PR1 = READ_PERIOD;		// read @ 1kHz
	IFS0bits.T1IF = 0;		// clear interrupt flag
	IEC0bits.T1IE = 1;		// set interrupt enable bit
	T1CONbits.TON = 1;		// start Timer1
}

void print_signal(int sensor)
{
	int i = 0;
	char buffer[COM_BUFFER_LEN];
	if (sensor >= NB_IR_SENSORS)
		return;
	
	sprintf(buffer,"sensor %d: ", sensor);
	uart_send_text(buffer);
	
	for(i=0; i<SAMPLING_WINDOW;i++)
	{
		sprintf(buffer,"%d", received_signal[sensor][i]);
		uart_send_text(buffer);
		if (i != SAMPLING_WINDOW -1)
			uart_send_static_text(",");
	}
	uart_send_static_text("\r\n");
}

signal_t preprocess_signal()
{
	int i = 0, max_diff = 0, threshold = 0;
	e_set_led(max_str_sensor,0);

	//find the receiver with biggest strength
	for (i=0; i<NB_IR_SENSORS; i++)
	{
		int j = 0, min = 4096, max = 0;
		for (j=0; j<SAMPLING_WINDOW; j++)
		{
			if (received_signal[i][j] < min)
			{
				min = received_signal[i][j];
			}
			else if (received_signal[i][j] > max)
			{
				max = received_signal[i][j];
			}
		}
		
		min_signal[i] = min;
		max_signal[i] = max;
		if (max_signal[i] - min_signal[i] > max_diff)
		{
			max_diff = max_signal[i] - min_signal[i];
			max_str_sensor = i;
		}
	}
	
	//we work on the best sensor
	
	//now we have the threshold and can normalize the signal
	threshold = max_diff/2;
	
	if (threshold < LOW_SIGNAL_TRSH)
	{
		uart_send_static_text("no signal\r\n");
		return NO_SIGNAL;
	}
	uart_send_static_text("demodulation should start\r\n");
	
	e_set_led(max_str_sensor,1);
	
	for (i=0; i<SAMPLING_WINDOW; i++)
	{
		if(received_signal[max_str_sensor][i] > threshold)
			received_signal[max_str_sensor][i] = 1;
		else
			received_signal[max_str_sensor][i] = 0;
	}
	
	//TODO: do the actual demodulation by comparing with the mark and space frequencies
	// use complex computations, not the optimized way yet
	return SIGNAL;
}

void _ISRFAST _T1Interrupt(void)
{
	
	IFS0bits.T1IF = 0;	// clear interrupt flag
	
	if (cur_state == READING)
	{
		if (e_ad_is_array_filled())
			preprocess_signal();
	}
	else if (cur_state == EMIT)
	{
		emit_interrupt();
	}
	else 
	{
		uart_send_static_text("unknown state!\r\n");
	}
}

/* ---- user calls ---- */

void e_init_range_bearing(state_t state)
{
	cur_state = state;
	if (state == EMIT)
		init_tmr_emit(emitted_value);
	else if (state == READING)
	{
		e_init_ad_scan();
		e_ad_scan_on();
		init_tmr_read();
		max_str_sensor = 0;
	}
}
	
void e_stop_range_bearing()
{
	 T1CONbits.TON = 0;
	 PULSE_IR0=PULSE_IR1=PULSE_IR2=PULSE_IR3=0;
}
