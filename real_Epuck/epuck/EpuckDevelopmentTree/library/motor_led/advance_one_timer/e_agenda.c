
#include "e_agenda.h"
#include "../e_epuck_ports.h"
#include <stdlib.h>

#define EXIT_OK 1

static Agenda *agenda_list = 0; //pointer on the agenda chained list


/**
 * Start the agendas processing by starting the Timer2
 *
 * @param void       IN not used
 * @return void      OUT not used
 */
void e_start_agendas_processing(void)
{
	T2CON = 0;							// reset Timer2 CONtrol register
	T2CONbits.TCKPS = 0;				// precsaler = 1
	TMR2 = 0;							// clear timer 2
	PR2 = (int)(0.1*MILLISEC);		// interrupt every 0.1 ms with 64 prescaler
	IFS0bits.T2IF = 0;					// clear interrupt flag
	IEC0bits.T2IE = 1;					// set interrupt enable bit
	T2CONbits.TON = 1;					// start Timer2
}


/**
 * Stop all the agendas by disabling Timer2 
 * (the memory allocated for the agenda isn't freed, use destroy_agenda for that)
 *
 * @param void
 * @return void      OUT not used
 */
void e_end_agendas_processing(void)
{
	T2CONbits.TON = 0;    // disable Timer2
}


/**
 * Activate an agenda and allocate memory for him if there isn't already
 * an agenda with the same callback function
 * (the agenda is active but isn't processed if he 
 * have a null cycle value)
 *
 * @param cycle      IN  cycle value in millisec/10
 * @param function	 IN  function called if the cycle value is reached by the counter
 * @return void      OUT not used
 */
int e_activate_agenda(void (*func)(void), int cycle)
{
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
			return(AG_ALREADY_CREATED);
		else
			current = current->next;
	}
	if(!(current = malloc(sizeof(Agenda))))
		exit (EXIT_FAILURE);

	current->cycle = cycle;
	current->counter = 0;
	current->activate = 1;
	current->function = func;
	current->next = agenda_list;

	agenda_list = current;
	return(EXIT_OK);
}


/**
 * Destroy the agenda with a given callback function
 *
 * @param func		 IN  function to test
 * @return void      OUT not used
 */
int e_destroy_agenda(void (*func)(void))
{
	Agenda *preceding = 0;
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
		{
			if (preceding)
				preceding->next = current->next;
			else
				agenda_list		= current->next;

			free(current);
			return(EXIT_OK);
		}			
		else
		{
			preceding = current;
			current = current->next;
		}
	}
	return(AG_NOT_FOUND);
}


/**
 * Change the cycle value of an agenda with a given callback function
 *
 * @param cycle      IN  new cycle value in millisec/10
 * @param func		 IN  function to test
 * @return void      OUT not used
 */
int e_set_agenda_cycle(void (*func)(void), int cycle)
{
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
		{
			current->cycle = cycle;
			return(EXIT_OK);
		}
		else
			current = current->next;
	}
	return(AG_NOT_FOUND);
}


/**
 *Reset an agenda's counter with a given callback function
 *
 * @param func		 IN  function to reset
 * @return void      OUT not used
 */
int e_reset_agenda(void (*func)(void))
{
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
		{
			current->counter = 0;
			return(EXIT_OK);
		}
		else
			current = current->next;
	}
	return(AG_NOT_FOUND);
}

/**
 * Pause an agenda but do not reset its information
 *
 * @param func		 IN  function to pause
 * @return void      OUT not used
 */
int e_pause_agenda(void (*func)(void))
{
	Agenda *current = agenda_list;
	
	while (current)
	{
		if (current->function == func)
		{
			current->activate = 0;
			return(EXIT_OK);
		}
		else
			current = current->next;
	}
	return(AG_NOT_FOUND);
	
}

/**
 * Restart an agenda previously paused
 *
 * @param func		 IN  function to restart
 * @return void      OUT not used
 */
int e_restart_agenda(void (*func)(void))
{
	Agenda *current = agenda_list;
	
	while (current)
	{
		if (current->function == func)
		{
			current->activate = 1;
			return(EXIT_OK);
		}
		else
			current = current->next;
	}
	return(AG_NOT_FOUND);
	
}


/**
 * Parse the chained list of agenda.
 * Increment counter only.
 * Check if agenda has to be treated according to the cycle value 
 * and current counter value.
 * Do it for number of cycle positive or null.
 * Check if a service has to be activated. 
 *
 * @return void 
 */
void __attribute__((__interrupt__))
 _T2Interrupt(void)
{
	Agenda *current = agenda_list;

	IFS0bits.T2IF = 0;

	while (current)
	{
		// agenda must be active with a positive non-null cycle value
		if(current->activate == 1 && current->cycle > 0) 
		{
			current->counter++;
			// check if the agenda event must be triggered
			if(current->counter > current->cycle-1) // a cycle value of 1 will be performed every interupt
			{
				current->function();	// trigger the associeted function
				current->counter=0;		// reset the counter
			}
		}
		current = current->next;
	}
  return;
}


/* End of File : alarm.c */
