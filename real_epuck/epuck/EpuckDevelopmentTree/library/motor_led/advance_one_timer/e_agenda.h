#ifndef __AGENDA_H__
#define __AGENDA_H__


#define AG_ALREADY_CREATED	1
#define AG_NOT_FOUND		2

/**********************************************************************
 * ------------------------ Type definition ---------------------------
 **********************************************************************/

typedef struct AgendaType Agenda;

struct AgendaType
{
  unsigned int  cycle;		// length in 10e of ms of a cycle between two alarms
  int  counter;				// counter 
  char activate;			// can be on=1 or off=0
  void (*function) (void);	// function called when counter > cycle, 
							// WARNING: This function must have the following prototype
							//			void func(void)
  Agenda *next;				// pointer on the next agenda
};


/***********************************************************************
 * ------------------------ From agenda.c file --------------------------
 **********************************************************************/
void e_start_agendas_processing(void);
void e_end_agendas_processing(void);

int e_activate_agenda(void (*func)(void), int cycle);
int e_destroy_agenda(void (*func)(void));

int e_set_agenda_cycle(void (*func)(void), int cycle);
int e_reset_agenda(void (*func)(void));

int e_pause_agenda(void (*func)(void));
int e_restart_agenda(void (*func)(void));

#endif /* __AGENDA_H__ */


/* End of File : agenda.h */
