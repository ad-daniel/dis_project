/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *  SoftwareRnB_VL/main.c
 *
 *  $Author$
 *  $Date: 2007-11-06 15:10:14 +0100 (Tue, 06 Nov 2007) $
 *  $HeadURL: https://grmapc10.epfl.ch/svn/students/Epuck/EpuckDevelopmentTree/program/SoftwareRnB_VL/main.c $
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <stdio.h>
#include <string.h>

#include "p30f6014a.h"                  // e-puck std library modules
#include "motor_led/e_led.h"            //
#include "motor_led/e_init_port.h"      //
#include "motor_led/e_epuck_ports.h"    //
#include "contrib/robot_id/e_robot_id.h"//

#include "e_uart_char.h"                // local version!
#include "e_ad_conv.h"                  // local version!

#include "e_range_bearing.h"


extern int received_signal[NB_IR_SENSORS][SAMPLING_WINDOW];


void e_auto_reset () 
{
    unsigned char ch;
    /*
    while (e_getchar_uart1(&ch))        // TODO: CHECK UART NUMBER!!!
        if (ch==0xC1) RESET();
    */
}


int main ()
{
	char c;

	e_init_port();
	e_init_uart1();

    // Reset if Power on (some problem for few robots)
    if (RCONbits.POR) {
        RCONbits.POR=0;
        __asm__ volatile ("reset");
    }

    int selector = e_get_selector();

    // Selector set to position 0 (3 o'clock) is the emitter.
	if ( selector == 0 ) {
		e_init_range_bearing(EMIT);
		e_set_led(0,2);
	}

    // Any other selector position will invoke receiving behavior. 
	else e_init_range_bearing(READING);


    // begin "forever" loop
    while (1) 
    {
	    if ( selector == 0 )
        { 
            Nop(); 
        }
        else
        {
			while (e_getchar_uart1(&c)==0);
			if (c == 'v')
			{
				int i=0;
				char buffer[COM_BUFFER_LEN];
				e_set_led(2,2);
				for(i=0; i<SAMPLING_WINDOW;i++)
				{
					sprintf(buffer,"%d", received_signal[0][i]);
					uart_send_text(buffer);
					if (i != SAMPLING_WINDOW -1)
						uart_send_static_text(",");
				}
				uart_send_static_text("\r\n");
			}
        }

        e_auto_reset();
    }

    return 0; // never reached
}
