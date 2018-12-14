/********************************************************************************

			Program to demonstrate the e-puck's open loop motor control capabilities
			2 February 2010
			Alexander Bahr

(c) 2010 Alexander Bahr

Distributed intelligent systems and algorithms laboratory http://disal.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/


/*! \file
 * \brief The main file of the open loop motor control demo.
 *
 * This file demonstrates the functionality of the two open loop motor control 
 * functions e_set_distance(distance,motor_speed) and e_set_turn(angle,motor_speed).
 * \n \n The program goes through the following sequence of open loop controlled motions:
 * - Turn CCW for 90 degrees
 * - Turn CW for 90 degrees
 * - Go forward for 200 mm
 * - Go backward for -100 mm
 * - Turn CW for 180 degrees
 * - Go forward for 100 mm
 * - Turn CCW for 90 degrees
 * After this sequence the robot should be in the same position and orientation from which it started
 * \warning This demo requires a new version of e_motors.c and e_motors.h
 * \author Code: AlexanderBahr \n Doc: Alexander Bahr
 */


#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"

#include "../uart/e_uart_char.h"
#include "e_init_port.h"
#include "e_epuck_ports.h"
#include "e_motors.h"
#include "main.h"

#define PI 3.14159265358979

int main()
{
	//system initialization 
	e_init_port();
	e_init_uart1();

	//Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR=0;
		__asm__ volatile ("reset");
	}

	dr_demo_4();

	while(1)
	{
	};

	return 0;
}

int open_loop_demo_1()
{
	char buffer[80];

	long int i;

	// clear screen
	sprintf(buffer, "\x1B[2J");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// set cursor to upper left corner	
	sprintf(buffer, "\x1B[1;1H");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Demonstrating open loop control (turn, back forth):\n\r\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// 90 degrees CCW turn at speed 200

	sprintf(buffer, "1.) Turning 90 degrees CCW at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_turn(90,200);

	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// 90 degrees CW turn at speed 200

	sprintf(buffer, "2.) Turning 90 degrees CW at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_turn(-90,200);

	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// Going forward 200mm at speed 200

	sprintf(buffer, "3.) Going forward for 200 mm at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_distance(200,200);
	
	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// Going forward -100mm at speed 200

	sprintf(buffer, "4.) Going backward for -100 mm at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_distance(-100,200);

	while(e_get_ol_ctrl_status() != 4);
	
	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// 180 degrees CW turn at speed 200

	sprintf(buffer, "5.) Turning 180 degrees CW at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_turn(-180,200);

	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// Going forward 100mm at speed 200

	sprintf(buffer, "6.) Going forward for 100 mm at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_distance(100,200);
	
	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// 180 degrees CCW turn at speed 200

	sprintf(buffer, "7.) Turning 180 degrees CCW at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_turn(180,200);

	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Sequence complete!\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	return 0;
}

int open_loop_demo_2()
{
	char buffer[80];

	long int i;

	// clear screen
	sprintf(buffer, "\x1B[2J");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// set cursor to upper left corner	
	sprintf(buffer, "\x1B[1;1H");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Demonstrating open loop control (square):\n\r\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// FIRST LEG

	// Going forward 750mm at speed 500

	sprintf(buffer, "1.) Going forward for 750mm at speed 500 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_distance(750,500);
	
	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// 90 degrees CCW turn at speed 200

	sprintf(buffer, "2.) Turning 90 degrees CCW at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_turn(90,200);

	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// SECOND LEG

	// Going forward 200mm at speed 200

	sprintf(buffer, "3.) Going forward for 750mm at speed 500 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_distance(750,500);
	
	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// 90 degrees CW turn at speed 200

	sprintf(buffer, "4.) Turning 90 degrees CCW at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_turn(90,200);

	while(e_get_ol_ctrl_status() != 4);

	// THIRD LEG

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms

	// Going forward 200mm at speed 200

	sprintf(buffer, "5.) Going forward for 750mm at speed 500 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_distance(750,500);
	
	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// 90 degrees CW turn at speed 200

	sprintf(buffer, "6.) Turning 90 degrees CCW at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_turn(90,200);

	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// FOURTH LEG

	// Going forward 750mm at speed 500

	sprintf(buffer, "7.) Going forward for 750mm at speed 500 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_distance(750,500);
	
	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// 90 degrees CW turn at speed 200

	sprintf(buffer, "8.) Turning 90 degrees CCW at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	e_set_turn(90,200);

	while(e_get_ol_ctrl_status() != 4);

	sprintf(buffer, "Done\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	return 0;
}

int dr_demo_1()
{
	char buffer[80];

	long int i;
	
	double dr_x = 0;
	double dr_y = 0;
	double dr_theta = 0;

	// init position (x=0,y=0,theta=0deg=>north=>up)
	e_set_dr_position(dr_x,dr_y,dr_theta);
	
	// clear screen
	sprintf(buffer, "\x1B[2J");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// set cursor to upper left corner	
	sprintf(buffer, "\x1B[1;1H");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Demonstrating dead-reckoning (going forward):\n\r\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Going forward at speed 200 ...\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// go forward
	e_set_speed_left(300);
	e_set_speed_right(300);

	// do dead reckoning and print position
	while(1)
	{
		e_do_dr();
		e_get_dr_position(&dr_x,&dr_y,&dr_theta);

		sprintf(buffer, "x=%fmm;y=%fmm;theta=%4.1fdeg\n\r",dr_x,dr_y,dr_theta);
		e_send_uart1_char(buffer, strlen(buffer));
		for(i=0;i<50000;i++);//wait 10ms;
	}

	return 0;
}

int dr_demo_2()
{
	char buffer[80];

	long int i;
	
	double dr_x = 0;
	double dr_y = 0;
	double dr_theta = 0;

	// init
	e_set_dr_position(dr_x,dr_y,dr_theta);
	
	// clear screen
	sprintf(buffer, "\x1B[2J");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// set cursor to upper left corner	
	sprintf(buffer, "\x1B[1;1H");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Demonstrating dead-reckoning (going in circle):\n\r\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Going forward at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// go forward
	e_set_speed_left(200);
	e_set_speed_right(300);

	// do dead reckoning and print position
	while(1)
	{
		e_do_dr();
		e_get_dr_position(&dr_x,&dr_y,&dr_theta);

		sprintf(buffer, "x=%fmm;y=%fmm;theta=%fdeg\n\r",dr_x,dr_y,dr_theta);
		e_send_uart1_char(buffer, strlen(buffer));
		for(i=0;i<20000;i++);//wait 10ms;
	}

	return 0;
}

int dr_demo_3()
{
	char buffer[80];

	long int i;
	
	double dr_x = 0;
	double dr_y = 0;
	double dr_theta = 0;

	// init
	e_set_dr_position(dr_x,dr_y,dr_theta);
	
	// clear screen
	sprintf(buffer, "\x1B[2J");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// set cursor to upper left corner	
	sprintf(buffer, "\x1B[1;1H");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Demonstrating dead-reckoning (going forward and stopping at 200mm):\n\r\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Going forward at speed 200 ... ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// go forward
	e_set_speed_left(300);
	e_set_speed_right(300);

	// do dead reckoning and print position
	while(dr_y<=200)
	{
		e_do_dr();
		e_get_dr_position(&dr_x,&dr_y,&dr_theta);

		sprintf(buffer, "x=%fmm;y=%fmm;theta=%fdeg\n\r",dr_x,dr_y,dr_theta);
		e_send_uart1_char(buffer, strlen(buffer));
		for(i=0;i<20000;i++);//wait 10ms;
	}

	// stop
	e_set_speed_left(0);
	e_set_speed_right(0);

	return 0;
}

int dr_demo_4()
{
	char buffer[80];

	long int i;
	
	double dr_x = 0;
	double dr_y = 0;
	double dr_theta = 0;

	// init dead reckoning
	e_set_dr_position(dr_x,dr_y,dr_theta);
	
	// clear screen
	sprintf(buffer, "\x1B[2J");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// set cursor to upper left corner	
	sprintf(buffer, "\x1B[1;1H");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Demonstrating dead-reckoning (going in 200mm forward, ");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;
	sprintf(buffer, "turning 180 degrees, returning and turning 180 degrees):\n\r\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	sprintf(buffer, "Going 200mm forward\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// FIRST leg - go forward
	e_set_speed_left(300);
	e_set_speed_right(300);

	while(dr_y<=200)
	{
		e_do_dr();
		e_get_dr_position(&dr_x,&dr_y,&dr_theta);

		for(i=0;i<20000;i++);//wait 10ms;
	}

	// STOP
	e_set_speed_left(0);
	e_set_speed_right(0);

	sprintf(buffer, "Turning 180 degrees clock-wise\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// FIRST turn
	e_set_speed_left(100);
	e_set_speed_right(-100);

	while((dr_theta<180) || (dr_theta>190))
	{
		e_do_dr();
		e_get_dr_position(&dr_x,&dr_y,&dr_theta);

		for(i=0;i<20000;i++);//wait 10ms;
	}

	sprintf(buffer, "Going 200mm forward\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// SECOND leg (go back)
	e_set_speed_left(300);
	e_set_speed_right(300);

	while(dr_y>0)
	{
		e_do_dr();
		e_get_dr_position(&dr_x,&dr_y,&dr_theta);

		for(i=0;i<20000;i++);//wait 10ms;
	}

	// STOP
	e_set_speed_left(0);
	e_set_speed_right(0);

	sprintf(buffer, "Turning 180 degrees clock-wise\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// SECOND turn
	e_set_speed_left(100);
	e_set_speed_right(-100);
	
	while((dr_theta<1) || (dr_theta>5))
	{
		e_do_dr();
		e_get_dr_position(&dr_x,&dr_y,&dr_theta);

		for(i=0;i<20000;i++);//wait 10ms;
	}

	sprintf(buffer, "Done!\n\r");
	e_send_uart1_char(buffer, strlen(buffer));
	for(i=0;i<20000;i++);//wait 10ms;

	// STOP
	e_set_speed_left(0);
	e_set_speed_right(0);

	return 0;
}
