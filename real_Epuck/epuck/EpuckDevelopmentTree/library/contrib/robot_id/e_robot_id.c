/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  e_robot_id.c
 *
 *  Interface to robot's unique id number stored in the EEPROM.
 *
 *  Adapted from initial version by X. Raemy.
 *
 *  $Author$
 *  $Date: 2007-09-20 14:21:06 +0200 (Thu, 20 Sep 2007) $
 *  $Revision$
 *  $HeadURL: https://grmapc10.epfl.ch/svn/students/Epuck/EpuckDevelopmentTree/library/contrib/robot_id/e_robot_id.c $
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include "e_robot_id.h"
#include "motor_led/e_epuck_ports.h"

static int robotid;
int readrobotidfromeeprom();

void e_init_robot_id() {
	robotid = readrobotidfromeeprom();
}

int e_get_robot_id() {
	return robotid;
}

int e_get_selector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

