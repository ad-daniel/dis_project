/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  e_robot_id.h
 *
 *  Interface to robot's unique id number stored in the EEPROM.
 *
 *  Adapted from initial version by X. Raemy.
 *
 *  $Author$
 *  $Date: 2007-09-20 14:21:06 +0200 (Thu, 20 Sep 2007) $
 *  $Revision$
 *  $HeadURL: https://grmapc10.epfl.ch/svn/students/Epuck/EpuckDevelopmentTree/library/contrib/robot_id/e_robot_id.h $
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#ifndef _ROBOT_ID_H
#define _ROBOT_ID_H


void e_init_robot_id();
int  e_get_robot_id();
int  e_get_selector();


#endif // _ROBOT_ID_H
