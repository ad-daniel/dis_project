/********************************************************************************

			I2C master module
			Version 1.0 may 2005 Davis Daidie


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005-2007 Davis Daidie

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup i2c
 * \brief Manage I2C basics.
 *
 * \author Code: Davis Daidie \n Doc: Jonathan Besuchet
 */

/*! \defgroup i2c I2C
 * 
 * \section intro_sec Introduction
 * This software allows using the I2C hardware module on a DsPic30f60xx
 * in a master mode for a single master system.
 * \n This module manage the I2C basics functions (low level 
 * I2C functions). They are made to perform the basics tasks like:
 * - initializing the I2C on the microcontroller (\ref e_i2c_init(void))
 * - sending the Start bit (\ref e_i2c_start(void))
 * - sending the Restart bit (\ref e_i2c_restart(void))
 * - sending the Stop bit (\ref e_i2c_stop(void))
 * - sending the acknowledgement bit (\ref e_i2c_ack(void))
 * - writing or receiving a byte (\ref e_i2c_write(char byte),
 *  \ref e_i2c_read(char *buf))
 * - ...
 *
 * \section prot_sect Overview of I2C protocol
 * The I2C-bus supports any IC fabrication process (NMOS,
 * CMOS, bipolar). Two wires, serial data (SDA) and serial
 * clock (SCL), carry information between the devices
 * connected to the bus. Each device is recognized by a
 * unique address (whether it's a microcontroller, LCD driver,
 * memory or keyboard interface) and can operate as either
 * a transmitter or receiver, depending on the function of the
 * device.
 * \subsection prot_subsect1 Master-slave relation
 * The I2C-bus is a multi-master bus. This means that more
 * than one device capable of controlling the bus can be
 * connected to it. As masters are usually microcontrollers,
 * let's consider the case of a data transfer between two
 * microcontrollers connected to the I2C-bus.
 * \n This highlights the master-slave and receiver-transmitter
 * relationships to be found on the I2C-bus. It should be noted
 * that these relationships are not permanent, but only
 * depend on the direction of data transfer at that time. The
 * transfer of data would proceed as follows:
 * \n \n 1) Suppose microcontroller A wants to send information to
 * module B:
 * - microcontroller A (master), addresses module B (slave)
 * - microcontroller A (master-transmitter), sends data to
 * module B (slave-receiver)
 * - microcontroller A terminates the transfer
 *
 * 2) If microcontroller A wants to receive information from
 * module B:
 * - microcontroller A (master) addresses module B
 * (slave)
 * - microcontroller A (master-receiver) receives data from
 * module B (slave-transmitter)
 * - microcontroller A terminates the transfer.
 *
 * Even in this case, the master (microcontroller A) generates
 * the timing and terminates the transfer.
 *
 * \subsection prot_subsect2 Start and Stop conditions
 * Within the procedure of the I2C-bus, unique situations
 * arise which are defined as START (S) and STOP (P)
 * conditions.
 * \n A HIGH to LOW transition on the SDA line while SCL is
 * HIGH is one such unique case. This situation indicates a
 * START condition.
 * \n A LOW to HIGH transition on the SDA line while SCL is
 * HIGH defines a STOP condition.
 * \n START and STOP conditions are always generated by the
 * master. The bus is considered to be busy after the START
 * condition. The bus is considered to be free again a certain
 * time after the STOP condition.
 * \n The bus stays busy if a repeated START (Sr) is generated
 * instead of a STOP condition. In this respect, the START
 * (S) and repeated START (Sr) conditions are functionally
 * identical).
 * Detection of START and STOP conditions by devices
 * connected to the bus is easy if they incorporate the
 * necessary interfacing hardware.
 *
 * \subsection prot_subsect3 Transfering data
 * Every byte put on the SDA line must be 8-bits long (char type).
 * \n \n Acknowledge:
 * \n Data transfer with acknowledge is obligatory. The
 * acknowledge-related clock pulse is generated by the
 * master. The transmitter releases the SDA line (HIGH)
 * during the acknowledge clock pulse.
 * \n The receiver must pull down the SDA line during the
 * acknowledge clock pulse so that it remains stable LOW
 * during the HIGH period of this clock pulse. Of
 * course, set-up and hold times must also be taken into account.
 * \n Usually, a receiver which has been addressed is obliged to
 * generate an acknowledge after each byte has been received.
 * \n When a slave doesn't acknowledge the slave address (for
 * example, it's unable to receive or transmit because it's
 * performing some real-time function), the data line must be
 * left HIGH by the slave. The master can then generate
 * either a STOP condition to abort the transfer, or a repeated
 * START condition to start a new transfer.
 * If a slave-receiver does acknowledge the slave address
 * but, some time later in the transfer cannot receive any
 * more data bytes, the master must again abort the transfer.
 * This is indicated by the slave generating the
 * not-acknowledge on the first byte to follow. The slave
 * leaves the data line HIGH and the master generates a
 * STOP or a repeated START condition.
 * \n If a master-receiver is involved in a transfer, it must signal
 * the end of data to the slave-transmitter by not generating
 * an acknowledge on the last byte that was clocked out of
 * the slave. The slave-transmitter must release the data line
 * to allow the master to generate a STOP or repeated
 * START condition.
 * 
 * \section use_on_epuck_sect Use I2C on e-puck
 * On the e-puck, the microcontroller is always the master.
 * \subsection use_on_epuck_subsect1 Basics functions
 * The functions of the files e_I2C_master_module.c and
 * e_I2C_master_module.h are low level I2C functions.
 * \n They are made to perform the basics tasks like:
 * - initializing the I2C on the microcontroller (\ref e_i2c_init(void))
 * - sending the Start bit (\ref e_i2c_start(void))
 * - sending the Restart bit (\ref e_i2c_restart(void))
 * - sending the Stop bit (\ref e_i2c_stop(void))
 * - sending the acknowledgement bit (\ref e_i2c_ack(void))
 * - writing or receiving a byte (\ref e_i2c_write(char byte),
 *  \ref e_i2c_read(char *buf))
 * - ...
 * \subsection use_on_epuck_subsect2 More developed functions
 * The functions of the files e_I2C_protocol.c and
 * e_I2C_protocol.h are made to directly send or 
 * receive data from or to a specified slave.
 *
 * \section reference Reference
 * For more information about I2C:
 * - http://en.wikipedia.org/wiki/I%C2%B2C
 * - http://www.nxp.com/acrobat_download/literature/9398/39340011.pdf
 * - http://tmd.havit.cz/Papers/I2C.pdf
 *
 * \author Doc: Jonathan Besuchet
 */

#ifndef _I2C_MASTER_MODULE
#define _I2C_MASTER_MODULE

#include "p30f6014a.h"


#define START			1
#define WRITE			2
#define ACKNOWLEDGE		3
#define READ			4
#define STOP			5
#define RESTART			6
#define ERROR			10
#define OPERATION_OK	0

// -use I2C_init() in your initialisation
// -I2C_enable(void) before anythig else to enable interrupts
// the others functions must be use generate a valide i2c message.

// all the functions return 1 to confirme the oparation and 0 for an error
// in this case, use char I2C_reset(void) and redo the wrong oparation.
// if there is no result, switch of evrything and beginn from the beginning.. :-) 

char e_i2c_init(void);
char e_i2c_start(void);
char e_i2c_restart(void);
char e_i2c_ack(void);
char e_i2c_nack(void);
char e_i2c_read(char *buf);
char e_i2c_stop(void);
char e_i2c_write(char byte);
char e_i2c_enable(void);
char e_i2c_disable(void);
char e_i2c_reset(void);


#endif
