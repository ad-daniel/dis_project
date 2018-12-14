// e_I2C_slave.c

/*
	Very simple (and partial) I2C slave implementation and is not safe for everyday's application
*/

#include <p30f6014a.h>
#include <string.h>

#include "e_I2C_slave.h"

#define I2C_SLAVE_BUFFER_SIZE		16

enum {
	SLAVE_IDLE = 0,
	SLAVE_AM_WRITE = 1,
	SLAVE_WRITE = 2,
	SLAVE_AM_READ = 4,
	SLAVE_READ = 5,
} i2c_slave_states;

static unsigned char 	I2C_Buffer[I2C_SLAVE_BUFFER_SIZE];		// Maximum I2C_SLAVE_BUFFER_SIZE bytes can be transmit at once
static unsigned char	I2C_Counter = 0;
static unsigned char	I2C_NewData = 0;

static void (*m_fctCallback)(void) = NULL;
static unsigned char	m_module_address = 0;

/**************************************************************************************************/
/**************************************************************************************************/
/*-------------------------------------- EXPORTED FUNCTIONS --------------------------------------*/
/**************************************************************************************************/
/**************************************************************************************************/

void e_I2C_slave_init(char module_address, void (*fctCallback)(void))
{
	m_fctCallback = fctCallback;
	
	I2CCONbits.I2CEN=0;		// Disable I2C
	
	I2CADD = module_address;	// Set the module address defined in the program
	m_module_address = module_address;
	
	I2CCONbits.SCLREL = 1;		// Release the clock
	I2CCONbits.IPMIEN = 0;		// Only acknowledge own address
	I2CCONbits.A10M = 0;		// 7bit slave address
	I2CCONbits.DISSLW = 1;		// Slew rate control disabled (enable for 400kHz operation!)
//	I2CCONbits.SMEN = 0;		// Disable SMBus Input thresholds (set for 3.3V operation!)
//	I2CCONbits.GCEN = 0;		// Disable interrupt on a general address call
	I2CCONbits.STREN = 1;		// Enable software or receive clock stretching (important when dealing with interrupts)
	I2CCONbits.ACKDT = 0;		// Send ACK during acknowledge
//	I2CCONbits.ACKEN = 0;		// Acknowledge sequence not in progress
//	I2CCONbits.RCEN = 0;		// Receive sequence not in progress
//	I2CCONbits.PEN = 0;		// STOP condition not in progress
//	I2CCONbits.RSEN = 0;		// Repeated START condition not in progress
//	I2CCONbits.SEN = 0;		// START condition not in progress
	
	_SI2CIF = 0;			// Clear the slave interrupt
	I2CSTATbits.I2COV = 0;      	// Clear Overflow flag
	_SI2CIE = 1;			// Enable the slave interrupt

	I2CCONbits.I2CEN = 1;		// Enables the I2C module and configures the SDA and SCL pins as serial port pins

	return;

}

void e_I2C_slave_inhibit(void)
{
	I2CADD = 0;
}

void e_I2C_slave_uninhibit(void)
{
	I2CADD = m_module_address ;
}		

void e_I2C_slave_set_address(unsigned char module_address)
{
	I2CADD = module_address;
	m_module_address = module_address;
	return;
}

unsigned char e_I2C_slave_get_address(void)
{
	return I2CADD;
}

void e_I2C_slave_refresh(void)
{
	if (e_I2C_slave_new_data()){
		// Wait for the bus to be free
		while (e_I2C_slave_is_bus_busy());
		// Call the function callback
		if (m_fctCallback != NULL) {
			m_fctCallback();
		}
	}
}

char e_I2C_slave_is_bus_busy(void)
{
	return I2CSTATbits.S;
}

char e_I2C_slave_new_data(void)
{
	return I2C_NewData;
}

unsigned char e_I2C_slave_get_data_length(void){
	return I2C_Counter;
}

char e_I2C_slave_get_data(unsigned char* dst_buffer, unsigned char len)
{
	unsigned char length;
	
	// Disable user interrupts
	__asm__ volatile("disi #0x3FFF");
	
	length = ( len < I2C_Counter ? len : I2C_Counter );
	if (length != 0) {
		memcpy(dst_buffer, I2C_Buffer, length);
		I2C_NewData = 0;
	}
	
	// Reenable user interrupts
	__asm__ volatile("disi #0x0000");
	
	return length;
}

/**************************************************************************************************/
/**************************************************************************************************/
/*---------------------------------- INTERRUPT SERVICE ROUTINE -----------------------------------*/
/**************************************************************************************************/
/**************************************************************************************************/

/**
\brief	Interrupt service routine for the slave interface

There are different causes of interrupts, which will call this function. The meaning of the interrupt is a function of some I2C flags and the
state of the state machine.
<ul>
	<li>RBF=1, D_A=0, R_W=0	An address match occured, Master writes to slave</li>
	<li>RBF=1, D_A=0, R_W=1	An address match occured, Master reads from slave</li>
 	<li>RBF=1, D_A=1, R_W=0	Data byte received, receive Buffer full</li>	
	<li>RBF=0, D_A=x, R_W=x	Multiple reading, Master is expecting data</li>	
</ul>

The state machine has the following states:
<ul>
	<li>I2C_State = SLAVE_IDLE	Idle</li>
	<li>I2C_State = SLAVE_AM_WRITE		Address match, Write to Slave</li>
	<li>I2C_State = SLAVE_WRITE			Bytes received</li>
	<li>I2C_State = SLAVE_AM_READ		Address match, Read first byte from Slave</li>
	<li>I2C_State = SLAVE_READ			Multiple reading</li>
</ul>

Another possible source of an interrupt is an overflow of the input buffer. But as we work with clock stretching (the master cannot send data
until the buffer is free) this interrupt cannot happen.

*/

void  __attribute__((__interrupt__, auto_psv)) _SI2CInterrupt(void)										// Interrupt service routine
{
	static unsigned char	I2C_State = SLAVE_IDLE;
	unsigned char 	I2C_Data = 0;

	if (I2CSTATbits.RBF)											// Receive Buffer full
	{																// -------------------
		I2C_Data = I2CRCV;											// save the received data
		if (I2CSTATbits.D_A)										// if D_A flag set we have received a data byte
		{															// I2C_State is FIRST_B_RECEIVED or ANOTHER_B_RECEIVED
			I2CCONbits.SCLREL = 1;									// release the clock line
		}		
		else														// else its an address
		{															// 
			I2C_State = SLAVE_AM_WRITE;								// R_W flag clear -> WRITE to slave -> state=SLAVE_AM_WRITE
			if (I2CSTATbits.R_W) I2C_State = SLAVE_AM_READ;			// R_W flag set -> READ from slave -> state=SLAVE_AM_READ
		}															// If the receive buffer is empty the interrupt cause is a multiple reading request
	}																// from the master. In this case we don't do anything as the state is already SLAVE_AM_READ.

    switch (I2C_State)
	{
		case SLAVE_AM_WRITE:										// MODULE ADDRESS RECEIVED
		{															// -----------------------
			I2C_Counter = 0;
			I2C_State = SLAVE_WRITE;								// next byte is write to slave
			break;
		}

		case SLAVE_WRITE:											// BYTE RECEIVED
		{															// -------------
			I2C_Buffer[I2C_Counter++] = I2C_Data;					// buffer the received byte
			I2C_NewData = 1;
			if (I2C_Counter == I2C_SLAVE_BUFFER_SIZE) {
				I2C_Counter = 0;
				I2C_State = SLAVE_IDLE;
			}
			break;
		}
/*
		case SLAVE_AM_READ:											// SEND FIRST BYTE
		{															// ---------------
			I2C_TrayPointer = m_TraysArray[I2C_Tray].Variable;	// TrayPointer points towards the selected tray;
			I2CTRN = *I2C_TrayPointer++;						// send the first byte	
			I2CCONbits.SCLREL = 1;		
			I2C_BfrPtr = I2C_Buffer;							// I2C_BfrPtr points towards the buffer
			while (--I2C_Counter)								// Buffer the rest of the bytes to send
				*I2C_BfrPtr++ = *I2C_TrayPointer++;								
			I2C_Counter = I2C_BfrPtr - I2C_Buffer;				// Number of bytes in the buffer -> I2C Counter
			I2C_BfrPtr = I2C_Buffer;							// I2C_BfrPtr points towards the buffer
			I2C_State = SLAVE_READ;												
			break;
		}

		case SLAVE_READ:											// SEND ANOTHER BYTE
		{															// -----------------
			I2CTRN = *I2C_BfrPtr++;									
			I2CCONbits.SCLREL = 1;
			I2C_Counter--;		
			break;	
		}*/
	}

	I2CSTATbits.I2COV = 0;			       					   		// Clear the Overflow flag. This is not really necessary because clock stretch is
	I2CCONbits.SCLREL = 1;											// enabled and theoretically no overflow can happen. Release the clock line again to be sure
	_SI2CIF = 0;													// clear Slave interrupt flag
}
