// e_I2C_multimaster_crc.c

#include <I2C/e_I2C_master_module.h>
#include <motor_led/e_led.h>		// test purpose
#include <stdlib.h>

#include "e_I2C_multimaster_crc.h"
#include "e_I2C_slave.h"
#include "e_crc8.h"

enum {
 	MULTIMASTER_ERROR = 0,
	MULTIMASTER_NO_ERROR = 1,
 	MULTIMASTER_COLLISION = 2,
};

/* ---------- PROTOTYPES FOR PRIVATE FUNCTIONS ---------- */

static char m_is_bus_busy(void);
static inline char m_check_collision(void);

static char m_check_crc(char device_address, char* message, char length);

static void m_read_string_collision(char device_add, char read_buffer[], char start_address, char string_length);
static void m_write_string_collision(char device_add, char write_buffer[], char start_address, char string_length);
static char m_i2c_read_crc_collision(char device_addr);

static char m_read_string(char device_add, char read_buffer[], char start_address, char string_length);
static char m_write_string (char device_add, char write_buffer[], char start_address, char string_length);
static char m_i2c_read_crc(char device_addr, char* crc);



/* ---------- PRIVATE GLOBAL VARIABLES ---------- */

static unsigned int failed_packets = 0;	// record the number of failed packets
static unsigned int attempts = 0;	// record the number of attempts
static void (*m_fctCallback)(void) = NULL;

/* ---------- PUBLIC FUNCTIONS IMPLEMENTATION ---------- */

void e_I2C_multimaster_crc_init(void (*fctCallback)(void))
{
	m_fctCallback = fctCallback;
	
//	long i;
	I2CCONbits.I2CEN=0;			// disable I2C
	
	I2CBRG=150;				// frequency of SCL at 100kHz
//	IFS0bits.MI2CIF=0;			// clear master interrupt flag
//	IFS0bits.SI2CIF=0;			// clear slave interrupt flag
//	IPC3bits.MI2CIP=5;			// priority level
//	IEC0bits.SI2CIE=0;			// diseble slave I2C interrupt
	
	_MI2CIF=0;				// clear master interrupt flag
	_MI2CIE=1;				// enable master I2C interrupt
	
	I2CCONbits.I2CEN=1;			// enable I2C
	I2CSTATbits.S = 0;

//	for(i=10000;i;i--);
//	return 1;
}

void e_I2C_multimaster_crc_disable(){
//	e_i2cp_disable();
	IFS0bits.MI2CIF=0;			// clear master interrupt flag
	IEC0bits.MI2CIE=0;			// disable master I2C interrupt
}

void e_I2C_multimaster_crc_write_bytes(char device_address, char reg, char message[], char nb_bytes){
	char crc_error = 1;
	char failed = 0;
	
	device_address <<= 1;
	while(crc_error){
		m_write_string_collision(device_address, message, reg, nb_bytes);
		crc_error = m_check_crc(device_address, message, nb_bytes);
		if(crc_error == 1){
			failed = 1;
		}
	}
	
	if(failed == 1){
		failed_packets++;
	}
}


void e_I2C_multimaster_crc_read_bytes(char device_address, char reg, char buffer[], char nb_bytes){
	char crc_error = 1;
	char failed = 0;
		
	device_address <<= 1;
	while(crc_error){
		m_read_string_collision(device_address, buffer, reg, nb_bytes);
		crc_error = m_check_crc(device_address, buffer, nb_bytes);
		if(crc_error == 1){
			failed = 1;
		}
	}
	
	if(failed == 1){
		failed_packets++;
	}
}

unsigned int e_I2C_multimaster_crc_get_failed(void){
	return failed_packets;
}

void e_I2C_multimaster_crc_reset_failed(void){
	failed_packets = 0;
}


unsigned int e_I2C_multimaster_crc_get_attempts(void){
	return attempts;
}

void e_I2C_multimaster_crc_reset_attempts(void){
	attempts = 0;
}


/* ---------- PRIVATE FUNCTIONS IMPLEMENTATION ---------- */

char m_is_bus_busy(void)
{
	return I2CSTATbits.S;
}

inline char m_check_collision(void)
{
	if (I2CSTATbits.BCL != 0){
		I2CSTATbits.BCL = 0;
		return 1;
	} 
	
	return 0;
}

char m_check_crc(char device_address, char* message, char length){
	// return 0 if no error
	// return 1 if error
	unsigned char true_crc;
	unsigned char received_crc;
	
	received_crc = m_i2c_read_crc_collision(device_address);
	true_crc = e_crc8_compute(message, length);
	
	if (received_crc != true_crc) {
		attempts++;
	}
	
	return (received_crc != true_crc);
}

void m_read_string_collision(char device_add, char read_buffer[], char start_address, char string_length)
{
	char error = MULTIMASTER_NO_ERROR;
	char collision = 0;
	
	do{
		while(m_is_bus_busy());
		e_I2C_slave_inhibit();
		error = m_read_string(device_add, read_buffer, start_address, string_length);
		e_I2C_slave_uninhibit();
	
		if (error == MULTIMASTER_COLLISION){
//			e_i2c_stop();
			I2CCONbits.I2CEN=0;
			I2CCONbits.I2CEN=1;
			collision = 1;
		}
	} while(error == MULTIMASTER_COLLISION);
	
	if (collision & (m_fctCallback != NULL))
	{	
		// Call the callback function
		m_fctCallback();
	}
}

void m_write_string_collision(char device_add, char write_buffer[], char start_address, char string_length)
{
	char error = MULTIMASTER_NO_ERROR;
	char collision = 0;
	
	do{
		while(m_is_bus_busy());
		e_I2C_slave_inhibit();
		error = m_write_string(device_add, write_buffer, start_address, string_length);
		e_I2C_slave_uninhibit();
	
		if (error == MULTIMASTER_COLLISION){
//			e_i2c_stop();
			I2CCONbits.I2CEN=0;
			I2CCONbits.I2CEN=1;
			collision = 1;
		}
	} while(error == MULTIMASTER_COLLISION);

	if (collision & (m_fctCallback != NULL))
	{	
		// Call the callback function
		m_fctCallback();
	}
}

char m_i2c_read_crc_collision(char device_addr)
{
	char error = MULTIMASTER_NO_ERROR;
	char crc = 0;
	char collision = 0;
	
	do{
		while(m_is_bus_busy());
		e_I2C_slave_inhibit();
		error = m_i2c_read_crc( device_addr, & crc);
		e_I2C_slave_uninhibit();
	
		if (error == MULTIMASTER_COLLISION){
//			e_i2c_stop();
			I2CCONbits.I2CEN=0;
			I2CCONbits.I2CEN=1;
			collision = 1;
		}
	} while(error == MULTIMASTER_COLLISION);
		
	if (collision & (m_fctCallback != NULL))
	{	
		// Call the callback function
		m_fctCallback();
	}
	
	return crc;
}


// Retrieved from e_I2C_protocol.c (but was commented out in this file)
char m_read_string(char device_add, char read_buffer[], char start_address, char string_length)
{
	char error = MULTIMASTER_ERROR;
	char i = 0;
	
	while(error == MULTIMASTER_ERROR)
	{
		error = MULTIMASTER_NO_ERROR;

		error&=e_i2c_start();
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		
		error&=e_i2c_write(device_add);    		// Device address
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
 		
		error&=e_i2c_write(start_address);		// Address of first register to be read
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		
		error&=e_i2c_restart();
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		
		error&=e_i2c_write(device_add+1);    		// To change data direction ([bit 0]=1)
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}

		for (i=0;i < string_length;i++)
		{
	 		error&=e_i2c_read(&read_buffer[(int)i]);  // Read the next byte
			if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
			if (i == (string_length-1))		// The last byte to be read, must send nack
				error&=e_i2c_nack();
			else
				error&=e_i2c_ack();		// Not the last byte, send ack
		}
		e_i2c_stop();             			// End read cycle
		if(error == MULTIMASTER_NO_ERROR)	// no error
			break;
		e_I2C_multimaster_crc_init(m_fctCallback);	// Reset in case of error
	}

	return MULTIMASTER_NO_ERROR;
}

// Same as above
char m_write_string (char device_add, char write_buffer[], char start_address, char string_length)
{
	char error = MULTIMASTER_ERROR;
	int i = 0;

	while(error == MULTIMASTER_ERROR)
	{
		error = MULTIMASTER_NO_ERROR;
		
		error&=e_i2c_start();
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		
		error&=e_i2c_write(device_add);    		// Writing the device (slave) I2C address
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		
		error&=e_i2c_write(start_address);		// Device register address
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		
		for (i=0;i<string_length;i++){
			error&=e_i2c_write(write_buffer[i]);	// Data to device
			if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		}
		
		error&=e_i2c_stop();				// Ending the communication	
		if(error == MULTIMASTER_NO_ERROR)		// No error
			break;
		e_I2C_multimaster_crc_init(m_fctCallback);	// Reset in case of error
	}

	return MULTIMASTER_NO_ERROR;
}

char m_i2c_read_crc(char device_addr, char* crc){
	char error = MULTIMASTER_ERROR;
	char value;
	
	while(error == MULTIMASTER_ERROR)
	{
		error = MULTIMASTER_NO_ERROR;

		if (attempts > 1){
			error = MULTIMASTER_NO_ERROR;
		}	
		
		
		error&=e_i2c_start();
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		
		error&=e_i2c_write(device_addr+1);
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		
		error&=e_i2c_read(&value);
		if (m_check_collision() != 0) {return MULTIMASTER_COLLISION;}
		
		error&=e_i2c_nack();
		error&=e_i2c_stop();
		if(error == MULTIMASTER_NO_ERROR)		// No error
			break;
		e_I2C_multimaster_crc_init(m_fctCallback);	// Reset in case of error
	}	
	
	*crc = value;
	//return value;
	return MULTIMASTER_NO_ERROR;
}


