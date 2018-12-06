// e_I2C_slave.h

#ifndef _SLAVE_I2C_H
#define _SLAVE_I2C_H

/***** General I2C functions *****/
/**
 *	\brief Initialize the I2C module.
 *
 *	Initialize the I2C module. Call this function prior to any others.
 *
 *	\param	ModuleAddress
 *			Address of the slave module. The address must be a 7-bits address.
 *
 *	\param	TraysArray
 *			Mapping between variables and trays.
 */
void e_I2C_slave_init(char module_address, void (*fctCallback)(void));

void e_I2C_slave_inhibit(void);
void e_I2C_slave_uninhibit(void);

/***** Slave functions *****/
/**
 *	\brief	Update the address of the slave module.
 *
 *	Set the address of the slave I2C module.
 *
 *	\param module_address I2C slave address
 */
void e_I2C_slave_set_address(unsigned char module_address);
unsigned char e_I2C_slave_get_address(void);

void e_I2C_slave_refresh(void);

char e_I2C_slave_new_data(void);
unsigned char e_I2C_slave_get_data_length(void);
char e_I2C_slave_get_data(unsigned char* dst_buffer, unsigned char len);

char e_I2C_slave_is_bus_busy(void);

#endif // _SLAVE_I2C_H
