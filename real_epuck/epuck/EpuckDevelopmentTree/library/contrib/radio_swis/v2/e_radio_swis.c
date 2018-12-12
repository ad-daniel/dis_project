/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  e_radio_swis.h
 *
 *  Interface to 802.15.4/ZigBee radio communication module developed at the
 *  Swarm-Intelligent Systems Group (SWIS), EPFL.
 *
 *	Current Version 1.1; october 2009
 *
 *	Version 1.0 for TinyOS 1.x: Xavier Raemy (SWIS), Christopher Cianci (SWIS) 
 *	Version 1.1 for TinyOS 2.x: Florian Vaussard (DISAL) 
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*! \file
 * \ingroup ComModule
 * \brief Radio communication using the Zigbee-compliant turret
 *
 * Before using this module, call e_radio_swis_init() or e_radio_swis_init_default(). \n Here is a simple code example that compile, 
 * broadcasting a one byte counter.
 * 
 * \code
#include <p30f6014A.h>

#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <motor_led/e_led.h>

#include "e_radio_swis.h"

static char timer_fired = 0;

int main(void)
{
	unsigned char own_counter;
	unsigned char receive_buffer[28];
	int nb_bytes;
	long int i;

	e_init_port();
	
	// Initialize the communication turret, and check the version number
	if (e_radio_swis_init(RADIO_SWIS_DEFAULT_GROUP, 0x01, RADIO_SWIS_HW_ATTENUATOR_0DB, RADIO_SWIS_SW_ATTENUATOR_0DB) < 2) {
		// not the right version number
		return 0;
	}

	// Init the timer 5
	T5CON = 0;
	T5CONbits.TCKPS=3;		// prescsaler = 256
	TMR5 = 0;			// clear timer 5
	PR5 = (FCY/256)/5;		// interrupt frequency
	IFS1bits.T5IF = 0;		// clear interrupt flag
	IEC1bits.T5IE = 1;		// set interrupt enable bit
	T5CONbits.TON = 1;		// start Timer5

	while(1){
		if (timer_fired){
			timer_fired = 0;
			own_counter++;
			// Broadcast the counter to member of the default group
			e_radio_swis_send(RADIO_SWIS_DEFAULT_GROUP, RADIO_SWIS_BROADCAST, &own_counter, 1);	
		}
		INTERRUPT_OFF();		// Avoid interrupting during the reception of data
		if (e_radio_swis_packet_ready(receive_buffer, &nb_bytes) != 0) {
			// We received new data
			
			// DO SOMETHING WITH receive_buffer[0]
		}
		INTERRUPT_ON();
		
		for (i=0; i<10000; i++){asm("nop");}
	}
	
	return 0;
}
 
void __attribute__((interrupt, auto_psv, shadow)) _T5Interrupt(void){
	IFS1bits.T5IF = 0;             // clear interrupt flag
	timer_fired = 1;
}
 * \endcode
 * 
 * As we use only one byte, it would be of course better to transmit only one byte through the I2C bus (the example above will transmit one byte 
 * during e_radio_swis_send(), but 
 * will download the full buffer during e_radio_swis_packet_ready()). To do that, just call e_radio_swis_set_payload_length() at the begining of your 
 * program, and use e_radio_swis_get_max_payload_length() to assess the maximum limit that a module can handle. 
 *
 * \warning This library IS NOT COMPATIBLE with the old module (using TinyOS 1.x). Instead, use the new version of the module running TinyOS 2.x
 * 
 * \author Version 1.0 for TinyOS 1.x: Xavier Raemy, Christopher Cianci \n Version 1.1 for TinyOS 2.x: Florian Vaussard (firmware, library and doc), Yuanfang Ma (firmware)
 */

#include <stdlib.h>
#include <motor_led/e_epuck_ports.h>

#include "e_radio_swis.h"
#include "e_radio_swis_const.h"
#include "e_I2C_multimaster_crc.h"
#include "e_I2C_slave.h"

// ********** Constants *********
//#define COM_MODULE_I2C_ADDR 	(COM_MODULE_I2C_ADDR << 1)	/*!<\brief Module i2c address */

// ********** Global variables **********
static i2c_operation_mode m_mode;

/*!<\brief set once at the begining of the program by retrieving the value from the ComModule board */
static unsigned char m_payload_size = 0;	
static unsigned char m_max_payload_size = 0;
static unsigned char old_destinationgroup = 0;
static unsigned int old_destinationaddr = 0;

// ********** Private functions prototypes **********
static void ReadRegister(unsigned char registeraddr, unsigned char* value_array, unsigned char length);
static void WriteRegister(unsigned char registeraddr, unsigned char* value_array, unsigned char length);
static void ReadPayloadLength(void);
static void ReadMaxPayloadLength(void);

// ********** Public functions **********

/*! \brief Initialize the communication turret, and set some important values (Zigbee's address, group, emission power)
 *
 * This function first set the i2c bus, then wait for the module to reply, and finally read and set important 
 * registers.
 * \param group 0x00 to 0xff. Split the robots into logical, separate groups. The module will only receive packets 
 * from that group ID.
 * \param address 0x0000 to 0xfffe. Address of the module inside a logical group. The address 0xffff is reserved 
 * as the broadcast address
 * \param hw_attenuator COM_MODULE_HW_ATTENUATOR_0DB (external attenuator off) or 
 * COM_MODULE_HW_ATTENUATOR_25DB (external attenuator on)
 * \param tx_power 1 (-25dBm) to 31 (full power: 0dBm): set the CC2420 transmission power, a lower power will reduce the range and the power consumption
 * \return Turret's firmware version (current version: 0x02). Can be used for compatibility check, only implemented 
 * in the new firmware (the one using TinyOS 2)
 * \see e_radio_swis_init_default()
 */
unsigned char e_radio_swis_init(
	i2c_operation_mode mode, 
 	void (*fct_callback)(void),
 	unsigned char group, 
  	unsigned int address, 
   	hw_att_state  hw_attenuator, 
    	sw_att_state tx_power
	) 
{
	unsigned char ret_val = e_radio_swis_init_default(mode, fct_callback, hw_attenuator, tx_power);
	
	e_radio_swis_set_group(group);
	e_radio_swis_set_address(address);

	return ret_val;
}

/*! \brief Initialize the communication turret, and set some important values (emission power)
 * 
 * Do exactly the same things than e_radio_swis_init() but leave the module's address and group setting unchanged.
 *
 * \param hw_attenuator COM_MODULE_HW_ATTENUATOR_0DB (external attenuator off) or 
 * COM_MODULE_HW_ATTENUATOR_25DB (external attenuator on)
 * \param tx_power 1 (-25dBm) to 31 (full power: 0dBm): set the CC2420 transmission power, a lower power will reduce the range and the power consumption
 * \return Turret's firmware version (current version: 0x02). Can be used for compatibility check, only implemented 
 * in the new firmware (the one using TinyOS 2)
 * \see e_radio_swis_init()
 */
unsigned char e_radio_swis_init_default(
	i2c_operation_mode mode, 
 	void (*fct_callback)(void),
 	hw_att_state hw_attenuator, 
  	sw_att_state tx_power
	) 
{
	m_mode = mode;
	
	if (mode == RADIO_SWIS_SLAVE_MASTER) {
		// The turret is working as a slave -> e-puck must poll for new packets
		e_I2C_multimaster_crc_init( NULL );
	} else if (mode == RADIO_SWIS_MULTIMASTER) {
		// The turret is working in multimaster mode -> also enable the slave on the e-puck to receive the command
		e_I2C_multimaster_crc_init(fct_callback);
		e_I2C_slave_init( EPUCK_I2C_ADDR, fct_callback);
	}

	while (e_radio_swis_present() == 0); // wait till module online

	ReadPayloadLength();
	ReadMaxPayloadLength();
	ReadRegister(DESTGROUP_REG_ADDR , &old_destinationgroup, DESTGROUP_REG_SIZE);
	ReadRegister(DESTADDR_REG_ADDR, (unsigned char*)&old_destinationaddr, DESTADDR_REG_SIZE);	
	e_radio_swis_set_i2c_mode(mode);
	e_radio_swis_set_hw_atten(hw_attenuator);
	e_radio_swis_set_tx_pwr(tx_power);
	
	return e_radio_swis_get_version_number();
}

/*! \brief Is the module plugged?
 *
 * \return 0: The module is not plugged \n 1: The module is plugged
 */
int e_radio_swis_present() {
	unsigned char value;
	value = e_radio_swis_status();
	if (value == 0xFF) // value read are always 0xff when module not plugged in
		return 0; 
	return 1;
}

/*! \brief Set the size (in bytes) of the payload used by the communication module
 * 
 * Internally the communication module is using a payload of 28 bytes (by default, set at compile time). But the e-puck can send
 * less data, which can save i2c ressources (the cc2420 chip will anyway send a full payload of 28 bytes, the only way to
 * change that is to recompile the module's firmware and load it again). Prior to this, set the payload's length by using this function.
 *
 * Once the change done, use payload of this size in the function e_radio_swis_send(). And thus the packets received by e_radio_swis_packet_ready() will
 * also have a payload of that size (you will receive only payloads of the correct size, other are discarded internally by the module).
 * 
 * Regarding this fact, it is advisable to set once the payload's length at startup, and stick to this value thereafter. If you don't, it is
 * more than likly that different modules will use different sizes if not well synchronized, leading to a massive lost of packets of
 * incompatible size...
 *
 * \param payload_length Length of the payloadpackets
 */
void e_radio_swis_set_payload_length(unsigned char payload_length){
	if (payload_length > m_max_payload_size) {
		// Too long, clip to the maximum length
		payload_length = m_max_payload_size;
	}
	
	WriteRegister(PAYLOAD_LENGTH_REG_ADDR, &payload_length, PAYLOAD_LENGTH_REG_SIZE);
	
	// Wait for the length to be synchronized
	while(e_radio_swis_status() & SYNC_LENGTH) {}
	
	// Store locally this size (avoid unnecessary i2c usage)
	m_payload_size = payload_length;
}


/*! \brief Enable or disable the radio module
 * 
 * \param mode 0: Disable the radio \n 1: Enable the radio
 */
void e_radio_swis_set_state(unsigned char mode) {
	unsigned char old_value;	
	ReadRegister(CONFIG_REG_ADDR, &old_value, CONFIG_REG_SIZE);
	if (mode != 0){
		// enable the radio
		old_value |= RADIO_ENABLED_FLAG;
		WriteRegister(CONFIG_REG_ADDR, &old_value, CONFIG_REG_SIZE);
	} else {
		// disable the radio
		old_value &= ~RADIO_ENABLED_FLAG;
		WriteRegister(CONFIG_REG_ADDR, &old_value, CONFIG_REG_SIZE);
	}
}

/*! \brief Enable or disable the hardware attenuator
 * 
 * When the hardware attenuator circuit is enabled, the signal is attenuated by about 25dBm
 *
 * \param mode RADIO_SWIS_HW_ATTENUATOR_0DB (external attenuator off) or 
 * RADIO_SWIS_HW_ATTENUATOR_25DB (external attenuator on)
 * \see e_radio_swis_init(), e_radio_swis_init_default()
 */
void e_radio_swis_set_hw_atten(hw_att_state mode) {
	unsigned char old_value;	
	ReadRegister(CONFIG_REG_ADDR, &old_value, CONFIG_REG_SIZE);
	if (mode != 0){
		old_value |= HARDWAREATT_SET_FLAG;
		WriteRegister(CONFIG_REG_ADDR, &old_value, CONFIG_REG_SIZE);
	} else {
		old_value &= ~HARDWAREATT_SET_FLAG;
		WriteRegister(CONFIG_REG_ADDR, &old_value, CONFIG_REG_SIZE);
	}
}

void e_radio_swis_set_i2c_mode(i2c_operation_mode current_mode)
{
	unsigned char old_value;	
	ReadRegister(CONFIG_REG_ADDR, &old_value, CONFIG_REG_SIZE);
	if (current_mode == RADIO_SWIS_MULTIMASTER){
		// Set the flag
		old_value |= I2C_MODE_MULTIMASTER;
		WriteRegister(CONFIG_REG_ADDR, &old_value, CONFIG_REG_SIZE);
	} else {
		// Reset the flag
		old_value &= ~I2C_MODE_MULTIMASTER;
		WriteRegister(CONFIG_REG_ADDR, &old_value, CONFIG_REG_SIZE);
	}
}

/*! \brief Enable or disable the software attenuator
 * 
 * The emission power of the CC2420 chip can be configured
 *
 * \param pwr RADIO_SWIS_SW_ATTENUATOR_25DB = 3 (-25dBm) to RADIO_SWIS_SW_ATTENUATOR_0DB = 31 (full power: 0dBm): set the CC2420 transmission 
 * power, a lower power will reduce the range and the power consumption
 * \see e_radio_swis_init(), e_radio_swis_init_default()
 */
void e_radio_swis_set_tx_pwr(sw_att_state pwr) {
	if (pwr > RADIO_SWIS_SW_ATTENUATOR_0DB)
		pwr = RADIO_SWIS_SW_ATTENUATOR_0DB;
	WriteRegister(SOFTATT_REG_ADDR, &pwr, SOFTATT_REG_SIZE);
}

/*! \brief Set the group ID
 * Split the robots into logical, separate groups. The module will only receive packets 
 * from that group ID.
 *
 * \param grp_id 0x00 to 0xff. Group ID. Default is RADIO_SWIS_DEFAULT_GROUP
 * \see e_radio_swis_init()
 */
void e_radio_swis_set_group(unsigned char grp_id) {
	WriteRegister(OWNGROUP_REG_ADDR, &grp_id, OWNGROUP_REG_SIZE);
}

/*! \brief Set the module's address
 *
 * \param address 0x0000 to 0xfffe. Address of the module inside a logical group. The address 0xffff is reserved 
 * as the broadcast address (RADIO_SWIS_BROADCAST)
 * \see e_radio_swis_init()
 */
void e_radio_swis_set_address(unsigned int address) {
	WriteRegister(OWNADDR_REG_ADDR, (unsigned char*)&address, OWNADDR_REG_SIZE);
}

/*! \brief Return the size (in bytes) of the payload used by the communication module
 * 
 * Internally the communication module is using a payload of 28 bytes (by default, set at compile time). But the e-puck can send
 * less data, which can save i2c ressources (the cc2420 chip will anyway send a full payload of 28 bytes, the only way to
 * change that is to recompile the module's firmware and load it again). Prior to this, set the payload's length by using e_radio_swis_set_payload_length().
 *
 * Once the change done, use payload of this size in the function e_radio_swis_send(). And thus the packets received by e_radio_swis_packet_ready() will
 * also have a payload of that size (you will receive only payloads of the correct size, other are discarded internally by the module).
 * 
 * Regarding this fact, it is advisable to set once the payload's length at startup, and stick to this value thereafter. If you don't, it is
 * more than likly that different modules will use different sizes if not well synchronized, leading to a massive lost of packets of
 * incompatible size...
 *
 * \return Packets length
 * \see e_radio_swis_get_max_payload_length(), e_radio_swis_set_payload_length()
 */
unsigned char e_radio_swis_get_payload_length(){
	return m_payload_size;
}

unsigned char e_radio_swis_get_max_payload_length(){
	return m_max_payload_size;
}

/*! \brief Return the state of the hardware attenuator circuit
 * 
 * If the attenuator is enabled, the range is greatly reduced
 *
 * \return 0: The attenuator is disabled \n 1: The attenuator is enabled
 * \see e_radio_swis_init(), e_radio_swis_init_default(), e_radio_swis_set_hw_atten()
 */
unsigned char e_radio_swis_get_hw_atten() {
	unsigned char value;
	ReadRegister(CONFIG_REG_ADDR, &value, CONFIG_REG_SIZE);
	if ( value & HARDWAREATT_SET_FLAG);
		return 1;
	return 0;
}

/*! \brief Return the state of the radio module
 * \return 0: The radio is disabled \n 1: The radio is enabled
 * \see e_radio_swis_set_state()
 */
unsigned char e_radio_swis_get_state() {
	unsigned char value;
	ReadRegister(CONFIG_REG_ADDR, &value, CONFIG_REG_SIZE);
	if ( value & RADIO_ENABLED_FLAG);
		return 1;
	return 0;
}

/*! \brief Return the state of the software attenuator
 * 
 * Return the emission power of the CC2420 chip
 *
 * \return 1 (-25dBm) to 31 (full power: 0dBm): the CC2420 transmission power
 * \see e_radio_swis_set_tx_pwr()
 */
unsigned char e_radio_swis_get_tx_pwr() {
	unsigned char value;
	ReadRegister(SOFTATT_REG_ADDR, &value, SOFTATT_REG_SIZE);
	return value;
}

/*! \brief Return the node's group ID
 * 
 * The module will only receive packets from that group ID.
 *
 * \return 0x00 to 0xff
 * \see e_radio_swis_set_group()
 */
unsigned char e_radio_swis_get_group() {
	unsigned char value;
	ReadRegister(OWNGROUP_REG_ADDR, &value, OWNGROUP_REG_SIZE);
	return value;
}

/*! \brief Return the node's address
 * 
 * \return  0x0000 to 0xfffe. Address of the module inside a logical group. The address 0xffff is reserved 
 * as the broadcast address (RADIO_SWIS_BROADCAST)
 * \see e_radio_swis_set_address()
 */
unsigned int e_radio_swis_get_address() {
	unsigned int value;
	ReadRegister(OWNADDR_REG_ADDR, (unsigned char*)&value, OWNADDR_REG_SIZE);
	return value;
}

/*! \brief Return the module's firmware version number
 * 
 * \return Current version: 0x02
 */
unsigned char e_radio_swis_get_version_number(){
	unsigned char value;
	ReadRegister(VERSION_REG_ADDR, (unsigned char*)&value, VERSION_REG_SIZE);
	return value;	
}

/*! \brief Return the status register
 * 
 * \return PACKET_READY_FLAG, TX_IDLE_FLAG, PACKET_LOST_FLAG, TX_SEND_ERROR
 */
unsigned char e_radio_swis_status() {
	unsigned char value;
	ReadRegister(STATUS_REG_ADDR, &value, STATUS_REG_SIZE);
	return value;
}

/*! \brief Send a packet of bytes over the radio
 * 
 * Send of packet of 'packetsize' bytes to a robot whose address is 'destinationaddress' and whose group is 
 * 'destinationgroup'
 *
 * \param dst_group Destination group
 * \param dst_address Destination address
 * \param packet Pointer to an array of bytes to send
 * \param packetsize Length in bytes of the packet pointed by 'packet'. This size must be under or equal to the value 
 * returned by GetPacketSize(). If the size is below this value, the module will anyway send a full message (thus 
 * including some meaningless bytes that e_radio_swis_packet_ready() will retrieve)
 * \return 0: Error (packet too long or module returned an error) \n 1: OK (this is not an acknowledgment that the 
 * packet has arrived)
 * \see e_radio_swis_packet_ready()
 */
int e_radio_swis_send(unsigned char dst_group, unsigned int dst_address, unsigned char* packet, int packetsize) {
	unsigned char send_command = 1;
	
	if (packetsize > m_payload_size)
		return 0; // not allowed -> return error

	while ((e_radio_swis_status() & TX_IDLE_FLAG) == 0); // wait till module is ready to transmit

/*	if (dst_group != old_destinationgroup) {
		WriteRegister(DESTGROUP_REG_ADDR, &destinationgroup, DESTGROUP_REG_SIZE);
		old_destinationgroup = destinationgroup;
	}*/
	if (dst_address != old_destinationaddr) {
		// Send only if the destination address has changed
		WriteRegister(DESTADDR_REG_ADDR, (unsigned char*)&dst_address, DESTADDR_REG_SIZE);
		old_destinationaddr = dst_address;
	}
	WriteRegister(SEND_BUFFER_ADDR, packet, packetsize);
	WriteRegister(SEND_REG_ADDR, &send_command, SEND_REG_SIZE);	// send packet !
	
	// maybe wait here
	
	if (e_radio_swis_status() & TX_SEND_ERROR) // flag TX error set -> return error
		return 0;

	
	return 1; // return OK
}

/*! \brief Ask the module if a new packet has arrived, and if this is the case download it
 * 
 * If no new packet is available, this function returns immediatly 0. Otherwise, the new packet is retrieved and the 
 * function returns 1.
 *
 * \param packet Pointer to a free buffer, in order to retrieve the new packet (for the size of this buffer, see the 
 * value returned by GetPacketSize())
 * \param packetsize Pointer to an integer, will store the amount of bytes read (same as GetPacketSize())
 * \return 0: No new packet available \n 1: A new packet is available
 * \see e_radio_swis_send()
 */
int e_radio_swis_packet_ready(unsigned char* packet, int* packetsize) {
	if ((packet == NULL) || (packetsize == NULL))
		return 0; 				// return withour reading NULL pointer means uC reset if write
	
	if (m_mode == RADIO_SWIS_MULTIMASTER) {
		if (e_I2C_slave_new_data()){
			unsigned char data;
			// Wait for the bus to be free
			while (e_I2C_slave_is_bus_busy());
			e_I2C_slave_get_data(&data, 1);
			if ( data != 0x55 ){
				return;
			}
		}		
	}
	
	if (e_radio_swis_status() & PACKET_READY_FLAG) { 		// a packet is ready !
		*packetsize = m_payload_size;

		ReadRegister(REC_BUFFER_ADDR, packet, *packetsize);
		return 1;
	}
	return 0; // no packet ready -> 0
}

void e_radio_swis_refresh(void)
{
	if (m_mode == RADIO_SWIS_MULTIMASTER){
		e_I2C_slave_refresh();
	}
}

// ********** Private functions **********

/*! \brief Interface for reading a register through i2c, using the i2c_crc library
 *
 * \param registeraddr 0x00 to 0xff. Address of the register (depends on the device)
 * \param value_array Pointer to an array of bytes to send
 * \param length Number of bytes to send
 * \see WriteRegister()
 */
void ReadRegister(unsigned char registeraddr, unsigned char* value_array, unsigned char length) {
	e_I2C_multimaster_crc_read_bytes(TURRET_I2C_ADDR, registeraddr, value_array, length);	
}

/*! \brief Interface for writing a register through i2c, using the i2c_crc library
 *
 * \param registeraddr 0x00 to 0xff. Address of the register (depends on the device)
 * \param value_array Pointer to an array of bytes to store the result
 * \param length Number of bytes to read
 * \see ReadRegister()
 */
void WriteRegister(unsigned char registeraddr, unsigned char* value_array, unsigned char length) {
	e_I2C_multimaster_crc_write_bytes(TURRET_I2C_ADDR, registeraddr, value_array, length);
}

/*! \brief Read the size of packets used by the communication module
 * 
 * Internally, the module is using packets of that size. Thus you have better to use packets of that size. The 
 * packet's size is stored in the global variable m_payload_size
 *
 */
void ReadPayloadLength(void){
	ReadRegister(PAYLOAD_LENGTH_REG_ADDR, &m_payload_size, PAYLOAD_LENGTH_REG_SIZE);
}

/*! \brief Read the size of packets used by the communication module
 * 
 * Internally, the module is using packets of that size. Thus you have better to use packets of that size. The 
 * packet's size is stored in the global variable m_payload_size
 *
 */
void ReadMaxPayloadLength(void){
	ReadRegister(MAX_PAYLOAD_LENGTH_REG_ADDR, &m_max_payload_size, MAX_PAYLOAD_LENGTH_REG_SIZE);
}


