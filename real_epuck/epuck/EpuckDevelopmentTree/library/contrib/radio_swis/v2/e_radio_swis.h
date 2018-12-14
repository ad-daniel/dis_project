
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


/*! \defgroup ComModule Radio communication
 * 
 * \section comment_sect Introduction
 * This module is made for the Zigbee-compliant turret developped by Xavier Raemy and Christopher Cianci (SWIS laboratory, which
 * later became the DISAL laboratory) at the Ecole Polytechnique Fédérale de Lausanne. A CC2420 802.15.4 transceiver and a TI MSP430
 * microcontroller are both embedded on this turret. This module is connected to the e-puck using the 
 * i2c bus, e-puck acting as a master and the MSP430 as a slave. Thus sending or receiving packets is simply a matter 
 * of reading and writing registers.
 *
 * \section sec_history History
 * The first version used TinyOS 1 (www.tinyos.net) as a framework for developping the firmware running on the
 * turret's microcontroller. As things evolved, TinyOS 2 came with several major improvements, leading to the
 * development of a second firmware. As several improvements have been made, mainly related to the way of using the
 * i2c bus (writing and reading streams of bytes for improved speed, addition of a CRC for improved reliability), this 
 * library has been slightly modified to take those changes into account.
 *
 * \author Version 1.0 for TinyOS 1.x: Xavier Raemy, Christopher Cianci \n Version 1.1 for TinyOS 2.x: Florian Vaussard (firmware, library and doc), Yuanfang Ma (firmware)
 */

#include "e_radio_swis_const.h"

#ifndef _RADIO_SWIS
#define _RADIO_SWIS

//#define RADIO_SWIS_MAX_TXPWR     31     // maximum transmission power
//#define RADIO_SWIS_MIN_TXPWR     3      // minimum transmission power

typedef enum _hw_att_state {
    RADIO_SWIS_HW_ATTENUATOR_0DB  = 0,  	/*!<\brief Hardware attenuator disabled: 0dBm of attenuation */
    RADIO_SWIS_HW_ATTENUATOR_25DB = 1,  	/*!<\brief Hardware attenuator enabled: approx. 25dBm of attenuation */
} hw_att_state;

// Those values are defined according to the CC2420 datasheet
typedef enum _sw_att_state {
	RADIO_SWIS_SW_ATTENUATOR_0DB 	=	31,	/*!<\brief Software attenuator: 0dBm attenuation (full power) */
	RADIO_SWIS_SW_ATTENUATOR_1DB 	=	27,	/*!<\brief Software attenuator: 1dBm attenuation */
	RADIO_SWIS_SW_ATTENUATOR_3DB 	=	23,	/*!<\brief Software attenuator: 3dBm attenuation */
	RADIO_SWIS_SW_ATTENUATOR_5DB 	=	19,	/*!<\brief Software attenuator: 5dBm attenuation */
	RADIO_SWIS_SW_ATTENUATOR_7DB 	=	15,	/*!<\brief Software attenuator: 7dBm attenuation */
	RADIO_SWIS_SW_ATTENUATOR_10DB 	=	11,	/*!<\brief Software attenuator: 10dBm attenuation */
	RADIO_SWIS_SW_ATTENUATOR_15DB 	=	7,	/*!<\brief Software attenuator: 15dBm attenuation */
	RADIO_SWIS_SW_ATTENUATOR_25DB 	=	3,	/*!<\brief Software attenuator: 25dBm attenuation */
} sw_att_state;

#define RADIO_SWIS_BASESTATION   	0x00   /*!<\brief basestation has id zero (convention) */
#define RADIO_SWIS_BROADCAST     	0xFFFF /*!<\brief TinyOS broadcast address */
#define RADIO_SWIS_DEFAULT_GROUP 	0x22   /*!<\brief TinyOS default group */

// RADIO_SWIS_MAXSIZE is replaced by a call to the function GetPacketSize()
//#define RADIO_SWIS_MAXSIZE       108    // largest possible message (in bytes)

// return the ComBoard's firmware versionun
unsigned char e_radio_swis_init(
	i2c_operation_mode mode, 
 	void (*fct_callback)(void),
        unsigned char group,            // group id to be specified in TOS msgs
        unsigned int  address,          // id for this radio (use the e-puck #)
        hw_att_state  hw_attenuator,    // desired state of hw attenuator
        sw_att_state tx_power           // transmission power: min=3, max=31
    );

// return the ComBoard's firmware versionun
unsigned char e_radio_swis_init_default(
	i2c_operation_mode mode, 
 	void (*fct_callback)(void),
	hw_att_state hw_attenuator,	// desired state of hw attenuator
	sw_att_state tx_power		// transmission power: min=3, max=31
      );

int e_radio_swis_present(); 
/*  returns: 0 = not connected, 1 = module is here. */

unsigned char e_radio_swis_get_version_number();		// return the ComBoard's firmware version

unsigned char e_radio_swis_get_max_payload_length();		// return the maximum possible size for the payload, 
								// useful when using e_radio_swis_set_payload_length()
								
unsigned char e_radio_swis_get_payload_length();		// return the size of the payload handled by the ComModule
void e_radio_swis_set_payload_length(
	unsigned char payload_length
      );
	
unsigned char e_radio_swis_get_state();
void e_radio_swis_set_state(
        unsigned char mode              // 1=enable, 0=disable
    );

unsigned char e_radio_swis_get_hw_atten();
void e_radio_swis_set_hw_atten(
        hw_att_state mode               // desired state of hw attenuator
    );

void e_radio_swis_set_i2c_mode(
        i2c_operation_mode current_mode
    );

unsigned char e_radio_swis_get_tx_pwr();
void e_radio_swis_set_tx_pwr(
        sw_att_state pwr               // transmission power: min=3, max=31
    );

unsigned char e_radio_swis_get_group();
void e_radio_swis_set_group(
        unsigned char grp_id            // ignore msgs not from this group
    );

unsigned int e_radio_swis_get_address();
void e_radio_swis_set_address(
        unsigned int address            // current address of this module
    );

unsigned char e_radio_swis_status();    // return status register 
/*
 *  bit 0 : PACKET_READY_FLAG : a packet is in receive buffer. auto clear when 
 *                              last byte of receive buffer is read.
 *  bit 1 : TX_IDLE_FLAG      : module is ready to send a new packet.
 *  bit 2 : PACKET_LOST_FLAG  : a packet was in receive buffer when another 
 *                              packet arrived. 2nd packet is lost. auto clear 
 *                              when last byte of receive buffer is read.
 *  bit 3 : TX_SEND_ERROR     : error during packet send. auto clear during 
 *                              next send attempt.
 */

int e_radio_swis_send(
        unsigned char   dst_group,      // group id for message
        unsigned int    dst_address,    // destination (node id or broadcast)
        unsigned char*  packet,         // message data/payload
        int             packetsize      // length of payload, < MAXSIZE
    ); 

int e_radio_swis_packet_ready(
        unsigned char*  packet,         // *pre-allocated* array to put msg in
        int*            packetsize      // size of msg actually written
    ); 

void e_radio_swis_refresh(void);

#endif
