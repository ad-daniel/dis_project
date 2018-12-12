/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  e_radio_swis.h
 *
 *  Interface to 802.15.4/ZigBee radio communication module developed at the
 *  Swarm-Intelligent Systems Group (SWIS), EPFL.
 *
 *  Adapted from initial version by X. Raemy.
 *
 *  $Author$
 *  $Date: 2008-04-30 09:29:37 +0200 (Wed, 30 Apr 2008) $
 *  $Revision$
 *  $HeadURL: https://grmapc10.epfl.ch/svn/students/Epuck/EpuckDevelopmentTree/library/contrib/radio_swis/e_radio_swis.h $
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef _RADIO_SWIS
#define _RADIO_SWIS


#define RADIO_SWIS_BASESTATION   0x00   // basestation has id zero (convention)
#define RADIO_SWIS_BROADCAST     0xFFFF // TinyOS broadcast address
#define RADIO_SWIS_DEFAULT_GROUP 0x7d   // TinyOS default group
#define RADIO_SWIS_MAXSIZE       108    // largest possible message (in bytes)
#define RADIO_SWIS_MAX_TXPWR     31     // maximum transmission power
#define RADIO_SWIS_MIN_TXPWR     3      // minimum transmission power

typedef enum _hw_att_state {
    RADIO_SWIS_HW_ATTENUATOR_0DB  = 0,  // attenuator off
    RADIO_SWIS_HW_ATTENUATOR_25DB = 1,  // attenuator on  (approx. -25dB)
} hw_att_state;

#define AM_MSGTYPE               0x0A   // AM_OSCOPE


#include "p30f6014a.h"


int e_radio_swis_present(); 
/*  returns: 0 = not connected, 1 = module is here. */

void e_radio_swis_init(
        unsigned char group,            // group id to be specified in TOS msgs
        unsigned int  address,          // id for this radio (use the e-puck #)
        hw_att_state  hw_attenuator,    // desired state of hw attenuator
        unsigned char tx_power          // transmission power: min=3, max=31
    );

unsigned char e_radio_swis_get_state();
void e_radio_swis_set_state(
        unsigned char mode              // 1=enable, 0=disable
    );

unsigned char e_radio_swis_get_hw_atten();
void e_radio_swis_set_hw_atten(
        hw_att_state mode               // desired state of hw attenuator
    );

unsigned char e_radio_swis_get_tx_pwr();
void e_radio_swis_set_tx_pwr(
        unsigned char pwr               // transmission power: min=3, max=31
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
int e_radio_swis_send_msgtype(
        unsigned char   dst_group,      // group id for message
        unsigned int    dst_address,    // destination (node id or broadcast)
        unsigned char*  packet,         // message data/payload
        int             packetsize,     // length of payload, < MAXSIZE
        unsigned char   amtype          // tos message type indicator
    ); 
/*  returns: 1 = success, 0 = failure */

int e_radio_swis_packet_ready_msgtype(
        unsigned char*  packet,         // *pre-allocated* array to put msg in
        int*            packetsize,     // size of msg actually written
        unsigned char*  amtype
    ); 
int e_radio_swis_packet_ready(
        unsigned char*  packet,         // *pre-allocated* array to put msg in
        int*            packetsize      // size of msg actually written
    ); 
/* 
 *  returns: 1 = packet & size successfully filled, 0 = nothing received.
 *
 *  note: allocate *packet to be RADIO_SWIS_MAXSIZE!
 */


#endif // _RADIO_SWIS
