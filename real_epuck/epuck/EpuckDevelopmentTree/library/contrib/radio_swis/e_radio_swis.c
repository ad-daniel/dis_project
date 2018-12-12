/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  e_radio_swis.c
 *
 *  Interface to 802.15.4/ZigBee radio communication module developed at the
 *  Swarm-Intelligent Systems Group (SWIS), EPFL.
 *
 *  Adapted from initial version by X. Raemy.
 *
 *  $Author$
 *  $Date: 2008-04-30 09:29:37 +0200 (Wed, 30 Apr 2008) $
 *  $Revision$
 *  $HeadURL: https://grmapc10.epfl.ch/svn/students/Epuck/EpuckDevelopmentTree/library/contrib/radio_swis/e_radio_swis.c $
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include "e_radio_swis.h"

#include <stdlib.h>
#include "motor_led/e_epuck_ports.h"
#include "I2C/e_I2C_protocol.h"
#include "I2C/e_I2C_master_module.h"


#define RADIO_SWIS_I2C_ADDR (0x3F<<1)
#define BUFFER_DATA_LENGTH  (RADIO_SWIS_MAXSIZE + 14)


#define STATUS_REG_ADDR     0x00  // status: PACKET_LOST_FLAG,TX_IDLE,PACKET_READY
#define CONFIG_REG_ADDR     0x01  // config like ..... hardware att. sleep...
#define SEND_REG_ADDR       0x02  // <> 0 -> send sendbuffer
#define SOFTATT_REG_ADDR    0x03
#define OWNGROUP_REG_ADDR   0x04
#define OWNADDRL_REG_ADDR   0x05
#define OWNADDRH_REG_ADDR   0x06
#define SEND_BUFFER_START   0x07
#define SEND_BUFFER_END     (SEND_BUFFER_START + BUFFER_DATA_LENGTH)
#define REC_BUFFER_START    (SEND_BUFFER_END + 1)
#define REC_BUFFER_END      (REC_BUFFER_START + BUFFER_DATA_LENGTH)

#define AM_MSGTYPE_IN_PACKET_OFFSET 0x08
#define ADDRLDATA_IN_PACKET_OFFSET  0x06
#define ADDRHDATA_IN_PACKET_OFFSET  0x07
#define TYPEDATA_IN_PACKET_OFFSET   0x08
#define GROUPDATA_IN_PACKET_OFFSET  0x09
#define FIRSTDATA_IN_PACKET_OFFSET  0x0A // how many byte after packet start should I write data ?

//status reg
#define PACKET_READY_FLAG   0x01 // bit 1
#define TX_IDLE_FLAG        0x02 // bit 2
#define PACKET_LOST_FLAG    0x04 // bit 3
#define TX_SEND_ERROR       0x08 // bit 4

//config reg
#define HARDWAREATT_SET_FLAG    0x01 //bit 1
#define RADIO_ENABLED_FLAG      0x80 //bit 8

// send reg
#define REQUEST_TO_SEND_FLAG    0x01

// local helper functions
static unsigned char ReadRegister(unsigned char registeraddr) {
    return (0x00FF & e_i2cp_read(RADIO_SWIS_I2C_ADDR, registeraddr));   
}

static void WriteRegister(unsigned char registeraddr, unsigned char value) {
    e_i2cp_write(RADIO_SWIS_I2C_ADDR, registeraddr, value);
}





int e_radio_swis_present()
{
    unsigned char value;
    value = e_radio_swis_status();
    if (value == 0xFF) return 0;        // always 0xff when not plugged in
    return 1;
}


void e_radio_swis_init(
        unsigned char group, 
        unsigned int  address, 
        hw_att_state  hw_attenuator, 
        unsigned char tx_power
    ) 
{
    e_i2cp_init();
    e_i2cp_enable();
                
    while (e_radio_swis_present()==0);  // wait until module is online

    e_radio_swis_set_group(group);
    e_radio_swis_set_address(address);
    e_radio_swis_set_hw_atten(hw_attenuator);
    e_radio_swis_set_tx_pwr(tx_power);
    e_radio_swis_set_state(1);
}


void e_radio_swis_set_state(unsigned char mode)
{
    unsigned char old_value;    
    old_value = ReadRegister(CONFIG_REG_ADDR);
    if (mode != 0)
        WriteRegister(CONFIG_REG_ADDR, old_value |= RADIO_ENABLED_FLAG);
    else
        WriteRegister(CONFIG_REG_ADDR, old_value &= ~RADIO_ENABLED_FLAG);
}


void e_radio_swis_set_hw_atten(hw_att_state attenuatormode)
{
    unsigned char old_value;    
    old_value = ReadRegister(CONFIG_REG_ADDR);
    if (attenuatormode != 0)
        WriteRegister(CONFIG_REG_ADDR, old_value |= HARDWAREATT_SET_FLAG);
    else
        WriteRegister(CONFIG_REG_ADDR, old_value &= ~HARDWAREATT_SET_FLAG);
}


void e_radio_swis_set_tx_pwr(unsigned char attenuatorvalue)
{
    if (attenuatorvalue > 31) attenuatorvalue = 31;
    WriteRegister(SOFTATT_REG_ADDR, attenuatorvalue);
}


void e_radio_swis_set_group(unsigned char group)
{
    WriteRegister(OWNGROUP_REG_ADDR, group);
}


void e_radio_swis_set_address(unsigned int address)
{
    WriteRegister(OWNADDRL_REG_ADDR, address & 0xFF);
    address=address>>8;
    WriteRegister(OWNADDRH_REG_ADDR, address & 0xFF);
}


unsigned char e_radio_swis_get_hw_atten()
{
    if (ReadRegister(CONFIG_REG_ADDR) & HARDWAREATT_SET_FLAG) return 1;
    return 0;
}


unsigned char e_radio_swis_get_state()
{
    if (ReadRegister(CONFIG_REG_ADDR) & RADIO_ENABLED_FLAG) return 1;
    return 0;
}


unsigned char e_radio_swis_get_tx_pwr()
{
    return ReadRegister(SOFTATT_REG_ADDR);
}


unsigned char e_radio_swis_get_group()
{
    return ReadRegister(OWNGROUP_REG_ADDR);
}


unsigned int e_radio_swis_get_address()
{
    unsigned char lowbyte, highbyte;
    lowbyte  = ReadRegister(OWNADDRL_REG_ADDR);
    highbyte = ReadRegister(OWNADDRH_REG_ADDR);
    return lowbyte | ((int)highbyte<<8);
}


unsigned char e_radio_swis_status()
{
    return ReadRegister(STATUS_REG_ADDR);
}


int e_radio_swis_send(
        unsigned char   dst_group, 
        unsigned int    dst_address, 
        unsigned char*  packet, 
        int             packetsize
    )
{
    return e_radio_swis_send_msgtype(
        dst_group,dst_address,packet,packetsize,AM_MSGTYPE
    );
}


int e_radio_swis_send_msgtype(
        unsigned char   dst_group, 
        unsigned int    dst_address, 
        unsigned char*  packet, 
        int             packetsize,
        unsigned char   amtype
    )
{
    unsigned int i;

    // return error if packet is too long
    if (packetsize > RADIO_SWIS_MAXSIZE) return 0;

    // wait till module is ready to transmit
    while ((e_radio_swis_status() & TX_IDLE_FLAG) == 0);

    // build packet in buffer
    WriteRegister(SEND_BUFFER_START + AM_MSGTYPE_IN_PACKET_OFFSET, amtype);
    WriteRegister(SEND_BUFFER_START + GROUPDATA_IN_PACKET_OFFSET, dst_group);
    WriteRegister(SEND_BUFFER_START + ADDRLDATA_IN_PACKET_OFFSET, 
            dst_address & 0xFF);
    dst_address=dst_address>>8;
    WriteRegister(SEND_BUFFER_START + ADDRHDATA_IN_PACKET_OFFSET, 
            dst_address & 0xFF);
    for (i=0; i<packetsize; i++) {
        WriteRegister(FIRSTDATA_IN_PACKET_OFFSET+SEND_BUFFER_START+i,packet[i]);
    }

    // send packet
    WriteRegister(SEND_REG_ADDR,1);

    // TODO: maybe wait here?

    // flag TX error set -> return error
    if (e_radio_swis_status() & TX_SEND_ERROR) return 0;

    // otherwise return OK
    return 1;
}


int e_radio_swis_packet_ready(unsigned char* packet, int* packetsize)
{
    return e_radio_swis_packet_ready_msgtype(packet,packetsize,NULL);
}


int e_radio_swis_packet_ready_msgtype(unsigned char* packet, int* packetsize, unsigned char* amtype)
{
    unsigned int i;

    // return without reading NULL pointer means uC reset if write
    if ((packet == NULL) || (packetsize == NULL))
        return 0;

    if (e_radio_swis_status() & PACKET_READY_FLAG) // a packet is ready!
    {
        *packetsize = ReadRegister(REC_BUFFER_START); // size of rec'd buffer
        if (amtype != NULL) 
            *amtype = ReadRegister(REC_BUFFER_START + 
                    AM_MSGTYPE_IN_PACKET_OFFSET);
        if (*packetsize > RADIO_SWIS_MAXSIZE)
            return 0; // error in transmission. size is too big and impossible
        for (i = 0; i< *packetsize ; i++) {
            packet[i] = ReadRegister(REC_BUFFER_START + 
                    FIRSTDATA_IN_PACKET_OFFSET+i);
        }
        ReadRegister(REC_BUFFER_END); 
        return 1;
    }

    for(i=0; i<0x1000; i++); // wait (TODO: better way of waiting?)

    return 0; // no packet ready -> 0
}


