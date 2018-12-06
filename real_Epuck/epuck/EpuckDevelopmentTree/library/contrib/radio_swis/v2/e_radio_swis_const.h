// e_radio_swis_const.h

#ifndef _RADIO_SWIS_CONST_H
#define _RADIO_SWIS_CONST_H

// I2C Slave address
typedef enum _i2c_address {
 	EPUCK_I2C_ADDR = 0x10,
	TURRET_I2C_ADDR = 0x20,
} i2c_address;

typedef enum _operation_mode {
	RADIO_SWIS_SLAVE_MASTER,
 	RADIO_SWIS_MULTIMASTER,
} i2c_operation_mode;

// Address of the registers
enum {
	STATUS_REG_ADDR				=	0x00,  // status like PACKET_LOST_FLAG,TX_IDLE,PACKET_READY
	CONFIG_REG_ADDR				=	0x01,  // config like ..... hardware att. sleep...
	SEND_REG_ADDR				=	0x02,  // <> 0 -> send sendbuffer
	SOFTATT_REG_ADDR			=	0x03,
	OWNGROUP_REG_ADDR			=	0x04,
	OWNADDR_REG_ADDR			=	0x05,
	DESTGROUP_REG_ADDR			=	0x06,
	DESTADDR_REG_ADDR			=	0x07,
	SEND_BUFFER_ADDR			=	0x08,
	REC_BUFFER_ADDR				=	0x09,
	PAYLOAD_LENGTH_REG_ADDR		=	0x0A,
	MAX_PAYLOAD_LENGTH_REG_ADDR	=	0x0B,
	VERSION_REG_ADDR			=	0x0C,
};

// Size of the registers
enum {
	STATUS_REG_SIZE				=	0x01,
	CONFIG_REG_SIZE				=	0x01,
	SEND_REG_SIZE				=	0x01,
	SOFTATT_REG_SIZE			=	0x01,
	OWNGROUP_REG_SIZE			=	0x01,
	OWNADDR_REG_SIZE			=	0x02,
	DESTGROUP_REG_SIZE			=	0x01,
	DESTADDR_REG_SIZE			=	0x02,
	PAYLOAD_LENGTH_REG_SIZE		=	0x01,
	MAX_PAYLOAD_LENGTH_REG_SIZE	=	0x01,
	VERSION_REG_SIZE			=	0x01,
};

// status reg
enum {
	PACKET_READY_FLAG		=	0x01, // bit 1; A new packet has arrived, please read it
	TX_IDLE_FLAG			=	0x02, // bit 2; Transmission is idle, ready to transmit a new packet
	PACKET_LOST_FLAG		=	0x04, // bit 3; A packet has arrived and overwrote the old (unread) one
	TX_SEND_ERROR			=	0x08, // bit 4; Error during transmission
	SYNC_LENGTH			=	0x10, // bit 5; The payload's length has changed, synchronization ongoing. Don't send or receive any packet to avoid length incompatibilities!
};

//config reg
enum {
	HARDWAREATT_SET_FLAG	=	0x01, //bit 1
	I2C_MODE_MULTIMASTER	=	0x02, //bit 2
	RADIO_ENABLED_FLAG		=	0x80, //bit 8
};

// send reg
enum {
	REQUEST_TO_SEND_FLAG	=	0x01,
};


#endif // _RADIO_SWIS_CONST_H

