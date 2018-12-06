// e_I2C_multimaster_crc.h

#ifndef _I2C_MULTIMASTER_CRC_H
#define _I2C_MULTIMASTER_CRC_H

void e_I2C_multimaster_crc_init(void (*fctCallback)(void));
void e_I2C_multimaster_crc_disable();

void e_I2C_multimaster_crc_write_bytes(char device_address, char reg, char message[], char nb_bytes);
void e_I2C_multimaster_crc_read_bytes(char device_address, char reg, char buffer[], char nb_bytes);

unsigned int e_I2C_multimaster_crc_get_failed(void);
void e_I2C_multimaster_crc_reset_failed(void);

unsigned int e_I2C_multimaster_crc_get_attempts(void);
void e_I2C_multimaster_crc_reset_attempts(void);

#endif // _I2C_MULTIMASTER_CRC_H
