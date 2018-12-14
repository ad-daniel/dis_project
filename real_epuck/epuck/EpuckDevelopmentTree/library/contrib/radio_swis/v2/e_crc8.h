#ifndef _CRC_H
#define _CRC_H

unsigned char e_crc8_recursive(unsigned char old_crc, unsigned char new_value);

unsigned char e_crc8_compute(unsigned char* buffer, unsigned char length);

#endif // _CRC_H
