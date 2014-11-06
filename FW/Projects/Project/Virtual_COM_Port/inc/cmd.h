
#ifndef _CMD_H_
#define _CMD_H_

#include <stdint.h>


#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    3

/* Commands */
#define VERSION         0x01
#define EEPROM_READ     0x02
#define EEPROM_WRITE    0x03
#define BLUENRG_RESET   0x04
#define HW_BOOTLOADER   0x05


void parse_cmd(uint8_t *hci_buffer, uint16_t hci_pckt_len);


#endif /* _CMD_H_ */