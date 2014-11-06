
#include <cmd.h>
#include <hci.h>
#include <hw_config.h>
#include "SDK_EVAL_Config.h"

#define MAX_RESP_SIZE 255

#define RESP_VENDOR_CODE_OFFSET     1
#define RESP_LEN_OFFSET_LSB         2
#define RESP_LEN_OFFSET_MSB         3
#define RESP_CMDCODE_OFFSET         4
#define RESP_STATUS_OFFSET          5
#define RESP_PARAM_OFFSET           6

/* Types of vendor codes */
#define ERROR               0
/* Error codes */
#define UNKNOWN_COMMAND	    0x01
#define INVALID_PARAMETERS	0x12

#define RESPONSE            1
/* end of vendor codes */


/* Command parser
 * bytes
 * 1         Type of packet (FF for special command)
 * 1         cmdcode
 * 2         cmd length (length of arguments)
 * variable  payload
 */

uint8_t response[MAX_RESP_SIZE];

void parse_cmd(uint8_t *hci_buffer, uint16_t hci_pckt_len)
{
    uint16_t len = 0;
    response[0] = HCI_VENDOR_PKT;
    response[RESP_VENDOR_CODE_OFFSET] = RESPONSE;
    response[RESP_CMDCODE_OFFSET] = hci_buffer[HCI_VENDOR_CMDCODE_OFFSET];
    response[RESP_STATUS_OFFSET] = 0;
    
    switch(hci_buffer[HCI_VENDOR_CMDCODE_OFFSET])
    {
    case VERSION:
        response[RESP_PARAM_OFFSET] = FW_VERSION_MAJOR;
        response[RESP_PARAM_OFFSET+1] = FW_VERSION_MINOR;
        len = 2;
        break;
    case EEPROM_READ:
        /* 2 bytes address
         * 1 byte num bytes
         */
        {
            uint16_t address = UNPACK_2_BYTE_PARAMETER(hci_buffer+HCI_VENDOR_PARAM_OFFSET);
            uint8_t bytes = hci_buffer[HCI_VENDOR_PARAM_OFFSET+2];
            if(bytes > MAX_RESP_SIZE - RESP_PARAM_OFFSET){
                response[RESP_STATUS_OFFSET] = INVALID_PARAMETERS;
                break;
            }            
            EepromRead(address, bytes, &response[RESP_PARAM_OFFSET]);            
            len = bytes;
        }
        break;
        
    case EEPROM_WRITE:
        /* 2 bytes address
         * 1 byte num bytes
         */
        {
            uint16_t address = UNPACK_2_BYTE_PARAMETER(hci_buffer+HCI_VENDOR_PARAM_OFFSET);
            uint8_t bytes = hci_buffer[HCI_VENDOR_PARAM_OFFSET+2];
            
            EepromWrite(address, bytes, &hci_buffer[HCI_VENDOR_PARAM_OFFSET+3]);
        }
        break;
    case BLUENRG_RESET:
        /* No parameters.
         */
        {            
            BlueNRG_RST();
        }
        break;
    case HW_BOOTLOADER:
        /* No parameters.
         */
        {            
            BlueNRG_HW_Bootloader();
        }
        break;
        
    default:
        response[RESP_STATUS_OFFSET] = UNKNOWN_COMMAND;
    }
    
    len += 2; // Status and Command code
    PACK_2_BYTE_PARAMETER(response+RESP_LEN_OFFSET_LSB,len);
    len += RESP_CMDCODE_OFFSET;     
    
    for(int i = 0; i < len; i++)
      SdkEval_IO_Send_Data(response[i]);
}


