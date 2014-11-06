
#include <hci.h>
#include "hw_config.h"
#include <cmd.h>
#include "SDK_EVAL_Spi_Driver.h"
#include "gp_timer.h"

#define HCI_PACKET_SIZE 512

static uint8_t hci_buffer[HCI_PACKET_SIZE];
static volatile uint16_t hci_pckt_len = 0;

void packet_received(void);

void hci_input(uint8_t *buff, uint16_t len)
{
	/*TODO Add a timer that reset state machine if bytes are not received for too long time. */
    
	static hci_state state = WAITING_TYPE;
    
	static uint16_t collected_payload_len = 0;
	static uint16_t payload_len;
    hci_acl_hdr *acl_hdr;
    uint8_t byte;
    uint8_t i = 0;
        
    if(state == WAITING_TYPE)
            hci_pckt_len = 0;
    
    while(hci_pckt_len < HCI_PACKET_SIZE && i++ < len){
        
        byte = *buff++;
        
        hci_buffer[hci_pckt_len++] = byte;        
        
        if(state == WAITING_TYPE){
            /* Only command or vendor packets are accepted. */
            if(byte == HCI_COMMAND_PKT){
                state = WAITING_OPCODE_1;
            }
            else if(byte == HCI_ACLDATA_PKT){
                state = WAITING_HANDLE;
            }
            else if(byte == HCI_VENDOR_PKT){
                state = WAITING_CMDCODE;
            }
            else{
                /* Incorrect type. Reset state machine. */
                state = WAITING_TYPE;
            }
        }
        else if(state == WAITING_OPCODE_1){
            state = WAITING_OPCODE_2;
        }
        else if(state == WAITING_OPCODE_2){
            state = WAITING_PARAM_LEN;
        }
        else if(state == WAITING_CMDCODE){
            state = WAITING_CMD_LEN1;
        }
        else if(state == WAITING_CMD_LEN1){
            payload_len = byte;
            state = WAITING_CMD_LEN2;
        }
        else if(state == WAITING_CMD_LEN2){
            payload_len += byte << 8;
            collected_payload_len = 0;
            if(payload_len == 0){
                state = WAITING_TYPE;
                packet_received();                
            }
            else {
                state = WAITING_PAYLOAD;
            }
        }
        else if(state == WAITING_PARAM_LEN){
            payload_len = byte;
            collected_payload_len = 0;
            if(payload_len == 0){
                state = WAITING_TYPE;
                packet_received();
            }
            else {
                state = WAITING_PAYLOAD;
            }            
        }
        
        /*** State transitions for ACL packets ***/
        else if(state == WAITING_HANDLE){
            state = WAITING_HANDLE_FLAG;
        }
        else if(state == WAITING_HANDLE_FLAG){
            state = WAITING_DATA_LEN1;
        }
        else if(state == WAITING_DATA_LEN1){
            state = WAITING_DATA_LEN2;
        }
        else if(state == WAITING_DATA_LEN2){
            acl_hdr = (void *)&hci_buffer[HCI_HDR_SIZE];
            payload_len = acl_hdr->dlen;
            collected_payload_len = 0;
            if(payload_len == 0){
                state = WAITING_TYPE;
                packet_received();
            }
            else{
                state = WAITING_PAYLOAD;
            }
        }
        /*****************************************/
        
        else if(state == WAITING_PAYLOAD){
            collected_payload_len += 1;
            if(collected_payload_len >= payload_len){
                /* Reset state machine. */
                state = WAITING_TYPE;                
                packet_received();
            }
        }
        
    }
}

void packet_received(void)
{
    struct timer t;
  
    if(hci_buffer[HCI_TYPE_OFFSET] == HCI_VENDOR_PKT){
        parse_cmd(hci_buffer, hci_pckt_len);
    }
    else {
      
        Timer_Set(&t, CLOCK_SECOND/10);
      
        while(1){
            if(BlueNRG_SPI_Write(hci_buffer, NULL, hci_pckt_len,0)==0) break;
            if(Timer_Expired(&t)){
                break;
            }
        }
    }    
}

