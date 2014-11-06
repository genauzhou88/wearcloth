#ifndef HCI_H_
#define HCI_H_

#include <stdint.h>


/* HCI Packet types */
#define HCI_COMMAND_PKT		0x01
#define HCI_ACLDATA_PKT		0x02
#define HCI_SCODATA_PKT		0x03
#define HCI_EVENT_PKT		0x04
#define HCI_VENDOR_PKT		0xff

#define HCI_TYPE_OFFSET                 0
#define HCI_VENDOR_CMDCODE_OFFSET       1
#define HCI_VENDOR_LEN_OFFSET           2
#define HCI_VENDOR_PARAM_OFFSET         4



typedef enum {
	WAITING_TYPE,
    WAITING_OPCODE_1,
    WAITING_OPCODE_2,
	WAITING_CMDCODE,
    WAITING_CMD_LEN1,
    WAITING_CMD_LEN2,
	WAITING_PARAM_LEN,
    WAITING_HANDLE,
	WAITING_HANDLE_FLAG,
    WAITING_DATA_LEN1,
	WAITING_DATA_LEN2,
	WAITING_PAYLOAD
}hci_state;


#ifdef __ICCARM__
#define PACKED
#endif

#ifdef __GNUC__
 #define __packed
#define PACKED __attribute__((packed))
#endif

typedef __packed struct _hci_uart_pckt{
	uint8_t	type;
	uint8_t	data[0];
} PACKED hci_uart_pckt;
#define HCI_HDR_SIZE 1

typedef __packed struct _hci_acl_hdr{
	uint16_t	handle;		/* Handle & Flags(PB, BC) */
	uint16_t	dlen;
} PACKED hci_acl_hdr;
#define HCI_ACL_HDR_SIZE 	4

/* HCI library functions. */
void hci_input(uint8_t *buff, uint16_t len);

#endif /* HCI_H_ */
