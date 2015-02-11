/**
  ******************************************************************************
  * @file    sn8200.h 
  * @author  Joe Todd
  * @version 
  * @date    
  * @brief   Header for sn8200.c
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SN8200_H
#define SN8200_H

#include <stdbool.h>
#include <stdint.h>

/* Command ID's --------------------------------------------------------------*/
#define CMD_ID_NACK			0x00
#define CMD_ID_WIFI         0x50
#define CMD_ID_SNIC         0x70
#define CMD_ID_ACK 			0x7F

/* Special characters --------------------------------------------------------*/
#define SOM_CHAR 			0x02
#define EOM_CHAR 			0x04
#define ESC_CHAR 			0x10
#define ACK_CMD				0x7F
#define NAK_CMD 			0x00

/* RFSCID's ------------------------------------------------------------------*/
#define WIFI_ON_REQ 				0x00
#define WIFI_OFF_REQ				0x01
#define WIFI_JOIN_REQ				0x02
#define WIFI_DISCONNECT_REQ 		0x03
#define WIFI_GET_STATUS_REQ 		0x04
#define WIFI_SCAN_REQ				0x05
#define WIFI_GET_STA_RSSI_REQ 		0x06
#define WIFI_AP_CTRL_REQ 			0x07

#define WIFI_NETWORK_STATUS_IND 	0x10
#define WIFI_SCAN_RESULT_IND		0x11
#define WIFI_RSSI_IND				0x12

#define WIFI_ON_RSP 				0x80
#define WIFI_OFF_RSP				0x81
#define WIFI_JOIN_RSP				0x82
#define WIFI_DISCONNECT_RSP 		0x83
#define WIFI_GET_STATUS_RSP 		0x84
#define WIFI_SCAN_RSP				0x85
#define WIFI_GET_STA_RSSI_RSP 		0x86
#define WIFI_AP_CTRL_RSP			0x87

#define	WIFI_SUCCESS				0x00
#define WIFI_FAIL 					0x01
#define WIFI_NETWORK_UP 			0x10
#define WIFI_NETWORK_DOWN			0x11

#define SNIC_INIT_REQ 				0x00
#define SNIC_CLEANUP_REQ			0x01
#define SNIC_SEND_FROM_SOCKET_REQ 	0x02
#define SNIC_CLOSE_SOCKET_REQ 		0x03
#define SNIC_SOCKET_PART_CLOSE_REQ  0x04
#define SNIC_GETSOCKOPT_REQ 		0x05
#define SNIC_SETSOCKOPT_REQ			0x06
#define SNIC_SOCKET_GETNAME_REQ		0x07
#define SNIC_SEND_ARP_REQ			0x08
#define SNIC_GET_DHCP_INFO_REQ		0x09
#define SNIC_RESOLVE_NAME_REQ		0x0A
#define SNIC_IP_CONFIG_REQ			0x0B

#define SNIC_TCP_CREATE_SOCKET_REQ  0x10
#define SNIC_TCP_CREATE_CONN_REQ	0x11
#define SNIC_TCP_CONN_TO_SERV_REQ	0x12
#define SNIC_UDP_CREATE_SOCKET_REQ	0x13
#define SNIC_UDP_START_RECV_REQ		0x14
#define SNIC_UDP_SIMPLE_SEND_REQ	0x15
#define SNIC_UDP_SEND_FROM_SOCK_REQ 0x16

#define SNIC_HTTP_REQ 				0x17
#define SNIC_HTTP_MORE_REQ			0x18
#define SNIC_HTTPS_REQ				0x19

#define SNIC_TCP_ADV_TLS_SOCK_REQ 	0x1A
#define SNIC_TCP_SIMPLE_TLS_SOCK_REQ 0x1B

#define SNIC_TCP_CONN_STATUS_IND 	0x20
#define SNIC_TCP_CLIENT_SOCKET_IND	0x21
#define SNIC_CONNECTION_RECV_IND	0x22
#define SNIC_UDP_RECV_IND			0x23
#define SNIC_ARP_REPLY_IND			0x24
#define SNIC_HTTP_RSP_IND			0x25

#define SNIC_SEND_RSP  				0x82
#define SNIC_CLOSE_SOCKET_RSP		0x83
#define SNIC_GET_DHCP_INFO_RSP 		0x89
#define SNIC_IP_CONFIG_RSP 			0x8B
#define SNIC_TCP_CREATE_SOCKET_RSP  0x90
#define SNIC_TCP_CREATE_CONN_RSP	0x91
#define SNIC_TCP_CONN_TO_SERV_RSP	0x92

#define SNIC_UDP_CREATE_SOCK_RSP  	0x93
#define SNIC_UDP_SEND_FROM_SOCK_RSP 0x96
#define SNIC_HTTP_RSP               0x97
		
#define SNIC_TCP_ADV_TLS_SOCK_RSP	0x9A
#define SNIC_TCP_SIMPLE_TLS_SOCK_RSP 0x9B

#define SNIC_SUCCESS				0x00
#define SNIC_FAIL  					0x01
#define SNIC_INIT_FAIL				0x02
#define SNIC_CLEANUP_FAIL			0x03
#define SNIC_GETADDRINFO_FAIL		0x04
#define SNIC_CREATE_SOCKET_FAIL		0x05
#define SNIC_BIND_SOCKET_FAIL		0x06
#define SNIC_LISTEN_SOCKET_FAIL		0x07
#define SNIC_ACCEPT_SOCKET_FAIL		0x08
#define SNIC_PARTIAL_CLOSE_FAIL 	0x09
#define SNIC_CONN_PARTIALLY_CLOSED 	0x0A
#define SNIC_CONNECTION_CLOSED		0x0B
#define SNIC_CLOSE_SOCKET_FAIL 		0x0C
#define SNIC_PACKET_TOO_LARGE		0x0D
#define SNIC_SEND_FAIL 				0x0E
#define SNIC_CONNECT_TO_SERVER_FAIL 0x0F
#define SNIC_NOT_ENOUGH_MEMORY 		0x10
#define SNIC_TIMEOUT				0x11
#define SNIC_CONNECTION_UP			0x12
#define SNIC_GETSOCKOPT_FAIL		0x13
#define SNIC_SETSOCKOPT_FAIL		0x14
#define SNIC_INVALID_ARGUMENT		0x15
#define SNIC_SEND_ARP_FAIL			0x16
#define SNIC_INVALID_SOCKET			0x17
#define SNIC_CONN_TO_SERV_PENDING	0x18
#define SNIC_SOCKET_NOT_BOUND		0x19
#define SNIC_SOCKET_NOT_CONNECTED	0x1A

#define MAX_PAYLOAD_LEN 	8000

#define NUM_RX_BUF 10

/* External typedefs ----------------------------------------------------------*/
typedef struct {
    bool available;
    unsigned short payload_len;
    unsigned char ack_reqd;
    unsigned char cmd_id;
    unsigned char rx_payload[MAX_PAYLOAD_LEN];
    unsigned char chksum;
    unsigned char ack_ok;
} rx_info_t;

typedef enum {
    IDLE,
    SOM_RECD,
    LEN_RECD,
    ACK_SEQ_RECD,
    CMD_RECD,
    PAYLOAD_RX,
    PAYLOAD_RX_ESC,
    CHKSUM_RECD,
    EOM_RECD,
    WAIT_FOR_ACK_NAK,
} serial_rx_state_t;

typedef enum {
    MODE_WIFI_OFF,
    MODE_NO_NETWORK,
    MODE_STA_JOINED,
    MODE_AP_STARTED,
    MODE_SNIC_INIT_NOT_DONE,
    MODE_SNIC_INIT_DONE,
    /* Non-mode special values */
    MODE_LIST_END,
    MODE_ANY,
} serial_wifi_mode_t;

/* Function Definitions -------------------------------------------------------*/
extern void sn8200_send_frame(unsigned char cmd_id, uint32_t payload_len, 
                unsigned char *payload, unsigned char ack_required);
extern void sn8200_handle_rsp(void);

#endif