/**
 ******************************************************************************
 * @file    sn8200.c
 * @author  Joe Todd
 * @version
 * @date    June 2014
 * @brief   WiFi Automation
 *
  ******************************************************************************/


/* Includes -------------------------------------------------------------------*/
#include "sn8200.h"
#include "uart.h"
#include "main.h"

/* Private variables ----------------------------------------------------------*/
static rx_info_t rx_frame[NUM_RX_BUF];
static serial_rx_state_t serial_rx_state;
static int32_t curr_buf_idx = 0;
static int32_t curr_read_idx = 0;
static int32_t rx_frm_payload_index;
static int32_t rx_frm_data_index;
static unsigned char rx_frm_chksum;

/* Function Declarations ------------------------------------------------------*/
static unsigned char sn8200_tx_esc_payload(uint32_t payload_len, unsigned char *payload);
static uint32_t sn8200_calc_payload_len(uint32_t payload_len, unsigned char *payload);
static int8_t sn8200_process_rsp(unsigned char rx_ch);
static bool sn8200_rx_frame_empty(void);
static int32_t sn8200_handle_rx_frame(uint8_t commandId, uint16_t paramLength, uint8_t *params);

/* Function Definitions -------------------------------------------------------*/
static uint32_t
sn8200_calc_payload_len(uint32_t payload_len, unsigned char *payload)
{
    uint32_t i;
    uint32_t len = 0;
    unsigned char c;

    for (i = 0; i < payload_len; ++i) {
        c = *payload++;
        if (c == SOM_CHAR || c == EOM_CHAR || c == ESC_CHAR) {
            len += 2;
        } else {
            len ++;
        }
    }
    return len;
}

extern void 
sn8200_send_frame(unsigned char cmd_id, uint32_t payload_len, 
                unsigned char *payload, unsigned char ack_required)
{
    uint32_t len;
    unsigned char data;
    unsigned char cksum = 0;
    unsigned char hdr = 0x80;

    len = sn8200_calc_payload_len(payload_len, payload);

    data = SOM_CHAR;
    uart_send_data(&data, 1);

    /* 
     * Send payload length
     */
    data = 0x80 | len;
    uart_send_data(&data, 1);
    cksum += data;

    hdr |= (ack_required << 6);

    if (len > 0x7f) {
        hdr |= (len >> 7);
    }

    data = 0x80 | hdr;
    cksum += data;
    uart_send_data(&data, 1);

    /* 
     * Send Command ID
     */
    data = 0x80 | cmd_id;
    uart_send_data(&data, 1);
    cksum += data;

    cksum += sn8200_tx_esc_payload(payload_len, payload);
    cksum |= 0x80;
    uart_send_data(&cksum, 1);

    data = EOM_CHAR;
    uart_send_data(&data, 1);
}

static unsigned char 
sn8200_tx_esc_payload(uint32_t payload_len, unsigned char *payload)
{
    uint32_t i;
    unsigned char chksum = 0;
    unsigned char tx;
    unsigned char data;

    for (i = 0; i < payload_len; ++i) {
        tx = *payload++;
        if (tx != SOM_CHAR && tx != EOM_CHAR && tx != ESC_CHAR) {
            uart_send_data(&tx, 1);
            chksum += tx;
        } else {
            data = ESC_CHAR;
            uart_send_data(&data, 1);
            data = (0x80 | tx);
            uart_send_data(&data, 1);
            chksum += ESC_CHAR;
            chksum += (0x80 | tx);
        }
    }
    return chksum;
}

extern void 
sn8200_handle_rsp(void)
{
    uint8_t data;

    while (!uart_buffer_empty()) {
        data = uart_read_byte();
        sn8200_process_rsp(data);
    }

    while (!sn8200_rx_frame_empty()) {
        if (rx_frame[curr_read_idx].available) {
            if (rx_frame[curr_read_idx].ack_ok) {
                sn8200_handle_rx_frame(rx_frame[curr_read_idx].cmd_id, rx_frame[curr_read_idx].payload_len, 
                                    rx_frame[curr_read_idx].rx_payload);
            }
            rx_frame[curr_read_idx].available = false;
        }
        curr_read_idx = (curr_read_idx + 1) % NUM_RX_BUF;
    }
}

static bool 
sn8200_rx_frame_empty(void)
{
    return (curr_read_idx == curr_buf_idx);
}

static int8_t 
sn8200_process_rsp(unsigned char rx_ch)
{
    int8_t ch = -1;

    switch (serial_rx_state) {
        case IDLE:
            if (rx_ch == SOM_CHAR) {
rx_process_char_RX_SOM:
                serial_rx_state = SOM_RECD;
                rx_frm_payload_index = 0;
                rx_frm_chksum = 0;
                rx_frm_data_index = 0;
            } 
            else {
                ch = rx_ch;
            }
            break;

        case SOM_RECD:
            if ((rx_ch & 0x80) == 0) {
                goto rx_process_char_error;
            }
            serial_rx_state = LEN_RECD;
            rx_frame[curr_buf_idx].payload_len = rx_ch & 0x7F;
            rx_frm_chksum += rx_ch;
            break;

        case LEN_RECD:
            if ((rx_ch & 0x80) == 0) {
                goto rx_process_char_error;
            }
            serial_rx_state = ACK_SEQ_RECD;
            rx_frame[curr_buf_idx].payload_len |= (rx_ch & 0x3f) << 7;
            rx_frame[curr_buf_idx].ack_reqd = (rx_ch >> 6) & 0x01;
            rx_frm_chksum += rx_ch;
            break;

        case ACK_SEQ_RECD:
            if ((rx_ch & 0x80) == 0) {
                goto rx_process_char_error;
            }
            serial_rx_state = CMD_RECD;
            rx_frame[curr_buf_idx].cmd_id = rx_ch & 0x7F;
            rx_frm_chksum += rx_ch;
            break;

        case CMD_RECD:
            serial_rx_state = PAYLOAD_RX;

        case PAYLOAD_RX:
            rx_frm_payload_index++;
            if (rx_ch == SOM_CHAR) {
                goto rx_process_char_error;
            } 
            else if (rx_ch == EOM_CHAR) {
                serial_rx_state = IDLE;
            } 
            else if (rx_ch == ESC_CHAR) {
                serial_rx_state = PAYLOAD_RX_ESC;
                rx_frm_chksum += rx_ch;
            } 
            else if (rx_frm_payload_index > rx_frame[curr_buf_idx].payload_len) {
                serial_rx_state = CHKSUM_RECD;
                rx_frame[curr_buf_idx].chksum = rx_ch;
            } 
            else {
                rx_frm_chksum += rx_ch;
                rx_frame[curr_buf_idx].rx_payload[rx_frm_data_index++] = rx_ch;
            }
            break;

        case PAYLOAD_RX_ESC:
            rx_frm_payload_index++;
            if (rx_ch > 127) {
                rx_frm_chksum += rx_ch;
                rx_frame[curr_buf_idx].rx_payload[rx_frm_data_index++] = rx_ch & 0x7F;
                serial_rx_state = PAYLOAD_RX;
            } 
            else {
                goto rx_process_char_error;
            }
            break;

        case CHKSUM_RECD:
            if (rx_ch != EOM_CHAR) {
                goto rx_process_char_error;
            } 
            else if ((rx_frame[curr_buf_idx].chksum & 0x7F) == (rx_frm_chksum & 0x07F)) { //checksum match
                rx_frame[curr_buf_idx].payload_len = rx_frm_data_index;
                rx_frame[curr_buf_idx].ack_ok = 1;
                rx_frame[curr_buf_idx].available = true;
                curr_buf_idx = (curr_buf_idx + 1) % NUM_RX_BUF;
                serial_rx_state = IDLE;
            } 
            else {
                /* Bad frame */
            }
            serial_rx_state = IDLE;
            break;

        default:
rx_process_char_error:
            serial_rx_state = IDLE;
            if (rx_ch == SOM_CHAR) {
                goto rx_process_char_RX_SOM;
            }
        break;
    }

    if (ch != -1) {
        return ch;
    }
    else {
        return -1;
    }
}

static int32_t 
sn8200_handle_rx_frame(uint8_t commandId, uint16_t paramLength, uint8_t *params)
{
    switch (commandId) {
    case CMD_ID_WIFI:
#ifdef WIFI_EN
        handleRxWiFi(params, paramLength);
#endif
        break;
    case CMD_ID_SNIC:
#ifdef WIFI_EN
        handleRxSNIC(params, paramLength);
#endif
        break;
    default:
        break;
    }
    return 0;
}