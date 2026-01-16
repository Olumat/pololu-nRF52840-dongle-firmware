#include "crtp.h"

/* Use the logging module registered in main.c */
LOG_MODULE_DECLARE(esb_prx);

uint8_t my_downlink_ctr = 0;

/* ============================ CRTP HANDLERS ============================ */
/* SCAN */
bool handle_scan_packet(const uint8_t *data, uint8_t len, uint8_t pipe)
{
    if (len == 3 && data[0] == 0xFF && data[1] == 0x05 && data[2] == 0x01) {
        struct esb_payload tx = {0};
        tx.pipe = pipe;
        tx.length = 3;
        memcpy(tx.data, data, 3);
        esb_send_async(&tx);
        LOG_INF("SCAN packet reply");
        return true;
    }
    return false;
}

/* KEEPALIVE */
bool handle_keepalive(const uint8_t *data, uint8_t len, uint8_t pipe)
{
    if (len == 1 && (data[0] & 0xF3) == 0xF3) {
        struct esb_payload tx = {0};
        tx.pipe = pipe;
        tx.length = 2;
        tx.data[0] = (0xAA & 0xF3) | (data[0] & 0x08) | (my_downlink_ctr << 2);
        tx.data[1] = 0x55;
        esb_send_async(&tx);
        my_downlink_ctr ^= 1;
        return true;
    }
    return false;
}


/* ========================== TX WORKER ========================== */
void tx_worker(void)
{
    struct esb_payload tx;

    while (1) {
        if (k_msgq_get(&tx_queue, &tx, K_FOREVER) == 0) {
            int ret;
            do {
                ret = esb_write_payload(&tx);
                if (ret != 0) {
                    k_msleep(1);
                }
            } while (ret != 0);
        }
    }
}


/* ========================== RX THREAD ========================== */
void rx_worker(void)
{
    struct esb_payload rx;
    while (1) {
        if (k_msgq_get(&rx_queue, &rx, K_FOREVER) == 0) {
            handle_received_packet(&rx);
        }
    }
}

void handle_received_packet(struct esb_payload *rx)
{
    uint8_t *data = rx->data;
    uint8_t len = rx->length;
    uint8_t pipe = rx->pipe;

    if (handle_scan_packet(data, len, pipe)) return;
    if (handle_keepalive(data, len, pipe)) return;

    uint8_t hdr  = data[0] & 0xF3;
	uint8_t chan = (hdr & 0x0F);
	uint8_t port = ((hdr >> 4) & 0x0F);
	const uint8_t *pl = &data[1];
	uint8_t plen = len - 1;

    bool handled = false;

    handled |= handle_platform_crtp(port, chan, pl, plen, hdr, pipe);
    handled |= handle_param_crtp(port, chan, pl, plen, hdr, pipe);
    handled |= handle_memory_crtp(port, chan, pl, plen, hdr, pipe);
    handled |= handle_log_crtp(port, chan, pl, plen, hdr, pipe);

    if (!handled) {
        print_uart_payload(rx->data, rx->length);
        LOG_INF("Forward non-CRTP packet to UART, len=%d %d %d", rx->data[0], rx->data[1], rx->data[2]);
    }
}

