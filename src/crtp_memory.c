#include "crtp_memory.h"

/* Use the logging module registered in main.c */
LOG_MODULE_DECLARE(esb_prx);

/* MEMORY */
bool handle_memory_crtp(uint8_t port, uint8_t chan, const uint8_t *pl,
                               uint8_t plen, uint8_t hdr, uint8_t pipe)
{
    if (port == 4 && chan == 0 && plen >= 1 && pl[0] == 0x01) {
        struct esb_payload tx = {0};
        tx.pipe = pipe;
        tx.length = 4;
        tx.data[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
        tx.data[1] = 0x01;
        tx.data[2] = 0x00;
        tx.data[3] = 0x00;
        esb_send_async(&tx);
        LOG_INF("MEMORY reply");
        my_downlink_ctr ^= 1;
        return true;
    }
    return false;
}