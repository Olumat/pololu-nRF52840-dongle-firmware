#include "crtp_platform.h"

/* Use the logging module registered in main.c */
LOG_MODULE_DECLARE(esb_prx);

/* PLATFORM */
bool handle_platform_crtp(uint8_t port, uint8_t chan, const uint8_t *pl,
                                 uint8_t plen, uint8_t hdr, uint8_t pipe)
{
    if (port == 13 && chan == 1 && plen >= 1 && pl[0] == 0x00) {
        struct esb_payload tx = {0};
        tx.pipe = pipe;
        tx.length = 3;
        tx.data[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
        tx.data[1] = 0x00;
        tx.data[2] = 0x0A; // version
        esb_send_async(&tx);
        LOG_INF("PLATFORM reply sent");
        my_downlink_ctr ^= 1;
        return true;
    }
    return false;
}