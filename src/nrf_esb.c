#include "nrf_esb.h"

/* Use the logging module registered in main.c */
LOG_MODULE_DECLARE(esb_prx);

K_MSGQ_DEFINE(tx_queue, sizeof(struct esb_payload), TX_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(rx_queue, sizeof(struct esb_payload), RX_QUEUE_SIZE, 4);

int esb_initialize(void)
{
    uint8_t base0[4] = {0xC2, 0xC2, 0xC2, 0x08}; //4 bytes: Base address
    uint8_t base1[4] = {0xE6, 0xE6, 0xE6, 0xEA};
    uint8_t prefix[8] = {0xE7, 0, 0, 0, 0, 0, 0, 0}; //1 byte prefiz

    struct esb_config cfg = ESB_DEFAULT_CONFIG;
    cfg.protocol = ESB_PROTOCOL_ESB_DPL;
    cfg.bitrate = ESB_BITRATE_2MBPS;
    cfg.mode = ESB_MODE_PRX;
    cfg.event_handler = event_handler;
    cfg.selective_auto_ack = false;

    esb_init(&cfg);
    esb_set_rf_channel(80);
    esb_set_base_address_0(base0);
    esb_set_base_address_1(base1);
    esb_set_prefixes(prefix, ARRAY_SIZE(prefix));
    return 0;
}

void esb_send_async(const struct esb_payload *src)
{
    struct esb_payload tx = {0};
    memcpy(&tx, src, sizeof(struct esb_payload));
    if (k_msgq_put(&tx_queue, &tx, K_NO_WAIT) != 0) {
        LOG_WRN("TX queue full, drop");
    }
}

void esb_send_async_with_uart(const struct esb_payload *src)
{
    esb_send_async(src);
    // Also forward to UART
    print_uart_payload(src->data, src->length);
}

/* ========================== EVENT HANDLER ========================== */
struct esb_payload rx_payload;

void event_handler(struct esb_evt const *event)
{
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        // LOG_DBG("TX SUCCESS");
        leds_update(0x02);  // Green LED (bit 1)
        break;
    case ESB_EVENT_TX_FAILED:
        // LOG_WRN("TX FAILED");
        leds_update(0x01);  // Red LED (bit 0)
        break;
    case ESB_EVENT_RX_RECEIVED:
        leds_update(0x04);  // Blue LED (bit 2)
        if (esb_read_rx_payload(&rx_payload) == 0)
            k_msgq_put(&rx_queue, &rx_payload, K_NO_WAIT);
        break;
    }
}