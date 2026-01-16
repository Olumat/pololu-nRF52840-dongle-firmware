/*
 * Async ESB PRX with CRTP Handshake + Log Timer + param TOC/read/write
 * Author: Jiaming Li/Charlotte Stentzler
 * (Zephyr + nRF52840, Async Queue Version)
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <nrf.h>
#include <esb.h>

#include "uart.h"
#include "led.h"
#include "robot_state.h"
#include "crtp.h"
#include "nrf_esb.h"
#include "crtp_param.h"
#include "crtp_log.h"


LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);


K_THREAD_DEFINE(rx_thread_id, 2048, rx_worker, NULL, NULL, NULL, 3, 0, 0);

K_THREAD_DEFINE(tx_thread_id, 1024, tx_worker, NULL, NULL, NULL, 5, 0, 0);

K_THREAD_DEFINE(log_thread_id, 1024, log_task, NULL, NULL, NULL, 6, 0, 0);

K_THREAD_DEFINE(decode_thread_id, 1024, decode_worker, NULL, NULL, NULL, 7, 0, 0);

/* ============================ SYSTEM INIT ============================ */
int clocks_start(void)
{
    int err, res;
    struct onoff_manager *mgr;
    struct onoff_client cli;
    mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!mgr) return -ENXIO;
    sys_notify_init_spinwait(&cli.notify);
    err = onoff_request(mgr, &cli);
    do { err = sys_notify_fetch_result(&cli.notify, &res); } while (err);
    return 0;
}

/* ============================ MAIN ============================ */
int main(void)
{
    /* BOOT MARKER - should appear immediately */
    LOG_INF("========================================");
    LOG_INF("=== nRF52840_Pololu Firmware v1.0 ===");
    LOG_INF("========================================");
    
    LOG_INF("Async ESB PRX Start");
    clocks_start();
    esb_initialize();
    esb_start_rx();
    log_blocks_init();
    uart_initialization();
    leds_init();

    LOG_INF("Ready for Crazyflie handshake + log streaming + param read/write");
    LOG_INF("========================================");
    LOG_INF("Missing components: robot status handling");
    LOG_INF("running basic tests ...");

    // Test: Send a parameter to robot on startup to verify UART TX
    k_msleep(1000);
    send_test_param_to_robot();

    uint32_t last_heartbeat = 0;
    uint32_t last_log_print = 0;
    uint32_t rx_count = 0;
    bool param_sync_reported = false;

    struct esb_payload cmd;
    while (1) {
        if (uart_queue_receive(&cmd) == 0) {
            rx_count++;
            if (is_parameter_packet(cmd.data, cmd.length)) {
                handle_parameter_packet(cmd.data, cmd.length);
                LOG_INF(">>> Param updated from ROBOT <<<");
                print_all_parameters();
            } else {
                // Forward other packets to radio
                esb_send_async(&cmd);
            }
        }

        uint32_t now = k_uptime_get_32();

        // Print log table every 30s
        if (now - last_log_print > 30000) {
            print_log_table();
            last_log_print = now;
        }

        // Heartbeat every 5s with RX counter
        if (now - last_heartbeat > 5000) {
            LOG_INF("Heartbeat: RX=%d uptime=%ds", rx_count, now/1000);
            last_heartbeat = now;

            // Check param sync status periodically until complete
            if (!param_sync_reported) {
                if (check_param_sync_status()) {
                    param_sync_reported = true;
                }
            }
        }

        k_msleep(50);
    }
}
