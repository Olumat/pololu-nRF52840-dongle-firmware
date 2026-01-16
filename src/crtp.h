#pragma once
#ifndef CRTP_H
#define CRTP_H

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
#include "robot_state.h"
#include "crtp_log.h"
#include "crtp_param.h"
#include "crtp_memory.h"
#include "crtp_platform.h"

extern uint8_t my_downlink_ctr;

/* Note: do NOT declare the log module in headers — declare per .c file */

extern struct k_msgq tx_queue;
extern struct k_msgq rx_queue;

/* ============================ CRTP HANDLERS ============================ */
bool handle_scan_packet(const uint8_t *data, uint8_t len, uint8_t pipe);
bool handle_keepalive(const uint8_t *data, uint8_t len, uint8_t pipe);

void tx_worker(void);
void rx_worker(void);
void handle_received_packet(struct esb_payload *rx);

#endif