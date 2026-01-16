#pragma once
#ifndef LOG_H
#define LOG_H

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

#include "crtp.h"
#include "nrf_esb.h"
#include "uart.h"
#include "robot_state.h"

#define LOG_BLOCK_ID_MAX 4

extern uint8_t my_downlink_ctr;

typedef struct {
    bool active;
    uint8_t block_id;
    uint32_t period_ms;
    float vars[3]; // roll pitch yaw
} log_block_t;

extern log_block_t log_blocks[LOG_BLOCK_ID_MAX];

void log_blocks_init(void);
void log_task(void);
bool handle_log_crtp(uint8_t port, uint8_t chan, const uint8_t *pl, uint8_t plen, uint8_t hdr, uint8_t pipe);
void print_log_table(void);

#endif