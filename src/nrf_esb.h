#pragma once
#ifndef NRF_ESB_H
#define NRF_ESB_H

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
#include "crtp.h"

/* ========================== TX & RX MESSAGE QUEUE ========================== */
#define TX_QUEUE_SIZE  128
#define RX_QUEUE_SIZE  128

extern struct esb_payload rx_payload;

/* ========================== INITIALIZATION ========================== */
int esb_initialize(void);

/* ========================== ASYNC SEND ========================== */
void esb_send_async(const struct esb_payload *src);
void esb_send_async_with_uart(const struct esb_payload *src);   // Send packet via radio and forward to UART

/* ========================== EVENT_HANDLER ========================== */
void event_handler(struct esb_evt const *event);

#endif