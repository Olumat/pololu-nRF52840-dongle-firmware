#pragma once
#ifndef CRTP_PLATFORM_H
#define CRTP_PLATFORM_H

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
#include "uart.h"
#include "nrf_esb.h"

extern uint8_t my_downlink_ctr;

bool handle_platform_crtp(uint8_t port, uint8_t chan, const uint8_t *pl, uint8_t plen, uint8_t hdr, uint8_t pipe);

#endif