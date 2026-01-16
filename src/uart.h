#pragma once
#ifndef UART_H
#define UART_H
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <esb.h>
#include <zephyr/logging/log.h>

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0)); 
//static const struct device *const uart_sys = DEVICE_DT_GET(DT_NODELABEL(uart1)); 
//https://docs.zephyrproject.org/latest/kernel/drivers/index.html#c.DEVICE_DT_GET

#define ESB_MAX_PL 32
#define UART2ESB_Q_DEPTH 8
#define MSG_SIZE 64

extern struct k_msgq uart2esb_q;

typedef enum  {
    new_msg,
    current_msg,
} uart_msg_t;

struct usb_command {
    char payload[64];
    uint32_t length;
};

void print_uart(char *buf);
void print_uart_payload(uint8_t *buf, uint16_t msg_len);

int uart_initialization();

void serial_cb(const struct device *dev, void *user_data);
void decode_worker(void);

int uart_queue_send(const struct esb_payload *command);
int uart_queue_receive( struct esb_payload *command);

// void try_queue_ack_from_uartq(uint8_t pipe);

#endif