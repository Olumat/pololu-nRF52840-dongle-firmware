#include "uart.h"
#include "robot_state.h"
#include "crtp_param.h"

K_MSGQ_DEFINE(command_queue, sizeof(struct esb_payload), 10, 4);
// K_MSGQ_DEFINE(uart2esb_q, ESB_MAX_PL, UART2ESB_Q_DEPTH, 4);

LOG_MODULE_REGISTER(uart, 4);

uart_msg_t msg = new_msg;
static uint8_t bytes_to_read = 0;
static uint8_t rx_buf[64];
static uint8_t rx_buf_pos = 0;

// Send null-terminated string via UART (blocking, for debug only)
void print_uart(char *buf)
{
    int msg_len = strlen(buf);

    for (int i = 0; i < msg_len; i++)
    {
        uart_poll_out(uart_dev, buf[i]);
    }
}

// Send raw bytes via UART (prepends length byte)
void print_uart_payload(uint8_t *buf, uint16_t msg_len)
{
    uart_poll_out(uart_dev, msg_len);
    for (int i = 0; i < msg_len; i++)
    {
        uart_poll_out(uart_dev, buf[i]);
    }
}

// Initialize UART and enable RX interrupts
int uart_initialization()
{
    if (!device_is_ready(uart_dev))
    {
        LOG_ERR("UART device not found!");
        return 1;
    }

    LOG_INF("UART device ready, enabling RX interrupts...");
    uint8_t err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    if (err)
    {
        LOG_ERR("Failed to set UART callback: %d", err);
        return err;
    }
    uart_irq_rx_enable(uart_dev);
    LOG_INF("UART RX interrupts enabled - waiting for robot data...");
    return err;
}

// UART RX interrupt handler - receives framed messages
void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    static uint32_t rx_byte_count = 0;

    if (!uart_irq_update(dev))
    {
        return;
    }

    if (!uart_irq_rx_ready(dev))
    {
        return;
    }

    uart_fifo_read(dev, &c, 1);
    rx_byte_count++;

    switch (msg)
    {
    case new_msg:
        bytes_to_read = c; // TODO: change back to hex read instead of char
        msg = current_msg;
        break;

    case current_msg:
        rx_buf[rx_buf_pos++] = c;
        if (rx_buf_pos >= bytes_to_read)
        {
            static struct esb_payload command;
            memcpy(command.data, rx_buf, sizeof(uint8_t) * (bytes_to_read));
            command.length = bytes_to_read;
            k_msgq_put(&command_queue, &command, K_NO_WAIT);
            bytes_to_read = 0;
            msg = new_msg;
            rx_buf_pos = 0;
        }

        break;
    default:
        break;
    }
}

// Worker thread to decode UART packets
void decode_worker(void)
{
    struct esb_payload command;

    while (1)
    {
        if (k_msgq_get(&command_queue, &command, K_FOREVER) == 0)
        {
            uint8_t *buf = command.data;
            uint8_t len = command.length;

            // Parameter packet
            if (len == 8 && buf[0] == 0x3C)
            {
                LOG_DBG("decode_worker: param packet (id=%d)", buf[1]);
                extern bool handle_parameter_packet(const uint8_t *data, uint8_t len);
                uint8_t full_pkt[9];
                full_pkt[0] = 8;
                memcpy(&full_pkt[1], buf, 8);
                handle_parameter_packet(full_pkt, 9);
                continue;
            }

            // Log snapshot packet
            if (len == LOG_PACKET_PAYLOAD_LEN && buf[0] == LOG_SNAPSHOT_HEADER)
            {
                // Reconstruct full packet: prepend length byte
                uint8_t full_pkt[LOG_PACKET_COMPACT_LEN];
                full_pkt[0] = LOG_PACKET_PAYLOAD_LEN;
                memcpy(&full_pkt[1], buf, LOG_PACKET_PAYLOAD_LEN);
                parse_log_snapshot_packet(full_pkt, LOG_PACKET_COMPACT_LEN);
                continue;
            }

            // Unknown packet type - log for debugging
            LOG_WRN("decode_worker: unknown packet len=%d hdr=0x%02X", len, buf[0]);
        }
    }
}

/**
 * @brief Enqueue an ESB command payload into the UART command queue.
 *
 * @details
 * This function performs a non-blocking insert (`K_NO_WAIT`) into
 * `command_queue`. If the queue is full, the message will be dropped and
 * the function returns a non-zero error code.
 *
 * @param[in] command
 *     Pointer to the ESB payload structure to enqueue.
 *
 * @return 0 on success.
 * @return -ENOMSG or a non-zero error code if the queue is full.
 *
 * @note
 * This function is safe to call from both thread context and interrupt
 * context, as long as `K_NO_WAIT` is used (which avoids blocking).
 */
int uart_queue_send(const struct esb_payload *command)
{
    return k_msgq_put(&command_queue, command, K_NO_WAIT);
}

/**
 * @brief Retrieve a command payload from the UART command queue.
 *
 * @details
 * This function performs a non-blocking read (`K_NO_WAIT`) from
 * `command_queue`. If the queue is empty, the function returns immediately
 * with a non-zero error code.
 *
 * @param[out] command
 *     Pointer to a buffer where the retrieved ESB payload will be stored.
 *
 * @return 0 on success (a payload was retrieved).
 * @return -ENOMSG or a non-zero error code if the queue is empty.
 *
 * @note
 * For blocking behavior, the caller should use `k_msgq_get()` with a timeout
 * or `K_FOREVER` instead of this wrapper.
 */
int uart_queue_receive(struct esb_payload *command)
{
    return k_msgq_get(&command_queue, command, K_NO_WAIT);
}

// void try_queue_ack_from_uartq(uint8_t pipe)
// {
//     uint8_t buf[ESB_MAX_PL];
//     if (k_msgq_get(&uart2esb_q, buf, K_NO_WAIT) != 0) return;

//     struct esb_payload pl = {0};
//     pl.pipe   = pipe;           // normally 0
//     pl.length = ESB_MAX_PL;     // can be adjusted to required length
//     memcpy(pl.data, buf, ESB_MAX_PL);

//     int err = esb_write_payload(&pl);  // loaded to ACK Queue
//     if (err) {
//         // ACK FIFO busy/full, wait for next chance
//         (void)k_msgq_put(&uart2esb_q, buf, K_NO_WAIT);
//         LOG_DBG("ACK FIFO busy, defer. err=%d", err);
//     } else {
//         LOG_DBG("Queued ACK payload len=%d pipe=%d", pl.length, pl.pipe);
//     }
// }