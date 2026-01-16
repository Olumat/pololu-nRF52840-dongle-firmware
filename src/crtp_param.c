// Definitions for parameters declared in param.h
#include "crtp_param.h"

/* Use the logging module registered in main.c */
LOG_MODULE_DECLARE(esb_prx);

// Some Robot Params --> dummies for pololu
float kp_inner = 0.0f;
float ki_inner = 0.1f;
uint8_t kd_inner = 0;

// Trajectory gains
float kx_traj = 10.0f;
float ky_traj = 10.0f;
float ktheta_traj = 5.0f;

// Robot parameters - extended set to match Rust RobotConfig
uint8_t robot_id = 7;
float joystick_control_dt_ms = 50.0f;
float traj_following_dt_s = 0.05f;
float wheel_radius = 0.016f;
float wheel_base = 0.085f;
float motor_direction_left = -1.0f;
float motor_direction_right = 1.0f;
float motor_max_duty_left = 1.0f;
float motor_max_duty_right = 1.0f;
float k_clip = 1.0f;
float gear_ratio = 100.37f;
float encoder_cpr = -1204.44f;
float max_speed = 0.5f;
float max_omega = 3.14f;
float wheel_max = 31.25f;

// Status parameters (read-only, received from robot)
uint8_t robot_mode = 0;      // 0=Menu, 1=TeleOp, 2=TrajMocap, 3=TrajDuty
uint8_t robot_running = 0;   // 0=idle, 1=running

// Parameter sync statistics
uint32_t param_packets_received = 0;
uint32_t param_packets_valid = 0;
uint32_t param_packets_invalid = 0;

// Tracks which config parameters (0-20) have been received from robot
uint32_t param_received_mask = 0;


// param handling
bool handle_param_crtp(uint8_t port, uint8_t chan, const uint8_t *pl, uint8_t plen, uint8_t hdr, uint8_t pipe)
{
    if (port != 2)
        return false;

    struct esb_payload tx = {0};
    tx.pipe = pipe;

    // TOC_INFO (chan 0, cmd 0x03)
    if (chan == 0 && plen >= 1 && pl[0] == 0x03)
    {
        uint32_t crc = param_calculate_crc();
        tx.length = 8;
        tx.data[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
        tx.data[1] = 0x03;                      // TOC_INFO echo
        tx.data[2] = PARAM_COUNT & 0xFF;        // count low byte
        tx.data[3] = (PARAM_COUNT >> 8) & 0xFF; // count high byte
        memcpy(&tx.data[4], &crc, 4);           // CRC (little-endian)
        esb_send_async(&tx);
        LOG_INF("PARAM TOC_INFO reply: count=%d crc=0x%08X", PARAM_COUNT, crc);
        my_downlink_ctr ^= 1;
        return true;
    }

    // TOC_ELEMENT (chan 0, cmd 0x02)
    if (chan == 0 && plen >= 3 && pl[0] == 0x02)
    {
        uint16_t id = pl[1] | (pl[2] << 8);

        if (id >= PARAM_COUNT)
        {
            LOG_WRN("PARAM TOC_ELEMENT: id=%d out of range", id);
            return true;
        }

        const param_t *p = &param_toc[id];

        // Build type byte: lower 4 bits = type, bit 6 = writable (0=writable, 1=readonly)
        uint8_t type_byte = p->type;
        if (p->access == PARAM_RONLY)
        {
            type_byte |= (1 << 6); // Set readonly bit
        }

        uint8_t *out = tx.data;
        out[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
        out[1] = 0x02;      // TOC_ELEMENT echo
        out[2] = id & 0xFF; // id low byte
        out[3] = id >> 8;   // id high byte
        out[4] = type_byte; // type + access flags

        size_t offset = 5;
        size_t gl = strlen(p->group) + 1;
        size_t nl = strlen(p->name) + 1;
        memcpy(&out[offset], p->group, gl);
        offset += gl;
        memcpy(&out[offset], p->name, nl);
        offset += nl;

        tx.length = offset;
        esb_send_async(&tx);
        LOG_INF("PARAM TOC_ELEMENT: id=%d %s.%s type=0x%02X", id, p->group, p->name, type_byte);
        my_downlink_ctr ^= 1;
        return true;
    }

    // READ (chan 1)
    if (chan == 1 && plen >= 2)
    {
        uint16_t id = pl[0] | (pl[1] << 8);

        if (id >= PARAM_COUNT)
        {
            LOG_WRN("PARAM READ: id=%d out of range", id);
            return true;
        }

        const param_t *p = &param_toc[id];
        uint8_t size = param_get_size(p->type);

        tx.data[0] = (hdr & 0xF3) | (my_downlink_ctr << 2);
        tx.data[1] = id & 0xFF;
        tx.data[2] = id >> 8;
        tx.data[3] = 0x00; // Success code

        // Copy value in little-endian format
        memcpy(&tx.data[4], p->address, size);
        tx.length = 4 + size;

        esb_send_async(&tx);
        LOG_INF("PARAM READ: id=%d %s.%s", id, p->group, p->name);
        my_downlink_ctr ^= 1;
        return true;
    }

    // WRITE (chan 2)
    if (chan == 2 && plen >= 3)
    {
        uint16_t id = pl[0] | (pl[1] << 8);

        if (id >= PARAM_COUNT)
        {
            LOG_WRN("PARAM WRITE: id=%d out of range", id);
            return true;
        }

        const param_t *p = &param_toc[id];

        // Check if writable
        if (p->access == PARAM_RONLY)
        {
            LOG_WRN("PARAM WRITE: id=%d is read-only", id);
            // Still echo back but don't write
            tx.data[0] = (hdr & 0xF3) | (my_downlink_ctr << 2);
            tx.data[1] = id & 0xFF;
            tx.data[2] = id >> 8;
            tx.data[3] = 0x01; // Error code: read-only
            tx.length = 4;
            esb_send_async(&tx);
            my_downlink_ctr ^= 1;
            return true;
        }

        uint8_t size = param_get_size(p->type);

        // Verify we have enough data
        if (plen < 2 + size)
        {
            LOG_WRN("PARAM WRITE: insufficient data");
            return true;
        }

        // Write the value to local cache
        memcpy(p->address, &pl[2], size);

        // Forward parameter to robot via UART
        // Format: [length=8][header=0x3C][param_id][value(4 bytes f32)][checksum(2 bytes)]
        float value_f32;
        if (p->type == PARAM_TYPE_UINT8) {
            value_f32 = (float)(*(uint8_t *)p->address);
        } else {
            value_f32 = *(float *)p->address;
        }

        parameter_packet_t uart_pkt;
        uart_pkt.length = 8;  // length excluding length byte itself
        uart_pkt.header = 0x3C;
        uart_pkt.param_id = (uint8_t)id;
        uart_pkt.value = value_f32;
        uart_pkt.checksum = param_packet_checksum(&uart_pkt);

        // Send parameter to robot via UART
        print_uart_payload((uint8_t *)&uart_pkt, sizeof(uart_pkt));
        LOG_INF("PARAM WRITE: id=%d %s.%s -> UART (value=%.3f, checksum=0x%04X)", 
                id, p->group, p->name, (double)value_f32, uart_pkt.checksum);

        // Echo back the written value to radio
        tx.data[0] = (hdr & 0xF3) | (my_downlink_ctr << 2);
        tx.data[1] = id & 0xFF;
        tx.data[2] = id >> 8;
        memcpy(&tx.data[3], &pl[2], size);
        tx.length = 3 + size;

        esb_send_async(&tx);
        my_downlink_ctr ^= 1;

        // Print param table after client update
        LOG_INF(">>> Param updated from CLIENT <<<");
        print_all_parameters();
        return true;
    }

    return false;
}

// crc32 for TOC
uint32_t param_calculate_crc(void)
{
    uint32_t crc = 0;
    for (int i = 0; i < PARAM_COUNT; i++)
    {
        const param_t *p = &param_toc[i];
        // Simple hash of group and name
        const char *s = p->group;
        while (*s)
            crc = crc * 31 + *s++;
        s = p->name;
        while (*s)
            crc = crc * 31 + *s++;
        crc += p->type;
    }
    return crc;
}

// get parameters size
uint8_t param_get_size(uint8_t type)
{
    switch (type)
    {
    case PARAM_TYPE_UINT8:
    case PARAM_TYPE_INT8:
        return 1;
    case PARAM_TYPE_UINT16:
    case PARAM_TYPE_INT16:
        return 2;
    case PARAM_TYPE_UINT32:
    case PARAM_TYPE_INT32:
    case PARAM_TYPE_FLOAT:
        return 4;
    default:
        return 0;
    }
}

// Calculate checksum for parameter packet (matches Rust implementation)
uint16_t param_packet_checksum(const parameter_packet_t *packet)
{
    const uint8_t *value_bytes = (const uint8_t *)&packet->value;
    uint16_t sum = packet->length + packet->header + packet->param_id +
                   value_bytes[0] + value_bytes[1] + value_bytes[2] + value_bytes[3];
    return sum;
}

// Parse and handle parameter packet from UART (received from robot)
// Format: [length=8][0x3C][param_id][value(4)][checksum(2)] = 9 bytes total
bool handle_parameter_packet(const uint8_t *data, uint8_t len)
{
    param_packets_received++;

    // Expect exactly 9 bytes: length(1) + header(1) + id(1) + value(4) + checksum(2)
    if (len != sizeof(parameter_packet_t))
    {
        LOG_WRN("Invalid parameter packet length: %d (expected %d)", len, sizeof(parameter_packet_t));
        param_packets_invalid++;
        return false;
    }

    parameter_packet_t packet;
    memcpy(&packet, data, sizeof(parameter_packet_t));

    // Verify length field
    if (packet.length != 8)
    {
        LOG_WRN("Invalid parameter packet length field: %d (expected 8)", packet.length);
        param_packets_invalid++;
        return false;
    }

    // Verify checksum
    uint16_t expected_checksum = param_packet_checksum(&packet);
    if (packet.checksum != expected_checksum)
    {
        LOG_WRN("Parameter packet checksum mismatch: got 0x%04X, expected 0x%04X",
                packet.checksum, expected_checksum);
        param_packets_invalid++;
        return false;
    }

    // Verify header
    if (packet.header != 0x3C)
    {
        LOG_WRN("Invalid parameter packet header: 0x%02X", packet.header);
        param_packets_invalid++;
        return false;
    }

    // Update parameter if valid ID
    if (packet.param_id >= PARAM_COUNT)
    {
        LOG_WRN("Parameter ID out of range: %d", packet.param_id);
        param_packets_invalid++;
        return false;
    }

    const param_t *p = &param_toc[packet.param_id];

    // Special handling for uint8_t parameters stored as float
    if (packet.param_id == 0)
    {
        robot_id = (uint8_t)packet.value;
        LOG_INF("PARAM SYNC: %s.%s = %d (uint8)", p->group, p->name, robot_id);
    }
    else if (packet.param_id == 12)
    {
        kd_inner = (uint8_t)packet.value;
        LOG_INF("PARAM SYNC: %s.%s = %d (uint8)", p->group, p->name, kd_inner);
    }
    // Status parameters (21=mode, 22=running) - read-only, received from robot
    else if (packet.param_id == 21)
    {
        robot_mode = (uint8_t)packet.value;
        LOG_INF("PARAM SYNC: %s.%s = %d (mode)", p->group, p->name, robot_mode);
    }
    else if (packet.param_id == 22)
    {
        robot_running = (uint8_t)packet.value;
        LOG_INF("PARAM SYNC: %s.%s = %d (running)", p->group, p->name, robot_running);
    }
    // All other parameters are floats
    else
    {
        *(float *)p->address = packet.value;
        LOG_INF("PARAM SYNC: %s.%s = %.6f", p->group, p->name, packet.value);
    }

    // Mark this parameter as received in the bitmask
    if (packet.param_id < 32)
    {
        param_received_mask |= (1u << packet.param_id);
    }

    param_packets_valid++;
    return true;
}

// Check if received UART data is a parameter packet
// Format: [length=8][header=0x3C][...] = 9 bytes total
bool is_parameter_packet(const uint8_t *data, uint8_t len)
{
    return (len == sizeof(parameter_packet_t) && data[0] == 8 && data[1] == 0x3C);
}

// Print all current parameter values (for debugging)
void print_all_parameters(void)
{
    LOG_INF("=== Parameter Status ===");
    LOG_INF("Packets: received=%u valid=%u invalid=%u",
            param_packets_received, param_packets_valid, param_packets_invalid);
    LOG_INF("=== Current Parameter Values ===");
    for (int i = 0; i < PARAM_COUNT; i++)
    {
        const param_t *p = &param_toc[i];
        if (i == 0)
        {
            LOG_INF("%2d: %s.%s = %d", i, p->group, p->name, robot_id);
        }
        else if (i == 12)
        {
            LOG_INF("%2d: %s.%s = %d", i, p->group, p->name, kd_inner);
        }
        else if (i == 21)
        {
            LOG_INF("%2d: %s.%s = %d (mode)", i, p->group, p->name, robot_mode);
        }
        else if (i == 22)
        {
            LOG_INF("%2d: %s.%s = %d (running)", i, p->group, p->name, robot_running);
        }
        else
        {
            LOG_INF("%2d: %s.%s = %.6f", i, p->group, p->name, *(float *)p->address);
        }
    }
    LOG_INF("=== End Parameter Values ===");
}

// Send a test parameter packet to robot via UART (for debugging)
// Sends param_id=10 (KP_INNER) with value=1.234
void send_test_param_to_robot(void)
{
    LOG_INF("=== UART TX TEST START ===");
    
    parameter_packet_t test_pkt;
    test_pkt.length = 8;
    test_pkt.header = 0x3C;
    test_pkt.param_id = 10;  // KP_INNER
    test_pkt.value = 1.234f;
    test_pkt.checksum = param_packet_checksum(&test_pkt);

    uint8_t *bytes = (uint8_t *)&test_pkt;
    LOG_INF("Packet size: %d bytes", (int)sizeof(test_pkt));
    LOG_INF("TX bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4],
            bytes[5], bytes[6], bytes[7], bytes[8]);

    LOG_INF("Calling print_uart_payload()...");
    print_uart_payload((uint8_t *)&test_pkt, sizeof(test_pkt));
    LOG_INF("=== UART TX TEST DONE ===");
}

// Check and report parameter sync status from robot
// Returns true if all parameters (0-22) have been received
bool check_param_sync_status(void)
{
    // Expected mask: bits 0-22 all set = 0x7FFFFF (23 bits)
    uint32_t expected_mask = (1u << PARAM_CONFIG_COUNT) - 1;
    uint32_t received = param_received_mask & expected_mask;

    int received_count = 0;
    for (int i = 0; i < PARAM_CONFIG_COUNT; i++)
    {
        if (received & (1u << i))
        {
            received_count++;
        }
    }

    if (received == expected_mask)
    {
        LOG_INF("=== PARAM SYNC OK: All %d/%d parameters received from robot ===",
                received_count, PARAM_CONFIG_COUNT);
        LOG_INF("  Packets: total=%u valid=%u invalid=%u",
                param_packets_received, param_packets_valid, param_packets_invalid);
        return true;
    }
    else
    {
        LOG_WRN("=== PARAM SYNC INCOMPLETE: %d/%d parameters received ===",
                received_count, PARAM_CONFIG_COUNT);
        LOG_WRN("  Missing parameters:");
        for (int i = 0; i < PARAM_CONFIG_COUNT; i++)
        {
            if (!(received & (1u << i)))
            {
                const param_t *p = &param_toc[i];
                LOG_WRN("    ID %d: %s.%s", i, p->group, p->name);
            }
        }
        LOG_WRN("  Packets: total=%u valid=%u invalid=%u",
                param_packets_received, param_packets_valid, param_packets_invalid);
        return false;
    }
}