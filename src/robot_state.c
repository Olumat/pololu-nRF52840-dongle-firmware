#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include "robot_state.h"

LOG_MODULE_REGISTER(robot_state, 4);

// Legacy state (for backward compatibility)
state_loopback_t robot_state;
K_MUTEX_DEFINE(robot_state_lock);

// New log snapshot state
log_snapshot_t robot_log;
K_MUTEX_DEFINE(robot_log_lock);

// Statistics
static uint32_t log_packets_received = 0;
static uint32_t log_packets_valid = 0;

// Convert fixed-point i16 back to float
static inline float from_i16_mm(int16_t val)
{
    return (float)val / 1000.0f; // mm to meters
}

static inline float from_i16_centirad(int16_t val)
{
    return (float)val / 100.0f; // centirads to radians
}

static inline float from_i16_duty(int16_t val)
{
    return (float)val / 10000.0f;
}

// Check if packet is log snapshot format
bool is_log_snapshot_packet(const uint8_t *data, uint8_t len)
{
    if (len < 3)
        return false;
    return (data[0] == LOG_PACKET_PAYLOAD_LEN && data[1] == LOG_SNAPSHOT_HEADER);
}

// Decode compact log packet from robot
// Wire format: [len:1][header:1][robot_id:1][t_ms:4][17×i16:34]
bool parse_log_snapshot_packet(const uint8_t *data, uint8_t len)
{
    log_packets_received++;

    if (len != LOG_PACKET_COMPACT_LEN)
    {
        LOG_WRN("Log packet wrong length: %d (expected %d)", len, LOG_PACKET_COMPACT_LEN);
        return false;
    }

    if (data[0] != LOG_PACKET_PAYLOAD_LEN || data[1] != LOG_SNAPSHOT_HEADER)
    {
        LOG_WRN("Log packet invalid header: len=%d hdr=0x%02X", data[0], data[1]);
        return false;
    }

    // Parse fields from compact format
    uint8_t robot_id = data[2];
    uint32_t t_ms = data[3] | (data[4] << 8) | (data[5] << 16) | (data[6] << 24);

    const uint8_t *p = &data[7];
#define READ_I16(ptr) ((int16_t)((ptr)[0] | ((ptr)[1] << 8)))

    // Position values (mm precision)
    float x = from_i16_mm(READ_I16(p));
    p += 2;
    float y = from_i16_mm(READ_I16(p));
    p += 2;
    float x_des = from_i16_mm(READ_I16(p));
    p += 2;
    float y_des = from_i16_mm(READ_I16(p));
    p += 2;
    float x_err = from_i16_mm(READ_I16(p));
    p += 2;
    float y_err = from_i16_mm(READ_I16(p));
    p += 2;

    // Angles (centirad precision)
    float yaw = from_i16_centirad(READ_I16(p));
    p += 2;
    float yaw_des = from_i16_centirad(READ_I16(p));
    p += 2;
    float yaw_err = from_i16_centirad(READ_I16(p));
    p += 2;

    // Feedforward velocities
    float v_ff = from_i16_mm(READ_I16(p));
    p += 2;
    float w_ff = from_i16_centirad(READ_I16(p));
    p += 2;

    // Wheel speeds
    float omega_l_cmd = from_i16_centirad(READ_I16(p));
    p += 2;
    float omega_r_cmd = from_i16_centirad(READ_I16(p));
    p += 2;
    float omega_l_meas = from_i16_centirad(READ_I16(p));
    p += 2;
    float omega_r_meas = from_i16_centirad(READ_I16(p));
    p += 2;

    // Duty cycles
    float duty_l = from_i16_duty(READ_I16(p));
    p += 2;
    float duty_r = from_i16_duty(READ_I16(p));
    p += 2;

#undef READ_I16

    // Write to global state
    k_mutex_lock(&robot_log_lock, K_FOREVER);
    robot_log.header = LOG_SNAPSHOT_HEADER;
    robot_log.robot_id = robot_id;
    robot_log.t_ms = t_ms;
    robot_log.x = x;
    robot_log.y = y;
    robot_log.yaw = yaw;
    robot_log.x_des = x_des;
    robot_log.y_des = y_des;
    robot_log.yaw_des = yaw_des;
    robot_log.v_ff = v_ff;
    robot_log.w_ff = w_ff;
    robot_log.omega_l_cmd = omega_l_cmd;
    robot_log.omega_r_cmd = omega_r_cmd;
    robot_log.omega_l_meas = omega_l_meas;
    robot_log.omega_r_meas = omega_r_meas;
    robot_log.duty_l = duty_l;
    robot_log.duty_r = duty_r;
    robot_log.x_err = x_err;
    robot_log.y_err = y_err;
    robot_log.yaw_err = yaw_err;
    k_mutex_unlock(&robot_log_lock);

    // Update legacy state for backward compatibility
    k_mutex_lock(&robot_state_lock, K_FOREVER);
    robot_state.header = LOG_SNAPSHOT_HEADER;
    robot_state.robot_id = robot_id;
    robot_state.x = x;
    robot_state.y = y;
    robot_state.z = 0.0f;
    robot_state.vx = v_ff;
    robot_state.vy = 0.0f;
    robot_state.vz = 0.0f;
    // Yaw to quaternion
    float half_yaw = yaw * 0.5f;
    robot_state.qw = cosf(half_yaw);
    robot_state.qx = 0.0f;
    robot_state.qy = 0.0f;
    robot_state.qz = sinf(half_yaw);
    k_mutex_unlock(&robot_state_lock);

    log_packets_valid++;

    if (log_packets_valid % 100 == 0)
    {
        LOG_INF("LOG PKT #%u: t=%u x=%.3f y=%.3f yaw=%.2f",
                log_packets_valid, t_ms, (double)x, (double)y, (double)yaw);
    }

    return true;
}