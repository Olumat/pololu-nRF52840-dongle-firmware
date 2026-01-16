#pragma once
#include <stdint.h>

// Log snapshot from robot firmware (matches Rust LogSnapshot)
// Packet format: [len:1][header:1][robot_id:1][t_ms:4][17×i16:34] = 41 bytes
// Header = 0xA2 for log snapshot packets
typedef struct {
    uint8_t header;      // 0xA2
    uint8_t robot_id;
    uint32_t t_ms;       // Timestamp in milliseconds

    // Pose (world frame)
    float x;             // Position X [meters]
    float y;             // Position Y [meters]
    float yaw;           // Orientation [radians]

    // Setpoint (desired state)
    float x_des;         // Desired X position [meters]
    float y_des;         // Desired Y position [meters]
    float yaw_des;       // Desired yaw [radians]

    // Feedforward
    float v_ff;          // Feedforward linear velocity [m/s]
    float w_ff;          // Feedforward angular velocity [rad/s]

    // Wheel command
    float omega_l_cmd;   // Left wheel command [rad/s]
    float omega_r_cmd;   // Right wheel command [rad/s]

    // Encoder (measured speeds)
    float omega_l_meas;  // Left wheel measured [rad/s]
    float omega_r_meas;  // Right wheel measured [rad/s]

    // Motor output
    float duty_l;        // Left motor duty cycle [-1, 1]
    float duty_r;        // Right motor duty cycle [-1, 1]

    // Tracking errors (body frame)
    float x_err;         // Position error X [m]
    float y_err;         // Position error Y [m]
    float yaw_err;       // Yaw error [rad]
} log_snapshot_t;

// Log packet header identifier
#define LOG_SNAPSHOT_HEADER 0xA2

// Log packet sizes
#define LOG_PACKET_COMPACT_LEN 41  // [len:1][header:1][robot_id:1][t_ms:4][17×i16:34]
#define LOG_PACKET_PAYLOAD_LEN 40  // Without length byte

extern log_snapshot_t robot_log;
extern struct k_mutex robot_log_lock;

// Parse log snapshot packet from UART (compact i16 format)
// Returns true if packet was valid and parsed
bool parse_log_snapshot_packet(const uint8_t *data, uint8_t len);

// Check if data is a log snapshot packet
bool is_log_snapshot_packet(const uint8_t *data, uint8_t len);

// Legacy compatibility - maps to log_snapshot for position/orientation
typedef struct {
    uint8_t header;
    uint8_t robot_id;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float qw;
    float qx;
    float qy;
    float qz;
} state_loopback_t;

extern state_loopback_t robot_state;
extern struct k_mutex robot_state_lock;
