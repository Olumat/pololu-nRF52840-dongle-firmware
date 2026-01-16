#pragma once
#ifndef CRTP_PARAM_H
#define CRTP_PARAM_H

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
#include "led.h"

extern uint8_t my_downlink_ctr;

/* ========================== PARAM Table ========================== */
// Parameter types
#define PARAM_TYPE_UINT8 0x00
#define PARAM_TYPE_INT8 0x01
#define PARAM_TYPE_UINT16 0x02
#define PARAM_TYPE_INT16 0x03
#define PARAM_TYPE_UINT32 0x04
#define PARAM_TYPE_INT32 0x05
#define PARAM_TYPE_FLOAT 0x06

// Parameter access
#define PARAM_RONLY 0x00
#define PARAM_RW 0x01

// Some Robot Params --> dummies for pololu
// Declarations only: actual definitions (with initializers) live in src/param.c
extern float kp_inner;
extern float ki_inner;
extern uint8_t kd_inner;

// Trajectory gains (declared here, defined in src/param.c)
extern float kx_traj;
extern float ky_traj;
extern float ktheta_traj;

// Robot parameters - extended set to match Rust RobotConfig
extern uint8_t robot_id;
extern float joystick_control_dt_ms;
extern float traj_following_dt_s;
extern float wheel_radius;
extern float wheel_base;
extern float motor_direction_left;
extern float motor_direction_right;
extern float motor_max_duty_left;
extern float motor_max_duty_right;
extern float k_clip;
extern float gear_ratio;
extern float encoder_cpr;
extern float max_speed;
extern float max_omega;
extern float wheel_max;

// Status parameters (read-only, received from robot)
// For orchestrator controlled by ROS node via controller interface
extern uint8_t robot_mode;    // 0=Menu, 1=TeleOp, 2=TrajMocap, 3=TrajDuty
extern uint8_t robot_running; // 0=idle, 1=running

// Parameter packet structure
// Must be packed to ensure exactly 9 bytes with no padding
// TODO: check again with crazyflie-lib ... unify with robot and dongle
#pragma pack(push, 1)
typedef struct
{
    uint8_t length;    // Byte 0: Packet length (8 = excluding length byte)
    uint8_t header;    // Byte 1: 0x3C (Parameter packet identifier)
    uint8_t param_id;  // Byte 2: Parameter ID (0-20)
    float value;       // Bytes 3-6: f32 Little-Endian
    uint16_t checksum; // Bytes 7-8: 16-bit checksum (LE)
} parameter_packet_t;
#pragma pack(pop)

// Parameter table entry
typedef struct
{
    uint8_t type;
    uint8_t access;
    const char *group;
    const char *name;
    void *address;
} param_t;

//=========================== PARAM Table ==========================/
// Parameter Table of Contents (matches Rust parameter IDs in robot firmware)
static const param_t param_toc[] = {
    {PARAM_TYPE_UINT8, PARAM_RW, "robot", "ROBOT_ID", &robot_id},                     // 0
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "JOYSTICK_DT_MS", &joystick_control_dt_ms}, // 1
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "TRAJ_DT_S", &traj_following_dt_s},         // 2
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "WHEEL_RADIUS", &wheel_radius},             // 3
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "WHEEL_BASE", &wheel_base},                 // 4
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "MOTOR_DIR_L", &motor_direction_left},      // 5
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "MOTOR_DIR_R", &motor_direction_right},     // 6
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "MOTOR_MAX_L", &motor_max_duty_left},       // 7
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "MOTOR_MAX_R", &motor_max_duty_right},      // 8
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "K_CLIP", &k_clip},                         // 9
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "KP_INNER", &kp_inner},                     // 10
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "KI_INNER", &ki_inner},                     // 11
    {PARAM_TYPE_UINT8, PARAM_RW, "robot", "KD_INNER", &kd_inner},                     // 12
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "KX_TRAJ", &kx_traj},                       // 13
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "KY_TRAJ", &ky_traj},                       // 14
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "KTHETA_TRAJ", &ktheta_traj},               // 15
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "GEAR_RATIO", &gear_ratio},                 // 16
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "ENCODER_CPR", &encoder_cpr},               // 17
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "MAX_SPEED", &max_speed},                   // 18
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "MAX_OMEGA", &max_omega},                   // 19
    {PARAM_TYPE_FLOAT, PARAM_RW, "robot", "WHEEL_MAX", &wheel_max},                   // 20
    // Status parameters (read-only, sent from robot to dongle)
    {PARAM_TYPE_UINT8, PARAM_RONLY, "status", "MODE", &robot_mode},       // 21
    {PARAM_TYPE_UINT8, PARAM_RONLY, "status", "RUNNING", &robot_running}, // 22
};

#define PARAM_COUNT (sizeof(param_toc) / sizeof(param_t))

bool handle_param_crtp(uint8_t port, uint8_t chan, const uint8_t *pl, uint8_t plen, uint8_t hdr, uint8_t pipe);

// crc32 for TOC
uint32_t param_calculate_crc(void);

// get parameters size
uint8_t param_get_size(uint8_t type);

/* ========================== PARAMETER SYNC ========================== */
// Parameter sync statistics
extern uint32_t param_packets_received;
extern uint32_t param_packets_valid;
extern uint32_t param_packets_invalid;

// Tracks which parameters (0-22) have been received from robot
// Bit i is set when param_id i has been received at least once
extern uint32_t param_received_mask;

// Number of parameters expected from robot at startup (IDs 0-22)
#define PARAM_CONFIG_COUNT 23

// Calculate checksum for parameter packet (matches robot firmware implementation)
uint16_t param_packet_checksum(const parameter_packet_t *packet);

// Parse and handle parameter packet from UART
bool handle_parameter_packet(const uint8_t *data, uint8_t len);

// Check if received UART data is a parameter packet
bool is_parameter_packet(const uint8_t *data, uint8_t len);

// Print all current parameter values (for debugging)
void print_all_parameters(void);

// Send a test parameter packet to robot via UART (for debugging)
void send_test_param_to_robot(void);

// Check and report parameter sync status from robot
// Returns true if all config parameters (0-20) have been received
bool check_param_sync_status(void);

#endif