#include "crtp_log.h"

/* Use the logging module registered in main.c */
LOG_MODULE_DECLARE(esb_prx);

/* Define the log blocks storage (single definition for the extern in header) */
log_block_t log_blocks[LOG_BLOCK_ID_MAX];

/* Access the new robot_log state */
extern log_snapshot_t robot_log;
extern struct k_mutex robot_log_lock;

/* ========================== LOG BLOCK HANDLING ========================== */
void log_blocks_init(void)
{
    for (int i = 0; i < LOG_BLOCK_ID_MAX; i++) {
        log_blocks[i].active = false;
        log_blocks[i].block_id = i;
        log_blocks[i].period_ms = 0;
        memset(log_blocks[i].vars, 0, sizeof(log_blocks[i].vars));
    }
}

/* ========================== LOG THREAD ========================== */
void log_task(void)
{
    uint32_t last_sent[LOG_BLOCK_ID_MAX] = {0};

    while (1) {
        uint32_t now = k_uptime_get_32();

        for (int i = 0; i < LOG_BLOCK_ID_MAX; i++) {
            if (!log_blocks[i].active)
                continue;

            if (now - last_sent[i] < (log_blocks[i].period_ms * 10))
                continue;

            last_sent[i] = now;

            // Read current log snapshot under mutex
            log_snapshot_t log;
            k_mutex_lock(&robot_log_lock, K_FOREVER);
            log = robot_log;
            k_mutex_unlock(&robot_log_lock);

            uint32_t ts = now & 0xFFFFFF;

            struct esb_payload tx = {0};
            tx.pipe = 0;
            tx.length = 1 + 3 + 1 + 6 * sizeof(float);
            tx.data[0] = (5 << 4) | 2 | (my_downlink_ctr << 2);
            tx.data[1] = log_blocks[i].block_id;
            memcpy(&tx.data[2], &ts, 3);

            if (log_blocks[i].block_id == 0) {
                // Block 0: position + velocity (x, y, yaw, x_des, y_des, yaw_des)
                memcpy(&tx.data[5], &log.x, 4);
                memcpy(&tx.data[9], &log.y, 4);
                memcpy(&tx.data[13], &log.yaw, 4);
                memcpy(&tx.data[17], &log.x_des, 4);
                memcpy(&tx.data[21], &log.y_des, 4);
                memcpy(&tx.data[25], &log.yaw_des, 4);
            }
            else if (log_blocks[i].block_id == 1) {
                // Block 1: errors + feedforward (x_err, y_err, yaw_err, v_ff, w_ff, 0)
                float zero = 0.0f;
                memcpy(&tx.data[5], &log.x_err, 4);
                memcpy(&tx.data[9], &log.y_err, 4);
                memcpy(&tx.data[13], &log.yaw_err, 4);
                memcpy(&tx.data[17], &log.v_ff, 4);
                memcpy(&tx.data[21], &log.w_ff, 4);
                memcpy(&tx.data[25], &zero, 4);
            }
            else if (log_blocks[i].block_id == 2) {
                // Block 2: wheel data (duty_l, duty_r, omega_l_cmd, omega_r_cmd, omega_l_meas, omega_r_meas)
                memcpy(&tx.data[5], &log.duty_l, 4);
                memcpy(&tx.data[9], &log.duty_r, 4);
                memcpy(&tx.data[13], &log.omega_l_cmd, 4);
                memcpy(&tx.data[17], &log.omega_r_cmd, 4);
                memcpy(&tx.data[21], &log.omega_l_meas, 4);
                memcpy(&tx.data[25], &log.omega_r_meas, 4);
            }
            else if (log_blocks[i].block_id == 3) {
                // Block 3: status (robot_mode, robot_running, t_ms as float, 0, 0, 0)
                extern uint8_t robot_mode;
                extern uint8_t robot_running;
                float mode_f = (float)robot_mode;
                float running_f = (float)robot_running;
                float t_ms_f = (float)log.t_ms;
                float zero = 0.0f;
                memcpy(&tx.data[5], &mode_f, 4);
                memcpy(&tx.data[9], &running_f, 4);
                memcpy(&tx.data[13], &t_ms_f, 4);
                memcpy(&tx.data[17], &zero, 4);
                memcpy(&tx.data[21], &zero, 4);
                memcpy(&tx.data[25], &zero, 4);
            }
            else {
                continue;
            }

            esb_send_async(&tx);
            my_downlink_ctr ^= 1;
        }
        k_msleep(1);
    }
}

/* ========================== LOG COMMANDS ========================== */
bool handle_log_crtp(uint8_t port, uint8_t chan, const uint8_t *pl, uint8_t plen, uint8_t hdr, uint8_t pipe)
{
	if (port != 5) return false;

    struct esb_payload tx = {0};
    tx.pipe = pipe;

    // TOC_INFO 
    if (port == 5 && chan == 0 && plen >= 1 && pl[0] == 0x03) {
        tx.length = 4;
        tx.data[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
        tx.data[1] = 0x03;
        tx.data[2] = 0x18;    // number of items
        tx.data[3] = 0x00;
        esb_send_async(&tx);
        LOG_INF("LOG TOC_INFO reply sent (DL ctr=%d)", my_downlink_ctr);
        my_downlink_ctr ^= 1;
        return true;
    }

	// LOG TOC_GET_ITEM
	if (port == 5 && chan == 0 && plen >= 1 && pl[0] == 0x02) {
		uint16_t id = pl[1] | (pl[2] << 8);

		// Log variable groups matching the new block structure
		const char *group_pose = "pose";           // Block 0: position/setpoint
		const char *group_error = "error";         // Block 1: tracking errors
		const char *group_wheel = "wheel";         // Block 2: wheel/motor data
		const char *group_status = "status";       // Block 3: robot status
		const char *group = NULL;
		const char *name = NULL;

		switch (id) {
			// Block 0: pose (x, y, yaw, x_des, y_des, yaw_des)
			case 0: name = "x"; group = group_pose; break;
			case 1: name = "y"; group = group_pose; break;
			case 2: name = "yaw"; group = group_pose; break;
			case 3: name = "x_des"; group = group_pose; break;
			case 4: name = "y_des"; group = group_pose; break;
			case 5: name = "yaw_des"; group = group_pose; break;

			// Block 1: error (x_err, y_err, yaw_err, v_ff, w_ff, reserved)
			case 6: name = "x_err"; group = group_error; break;
			case 7: name = "y_err"; group = group_error; break;
			case 8: name = "yaw_err"; group = group_error; break;
			case 9: name = "v_ff"; group = group_error; break;
			case 10: name = "w_ff"; group = group_error; break;
			case 11: name = "reserved"; group = group_error; break;

			// Block 2: wheel (duty_l, duty_r, omega_l_cmd, omega_r_cmd, omega_l_meas, omega_r_meas)
			case 12: name = "duty_l"; group = group_wheel; break;
			case 13: name = "duty_r"; group = group_wheel; break;
			case 14: name = "omega_l_cmd"; group = group_wheel; break;
			case 15: name = "omega_r_cmd"; group = group_wheel; break;
			case 16: name = "omega_l_meas"; group = group_wheel; break;
			case 17: name = "omega_r_meas"; group = group_wheel; break;

			// Block 3: status (mode, running, t_ms, reserved...)
			case 18: name = "mode"; group = group_status; break;
			case 19: name = "running"; group = group_status; break;
			case 20: name = "t_ms"; group = group_status; break;
			case 21: name = "rsv1"; group = group_status; break;
			case 22: name = "rsv2"; group = group_status; break;
			case 23: name = "rsv3"; group = group_status; break;

			default: return true;
		}

		struct esb_payload tx = {0};
		tx.pipe = pipe;
		uint8_t *p = tx.data;

		// header
		p[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
		// TOC_GET_ITEM echo
		p[1] = pl[0];
		// id
		p[2] = id & 0xFF;
		p[3] = id >> 8;

		p[4] = 0x07;   // type = float

		size_t offset = 5;
		size_t gl = strlen(group) + 1;
		memcpy(&p[offset], group, gl);
		offset += gl;
		size_t nl = strlen(name) + 1;
		memcpy(&p[offset], name, nl);
		offset += nl;

		tx.length = offset;
		esb_send_async(&tx);

		LOG_INF("LOG TOC_GET_ITEM reply id=%d (%s.%s)", id, group, name);
		my_downlink_ctr ^= 1;
		return true;
	}

	// CREATE BLOCK
    if (port == 5 && chan == 1 && plen >= 1 && pl[0] == 0x06) {
        struct esb_payload tx = {0};
		tx.length = 4;
					
		tx.data[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
		tx.data[1] = pl[0];  // command echo (RESET)
		tx.data[2] = pl[1];
		tx.data[3] = 0x00;
		tx.pipe = pipe;
		esb_send_async(&tx);

		LOG_INF("LOG CREATE BLOCK reply sent");
		my_downlink_ctr ^= 1;  // Flip to match the radio protocol

        return true;
    }

	// LOG TOC APPEND_BLOCK_V2
	if (port == 5 && chan == 1 && plen >= 4 && pl[0] == 0x07) {
		uint8_t block_id = pl[1];
		uint8_t var_type = pl[2];
		uint16_t var_id = pl[3] | (pl[4] << 8);
		LOG_INF("APPEND_BLOCK_V2: block=%d var_id=%d type=0x%02X", block_id, var_id, var_type);

		struct esb_payload tx = {0};
		tx.pipe = pipe;
		tx.length = 4;
		tx.data[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
		tx.data[1] = 0x07;       // echo APPEND_BLOCK_V2
		tx.data[2] = block_id;   // same block_id
		tx.data[3] = 0x00;       // success code (0=OK)
		esb_send_async(&tx);

		LOG_INF("LOG APPEND_BLOCK_V2 reply sent (block=%d)", block_id);
		my_downlink_ctr ^= 1;
		return true;
	}

	// LOG START_BLOCK
	if (port == 5 && chan == 1 && plen >= 3 && pl[0] == 0x03) {
		uint8_t block_id = pl[1];
		uint8_t period = pl[2];
		LOG_INF("LOG START_BLOCK received: block_id=%d, period=%d", block_id, period*10);

		struct esb_payload tx = {0};
		tx.pipe = pipe;
		tx.length = 4;
		tx.data[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
		tx.data[1] = pl[0];     // echo START_BLOCK
		tx.data[2] = block_id; // same block_id
		tx.data[3] = 0x00;     // success code (0)
		esb_send_async(&tx);
		LOG_INF("LOG START_BLOCK reply sent");

        log_blocks[block_id].active = true;
        log_blocks[block_id].period_ms = period;
        LOG_INF("LOG START_BLOCK activated: block=%d, %dms", block_id, period*10);

		my_downlink_ctr ^= 1;
		return true;
	}

	// STOP_BLOCK (0x04)
	if (port == 5 && chan == 1 && plen >= 2 && pl[0] == 0x04) {
		uint8_t block_id = pl[1];
        log_blocks[block_id].active = false;

		struct esb_payload tx = {0};
		tx.length = 4;
		tx.data[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
		tx.data[1] = pl[0];
		tx.data[2] = block_id;
		tx.data[3] = 0x00;
		tx.pipe = pipe;
		esb_send_async(&tx);

		LOG_INF("LOG STOP_BLOCK reply sent (block=%d)", block_id);
		my_downlink_ctr ^= 1;
		return true;
	}

    // RESET
    if (port == 5 && chan == 1 && plen >= 1 && pl[0] == 0x05) {
        struct esb_payload tx = {0};
		tx.length = 4;
					
		tx.data[0] = (hdr & 0xF3) | (pl[0] & 0x08) | (my_downlink_ctr << 2);
		tx.data[1] = 0x05;  // command echo (RESET)
		tx.data[2] = 0x00;
		tx.data[3] = 0x00;
		tx.pipe = pipe;
		esb_send_async(&tx);
		LOG_INF("LOG RESET reply sent");
		my_downlink_ctr ^= 1;  // Flip to match the radio protocol

        return true;
    }

    return false;
}

// Print current log snapshot values (for debugging)
void print_log_table(void)
{
    log_snapshot_t log;
    k_mutex_lock(&robot_log_lock, K_FOREVER);
    log = robot_log;
    k_mutex_unlock(&robot_log_lock);

    LOG_INF("=== Log Table (t=%u ms) ===", log.t_ms);
    LOG_INF("  pose: x=%.3f y=%.3f yaw=%.3f", (double)log.x, (double)log.y, (double)log.yaw);
    LOG_INF("  des:  x=%.3f y=%.3f yaw=%.3f", (double)log.x_des, (double)log.y_des, (double)log.yaw_des);
    LOG_INF("  err:  x=%.3f y=%.3f yaw=%.3f", (double)log.x_err, (double)log.y_err, (double)log.yaw_err);
    LOG_INF("  ff:   v=%.3f w=%.3f", (double)log.v_ff, (double)log.w_ff);
    LOG_INF("  wheel cmd: L=%.3f R=%.3f", (double)log.omega_l_cmd, (double)log.omega_r_cmd);
    LOG_INF("  wheel meas: L=%.3f R=%.3f", (double)log.omega_l_meas, (double)log.omega_r_meas);
    LOG_INF("  duty: L=%.3f R=%.3f", (double)log.duty_l, (double)log.duty_r);
}