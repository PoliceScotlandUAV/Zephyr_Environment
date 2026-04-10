/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAVLink telemetry bridge — STM32H5 / Zephyr
 *
 * - Sends HEARTBEAT + PARAM_VALUE responses over UART4 (433MHz SiK radio)
 * - Receives MANUAL_CONTROL and COMMAND_LONG (arm/disarm)
 * - Forwards control inputs over FDCAN2
 * - Debug output via printk (RTT)
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* ── Devices ─────────────────────────────────────────────────────────── */
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart4));
static const struct device *const can_dev  = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

/* ── MAVLink identity ────────────────────────────────────────────────── */
#define SYS_ID          1
#define COMP_ID         1
#define MAVLINK_STX     0xFE

/* ── Message IDs ─────────────────────────────────────────────────────── */
#define MSG_HEARTBEAT        0
#define MSG_PARAM_VALUE      22
#define MSG_PARAM_REQUEST_LIST 21
#define MSG_PARAM_REQUEST_READ 20
#define MSG_COMMAND_LONG     76
#define MSG_COMMAND_ACK      77
#define MSG_MANUAL_CONTROL   69
#define MSG_AUTOPILOT_VERSION     148

/* ── CRC-extra per message ID ────────────────────────────────────────── */
#define CRC_EXTRA_HEARTBEAT        50
#define CRC_EXTRA_PARAM_VALUE      132
#define CRC_EXTRA_PARAM_REQ_LIST   159
#define CRC_EXTRA_PARAM_REQ_READ   214
#define CRC_EXTRA_COMMAND_LONG     152
#define CRC_EXTRA_COMMAND_ACK      143
#define CRC_EXTRA_MANUAL_CONTROL   243
#define CRC_EXTRA_AUTOPILOT_VER   178

/* ── Parameters ──────────────────────────────────────────────────────── */
/*
 * Minimal parameter set — just enough to silence QGC's "missing params"
 * warning. Values are all 0 (disabled) which is safe for a custom vehicle.
 */
#define MAV_PARAM_TYPE_INT32  6

bool armed = false;

/* ── CAN TX callback ─────────────────────────────────────────────────── */
static void can_tx_cb(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    if (error != 0) {
        printk("CAN TX error: %d\n", error);
    }
}

/* ── MAVLink CRC ─────────────────────────────────────────────────────── */
static void crc_accumulate(uint8_t data, uint16_t *crc)
{
    uint8_t tmp = data ^ (uint8_t)(*crc & 0xFF);
    tmp ^= (tmp << 4);
    *crc = (*crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4);
}

static uint16_t mavlink_crc(const uint8_t *buf, uint8_t len, uint8_t crc_extra)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc_accumulate(buf[i], &crc);
    }
    crc_accumulate(crc_extra, &crc);
    return crc;
}

/* ── UART TX ─────────────────────────────────────────────────────────── */
static void uart_send(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

/* ── MAVLink frame builder ───────────────────────────────────────────── */
static uint8_t tx_seq = 0;

static size_t build_frame(uint8_t *buf, uint8_t msg_id, uint8_t crc_extra,
                          const uint8_t *payload, uint8_t payload_len)
{
    buf[0] = MAVLINK_STX;
    buf[1] = payload_len;
    buf[2] = tx_seq++;
    buf[3] = SYS_ID;
    buf[4] = COMP_ID;
    buf[5] = msg_id;
    memcpy(&buf[6], payload, payload_len);

    uint16_t crc = mavlink_crc(&buf[1], 5 + payload_len, crc_extra);
    buf[6 + payload_len]     = (uint8_t)(crc & 0xFF);
    buf[6 + payload_len + 1] = (uint8_t)(crc >> 8);

    return 6 + payload_len + 2;
}

/* ── Send HEARTBEAT ──────────────────────────────────────────────────── */
static void send_heartbeat(void)
{
    uint8_t state = armed ? 64 : 0;

    uint8_t payload[9] = {
        0x00, 0x00, 0x00, 0x00,
        43,    /* MAV_TYPE_GENERIC      */
        0,    /* MAV_AUTOPILOT_GENERIC */
        state,    /* MAV_MODE_PREFLIGHT    */ // changed from 64
        3,    /* MAV_STATE_STANDBY     */ // changed from 4
        3,
    };

    uint8_t frame[17];
    size_t  len = build_frame(frame, MSG_HEARTBEAT, CRC_EXTRA_HEARTBEAT, payload, 9);
    uart_send(frame, len);
}


/* ── Send COMMAND_ACK ────────────────────────────────────────────────── */
static void send_command_ack(uint16_t command, uint8_t result)
{
    uint8_t payload[3] = {
        (uint8_t)(command & 0xFF),
        (uint8_t)(command >> 8),
        result,
    };

    uint8_t frame[11];
    size_t  len = build_frame(frame, MSG_COMMAND_ACK, CRC_EXTRA_COMMAND_ACK, payload, 3);
    uart_send(frame, len);
}

static void send_autopilot_version(void)
{
    /*
     * AUTOPILOT_VERSION payload (60 bytes):
     *   uint64 capabilities        bytes  0-7
     *   uint64 uid                 bytes  8-15  (vehicle unique id)
     *   uint32 flight_sw_version   bytes 16-19
     *   uint32 middleware_version  bytes 20-23
     *   uint32 os_version          bytes 24-27
     *   uint32 board_version       bytes 28-31
     *   uint8[8] flight_custom_ver bytes 32-39
     *   uint8[8] middleware_custom  bytes 40-47
     *   uint8[8] os_custom_version bytes 48-55
     *   uint16 vendor_id           bytes 56-57
     *   uint16 product_id          bytes 58-59
     */
    uint8_t payload[60] = {0};

    /*
     * capabilities bitmask — advertise only what we actually support:
     *   MAV_PROTOCOL_CAPABILITY_MAVLINK2          = (1 << 8)  — we speak v1 but claim v1
     *   MAV_PROTOCOL_CAPABILITY_COMMAND_INT       = (1 << 5)
     *
     * For a minimal custom vehicle just advertise 0 — QGC accepts it.
     */
    uint64_t caps = 0;
    memcpy(&payload[0], &caps, sizeof(uint64_t));

    /* uid — use 0, QGC accepts it */
    /* everything else stays 0 */

    uint8_t frame[60 + 8];
    size_t len = build_frame(frame, MSG_AUTOPILOT_VERSION,
                             CRC_EXTRA_AUTOPILOT_VER, payload, 60);
    uart_send(frame, len);
    printk("AUTOPILOT_VERSION sent\n");
}

/* ── Handle COMMAND_LONG ─────────────────────────────────────────────── */
static void handle_command_long(const uint8_t *payload)
{
    /*
     * COMMAND_LONG payload (33 bytes):
     *   float  param1..param7   bytes  0-27
     *   uint16 command          bytes 28-29
     *   uint8  target_system    byte  30
     *   uint8  target_component byte  31
     *   uint8  confirmation     byte  32
     */
    uint16_t command = (uint16_t)(payload[28] | (payload[29] << 8));
    printk("COMMAND_LONG: cmd=%d\n", command);

    if (command == 400) {
        /* MAV_CMD_COMPONENT_ARM_DISARM — param1: 1.0=arm, 0.0=disarm */
        uint32_t raw;
        memcpy(&raw, &payload[0], sizeof(float));
        float param1;
        memcpy(&param1, &raw, sizeof(float));

        uint8_t arm = (param1 > 0.5f) ? 1 : 0;
        printk("ARM/DISARM: %s\n", arm ? "ARM" : "DISARM");

        armed = arm;

        struct can_frame cf = { .id = 0x202, .dlc = 1, .flags = 0 };
        cf.data[0] = arm;
        can_send(can_dev, &cf, K_MSEC(10), can_tx_cb, NULL);

        send_command_ack(command, 0);  /* MAV_RESULT_ACCEPTED */
    } else if (command == 520) {
        send_command_ack(command, 0);  /* MAV_RESULT_ACCEPTED */
        send_autopilot_version();
    } else if (command == 521) {
    /* MAV_CMD_REQUEST_PROTOCOL_VERSION — just ACK accepted */
        send_command_ack(command, 0);
    } else if (command == 512) {
        /* MAV_CMD_REQUEST_MESSAGE — ACK accepted, ignore for now */
        send_command_ack(command, 0);
    } else {
        send_command_ack(command, 3);  /* MAV_RESULT_UNSUPPORTED */
    }
}

/* ── Handle MANUAL_CONTROL ───────────────────────────────────────────── */
static void handle_manual_control(const uint8_t *payload)
{
    /*
     * MANUAL_CONTROL payload (11 bytes):
     *   int16 x        bytes 0-1   pitch  (-1000..1000)
     *   int16 y        bytes 2-3   roll   (-1000..1000)
     *   int16 z        bytes 4-5   thrust (0..1000)
     *   int16 r        bytes 6-7   yaw    (-1000..1000)
     *   uint8 target   byte  8
     *   uint16 buttons bytes 9-10
     */
    int16_t x = (int16_t)(payload[0] | (payload[1] << 8));
    int16_t y = (int16_t)(payload[2] | (payload[3] << 8));
    int16_t z = (int16_t)(payload[4] | (payload[5] << 8));
    int16_t r = (int16_t)(payload[6] | (payload[7] << 8));

    printk("MANUAL_CONTROL x=%d y=%d z=%d r=%d\n", x, y, z, r);

    struct can_frame cf = { .dlc = 4, .flags = 0 };

    /* CAN 0x200: pitch + roll */
    cf.id      = 0x200;
    cf.data[0] = (uint8_t)(x & 0xFF);
    cf.data[1] = (uint8_t)(x >> 8);
    cf.data[2] = (uint8_t)(y & 0xFF);
    cf.data[3] = (uint8_t)(y >> 8);
    can_send(can_dev, &cf, K_MSEC(10), can_tx_cb, NULL);

    /* CAN 0x201: thrust + yaw */
    cf.id      = 0x201;
    cf.data[0] = (uint8_t)(z & 0xFF);
    cf.data[1] = (uint8_t)(z >> 8);
    cf.data[2] = (uint8_t)(r & 0xFF);
    cf.data[3] = (uint8_t)(r >> 8);
    can_send(can_dev, &cf, K_MSEC(10), can_tx_cb, NULL);
}


/* ── MAVLink parser ──────────────────────────────────────────────────── */
static enum {
    WAIT_STX, WAIT_LEN, WAIT_SEQ, WAIT_SYSID,
    WAIT_COMPID, WAIT_MSGID, WAIT_PAYLOAD, WAIT_CRC1, WAIT_CRC2,
} parse_state = WAIT_STX;

static uint8_t  p_len, p_seq, p_sysid, p_compid, p_msgid;
static uint8_t  p_payload[64];
static uint8_t  p_idx;
static uint16_t p_crc;

static uint8_t crc_extra_for(uint8_t msg_id)
{
    switch (msg_id) {
    case MSG_HEARTBEAT:          return CRC_EXTRA_HEARTBEAT;
    case MSG_PARAM_REQUEST_LIST: return CRC_EXTRA_PARAM_REQ_LIST;
    case MSG_PARAM_REQUEST_READ: return CRC_EXTRA_PARAM_REQ_READ;
    case MSG_COMMAND_LONG:       return CRC_EXTRA_COMMAND_LONG;
    case MSG_MANUAL_CONTROL:     return CRC_EXTRA_MANUAL_CONTROL;
    default:                     return 0;
    }
}

static void dispatch(void)
{
    /* Validate CRC */
    uint8_t header[5] = { p_len, p_seq, p_sysid, p_compid, p_msgid };
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 5; i++)      crc_accumulate(header[i], &crc);
    for (int i = 0; i < p_len; i++)  crc_accumulate(p_payload[i], &crc);
    crc_accumulate(crc_extra_for(p_msgid), &crc);

    printk("RX msg=%d len=%d crc=%s\n", p_msgid, p_len,
           (crc == p_crc) ? "OK" : "FAIL");

    if (crc != p_crc) {
        return;
    }

    switch (p_msgid) {
    case MSG_COMMAND_LONG:
        handle_command_long(p_payload);
        break;
    case MSG_MANUAL_CONTROL:
        handle_manual_control(p_payload);
        break;
    default:
        break;
    }
}

static void parse_byte(uint8_t c)
{
    switch (parse_state) {
    case WAIT_STX:
        if (c == MAVLINK_STX) parse_state = WAIT_LEN;
        break;
    case WAIT_LEN:
        /* Reject implausible lengths immediately */
        if (c > 64) {
            parse_state = WAIT_STX;
            break;
        }
        p_len = c;
        parse_state = WAIT_SEQ;
        break;
    case WAIT_SEQ:    p_seq    = c; parse_state = WAIT_SYSID;   break;
    case WAIT_SYSID:  p_sysid  = c; parse_state = WAIT_COMPID;  break;
    case WAIT_COMPID: p_compid = c; parse_state = WAIT_MSGID;   break;
    case WAIT_MSGID:
        /* Reject unknown message IDs we'll never handle */
        if (c != MSG_HEARTBEAT      &&
            c != MSG_PARAM_REQUEST_LIST &&
            c != MSG_PARAM_REQUEST_READ &&
            c != MSG_COMMAND_LONG   &&
            c != MSG_MANUAL_CONTROL) {
            parse_state = WAIT_STX;
            break;
        }
        p_msgid = c;
        p_idx   = 0;
        parse_state = (p_len > 0) ? WAIT_PAYLOAD : WAIT_CRC1;
        break;
    case WAIT_PAYLOAD:
        p_payload[p_idx++] = c;
        if (p_idx >= p_len) parse_state = WAIT_CRC1;
        break;
    case WAIT_CRC1:
        p_crc = c;
        parse_state = WAIT_CRC2;
        break;
    case WAIT_CRC2:
        p_crc |= (uint16_t)c << 8;
        dispatch();
        parse_state = WAIT_STX;
        break;
    }
}

/* ── Main ────────────────────────────────────────────────────────────── */
int main(void)
{
    if (!device_is_ready(uart_dev)) {
        printk("UART not ready\n");
        return 0;
    }

    if (!device_is_ready(can_dev)) {
        printk("CAN not ready\n");
        return 0;
    }

    can_start(can_dev);

    /* Wait for radio to link before sending anything */
    k_sleep(K_MSEC(5000));

    printk("MAVLink bridge started\n");

    uint32_t last_hb = 0;
    uint8_t  c;

    while (true) {
        uint32_t now = k_uptime_get_32();

        if (now - last_hb >= 1000) {
            send_heartbeat();
            last_hb = now;
        }

        if (uart_poll_in(uart_dev, &c) == 0) {
            parse_byte(c);
        }
    }

    return 0;
}