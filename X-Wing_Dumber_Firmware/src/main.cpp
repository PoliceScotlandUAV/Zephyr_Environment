#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart4));

static void crc_accumulate(uint8_t data, uint16_t *crc)
{
    uint8_t tmp = data ^ (uint8_t)(*crc & 0xFF);
    tmp ^= (tmp << 4);
    *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

int main(void)
{
    if (!device_is_ready(uart_dev)) {
        return 0;
    }

    k_sleep(K_MSEC(5000));

    uint8_t seq = 0;

    while (true) {
        uint8_t payload[9] = {
            0x00, 0x00, 0x00, 0x00,  /* custom_mode           */
            0,                        /* MAV_TYPE_GENERIC      */
            0,                        /* MAV_AUTOPILOT_GENERIC */
            64,                       /* MAV_MODE_MANUAL       */
            4,                        /* MAV_STATE_ACTIVE      */
            3,                        /* mavlink_version       */
        };

        uint8_t frame[17];
        frame[0] = 0xFE;
        frame[1] = 9;
        frame[2] = seq++;
        frame[3] = 1;    // system id
        frame[4] = 1;    // MAV_COMP_ID_AUTOPILOT1
        frame[5] = 0;
        memcpy(&frame[6], payload, 9);

        /* CRC over bytes 1..14 then CRC-extra */
        uint16_t crc = 0xFFFF;
        for (int i = 1; i <= 14; i++) {
            crc_accumulate(frame[i], &crc);
        }
        crc_accumulate(50, &crc);  /* CRC-extra for HEARTBEAT */

        frame[15] = static_cast<uint8_t>(crc & 0xFF);
        frame[16] = static_cast<uint8_t>(crc >> 8);

        for (int i = 0; i < 17; i++) {
            uart_poll_out(uart_dev, frame[i]);
        }

        k_sleep(K_MSEC(1000));
    }

    return 0;
}