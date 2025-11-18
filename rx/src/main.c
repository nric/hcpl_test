#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

// RX: listens on UART0 GP1 at 1000 baud, SLIP framing with CRC16-CCITT.
// Handles inverted line (HCPL2630 output) via GPIO in-over invert.
// Reports frame statistics and recognizes built-in test frames.

#define RX_PIN 1
#define TX_PIN 0  // unused
#define LED_PIN 25
#define UART_BAUD 1000

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define FRAME_TIMEOUT_MS 15000
#define STATS_PERIOD_MS 2000
#define MAX_FRAME 512

#define TEST_SHORT_ID 0xA1
#define TEST_LONG_ID 0xB2

typedef struct {
    uint32_t frames_ok;
    uint32_t frames_crc_fail;
    uint32_t frames_too_short;
    uint32_t frames_too_long;
    uint32_t frames_timeout;
    uint32_t bytes_payload;
    uint32_t test_short_ok;
    uint32_t test_long_ok;
} stats_t;

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

static void process_frame(const uint8_t *data, size_t len, stats_t *st) {
    if (len < 2) {
        st->frames_too_short++;
        return;
    }
    const size_t payload_len = len - 2;
    const uint16_t crc_calc = crc16_ccitt(data, payload_len);
    const uint16_t crc_rx = ((uint16_t)data[payload_len] << 8) | data[payload_len + 1];
    if (crc_calc != crc_rx) {
        st->frames_crc_fail++;
        return;
    }

    st->frames_ok++;
    st->bytes_payload += payload_len;

    if (payload_len > 0) {
        if (data[0] == TEST_SHORT_ID) st->test_short_ok++;
        else if (data[0] == TEST_LONG_ID) st->test_long_ok++;
    }
}

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    uart_init(uart0, UART_BAUD);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_inover(RX_PIN, GPIO_OVERRIDE_INVERT);  // HCPL2630 inverts the signal

    sleep_ms(2000);  // allow USB CDC to enumerate
    printf("RX ready: UART0 GP1 inverted, %u baud, SLIP+CRC16 listening...\n", UART_BAUD);

    uint8_t frame_buf[MAX_FRAME];
    size_t frame_len = 0;
    bool esc = false;
    bool overflow = false;
    stats_t st = {0};
    uint64_t last_byte_ms = to_ms_since_boot(get_absolute_time());
    uint64_t last_report_ms = last_byte_ms;
    uint64_t last_led_ms = last_byte_ms;

    while (true) {
        while (uart_is_readable(uart0)) {
            uint8_t b = (uint8_t)uart_getc(uart0);
            last_byte_ms = to_ms_since_boot(get_absolute_time());

            if (overflow) {
                if (b == SLIP_END) {
                    st.frames_too_long++;
                    frame_len = 0;
                    esc = false;
                    overflow = false;
                }
                continue;
            }

            if (b == SLIP_END) {
                if (frame_len > 0) {
                    process_frame(frame_buf, frame_len, &st);
                    frame_len = 0;
                }
                esc = false;
                continue;
            }

            if (b == SLIP_ESC) {
                esc = true;
                continue;
            }

            if (esc) {
                if (b == SLIP_ESC_END) b = SLIP_END;
                else if (b == SLIP_ESC_ESC) b = SLIP_ESC;
                esc = false;
            }

            if (frame_len < MAX_FRAME) {
                frame_buf[frame_len++] = b;
            } else {
                overflow = true;
            }
        }

        const uint64_t now_ms = to_ms_since_boot(get_absolute_time());
        if (frame_len > 0 && (now_ms - last_byte_ms) > FRAME_TIMEOUT_MS) {
            st.frames_timeout++;
            frame_len = 0;
            esc = false;
            overflow = false;
        }

        if (now_ms - last_led_ms >= 1000) {
            last_led_ms = now_ms;
            gpio_xor_mask(1u << LED_PIN);
        }

        if (now_ms - last_report_ms >= STATS_PERIOD_MS) {
            last_report_ms = now_ms;
            printf("RX stats ok=%lu crc_fail=%lu too_short=%lu too_long=%lu timeout=%lu bytes=%lu test_short=%lu test_long=%lu\n",
                   (unsigned long)st.frames_ok, (unsigned long)st.frames_crc_fail,
                   (unsigned long)st.frames_too_short, (unsigned long)st.frames_too_long,
                   (unsigned long)st.frames_timeout, (unsigned long)st.bytes_payload,
                   (unsigned long)st.test_short_ok, (unsigned long)st.test_long_ok);
        }
    }
}
