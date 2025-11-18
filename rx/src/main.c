#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

// RX: listens on UART0 GP1 at 2500000 baud, SLIP framing with CRC16-CCITT.
// Handles inverted line (HCPL2630 output) via GPIO in-over invert.
// Reports frame statistics and recognizes built-in test frames.

#define RX_PIN 1
#define TX_PIN 0  // unused
#define LED_PIN 25
#define UART_BAUD 2500000

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define FRAME_TIMEOUT_MS 15000
#define STATS_PERIOD_MS 2000
#define MAX_FRAME 1200

#define TEST_SHORT_ID 0xA1
#define TEST_LONG_ID 0xB2
#define DATA_ID 0xD0
#define SEQ_BYTES 4
#define TEST_LONG_LEN 64
#define TEST_SHORT_LEN 6
#define TEST_LONG_TOTAL (SEQ_BYTES + TEST_LONG_LEN)
#define TEST_SHORT_TOTAL (SEQ_BYTES + TEST_SHORT_LEN)
#define FAIL_DUMP_BYTES 24

typedef struct {
    uint32_t frames_ok;
    uint32_t frames_crc_fail;
    uint32_t frames_too_short;
    uint32_t frames_too_long;
    uint32_t frames_timeout;
    uint32_t bytes_payload;
    uint32_t test_short_ok;
    uint32_t test_long_ok;
    uint32_t test_short_crc_fail;
    uint32_t test_long_crc_fail;
    uint32_t crc_fail_bitflips;
    uint32_t missed_frames;
    uint32_t seq_resets;
    uint32_t data_frames;
    uint32_t data_samples;
    uint64_t data_sum;
    uint16_t data_min;
    uint16_t data_max;
    bool data_init;
    bool last_seq_valid;
    uint32_t last_seq;
    bool last_fail_valid;
    uint32_t last_fail_seq;
    uint8_t last_fail_id;
    uint16_t last_fail_crc_calc;
    uint16_t last_fail_crc_rx;
    size_t last_fail_len;
    size_t last_fail_dump_len;
    uint8_t last_fail_dump[FAIL_DUMP_BYTES];
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

static size_t build_expected(uint8_t id, uint32_t seq, uint8_t *out, size_t max_len) {
    if (id == TEST_SHORT_ID && max_len >= TEST_SHORT_TOTAL) {
        out[0] = (uint8_t)(seq & 0xFF);
        out[1] = (uint8_t)((seq >> 8) & 0xFF);
        out[2] = (uint8_t)((seq >> 16) & 0xFF);
        out[3] = (uint8_t)((seq >> 24) & 0xFF);
        out[4] = TEST_SHORT_ID;
        const uint8_t exp[TEST_SHORT_LEN - 1] = {'S', 'H', 'O', 'R', 'T'};
        memcpy(&out[5], exp, sizeof(exp));
        return TEST_SHORT_TOTAL;
    }
    if (id == TEST_LONG_ID && max_len >= TEST_LONG_TOTAL) {
        out[0] = (uint8_t)(seq & 0xFF);
        out[1] = (uint8_t)((seq >> 8) & 0xFF);
        out[2] = (uint8_t)((seq >> 16) & 0xFF);
        out[3] = (uint8_t)((seq >> 24) & 0xFF);
        out[4] = TEST_LONG_ID;
        for (size_t i = 5; i < TEST_LONG_TOTAL; ++i) {
            out[i] = (uint8_t)((i - 4) & 0xFF);
        }
        return TEST_LONG_TOTAL;
    }
    return 0;
}

static uint32_t count_bitflips(const uint8_t *a, const uint8_t *b, size_t len) {
    uint32_t flips = 0;
    for (size_t i = 0; i < len; ++i) {
        uint8_t x = a[i] ^ b[i];
        x = x - ((x >> 1) & 0x55);
        x = (x & 0x33) + ((x >> 2) & 0x33);
        flips += (((x + (x >> 4)) & 0x0F) * 0x01);
    }
    return flips;
}

static void accumulate_data_samples(const uint8_t *bytes, size_t len, stats_t *st) {
    size_t i = 0;
    while (i + 2 < len) {
        uint16_t s0 = (uint16_t)bytes[i] | ((uint16_t)(bytes[i + 1] & 0x0F) << 8);
        uint16_t s1 = ((uint16_t)(bytes[i + 1] >> 4) & 0x0F) | ((uint16_t)bytes[i + 2] << 4);
        uint16_t vals[2] = {s0, s1};
        for (int v = 0; v < 2; ++v) {
            uint16_t sample = vals[v];
            st->data_samples++;
            st->data_sum += sample;
            if (!st->data_init) {
                st->data_min = st->data_max = sample;
                st->data_init = true;
            } else {
                if (sample < st->data_min) st->data_min = sample;
                if (sample > st->data_max) st->data_max = sample;
            }
        }
        i += 3;
    }
    if (i + 1 < len) {
        uint16_t s0 = (uint16_t)bytes[i] | ((uint16_t)(bytes[i + 1] & 0x0F) << 8);
        st->data_samples++;
        st->data_sum += s0;
        if (!st->data_init) {
            st->data_min = st->data_max = s0;
            st->data_init = true;
        } else {
            if (s0 < st->data_min) st->data_min = s0;
            if (s0 > st->data_max) st->data_max = s0;
        }
    }
}

static void process_frame(const uint8_t *data, size_t len, stats_t *st) {
    if (len < 2) {
        st->frames_too_short++;
        return;
    }
    const size_t payload_len = len - 2;
    const uint16_t crc_calc = crc16_ccitt(data, payload_len);
    const uint16_t crc_rx = ((uint16_t)data[payload_len] << 8) | data[payload_len + 1];
    uint32_t seq = 0;
    bool has_seq = false;
    if (payload_len >= SEQ_BYTES) {
        seq = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) |
              ((uint32_t)data[3] << 24);
        has_seq = true;
    }

    if (crc_calc != crc_rx) {
        st->frames_crc_fail++;
        st->last_fail_valid = true;
        st->last_fail_seq = seq;
        st->last_fail_id = payload_len > 0 ? data[has_seq ? SEQ_BYTES : 0] : 0x00;
        st->last_fail_crc_calc = crc_calc;
        st->last_fail_crc_rx = crc_rx;
        st->last_fail_len = payload_len;
        st->last_fail_dump_len = payload_len > FAIL_DUMP_BYTES ? FAIL_DUMP_BYTES : payload_len;
        memcpy(st->last_fail_dump, data, st->last_fail_dump_len);

        uint8_t exp[MAX_FRAME];
        const size_t exp_len = build_expected(st->last_fail_id, seq, exp, sizeof(exp));
        if (exp_len == payload_len && exp_len > 0) {
            st->crc_fail_bitflips += count_bitflips(exp, data, exp_len);
            if (st->last_fail_id == TEST_SHORT_ID) st->test_short_crc_fail++;
            else if (st->last_fail_id == TEST_LONG_ID) st->test_long_crc_fail++;
        }
        return;
    }

    st->frames_ok++;
    st->bytes_payload += payload_len;

    if (has_seq) {
        if (!st->last_seq_valid) {
            st->last_seq_valid = true;
            st->last_seq = seq;
        } else {
            if (seq > st->last_seq) {
                st->missed_frames += (seq - st->last_seq - 1);
            } else {
                st->seq_resets++;
            }
            st->last_seq = seq;
        }
    }

    if (payload_len >= SEQ_BYTES + 1) {
        const uint8_t id = data[SEQ_BYTES];
        if (payload_len == TEST_SHORT_TOTAL && id == TEST_SHORT_ID) {
            st->test_short_ok++;
        } else if (payload_len == TEST_LONG_TOTAL && id == TEST_LONG_ID) {
            st->test_long_ok++;
        } else if (id == DATA_ID && payload_len > (SEQ_BYTES + 1)) {
            const size_t count = payload_len - (SEQ_BYTES + 1);
            const uint8_t *samples = &data[SEQ_BYTES + 1];
            st->data_frames++;
            accumulate_data_samples(samples, count, st);
        }
    }
}

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    uart_init(uart0, UART_BAUD);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);  // 1 stop bit for throughput
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
            printf("RX stats ok=%lu crc_fail=%lu too_short=%lu too_long=%lu timeout=%lu bytes=%lu test_short=%lu test_long=%lu crc_fail_short=%lu crc_fail_long=%lu data_frames=%lu data_samples=%lu data_mean=%lu data_min=%u data_max=%u bitflips_sum=%lu missed_frames=%lu seq_resets=%lu",
                   (unsigned long)st.frames_ok, (unsigned long)st.frames_crc_fail,
                   (unsigned long)st.frames_too_short, (unsigned long)st.frames_too_long,
                   (unsigned long)st.frames_timeout, (unsigned long)st.bytes_payload,
                   (unsigned long)st.test_short_ok, (unsigned long)st.test_long_ok,
                   (unsigned long)st.test_short_crc_fail, (unsigned long)st.test_long_crc_fail,
                   (unsigned long)st.data_frames, (unsigned long)st.data_samples,
                   st.data_samples ? (unsigned long)(st.data_sum / st.data_samples) : 0ul,
                   st.data_samples ? st.data_min : 0, st.data_samples ? st.data_max : 0,
                   (unsigned long)st.crc_fail_bitflips, (unsigned long)st.missed_frames,
                   (unsigned long)st.seq_resets);
            if (st.last_fail_valid) {
                printf(" last_fail seq=%lu id=0x%02X len=%lu crc_rx=0x%04X crc_calc=0x%04X dump=",
                       (unsigned long)st.last_fail_seq, st.last_fail_id, (unsigned long)st.last_fail_len,
                       st.last_fail_crc_rx, st.last_fail_crc_calc);
                for (size_t i = 0; i < st.last_fail_dump_len; ++i) {
                    printf("%02X", st.last_fail_dump[i]);
                }
            }
            printf("\n");
        }
    }
}
