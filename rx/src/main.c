#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include <hardware/uart.h>
#include <hardware/irq.h>

// RX: listens on UART0 GP1 at 1000000 baud, SLIP framing with CRC16-CCITT.
// Handles inverted line (HCPL2630 output) via GPIO in-over invert.
// Reports frame statistics and per-packet ADC sample stats (mean/min/max).
// Uses interrupt-driven RX with a large ring buffer to prevent data loss.
// Unpacks 12-bit samples (2 samples per 3 bytes).

#define RX_PIN 1
#define TX_PIN 0  // unused
#define LED_PIN 25
#define UART_BAUD 1000000

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define FRAME_TIMEOUT_MS 15000
#define STATS_PERIOD_MS 2000
#define MAX_FRAME 512

#define SEQ_BYTES 4
#define FAIL_DUMP_BYTES 24

#define RX_BUFFER_SIZE 8192

typedef struct {
    uint32_t frames_ok;
    uint32_t frames_crc_fail;
    uint32_t frames_too_short;
    uint32_t frames_too_long;
    uint32_t frames_bad_len;
    uint32_t frames_timeout;
    uint32_t bytes_payload;
    uint32_t missed_frames;
    uint32_t seq_resets;
    
    // UART HW errors
    uint32_t uart_overrun_err;
    uint32_t uart_break_err;
    uint32_t uart_parity_err;
    uint32_t uart_framing_err;
    uint32_t ring_buffer_overflow;

    bool last_seq_valid;
    uint32_t last_seq;
    bool last_fail_valid;
    uint32_t last_fail_seq;
    uint8_t last_fail_id;
    uint16_t last_fail_crc_calc;
    uint16_t last_fail_crc_rx;
    size_t last_fail_len;
    size_t last_fail_expected_len;
    size_t last_fail_dump_len;
    uint8_t last_fail_dump[FAIL_DUMP_BYTES];
    uint32_t last_samples;
    uint16_t last_min;
    uint16_t last_max;
    uint16_t last_mean;
    uint32_t last_payload_len;
} stats_t;

static stats_t st = {0};

// Ring buffer
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile size_t rx_head = 0;
static volatile size_t rx_tail = 0;

static void on_uart_rx() {
    while (uart_is_readable(uart0)) {
        // Check for errors first
        uint32_t dr = uart_get_hw(uart0)->dr;
        uint8_t ch = dr & 0xFF;
        
        if (dr & UART_UARTDR_OE_BITS) st.uart_overrun_err++;
        if (dr & UART_UARTDR_BE_BITS) st.uart_break_err++;
        if (dr & UART_UARTDR_PE_BITS) st.uart_parity_err++;
        if (dr & UART_UARTDR_FE_BITS) st.uart_framing_err++;

        size_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
        if (next_head != rx_tail) {
            rx_buffer[rx_head] = ch;
            rx_head = next_head;
        } else {
            st.ring_buffer_overflow++;
        }
    }
}

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
    uint32_t seq = 0;
    if (payload_len >= SEQ_BYTES) {
        seq = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) |
              ((uint32_t)data[3] << 24);
    }

    if (payload_len < SEQ_BYTES + 1) {
        st->frames_too_short++;
        return;
    }

    const uint8_t sample_count = data[SEQ_BYTES];
    // 12-bit packing: 2 samples in 3 bytes.
    // Packed bytes = ceil(sample_count * 1.5)
    // If sample_count is odd, we have (sample_count / 2) * 3 + 2 bytes?
    // Let's assume sample_count is always even for simplicity in packing, or handle odd case.
    // TX sends pairs. If odd, last one takes 2 bytes? Or maybe just ceil(count * 12 / 8).
    // count * 12 / 8 = count * 1.5.
    // If count=1, 1.5 -> 2 bytes.
    // If count=2, 3 bytes.
    // If count=40, 60 bytes.
    size_t packed_bytes = (sample_count * 3 + 1) / 2; // Integer math for ceil(count * 1.5)
    
    const size_t expected_len = SEQ_BYTES + 1 + packed_bytes;
    if (payload_len != expected_len) {
        st->frames_bad_len++;
        st->last_fail_valid = true;
        st->last_fail_seq = seq;
        st->last_fail_id = payload_len > SEQ_BYTES ? data[SEQ_BYTES] : 0x00;
        st->last_fail_len = payload_len;
        st->last_fail_expected_len = expected_len;
        st->last_fail_dump_len = payload_len > FAIL_DUMP_BYTES ? FAIL_DUMP_BYTES : payload_len;
        memcpy(st->last_fail_dump, data, st->last_fail_dump_len);
        return;
    }
    if (sample_count == 0) return;

    const uint16_t crc_calc = crc16_ccitt(data, payload_len);
    const uint16_t crc_rx = ((uint16_t)data[payload_len] << 8) | data[payload_len + 1];
    if (crc_calc != crc_rx) {
        st->frames_crc_fail++;
        st->last_fail_valid = true;
        st->last_fail_seq = seq;
        st->last_fail_id = payload_len > SEQ_BYTES ? data[SEQ_BYTES] : 0x00;
        st->last_fail_crc_calc = crc_calc;
        st->last_fail_crc_rx = crc_rx;
        st->last_fail_len = payload_len;
        st->last_fail_expected_len = expected_len;
        st->last_fail_dump_len = payload_len > FAIL_DUMP_BYTES ? FAIL_DUMP_BYTES : payload_len;
        memcpy(st->last_fail_dump, data, st->last_fail_dump_len);
        return;
    }

    st->frames_ok++;
    st->bytes_payload += payload_len;

    if (!st->last_seq_valid) {
        st->last_seq_valid = true;
        st->last_seq = seq;
    } else {
        if (seq > st->last_seq) {
            st->missed_frames += (seq - st->last_seq - 1);
        } else if (seq != st->last_seq) {
            st->seq_resets++;
        }
        st->last_seq = seq;
    }

    uint16_t min_v = 0xFFFF;
    uint16_t max_v = 0;
    uint32_t sum = 0;
    size_t offset = SEQ_BYTES + 1;
    
    // Unpack samples
    for (uint8_t i = 0; i < sample_count; i += 2) {
        // Get 3 bytes for 2 samples (or 2 bytes for 1 sample if last is odd)
        uint8_t b0 = data[offset++];
        uint8_t b1 = (i + 1 < sample_count) ? data[offset++] : 0;
        uint8_t b2 = (i + 1 < sample_count) ? data[offset++] : 0;
        
        uint16_t s0 = (uint16_t)b0 | ((uint16_t)(b1 & 0x0F) << 8);
        if (s0 < min_v) min_v = s0;
        if (s0 > max_v) max_v = s0;
        sum += s0;
        
        if (i + 1 < sample_count) {
            uint16_t s1 = (uint16_t)((b1 & 0xF0) >> 4) | ((uint16_t)b2 << 4);
            if (s1 < min_v) min_v = s1;
            if (s1 > max_v) max_v = s1;
            sum += s1;
        }
    }
    
    st->last_samples = sample_count;
    st->last_min = min_v;
    st->last_max = max_v;
    st->last_mean = (uint16_t)(sum / sample_count);
    st->last_payload_len = payload_len;
}

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    uart_init(uart0, UART_BAUD);
    uart_set_format(uart0, 8, 2, UART_PARITY_NONE);  // 2 stop bits for margin
    uart_set_fifo_enabled(uart0, true);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_inover(RX_PIN, GPIO_OVERRIDE_INVERT);  // HCPL2630 inverts the signal

    // Setup interrupt
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);

    sleep_ms(2000);  // allow USB CDC to enumerate
    printf("RX ready: UART0 GP1 inverted, %u baud, SLIP+CRC16 listening...\n", UART_BAUD);
    printf("RX features: Interrupt-driven, Ring Buffer %d bytes, 12-bit unpacking\n", RX_BUFFER_SIZE);

    uint8_t frame_buf[MAX_FRAME];
    size_t frame_len = 0;
    bool esc = false;
    bool overflow = false;
    uint64_t last_byte_ms = to_ms_since_boot(get_absolute_time());
    uint64_t last_report_ms = last_byte_ms;
    uint64_t last_led_ms = last_byte_ms;

    while (true) {
        // Process ring buffer
        while (rx_head != rx_tail) {
            uint8_t b = rx_buffer[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
            
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
            printf("RX stats ok=%lu crc_fail=%lu too_short=%lu too_long=%lu bad_len=%lu timeout=%lu bytes=%lu missed_frames=%lu seq_resets=%lu\n",
                   (unsigned long)st.frames_ok, (unsigned long)st.frames_crc_fail,
                   (unsigned long)st.frames_too_short, (unsigned long)st.frames_too_long,
                   (unsigned long)st.frames_bad_len, (unsigned long)st.frames_timeout,
                   (unsigned long)st.bytes_payload, (unsigned long)st.missed_frames,
                   (unsigned long)st.seq_resets);
            printf("RX errs  overrun=%lu break=%lu parity=%lu framing=%lu rb_overflow=%lu\n",
                   (unsigned long)st.uart_overrun_err, (unsigned long)st.uart_break_err,
                   (unsigned long)st.uart_parity_err, (unsigned long)st.uart_framing_err,
                   (unsigned long)st.ring_buffer_overflow);
            
            if (st.last_samples > 0) {
                printf(" last_ok seq=%lu samples=%lu len=%lu mean=%u min=%u max=%u\n",
                       (unsigned long)st.last_seq, (unsigned long)st.last_samples,
                       (unsigned long)st.last_payload_len, st.last_mean, st.last_min, st.last_max);
            }
            if (st.last_fail_valid) {
                printf(" last_fail seq=%lu id=0x%02X len=%lu exp_len=%lu crc_rx=0x%04X crc_calc=0x%04X dump=",
                       (unsigned long)st.last_fail_seq, st.last_fail_id, (unsigned long)st.last_fail_len,
                       (unsigned long)st.last_fail_expected_len, st.last_fail_crc_rx, st.last_fail_crc_calc);
                for (size_t i = 0; i < st.last_fail_dump_len; ++i) {
                    printf("%02X", st.last_fail_dump[i]);
                }
                printf("\n");
            }
            printf("\n");
        }
    }
}

