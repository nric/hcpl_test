#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <hardware/uart.h>
#include <hardware/irq.h>

// RX: listens on UART0 GP1 at 1000000 baud, SLIP framing with CRC16-CCITT.
// Handles inverted line (HCPL2630 output) via GPIO in-over invert.
// Triggered Capture Mode:
// 1. Wait for first valid packet -> Calculate Baseline (Mean, StdDev).
// 2. Monitor stream -> Trigger if sample > Mean + 5*StdDev or < Mean - 5*StdDev.
// 3. Capture buffer of samples.
// 4. Report stats + raw values.
// 5. Return to Monitor.

#define RX_PIN 1
#define TX_PIN 0  // unused
#define LED_PIN 25
#define UART_BAUD 1000000

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define FRAME_TIMEOUT_MS 15000
#define MAX_FRAME 512
#define SEQ_BYTES 4
#define RX_BUFFER_SIZE 8192

#define CAPTURE_BUFFER_SIZE 1000
#define CALIBRATION_TARGET_SAMPLES 40000 // ~1 second at 40ksps
#define TRIGGER_SIGMA 6.0
#define TRIGGER_CONSECUTIVE_SAMPLES 3

typedef enum {
    STATE_WAIT_FOR_BASELINE,
    STATE_MONITOR,
    STATE_CAPTURE,
    STATE_REPORT
} rx_state_t;

typedef struct {
    rx_state_t state;
    
    // Baseline stats accumulation
    double calibration_sum;
    double calibration_sum_sq;
    size_t calibration_count;

    // Final Baseline
    double baseline_mean;
    double baseline_std_dev;
    bool baseline_valid;

    // Trigger Debouncing
    size_t trigger_consecutive;

    // Capture buffer
    uint16_t capture_buf[CAPTURE_BUFFER_SIZE];
    size_t capture_idx;

    // General stats (kept for debugging if needed, but less verbose now)
    uint32_t frames_ok;
    uint32_t frames_crc_fail;
} app_ctx_t;

static app_ctx_t ctx = {0};

// Ring buffer for UART RX
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile size_t rx_head = 0;
static volatile size_t rx_tail = 0;

static void on_uart_rx() {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);
        size_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
        if (next_head != rx_tail) {
            rx_buffer[rx_head] = ch;
            rx_head = next_head;
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

// Helper to calculate mean and std dev of a buffer
static void calc_stats(const uint16_t *data, size_t count, double *mean_out, double *std_out) {
    if (count == 0) {
        *mean_out = 0;
        *std_out = 0;
        return;
    }
    double sum = 0;
    for (size_t i = 0; i < count; ++i) {
        sum += data[i];
    }
    double mean = sum / count;
    
    double sum_sq_diff = 0;
    for (size_t i = 0; i < count; ++i) {
        double diff = data[i] - mean;
        sum_sq_diff += diff * diff;
    }
    *mean_out = mean;
    *std_out = sqrt(sum_sq_diff / count);
}

static void process_samples(const uint16_t *samples, size_t count) {
    if (count == 0) return;

    switch (ctx.state) {
        case STATE_WAIT_FOR_BASELINE: {
            for (size_t i = 0; i < count; ++i) {
                ctx.calibration_sum += samples[i];
                ctx.calibration_sum_sq += (double)samples[i] * samples[i];
                ctx.calibration_count++;

                if (ctx.calibration_count >= CALIBRATION_TARGET_SAMPLES) {
                    // Calculate final stats
                    ctx.baseline_mean = ctx.calibration_sum / ctx.calibration_count;
                    double variance = (ctx.calibration_sum_sq / ctx.calibration_count) - (ctx.baseline_mean * ctx.baseline_mean);
                    ctx.baseline_std_dev = sqrt(variance);
                    
                    ctx.baseline_valid = true;
                    printf("Baseline established over %zu samples: Mean=%.2f, StdDev=%.2f\n", 
                           ctx.calibration_count, ctx.baseline_mean, ctx.baseline_std_dev);
                    
                    ctx.state = STATE_MONITOR;
                    
                    // Reset trigger state
                    ctx.trigger_consecutive = 0;

                    // Stop processing this packet as baseline (any remaining samples are ignored or could be monitored)
                    // For simplicity, we just switch state and will start monitoring on next packet or remaining samples
                    // Let's process remaining samples in this packet as MONITOR to avoid gaps
                    size_t remaining = count - 1 - i;
                    if (remaining > 0) {
                         // Recursive call or just fall through? 
                         // Simpler to just return and let next packet handle it, 
                         // or loop here? Let's just break and drop a few samples, it's safer.
                    }
                    return; 
                }
            }
            break;
        }

        case STATE_MONITOR: {
            double threshold_high = ctx.baseline_mean + TRIGGER_SIGMA * ctx.baseline_std_dev;
            double threshold_low = ctx.baseline_mean - TRIGGER_SIGMA * ctx.baseline_std_dev;

            for (size_t i = 0; i < count; ++i) {
                if (samples[i] > threshold_high || samples[i] < threshold_low) {
                    ctx.trigger_consecutive++;
                    
                    if (ctx.trigger_consecutive >= TRIGGER_CONSECUTIVE_SAMPLES) {
                        // Triggered!
                        ctx.state = STATE_CAPTURE;
                        ctx.capture_idx = 0;
                        printf("Triggered! Value=%u (Mean=%.2f, Sigma=%.2f)\n", samples[i], ctx.baseline_mean, ctx.baseline_std_dev);

                        // Start capturing from the current sample (or potentially 'i - consecutive + 1' if we want to capture the spike start)
                        // Let's capture from 'i' for now.
                        
                        size_t remaining = count - i;
                        size_t to_copy = (remaining > CAPTURE_BUFFER_SIZE) ? CAPTURE_BUFFER_SIZE : remaining;
                        memcpy(&ctx.capture_buf[ctx.capture_idx], &samples[i], to_copy * sizeof(uint16_t));
                        ctx.capture_idx += to_copy;
                        
                        if (ctx.capture_idx >= CAPTURE_BUFFER_SIZE) {
                            ctx.state = STATE_REPORT;
                        }
                        return; // Done with this packet
                    }
                } else {
                    ctx.trigger_consecutive = 0;
                }
            }
            break;
        }

        case STATE_CAPTURE: {
            size_t space_left = CAPTURE_BUFFER_SIZE - ctx.capture_idx;
            size_t to_copy = (count > space_left) ? space_left : count;
            
            memcpy(&ctx.capture_buf[ctx.capture_idx], samples, to_copy * sizeof(uint16_t));
            ctx.capture_idx += to_copy;

            if (ctx.capture_idx >= CAPTURE_BUFFER_SIZE) {
                ctx.state = STATE_REPORT;
            }
            break;
        }

        case STATE_REPORT:
            break;
    }
}

static void process_frame(const uint8_t *data, size_t len) {
    if (len < SEQ_BYTES + 1) return;

    const size_t payload_len = len - 2; // minus CRC
    // Verify CRC
    const uint16_t crc_calc = crc16_ccitt(data, payload_len);
    const uint16_t crc_rx = ((uint16_t)data[payload_len] << 8) | data[payload_len + 1];
    
    if (crc_calc != crc_rx) {
        ctx.frames_crc_fail++;
        return;
    }
    ctx.frames_ok++;

    // Unpack samples
    uint8_t sample_count = data[SEQ_BYTES];
    if (sample_count == 0) return;

    // We need a temporary buffer for unpacked samples from this frame
    uint16_t frame_samples[256]; // Max samples per frame is small (40), 256 is plenty safe
    size_t unpacked_count = 0;
    size_t offset = SEQ_BYTES + 1;

    for (uint8_t i = 0; i < sample_count; i += 2) {
        if (offset + 2 > payload_len) break; // Should not happen if len check passed, but safety

        uint8_t b0 = data[offset++];
        uint8_t b1 = (i + 1 < sample_count) ? data[offset++] : 0;
        uint8_t b2 = (i + 1 < sample_count) ? data[offset++] : 0;
        
        uint16_t s0 = (uint16_t)b0 | ((uint16_t)(b1 & 0x0F) << 8);
        frame_samples[unpacked_count++] = s0;
        
        if (i + 1 < sample_count) {
            uint16_t s1 = (uint16_t)((b1 & 0xF0) >> 4) | ((uint16_t)b2 << 4);
            frame_samples[unpacked_count++] = s1;
        }
    }

    process_samples(frame_samples, unpacked_count);
}

int main(void) {
    stdio_init_all();
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    uart_init(uart0, UART_BAUD);
    uart_set_format(uart0, 8, 2, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_inover(RX_PIN, GPIO_OVERRIDE_INVERT);

    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);

    sleep_ms(2000);
    printf("RX Triggered Mode Ready. Waiting for baseline...\n");

    uint8_t frame_buf[MAX_FRAME];
    size_t frame_len = 0;
    bool esc = false;
    bool overflow = false;
    uint64_t last_led_ms = to_ms_since_boot(get_absolute_time());

    ctx.state = STATE_WAIT_FOR_BASELINE;

    while (true) {
        // Process Ring Buffer
        while (rx_head != rx_tail) {
            uint8_t b = rx_buffer[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

            if (overflow) {
                if (b == SLIP_END) {
                    frame_len = 0;
                    esc = false;
                    overflow = false;
                }
                continue;
            }

            if (b == SLIP_END) {
                if (frame_len > 0) {
                    process_frame(frame_buf, frame_len);
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

        // Handle Reporting
        if (ctx.state == STATE_REPORT) {
            double mean, std;
            uint16_t min_v = 0xFFFF;
            uint16_t max_v = 0;
            
            calc_stats(ctx.capture_buf, CAPTURE_BUFFER_SIZE, &mean, &std);
            
            for(int i=0; i<CAPTURE_BUFFER_SIZE; i++) {
                if(ctx.capture_buf[i] < min_v) min_v = ctx.capture_buf[i];
                if(ctx.capture_buf[i] > max_v) max_v = ctx.capture_buf[i];
            }

            printf("\n--- TRIGGERED EVENT ---\n");
            printf("Stats: Min=%u, Max=%u, Mean=%.2f\n", min_v, max_v, mean);
            printf("First 10 raw values:\n");
            for (int i = 0; i < 10 && i < CAPTURE_BUFFER_SIZE; i++) {
                printf("%u\n", ctx.capture_buf[i]);
            }
            printf("-----------------------\n");

            // Reset to monitor
            ctx.state = STATE_MONITOR;
        }

        // Heartbeat Blink
        uint64_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_led_ms >= 500) {
            last_led_ms = now_ms;
            gpio_xor_mask(1u << LED_PIN);
        }
    }
}

