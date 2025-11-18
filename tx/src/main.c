#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#include <pico/stdio_uart.h>
#include <hardware/adc.h>
#include <hardware/dma.h>
#include <stdio.h>
#include <string.h>

// Unidirectional framed TX at 2500000 baud using SLIP-style framing + CRC16-CCITT.
// Frame: SLIP(CRC16(payload) appended big-endian). Delimiter is 0xC0 start/end.
// Captures ADC (ACS712 current sense) at 100 ksps into a ring buffer (DMA),
// streams packed 12-bit data frames with sequence counters, and prints a 1 s mean.

#define UART_TX_PIN 0
#define UART_RX_PIN 1  // unused
#define UART_BAUD 2500000

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define MAX_PAYLOAD 1024
#define MAX_FRAME_ENCODED ((MAX_PAYLOAD + 2) * 2 + 2)  // worst case escapes + END delimiters

// Test payload identifiers
#define TEST_SHORT_ID 0xA1
#define TEST_LONG_ID 0xB2
#define SEQ_BYTES 4
#define TEST_SHORT_LEN 6             // ID + "SHORT"
#define TEST_LONG_LEN 64
#define TEST_SHORT_TOTAL (SEQ_BYTES + TEST_SHORT_LEN)
#define TEST_LONG_TOTAL (SEQ_BYTES + TEST_LONG_LEN)
#define DATA_ID 0xD0

// ADC capture
#define ADC_CHANNEL 0                  // GPIO26 / ADC0
#define ADC_RING_SAMPLES 32768         // 32K samples -> 64 KB (fits under 70% RAM)
#define ADC_RING_MASK (ADC_RING_SAMPLES - 1)
#define ADC_DECIM 1                    // no decimation (full rate)
#define ADC_PACKET_SAMPLES 400         // samples per data frame (raw 12-bit, packed 2->3 bytes)
#define ADC_TARGET_KSPS 100.0f

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

static size_t slip_encode(const uint8_t *in, size_t len, uint8_t *out, size_t max_out) {
    size_t o = 0;
    if (o < max_out) out[o++] = SLIP_END;  // leading END to clear the line
    for (size_t i = 0; i < len && o < max_out; ++i) {
        uint8_t b = in[i];
        if (b == SLIP_END) {
            if (o + 2 > max_out) break;
            out[o++] = SLIP_ESC;
            out[o++] = SLIP_ESC_END;
        } else if (b == SLIP_ESC) {
            if (o + 2 > max_out) break;
            out[o++] = SLIP_ESC;
            out[o++] = SLIP_ESC_ESC;
        } else {
            out[o++] = b;
        }
    }
    if (o < max_out) out[o++] = SLIP_END;
    return o;
}

static void send_frame(const uint8_t *payload, size_t len) {
    uint8_t buf[MAX_PAYLOAD + 2];
    if (len > MAX_PAYLOAD) return;
    memcpy(buf, payload, len);
    const uint16_t crc = crc16_ccitt(payload, len);
    buf[len] = (uint8_t)(crc >> 8);
    buf[len + 1] = (uint8_t)(crc & 0xFF);

    uint8_t encoded[MAX_FRAME_ENCODED];
    const size_t encoded_len = slip_encode(buf, len + 2, encoded, sizeof(encoded));
    uart_write_blocking(uart0, encoded, encoded_len);
}

static void build_short_payload(uint8_t *buf, uint32_t seq) {
    buf[0] = (uint8_t)(seq & 0xFF);
    buf[1] = (uint8_t)((seq >> 8) & 0xFF);
    buf[2] = (uint8_t)((seq >> 16) & 0xFF);
    buf[3] = (uint8_t)((seq >> 24) & 0xFF);
    buf[4] = TEST_SHORT_ID;
    const uint8_t text[TEST_SHORT_LEN - 1] = {'S', 'H', 'O', 'R', 'T'};
    memcpy(&buf[5], text, sizeof(text));
}

static void build_long_payload(uint8_t *buf, uint32_t seq) {
    buf[0] = (uint8_t)(seq & 0xFF);
    buf[1] = (uint8_t)((seq >> 8) & 0xFF);
    buf[2] = (uint8_t)((seq >> 16) & 0xFF);
    buf[3] = (uint8_t)((seq >> 24) & 0xFF);
    buf[4] = TEST_LONG_ID;
    for (size_t i = 5; i < TEST_LONG_TOTAL; ++i) {
        buf[i] = (uint8_t)((i - 4) & 0xFF);
    }
}

static inline size_t pack_samples12(const uint16_t *samples, size_t count, uint8_t *out, size_t max_out) {
    size_t o = 0;
    size_t i = 0;
    while (i + 1 < count && o + 3 <= max_out) {
        uint16_t s0 = samples[i++] & 0x0FFFu;
        uint16_t s1 = samples[i++] & 0x0FFFu;
        out[o++] = (uint8_t)(s0 & 0xFF);
        out[o++] = (uint8_t)(((s0 >> 8) & 0x0F) | ((s1 & 0x0F) << 4));
        out[o++] = (uint8_t)((s1 >> 4) & 0xFF);
    }
    if (i < count && o + 2 <= max_out) {
        uint16_t s0 = samples[i] & 0x0FFFu;
        out[o++] = (uint8_t)(s0 & 0xFF);
        out[o++] = (uint8_t)((s0 >> 8) & 0x0F);
    }
    return o;
}

static size_t build_data_payload(uint8_t *buf, uint32_t seq, const uint16_t *samples, size_t count) {
    buf[0] = (uint8_t)(seq & 0xFF);
    buf[1] = (uint8_t)((seq >> 8) & 0xFF);
    buf[2] = (uint8_t)((seq >> 16) & 0xFF);
    buf[3] = (uint8_t)((seq >> 24) & 0xFF);
    buf[4] = DATA_ID;
    size_t packed = pack_samples12(samples, count, &buf[5], MAX_PAYLOAD - 5);
    return 5 + packed;
}

// ADC/DMA ring buffer
static uint16_t adc_ring[ADC_RING_SAMPLES];
static volatile uint32_t adc_dma_write_idx = 0;
static int adc_dma_chan = -1;

static void adc_dma_setup(void) {
    adc_gpio_init(26 + ADC_CHANNEL);
    adc_init();
    adc_select_input(ADC_CHANNEL);
    // Target ~100 ksps. ADC clock ~48 MHz; divider of 4.8 gives ~10 MHz -> ~104 ksps.
    adc_set_clkdiv(4.8f);

    adc_fifo_setup(true,    // enable
                   true,    // enable DMA data request
                   1,       // DREQ at least 1 sample
                   false,   // no ERR bit
                   false);  // no shift, keep full 12-bit in 16-bit word

    adc_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(adc_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);
    // Ring on write address (bytes). 2 bytes per sample -> ring size bits = log2(ADC_RING_SAMPLES*2) = 16.
    channel_config_set_ring(&c, true, 16);

    dma_channel_configure(adc_dma_chan, &c,
                          adc_ring,      // write addr
                          &adc_hw->fifo, // read addr
                          0xFFFFFFFF,    // transfer count (continuous)
                          false);

    dma_hw->ch[adc_dma_chan].al2_transfer_count = 0xFFFFFFFF;
    dma_channel_start(adc_dma_chan);
    adc_run(true);
}

int main(void) {
    stdio_usb_init();                     // USB CDC only
    stdio_set_driver_enabled(&stdio_uart, false);  // hard-disable UART stdio
    setvbuf(stdout, NULL, _IONBF, 0);

    uart_init(uart0, UART_BAUD);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);  // higher throughput
    uart_set_fifo_enabled(uart0, true);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    sleep_ms(2000);  // allow USB enumerate
    adc_dma_setup();

    uint32_t seq = 0;
    uint8_t short_payload[TEST_SHORT_TOTAL];
    uint8_t long_payload[TEST_LONG_TOTAL];
    uint8_t data_payload[5 + ADC_PACKET_SAMPLES * 2];
    uint16_t sample_buf[ADC_PACKET_SAMPLES];
    size_t sample_buf_len = 0;
    uint32_t data_seq = 0;
    uint64_t last_mean_ms = to_ms_since_boot(get_absolute_time());
    uint64_t last_tx_test_ms = last_mean_ms;
    uint64_t last_data_tx_ms = last_mean_ms;
    uint32_t mean_accum = 0;
    uint32_t mean_count = 0;
    uint32_t decim_counter = 0;
    uint32_t consume_idx = 0;

    while (true) {
        // Drain ADC ring
        uint32_t dma_write_bytes = dma_hw->ch[adc_dma_chan].write_addr - (uint32_t)adc_ring;
        uint32_t write_idx = (dma_write_bytes >> 1) & ADC_RING_MASK;
        while (consume_idx != write_idx) {
            uint16_t sample = adc_ring[consume_idx];
            consume_idx = (consume_idx + 1) & ADC_RING_MASK;
            mean_accum += sample;
            mean_count++;
            decim_counter++;
            if ((decim_counter % ADC_DECIM) == 0 && sample_buf_len < ADC_PACKET_SAMPLES) {
                sample_buf[sample_buf_len++] = sample;  // full 12-bit raw
            }
            if (sample_buf_len >= ADC_PACKET_SAMPLES) {
                size_t payload_len = build_data_payload(data_payload, data_seq++, sample_buf, sample_buf_len);
                send_frame(data_payload, payload_len);
                sample_buf_len = 0;
            }
        }

        uint64_t now_ms = to_ms_since_boot(get_absolute_time());

        // Periodic data flush to keep latency bounded
        if (sample_buf_len > 0 && (now_ms - last_data_tx_ms) >= 20) {
            size_t payload_len = build_data_payload(data_payload, data_seq++, sample_buf, sample_buf_len);
            send_frame(data_payload, payload_len);
            sample_buf_len = 0;
            last_data_tx_ms = now_ms;
        }

        // Periodic test frames (keep-alive for protocol sanity)
        if (now_ms - last_tx_test_ms >= 500) {
            build_short_payload(short_payload, seq++);
            send_frame(short_payload, sizeof(short_payload));
            build_long_payload(long_payload, seq++);
            send_frame(long_payload, sizeof(long_payload));
            last_tx_test_ms = now_ms;
        }

        // 1s mean print (USB only)
        if (now_ms - last_mean_ms >= 1000) {
            uint32_t count = mean_count ? mean_count : 1;
            printf("ADC mean=%lu (raw 12-bit), samples=%lu, data_seq=%lu\n",
                   (unsigned long)(mean_accum / count), (unsigned long)mean_count, (unsigned long)data_seq);
            mean_accum = 0;
            mean_count = 0;
            last_mean_ms = now_ms;
        }

        tight_loop_contents();
    }
}
