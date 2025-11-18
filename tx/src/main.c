#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#include <pico/stdio_uart.h>
#include <hardware/adc.h>
#include <stdio.h>
#include <string.h>

// Unidirectional framed TX at 1000000 baud using SLIP-style framing + CRC16-CCITT.
// Frame: SLIP(CRC16(payload) appended big-endian). Delimiter is 0xC0 start/end.
// Streams ADC samples (16-bit, ADC is 12-bit) captured from the current-sense input.

#define UART_TX_PIN 0
#define UART_RX_PIN 1  // unused
#define UART_BAUD 1000000

#define ADC_GPIO 26  // ADC0 on RP2040
#define SAMPLE_RATE_HZ 40000  // chosen to fit 1 Mbps 8N2 throughput with framing
#define SAMPLES_PER_FRAME 40

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define MAX_PAYLOAD 256
#define MAX_FRAME_ENCODED ((MAX_PAYLOAD + 2) * 2 + 2)  // worst case escapes + END delimiters
#define SEQ_BYTES 4

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

static size_t build_samples_payload(uint8_t *buf, uint32_t seq, const uint16_t *samples, size_t count) {
    if (count > 255) count = 255;
    const size_t payload_len = SEQ_BYTES + 1 + (count * 2);
    if (payload_len > MAX_PAYLOAD) return 0;
    buf[0] = (uint8_t)(seq & 0xFF);
    buf[1] = (uint8_t)((seq >> 8) & 0xFF);
    buf[2] = (uint8_t)((seq >> 16) & 0xFF);
    buf[3] = (uint8_t)((seq >> 24) & 0xFF);
    buf[4] = (uint8_t)count;
    for (size_t i = 0; i < count; ++i) {
        buf[5 + i * 2] = (uint8_t)(samples[i] & 0xFF);
        buf[5 + i * 2 + 1] = (uint8_t)(samples[i] >> 8);
    }
    return payload_len;
}

int main(void) {
    stdio_usb_init();                     // USB CDC only
    stdio_set_driver_enabled(&stdio_uart, false);  // hard-disable UART stdio
    setvbuf(stdout, NULL, _IONBF, 0);

    adc_init();
    adc_gpio_init(ADC_GPIO);
    adc_select_input(0);  // ADC0 = GPIO26

    uart_init(uart0, UART_BAUD);
    uart_set_format(uart0, 8, 2, UART_PARITY_NONE);  // add margin with 2 stop bits
    uart_set_fifo_enabled(uart0, true);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    sleep_ms(2000);  // allow USB enumerate

    uint32_t seq = 0;
    uint16_t samples[SAMPLES_PER_FRAME];
    uint8_t payload[MAX_PAYLOAD];
    const uint32_t sample_interval_us = 1000000u / SAMPLE_RATE_HZ;
    absolute_time_t next_sample = get_absolute_time();

    while (true) {
        for (size_t i = 0; i < SAMPLES_PER_FRAME; ++i) {
            next_sample = delayed_by_us(next_sample, sample_interval_us);
            sleep_until(next_sample);
            samples[i] = adc_read();
        }

        const size_t payload_len = build_samples_payload(payload, seq++, samples, SAMPLES_PER_FRAME);
        if (payload_len > 0) {
            send_frame(payload, payload_len);
        }
    }
}
