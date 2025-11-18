#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#include <pico/stdio_uart.h>
#include <stdio.h>
#include <string.h>

// Unidirectional framed TX at 1000000 baud using SLIP-style framing + CRC16-CCITT.
// Frame: SLIP(CRC16(payload) appended big-endian). Delimiter is 0xC0 start/end.
// Test mode repeatedly sends a short and long payload to exercise RX.

#define UART_TX_PIN 0
#define UART_RX_PIN 1  // unused
#define UART_BAUD 1000000

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define MAX_PAYLOAD 256
#define MAX_FRAME_ENCODED ((MAX_PAYLOAD + 2) * 2 + 2)  // worst case escapes + END delimiters

// Test payload identifiers
#define TEST_SHORT_ID 0xA1
#define TEST_LONG_ID 0xB2
#define SEQ_BYTES 4
#define TEST_SHORT_LEN 6             // ID + "SHORT"
#define TEST_LONG_LEN 64
#define TEST_SHORT_TOTAL (SEQ_BYTES + TEST_SHORT_LEN)
#define TEST_LONG_TOTAL (SEQ_BYTES + TEST_LONG_LEN)

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

int main(void) {
    stdio_usb_init();                     // USB CDC only
    stdio_set_driver_enabled(&stdio_uart, false);  // hard-disable UART stdio
    setvbuf(stdout, NULL, _IONBF, 0);

    uart_init(uart0, UART_BAUD);
    uart_set_format(uart0, 8, 2, UART_PARITY_NONE);  // add margin with 2 stop bits
    uart_set_fifo_enabled(uart0, true);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    sleep_ms(2000);  // allow USB enumerate

    uint32_t seq = 0;
    uint8_t short_payload[TEST_SHORT_TOTAL];
    uint8_t long_payload[TEST_LONG_TOTAL];

    while (true) {
        build_short_payload(short_payload, seq++);
        send_frame(short_payload, sizeof(short_payload));
        sleep_ms(500);

        build_long_payload(long_payload, seq++);
        send_frame(long_payload, sizeof(long_payload));
        sleep_ms(1500);  // keep below 15s idle requirement
    }
}
