#include <pico/stdlib.h>
#include <stdio.h>

// Minimal RX with inversion toggle: UART0 RX on GP1, expect continuous 0xAA at 1000 baud.
// Counts bytes per 1s window, prints histogram, and flips inversion each window to diagnose polarity.
// LED on GPIO25 blinks every second.

#define RX_PIN 1
#define TX_PIN 0  // unused
#define LED_PIN 25
#define BAUD 1000
#define EXPECT_BYTE 0xAA

static void set_invert(bool inv) {
    gpio_set_inover(RX_PIN, inv ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
}

int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    uart_init(uart0, BAUD);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);

    sleep_ms(2000);  // allow USB CDC to enumerate

    bool invert = true;  // start inverted for HCPL path
    set_invert(invert);
    printf("RX boot: GP1 @ %u baud, expect 0x%02X, invert=%s (toggles each second)\n",
           BAUD, EXPECT_BYTE, invert ? "on" : "off");

    uint64_t last_report = to_ms_since_boot(get_absolute_time());
    uint64_t last_led = last_report;
    uint32_t total = 0;
    uint32_t good = 0;
    uint32_t count_aa = 0, count_55 = 0, count_00 = 0, count_ff = 0, count_other = 0;

    while (true) {
        while (uart_is_readable(uart0)) {
            uint8_t b = (uint8_t)uart_getc(uart0);
            total++;
            if (b == EXPECT_BYTE) good++;
            if (b == 0xAA) count_aa++;
            else if (b == 0x55) count_55++;
            else if (b == 0x00) count_00++;
            else if (b == 0xFF) count_ff++;
            else count_other++;
        }
        const uint64_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_led >= 1000) {
            last_led = now;
            gpio_xor_mask(1u << LED_PIN);
        }
        if (now - last_report >= 1000) {
            last_report = now;
            printf("RX stats inv=%s total=%lu good=%lu bad=%lu aa=%lu 55=%lu 00=%lu ff=%lu other=%lu\n",
                   invert ? "on" : "off",
                   (unsigned long)total, (unsigned long)good,
                   (unsigned long)(total - good),
                   (unsigned long)count_aa, (unsigned long)count_55,
                   (unsigned long)count_00, (unsigned long)count_ff,
                   (unsigned long)count_other);
            total = good = count_aa = count_55 = count_00 = count_ff = count_other = 0;
            invert = !invert;
            set_invert(invert);
        }
    }
}
