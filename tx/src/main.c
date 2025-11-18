#include <pico/stdlib.h>
#include <stdio.h>
#include <pico/stdio_usb.h>

// TX pattern generator: bit-bangs GP0 at 10,000 baud with asymmetric pattern
// 1111001111001010101010 then 1 ms pause, repeat. USB prints low-rate alive.

#define TX_PIN 0
#define BAUD 10000u
#define BIT_US (1000000u / BAUD)  // 100 us per bit
#define PAUSE_US 1000u            // 1 ms pause after pattern
#define ALIVE_PERIOD_MS 1000

static const uint8_t pattern_bits[] = {
    1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0,
};
static const size_t pattern_len = sizeof(pattern_bits) / sizeof(pattern_bits[0]);

static bool alive_timer_cb(repeating_timer_t *t) {
    if (stdio_usb_connected()) {
        printf("TX alive: pattern 1111001111001010101010 @ %u baud, pause %u us\n", BAUD, PAUSE_US);
    }
    return true;  // keep repeating
}

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);  // keep USB prints non-blocking/buffer-free

    gpio_init(TX_PIN);
    gpio_set_dir(TX_PIN, GPIO_OUT);
    gpio_put(TX_PIN, 0);

    sleep_ms(2000);  // allow USB to enumerate before we start

    repeating_timer_t alive_timer;
    add_repeating_timer_ms(ALIVE_PERIOD_MS, alive_timer_cb, NULL, &alive_timer);

    absolute_time_t next = get_absolute_time();
    while (true) {
        for (size_t i = 0; i < pattern_len; ++i) {
            gpio_put(TX_PIN, pattern_bits[i]);
            next = delayed_by_us(next, BIT_US);
            sleep_until(next);
        }

        gpio_put(TX_PIN, 0);  // hold low for the pause window
        next = delayed_by_us(next, PAUSE_US);
        sleep_until(next);
    }
}
