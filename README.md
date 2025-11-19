# Isolated Current Measurement Link (RP2040 Zero → HCPL2630 → Pico)

This is a development project that will enable the measurment of the current (and potentiall later differential voltage) of a high voltage high current pulse generator (Medical Purposes, pulsed electric fields) and safely transfer the data to a computer for analysis. The project is based on the RP2040 Zero and the Raspberry Pi Pico but the pico is just a placeholder for the rpi 4b that will be the final recipient. Goal is to have a compact and reliable solution that can transfer the data during a pulse quickly with at least 100ksps to the computer. Since the comptuer gives the "go" signal for the pulses and duplex would be more difficult to implement this is unidirectional so the data is basically streaming non stop and the recipient will have to decide when to read it or not. Until this is rock solid and proven to be reliable it will be a development project with a sacrifical rpi pico as recipient and a sacrifical rp2040 zero as transmitter. 

## Overview
- TX (RP2040 Zero) reads an ACS712 current sensor on ADC0 (GPIO26), low-passes it with 9 kΩ / 1 nF, samples at a configured rate, and streams samples over UART0 through a HCPL-2630 isolator.
- RX (Raspberry Pi Pico) receives on UART0 (inverted), SLIP-decodes frames, CRC-checks, tracks sequence gaps, and prints stats plus mean/min/max of the last good packet over USB-CDC (115200 baud).
- Payload carries 12-bit packed samples (3 bytes per 2 samples); CRC16-CCITT protects each frame.
- Note on rate: 1 Mbps at 8N2 tops out around ~45 k samples/s for 16-bit data once framing is included. The firmware currently targets `SAMPLE_RATE_HZ=40000` to stay within link budget; increase only if you also raise link speed or relax stop bits.

## Firmware layout
- `tx/`: RP2040 Zero transmitter. USB CDC for logs only (UART stdio disabled). Samples ADC0 at `SAMPLE_RATE_HZ`, batches `SAMPLES_PER_FRAME` (default 40), frames them using 12-bit packing (2 samples in 3 bytes), and sends via UART0 TX on GP0 at 1,000,000 baud, 8N2. Source: `tx/src/main.c`.
- `rx/`: Pico receiver. UART0 RX on GP1 (line inversion enabled in hardware to counter the HCPL2630), SLIP decode + CRC16 check. Uses an **interrupt-driven UART with a large ring buffer** to prevent data loss. Tracks seq gaps, and reports totals plus per-packet mean/min/max for the last valid frame. Source: `rx/src/main.c`.
- PlatformIO uses the PicoSDK platform (`platform = https://github.com/maxgerhardt/platform-raspberrypi.git`, `PICO_STDIO_USB=1`, `PICO_STDIO_UART=0`).

## Protocol (SLIP + CRC16 + sequence)
- Physical: UART0, TX=GP0, RX=GP1, 1,000,000 baud, 8 data bits, 2 stop bits, no parity. RX input is inverted (`gpio_set_inover`) because the HCPL-2630 inverts the signal.
- Framing: SLIP (0xC0 END, 0xDB ESC with 0xDC/0xDD substitutions). Each frame = `0xC0 | (payload+CRC escaped) | 0xC0`.
- CRC: CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection) computed over the payload bytes, appended big-endian `[CRC_hi, CRC_lo]` before SLIP encoding.
- Payload layout (little-endian sequence):
  - Bytes 0..3: `seq` (uint32_t, increments each frame, wraps on overflow). RX counts missed frames from gaps.
  - Byte 4: `count` (uint8_t number of samples in this frame).
  - Bytes 5..N: Packed 12-bit samples. Every 2 samples occupy 3 bytes.
    - **Byte 0**: Sample A [7:0]
    - **Byte 1**: Sample A [11:8] (lower nibble) | Sample B [3:0] (upper nibble)
    - **Byte 2**: Sample B [11:4]
    - If `count` is odd, the last sample uses 2 bytes (padded).

## Hardware wiring
### Power & isolation domains
- Both boards are USB-powered from a common hub. Each uses its own 3.3 V regulator.
- HCPL-2630 “output side” is powered from the RX Pico 3.3 V. “Input side” shares ground/supply with the TX RP2040 Zero.
- Shielded cable recommended between HCPL output and RX, shield grounded at RX only.

### Current sensor frontend (TX side, RP2040 Zero)
- ACS712 (5 V module):
  - VCC → 5 V (USB VBUS or 5 V rail on the Zero).
  - GND → TX ground. Add 100 nF + 1 µF decoupling at the module.
  - OUT → ADC0 (GPIO26) through 9 kΩ series resistor.
  - 1 nF from ADC0 to GND (after the 9 kΩ) for RC anti-alias filter.
  - 15 kohm from ADC to GND as voltage divider as the output of the ACS712 is 5V peak otherwise.
  - Optional: Schottky clamps to 3.3 V and GND at ADC pin for over/under-voltage protection.

### UART / isolation path
- TX (RP2040 Zero):
  - GP0 (UART0 TX) → 220 Ω → HCPL2630 pin 1 (anode).
  - TX GND → HCPL2630 pin 2 (cathode).
- RX (Raspberry Pi Pico):
  - HCPL2630 pin 8 → Pico 3.3 V; pin 5 → Pico GND; 100 nF across pins 8–5.
  - Pull-up 550 Ω from pin 8 to pin 7.
  - HCPL2630 pin 7 (open-collector out) → Pico GP1 (UART0 RX, inverted in software).
  - Pico GP0 (UART0 TX) unused but kept set to UART function.

## Flashing
- BOOTSEL mount: `/media/nric/RPI-RP2`.
- TX flash: `cd /home/nric/src/current_hcpl_2/tx && PATH="$HOME/.platformio/penv/bin:$PATH" pio run -t upload --upload-port /media/nric/RPI-RP2`
- RX flash: `cd /home/nric/src/current_hcpl_2/rx && PATH="$HOME/.platformio/penv/bin:$PATH" pio run -t upload --upload-port /media/nric/RPI-RP2`
- RX USB CDC: 115200 baud for logs.

## What to expect on RX USB console
- Periodic stats every 2 s: `ok`, `crc_fail`, `too_short`, `too_long`, `bad_len`, `timeout`, `bytes`, `missed_frames`, `seq_resets`.
- Hardware/Buffer errors: `overrun`, `break`, `parity`, `framing`, `ring_buffer_overflow`.
- Last good packet: `last_ok seq=<n> samples=<cnt> len=<payload_bytes> mean=<adc> min=<adc> max=<adc>`.
- If a CRC fails, prints last failing seq/id/len/CRCs plus a short hex dump.

## Tuning knobs
- `tx/src/main.c`: `SAMPLE_RATE_HZ` (default 40 kHz) and `SAMPLES_PER_FRAME` (default 40) balance capture vs. link throughput. With 1 Mbps 8N2 and SLIP, ~40 k samples/s of 16-bit data is practical; sending full 100 kS/s raw would exceed link capacity unless baud/stop bits change or data is compressed.
