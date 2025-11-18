# Current HCPL link test (PicoSDK)

- `tx/` – RP2040 Zero transmitter, PicoSDK, USB CDC logging only (`stdio_init_all`). UART0 TX on GP0 at 2,500,000 baud, 8N1 (`tx/src/main.c`). Uses SLIP framing (0xC0 delimiter, 0xDB escapes) with CRC16-CCITT over the payload; frames are SLIP(payload + CRC16 big-endian). Each payload starts with a 32-bit sequence counter (little-endian) followed by an ID (short=0xA1, long=0xB2, data=0xD0) and payload bytes. Test mode sends a short and a long payload, auto-incrementing the sequence; data frames carry raw ADC samples.
- `rx/` – Pico receiver, PicoSDK, USB CDC logging only. UART0 RX on GP1 at 2,500,000 baud, 8N1 (`rx/src/main.c`). Line inversion is enabled in hardware (`gpio_set_inover`) to compensate for the HCPL2630 inversion. SLIP decode + CRC16 check; prints stats for good/failed frames, counts for the built-in short/long test payloads, reports the last CRC-failed frame bytes/CRC, tracks missed frames via the sequence counter, and aggregates stats for data frames (count, samples, mean/min/max).
- PlatformIO configs use the PicoSDK platform (`platform = https://github.com/maxgerhardt/platform-raspberrypi.git`, `PICO_STDIO_USB=1`, `PICO_STDIO_UART=0`).

## Protocol (unidirectional, SLIP + CRC16 + sequence)
- Physical: UART0, TX=GP0, RX=GP1, 2,500,000 baud, 8 data bits, 1 stop bit, no parity (8N1). RX input is inverted in hardware to counter the HCPL2630 inversion (HCPL2630 inverts the signal).
- Framing: SLIP (RFC1055-style). 0xC0 = END delimiter. 0xDB = ESC, with substitutions 0xDC (ESC_END) for literal 0xC0 and 0xDD (ESC_ESC) for literal 0xDB. Each frame is: `0xC0 | (payload+CRC escaped) | 0xC0`. Idle gaps of any length are allowed. If an END is seen mid-frame, the frame closes and CRC is checked.
- Payload layout (little-endian sequence):
  - Bytes 0..3: `seq` (uint32_t, increments each frame, wraps on overflow). RX tracks missed frames from gaps in `seq`.
  - Byte 4: `id` (0xA1 = short test, 0xB2 = long test, 0xD0 = data).
  - Bytes 5..N: payload data.
- Integrity: CRC16-CCITT (poly 0x1021, init 0xFFFF, reflect=false) computed over the SLIP-unescaped payload bytes (including `seq` and `id`), appended big-endian `[CRC_hi, CRC_lo]` before SLIP-encoding.
- Test patterns used here:
  - Short: `seq` (4 bytes) + 0xA1 + ASCII "SHORT" (5 bytes). Total = 10 bytes before CRC.
  - Long: `seq` (4 bytes) + 0xB2 + ascending bytes 0x01..0x3C. Total = 69 bytes before CRC.
- Data frame: `seq` (4 bytes) + 0xD0 + raw ADC samples, packed 12-bit (two samples per 3 bytes; a final single sample uses 2 bytes). TX uses packets of 400 samples by default (~605 bytes before CRC); frames are emitted frequently (≤20 ms) to keep up with ~100 ksps input.
- Error reporting (current RX implementation): counts CRC failures, too-short/too-long/timeouts, cumulative bit-flip sum for known test frames, missed frames from sequence gaps, last failed frame dump (id/seq/len/CRCs + first bytes), and aggregates stats for data frames (count, total samples, mean/min/max).

## Acquisition and streaming (TX behavior)
- ADC capture: ADC0 @ ~100 ksps via DMA into a 32K-sample ring buffer (uint16_t). Ring size ≈ 64 KB (<70% of RP2040 RAM).
- No decimation: all samples kept. Data frames pack 12-bit raw samples (2 samples per 3 bytes; lone trailing sample uses 2 bytes). Current packet size: 400 samples → ~605 bytes before CRC; at 2.5 Mbaud 8N1 this fits with headroom.
- Data frames: `id=0xD0`, payload = `seq` (4 bytes) + `id` + packed samples. Frames are emitted frequently (≤20 ms) to keep up with ~100 ksps input. Short/long test frames still send periodically as keep-alives.
- USB diagnostics: once per second, TX prints the raw ADC mean and sample count; RX reports data frame counts, mean/min/max of received samples, CRC stats, and missed frames.
- Framing: SLIP (0xC0 END delimiter, 0xDB ESC with 0xDC/0xDD substitutions). Each frame starts/ends at 0xC0; idle gaps of any length are fine (timeout logic clears partial frames after 15 s of silence).
- Integrity: CRC16-CCITT (poly 0x1021, init 0xFFFF) computed over the payload, appended big-endian, and SLIP-encoded with the payload.
- Test payloads: short frame starts with byte 0xA1 and string "SHORT"; long frame starts with 0xB2 followed by incrementing bytes. TX sends both in a loop with ~0.5–1.5 s spacing; RX counts them in stats.

## Flashing
- BOOTSEL mount appears at `/media/nric/RPI-RP2`.
- TX flash: `cd /home/nric/src/current_hcpl_2/tx && PATH="$HOME/.platformio/penv/bin:$PATH" pio run -t upload --upload-port /media/nric/RPI-RP2`
- RX flash: `cd /home/nric/src/current_hcpl_2/rx && PATH="$HOME/.platformio/penv/bin:$PATH" pio run -t upload --upload-port /media/nric/RPI-RP2`
- USB CDC baud: 115200.

## Wiring (isolated via HCPL2630 as discussed)
- TX (RP2040 Zero):
  - GP0 (UART0 TX) → 220 Ω → HCPL2630 pin 1 (anode).
  - GND → HCPL2630 pin 2 (cathode).
- RX (Raspberry Pi Pico):
  - HCPL2630 pin 8 → Pico 3.3 V.
  - HCPL2630 pin 5 → Pico GND.
  - 100 nF across pins 8–5.
  - 550 Ω pull-up from pin 8 to pin 7.
  - HCPL2630 pin 7 (open-collector out) → Pico GP1 (note: silkscreen “1” on top corresponds to GP1).
  - Keep commons as above; USB hub supplies power/ground to both boards.

## Notes / current debugging status
- Scope shows clean 0xAA waveform at RX GP1 up to 1 Mbps.
- Minimal test: TX sends 0xAA @1 kbaud; RX counts per second and toggles inversion. Typical log: inv=on ~40 aa /93 total; inv=off ~14 aa /93 total → inversion helps, but many bytes still “other” (not 0x55/0x00/0xFF), indicating framing/timing issues on the isolated path.
- Action items: lock inversion on and dump actual bad byte values; try weaker pull-up on HCPL output (e.g., 1–2.2 kΩ vs 550 Ω); verify idle level on GP1; optionally drop baud to 300 to see if errors collapse.
- Observation: with the asymmetric TX test pattern (1111001111001010101010 @ 10 kbaud + 1 ms low pause) GP1 on the RX Pico sees the pattern inverted after the HCPL2630 (logic levels flip). Account for this when interpreting waveforms or set RX inversion accordingly.

## ACS712 current sense wiring to TX (ADC capture)
- Sensor: ACS712 module (5 A variant) wired to the TX RP2040 Zero.
- Power: ACS712 Vcc → TX 5 V; ACS712 GND → TX GND.
- Signal: ACS712 OUT → TX ADC0 (GPIO26). Optional RC filter (e.g., 1 kΩ + 10 nF) at the ADC pin to tame noise. Ensure the output never exceeds 3.3 V at the ADC (the ACS712 outputs ~Vcc/2 ± sensitivity, so stay within the safe current range or add clamping/divider as needed).
- Load: run the measured conductor through the ACS712 IP+ / IP– terminals in series with the DUT supply.
