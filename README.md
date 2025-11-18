# Current HCPL link test (PicoSDK)

## Code layout (current test)
- `tx/` – RP2040 Zero transmitter, PicoSDK, USB CDC logging only (`stdio_init_all`). UART0 TX on GP0 at 100,000 baud, 8N2 (`tx/src/main.c`). Uses SLIP framing (0xC0 delimiter, 0xDB escapes) with CRC16-CCITT over the payload; frames are SLIP(payload + CRC16 big-endian). Test mode repeatedly sends a short and a long payload.
- `rx/` – Pico receiver, PicoSDK, USB CDC logging only. UART0 RX on GP1 at 100,000 baud, 8N2 (`rx/src/main.c`). Line inversion is enabled in hardware (`gpio_set_inover`) to compensate for the HCPL2630 inversion. SLIP decode + CRC16 check; prints stats for good/failed frames, counts for the built-in short/long test payloads, and reports the last CRC-failed frame bytes/CRC along with bit-flip sum vs expected.
- PlatformIO configs use the PicoSDK platform (`platform = https://github.com/maxgerhardt/platform-raspberrypi.git`, `PICO_STDIO_USB=1`, `PICO_STDIO_UART=0`).

## Protocol (unidirectional)
- Physical: UART0, TX=GP0, RX=GP1, 100,000 baud, 8N2. RX input is inverted in hardware to counter the HCPL2630 inversion.
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
