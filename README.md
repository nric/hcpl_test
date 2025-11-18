# Current HCPL link test (PicoSDK)

## Code layout (current test)
- `tx/` – RP2040 Zero transmitter, PicoSDK, USB CDC logging only (`stdio_init_all`). UART0 TX on GP0. Current test firmware sends continuous `0xAA` at 1000 baud (`tx/src/main.c`), LED on GPIO25 blinks every 500 ms, USB prints “TX alive baud=1000”.
- `rx/` – Pico receiver, PicoSDK, USB CDC logging only. UART0 RX on GP1. Current test firmware counts bytes once per second, prints good/bad plus histogram (aa/55/00/ff/other), and toggles RX input inversion each second to diagnose polarity (`rx/src/main.c`). LED blinks every second.
- PlatformIO configs use the PicoSDK platform (`platform = https://github.com/maxgerhardt/platform-raspberrypi.git`, `PICO_STDIO_USB=1`, `PICO_STDIO_UART=0`).

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
