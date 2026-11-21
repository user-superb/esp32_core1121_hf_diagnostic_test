# Core1121-HF Single-Module Diagnostic Suite
PlatformIO diagnostic firmware for testing **Waveshare Core1121-HF** (Semtech **LR1121**) LoRa transceiver modules one at a time on a classic **ESP32 DevKit (`esp32dev`)** using **RadioLib 8.0.0**.

## What This Code Does
On boot, the firmware automatically runs an **8-step Pass/Fail hardware self-test** over Serial (`115200` baud) to verify every subsystem of a single Core1121-HF module:
1. **RESET & BUSY Handshake** — Resets the module and confirms the `BUSY` line pulses and settles LOW.
2. **SPI & LR1121 Initialization** — Initializes VSPI and configures Core1121-HF hardware settings (`3.0 V` TCXO, DC-DC regulator, PE4259 RF switch on `DIO5`/`DIO6`, High-Power Sub-GHz PA).
3. **Chip Identity & Unique EUI** — Confirms authentic `LR1121 (0x03)` silicon, firmware version, and prints the module's unique `ChipEUI` so multiple modules can be distinguished.
4. **Supply Voltage (`VBAT`) & Temperature** — Reads the internal voltage rail and die temperature sensor (using the Semtech LR1121 datasheet formula directly).
5. **Oscillators & Calibration** — Verifies both onboard crystals (`32 MHz` TCXO and `32.768 kHz` XTAL), runs full internal + Sub-GHz image calibration, and checks that LR1121 error flags are `0x0000`.
6. **Hardware TRNG** — Reads four 32-bit values from the chip's true random number generator.
7. **Sub-GHz RX Front-End & LoRa CAD** — Measures the Sub-GHz RSSI noise floor at `868 MHz` and `915 MHz` and runs a LoRa Channel Activity Detection check.
8. **`DIO9` Hardware Interrupt & Safe TX** — Sends a short `0 dBm` LoRa test packet on Sub-GHz and verifies that the `DIO9` hardware interrupt fires cleanly with zero errors.
After the self-test finishes, an **interactive Serial menu** lets you run additional checks without reflashing.

## Interactive Serial Commands (`115200` baud)
| Key | Action |
| :---: | :--- |
| `r` | Re-run the full 8-step diagnostic suite |
| `s` | Sub-GHz RSSI spectrum sweep (`850–930 MHz`, 2 MHz step, RX-only) |
| `w` | `2.4 GHz` **RX-only** RSSI sweep (`2400–2484 MHz`, 4 MHz step — never transmits) |
| `t` | Send one Sub-GHz LoRa test packet (`0 dBm`) and verify the `DIO9` IRQ |
| `l` | Continuous Sub-GHz LoRa RX listen mode (send any key to stop) |
| `h` | Print the command menu |

## Wiring (`esp32dev` ↔ Core1121-HF)
| Core1121-HF Pin | ESP32 (`esp32dev`) Pin | Function |
| :--- | :--- | :--- |
| `3.3V` | `3V3` | 3.3 V power |
| `GND` | `GND` | Common ground |
| `CLK` | `GPIO18` | VSPI SCK |
| `MISO` | `GPIO19` | VSPI MISO |
| `MOSI` | `GPIO23` | VSPI MOSI |
| `CS` | `GPIO5` | VSPI CS |
| `RESET` | `GPIO27` | Active-low reset |
| `BUSY` | `GPIO26` | SPI busy handshake |
| `DIO9` | `GPIO25` | Hardware IRQ output |

## Project Files
- `platformio.ini` — PlatformIO configuration (`esp32dev`, Arduino framework, `jgromes/RadioLib@^8.0.0`).
- `include/pin_config.h` — Pin definitions, test frequency (`CORE1121_TEST_FREQ_HZ`, default `915 MHz`; change to `868000000UL` for EU868), safe TX power, and PE4259 RF switch table.
- `src/main.cpp` — 8-step diagnostic suite and interactive Serial menu.
