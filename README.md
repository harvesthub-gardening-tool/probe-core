# embassy_core — STM32WB55CGU6

Embassy-rs firmware for the **STM32WB55CGU6** (Cortex-M4F + Cortex-M0+).
The M0+ is assumed to be pre-flashed with the **ST BLE Full Stack v1.24.0.3**.
The M4 is flashed manually over USART1 via `stm32flash` (no SWD probe required).

## Simple architecture

The firmware is split into a few small runtime pieces:

- `main.rs` configures clocks, USART1 logging, BLE advertising, and the sensor scheduler.
- `logger.rs` owns the shared USART1 TX logger used by all tasks.
- `bme280.rs` contains the BME280/BMP280 chip-id probe, calibration parsing, forced-mode reads,
  and compensation math ported from the sibling Cube/HAL `../core` firmware.
- The **heartbeat task** prints a low-rate uptime message so the board is visibly alive without
  flooding the UART.
- The main loop runs a **single cycle lifecycle**: sleep, measure air, measure soil, update GATT,
  advertise once, then return to sleep.
- The **BLE stack setup** (TL mailbox, CPU2 BLE init, reset/config, GATT/GAP init, and
  service/characteristic registration) runs exactly once at boot.

At boot, both sensor rails are off. The BME/BMP rail is controlled by **PB2** and the RS485 rail by
**PB0**; both controls are active-low, so low means powered and high means off.

## Quick setup

1. Make sure the STM32WB M0+ wireless stack is already flashed with **BLE Full Stack v1.24.0.3**.
2. Wire USART1 on **PA9/PA10** for logs/flashing and keep **BOOT0** accessible.
3. Wire the BME/BMP sensor on **PB6/PB7**, with **CSB high** for I2C mode and **SDO** strapped to
   select `0x76` or `0x77`.
4. Wire the RS485/ZTS3000 interface on **PA2/PA3/PB1** and its power control on **PB0**.
5. Build with `cargo build --release`, convert to `.bin`, then flash with `stm32flash` as shown
   below.

## Hardware

| Pin   | Function          | Notes                       |
|-------|-------------------|-----------------------------|
| PA9   | USART1_TX         | 115200 8N1 — debug log out  |
| PA10  | USART1_RX         | 115200 8N1                  |
| PB6   | I2C1_SCL          | External pull-up recommended |
| PB7   | I2C1_SDA          | External pull-up recommended |
| PB2   | I2C sensor power  | Active-low NPN control: low = on, high = off |
| BME/BMP CSB | Interface select | Tie to 3.3 V / VDDIO for I2C mode |
| BME/BMP SDO | Address select | Tie to GND for `0x76`, or 3.3 V / VDDIO for `0x77` |
| PA2   | LPUART1_TX        | RS485 DI, 4800 8N1 ZTS3000 probe |
| PA3   | LPUART1_RX        | RS485 RO                       |
| PB1   | LPUART1_DE        | RS485 DE/RE tied together, hardware driver-enable |
| PB0   | RS485 sensor power| Active-low NPN control: low = on, high = off |
| OSC_IN| HSE 32 MHz        | Required by the BLE radio/IPCC wakeup clock |
| BOOT0 | High → bootloader | Pull high then reset to flash |

## Memory layout

STM32WB55**CG**U6 has 512 KB flash and 256 KB RAM (SRAM1 192 K + SRAM2a 32 K + SRAM2b 32 K).
With BLE Full Stack v1.24, the M0+ binary occupies the upper portion of flash. We reserve
the **lower 192 KB** of flash for the M4 application — adjust `FLASH.LENGTH` in `memory.x`
if you switch to a smaller stack (e.g. BLE_HCI or BLE_LIGHT).

The IPCC/Transport-Layer mailbox sections in `memory.x` are placed in SRAM2a at
`0x2003_0000` using the current Embassy WPAN section names (`TL_REF_TABLE`, `MB_MEM1`,
`MB_MEM2`). Do **not** move them into SRAM1; the M0+ wireless stack dereferences these
shared SRAM2 pointers directly.

## Build

```bash
rustup target add thumbv7em-none-eabihf
cargo build --release
```

`build.rs` injects `PROBE_BUILD_UUID` at compile time (`cargo:rustc-env`) as a lowercase RFC4122
v4 UUID (36 ASCII bytes) generated from `/dev/urandom`. Each firmware build therefore gets its own
probe UUID value published over GATT.

Output ELF: `target/thumbv7em-none-eabihf/release/embassy_core`

Current release size with BLE advertising and sensor bring-up enabled is about **110 KiB**,
below the 192 KiB M4 flash window configured in `memory.x`.

## Flash with stm32flash

1. Pull **BOOT0 high**, press **NRST** (system bootloader is now active on USART1).
2. Connect a USB-UART adapter to PA9 (MCU TX → adapter RX) / PA10 (MCU RX → adapter TX) / GND.
3. Run:

```bash
arm-none-eabi-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/embassy_core \
  target/thumbv7em-none-eabihf/release/embassy_core.bin

stm32flash -w target/thumbv7em-none-eabihf/release/embassy_core.bin \
           -v -g 0x0 -b 115200 /dev/tty.usbserial-XXXX
```

4. Pull **BOOT0 low**, press **NRST** to run the application.
5. Open a serial monitor at **115200 8N1** to see the boot banner, heartbeat, BLE init logs,
   and sensor bring-up logs.

A `cargo run --release` shortcut is wired in `.cargo/config.toml` — it builds, runs
`objcopy`, and prints the exact `stm32flash` command for you (override the port via
`SERIAL_PORT=/dev/tty.usbserial-XXXX cargo run --release`).

## Expected serial output

```
[       123 ms]
[       123 ms] ==========================================
[       124 ms]  embassy_core boot
[       125 ms]  target  : STM32WB55CGU6
[       126 ms]  sysclk  : 64 MHz PLL from HSE 32 MHz
[       127 ms]  uart    : USART1 PA9/PA10 @ 115200 8N1
[       128 ms]  i2c     : I2C1 PB6/PB7, PB2 active-low power
[       129 ms]  rs485   : LPUART1 PA2/PA3, PB1 DE, PB0 active-low power
[       130 ms]  ble     : HH-PROBE-A peripheral advertising + Environmental Sensing
[       131 ms] ==========================================
[       132 ms]
[       133 ms] [init] spawning heartbeat task
[       134 ms] [init] spawning sensor task
[       135 ms] [ble ] initializing TL mailbox
[       136 ms] [ble ] spawning memory-manager queue
[       137 ms] [ble ] starting CPU2 BLE stack
...
[       250 ms] [ble ] advertising; open ST BLE Tool and scan for HH-PROBE-A
[      1133 ms] [hb  ] beat #1 uptime_ms=1133
[      2133 ms] [hb  ] beat #2 uptime_ms=2133
...
```

## Cycle lifecycle (development timing)

Current development constants in `src/main.rs`:

- `SLEEP_DURATION_MS = 30_000` (30 s)
- `SENSOR_ACTIVE_PHASE_MS = 10_000` (10 s for BME/BMP phase)
- `SENSOR_ACTIVE_PHASE_MS = 10_000` (10 s for ZTS3000 phase)
- `BLE_ADV_TIMEOUT_MS = 60_000` (stop advertising if hub absent)

Cycle flow is:

1. idle/sleep 30 s (`Timer::after_millis`, not STOP mode),
2. power BME/BMP rail (**PB2 low**), sample for 10 s, drop I2C scope, power rail off (**PB2 high**),
3. power ZTS3000 RS485 rail (**PB0 low**), sample for 10 s, power rail off (**PB0 high**),
4. update all GATT characteristic values,
5. advertise connectable,
6. if no central connects before timeout: set GAP non-discoverable,
7. if connected: wait for disconnect (or force terminate stale connection), then return to step 1.

Production target sleep should be changed to 5 hours:

- `SLEEP_DURATION_MS = 18_000_000`

## Sensor bring-up

The firmware starts both sensor rails off and runs battery-oriented measurement phases. The phase
lengths are easy to tune in `src/main.rs` with `SENSOR_ACTIVE_PHASE_MS` and
`SENSOR_SAMPLE_INTERVAL_MS`. The default measurement windows are:

1. drive **PB2 low** for 10 s, initialize the BME280/BMP280, take forced-mode samples every 2 s,
   print one compensated average, then drive **PB2 high**;
2. drive **PB0 low** for 10 s, poll the ZTS3000 every 2 s, print one averaged Modbus reading,
   then drive **PB0 high**;

The BME/BMP flow follows the sibling Cube/HAL `../core` firmware: BME280 chip-id is `0x60`, while
BMP280 can report `0x58`, `0x56`, or `0x57`. Tie **CSB/CS** to **3.3 V / VDDIO** so the sensor is
in I2C mode. Tie **SDO/ADR** to **GND** for address `0x76`, or to **3.3 V / VDDIO** for address
`0x77`; do not leave CSB or SDO floating. PB6/PB7 should have real external pull-ups; when PB2
switches off the BME rail, those pull-ups should not back-power the unpowered sensor through SDA
or SCL.

Typical sensor logs look like:

```text
[sens] power rails off: PB2=high PB0=high (active-low)
[phase] BME on PB2=low for 10s
[bme ] avg 5 samples addr=0x76 temp=22.0C pressure=1013.25hPa humidity=42.1%
[phase] BME off PB2=high
[phase] idle after BME for 10s
[phase] RS485 on PB0=low for 10s
[rs485] avg 5 samples humidity=42.1% temperature=21.7C
[phase] RS485 off PB0=high
[phase] idle after RS485 for 10s
```

The STM32WB55CG target generated by Embassy exposes **LPUART1**, not USART2. PA2/PA3 are
therefore used as LPUART1 TX/RX for RS485, while USART1 remains dedicated to flashing/debug.

## Battery-life notes

Current firmware optimizations are intentionally simple and hardware-friendly:

- **Duty-cycled sensors:** the BME/BMP and RS485 sensors are not powered continuously. Each rail is
  enabled for a 10 s measurement window, then disabled for a 10 s idle window.
- **Forced BME/BMP reads:** the BME/BMP is initialized during its active phase and sampled in forced
  mode every 2 s instead of continuous measurement mode.
- **Scoped I2C peripheral:** I2C1 is created only inside the BME/BMP phase using Embassy
  `Peri::reborrow()`. The I2C driver is dropped before **PB2** is driven high, which releases the
  I2C RCC clock and disconnects SCL/SDA from the alternate-function driver before the sensor rail is
  switched off.
- **RS485 power gating:** the RS485/ZTS3000 rail is only enabled during its active phase; hardware
  driver-enable on **PB1** is used for Modbus transmission.
- **Reduced debug noise:** heartbeat logs run every 30 s and sensor phases print averaged results
  instead of logging every low-level transaction.
- **No internal I2C pull-ups:** PB6/PB7 internal pull-ups are disabled. Use real external pull-ups,
  and make sure they do not back-power the unpowered BME/BMP through SDA/SCL when PB2 is high.

Future battery improvements can go further by entering STM32 low-power sleep/stop modes during idle
windows, reducing BLE advertising duty cycle, and measuring board-level leakage from pull-ups,
transistor bias networks, and sensor breakout regulators.

## Verify with ST BLE Tool on iOS

1. Flash and run the firmware with **BOOT0 low**.
2. Keep the serial monitor open and wait for:

```text
[ble ] advertising; open ST BLE Tool and scan for HH-PROBE-A
```

3. Open **ST BLE Tool** on iOS and start scanning.
4. The peripheral should appear as **HH-PROBE-A** and be connectable.

The primary advertising packet carries HarvestHub manufacturer data (`0xFF`, company `0x1234`,
marker `HH-PROBE`, version `1.0`, and name `HH-PROBE-A`). The complete local name is provided in
scan response data to keep the primary advertisement under the 31-byte limit.

## Clock notes

The firmware uses the same 64 MHz HSE-derived CPU1 clock as Embassy's STM32WB examples, but
selects `HSE/1024` as the RF wakeup clock instead of LSE. The CPU2 AHB prescaler is `/2`,
matching Embassy's `WPAN_DEFAULT` 64 MHz / 32 MHz split, and the BLE init parameter sets
`ls_source = 0b101` so CPU2 is told to use HSE/1024 with calibration.

Avoid plain `WPAN_DEFAULT` on this board unless a 32.768 kHz LSE crystal is also confirmed:
`WPAN_DEFAULT` enables LSE and can hang before the UART banner if LSE is absent or not
oscillating.

## ISR-delay notes

The BLE stack can emit `HardwareError(IsrDelay)` when CPU2 radio/IPCC deadlines are missed.
UART debug output must therefore avoid long interrupt-masked sections. The logger only disables
interrupts while taking/replacing the USART TX handle; the slow blocking UART writes happen with
interrupts enabled, and repeated ISR-delay events are throttled in the BLE event loop.

If the firmware prints the boot banner but stops around BLE mailbox/init logs, investigate
the M0+ wireless stack start state and the 32 MHz HSE path. If the boot banner never appears,
the HSE ready wait is failing before USART1 setup.
