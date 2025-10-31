# harvesthub-stm32

Deux modes:
- **Simulation PC**: `cargo computer`
- **Firmware STM32F103C8**: `cargo firmware`

Flash (quand carte disponible):
```
cargo install probe-rs-tools
cargo flash --chip STM32F103C8Tx --release --features firmware
```
