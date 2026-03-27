// src/config.rs

pub const COMPANY_ID: u16 = 0x1234;
pub const MAGIC_MARKER: &[u8; 8] = b"HH-PROBE";

// Les infos spécifiques de cette sonde
pub const PROBE_VERSION: [u8; 2] = [1, 0];
pub const PROBE_NAME: &str = "Sonde-1";

// Les identifiants Bluetooth (GATT) pour plus tard
pub const ENVIRONMENTAL_SENSING_SERVICE_UUID16: u16 = 0x181A;
pub const TEMP_CHAR_UUID16: u16 = 0x2A6E;
pub const HUM_CHAR_UUID16:  u16 = 0x2A6F;