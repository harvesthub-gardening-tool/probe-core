use core::fmt;

use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};

const REG_CHIP_ID: u8 = 0xd0;
const REG_RESET: u8 = 0xe0;
const REG_CTRL_HUM: u8 = 0xf2;
const REG_CTRL_MEAS: u8 = 0xf4;
const REG_DATA: u8 = 0xf7;
const REG_CALIB1: u8 = 0x88;
const REG_CALIB2: u8 = 0xe1;
const CHIP_ID_BME280: u8 = 0x60;
const CHIP_IDS_BMP280: [u8; 3] = [0x58, 0x56, 0x57];
const SOFT_RESET: u8 = 0xb6;
const CTRL_HUM_OSRS_X1: u8 = 0x01;
const CTRL_MEAS_FORCED_X1: u8 = 0x25;

#[derive(Clone, Copy)]
pub struct Reading {
    pub temperature_centidegrees: i32,
    pub pressure_pa: u32,
    pub humidity_q1024: u32,
    pub is_bmp280: bool,
}

pub enum Error {
    I2c(i2c::Error),
    Missing,
    UnexpectedChipId(u8),
}

impl fmt::Debug for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::I2c(err) => f.debug_tuple("I2c").field(err).finish(),
            Self::Missing => f.write_str("Missing"),
            Self::UnexpectedChipId(id) => f
                .debug_tuple("UnexpectedChipId")
                .field(&HexByte(*id))
                .finish(),
        }
    }
}

struct HexByte(u8);

impl fmt::Debug for HexByte {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "0x{:02X}", self.0)
    }
}

impl From<i2c::Error> for Error {
    fn from(err: i2c::Error) -> Self {
        Self::I2c(err)
    }
}

#[derive(Clone, Copy)]
struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
    t_fine: i32,
}

impl Calibration {
    fn parse(buf1: &[u8; 26], buf2: &[u8; 7]) -> Self {
        Self {
            dig_t1: u16::from_le_bytes([buf1[0], buf1[1]]),
            dig_t2: i16::from_le_bytes([buf1[2], buf1[3]]),
            dig_t3: i16::from_le_bytes([buf1[4], buf1[5]]),
            dig_p1: u16::from_le_bytes([buf1[6], buf1[7]]),
            dig_p2: i16::from_le_bytes([buf1[8], buf1[9]]),
            dig_p3: i16::from_le_bytes([buf1[10], buf1[11]]),
            dig_p4: i16::from_le_bytes([buf1[12], buf1[13]]),
            dig_p5: i16::from_le_bytes([buf1[14], buf1[15]]),
            dig_p6: i16::from_le_bytes([buf1[16], buf1[17]]),
            dig_p7: i16::from_le_bytes([buf1[18], buf1[19]]),
            dig_p8: i16::from_le_bytes([buf1[20], buf1[21]]),
            dig_p9: i16::from_le_bytes([buf1[22], buf1[23]]),
            dig_h1: buf1[25],
            dig_h2: i16::from_le_bytes([buf2[0], buf2[1]]),
            dig_h3: buf2[2],
            dig_h4: (((buf2[3] as i8 as i16) << 4) | i16::from(buf2[4] & 0x0f)),
            dig_h5: (((buf2[5] as i8 as i16) << 4) | i16::from(buf2[4] >> 4)),
            dig_h6: buf2[6] as i8,
            t_fine: 0,
        }
    }

    fn compensate_temperature(&mut self, adc_t: i32) -> i32 {
        let var1 = (((adc_t >> 3) - (i32::from(self.dig_t1) << 1)) * i32::from(self.dig_t2)) >> 11;
        let var2_base = (adc_t >> 4) - i32::from(self.dig_t1);
        let var2 = (((var2_base * var2_base) >> 12) * i32::from(self.dig_t3)) >> 14;

        self.t_fine = var1 + var2;
        (self.t_fine * 5 + 128) >> 8
    }

    fn compensate_pressure(&self, adc_p: i32) -> u32 {
        let mut var1 = i64::from(self.t_fine) - 128_000;
        let mut var2 = var1 * var1 * i64::from(self.dig_p6);
        var2 += (var1 * i64::from(self.dig_p5)) << 17;
        var2 += i64::from(self.dig_p4) << 35;
        var1 =
            ((var1 * var1 * i64::from(self.dig_p3)) >> 8) + ((var1 * i64::from(self.dig_p2)) << 12);
        var1 = (((1_i64 << 47) + var1) * i64::from(self.dig_p1)) >> 33;

        if var1 == 0 {
            return 0;
        }

        let mut pressure = 1_048_576_i64 - i64::from(adc_p);
        pressure = (((pressure << 31) - var2) * 3125) / var1;
        var1 = (i64::from(self.dig_p9) * (pressure >> 13) * (pressure >> 13)) >> 25;
        var2 = (i64::from(self.dig_p8) * pressure) >> 19;
        pressure = ((pressure + var1 + var2) >> 8) + (i64::from(self.dig_p7) << 4);

        (pressure >> 8) as u32
    }

    fn compensate_humidity(&self, adc_h: i32) -> u32 {
        let mut value = self.t_fine - 76_800;
        value =
            ((((adc_h << 14) - (i32::from(self.dig_h4) << 20) - (i32::from(self.dig_h5) * value))
                + 16_384)
                >> 15)
                * (((((((value * i32::from(self.dig_h6)) >> 10)
                    * (((value * i32::from(self.dig_h3)) >> 11) + 32_768))
                    >> 10)
                    + 2_097_152)
                    * i32::from(self.dig_h2)
                    + 8192)
                    >> 14);
        value -= ((((value >> 15) * (value >> 15)) >> 7) * i32::from(self.dig_h1)) >> 4;

        value.clamp(0, 419_430_400) as u32 >> 12
    }
}

pub struct Bme280 {
    address: u8,
    is_bmp280: bool,
    calibration: Calibration,
}

impl Bme280 {
    pub async fn init(i2c: &mut SensorI2c<'_>, addresses: &[u8]) -> Result<Self, Error> {
        let mut chip_id = 0;
        let mut address = 0;

        for candidate in addresses {
            let mut id = [0u8; 1];
            if i2c
                .write_read(*candidate, &[REG_CHIP_ID], &mut id)
                .await
                .is_ok()
            {
                chip_id = id[0];
                address = *candidate;
                break;
            }
        }

        if address == 0 {
            return Err(Error::Missing);
        }

        let is_bmp280 = match chip_id {
            CHIP_ID_BME280 => false,
            id if CHIP_IDS_BMP280.contains(&id) => true,
            id => return Err(Error::UnexpectedChipId(id)),
        };

        write_reg(i2c, address, REG_RESET, SOFT_RESET).await?;
        Timer::after_millis(3).await;

        let mut buf1 = [0u8; 26];
        let mut buf2 = [0u8; 7];
        i2c.write_read(address, &[REG_CALIB1], &mut buf1).await?;
        i2c.write_read(address, &[REG_CALIB2], &mut buf2).await?;

        let calibration = Calibration::parse(&buf1, &buf2);

        if !is_bmp280 {
            write_reg(i2c, address, REG_CTRL_HUM, CTRL_HUM_OSRS_X1).await?;
        }

        Ok(Self {
            address,
            is_bmp280,
            calibration,
        })
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    pub async fn read_forced(&mut self, i2c: &mut SensorI2c<'_>) -> Result<Reading, Error> {
        write_reg(i2c, self.address, REG_CTRL_MEAS, CTRL_MEAS_FORCED_X1).await?;
        Timer::after(Duration::from_millis(10)).await;

        let mut raw = [0u8; 8];
        i2c.write_read(self.address, &[REG_DATA], &mut raw).await?;

        let adc_p = (i32::from(raw[0]) << 12) | (i32::from(raw[1]) << 4) | i32::from(raw[2] >> 4);
        let adc_t = (i32::from(raw[3]) << 12) | (i32::from(raw[4]) << 4) | i32::from(raw[5] >> 4);
        let adc_h = (i32::from(raw[6]) << 8) | i32::from(raw[7]);

        let temperature_centidegrees = self.calibration.compensate_temperature(adc_t);
        let pressure_pa = self.calibration.compensate_pressure(adc_p);
        let humidity_q1024 = if self.is_bmp280 {
            0
        } else {
            self.calibration.compensate_humidity(adc_h)
        };

        Ok(Reading {
            temperature_centidegrees,
            pressure_pa,
            humidity_q1024,
            is_bmp280: self.is_bmp280,
        })
    }
}

type SensorI2c<'d> = I2c<'d, Async, i2c::Master>;

async fn write_reg(
    i2c: &mut SensorI2c<'_>,
    address: u8,
    register: u8,
    value: u8,
) -> Result<(), i2c::Error> {
    i2c.write(address, &[register, value]).await
}
