#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt;
use core::time::Duration as CoreDuration;

use cortex_m::interrupt::Mutex;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::ipcc::{
    Config as IpccConfig, ReceiveInterruptHandler, TransmitInterruptHandler,
};
use embassy_stm32::peripherals;
use embassy_stm32::rcc::mux::{I2c1sel, Lpuart1sel, Rfwkpsel};
use embassy_stm32::rcc::{
    AHBPrescaler, Hse, HseMode, HsePrescaler, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv,
    PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, Uart, UartTx};
use embassy_stm32::{bind_interrupts, dma, usart, Peri};
use embassy_stm32_wpan::hci::event::command::ReturnParameters;
use embassy_stm32_wpan::hci::event::HardwareError;
use embassy_stm32_wpan::hci::host::uart::{Packet, UartHci};
use embassy_stm32_wpan::hci::host::{
    AdvertisingFilterPolicy, EncryptionKey, HostHci, OwnAddressType,
};
use embassy_stm32_wpan::hci::types::AdvertisingType;
use embassy_stm32_wpan::hci::vendor::command::gap::{DiscoverableParameters, GapCommands, Role};
use embassy_stm32_wpan::hci::vendor::command::gatt::GattCommands;
use embassy_stm32_wpan::hci::vendor::command::gatt::{
    AddCharacteristicParameters, AddServiceParameters, CharacteristicEvent,
    CharacteristicPermission, CharacteristicProperty, EncryptionKeySize, ServiceType,
    UpdateCharacteristicValueParameters, Uuid,
};
use embassy_stm32_wpan::hci::vendor::command::hal::{ConfigData, HalCommands, PowerLevel};
use embassy_stm32_wpan::hci::vendor::event::command::VendorReturnParameters;
use embassy_stm32_wpan::hci::vendor::event::AttributeHandle;
use embassy_stm32_wpan::hci::BdAddr;
use embassy_stm32_wpan::hci::Event;
use embassy_stm32_wpan::hci::Status;
use embassy_stm32_wpan::lhci::LhciC1DeviceInformationCcrp;
use embassy_stm32_wpan::shci::ShciBleInitCmdParam;
use embassy_stm32_wpan::sub::mm;
use embassy_stm32_wpan::TlMbox;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use panic_halt as _;
use static_cell::StaticCell;

mod bme280;
mod logger;

use logger::{set_logger, Logger};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    LPUART1 => usart::InterruptHandler<peripherals::LPUART1>;
    DMA1_CHANNEL3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    DMA1_CHANNEL6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    IPCC_C1_RX => ReceiveInterruptHandler;
    IPCC_C1_TX => TransmitInterruptHandler;
});

type SharedTx = Mutex<RefCell<Option<UartTx<'static, embassy_stm32::mode::Async>>>>;

static TX_CELL: StaticCell<SharedTx> = StaticCell::new();

const DEVICE_NAME: &[u8] = b"HH-PROBE-A";
const PROBE_UUID: &str = env!("PROBE_BUILD_UUID");
const BLE_GAP_DEVICE_NAME_LENGTH: u8 = DEVICE_NAME.len() as u8;
const PROBE_VERSION_MAJOR: u8 = 1;
const PROBE_VERSION_MINOR: u8 = 0;
const HUB_COMPANY_ID_LE: [u8; 2] = 0x1234_u16.to_le_bytes();
const PROBE_ADV_DATA: [u8; 24] = [
    0xff,
    HUB_COMPANY_ID_LE[0],
    HUB_COMPANY_ID_LE[1],
    b'H',
    b'H',
    b'-',
    b'P',
    b'R',
    b'O',
    b'B',
    b'E',
    PROBE_VERSION_MAJOR,
    PROBE_VERSION_MINOR,
    DEVICE_NAME.len() as u8,
    b'H',
    b'H',
    b'-',
    b'P',
    b'R',
    b'O',
    b'B',
    b'E',
    b'-',
    b'A',
];
const PROBE_SCAN_RESPONSE_DATA: [u8; 12] = [
    11, 0x09, b'H', b'H', b'-', b'P', b'R', b'O', b'B', b'E', b'-', b'A',
];
const ENVIRONMENTAL_SENSING_SERVICE_UUID: u16 = 0x181a;
const TEMPERATURE_CHAR_UUID: u16 = 0x2a6e;
const PRESSURE_CHAR_UUID: u16 = 0x2a6d;
const HUMIDITY_CHAR_UUID: u16 = 0x2a6f;
const PROBE_UUID_CHAR_UUID: [u8; 16] = [
    0x12, 0x34, 0x00, 0x02, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb,
];
const SOIL_TEMPERATURE_CHAR_UUID: [u8; 16] = [
    0x12, 0x34, 0x00, 0x03, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb,
];
const SOIL_HUMIDITY_CHAR_UUID: [u8; 16] = [
    0x12, 0x34, 0x00, 0x04, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb,
];

const BLE_CFG_IRK: [u8; 16] = [
    0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
];
const BLE_CFG_ERK: [u8; 16] = [
    0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21, 0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21,
];

const HEARTBEAT_INTERVAL_MS: u64 = 30_000;
const SLEEP_DURATION_MS: u64 = 30_000;
const SENSOR_ACTIVE_PHASE_MS: u64 = 10_000;
const SENSOR_SAMPLE_INTERVAL_MS: u64 = 2_000;
const BME280_POWER_SETTLE_MS: u64 = 1_500;
const RS485_POWER_SETTLE_MS: u64 = 500;
const BLE_ADV_TIMEOUT_MS: u64 = 60_000;
const BLE_CONNECTED_TIMEOUT_MS: u64 = 60_000;
const BLE_TERMINATE_WAIT_MS: u64 = 10_000;
const BME280_ADDR_PRIMARY: u8 = 0x76;
const BME280_ADDR_SECONDARY: u8 = 0x77;
const ZTS3000_READ_HUM_TEMP: [u8; 8] = [0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b];
const ZTS3000_RESPONSE_LEN: usize = 9;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(32_000_000),
        mode: HseMode::Oscillator,
        prescaler: HsePrescaler::Div1,
    });
    config.rcc.pll = Some(Pll {
        source: PllSource::Hse,
        prediv: PllPreDiv::Div2,
        mul: PllMul::Mul12,
        divp: Some(PllPDiv::Div3),
        divq: Some(PllQDiv::Div4),
        divr: Some(PllRDiv::Div3),
    });
    config.rcc.sys = Sysclk::Pll1R;
    config.rcc.core2_ahb_pre = AHBPrescaler::Div2;
    config.rcc.mux.rfwkpsel = Rfwkpsel::HseDiv1024;
    config.rcc.mux.i2c1sel = I2c1sel::Pclk1;
    config.rcc.mux.lpuart1sel = Lpuart1sel::Pclk1;
    let p = embassy_stm32::init(config);

    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;

    let uart = usart::Uart::new(
        p.USART1, p.PA10, p.PA9, p.DMA1_CH1, p.DMA1_CH2, Irqs, uart_cfg,
    )
    .expect("failed to init USART1");

    let (tx, _rx) = uart.split();

    let shared: &'static SharedTx = TX_CELL.init(Mutex::new(RefCell::new(Some(tx))));
    set_logger(Logger::new(shared));

    log!("");
    log!("==========================================");
    log!(" embassy_core boot");
    log!(" target  : STM32WB55CGU6");
    log!(" sysclk  : 64 MHz PLL from HSE 32 MHz");
    log!(" uart    : USART1 PA9/PA10 @ 115200 8N1");
    log!(" i2c     : I2C1 PB6/PB7, PB2 active-low power");
    log!(" rs485   : LPUART1 PA2/PA3, PB1 DE, PB0 active-low power");
    log!(" ble     : HH-PROBE-A peripheral advertising + Environmental Sensing");
    log!("==========================================");
    log!("");

    log!("[init] spawning heartbeat task");
    spawner.spawn(heartbeat().unwrap());

    log!("[ble ] initializing TL mailbox");
    let mbox = TlMbox::init(p.IPCC, Irqs, IpccConfig::default())
        .await
        .unwrap();
    let mut sys = mbox.sys_subsystem;
    let mut ble = mbox.ble_subsystem;

    log!("[ble ] spawning memory-manager queue");
    spawner.spawn(run_mm_queue(mbox.mm_subsystem).unwrap());

    if let Some(fw_info) = sys.wireless_fw_info() {
        log!(
            "[ble ] wireless fw version {}.{}.{}  SRAM2a={}K SRAM2b={}K flash={}x4K",
            fw_info.version_major(),
            fw_info.version_minor(),
            fw_info.subversion(),
            fw_info.sram2a_size(),
            fw_info.sram2b_size(),
            fw_info.flash_size()
        );
    } else {
        log!("[ble ] wireless fw info not populated yet");
    }

    log!("[ble ] starting CPU2 BLE stack");
    let ble_init = ShciBleInitCmdParam {
        ls_source: 0b101,
        ..Default::default()
    };
    match sys.shci_c2_ble_init(ble_init).await {
        Ok(_) => log!("[ble ] shci_c2_ble_init command accepted"),
        Err(()) => log!("[ble ] shci_c2_ble_init failed"),
    }

    log!("[ble ] reset");
    ble.reset().await;
    log!("[ble ] reset response {:?}", ble.read().await);

    log!("[ble ] config public address");
    ble.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
        .await;
    log!("[ble ] public address response {:?}", ble.read().await);

    log!("[ble ] config random address");
    ble.write_config_data(&ConfigData::random_address(get_random_addr()).build())
        .await;
    log!("[ble ] random address response {:?}", ble.read().await);

    log!("[ble ] config identity root");
    ble.write_config_data(&ConfigData::identity_root(&get_irk()).build())
        .await;
    log!("[ble ] identity root response {:?}", ble.read().await);

    log!("[ble ] config encryption root");
    ble.write_config_data(&ConfigData::encryption_root(&get_erk()).build())
        .await;
    log!("[ble ] encryption root response {:?}", ble.read().await);

    log!("[ble ] set TX power");
    ble.set_tx_power_level(PowerLevel::ZerodBm).await;
    log!("[ble ] TX power response {:?}", ble.read().await);

    log!("[ble ] init GATT");
    ble.init_gatt().await;
    log!("[ble ] GATT response {:?}", ble.read().await);

    log!("[ble ] init GAP");
    ble.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME_LENGTH)
        .await;
    log!("[ble ] GAP response {:?}", ble.read().await);

    log!("[ble ] set scan response data");
    match ble
        .le_set_scan_response_data(&PROBE_SCAN_RESPONSE_DATA)
        .await
    {
        Ok(()) => log!("[ble ] scan response command sent"),
        Err(_) => log!("[ble ] scan response command failed"),
    }
    log!("[ble ] scan response {:?}", ble.read().await);

    let discovery_params = DiscoverableParameters {
        advertising_type: AdvertisingType::ConnectableUndirected,
        advertising_interval: Some((
            CoreDuration::from_millis(250),
            CoreDuration::from_millis(250),
        )),
        address_type: OwnAddressType::Public,
        filter_policy: AdvertisingFilterPolicy::AllowConnectionAndScan,
        local_name: None,
        advertising_data: &PROBE_ADV_DATA,
        conn_interval: (None, None),
    };

    log!("[ble ] add Environmental Sensing service (0x181A)");
    ble.add_service(&AddServiceParameters {
        uuid: Uuid::Uuid16(ENVIRONMENTAL_SENSING_SERVICE_UUID),
        service_type: ServiceType::Primary,
        max_attribute_records: 14,
    })
    .await;
    let env_service_handle = match wait_for_gatt_add_service_complete(&mut ble).await {
        Some(handle) => {
            log!("[ble ] service handle: {:?}", handle);
            handle
        }
        None => {
            log!("[ble ] failed to add Environmental Sensing service");
            return;
        }
    };

    log!("[ble ] add probe UUID characteristic (custom 128-bit)");
    ble.add_characteristic(&AddCharacteristicParameters {
        service_handle: env_service_handle,
        characteristic_uuid: Uuid::Uuid128(PROBE_UUID_CHAR_UUID),
        characteristic_properties: CharacteristicProperty::READ,
        characteristic_value_len: 36,
        security_permissions: CharacteristicPermission::empty(),
        gatt_event_mask: CharacteristicEvent::empty(),
        encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
        is_variable: false,
    })
    .await;
    let probe_uuid_char_handle = match wait_for_gatt_add_characteristic_complete(&mut ble).await {
        Some(handle) => {
            log!("[ble ] probe_uuid char handle: {:?}", handle);
            handle
        }
        None => {
            log!("[ble ] failed to add probe UUID characteristic");
            return;
        }
    };

    log!("[ble ] add air temperature characteristic (0x2A6E)");
    ble.add_characteristic(&AddCharacteristicParameters {
        service_handle: env_service_handle,
        characteristic_uuid: Uuid::Uuid16(TEMPERATURE_CHAR_UUID),
        characteristic_properties: CharacteristicProperty::READ,
        characteristic_value_len: 2,
        security_permissions: CharacteristicPermission::empty(),
        gatt_event_mask: CharacteristicEvent::empty(),
        encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
        is_variable: false,
    })
    .await;
    let air_temp_char_handle = match wait_for_gatt_add_characteristic_complete(&mut ble).await {
        Some(handle) => {
            log!("[ble ] air_temp char handle: {:?}", handle);
            handle
        }
        None => {
            log!("[ble ] failed to add temperature characteristic");
            return;
        }
    };

    log!("[ble ] add air pressure characteristic (0x2A6D)");
    ble.add_characteristic(&AddCharacteristicParameters {
        service_handle: env_service_handle,
        characteristic_uuid: Uuid::Uuid16(PRESSURE_CHAR_UUID),
        characteristic_properties: CharacteristicProperty::READ,
        characteristic_value_len: 4,
        security_permissions: CharacteristicPermission::empty(),
        gatt_event_mask: CharacteristicEvent::empty(),
        encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
        is_variable: false,
    })
    .await;
    let air_pressure_char_handle = match wait_for_gatt_add_characteristic_complete(&mut ble).await {
        Some(handle) => {
            log!("[ble ] air_pressure char handle: {:?}", handle);
            handle
        }
        None => {
            log!("[ble ] failed to add pressure characteristic");
            return;
        }
    };

    log!("[ble ] add air humidity characteristic (0x2A6F)");
    ble.add_characteristic(&AddCharacteristicParameters {
        service_handle: env_service_handle,
        characteristic_uuid: Uuid::Uuid16(HUMIDITY_CHAR_UUID),
        characteristic_properties: CharacteristicProperty::READ,
        characteristic_value_len: 2,
        security_permissions: CharacteristicPermission::empty(),
        gatt_event_mask: CharacteristicEvent::empty(),
        encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
        is_variable: false,
    })
    .await;
    let air_hum_char_handle = match wait_for_gatt_add_characteristic_complete(&mut ble).await {
        Some(handle) => {
            log!("[ble ] air_hum char handle: {:?}", handle);
            handle
        }
        None => {
            log!("[ble ] failed to add humidity characteristic");
            return;
        }
    };

    log!("[ble ] add soil temperature characteristic (custom 128-bit)");
    ble.add_characteristic(&AddCharacteristicParameters {
        service_handle: env_service_handle,
        characteristic_uuid: Uuid::Uuid128(SOIL_TEMPERATURE_CHAR_UUID),
        characteristic_properties: CharacteristicProperty::READ,
        characteristic_value_len: 2,
        security_permissions: CharacteristicPermission::empty(),
        gatt_event_mask: CharacteristicEvent::empty(),
        encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
        is_variable: false,
    })
    .await;
    let soil_temp_char_handle = match wait_for_gatt_add_characteristic_complete(&mut ble).await {
        Some(handle) => {
            log!("[ble ] soil_temp char handle: {:?}", handle);
            handle
        }
        None => {
            log!("[ble ] failed to add soil temperature characteristic");
            return;
        }
    };

    log!("[ble ] add soil humidity characteristic (custom 128-bit)");
    ble.add_characteristic(&AddCharacteristicParameters {
        service_handle: env_service_handle,
        characteristic_uuid: Uuid::Uuid128(SOIL_HUMIDITY_CHAR_UUID),
        characteristic_properties: CharacteristicProperty::READ,
        characteristic_value_len: 2,
        security_permissions: CharacteristicPermission::empty(),
        gatt_event_mask: CharacteristicEvent::empty(),
        encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
        is_variable: false,
    })
    .await;
    let soil_hum_char_handle = match wait_for_gatt_add_characteristic_complete(&mut ble).await {
        Some(handle) => {
            log!("[ble ] soil_hum char handle: {:?}", handle);
            handle
        }
        None => {
            log!("[ble ] failed to add soil humidity characteristic");
            return;
        }
    };

    let gatt_handles = GattHandles {
        probe_uuid: probe_uuid_char_handle,
        air_temp: air_temp_char_handle,
        air_pressure: air_pressure_char_handle,
        air_humidity: air_hum_char_handle,
        soil_temp: soil_temp_char_handle,
        soil_humidity: soil_hum_char_handle,
    };

    let mut i2c_resources = I2cSensorPeripherals {
        i2c1: p.I2C1,
        scl: p.PB6,
        sda: p.PB7,
        tx_dma: p.DMA1_CH3,
        rx_dma: p.DMA1_CH4,
    };

    let mut i2c_power = Output::new(p.PB2, Level::High, Speed::Low);
    let mut rs485_power = Output::new(p.PB0, Level::High, Speed::Low);
    log!("[sens] power rails off: PB2=high PB0=high (active-low)");

    let mut rs485_config = UartConfig::default();
    rs485_config.baudrate = 4_800;
    rs485_config.de_assertion_time = 1;
    rs485_config.de_deassertion_time = 1;
    let mut rs485 = match Uart::new_with_de(
        p.LPUART1,
        p.PA3,
        p.PA2,
        p.PB1,
        p.DMA1_CH5,
        p.DMA1_CH6,
        Irqs,
        rs485_config,
    ) {
        Ok(uart) => uart,
        Err(err) => {
            log!("[rs485] LPUART1 init failed: {:?}", err);
            return;
        }
    };

    let mut snapshot = SensorSnapshot::default();
    if !update_ble_environmental_values(&mut ble, env_service_handle, &gatt_handles, &snapshot)
        .await
    {
        log!("[ble ] initial GATT value update failed");
        return;
    }

    loop {
        log!(
            "[cycle] sleep/idle for {}s before measurement",
            SLEEP_DURATION_MS / 1000
        );
        Timer::after_millis(SLEEP_DURATION_MS).await;

        let bme_reading = run_bme280_phase(&mut i2c_resources, &mut i2c_power).await;
        let soil_reading = run_zts3000_phase(&mut rs485, &mut rs485_power).await;
        snapshot = SensorSnapshot::from_phase_results(bme_reading, soil_reading);

        if !update_ble_environmental_values(&mut ble, env_service_handle, &gatt_handles, &snapshot)
            .await
        {
            log!("[ble ] GATT value update failed before advertise");
            continue;
        }

        log!("[ble ] set discoverable as HH-PROBE-A");
        if ble.set_discoverable(&discovery_params).await.is_err() {
            log!("[ble ] discoverable command failed");
            continue;
        }
        if !wait_for_gap_set_discoverable_complete(&mut ble).await {
            log!("[ble ] discoverable command-complete indicated failure");
            continue;
        }

        run_advertising_session(&mut ble).await;
    }
}

async fn wait_for_gatt_add_service_complete(ble: &mut impl UartHci) -> Option<AttributeHandle> {
    loop {
        let response = ble.read().await;
        if let Ok(Packet::Event(Event::CommandComplete(command_complete))) = response {
            if let ReturnParameters::Vendor(VendorReturnParameters::GattAddService(gatt_service)) =
                command_complete.return_params
            {
                if gatt_service.status == Status::Success {
                    return Some(gatt_service.service_handle);
                }

                log!(
                    "[ble ] GattAddService failed with status {:?}",
                    gatt_service.status
                );
                return None;
            }
        }
    }
}

async fn wait_for_gatt_add_characteristic_complete(
    ble: &mut impl UartHci,
) -> Option<AttributeHandle> {
    loop {
        let response = ble.read().await;
        if let Ok(Packet::Event(Event::CommandComplete(command_complete))) = response {
            if let ReturnParameters::Vendor(VendorReturnParameters::GattAddCharacteristic(
                gatt_char,
            )) = command_complete.return_params
            {
                if gatt_char.status == Status::Success {
                    return Some(gatt_char.characteristic_handle);
                }

                log!(
                    "[ble ] GattAddCharacteristic failed with status {:?}",
                    gatt_char.status
                );
                return None;
            }
        }
    }
}

async fn wait_for_gatt_update_complete(ble: &mut impl UartHci) -> bool {
    loop {
        let response = ble.read().await;
        if let Ok(Packet::Event(Event::CommandComplete(command_complete))) = response {
            if let ReturnParameters::Vendor(
                VendorReturnParameters::GattUpdateCharacteristicValue(status),
            ) = command_complete.return_params
            {
                if status == Status::Success {
                    return true;
                }

                log!(
                    "[ble ] GattUpdateCharacteristicValue failed with status {:?}",
                    status
                );
                return false;
            }
        }
    }
}

#[derive(Clone, Copy)]
struct GattHandles {
    probe_uuid: AttributeHandle,
    air_temp: AttributeHandle,
    air_pressure: AttributeHandle,
    air_humidity: AttributeHandle,
    soil_temp: AttributeHandle,
    soil_humidity: AttributeHandle,
}

#[derive(Clone, Copy, Default)]
struct SensorSnapshot {
    air_temperature_centi: i16,
    air_pressure_pa: u32,
    air_humidity_centi_pct: u16,
    soil_temperature_centi: i16,
    soil_humidity_centi_pct: u16,
}

impl SensorSnapshot {
    fn from_phase_results(
        air: Option<bme280::Reading>,
        soil: Option<Zts3000Reading>,
    ) -> SensorSnapshot {
        let (air_temperature_centi, air_pressure_pa, air_humidity_centi_pct) = match air {
            Some(reading) => {
                let temp_centi = reading
                    .temperature_centidegrees
                    .clamp(i16::MIN as i32, i16::MAX as i32)
                    as i16;
                let humidity_centi = if reading.is_bmp280 {
                    0
                } else {
                    let centi = (u64::from(reading.humidity_q1024) * 10_000 + 512) / 1024;
                    centi.min(u16::MAX as u64) as u16
                };

                (temp_centi, reading.pressure_pa, humidity_centi)
            }
            None => (0, 0, 0),
        };

        let (soil_temperature_centi, soil_humidity_centi_pct) = match soil {
            Some(reading) => (
                i32::from(reading.temperature_tenths)
                    .saturating_mul(10)
                    .clamp(i16::MIN as i32, i16::MAX as i32) as i16,
                u32::from(reading.humidity_tenths)
                    .saturating_mul(10)
                    .min(u16::MAX as u32) as u16,
            ),
            None => (0, 0),
        };

        SensorSnapshot {
            air_temperature_centi,
            air_pressure_pa,
            air_humidity_centi_pct,
            soil_temperature_centi,
            soil_humidity_centi_pct,
        }
    }
}

async fn update_ble_environmental_values(
    ble: &mut (impl UartHci + GattCommands),
    service_handle: AttributeHandle,
    handles: &GattHandles,
    snapshot: &SensorSnapshot,
) -> bool {
    let probe_uuid_bytes = PROBE_UUID.as_bytes();
    let air_temp_be = snapshot.air_temperature_centi.to_be_bytes();
    let air_pressure_be = snapshot.air_pressure_pa.to_be_bytes();
    let air_humidity_be = snapshot.air_humidity_centi_pct.to_be_bytes();
    let soil_temp_be = snapshot.soil_temperature_centi.to_be_bytes();
    let soil_humidity_be = snapshot.soil_humidity_centi_pct.to_be_bytes();

    if ble
        .update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: handles.probe_uuid,
            offset: 0,
            value: probe_uuid_bytes,
        })
        .await
        .is_err()
    {
        log!("[ble ] probe_uuid update command failed");
        return false;
    }
    if !wait_for_gatt_update_complete(ble).await {
        return false;
    }

    if ble
        .update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: handles.air_temp,
            offset: 0,
            value: &air_temp_be,
        })
        .await
        .is_err()
    {
        log!("[ble ] air temperature update command failed");
        return false;
    }
    if !wait_for_gatt_update_complete(ble).await {
        return false;
    }

    if ble
        .update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: handles.air_pressure,
            offset: 0,
            value: &air_pressure_be,
        })
        .await
        .is_err()
    {
        log!("[ble ] air pressure update command failed");
        return false;
    }
    if !wait_for_gatt_update_complete(ble).await {
        return false;
    }

    if ble
        .update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: handles.air_humidity,
            offset: 0,
            value: &air_humidity_be,
        })
        .await
        .is_err()
    {
        log!("[ble ] air humidity update command failed");
        return false;
    }
    if !wait_for_gatt_update_complete(ble).await {
        return false;
    }

    if ble
        .update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: handles.soil_temp,
            offset: 0,
            value: &soil_temp_be,
        })
        .await
        .is_err()
    {
        log!("[ble ] soil temperature update command failed");
        return false;
    }
    if !wait_for_gatt_update_complete(ble).await {
        return false;
    }

    if ble
        .update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: handles.soil_humidity,
            offset: 0,
            value: &soil_humidity_be,
        })
        .await
        .is_err()
    {
        log!("[ble ] soil humidity update command failed");
        return false;
    }
    wait_for_gatt_update_complete(ble).await
}

async fn wait_for_gap_set_discoverable_complete(ble: &mut impl UartHci) -> bool {
    loop {
        let response = ble.read().await;
        if let Ok(Packet::Event(Event::CommandComplete(command_complete))) = response {
            if let ReturnParameters::Vendor(VendorReturnParameters::GapSetDiscoverable(status)) =
                command_complete.return_params
            {
                if status != Status::Success {
                    log!("[ble ] GapSetDiscoverable failed with status {:?}", status);
                }
                return status == Status::Success;
            }
        }
    }
}

async fn wait_for_gap_set_nondiscoverable_complete(ble: &mut impl UartHci) -> bool {
    loop {
        let response = ble.read().await;
        if let Ok(Packet::Event(Event::CommandComplete(command_complete))) = response {
            if let ReturnParameters::Vendor(VendorReturnParameters::GapSetNonDiscoverable(status)) =
                command_complete.return_params
            {
                if status != Status::Success {
                    log!(
                        "[ble ] GapSetNonDiscoverable failed with status {:?}",
                        status
                    );
                }
                return status == Status::Success;
            }
        }
    }
}

async fn run_advertising_session(ble: &mut (impl UartHci + GapCommands)) {
    log!(
        "[ble ] advertising for up to {}s waiting for central",
        BLE_ADV_TIMEOUT_MS / 1000
    );

    let mut isr_delay_count = 0u32;
    let adv_deadline = Instant::now() + Duration::from_millis(BLE_ADV_TIMEOUT_MS);
    let mut active_conn_handle = None;
    let mut conn_deadline = None;

    loop {
        let now = Instant::now();

        if active_conn_handle.is_none() && now >= adv_deadline {
            log!("[ble ] advertise timeout reached; stop discoverable");
            let _ = ble.gap_set_nondiscoverable().await;
            let _ = wait_for_gap_set_nondiscoverable_complete(ble).await;
            return;
        }

        if let (Some(conn_handle), Some(deadline)) = (active_conn_handle, conn_deadline) {
            if now >= deadline {
                log!("[ble ] force terminate stale connection {:?}", conn_handle);
                if ble
                    .terminate(conn_handle, Status::RemoteTerminationByUser)
                    .await
                    .is_err()
                {
                    log!("[ble ] terminate command failed");
                    return;
                }
                conn_deadline = Some(Instant::now() + Duration::from_millis(BLE_TERMINATE_WAIT_MS));
            }
        }

        let response = with_timeout(Duration::from_millis(250), ble.read()).await;
        let Ok(packet_result) = response else {
            continue;
        };

        match packet_result {
            Ok(Packet::Event(event)) => match event {
                Event::HardwareError(HardwareError::IsrDelay) => {
                    isr_delay_count = isr_delay_count.wrapping_add(1);
                    if isr_delay_count == 1 || isr_delay_count.is_multiple_of(32) {
                        log!(
                            "[ble ] ISR-delay warnings: {} (UART logging throttled)",
                            isr_delay_count
                        );
                    }
                }
                Event::LeConnectionComplete(conn) if conn.status == Status::Success => {
                    active_conn_handle = Some(conn.conn_handle);
                    conn_deadline =
                        Some(Instant::now() + Duration::from_millis(BLE_CONNECTED_TIMEOUT_MS));
                    log!("[ble ] connected handle={:?}", conn.conn_handle);
                }
                Event::LeConnectionComplete(_) => {}
                Event::DisconnectionComplete(disconnection) => {
                    log!(
                        "[ble ] disconnected handle={:?} reason={:?}; end advertising session",
                        disconnection.conn_handle,
                        disconnection.reason
                    );
                    return;
                }
                _ => {}
            },
            Err(err) => log!("[ble ] read error {:?}", err),
        }
    }
}

#[embassy_executor::task]
async fn heartbeat() {
    log!("[hb  ] task started");
    let mut count: u32 = 0;
    loop {
        Timer::after_millis(HEARTBEAT_INTERVAL_MS).await;
        count = count.wrapping_add(1);
        log!(
            "[hb  ] beat #{} uptime_ms={}",
            count,
            Instant::now().as_millis()
        );
    }
}

#[embassy_executor::task]
async fn run_mm_queue(mut memory_manager: mm::MemoryManager<'static>) {
    memory_manager.run_queue().await;
}

struct I2cSensorPeripherals {
    i2c1: Peri<'static, peripherals::I2C1>,
    scl: Peri<'static, peripherals::PB6>,
    sda: Peri<'static, peripherals::PB7>,
    tx_dma: Peri<'static, peripherals::DMA1_CH3>,
    rx_dma: Peri<'static, peripherals::DMA1_CH4>,
}

async fn run_bme280_phase(
    resources: &mut I2cSensorPeripherals,
    power: &mut Output<'static>,
) -> Option<bme280::Reading> {
    power.set_low();
    log!(
        "[phase] BME on PB2=low for {}s",
        SENSOR_ACTIVE_PHASE_MS / 1000
    );
    let phase_deadline = Instant::now() + Duration::from_millis(SENSOR_ACTIVE_PHASE_MS);
    Timer::after_millis(BME280_POWER_SETTLE_MS).await;

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz(100_000);
    i2c_config.timeout = Duration::from_millis(100);
    i2c_config.scl_pullup = false;
    i2c_config.sda_pullup = false;

    let mut result = None;

    {
        let mut i2c = I2c::new(
            resources.i2c1.reborrow(),
            resources.scl.reborrow(),
            resources.sda.reborrow(),
            resources.tx_dma.reborrow(),
            resources.rx_dma.reborrow(),
            Irqs,
            i2c_config,
        );

        match bme280::Bme280::init(&mut i2c, &[BME280_ADDR_PRIMARY, BME280_ADDR_SECONDARY]).await {
            Ok(mut sensor) => {
                let mut average = Bme280Average::default();

                while Instant::now() < phase_deadline {
                    match sensor.read_forced(&mut i2c).await {
                        Ok(reading) => average.add(reading),
                        Err(err) => log!("[bme ] sample failed: {:?}", err),
                    }
                    Timer::after_millis(SENSOR_SAMPLE_INTERVAL_MS).await;
                }

                match average.finish() {
                    Some(reading) => {
                        log_bme280_average(sensor.address(), reading, average.count);
                        result = Some(reading);
                    }
                    None => log!("[bme ] no valid samples in active phase"),
                }
            }
            Err(err) => log!("[bme ] init failed: {:?}", err),
        }
    }

    power.set_high();
    log!("[phase] BME off PB2=high");
    result
}

#[derive(Default)]
struct Bme280Average {
    temperature_sum: i64,
    pressure_sum: u64,
    humidity_sum: u64,
    count: u32,
    is_bmp280: bool,
}

impl Bme280Average {
    fn add(&mut self, reading: bme280::Reading) {
        self.temperature_sum += i64::from(reading.temperature_centidegrees);
        self.pressure_sum += u64::from(reading.pressure_pa);
        self.humidity_sum += u64::from(reading.humidity_q1024);
        self.count += 1;
        self.is_bmp280 = reading.is_bmp280;
    }

    fn finish(&self) -> Option<bme280::Reading> {
        if self.count == 0 {
            return None;
        }

        let count_i64 = i64::from(self.count);
        let count_u64 = u64::from(self.count);
        Some(bme280::Reading {
            temperature_centidegrees: (self.temperature_sum / count_i64) as i32,
            pressure_pa: (self.pressure_sum / count_u64) as u32,
            humidity_q1024: (self.humidity_sum / count_u64) as u32,
            is_bmp280: self.is_bmp280,
        })
    }
}

fn log_bme280_average(address: u8, reading: bme280::Reading, samples: u32) {
    let temp_abs = reading.temperature_centidegrees.unsigned_abs();
    let temp_sign = if reading.temperature_centidegrees < 0 {
        "-"
    } else {
        ""
    };
    let pressure_hpa = reading.pressure_pa / 100;
    let pressure_frac = reading.pressure_pa % 100;

    if reading.is_bmp280 {
        log!(
            "[bme ] avg {} samples addr=0x{:02X} temp={}{}.{}C pressure={}.{:02}hPa humidity=n/a",
            samples,
            address,
            temp_sign,
            temp_abs / 100,
            (temp_abs % 100) / 10,
            pressure_hpa,
            pressure_frac
        );
    } else {
        let humidity_x10 = (reading.humidity_q1024 * 10) / 1024;
        log!(
            "[bme ] avg {} samples addr=0x{:02X} temp={}{}.{}C pressure={}.{:02}hPa humidity={}.{}%",
            samples,
            address,
            temp_sign,
            temp_abs / 100,
            (temp_abs % 100) / 10,
            pressure_hpa,
            pressure_frac,
            humidity_x10 / 10,
            humidity_x10 % 10
        );
    }
}

async fn run_zts3000_phase(
    rs485: &mut Uart<'static, embassy_stm32::mode::Async>,
    power: &mut Output<'static>,
) -> Option<Zts3000Reading> {
    power.set_low();
    log!(
        "[phase] RS485 on PB0=low for {}s",
        SENSOR_ACTIVE_PHASE_MS / 1000
    );
    let phase_deadline = Instant::now() + Duration::from_millis(SENSOR_ACTIVE_PHASE_MS);
    Timer::after_millis(RS485_POWER_SETTLE_MS).await;

    let mut average = Zts3000Average::default();

    while Instant::now() < phase_deadline {
        match read_zts3000(rs485).await {
            Ok(reading) => average.add(reading),
            Err(err) => log!("[rs485] sample failed: {:?}", err),
        }
        Timer::after_millis(SENSOR_SAMPLE_INTERVAL_MS).await;
    }

    let result = match average.finish() {
        Some(reading) => {
            log_zts3000_average(reading, average.count);
            Some(reading)
        }
        None => {
            log!("[rs485] no valid samples in active phase");
            None
        }
    };

    power.set_high();
    log!("[phase] RS485 off PB0=high");
    result
}

async fn read_zts3000(
    rs485: &mut Uart<'static, embassy_stm32::mode::Async>,
) -> Result<Zts3000Reading, Zts3000ReadError> {
    rs485
        .write(&ZTS3000_READ_HUM_TEMP)
        .await
        .map_err(Zts3000ReadError::Uart)?;
    rs485.flush().await.map_err(Zts3000ReadError::Uart)?;

    let mut response = [0u8; 32];
    let len = match with_timeout(
        Duration::from_millis(700),
        rs485.read_until_idle(&mut response),
    )
    .await
    {
        Ok(Ok(len)) => len,
        Ok(Err(err)) => return Err(Zts3000ReadError::Uart(err)),
        Err(_) => return Err(Zts3000ReadError::Timeout),
    };

    parse_zts3000_response(&response[..len]).map_err(Zts3000ReadError::Parse)
}

enum Zts3000ReadError {
    Uart(usart::Error),
    Timeout,
    Parse(Zts3000ParseError),
}

impl fmt::Debug for Zts3000ReadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Uart(err) => f.debug_tuple("Uart").field(err).finish(),
            Self::Timeout => f.write_str("Timeout"),
            Self::Parse(err) => f.debug_tuple("Parse").field(err).finish(),
        }
    }
}

#[derive(Default)]
struct Zts3000Average {
    humidity_sum: u32,
    temperature_sum: i32,
    count: u32,
}

impl Zts3000Average {
    fn add(&mut self, reading: Zts3000Reading) {
        self.humidity_sum += u32::from(reading.humidity_tenths);
        self.temperature_sum += i32::from(reading.temperature_tenths);
        self.count += 1;
    }

    fn finish(&self) -> Option<Zts3000Reading> {
        if self.count == 0 {
            return None;
        }

        Some(Zts3000Reading {
            humidity_tenths: (self.humidity_sum / self.count) as u16,
            temperature_tenths: (self.temperature_sum / self.count as i32) as i16,
        })
    }
}

#[derive(Clone, Copy)]
struct Zts3000Reading {
    humidity_tenths: u16,
    temperature_tenths: i16,
}

#[derive(Debug)]
enum Zts3000ParseError {
    ShortFrame,
    LongFrame,
    WrongAddress,
    WrongFunction,
    WrongByteCount,
    Crc,
}

fn parse_zts3000_response(frame: &[u8]) -> Result<Zts3000Reading, Zts3000ParseError> {
    if frame.len() < ZTS3000_RESPONSE_LEN {
        return Err(Zts3000ParseError::ShortFrame);
    }
    if frame.len() > ZTS3000_RESPONSE_LEN {
        return Err(Zts3000ParseError::LongFrame);
    }

    if !modbus_crc_is_valid(frame) {
        return Err(Zts3000ParseError::Crc);
    }
    if frame[0] != 0x01 {
        return Err(Zts3000ParseError::WrongAddress);
    }
    if frame[1] != 0x03 {
        return Err(Zts3000ParseError::WrongFunction);
    }
    if frame[2] != 0x04 {
        return Err(Zts3000ParseError::WrongByteCount);
    }

    Ok(Zts3000Reading {
        humidity_tenths: u16::from_be_bytes([frame[3], frame[4]]),
        temperature_tenths: i16::from_be_bytes([frame[5], frame[6]]),
    })
}

fn modbus_crc_is_valid(frame: &[u8]) -> bool {
    if frame.len() < 2 {
        return false;
    }

    let data_len = frame.len() - 2;
    let expected = modbus_crc16(&frame[..data_len]);
    let actual = u16::from_le_bytes([frame[data_len], frame[data_len + 1]]);
    expected == actual
}

fn modbus_crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0xffffu16;

    for byte in bytes {
        crc ^= u16::from(*byte);
        for _ in 0..8 {
            if crc & 0x0001 != 0 {
                crc = (crc >> 1) ^ 0xa001;
            } else {
                crc >>= 1;
            }
        }
    }

    crc
}

fn log_zts3000_average(reading: Zts3000Reading, samples: u32) {
    let humidity_whole = reading.humidity_tenths / 10;
    let humidity_frac = reading.humidity_tenths % 10;
    let temperature_abs = reading.temperature_tenths.unsigned_abs();
    let temperature_sign = if reading.temperature_tenths < 0 {
        "-"
    } else {
        ""
    };

    log!(
        "[rs485] avg {} samples humidity={}.{}% temperature={}{}.{}C",
        samples,
        humidity_whole,
        humidity_frac,
        temperature_sign,
        temperature_abs / 10,
        temperature_abs % 10
    );
}

fn get_bd_addr() -> BdAddr {
    let lhci_info = LhciC1DeviceInformationCcrp::new();
    BdAddr([
        (lhci_info.uid64 & 0xff) as u8,
        ((lhci_info.uid64 >> 8) & 0xff) as u8,
        ((lhci_info.uid64 >> 16) & 0xff) as u8,
        lhci_info.device_type_id,
        (lhci_info.st_company_id & 0xff) as u8,
        ((lhci_info.st_company_id >> 8) & 0xff) as u8,
    ])
}

fn get_random_addr() -> BdAddr {
    let lhci_info = LhciC1DeviceInformationCcrp::new();
    BdAddr([
        (lhci_info.uid64 & 0xff) as u8,
        ((lhci_info.uid64 >> 8) & 0xff) as u8,
        ((lhci_info.uid64 >> 16) & 0xff) as u8,
        0,
        0x6E,
        0xED,
    ])
}

fn get_irk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_IRK)
}

fn get_erk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_ERK)
}
