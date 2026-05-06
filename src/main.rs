#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt;
use core::time::Duration as CoreDuration;

use cortex_m::interrupt::Mutex;
use embassy_executor::Spawner;
use embassy_stm32::flash::{Blocking, Error as FlashError, Flash};
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
use embassy_stm32_wpan::hci::vendor::event::{AttributeHandle, VendorEvent};
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
const SETUP_DEVICE_NAME: &[u8] = b"HH-PROBE-SETUP";
const PROBE_UUID: &str = env!("PROBE_BUILD_UUID");
const BLE_GAP_DEVICE_NAME_LENGTH: u8 = if SETUP_DEVICE_NAME.len() > DEVICE_NAME.len() {
    SETUP_DEVICE_NAME.len() as u8
} else {
    DEVICE_NAME.len() as u8
};
const PROBE_VERSION_MAJOR: u8 = 1;
const PROBE_VERSION_MINOR: u8 = 0;
const HUB_COMPANY_ID_LE: [u8; 2] = 0x1234_u16.to_le_bytes();
const PROBE_ADV_DATA: [u8; 14] = [
    13,
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
];
const PROBE_SCAN_RESPONSE_DATA: [u8; 12] = [
    11, 0x09, b'H', b'H', b'-', b'P', b'R', b'O', b'B', b'E', b'-', b'A',
];
const SETUP_PROBE_ADV_DATA: [u8; 14] = [
    13,
    0xff,
    HUB_COMPANY_ID_LE[0],
    HUB_COMPANY_ID_LE[1],
    b'H',
    b'H',
    b'-',
    b'S',
    b'E',
    b'T',
    b'U',
    b'P',
    PROBE_VERSION_MAJOR,
    PROBE_VERSION_MINOR,
];
const SETUP_PROBE_SCAN_RESPONSE_DATA: [u8; 16] = [
    15, 0x09, b'H', b'H', b'-', b'P', b'R', b'O', b'B', b'E', b'-', b'S', b'E', b'T', b'U', b'P',
];
const ENVIRONMENTAL_SENSING_SERVICE_UUID: u16 = 0x181a;
const TEMPERATURE_CHAR_UUID: u16 = 0x2a6e;
const PRESSURE_CHAR_UUID: u16 = 0x2a6d;
const HUMIDITY_CHAR_UUID: u16 = 0x2a6f;
// Embassy STM32 WPAN passes 128-bit UUIDs to the ST BLE stack in little-endian
// byte order. These publish as 1234000x-0000-1000-8000-00805f9b34fb.
const PROBE_UUID_CHAR_UUID: [u8; 16] = [
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x34, 0x12,
];
const SOIL_TEMPERATURE_CHAR_UUID: [u8; 16] = [
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x03, 0x00, 0x34, 0x12,
];
const SOIL_HUMIDITY_CHAR_UUID: [u8; 16] = [
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x04, 0x00, 0x34, 0x12,
];
const SETUP_CONFIRM_CHAR_UUID: [u8; 16] = [
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x05, 0x00, 0x34, 0x12,
];
const MOTOR_COMMAND_CHAR_UUID: [u8; 16] = [
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x06, 0x00, 0x34, 0x12,
];

// Motor command write payload layout (little-endian fixed-width fields):
// [0..4]   magic      = MOTOR_COMMAND_PAYLOAD_MAGIC
// [4]      version    = MOTOR_COMMAND_PAYLOAD_VERSION
// [5]      action     = MOTOR_COMMAND_ACTION_*
// [6..22]  command_id = 16-byte command identifier (compact UUID bytes)
// [22..26] duration   = requested motor run duration in ms (u32 LE)
// [26..30] expires_at = remaining TTL in ms at hub write-time (u32 LE)
// Safety defaults for later command handlers:
// - Probe keeps only one active command at a time.
// - Duplicate command_id values are ignored for MOTOR_COMMAND_DUPLICATE_RETENTION_MS.
// - Duration is clamped to MOTOR_COMMAND_MAX_DURATION_MS.
// - Expired commands must be ignored.
const MOTOR_COMMAND_PAYLOAD_MAGIC: &[u8; 4] = b"HHMC";
const MOTOR_COMMAND_PAYLOAD_VERSION: u8 = 1;
const MOTOR_COMMAND_ACTION_STOP: u8 = 0;
const MOTOR_COMMAND_ACTION_RUN_FOR_DURATION: u8 = 1;
const MOTOR_COMMAND_PAYLOAD_MAGIC_OFFSET: usize = 0;
const MOTOR_COMMAND_PAYLOAD_VERSION_OFFSET: usize = 4;
const MOTOR_COMMAND_PAYLOAD_ACTION_OFFSET: usize = 5;
const MOTOR_COMMAND_PAYLOAD_COMMAND_ID_OFFSET: usize = 6;
const MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN: usize = 16;
const MOTOR_COMMAND_PAYLOAD_DURATION_MS_OFFSET: usize = 22;
const MOTOR_COMMAND_PAYLOAD_EXPIRY_MS_OFFSET: usize = 26;
const MOTOR_COMMAND_PAYLOAD_LEN: usize = 30;
const MOTOR_COMMAND_MAX_DURATION_MS: u32 = 5_000;
const MOTOR_COMMAND_DEFAULT_EXPIRY_MS: u32 = 80_000;
const MOTOR_COMMAND_DUPLICATE_RETENTION_MS: u32 = MOTOR_COMMAND_DEFAULT_EXPIRY_MS;
const MOTOR_COMMAND_ACCEPTED_HISTORY_LEN: usize = 4;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotorCommandAction {
    Stop,
    RunForDuration,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MotorCommandPayload {
    action: MotorCommandAction,
    command_id: [u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
    duration_ms: u32,
    expires_after_ms: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotorCommandValidationError {
    InvalidLength,
    InvalidMagic,
    UnsupportedVersion,
    UnknownAction,
    DurationTooLong,
    Expired,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotorCommandWriteOutcome {
    Accepted(MotorCommandPayload),
    Duplicate,
    DuplicateActive,
    Invalid(MotorCommandValidationError),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct AcceptedMotorCommandEntry {
    command_id: [u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
    accepted_at_ms: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PendingMotorCommandRequest {
    payload: MotorCommandPayload,
    accepted_at_ms: u32,
}

#[derive(Clone, Copy, Debug)]
struct MotorCommandState {
    pending_request: Option<PendingMotorCommandRequest>,
    accepted_history: [Option<AcceptedMotorCommandEntry>; MOTOR_COMMAND_ACCEPTED_HISTORY_LEN],
}

impl MotorCommandState {
    const fn new() -> Self {
        Self {
            pending_request: None,
            accepted_history: [None; MOTOR_COMMAND_ACCEPTED_HISTORY_LEN],
        }
    }

    fn handle_write_payload(
        &mut self,
        payload: &[u8],
        now_ms: u32,
        active_command_id: Option<&[u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN]>,
    ) -> MotorCommandWriteOutcome {
        let parsed = match parse_motor_command_payload(payload, now_ms) {
            Ok(command) => command,
            Err(err) => return MotorCommandWriteOutcome::Invalid(err),
        };

        if active_command_id
            .map(|current| *current == parsed.command_id)
            .unwrap_or(false)
        {
            return MotorCommandWriteOutcome::DuplicateActive;
        }

        if self
            .pending_request
            .as_ref()
            .map(|pending| pending.payload.command_id == parsed.command_id)
            .unwrap_or(false)
        {
            return MotorCommandWriteOutcome::DuplicateActive;
        }

        self.prune_expired_history(now_ms);
        if self.is_duplicate(&parsed.command_id, now_ms) {
            return MotorCommandWriteOutcome::Duplicate;
        }

        self.record_accepted(&parsed, now_ms);
        MotorCommandWriteOutcome::Accepted(parsed)
    }

    fn prune_expired_history(&mut self, now_ms: u32) {
        for slot in &mut self.accepted_history {
            if let Some(entry) = slot {
                let age_ms = now_ms.wrapping_sub(entry.accepted_at_ms);
                if age_ms > MOTOR_COMMAND_DUPLICATE_RETENTION_MS {
                    *slot = None;
                }
            }
        }
    }

    fn is_duplicate(
        &self,
        command_id: &[u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
        now_ms: u32,
    ) -> bool {
        self.accepted_history.iter().any(|entry| {
            entry
                .as_ref()
                .map(|accepted| {
                    accepted.command_id == *command_id
                        && now_ms.wrapping_sub(accepted.accepted_at_ms)
                            <= MOTOR_COMMAND_DUPLICATE_RETENTION_MS
                })
                .unwrap_or(false)
        })
    }

    fn record_accepted(&mut self, payload: &MotorCommandPayload, now_ms: u32) {
        self.pending_request = Some(PendingMotorCommandRequest {
            payload: *payload,
            accepted_at_ms: now_ms,
        });

        let entry = AcceptedMotorCommandEntry {
            command_id: payload.command_id,
            accepted_at_ms: now_ms,
        };

        if let Some(slot) = self.accepted_history.iter_mut().find(|slot| slot.is_none()) {
            *slot = Some(entry);
            return;
        }

        let oldest_index = self
            .accepted_history
            .iter()
            .enumerate()
            .max_by_key(|(_, slot)| {
                slot.map(|accepted| now_ms.wrapping_sub(accepted.accepted_at_ms))
                    .unwrap_or(0)
            })
            .map(|(index, _)| index)
            .unwrap_or(0);
        self.accepted_history[oldest_index] = Some(entry);
    }
}

fn now_ms_u32() -> u32 {
    Instant::now().as_millis() as u32
}

fn parse_motor_command_payload(
    payload: &[u8],
    _now_ms: u32,
) -> Result<MotorCommandPayload, MotorCommandValidationError> {
    if payload.len() != MOTOR_COMMAND_PAYLOAD_LEN {
        return Err(MotorCommandValidationError::InvalidLength);
    }

    if &payload[MOTOR_COMMAND_PAYLOAD_MAGIC_OFFSET..MOTOR_COMMAND_PAYLOAD_VERSION_OFFSET]
        != MOTOR_COMMAND_PAYLOAD_MAGIC
    {
        return Err(MotorCommandValidationError::InvalidMagic);
    }

    if payload[MOTOR_COMMAND_PAYLOAD_VERSION_OFFSET] != MOTOR_COMMAND_PAYLOAD_VERSION {
        return Err(MotorCommandValidationError::UnsupportedVersion);
    }

    let action = match payload[MOTOR_COMMAND_PAYLOAD_ACTION_OFFSET] {
        MOTOR_COMMAND_ACTION_STOP => MotorCommandAction::Stop,
        MOTOR_COMMAND_ACTION_RUN_FOR_DURATION => MotorCommandAction::RunForDuration,
        _ => return Err(MotorCommandValidationError::UnknownAction),
    };

    let mut command_id = [0u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN];
    command_id.copy_from_slice(
        &payload[MOTOR_COMMAND_PAYLOAD_COMMAND_ID_OFFSET..MOTOR_COMMAND_PAYLOAD_DURATION_MS_OFFSET],
    );

    let duration_ms = u32::from_le_bytes(
        payload[MOTOR_COMMAND_PAYLOAD_DURATION_MS_OFFSET..MOTOR_COMMAND_PAYLOAD_EXPIRY_MS_OFFSET]
            .try_into()
            .expect("duration slice has fixed width"),
    );
    if duration_ms > MOTOR_COMMAND_MAX_DURATION_MS {
        return Err(MotorCommandValidationError::DurationTooLong);
    }

    let expires_after_ms = u32::from_le_bytes(
        payload[MOTOR_COMMAND_PAYLOAD_EXPIRY_MS_OFFSET..MOTOR_COMMAND_PAYLOAD_LEN]
            .try_into()
            .expect("expiry slice has fixed width"),
    );
    if expires_after_ms == 0 {
        return Err(MotorCommandValidationError::Expired);
    }

    Ok(MotorCommandPayload {
        action,
        command_id,
        duration_ms,
        expires_after_ms,
    })
}

// Motor command UART frame marker/prefix. This binary envelope is intentionally
// distinct from human-readable debug UART log lines emitted through `log!`.
// TODO(motor-uart-adapter): finalize full on-wire framing/checksum once motor
// firmware protocol contract is provided.
const MOTOR_UART_FRAME_MARKER: &[u8; 4] = b"HHMC";
const MOTOR_UART_FRAME_VERSION: u8 = 1;
const MOTOR_UART_FRAME_PREFIX_LEN: usize = 5;
const MOTOR_UART_RESPONSE_TIMEOUT_MS: u64 = 150;
const MOTOR_UART_ACTION_OFFSET: usize = MOTOR_UART_FRAME_PREFIX_LEN;
const MOTOR_UART_COMMAND_ID_OFFSET: usize = MOTOR_UART_ACTION_OFFSET + 1;
const MOTOR_UART_DURATION_OFFSET: usize =
    MOTOR_UART_COMMAND_ID_OFFSET + MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN;
const MOTOR_UART_FRAME_LEN: usize = MOTOR_UART_DURATION_OFFSET + 4;
const MOTOR_UART_ACK_STATUS_OFFSET: usize = MOTOR_UART_FRAME_PREFIX_LEN;
const MOTOR_UART_ACK_MIN_LEN: usize = MOTOR_UART_FRAME_PREFIX_LEN + 1;
const MOTOR_UART_ACK_STATUS_SUCCESS: u8 = 0;

type SharedLpuart1 = Uart<'static, embassy_stm32::mode::Async>;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotorUartResult {
    Success,
    Rejected { status: u8 },
    InvalidResponse,
    Timeout,
    UartError,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotorDispatchOutcome {
    RunStarted {
        command_id: [u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
        duration_ms: u32,
    },
    Stopped {
        command_id: [u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
    },
    Failed {
        command_id: [u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
        action: MotorCommandAction,
        result: MotorUartResult,
    },
    ExpiredBeforeDispatch {
        command_id: [u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotorTimeoutFailsafeAction {
    SendBestEffortStop,
    NoImmediateStop,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MotorUartSimulationMatrix {
    valid_ack: MotorUartResult,
    rejected_ack: MotorUartResult,
    invalid_ack: MotorUartResult,
    timeout_failsafe: MotorTimeoutFailsafeAction,
    marker_distinct_from_zts3000_request: bool,
}

struct MotorUartAdapter;

impl MotorUartAdapter {
    // NOTE: Motor UART currently shares the same physical LPUART1 peripheral used
    // for soil RS485 wiring (PA2/PA3/PB1) on this hardware revision. Isolation is
    // enforced logically by dedicated motor frame encode/parse helpers, never by
    // reusing soil Modbus request/parse paths.
    async fn run_motor_for_duration(
        uart: &mut SharedLpuart1,
        command_id: &[u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
        duration_ms: u32,
    ) -> MotorUartResult {
        let clamped_duration_ms = duration_ms.min(MOTOR_COMMAND_MAX_DURATION_MS);
        Self::send_command(
            uart,
            MOTOR_COMMAND_ACTION_RUN_FOR_DURATION,
            command_id,
            clamped_duration_ms,
        )
        .await
    }

    async fn stop_motor(
        uart: &mut SharedLpuart1,
        command_id: &[u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
    ) -> MotorUartResult {
        Self::send_command(uart, MOTOR_COMMAND_ACTION_STOP, command_id, 0).await
    }

    async fn send_command(
        uart: &mut SharedLpuart1,
        action: u8,
        command_id: &[u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
        duration_ms: u32,
    ) -> MotorUartResult {
        log!(
            "[motor] UART command write start command_id={:?} action={} duration_ms={} frame_marker={:?}",
            command_id,
            action,
            duration_ms,
            MOTOR_UART_FRAME_MARKER,
        );
        let frame = encode_motor_uart_frame(action, command_id, duration_ms);

        if uart.write(&frame).await.is_err() {
            log!(
                "[motor] UART command write failed command_id={:?} action={} reason_code=UART_TIMEOUT stage=write",
                command_id,
                action,
            );
            return MotorUartResult::UartError;
        }
        if uart.flush().await.is_err() {
            log!(
                "[motor] UART command flush failed command_id={:?} action={} reason_code=UART_TIMEOUT stage=flush",
                command_id,
                action,
            );
            return MotorUartResult::UartError;
        }

        let mut ack = [0u8; 32];
        let ack_len = match with_timeout(
            Duration::from_millis(MOTOR_UART_RESPONSE_TIMEOUT_MS),
            uart.read_until_idle(&mut ack),
        )
        .await
        {
            Ok(Ok(len)) => len,
            Ok(Err(_)) => {
                log!(
                    "[motor] UART ack read failed command_id={:?} action={} reason_code=UART_TIMEOUT stage=read",
                    command_id,
                    action,
                );
                return MotorUartResult::UartError;
            }
            Err(_) => {
                log!(
                    "[motor] UART ack timeout command_id={:?} action={} reason_code=UART_TIMEOUT timeout_ms={}",
                    command_id,
                    action,
                    MOTOR_UART_RESPONSE_TIMEOUT_MS,
                );
                return MotorUartResult::Timeout;
            }
        };

        let parsed = parse_motor_uart_ack(&ack[..ack_len]);
        log!(
            "[motor] UART ack parsed command_id={:?} action={} result={:?} reason_code={}",
            command_id,
            action,
            parsed,
            motor_reason_code_for_uart_result(parsed),
        );
        parsed
    }
}

struct MotorWatchdogState {
    active: Option<MotorActiveCommand>,
}

#[derive(Clone, Copy)]
struct MotorActiveCommand {
    command_id: [u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
    deadline: Instant,
}

impl MotorWatchdogState {
    const fn new() -> Self {
        Self { active: None }
    }

    fn arm(&mut self, command_id: [u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN], duration_ms: u32) {
        let clamped_duration_ms = duration_ms.min(MOTOR_COMMAND_MAX_DURATION_MS);
        self.active = Some(MotorActiveCommand {
            command_id,
            deadline: Instant::now() + Duration::from_millis(clamped_duration_ms as u64),
        });
    }

    fn disarm(&mut self) {
        self.active = None;
    }

    fn active_command_id(&self) -> Option<&[u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN]> {
        self.active.as_ref().map(|active| &active.command_id)
    }

    async fn enforce_elapsed(&mut self, motor_uart: &mut SharedLpuart1) -> Option<MotorUartResult> {
        let Some(active) = self.active else {
            return None;
        };

        if Instant::now() < active.deadline {
            return None;
        }

        let stop_result = MotorUartAdapter::stop_motor(motor_uart, &active.command_id).await;
        self.disarm();
        Some(stop_result)
    }
}

fn encode_motor_uart_frame(
    action: u8,
    command_id: &[u8; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN],
    duration_ms: u32,
) -> [u8; MOTOR_UART_FRAME_LEN] {
    let mut frame = [0u8; MOTOR_UART_FRAME_LEN];
    frame[..MOTOR_UART_FRAME_MARKER.len()].copy_from_slice(MOTOR_UART_FRAME_MARKER);
    frame[MOTOR_UART_FRAME_MARKER.len()] = MOTOR_UART_FRAME_VERSION;
    frame[MOTOR_UART_ACTION_OFFSET] = action;
    frame[MOTOR_UART_COMMAND_ID_OFFSET..MOTOR_UART_DURATION_OFFSET].copy_from_slice(command_id);
    frame[MOTOR_UART_DURATION_OFFSET..MOTOR_UART_FRAME_LEN]
        .copy_from_slice(&duration_ms.to_le_bytes());
    frame
}

fn parse_motor_uart_ack(ack: &[u8]) -> MotorUartResult {
    if ack.len() < MOTOR_UART_ACK_MIN_LEN {
        return MotorUartResult::InvalidResponse;
    }

    if !ack.starts_with(MOTOR_UART_FRAME_MARKER) {
        return MotorUartResult::InvalidResponse;
    }

    if ack[MOTOR_UART_FRAME_MARKER.len()] != MOTOR_UART_FRAME_VERSION {
        return MotorUartResult::InvalidResponse;
    }

    let status = ack[MOTOR_UART_ACK_STATUS_OFFSET];
    if status == MOTOR_UART_ACK_STATUS_SUCCESS {
        MotorUartResult::Success
    } else {
        MotorUartResult::Rejected { status }
    }
}

async fn dispatch_pending_motor_command(
    motor_command_state: &mut MotorCommandState,
    motor_watchdog: &mut MotorWatchdogState,
    motor_uart: &mut SharedLpuart1,
) -> Option<MotorDispatchOutcome> {
    let pending = motor_command_state.pending_request.take()?;
    let age_ms = now_ms_u32().wrapping_sub(pending.accepted_at_ms);
    if age_ms >= pending.payload.expires_after_ms {
        log!(
            "[motor] pending command expired before UART dispatch command_id={:?} reason_code=EXPIRED age_ms={} expires_after_ms={}",
            pending.payload.command_id,
            age_ms,
            pending.payload.expires_after_ms,
        );
        return Some(MotorDispatchOutcome::ExpiredBeforeDispatch {
            command_id: pending.payload.command_id,
        });
    }

    match pending.payload.action {
        MotorCommandAction::Stop => {
            let result =
                MotorUartAdapter::stop_motor(motor_uart, &pending.payload.command_id).await;
            if result == MotorUartResult::Success {
                motor_watchdog.disarm();
                Some(MotorDispatchOutcome::Stopped {
                    command_id: pending.payload.command_id,
                })
            } else {
                Some(MotorDispatchOutcome::Failed {
                    command_id: pending.payload.command_id,
                    action: pending.payload.action,
                    result,
                })
            }
        }
        MotorCommandAction::RunForDuration => {
            let clamped_duration_ms = pending
                .payload
                .duration_ms
                .min(MOTOR_COMMAND_MAX_DURATION_MS);
            let result = MotorUartAdapter::run_motor_for_duration(
                motor_uart,
                &pending.payload.command_id,
                clamped_duration_ms,
            )
            .await;

            if result == MotorUartResult::Success {
                motor_watchdog.arm(pending.payload.command_id, clamped_duration_ms);
                Some(MotorDispatchOutcome::RunStarted {
                    command_id: pending.payload.command_id,
                    duration_ms: clamped_duration_ms,
                })
            } else {
                if timeout_failsafe_action_for_result(result)
                    == MotorTimeoutFailsafeAction::SendBestEffortStop
                {
                    let _ =
                        MotorUartAdapter::stop_motor(motor_uart, &pending.payload.command_id).await;
                    motor_watchdog.disarm();
                }
                Some(MotorDispatchOutcome::Failed {
                    command_id: pending.payload.command_id,
                    action: pending.payload.action,
                    result,
                })
            }
        }
    }
}

async fn idle_with_motor_watchdog(
    duration_ms: u64,
    motor_watchdog: &mut MotorWatchdogState,
    motor_uart: &mut SharedLpuart1,
) {
    let deadline = Instant::now() + Duration::from_millis(duration_ms);

    loop {
        if let Some(stop_result) = motor_watchdog.enforce_elapsed(motor_uart).await {
            log!(
                "[motor] watchdog stop enforced during idle result={:?} reason_code={} watchdog_event=elapsed_duration",
                stop_result,
                motor_reason_code_for_uart_result(stop_result),
            );
        }

        let now = Instant::now();
        if now >= deadline {
            return;
        }

        let remaining_ms = deadline.saturating_duration_since(now).as_millis() as u64;
        let sleep_chunk_ms = remaining_ms.min(100);
        Timer::after_millis(sleep_chunk_ms).await;
    }
}

fn timeout_failsafe_action_for_result(result: MotorUartResult) -> MotorTimeoutFailsafeAction {
    if result == MotorUartResult::Timeout {
        MotorTimeoutFailsafeAction::SendBestEffortStop
    } else {
        MotorTimeoutFailsafeAction::NoImmediateStop
    }
}

fn motor_reason_code_for_uart_result(result: MotorUartResult) -> &'static str {
    match result {
        MotorUartResult::Success => "NONE",
        MotorUartResult::Timeout | MotorUartResult::UartError => "UART_TIMEOUT",
        MotorUartResult::Rejected { .. } | MotorUartResult::InvalidResponse => "UART_REJECTED",
    }
}

fn simulate_motor_uart_ack_and_failsafe_matrix() -> MotorUartSimulationMatrix {
    let mut valid_ack = [0u8; MOTOR_UART_ACK_MIN_LEN];
    valid_ack[..MOTOR_UART_FRAME_MARKER.len()].copy_from_slice(MOTOR_UART_FRAME_MARKER);
    valid_ack[MOTOR_UART_FRAME_MARKER.len()] = MOTOR_UART_FRAME_VERSION;
    valid_ack[MOTOR_UART_ACK_STATUS_OFFSET] = MOTOR_UART_ACK_STATUS_SUCCESS;

    let mut rejected_ack = valid_ack;
    rejected_ack[MOTOR_UART_ACK_STATUS_OFFSET] = 7;

    let invalid_ack = [0x00, 0x11, 0x22, 0x33, MOTOR_UART_FRAME_VERSION, 0x00];

    let marker_distinct_from_zts3000_request =
        MOTOR_UART_FRAME_MARKER != &ZTS3000_READ_HUM_TEMP[..MOTOR_UART_FRAME_MARKER.len()];

    MotorUartSimulationMatrix {
        valid_ack: parse_motor_uart_ack(&valid_ack),
        rejected_ack: parse_motor_uart_ack(&rejected_ack),
        invalid_ack: parse_motor_uart_ack(&invalid_ack),
        timeout_failsafe: timeout_failsafe_action_for_result(MotorUartResult::Timeout),
        marker_distinct_from_zts3000_request,
    }
}

fn motor_uart_simulation_matrix_is_expected(matrix: MotorUartSimulationMatrix) -> bool {
    matrix.valid_ack == MotorUartResult::Success
        && matrix.rejected_ack == MotorUartResult::Rejected { status: 7 }
        && matrix.invalid_ack == MotorUartResult::InvalidResponse
        && matrix.timeout_failsafe == MotorTimeoutFailsafeAction::SendBestEffortStop
        && matrix.marker_distinct_from_zts3000_request
}

fn simulate_motor_command_duplicate_and_expiry_rules() -> bool {
    let now_ms = 10_000u32;
    let expires_after_ms = 2_000u32;
    let duration_ms = 1_000u32;
    let command_id = [0xAB; MOTOR_COMMAND_PAYLOAD_COMMAND_ID_LEN];

    let payload = MotorCommandPayload {
        action: MotorCommandAction::RunForDuration,
        command_id,
        duration_ms,
        expires_after_ms,
    };

    let mut encoded = [0u8; MOTOR_COMMAND_PAYLOAD_LEN];
    encoded[MOTOR_COMMAND_PAYLOAD_MAGIC_OFFSET..MOTOR_COMMAND_PAYLOAD_VERSION_OFFSET]
        .copy_from_slice(MOTOR_COMMAND_PAYLOAD_MAGIC);
    encoded[MOTOR_COMMAND_PAYLOAD_VERSION_OFFSET] = MOTOR_COMMAND_PAYLOAD_VERSION;
    encoded[MOTOR_COMMAND_PAYLOAD_ACTION_OFFSET] = MOTOR_COMMAND_ACTION_RUN_FOR_DURATION;
    encoded[MOTOR_COMMAND_PAYLOAD_COMMAND_ID_OFFSET..MOTOR_COMMAND_PAYLOAD_DURATION_MS_OFFSET]
        .copy_from_slice(&payload.command_id);
    encoded[MOTOR_COMMAND_PAYLOAD_DURATION_MS_OFFSET..MOTOR_COMMAND_PAYLOAD_EXPIRY_MS_OFFSET]
        .copy_from_slice(&payload.duration_ms.to_le_bytes());
    encoded[MOTOR_COMMAND_PAYLOAD_EXPIRY_MS_OFFSET..MOTOR_COMMAND_PAYLOAD_LEN]
        .copy_from_slice(&payload.expires_after_ms.to_le_bytes());

    let mut state = MotorCommandState::new();
    let first = state.handle_write_payload(&encoded, now_ms, None);
    let duplicate_pending = state.handle_write_payload(&encoded, now_ms + 1, None);
    let duplicate_active = state.handle_write_payload(&encoded, now_ms + 2, Some(&command_id));

    let pending = PendingMotorCommandRequest {
        payload,
        accepted_at_ms: now_ms,
    };
    let age_ms = (now_ms + expires_after_ms + 1).wrapping_sub(pending.accepted_at_ms);
    let expired_before_dispatch = age_ms >= pending.payload.expires_after_ms;

    first == MotorCommandWriteOutcome::Accepted(payload)
        && duplicate_pending == MotorCommandWriteOutcome::DuplicateActive
        && duplicate_active == MotorCommandWriteOutcome::DuplicateActive
        && expired_before_dispatch
}

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
const BLE_COMMAND_TIMEOUT_MS: u64 = 5_000;
const SETUP_FLAG_PAGE_OFFSET: u32 = 188 * 1024;
const SETUP_FLAG_PAGE_SIZE: u32 = 4 * 1024;
const SETUP_COMPLETE_MAGIC: &[u8; 8] = b"HHSETUP1";
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
    let mut flash = Flash::new_blocking(p.FLASH);

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
        advertising_data: &[],
        conn_interval: (None, None),
    };

    let setup_discovery_params = DiscoverableParameters {
        advertising_type: AdvertisingType::ConnectableUndirected,
        advertising_interval: Some((
            CoreDuration::from_millis(250),
            CoreDuration::from_millis(250),
        )),
        address_type: OwnAddressType::Public,
        filter_policy: AdvertisingFilterPolicy::AllowConnectionAndScan,
        local_name: None,
        advertising_data: &[],
        conn_interval: (None, None),
    };

    log!("[ble ] add Environmental Sensing service (0x181A)");
    ble.add_service(&AddServiceParameters {
        uuid: Uuid::Uuid16(ENVIRONMENTAL_SENSING_SERVICE_UUID),
        service_type: ServiceType::Primary,
        max_attribute_records: 24,
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

    log!("[ble ] add setup confirmation characteristic (custom 128-bit)");
    ble.add_characteristic(&AddCharacteristicParameters {
        service_handle: env_service_handle,
        characteristic_uuid: Uuid::Uuid128(SETUP_CONFIRM_CHAR_UUID),
        characteristic_properties: CharacteristicProperty::WRITE,
        characteristic_value_len: SETUP_COMPLETE_MAGIC.len() as u16,
        security_permissions: CharacteristicPermission::empty(),
        gatt_event_mask: CharacteristicEvent::CONFIRM_WRITE,
        encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
        is_variable: false,
    })
    .await;
    let setup_confirm_char_handle = match wait_for_gatt_add_characteristic_complete(&mut ble).await
    {
        Some(handle) => {
            log!("[ble ] setup_confirm char handle: {:?}", handle);
            handle
        }
        None => {
            log!("[ble ] failed to add setup confirmation characteristic");
            return;
        }
    };

    log!("[ble ] add motor command characteristic (custom 128-bit)");
    ble.add_characteristic(&AddCharacteristicParameters {
        service_handle: env_service_handle,
        characteristic_uuid: Uuid::Uuid128(MOTOR_COMMAND_CHAR_UUID),
        characteristic_properties: CharacteristicProperty::WRITE,
        characteristic_value_len: MOTOR_COMMAND_PAYLOAD_LEN as u16,
        security_permissions: CharacteristicPermission::empty(),
        gatt_event_mask: CharacteristicEvent::CONFIRM_WRITE,
        encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
        is_variable: false,
    })
    .await;
    let motor_command_char_handle = match wait_for_gatt_add_characteristic_complete(&mut ble).await
    {
        Some(handle) => {
            log!("[ble ] motor_command char handle: {:?}", handle);
            handle
        }
        None => {
            log!("[ble ] failed to add motor command characteristic");
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
        setup_confirm: setup_confirm_char_handle,
        motor_command: motor_command_char_handle,
    };

    let mut motor_command_state = MotorCommandState::new();

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
    // Physical wiring constraint: there is a single LPUART1 (PA2/PA3/PB1) used
    // for RS485 transceiver access. This handle is shared by:
    // - soil ZTS3000 Modbus requests/responses (`read_zts3000` parser path)
    // - motor UART adapter frames (`encode_motor_uart_frame`/`parse_motor_uart_ack` path)
    // The protocols remain logically isolated and frame formats are distinct.
    let mut shared_lpuart1 = match Uart::new_with_de(
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
    let mut motor_watchdog = MotorWatchdogState::new();
    let motor_simulation = simulate_motor_uart_ack_and_failsafe_matrix();
    log!(
        "[motor] simulation valid_ack={:?} rejected_ack={:?} invalid_ack={:?} timeout_failsafe={:?} marker_distinct={}",
        motor_simulation.valid_ack,
        motor_simulation.rejected_ack,
        motor_simulation.invalid_ack,
        motor_simulation.timeout_failsafe,
        motor_simulation.marker_distinct_from_zts3000_request,
    );
    if !motor_uart_simulation_matrix_is_expected(motor_simulation) {
        log!("[motor] simulation matrix mismatch; refusing to start main lifecycle");
        return;
    }

    if !simulate_motor_command_duplicate_and_expiry_rules() {
        log!("[motor] duplicate/expiry simulation mismatch; refusing to start main lifecycle");
        return;
    }

    if !update_ble_environmental_values(&mut ble, env_service_handle, &gatt_handles, &snapshot)
        .await
    {
        log!("[ble ] initial GATT value update failed");
        return;
    }

    while first_boot_setup_is_pending(&mut flash) {
        log!("[setup] first boot pending; advertising continuously as HH-PROBE-SETUP");
        if !run_setup_advertising_session(&mut ble, &setup_discovery_params, &gatt_handles).await {
            log!("[setup] setup advertising failed; retrying shortly");
            Timer::after_millis(1_000).await;
            continue;
        }

        if mark_first_boot_setup_complete(&mut flash) {
            break;
        }

        log!("[setup] setup flag was not persisted; staying in setup mode");
        Timer::after_millis(1_000).await;
    }

    if !first_boot_setup_is_pending(&mut flash) {
        log!("[ble ] restore normal scan response data");
        match ble
            .le_set_scan_response_data(&PROBE_SCAN_RESPONSE_DATA)
            .await
        {
            Ok(()) => log!("[ble ] normal scan response command sent"),
            Err(_) => log!("[ble ] normal scan response command failed"),
        }
        log!("[ble ] normal scan response {:?}", ble.read().await);
    }

    log!("[setup] first boot complete; normal lifecycle enabled");

    loop {
        log!(
            "[cycle] sleep/idle for {}s before measurement",
            SLEEP_DURATION_MS / 1000
        );
        idle_with_motor_watchdog(SLEEP_DURATION_MS, &mut motor_watchdog, &mut shared_lpuart1).await;

        let bme_reading = run_bme280_phase(&mut i2c_resources, &mut i2c_power).await;
        let soil_reading = run_zts3000_phase(&mut shared_lpuart1, &mut rs485_power).await;
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

        log!("[ble ] update normal advertising data");
        if ble.update_advertising_data(&PROBE_ADV_DATA).await.is_err() {
            log!("[ble ] normal advertising data update command failed");
            continue;
        }
        if !wait_for_gap_update_advertising_data_complete(&mut ble).await {
            log!("[ble ] normal advertising data update indicated failure");
            continue;
        }

        run_advertising_session(
            &mut ble,
            &gatt_handles,
            &mut motor_command_state,
            &mut motor_watchdog,
            &mut shared_lpuart1,
        )
        .await;
    }
}

async fn run_setup_advertising_session(
    ble: &mut (impl UartHci + GapCommands + GattCommands),
    discovery_params: &DiscoverableParameters<'_, '_>,
    gatt_handles: &GattHandles,
) -> bool {
    log!("[ble ] set setup scan response data");
    match ble
        .le_set_scan_response_data(&SETUP_PROBE_SCAN_RESPONSE_DATA)
        .await
    {
        Ok(()) => log!("[ble ] setup scan response command sent"),
        Err(_) => {
            log!("[ble ] setup scan response command failed");
            return false;
        }
    }
    log!("[ble ] setup scan response {:?}", ble.read().await);

    log!("[ble ] set discoverable as HH-PROBE-SETUP");
    if ble.set_discoverable(discovery_params).await.is_err() {
        log!("[ble ] setup discoverable command failed");
        return false;
    }
    if !wait_for_gap_set_discoverable_complete(ble).await {
        log!("[ble ] setup discoverable command-complete indicated failure");
        return false;
    }

    log!("[ble ] update setup advertising data");
    if ble
        .update_advertising_data(&SETUP_PROBE_ADV_DATA)
        .await
        .is_err()
    {
        log!("[ble ] setup advertising data update command failed");
        return false;
    }
    if !wait_for_gap_update_advertising_data_complete(ble).await {
        log!("[ble ] setup advertising data update indicated failure");
        return false;
    }

    log!("[setup] waiting indefinitely for hub pickup");
    let mut isr_delay_count = 0u32;
    let mut active_conn_handle = None;
    let mut conn_deadline = None;
    let mut setup_confirmed = false;

    loop {
        if let (Some(conn_handle), Some(deadline)) = (active_conn_handle, conn_deadline) {
            if Instant::now() >= deadline {
                log!(
                    "[ble ] force terminate stale setup connection {:?}",
                    conn_handle
                );
                if ble
                    .terminate(conn_handle, Status::RemoteTerminationByUser)
                    .await
                    .is_err()
                {
                    log!("[ble ] setup terminate command failed");
                    return false;
                }
                conn_deadline = Some(Instant::now() + Duration::from_millis(BLE_TERMINATE_WAIT_MS));
            }
        }

        match ble.read().await {
            Ok(Packet::Event(event)) => match event {
                Event::HardwareError(HardwareError::IsrDelay) => {
                    isr_delay_count = isr_delay_count.wrapping_add(1);
                    if isr_delay_count == 1 || isr_delay_count.is_multiple_of(32) {
                        log!(
                            "[ble ] setup ISR-delay warnings: {} (UART logging throttled)",
                            isr_delay_count
                        );
                    }
                }
                Event::LeConnectionComplete(conn) if conn.status == Status::Success => {
                    active_conn_handle = Some(conn.conn_handle);
                    conn_deadline =
                        Some(Instant::now() + Duration::from_millis(BLE_CONNECTED_TIMEOUT_MS));
                    log!(
                        "[setup] hub pickup connection handle={:?}; waiting for disconnect",
                        conn.conn_handle
                    );
                }
                Event::LeConnectionComplete(conn) => {
                    log!(
                        "[ble ] setup connection failed with status {:?}",
                        conn.status
                    );
                }
                Event::DisconnectionComplete(disconnection) => {
                    if setup_confirmed {
                        log!(
                            "[setup] pickup complete: disconnected handle={:?} reason={:?}",
                            disconnection.conn_handle,
                            disconnection.reason
                        );
                        return true;
                    }

                    log!(
                        "[setup] disconnect without hub confirmation: handle={:?} reason={:?}; staying in setup mode",
                        disconnection.conn_handle,
                        disconnection.reason
                    );
                    active_conn_handle = None;
                    conn_deadline = None;
                    if ble.set_discoverable(discovery_params).await.is_err() {
                        log!("[ble ] setup rediscoverable command failed");
                        return false;
                    }
                    if !wait_for_gap_set_discoverable_complete(ble).await {
                        log!("[ble ] setup rediscoverable command-complete indicated failure");
                        return false;
                    }
                    log!("[ble ] refresh setup advertising data");
                    if ble
                        .update_advertising_data(&SETUP_PROBE_ADV_DATA)
                        .await
                        .is_err()
                    {
                        log!("[ble ] setup advertising data refresh command failed");
                        return false;
                    }
                    if !wait_for_gap_update_advertising_data_complete(ble).await {
                        log!("[ble ] setup advertising data refresh indicated failure");
                        return false;
                    }
                }
                Event::Vendor(VendorEvent::AttWritePermitRequest(request)) => {
                    let setup_confirm_value_handle = gatt_handles.setup_confirm_value_handle();
                    let motor_command_value_handle = gatt_handles.motor_command_value_handle();
                    let status = if request.attribute_handle == setup_confirm_value_handle
                        && request.value() == SETUP_COMPLETE_MAGIC
                    {
                        setup_confirmed = true;
                        log!("[setup] hub pickup confirmation received");
                        Ok(())
                    } else if request.attribute_handle == motor_command_value_handle {
                        log!(
                            "[motor] motor command rejected during setup mode attr={:?}",
                            request.attribute_handle
                        );
                        Err(Status::InvalidParameters)
                    } else {
                        log!(
                            "[setup] rejected write attr={:?} expected_attr={:?} value={:?}",
                            request.attribute_handle,
                            setup_confirm_value_handle,
                            request.value()
                        );
                        Err(Status::InvalidParameters)
                    };

                    if ble
                        .write_response(&embassy_stm32_wpan::hci::vendor::command::gatt::WriteResponseParameters {
                            conn_handle: request.conn_handle,
                            attribute_handle: request.attribute_handle,
                            status,
                            value: request.value(),
                        })
                        .await
                        .is_err()
                    {
                        log!("[ble ] setup write response command failed");
                        return false;
                    }
                }
                _ => {}
            },
            Err(err) => log!("[ble ] setup read error {:?}", err),
        }
    }
}

fn first_boot_setup_is_pending(flash: &mut Flash<'_, Blocking>) -> bool {
    let mut stored = [0u8; SETUP_COMPLETE_MAGIC.len()];

    match flash.blocking_read(SETUP_FLAG_PAGE_OFFSET, &mut stored) {
        Ok(()) if &stored == SETUP_COMPLETE_MAGIC => false,
        Ok(()) => true,
        Err(err) => {
            log!(
                "[setup] failed to read setup flag: {:?}; assuming pending",
                err
            );
            true
        }
    }
}

fn mark_first_boot_setup_complete(flash: &mut Flash<'_, Blocking>) -> bool {
    let page_end = SETUP_FLAG_PAGE_OFFSET + SETUP_FLAG_PAGE_SIZE;

    log!(
        "[setup] writing first boot complete flag at flash offset 0x{:05X}",
        SETUP_FLAG_PAGE_OFFSET
    );

    match write_setup_complete_flag(flash, page_end) {
        Ok(()) => {
            log!("[setup] first boot complete flag persisted");
            true
        }
        Err(err) => {
            log!(
                "[setup] failed to persist first boot complete flag: {:?}",
                err
            );
            false
        }
    }
}

fn write_setup_complete_flag(
    flash: &mut Flash<'_, Blocking>,
    page_end: u32,
) -> Result<(), FlashError> {
    flash.blocking_erase(SETUP_FLAG_PAGE_OFFSET, page_end)?;
    flash.blocking_write(SETUP_FLAG_PAGE_OFFSET, SETUP_COMPLETE_MAGIC)
}

async fn wait_for_gatt_add_service_complete(ble: &mut impl UartHci) -> Option<AttributeHandle> {
    loop {
        let response =
            with_timeout(Duration::from_millis(BLE_COMMAND_TIMEOUT_MS), ble.read()).await;
        let Ok(response) = response else {
            log!("[ble ] timed out waiting for GattAddService complete");
            return None;
        };
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
        let response =
            with_timeout(Duration::from_millis(BLE_COMMAND_TIMEOUT_MS), ble.read()).await;
        let Ok(response) = response else {
            log!("[ble ] timed out waiting for GattAddCharacteristic complete");
            return None;
        };
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
        let response =
            with_timeout(Duration::from_millis(BLE_COMMAND_TIMEOUT_MS), ble.read()).await;
        let Ok(response) = response else {
            log!("[ble ] timed out waiting for GattUpdateCharacteristicValue complete");
            return false;
        };
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
    setup_confirm: AttributeHandle,
    motor_command: AttributeHandle,
}

impl GattHandles {
    fn setup_confirm_value_handle(&self) -> AttributeHandle {
        // ST returns the characteristic declaration handle; central writes target the value handle.
        AttributeHandle(self.setup_confirm.0 + 1)
    }

    fn motor_command_value_handle(&self) -> AttributeHandle {
        // ST returns the characteristic declaration handle; central writes target the value handle.
        AttributeHandle(self.motor_command.0 + 1)
    }
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
                    let centi = (u64::from(reading.humidity_q1024) * 100 + 512) / 1024;
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
        let response =
            with_timeout(Duration::from_millis(BLE_COMMAND_TIMEOUT_MS), ble.read()).await;
        let Ok(response) = response else {
            log!("[ble ] timed out waiting for GapSetDiscoverable complete");
            return false;
        };
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

async fn wait_for_gap_update_advertising_data_complete(ble: &mut impl UartHci) -> bool {
    loop {
        let response =
            with_timeout(Duration::from_millis(BLE_COMMAND_TIMEOUT_MS), ble.read()).await;
        let Ok(response) = response else {
            log!("[ble ] timed out waiting for GapUpdateAdvertisingData complete");
            return false;
        };
        if let Ok(Packet::Event(Event::CommandComplete(command_complete))) = response {
            if let ReturnParameters::Vendor(VendorReturnParameters::GapUpdateAdvertisingData(
                status,
            )) = command_complete.return_params
            {
                if status != Status::Success {
                    log!(
                        "[ble ] GapUpdateAdvertisingData failed with status {:?}",
                        status
                    );
                }
                return status == Status::Success;
            }
        }
    }
}

async fn wait_for_gap_set_nondiscoverable_complete(ble: &mut impl UartHci) -> bool {
    loop {
        let response =
            with_timeout(Duration::from_millis(BLE_COMMAND_TIMEOUT_MS), ble.read()).await;
        let Ok(response) = response else {
            log!("[ble ] timed out waiting for GapSetNonDiscoverable complete");
            return false;
        };
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

async fn run_advertising_session(
    ble: &mut (impl UartHci + GapCommands + GattCommands),
    gatt_handles: &GattHandles,
    motor_command_state: &mut MotorCommandState,
    motor_watchdog: &mut MotorWatchdogState,
    motor_uart: &mut SharedLpuart1,
) {
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

        if let Some(dispatch_outcome) =
            dispatch_pending_motor_command(motor_command_state, motor_watchdog, motor_uart).await
        {
            match dispatch_outcome {
                MotorDispatchOutcome::RunStarted {
                    command_id,
                    duration_ms,
                } => log!(
                    "[motor] UART run command sent command_id={:?} duration_ms={} reason_code=NONE validation=accepted (clamped/local-safe)",
                    command_id,
                    duration_ms
                ),
                MotorDispatchOutcome::Stopped { command_id } => {
                    log!("[motor] UART stop command sent command_id={:?} reason_code=NONE", command_id)
                }
                MotorDispatchOutcome::Failed {
                    command_id,
                    action,
                    result,
                } => log!(
                    "[motor] UART dispatch failed command_id={:?} action={:?} result={:?} reason_code={}",
                    command_id,
                    action,
                    result,
                    motor_reason_code_for_uart_result(result),
                ),
                MotorDispatchOutcome::ExpiredBeforeDispatch { command_id } => log!(
                    "[motor] pending command expired before UART dispatch command_id={:?} reason_code=EXPIRED",
                    command_id
                ),
            }
        }

        if let Some(stop_result) = motor_watchdog.enforce_elapsed(motor_uart).await {
            log!(
                "[motor] watchdog stop enforced after elapsed duration result={:?} reason_code={} watchdog_event=elapsed_duration",
                stop_result,
                motor_reason_code_for_uart_result(stop_result),
            );
        }

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
                Event::Vendor(VendorEvent::AttWritePermitRequest(request)) => {
                    let motor_command_value_handle = gatt_handles.motor_command_value_handle();
                    let status = if request.attribute_handle == motor_command_value_handle {
                        match motor_command_state.handle_write_payload(
                            request.value(),
                            now_ms_u32(),
                            motor_watchdog.active_command_id(),
                        ) {
                            MotorCommandWriteOutcome::Accepted(command) => {
                                log!(
                                    "[motor] accepted command command_id={:?} action={:?} duration_ms={} expires_after_ms={} reason_code=NONE validation=accepted (dispatch to UART adapter pending loop scheduling)",
                                    command.command_id,
                                    command.action,
                                    command.duration_ms,
                                    command.expires_after_ms
                                );
                                Ok(())
                            }
                            MotorCommandWriteOutcome::Duplicate => {
                                log!("[motor] duplicate command ignored reason_code=DUPLICATE");
                                Err(Status::InvalidParameters)
                            }
                            MotorCommandWriteOutcome::DuplicateActive => {
                                log!("[motor] duplicate active command rejected reason_code=DUPLICATE");
                                Err(Status::InvalidParameters)
                            }
                            MotorCommandWriteOutcome::Invalid(err) => {
                                log!("[motor] invalid command rejected: {:?} reason_code=UART_REJECTED", err);
                                Err(Status::InvalidParameters)
                            }
                        }
                    } else {
                        log!(
                            "[ble ] rejected non-motor write attr={:?} expected_attr={:?}",
                            request.attribute_handle,
                            motor_command_value_handle
                        );
                        Err(Status::InvalidParameters)
                    };

                    if ble
                        .write_response(&embassy_stm32_wpan::hci::vendor::command::gatt::WriteResponseParameters {
                            conn_handle: request.conn_handle,
                            attribute_handle: request.attribute_handle,
                            status,
                            value: request.value(),
                        })
                        .await
                        .is_err()
                    {
                        log!("[ble ] write response command failed");
                        return;
                    }
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
    rs485: &mut SharedLpuart1,
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

async fn read_zts3000(rs485: &mut SharedLpuart1) -> Result<Zts3000Reading, Zts3000ReadError> {
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
