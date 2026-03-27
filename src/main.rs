#![no_std]
#![no_main]

extern crate alloc;

mod config;

use static_cell::StaticCell;
use esp_backtrace as _;
use esp_println::println;

use embassy_time::{Duration, Timer};
use esp_hal::timer::timg::TimerGroup;

use bt_hci::controller::ExternalController;
use esp_radio::ble::controller::BleConnector;

use trouble_host::prelude::*;
use trouble_host::advertise::{AdStructure, Advertisement, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE};
use trouble_host::attribute::AttributeTable;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

esp_bootloader_esp_idf::esp_app_desc!();

type MyController = ExternalController<BleConnector<'static>, 4>;
type MyPacketPool = trouble_host::prelude::DefaultPacketPool;
type MyRunner = Runner<'static, MyController, MyPacketPool>;

#[embassy_executor::task]
async fn ble_runner_task(mut runner: MyRunner) {
    if let Err(e) = runner.run().await {
        println!("[BLE] Erreur du runner : {:?}", e);
    }
}

fn init_heap() {
    esp_alloc::heap_allocator!(size: 72 * 1024);
}

#[gatt_service(uuid = "181A")]
struct ServiceEnvironnemental {
    #[characteristic(uuid = "2A6E", read)]
    temperature: i16,
    #[characteristic(uuid = "2A6F", read)]
    humidite: u16,
}

#[gatt_server(mutex_type = NoopRawMutex)]
struct ServeurSonde {
    environnement: ServiceEnvironnemental,
}

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    // ORDRE CRITIQUE :
    // 1. Heap d'abord — esp_radio en a besoin pour s'initialiser
    // 2. esp_hal::init — configure les périphériques HAL
    // 3. esp_radio::init — démarre la radio (prend le périphérique RADIO du HAL)
    init_heap();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // OBLIGATOIRE : démarrer le scheduler RTOS avant esp_radio::init()
    // Sans ça, esp_radio retourne SchedulerNotInitialized
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    println!("[BOOT] Systeme demarre");

    // esp_radio::init() vérifie que le scheduler est prêt (preempt::initialized())
    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio = RADIO.init(esp_radio::init().unwrap());

    let connector = BleConnector::new(
        radio,
        peripherals.BT,
        esp_radio::ble::Config::default(),
    ).unwrap();
    let controller = ExternalController::new(connector);

    static RESOURCES: StaticCell<trouble_host::prelude::HostResources<MyPacketPool, 1, 1>> = StaticCell::new();
    let resources = RESOURCES.init(trouble_host::prelude::HostResources::new());

    static STACK: StaticCell<trouble_host::Stack<'static, MyController, MyPacketPool>> = StaticCell::new();
    let stack = STACK.init(trouble_host::new(controller, resources));

    let trouble_host::Host { mut peripheral, runner, .. } = stack.build();

    spawner.spawn(ble_runner_task(runner)).ok();

    let table = AttributeTable::new();
    let mut _serveur = ServeurSonde::new(table);

    loop {
        println!("[BLE] Diffusion du nom : {}", config::PROBE_NAME);

        let mut encoded_adv_data = [0u8; 31];
        let offset = AdStructure::encode_slice(
            &[
                AdStructure::CompleteLocalName(config::PROBE_NAME.as_bytes()),
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            ],
            &mut encoded_adv_data[..],
        ).expect("Erreur encodage");

        let mut advertiser = peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &encoded_adv_data[..offset],
                    scan_data: &[],
                },
            )
            .await
            .unwrap();

        println!("[BLE] En attente d'un appareil...");

        let _connexion = advertiser.accept().await.unwrap();

        println!("[BLE] Connexion etablie avec le telephone !");

        Timer::after(Duration::from_secs(30)).await;

        println!("[BLE] Fin du cycle, redémarrage de l'annonce...");
    }
}