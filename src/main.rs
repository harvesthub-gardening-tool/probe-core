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
use trouble_host::advertise::{
    AdStructure, Advertisement, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
};
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

// ---------------------------------------------------------------------------
// GATT Service + Server
// ---------------------------------------------------------------------------
// La macro génère un struct ServiceEnvironnemental avec des champs publics :
//   .temperature : Characteristic<i16>
//   .humidite    : Characteristic<u16>
//
// Pour lire/écrire la valeur locale : characteristic.set(&server, &value)
//                                     characteristic.get(&server)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Payload fabricant : [company_id LE][MAGIC_MARKER][ver_maj][ver_min][name_len][name]
// ---------------------------------------------------------------------------
fn build_mfg_payload() -> ([u8; 31], usize) {
    let mut buf = [0u8; 31];
    let mut i = 0usize;

    let cid = config::COMPANY_ID.to_le_bytes();
    buf[i] = cid[0]; i += 1;
    buf[i] = cid[1]; i += 1;

    let marker = config::MAGIC_MARKER;
    buf[i..i + marker.len()].copy_from_slice(marker);
    i += marker.len();

    buf[i] = config::PROBE_VERSION[0]; i += 1;
    buf[i] = config::PROBE_VERSION[1]; i += 1;

    let name = config::PROBE_NAME.as_bytes();
    let name_len = name.len().min(16) as u8;
    buf[i] = name_len; i += 1;
    buf[i..i + name_len as usize].copy_from_slice(&name[..name_len as usize]);
    i += name_len as usize;

    (buf, i)
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    init_heap();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    println!("[BOOT] Systeme demarre");

    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio = RADIO.init(esp_radio::init().unwrap());

    let connector = BleConnector::new(
        radio,
        peripherals.BT,
        esp_radio::ble::Config::default(),
    ).unwrap();
    let controller = ExternalController::new(connector);

    static RESOURCES: StaticCell<trouble_host::prelude::HostResources<MyPacketPool, 1, 1>> =
        StaticCell::new();
    let resources = RESOURCES.init(trouble_host::prelude::HostResources::new());

    static STACK: StaticCell<trouble_host::Stack<'static, MyController, MyPacketPool>> =
        StaticCell::new();
    let stack = STACK.init(trouble_host::new(controller, resources));

    let trouble_host::Host { mut peripheral, runner, .. } = stack.build();

    spawner.spawn(ble_runner_task(runner)).ok();

// -----------------------------------------------------------------------
    // Créer le serveur GATT
    // -----------------------------------------------------------------------
    // 1. On crée la table d'attributs requise par le code généré. 
    // Le compilateur a calculé qu'il lui faut une capacité de 11.
    let mut table: trouble_host::prelude::AttributeTable<'_, NoopRawMutex, 11> = 
        trouble_host::prelude::AttributeTable::new();

    // 2. On initialise le serveur en passant la référence mutable de la table.
    // Attention : pas de .unwrap() car new() retourne l'instance directement.
    let server = ServeurSonde::new(table);

    // Valeurs fictives : 21.50°C = 2150, 60.00% = 6000
    // .set() est synchrone, prend une ref sur le server interne
    server.environnement.temperature
        .set(&server, &2150i16)
        .unwrap();
    server.environnement.humidite
        .set(&server, &6000u16)
        .unwrap();

    println!("[GATT] Valeurs initiales posées : temp=21.50°C, hum=60.00%");

    // Payload fabricant calculé une seule fois
    let (mfg_buf, mfg_len) = build_mfg_payload();
    // AdStructure::ManufacturerSpecificData prend le payload SANS les 2 octets company_id
    let mfg_data = &mfg_buf[2..mfg_len];

    // -----------------------------------------------------------------------
    // Boucle principale : annonce → attend connexion → laisse lire → déconnecte
    // -----------------------------------------------------------------------
    loop {
        println!("[BLE] Annonce : {}", config::PROBE_NAME);

        // 1. Paquet principal (Annonce) - Max 31 octets
        let mut adv_data = [0u8; 31];
        let offset_adv = AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                // On garde la signature constructeur indispensable pour le HUB
                AdStructure::ManufacturerSpecificData {
                    company_identifier: config::COMPANY_ID,
                    payload: mfg_data,
                },
            ],
            &mut adv_data[..],
        ).expect("Erreur encodage adv_data");

        // 2. Paquet de réponse (Scan Response) - Max 31 octets
        let mut scan_data = [0u8; 31];
        let offset_scan = AdStructure::encode_slice(
            &[
                // On garde juste le nom ici, c'est amplement suffisant !
                AdStructure::CompleteLocalName(config::PROBE_NAME.as_bytes()),
            ],
            &mut scan_data[..],
        ).expect("Erreur encodage scan_data");

        // 3. Lancement de l'annonce avec les deux paquets séparés
        let mut advertiser = peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_data[..offset_adv],
                    scan_data: &scan_data[..offset_scan],
                },
            )
            .await
            .unwrap();

        println!("[BLE] En attente d'une connexion...");
        let connexion = advertiser.accept().await.unwrap();
        println!("[BLE] Connexion etablie !");

        // ----- Mise à jour avec de nouvelles valeurs "au pif" -----
        // Pour simuler une évolution des données à chaque nouvelle connexion
        let temp_pif = 2450; // 24.50 °C
        let hum_pif = 6200;  // 62.00 %
        server.environnement.temperature.set(&server, &temp_pif).unwrap();
        server.environnement.humidite.set(&server, &hum_pif).unwrap();
        // ----------------------------------------------------------

        // Laisser le HUB lire les données (10s)
        Timer::after(Duration::from_secs(10)).await;

        drop(connexion);
        println!("[BLE] Deconnecte. Prochain cycle dans 30s...");

        Timer::after(Duration::from_secs(30)).await;
    }
}