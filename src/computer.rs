use std::{thread, time::Duration};

use probe_core::from_bytes;
use probe_core::to_bytes;
use probe_core::Measure;

/// Host-side simulation (std)
pub fn run() {
    println!("Computer Simulation:");

    let (mut t, mut rh) = (20.0f32, 40.0f32);

    loop {
        // Build measurement
        let msg = Measure {
            temperature_c: t,
            humidity_rh: rh,
        };

        // Serialize using bincode v2 (through probe_core::to_bytes)
        let buf = to_bytes(&msg);

        // Simulate "transmitting" the bytes
        print!("TX: ");
        for b in &buf {
            print!("{:02X} ", b);
        }

        // Decode again to verify round-trip
        if let Some(msg2) = from_bytes(&buf) {
            println!(
                "\nMeasure: T={:.2}°C  RH={:.2}%",
                msg2.temperature_c, msg2.humidity_rh
            );
        } else {
            println!("\nDecode failed!");
        }

        // Update fake sensor readings
        t += 0.3;
        rh += 0.7;

        // Wait a second
        thread::sleep(Duration::from_secs(1));
    }
}
