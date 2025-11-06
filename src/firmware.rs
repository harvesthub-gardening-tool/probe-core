#![allow(dead_code)]

extern crate alloc;
use alloc::vec::Vec;

use probe_core::Measure;
use probe_core::to_bytes;

/// Firmware loop (no_std + alloc).
pub fn run() -> ! {
    let (mut t, mut rh) = (20.0f32, 40.0f32);

    loop {
        let msg = Measure { temperature_c: t, humidity_rh: rh };

        let buf: Vec<u8> = to_bytes(&msg);

        // TODO: replace with your UART/USB send
        send_bytes(&buf);

        t += 0.3;
        rh += 0.7;
    }
}

fn send_bytes(_data: &[u8]) {
    // for &b in data { nb::block!(uart.write(b)).ok(); }
}