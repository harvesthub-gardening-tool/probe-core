use std::env;
use std::fs;
use std::io::Read;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).expect("failed to copy memory.x to OUT_DIR");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");

    let probe_uuid = generate_uuid_v4();
    println!("cargo:rustc-env=PROBE_BUILD_UUID={probe_uuid}");
}

fn generate_uuid_v4() -> String {
    let mut random = [0u8; 16];
    fs::File::open("/dev/urandom")
        .expect("failed to open /dev/urandom")
        .read_exact(&mut random)
        .expect("failed to read random bytes");

    // RFC 4122 v4: set version and variant bits.
    random[6] = (random[6] & 0x0f) | 0x40;
    random[8] = (random[8] & 0x3f) | 0x80;

    format!(
        "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        random[0],
        random[1],
        random[2],
        random[3],
        random[4],
        random[5],
        random[6],
        random[7],
        random[8],
        random[9],
        random[10],
        random[11],
        random[12],
        random[13],
        random[14],
        random[15],
    )
}
