#![cfg_attr(not(feature = "computer"), no_std)]

extern crate alloc;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Measure {
    pub temperature_c: f32,
    pub humidity_rh: f32,
}

#[cfg(feature = "computer")]
mod host_codec {
    use super::Measure;
    use bincode::config::standard;
    use bincode::serde::{decode_from_slice, encode_to_vec};
    pub fn to_bytes(m: &Measure) -> Vec<u8> {
        encode_to_vec(m, standard()).expect("encode")
    }
    pub fn from_bytes(b: &[u8]) -> Option<Measure> {
        decode_from_slice(b, standard()).ok().map(|(v, _)| v)
    }
    use alloc::vec::Vec;
}

#[cfg(feature = "firmware")]
mod fw_codec {
    use super::Measure;
    pub fn to_bytes(m: &Measure) -> alloc::vec::Vec<u8> {
        postcard::to_allocvec(m).expect("encode")
    }
    pub fn from_bytes(b: &[u8]) -> Option<Measure> {
        postcard::from_bytes(b).ok()
    }
}

#[cfg(feature = "computer")]
pub use host_codec::{from_bytes, to_bytes};

#[cfg(feature = "firmware")]
pub use fw_codec::{from_bytes, to_bytes};
