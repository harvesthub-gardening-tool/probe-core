#![cfg_attr(feature = "firmware", no_std)]
#![cfg_attr(feature = "firmware", no_main)]

#[cfg(feature = "computer")]
mod computer;
#[cfg(feature = "firmware")]
mod firmware;

#[cfg(feature = "firmware")]
use cortex_m_rt::entry;

#[cfg(feature = "firmware")]
use embedded_alloc::LlffHeap;

#[cfg(feature = "firmware")]
use panic_halt as _; // panic handler

// Force-link cortex-m so its critical-section backend is included.
#[cfg(feature = "firmware")]
use cortex_m as _;

//
// ─── Firmware allocator (define exactly once) ─────────────────────────────────
//
#[cfg(feature = "firmware")]
#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

#[cfg(feature = "firmware")]
#[allow(static_mut_refs)]
fn init_heap() {
    use core::mem::MaybeUninit;

    const HEAP_SIZE: usize = 16 * 1024;
    static mut HEAP_MEM: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        // LlffHeap::init(start_addr: usize, size: usize)
        let start_addr: usize = HEAP_MEM.as_mut_ptr().cast::<u8>() as usize;
        HEAP.init(start_addr, HEAP_SIZE);
    }
}

//
// ─── Host (PC) entrypoint ─────────────────────────────────────────────────────
//
#[cfg(feature = "computer")]
fn main() {
    computer::run();
}

//
// ─── Firmware (embedded) entrypoint ───────────────────────────────────────────
//
#[cfg(feature = "firmware")]
#[entry]
fn main() -> ! {
    // Touch cortex_m so the critical-section backend links in.
    cortex_m::interrupt::free(|_| {});
    init_heap();
    firmware::run()
}