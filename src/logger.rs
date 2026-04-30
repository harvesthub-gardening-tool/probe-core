use core::cell::RefCell;
use core::fmt::Write;
use core::ptr;
use core::sync::atomic::{AtomicPtr, Ordering};

use cortex_m::interrupt::Mutex;
use embassy_stm32::usart::UartTx;
use embassy_time::Instant;

type SharedTx = Mutex<RefCell<Option<UartTx<'static, embassy_stm32::mode::Async>>>>;

static LOGGER: AtomicPtr<Logger> = AtomicPtr::new(ptr::null_mut());

pub struct Logger {
    tx: &'static SharedTx,
}

impl Logger {
    pub fn new(tx: &'static SharedTx) -> Self {
        Self { tx }
    }

    pub fn write_line(&self, args: core::fmt::Arguments<'_>) {
        let ms = Instant::now().as_millis();
        let tx = cortex_m::interrupt::free(|cs| {
            let mut slot = self.tx.borrow(cs).borrow_mut();
            slot.take()
        });

        if let Some(mut tx) = tx {
            let _ = write!(BlockingWriter(&mut tx), "[{:>10} ms] ", ms);
            let _ = BlockingWriter(&mut tx).write_fmt(args);
            let _ = BlockingWriter(&mut tx).write_str("\r\n");

            cortex_m::interrupt::free(|cs| {
                self.tx.borrow(cs).replace(Some(tx));
            });
        }
    }
}

// SAFETY: `logger` must outlive the program. Caller passes a 'static
// reference (the StaticCell-backed Logger in main), so the pointer is
// valid for the remainder of program execution.
pub fn set_logger(logger: Logger) {
    let boxed: &'static Logger = {
        static CELL: static_cell::StaticCell<Logger> = static_cell::StaticCell::new();
        CELL.init(logger)
    };
    LOGGER.store(boxed as *const Logger as *mut Logger, Ordering::Release);
}

pub fn with_logger(args: core::fmt::Arguments<'_>) {
    let ptr = LOGGER.load(Ordering::Acquire);
    if !ptr.is_null() {
        // SAFETY: pointer was set from a 'static reference in set_logger
        // and is never deallocated.
        let l: &Logger = unsafe { &*ptr };
        l.write_line(args);
    }
}

struct BlockingWriter<'a, 'd>(&'a mut UartTx<'d, embassy_stm32::mode::Async>);

impl<'a, 'd> core::fmt::Write for BlockingWriter<'a, 'd> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0
            .blocking_write(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        self.0.blocking_flush().map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => {{
        $crate::logger::with_logger(format_args!($($arg)*));
    }};
}
