/*
 * STM32WB55CGU6 — M4 application linker script.
 *
 * Memory map (per ST RM0434 + STM32WB Release_Notes.html for
 * BLE Full Stack v1.24.0.3 on WB55xC, 512K flash variant):
 *   - Total flash         : 512 KB @ 0x0800_0000
 *   - BLE Full v1.24      : installs at 0x0803_0000 (upper 64K reserved for M0+ stack + secure)
 *   - M4 user flash       : 192 KB @ 0x0800_0000 .. 0x0802_FFFF  (HARD LIMIT — overflow bricks BLE)
 *   - SRAM1               : 192 KB @ 0x2000_0000  (M4 application RAM)
 *   - SRAM2a              :  32 KB @ 0x2003_0000  (shared — IPCC / TL mailbox lives here)
 *   - SRAM2b              :  32 KB @ 0x2003_8000  (shared with M0+, currently unused by M4)
 *
 * The IPCC/Transport-Layer mailbox sections below use FIXED ABSOLUTE
 * addresses hard-coded by ST BLE Full v1.24.0.3. Do NOT move them.
 */
MEMORY
{
    FLASH      (rx)  : ORIGIN = 0x08000000, LENGTH = 192K
    RAM        (xrw) : ORIGIN = 0x20000000, LENGTH = 192K
    RAM_SHARED (xrw) : ORIGIN = 0x20030000, LENGTH = 10K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS {
    TL_REF_TABLE                       (NOLOAD) : { *(TL_REF_TABLE) }            >RAM_SHARED
    MB_MEM1                             (NOLOAD) : { *(MB_MEM1) }                >RAM_SHARED
    MB_MEM2                             (NOLOAD) : { *(MB_MEM2) }                >RAM_SHARED
}
