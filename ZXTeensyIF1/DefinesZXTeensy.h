
#ifndef DEFINES_ZX_TEENSY_H
#define DEFINES_ZX_TEENSY_H

#define ZXTEENSY_VERSION "20260627"
#define ENABLE_BUILTIN_ROM_IF1
//define DEBUG_OUTPUT
//define ENABLE_JOYSTICK_DEBUG

#define MAX_PATH 256

#define ESXMMC_BIN_PATH ((const char*)F("/ZXTEENSY/esxmmc.bin"))
#define MF128_ROM_PATH ((const char*)F("/ZXTEENSY/mf128.rom"))
#define LPRINT_ROM_PATH ((const char*)F("/ZXTEENSY/lprint32.rom"))
#define MODEM_ROM_PATH ((const char*)F("/ZXTEENSY/vtx.rom"))
#define IF1_ROM_PATH ((const char*)F("/ZXTEENSY/if1.rom"))
#define MENU_ROM_PATH ((const char*)F("/ZXTEENSY/menu.rom"))
#define MDR_EMULATOR_ROM_PATH ((const char*)F("/ZXTEENSY/SPECTRA_IF1_ED2_ME_ROM_Formatted.bin"))
#define ZXTEENSY_CFG_PATH ((const char*)F("/ZXTEENSY/ZXTEENSY.CFG"))
#define NETMAN_Z80_PATH ((const char*)F("/ZXTEENSY/netman.z80"))
#define RTC_SETUP_Z80_PATH ((const char*)F("/ZXTEENSY/rtc_setup.z80"))
#define PRINTER_OUT_PATH ((const char*)F("/printer.txt"))

#define FWUPDATE_HEX_PATH "ZXTEENSY.HEX"
#define MODEM_URL_PATH ((const char*)F("\"glasstty.com\",6502"))

// Run the Teensy 4.1 with slight overclock at 768MHz
#define TEENSY_CLK_FREQ 768000000ULL

// Tick the SD and Serial at 7MHz
#define TICK_FREQ 7000000ULL
#define TICK_CYCCNT (TEENSY_CLK_FREQ / TICK_FREQ)

// Allow ~500ms for reset/button to debounce
#define TRIGGER_DELAY_MS 500
#define TRIGGER_DELAY_CNT ((TRIGGER_DELAY_MS * TEENSY_CLK_FREQ) / (TICK_CYCCNT * 1000))

// If reset is held for an additional 2 seconds, then perform a hard reset
#define HARD_RESET_DELAY_MS 2000
#define HARD_RESET_DELAY_CNT ((HARD_RESET_DELAY_MS * TEENSY_CLK_FREQ) / (TICK_CYCCNT * 1000))

// ZXC3 flash erasure delay
#define ZXC3_ERASE_DELAY_MS 100
#define ZXC3_ERASE_DELAY_CNT ((ZXC3_ERASE_DELAY_MS * TEENSY_CLK_FREQ) / (TICK_CYCCNT * 1000))

// Tape T-state delay
#define TAPE_DELAY_CNT (TEENSY_CLK_FREQ / 3500000ULL)

// VTX5000 read data rate to 120 bps (~1200 baud)
#define MODEM_DELAY_CNT (TEENSY_CLK_FREQ / 120ULL)

// Maximum length of a visible file name
#define ROM_NAME_LEN 32

// Maximum length of ROM_NAME_LEN, tab and 2 x right icons
#define MENU_TXT_LEN (ROM_NAME_LEN + 3)

// Maximum length of menu entry
// NOTE: Allow for left icon, ROM_NAME_LEN, tab, 2 x right icons, and new-line
#define MENU_STR_LEN (ROM_NAME_LEN + 5)

#define PAGE_IN_ROM(bit) (romPaged |= (1UL << (bit)))
#define PAGE_OUT_ROM(bit) (romPaged &= ~(1UL << (bit)))
#define IS_ROM_PAGED(bit) ((romPaged >> (bit)) & 0x01)
#define IS_ROM_PRIORITY(bit) ((romPaged >= (1UL << (bit))))

#endif
