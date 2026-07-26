
#include <SD.h>
#include <SdFat.h>
#include "USBHost_t36.h"
#include "if1-2_rom.h"
#include "RingBuffer.h"
#include "PrintableString.h"
#include "SdHdfZXTeensy.h"
#include "SdSdioZXTeensy.h"
#include "UartZXTeensy.h"
#include "RtcZXTeensy.h"
#include "EspNtpZXTeensy.h"
#include "TzxPlayerZXTeensy.h"
#include "Dsk765ZXTeensy.h"
#include "PrinterZXTeensy.h"
#include "DefinesZXTeensy.h"

extern "C" volatile uint32_t systick_millis_count;
extern "C" uint32_t set_arm_clock(uint32_t frequency);

static const char PROGMEM VERSION_STR[9] = ZXTEENSY_VERSION;

typedef enum {
    MENU_ACTION_TOP_MENU,
    MENU_ACTION_SETTING,
    MENU_ACTION_LOAD_CFG,
    MENU_ACTION_LOAD_ROM,
    MENU_ACTION_LOAD_CART,
    MENU_ACTION_UPDATE_FW,
    MENU_ACTION_NTP_TZ,
    MENU_ACTION_LOAD_NETMAN,
    MENU_ACTION_LOAD_RTC_SETUP,
    MENU_ACTION_BROWSER_CD,
    MENU_ACTION_BROWSER_PAGE,
    MENU_ACTION_BROWSER_EXPAND,
    MENU_ACTION_BROWSER_OPEN,
    MENU_ACTION_BROWSER_OPEN_ROM,
    MENU_ACTION_BROWSER_OPEN_HDF,
    MENU_ACTION_BROWSER_OPEN_DSK,
    MENU_ACTION_BROWSER_LOAD_CART,
    MENU_ACTION_BROWSER_LOAD_ZXC2,
    MENU_ACTION_BROWSER_LOAD_ZXC3,
    MENU_ACTION_BROWSER_LOAD_MLD,
    MENU_ACTION_BROWSER_LOAD_Z80,
    MENU_ACTION_BROWSER_LOAD_TZX,
    MENU_ACTION_BROWSER_LOAD_MDR,
    MENU_ACTION_BROWSER_LOAD_MF128,
    MENU_ACTION_BROWSER_OPEN_POK,
    MENU_ACTION_BROWSER_MOUNT_SDA,
    MENU_ACTION_BROWSER_MOUNT_SDB,
    MENU_ACTION_BROWSER_MOUNT_FDA,
    MENU_ACTION_BROWSER_MOUNT_FDB,
    MENU_ACTION_START_SERVER,
    MENU_ACTION_STOP_SERVER,
    MENU_ACTION_SELECT_LOAD_SLOT,
    MENU_ACTION_SELECT_SAVE_SLOT,
    MENU_ACTION_LOAD_STATE_SLOT,
    MENU_ACTION_POK_TOGGLE_TRAINER,
    MENU_ACTION_IN_GAME_EXIT,
    MENU_ACTION_IN_GAME_EXIT_TAPE,
    MENU_ACTION_IN_GAME_EXIT_BASIC,
    MENU_ACTION_IN_GAME_SEEK_TAPE,
    MENU_ACTION_IN_GAME_SAVE_STATE,
    MENU_ACTION_IN_GAME_APPLY_POK,
    MENU_ACTION_IN_GAME_UNMOUNT_FDA,
    MENU_ACTION_IN_GAME_UNMOUNT_FDB,
    MENU_ACTION_IN_GAME_EJECT_TAPE,
    MENU_ACTION_IN_GAME_NMI,
    MENU_ACTION_IN_GAME_MF128,
    MENU_ACTION_IN_GAME_DIVMMC,
    MENU_ACTION_IN_GAME_RESET
} menu_action_t;

typedef enum {
    ICON_TYPE_NONE,
    ICON_TYPE_DSK,
    ICON_TYPE_ZXC2,
    ICON_TYPE_CART,
    ICON_TYPE_Z80,
    ICON_TYPE_TZX
} icon_type_t;

typedef enum {
    POKE_PARSE_METADATA,
    POKE_PARSE_PREPARE,
    POKE_PARSE_APPLY
} poke_parse_op_t;

typedef enum {
    STATE_ROM_DISABLE = 0x00,
    STATE_ROM_ENABLE  = 0x01,
    STATE_RESET       = 0x02,
    STATE_RESET_MENU  = 0x03
} run_state_t;

typedef enum {
    TRIGGER_ACTIVE,
    TRIGGER_HOLD,
    TRIGGER_DELAY,
    TRIGGER_READY
} trigger_state_t;

typedef enum {
    BANK_ROM0   = 0x0001,
    BANK_ROM1   = 0x0002,
    BANK_ROM2   = 0x0004,
    BANK_ROM3   = 0x0008,
    BANK_IF1    = 0x0010,
    BANK_MF128  = 0x0020,
    BANK_DIVMMC = 0x0040,
    BANK_LPRINT = 0x0080,
    BANK_MENU   = 0x0100,
    BANK_RAM    = 0x0200
} bank_select_t;

// NOTE: This also defines the priority for active paged ROMs
typedef enum {
    // 16KB ROMs
    ROM_ROM0,
    ROM_ROM1,
    ROM_ROM2,
    ROM_ROM3,
    ROM_IF1,
    // 8KB ROM and 8KB RAM
    ROM_MF128,
    // 8KB ROMs
    ROM_DIVMMC,
    // ROM_LPRINT uses separate 2KB ROM array
    ROM_LPRINT,
    // "Dynamic ROMs" that use DivMMC RAM
    ROM_MODEM,
    ROM_ZXC2,
    ROM_MLD,
    ROM_SNA,
    // Menu ROM as highest priority
    ROM_MENU
} rom_select_t;

typedef enum {
    ROM_PAGE_ROM0      = 0,
    ROM_PAGE_ROM1      = 2,
    ROM_PAGE_ROM2      = 4,
    ROM_PAGE_ROM3      = 6,
    ROM_PAGE_IF1       = 8,
    ROM_PAGE_MF128     = 10,
    ROM_PAGE_DIVMMC    = 12,
    ROM_PAGE_MENU      = 13,
    ROM_PAGE_COUNT
} rom_page_t;

typedef enum {
    TYPE_ROM,
    TYPE_ZXC2,
    TYPE_ZXC3,
    TYPE_CART,
    TYPE_Z80,
    TYPE_SNA,
    TYPE_MLD
} rom_type_t;

typedef enum {
    SD_SPI_WRITE,
    SD_SPI_SELECT
} sd_spi_action_t;

typedef enum {
    DIVMMC_NONE,
    DIVMMC_SDHC,
    DIVMMC_HDF_A,
    DIVMMC_HDF_B
} divmmc_spi_t;

typedef enum {
    ZXC3_FLASH_IDLE,
    ZXC3_FLASH_UNLOCK,
    ZXC3_FLASH_CMD,
    ZXC3_FLASH_WRITE
} zxc3_flash_state_t;

typedef enum {
    MLD_PULSE_IDLE,
    MLD_PULSE_COMMAND,
    MLD_PULSE_DATA1,
    MLD_PULSE_DATA2,
    MLD_PULSE_FAST
} mld_pulse_state_t;

typedef enum {
    MLD_EEP_IDLE,
    MLD_EEP_PROGRAM_BYTE,
    MLD_EEP_ERASE_SECTOR,
    MLD_EEP_PROGRAM_SECTOR
} mld_eep_program_t;

typedef enum {
    MENU_ROM_CMD_IDLE = 0,
    MENU_ROM_CMD_REDRAW = 1,
    MENU_ROM_CMD_IN_GAME_EXIT = 2,
    MENU_ROM_CMD_STATE_CAPTURE_48 = 3,
    MENU_ROM_CMD_STATE_CAPTURE_128 = 4,
    MENU_ROM_CMD_STATE_BLOCK_DONE = 5,
    MENU_ROM_CMD_STATE_COMPLETE = 6,
    MENU_ROM_CMD_STATE_FAILED = 7,
    MENU_ROM_CMD_STATE_PREVIEW = 8,
    MENU_ROM_CMD_BROWSER_EXPAND = 9
} menu_rom_action_t;

// I/O pin assignments
static const uint8_t LED_PIN = 13;
static const uint8_t DATA_DIS_PIN = 29;
static const uint8_t DATA_OUT_PIN = 36;  // 1 = output, 0 = input
static const uint8_t RESET_PIN = 31;
static const uint8_t RESET_IN_PIN = 2;
static const uint8_t BUTTON_PIN = 33;
static const uint8_t ROMCS_PIN = 37;
static const uint8_t ROMCS_IN_PIN = 3;
static const uint8_t IF1_DIS_PIN = 5;
static const uint8_t NMI_PIN = 30;
static const uint8_t RD_PIN = 1;
static const uint8_t WR_PIN = 0;
static const uint8_t MREQ_PIN = 24;
static const uint8_t IOREQ_PIN = 25;
static const uint8_t M1_PIN = 4;
static const uint8_t ESP_ENABLE = 28;
static const uint8_t SD_CS_PIN = 46;

static const uint8_t INPUT_PINS[] = {
    RESET_IN_PIN, MREQ_PIN, RD_PIN, IOREQ_PIN, WR_PIN, M1_PIN, ROMCS_IN_PIN,
    19, 18, 14, 15, 40, 41, 17, 16, 22, 23, 20, 21, 38, 39, 26, 27, // Address bus
    BUTTON_PIN, ESP_ENABLE
};

static const uint32_t RD_PIN_BITMASK = CORE_PIN1_BITMASK;
static const uint32_t WR_PIN_BITMASK = CORE_PIN0_BITMASK;
static const uint32_t M1_PIN_BITMASK = CORE_PIN4_BITMASK;
static const uint32_t IOREQ_PIN_BITMASK = CORE_PIN25_BITMASK;
static const uint32_t MREQ_PIN_BITMASK = CORE_PIN24_BITMASK;
static const uint32_t A15_PIN_BITMASK = CORE_PIN27_BITMASK;
static const uint32_t A14_PIN_BITMASK = CORE_PIN26_BITMASK;
static const uint32_t A13_PIN_BITMASK = CORE_PIN39_BITMASK;
static const uint32_t A12_PIN_BITMASK = CORE_PIN38_BITMASK;
static const uint32_t ROMCS_IN_PIN_BITMASK = CORE_PIN3_BITMASK;
static const uint32_t DATA_OUT_PIN_BITMASK = CORE_PIN36_BITMASK;

static const uint8_t DATA_PINS[] = { 6, 7, 8, 9, 10, 11, 12, 32 };

static const uint32_t GPIO7_DATA_MASK = (CORE_PIN6_BITMASK | CORE_PIN7_BITMASK |
    CORE_PIN8_BITMASK | CORE_PIN9_BITMASK |
    CORE_PIN10_BITMASK | CORE_PIN11_BITMASK |
    CORE_PIN12_BITMASK | CORE_PIN32_BITMASK);

static const uint8_t OUTPUT_PINS[] = {
    LED_PIN, ROMCS_PIN, NMI_PIN, IF1_DIS_PIN
};

// Mask for A15 and A14 and ^MREQ
static const uint32_t ROM_ADDRESS_MASK = (A15_PIN_BITMASK | A14_PIN_BITMASK | MREQ_PIN_BITMASK);

// Number of SD detection retries
static const uint8_t NUM_SD_RETRIES = 5;

// Global state
volatile bool bootIntoMenu = false;
volatile bool afterFirstReset = false;
volatile bool isDeviceDisabled = false;
volatile bool loadRomSets = false;
volatile bool sdCardPresent = false;
volatile bool sdioEnabled = false;
volatile run_state_t globalState = STATE_RESET;
volatile bool busRdActive = false;
volatile bool nmiPending = false;
volatile rom_select_t nmiRomTarget = ROM_ROM0;

// State save and load
volatile int8_t stateActiveSlot = -1;
volatile uint8_t stateSaveSlot = 0;
volatile bool stateStartLoad = false;

// Reset and NMI debouncing
volatile trigger_state_t resetTrigState = TRIGGER_ACTIVE;
volatile uint32_t resetTrigExitCount = TRIGGER_DELAY_CNT;
volatile uint32_t resetHardTrigCount = HARD_RESET_DELAY_CNT;
volatile trigger_state_t buttonTrigState = TRIGGER_READY;
volatile uint32_t buttonTrigExitCount = 0;

// ROM banking
static const uint16_t RAM_PAGE_SIZE = 0x2000;
static const uint16_t ROM_PAGE_SIZE = (RAM_PAGE_SIZE * 2);
static const uint16_t LPRINT_ROM_SIZE = 0x800;
volatile rom_select_t romSelected = ROM_ROM0;
volatile bank_select_t romArraySelected = BANK_ROM0;
volatile uint32_t romPaged = 0x01;
volatile uint8_t romArray[ROM_PAGE_COUNT][RAM_PAGE_SIZE] __attribute__((aligned(16)));
volatile uint8_t lprintRom[LPRINT_ROM_SIZE] __attribute__((aligned(16)));
volatile uint8_t* romPtr = romArray[0];
volatile uint16_t romArrayPresent = 0;
volatile bool romEnabled = false;
volatile bool romCsEnable = false;
volatile bool romCsDisable = false;

// Spectrum 128k/+3 ROMs
volatile bool rom1Present = false;
volatile bool rom23Present = false;
volatile bool rom1Paged = false;
volatile bool rom23Paged = false;

// DivMMC with total 512KB of RAM
static const uint16_t RAM_PAGE_COUNT = 16;
static const uint16_t EXT_RAM_PAGE_COUNT = 48;
volatile uint8_t divMmcRamArray[RAM_PAGE_COUNT][RAM_PAGE_SIZE] __attribute__((aligned(16)));
volatile DMAMEM uint8_t divMmcExtRamArray[EXT_RAM_PAGE_COUNT][RAM_PAGE_SIZE] __attribute__((aligned(16)));
volatile bool divMmcPresent = false;
volatile bool divMmcRomPresent = false;
volatile bool divMmcEnabled = false;
volatile bool divMmcExtRamPresent = false;
volatile bool divMmcRomEnabled = false;
volatile bool divMmcSdReadOnly = false;
volatile bool divMmcToggle = false;
volatile bool divMmcAutoMap = false;
volatile bool divMmcConMem = false;
volatile bool divMmcMapRam = false;
volatile uint8_t divMmcRamBank = 0;
volatile bool divMmcRamBankThree = false;
volatile bool divMmcPreserveRam = false;
volatile bool divMmcExtRamEnabled = false;
volatile uint8_t* divMmcRamPtr;

// Multiface 128
volatile bool mf128Present = false;
volatile bool mf128Enabled = false;
volatile bool mf128ActiveNMI = false;
volatile bool mf128LoadGenie = false;

// Interface 1
volatile bool interface1Present = false;
volatile bool interface1Enabled = false;
volatile bool interface1Removed = false;

// ZXC2 cartridge
volatile bool zxC2Present = false;
volatile bool zxC2Lock = false;
volatile bool zxC2ShadowRom = false;
volatile uint8_t zxC2RomBank = 0x00;

// ZXC3 flash cartridge
static const uint16_t ZXC3_PAGE_COUNT = 16;
volatile bool zxC3Present = false;
volatile bool zxC3Write = false;
volatile bool zxC3EraseBusy = false;
volatile zxc3_flash_state_t zxC3FlashState = ZXC3_FLASH_IDLE;
volatile bool zxC3FlashSetup = false;
volatile trigger_state_t zxC3WriteTrigState = TRIGGER_READY;
volatile uint32_t zxC3WriteTrigExitCount = 0;
volatile trigger_state_t zxC3EraseTrigState = TRIGGER_READY;
volatile uint32_t zxC3EraseTrigExitCount = 0;
RingBuffer<EXT_RAM_PAGE_COUNT> zxC3EraseBuffer;

// Dandanator Mini flash cartridge
static const uint16_t MLD_HEADER_OFFSET = 0x3FEA;
static const uint16_t MLD_SIGNATURE_OFFSET = 0x3FFC;
static const uint8_t MLD_MAX_PAGE_COUNT = RAM_PAGE_COUNT + EXT_RAM_PAGE_COUNT;
static const uint8_t MLD_MAX_SLOT_COUNT = MLD_MAX_PAGE_COUNT / 2;
static const uint8_t MLD_SECTORS_PER_SLOT = 4;
static const uint8_t MLD_MAX_SAVE_SECTOR_COUNT = 4;
static const uint8_t MLD_SECTOR_COUNT = MLD_MAX_PAGE_COUNT * 2;
static const uint32_t MLD_PULSE_TIMEOUT_CNT =
    (uint32_t)((TEENSY_CLK_FREQ / 1000000ULL) * 32ULL);
static const uint32_t MLD_SPECIAL_TIMEOUT_CNT =
    (uint32_t)((TEENSY_CLK_FREQ / 1000000ULL) * 512ULL);
volatile bool mldPresent = false;
volatile uint8_t mldSlotCount = 0;
volatile bool mldCmdLocked = false;
volatile bool mldCmdDisabled = false;
volatile uint8_t mldCurrentSlot = 1;
volatile uint8_t mldPreviousSlot = 1;
volatile mld_eep_program_t mldEepProgram = MLD_EEP_IDLE;
volatile uint8_t mldEepSector = 0x00;
volatile uint16_t mldEepProgramRemaining = 0;
volatile uint8_t mldCmdOpcode = 0x00;
volatile uint8_t mldCmdData1 = 0x00;
volatile uint8_t mldCmdData2 = 0x00;
volatile uint8_t mldCmdRepeat = 0x00;
volatile mld_pulse_state_t mldPulseState = MLD_PULSE_IDLE;
volatile uint32_t mldPulseCycle = 0;

// Microdrive emulator
static const uint8_t MDR_MAX_SECTOR = 0xB4;
volatile bool mdrPresent = false;
volatile bool mdrEnabled = false;
volatile uint8_t mdrMaxSector = 0;

// Z80 snapshot loader banking
static const size_t SNA_LOADER_FIRST_SIZE = 236;
static const size_t SNA_LOADER_FINAL_SIZE = 12;
static const uint32_t SNA_LOADER_BANK_5_SIZE =
    (ROM_PAGE_SIZE - (SNA_LOADER_FIRST_SIZE + SNA_LOADER_FINAL_SIZE + 1));
volatile bool snaLoaderPresent = false;
volatile uint8_t snaLoaderBanks = 0;

// Boot menu ROM
static const uint16_t MENU_PAGE_COUNT = 4;
static const uint16_t MENU_BUFFER_SIZE = 16;
volatile bool menuEnableInGame = false;
volatile bool menuEnterOnReset = false;
volatile bool menuSelected = false;
volatile bool menuRedraw = false;
volatile bool menuTriggerNMI = false;
volatile bool menuTriggerExitNMI = false;
volatile uint8_t menuSelectedIndex = 0;
RingBuffer<MENU_BUFFER_SIZE> menuBuffer;
volatile DMAMEM uint8_t menuRamArray[MENU_PAGE_COUNT][RAM_PAGE_SIZE] __attribute__((aligned(16)));
volatile uint8_t* menuRamPtr;
volatile rom_select_t menuPrevRomSelected = ROM_ROM0;
volatile uint8_t* menuPrevRomPtr;

// DivMMC SPI/SD
static const size_t READ_BUFFER_SIZE = 1024;
static const size_t WRITE_BUFFER_SIZE = 16;
RingBuffer<WRITE_BUFFER_SIZE> sdSpiWriteBuffer;
RingBuffer<WRITE_BUFFER_SIZE> sdSpiFlagsBuffer;
SdSdioZXTeensy<READ_BUFFER_SIZE> divMmcSpi;
SdHdfZXTeensy<READ_BUFFER_SIZE> divMmcHdf;
SdHdfZXTeensy<READ_BUFFER_SIZE> divMmcSecondHdf;
volatile divmmc_spi_t divMmcDrive = DIVMMC_NONE;
volatile divmmc_spi_t divMmcDriveSlot[2];

// MB03+ UART
static const size_t UART_BUFFER_SIZE = ROM_PAGE_SIZE;
uint8_t uartBuffer[UART_BUFFER_SIZE] __attribute__((aligned(16)));
volatile bool uartPresent = false;
volatile bool uartEnabled = false;
UartZXTeensy espUart;

// VTX5000
volatile bool modemPresent = false;
volatile bool modemEnabled = false;
volatile bool modemOnReset = true;

// RTC module
volatile bool wifiNtpPresent = false;
volatile bool wifiNtpEnabled = false;
volatile bool rtcHasTime = false;
volatile uint8_t wifiNtpTz = 48;
RtcZXTeensy rtcTeensy;
EspNtpZXTeensy wifiNtp;

// USB Kempston mouse and gamepad
USBHost usbHost;
USBHub usbHub1(usbHost);
USBHub usbHub2(usbHost);
USBHIDParser usbHidParser1(usbHost);
USBHIDParser usbHidParser2(usbHost);
MouseController usbMouse(usbHost);
JoystickController usbJoystick(usbHost);
KeyboardController usbKeyboard(usbHost);
volatile bool usbPresent = false;
volatile bool usbEnabled = false;
volatile bool gamepadButtons = false;
volatile bool mousePresent = false;
volatile uint32_t mouseX = 0;
volatile uint32_t mouseY = 0;
volatile uint8_t mouseBtn = 0;
volatile bool joystickPresent = false;
volatile uint8_t joystickData = 0;

// Tape player
TzxPlayerZXTeensy tzxPlayer;
volatile bool tzxPresent = false;
volatile bool tzxEnabled = false;

// uPD765 disk controller
Dsk765ZXTeensy dskController;
volatile bool dskPresent = false;
volatile bool dskEnabled = false;
volatile bool dskEnableDriveB = false;

// Centronics printer port
PrinterZXTeensy printerPort;
volatile bool printerPresent = false;
volatile bool printerEnabled = false;
volatile bool lprintPresent = false;
volatile bool lprintEnabled = false;
volatile bool printerStrobe = true;
volatile uint8_t printerByte = 0x00;

// Spectrum read-only port state
volatile uint8_t spectrumBorder = 0x00;
volatile uint8_t spectrumBankM = 0x00;
volatile uint8_t spectrumBank678 = 0x00;
volatile uint8_t spectrumAyReg = 0x00;

// SPI and UART tick cycle counter
volatile uint32_t globalCycleCount;

// Optimised ISR functions
FASTRUN void isrFastGpios() __attribute__((hot, optimize("O3")));
FASTRUN void isrRdEvent() __attribute__((hot, optimize("O3")));
FASTRUN void isrWrEvent() __attribute__((hot, optimize("O3")));

// Optimised task loop functions
FASTRUN void loop() __attribute__((hot, optimize("O3")));
inline void sdSpiOnTick() __attribute__((always_inline, hot, optimize("O3")));
inline void zxC3OnTick() __attribute__((always_inline, hot, optimize("O3")));
inline void mldPulseOnTick(uint32_t cycle) __attribute__((always_inline, hot, optimize("O3")));

// Optimised functions used by ISR
void stateLoaderFinished(bool onPageOut) __attribute__((optimize("O3")));
inline void mldClearCommand() __attribute__((always_inline, hot, optimize("O3")));
inline void mldClearEepProgram() __attribute__((always_inline, hot, optimize("O3")));
void mldRunCommand(uint8_t command) __attribute__((hot, optimize("O3")));
void updateMldSlotPtr(uint8_t slot) __attribute__((hot, optimize("O3")));

// Optimised read ISR ROM functions
inline void updateRomPtr(bool pageNow) __attribute__((always_inline, hot, optimize("O3")));
inline void updateRomIndex(bool pageNow) __attribute__((always_inline, hot, optimize("O3")));
inline void writeRomData(uint16_t address) __attribute__((always_inline, hot, optimize("O3")));
inline void writePagedRomData(uint16_t address) __attribute__((always_inline, hot, optimize("O3")));
inline void writeDivMmcRomData(uint16_t address) __attribute__((always_inline, hot, optimize("O3")));

#ifdef DEBUG_OUTPUT

// Debug data buffer
const size_t DEBUG_BUFFER_SIZE = (64 * 1024);
RingBuffer<DEBUG_BUFFER_SIZE> debugBuffer;
volatile bool debugTraceOn = false;
volatile uint8_t debugTraceData = 0;

inline __attribute__((always_inline)) bool writeDebugData(uint8_t data)
{
    if (debugBuffer.canWrite())
    {
        debugBuffer.write(data);
        return true;
    }
    return false;
}

inline __attribute__((always_inline)) uint8_t readDebugData()
{
    uint8_t data;
    if (debugBuffer.read(&data))
    {
        return data;
    }
    return 0xFF;
}

inline __attribute__((always_inline)) bool hasDebugData()
{
    return debugBuffer.canRead();
}

inline __attribute__((always_inline)) void traceDebug(uint16_t address)
{
    if (debugTraceOn)
    {
        debugBuffer.write(romSelected);
        debugBuffer.write(address >> 8);
        debugBuffer.write(address);
        debugBuffer.write(debugTraceData);
        /*if (!writeDebugData(romSelected) ||
            !writeDebugData(address >> 8) ||
            !writeDebugData(address) ||
            !writeDebugData(debugTraceData))
        {
            debugTraceOn = false;
        }*/
    }
}

#endif

inline __attribute__((always_inline)) void writeData(uint8_t data)
{
    // Output D[7:0] to GPIO2/7
    uint32_t gpioSeven = ((data & 0x07) | ((data & 0x38) << 7) | ((data & 0xc0) << 10));
    CORE_PIN34_PORTSET = DATA_OUT_PIN_BITMASK;
    CORE_PIN10_DDRREG |= GPIO7_DATA_MASK;
    CORE_PIN10_PORTSET = gpioSeven & GPIO7_DATA_MASK;
    CORE_PIN10_PORTCLEAR = (~gpioSeven) & GPIO7_DATA_MASK;
#ifdef DEBUG_OUTPUT
    debugTraceData = data;
#endif
}

inline __attribute__((always_inline)) uint8_t readData()
{
    // Decode D[7:0] from GPIO2/7
    uint32_t gpioSeven = (*(volatile uint32_t *)IMXRT_GPIO7_ADDRESS);
    uint32_t data = ((gpioSeven & 0x07) | ((gpioSeven & 0x1c00) >> 7) |
        ((gpioSeven & 0x30000) >> 10));
    return data;
}

inline __attribute__((always_inline)) void disableData()
{
    // Set data direction to input
    CORE_PIN10_DDRREG &= ~GPIO7_DATA_MASK;
    CORE_PIN34_PORTCLEAR = DATA_OUT_PIN_BITMASK;
}

inline __attribute__((always_inline)) uint16_t decodeAddress(uint32_t gpioSix)
{
    // Decode A[15:0] from GPIO1/6
    return (gpioSix >> 16);
}

inline __attribute__((always_inline)) uint16_t decodeRamAddress(uint32_t gpioSix)
{
    // Decode A[12:0] from GPIO1/6
    return ((gpioSix & 0x1fff0000) >> 16);
}

inline __attribute__((always_inline)) uint8_t decodeLowAddress(uint32_t gpioSix)
{
    // Decode A[7:0] from GPIO1/6
    return ((gpioSix & 0x00ff0000) >> 16);
}

inline __attribute__((always_inline)) uint8_t decodeHighAddress(uint32_t gpioSix)
{
    // Decode A[15:8] from GPIO1/6
    return ((gpioSix & 0xff000000) >> 24);
}

inline __attribute__((always_inline)) void writeSdSpiWriteBuffer(sd_spi_action_t spiAction,
    uint8_t data)
{
    sdSpiFlagsBuffer.write((uint8_t)spiAction);
    sdSpiWriteBuffer.write(data);
}

inline __attribute__((always_inline)) sd_spi_action_t readSdSpiWriteBuffer(uint8_t* data)
{
    sd_spi_action_t spiAction = (sd_spi_action_t)sdSpiFlagsBuffer.readRaw();
    *data = sdSpiWriteBuffer.readRaw();
    return spiAction;
}

inline __attribute__((always_inline)) void resetSdSpi()
{
    sdSpiFlagsBuffer.clear();
    sdSpiWriteBuffer.clear();
    divMmcSpi.reset(false);
    divMmcHdf.reset(false);
    divMmcSecondHdf.reset(false);
}

// NOTE: sdSpiOnTick is main loop, so optimize
inline void sdSpiOnTick()
{
    if (sdSpiWriteBuffer.canRead())
    {
        uint8_t data;
        switch (readSdSpiWriteBuffer(&data))
        {
            case SD_SPI_SELECT :
                switch (data & 0x03)
                {
                    case 0x01 :
                        divMmcDrive = divMmcDriveSlot[1];
                        break;
                    case 0x02 :
                        divMmcDrive = divMmcDriveSlot[0];
                        break;
                    default :
                        divMmcDrive = DIVMMC_NONE;
                        break;
                }
                switch (divMmcDrive)
                {
                    case DIVMMC_SDHC :
                        if (!beginDivMmcSd())
                        {
                            divMmcDrive = DIVMMC_NONE;
                        }
                        break;
                    case DIVMMC_HDF_A :
                    case DIVMMC_HDF_B :
                        if (!beginSdfsSd())
                        {
                            divMmcDrive = DIVMMC_NONE;
                        }
                        break;
                    default :
                        break;
                }
                break;
            default :
                switch (divMmcDrive)
                {
                    case DIVMMC_SDHC :
                        divMmcSpi.performTick(data);
                        break;
                    case DIVMMC_HDF_A :
                        divMmcHdf.performTick(data);
                        break;
                    case DIVMMC_HDF_B :
                        divMmcSecondHdf.performTick(data);
                        break;
                    default :
                        break;
                }
                break;
        }
    }
}

// NOTE: sdSpiOnTick is main loop, so optimize
inline void zxC3OnTick()
{
    if (zxC3Present)
    {
        if (zxC3EraseBuffer.canRead())
        {
            switch (zxC3EraseTrigState)
            {
                case TRIGGER_HOLD :
                    if (!zxC3Write)
                    {
                        zxC3EraseTrigState = TRIGGER_DELAY;
                        zxC3EraseTrigExitCount = ZXC3_ERASE_DELAY_CNT;
                    }
                    break;
                case TRIGGER_DELAY :
                    if (!zxC3Write)
                    {
                        --zxC3EraseTrigExitCount;
                        if (zxC3EraseTrigExitCount == 0)
                        {
                            if (zxC3EraseBuffer.canRead())
                            {
                                uint8_t sector = zxC3EraseBuffer.readRaw();
                                if (mldPresent)
                                {
                                    // Dandandator 512KB flash uses 4KB sectors
                                    if (sector < 0xFF)
                                    {
                                        uint16_t address = (sector & 0x0001) ? 0x1000 : 0x0000;
                                        sector >>= 1;
                                        if (sector >= RAM_PAGE_COUNT)
                                        {
                                            memset((void*)&(divMmcExtRamArray[(sector - RAM_PAGE_COUNT)][address]),
                                                0xFF, (RAM_PAGE_SIZE / 2));
                                        } else {
                                            memset((void*)&(divMmcRamArray[sector][address]),
                                                0xFF, (RAM_PAGE_SIZE / 2));
                                        }
                                    } else {
                                        memset((void*)&(divMmcRamArray[0][0]),
                                            0xFF, (RAM_PAGE_COUNT * RAM_PAGE_SIZE));
                                        memset((void*)&(divMmcExtRamArray[0][0]),
                                            0xFF, (EXT_RAM_PAGE_COUNT * RAM_PAGE_SIZE));
                                    }
                                } else {
                                    // ZXC3 128KB flash uses 16KB sectors
                                    if (sector < 0xFF)
                                    {
                                        memset((void*)&(divMmcExtRamArray[sector][0]),
                                            0xFF, ROM_PAGE_SIZE);
                                    } else {
                                        memset((void*)&(divMmcExtRamArray[0][0]),
                                            0xFF, (ZXC3_PAGE_COUNT * RAM_PAGE_SIZE));
                                    }
                                }
                            }
                            if (zxC3EraseBuffer.canRead())
                            {
                                zxC3EraseTrigExitCount = ZXC3_ERASE_DELAY_CNT;
                            } else {
                                zxC3EraseTrigState = TRIGGER_READY;
                                zxC3EraseBusy = false;
                            }
                        }
                    } else {
                        zxC3EraseTrigExitCount = ZXC3_ERASE_DELAY_CNT;
                    }
                    break;
                default :
                    if (!zxC3Write)
                    {
                        zxC3EraseTrigState = TRIGGER_HOLD;
                    }
                    break;
            }
        }

        switch (zxC3WriteTrigState)
        {
            case TRIGGER_ACTIVE :
                if (!zxC3Write)
                {
                    zxC3WriteTrigState = TRIGGER_HOLD;
                }
                break;
            case TRIGGER_HOLD :
                if (!zxC3Write)
                {
                    zxC3WriteTrigState = TRIGGER_DELAY;
                    zxC3WriteTrigExitCount = TRIGGER_DELAY_CNT;
                }
                break;
            case TRIGGER_DELAY :
                if (!zxC3Write)
                {
                    --zxC3WriteTrigExitCount;
                    if (zxC3WriteTrigExitCount == 0)
                    {
                        if (mldPresent)
                        {
                            saveMldRomFile(menuGetBrowserPath());
                        } else if (mdrEnabled)
                        {
                            saveMdrEmulatorFile(menuGetBrowserPath());
                        } else {
                            saveZXC3RomFile(menuGetBrowserPath());
                        }
                        zxC3WriteTrigState = TRIGGER_READY;
                    }
                } else {
                    zxC3WriteTrigExitCount = TRIGGER_DELAY_CNT;
                }
                break;
            default :
                break;
        }
    }
}

inline __attribute__((always_inline)) bool isGlobalStateReset()
{
    return ((globalState & 0x02) != 0);
}

inline __attribute__((always_inline)) bool isDivMmcSelected()
{
    return (divMmcEnabled && ((romArraySelected & (BANK_MF128 | BANK_IF1)) == 0));
}

inline __attribute__((always_inline)) void divMmcUpdateInterfaceOne()
{
    if (!interface1Present || isDivMmcSelected())
    {
        digitalWriteFast(IF1_DIS_PIN, 1);
    } else {
        digitalWriteFast(IF1_DIS_PIN, 0);
    }
}

inline __attribute__((always_inline)) void disableInternalRom()
{
    digitalWriteFast(ROMCS_PIN, 1);
    romEnabled = true;
}

inline __attribute__((always_inline)) void enableInternalRom()
{
    digitalWriteFast(ROMCS_PIN, 0);
    romEnabled = false;
}

void setState(run_state_t state_)
{
    switch (state_)
    {
        case STATE_RESET :
        case STATE_RESET_MENU :
            resetTrigState = TRIGGER_ACTIVE;
            digitalWriteFast(DATA_DIS_PIN, 1);
            digitalWriteFast(RESET_PIN, 1);
            digitalWriteFast(LED_PIN, 0);
            enableInternalRom();
            disableData();
            break;
        case STATE_ROM_ENABLE :
            resetTrigState = TRIGGER_HOLD;
            resetHardTrigCount = HARD_RESET_DELAY_CNT;
            digitalWriteFast(DATA_DIS_PIN, 0);
            digitalWriteFast(RESET_PIN, 0);
            digitalWriteFast(LED_PIN, 1);
            break;
        case STATE_ROM_DISABLE :
            resetTrigState = TRIGGER_HOLD;
            resetHardTrigCount = HARD_RESET_DELAY_CNT;
            digitalWriteFast(DATA_DIS_PIN, 1);
            digitalWriteFast(RESET_PIN, 0);
            digitalWriteFast(LED_PIN, 0);
            enableInternalRom();
            disableData();
            break;
    }
    globalState = state_;
}

void sdFatDateTime(uint16_t* date, uint16_t* time)
{
  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(year(), month(), day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(hour(), minute(), second());
}

FLASHMEM void startup_early_hook()
{
    // Force initial reset
    IOMUXC_GPR_GPR28 = 0xFFFFFFFF;
    pinMode(RESET_PIN, OUTPUT);
    digitalWriteFast(RESET_PIN, 1);

    // Set the data bus to high impedance
    IOMUXC_GPR_GPR29 = 0xFFFFFFFF;
    pinMode(DATA_DIS_PIN, OUTPUT);
    digitalWriteFast(DATA_DIS_PIN, 1);
}

FLASHMEM void startup_middle_hook()
{
    // force millis() to be 300 to skip startup delays
    systick_millis_count = 300;
}

void setup()
{
    // Apply slight overclock
    set_arm_clock(TEENSY_CLK_FREQ);

    // Force initial reset
    pinMode(RESET_PIN, OUTPUT);
    digitalWriteFast(RESET_PIN, 1);

    // Set the data bus to high impedance
    pinMode(DATA_DIS_PIN, OUTPUT);
    digitalWriteFast(DATA_DIS_PIN, 1);

    // Configure DIR pin to high speed
    pinMode(DATA_OUT_PIN, OUTPUT);
    CORE_PIN23_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);

    // Configure spectrum I/Os
    for (uint8_t i_ = 0; i_ < sizeof(OUTPUT_PINS); i_++) pinMode(OUTPUT_PINS[i_], OUTPUT);
    for (uint8_t i_ = 0; i_ < sizeof(INPUT_PINS); i_++) pinMode(INPUT_PINS[i_], INPUT_PULLUP);
    for (uint8_t i_ = 0; i_ < sizeof(DATA_PINS); i_++) pinMode(DATA_PINS[i_], OUTPUT);

    // Set data bus to high speed
    CORE_PIN6_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);
    CORE_PIN7_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);
    CORE_PIN8_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);
    CORE_PIN9_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);
    CORE_PIN10_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);
    CORE_PIN11_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);
    CORE_PIN12_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);
    CORE_PIN32_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);

    // Force the spectrum into reset
    globalCycleCount = ARM_DWT_CYCCNT;
    setState(STATE_RESET);

    // Detect debug and crashes
    menuClearDebug();
    if (CrashReport)
    {
        PrintableString report;
        CrashReport.printTo(report);
        menuPrintDebug(false, "%s", report.c_str());
    }
#ifdef DEBUG_OUTPUT
    Serial.begin(115200);
#endif

    // Configure UART
    Serial8.addMemoryForRead(uartBuffer, UART_BUFFER_SIZE);

    // Set SdFat date and time callback
    FsDateTime::setCallback(sdFatDateTime);

    // Setup RD, WR, ROMCS, reset and button ISRs
    // NOTE: Set GPIO interrupt as high priority, to avoid misses
    attachInterrupt(digitalPinToInterrupt(RD_PIN), isrRdEvent, CHANGE);
    attachInterrupt(digitalPinToInterrupt(WR_PIN), isrWrEvent, FALLING);
    attachInterrupt(digitalPinToInterrupt(ROMCS_IN_PIN), isrRdEvent, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RESET_IN_PIN), isrPinReset, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isrPinButton, FALLING);
    attachInterruptVector(IRQ_GPIO6789, &isrFastGpios);
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 16);
}

bool beginSdCard()
{
    // Quick detect the presence of the SD card
    pinMode(SD_CS_PIN, INPUT_PULLDOWN);
    delayMicroseconds(5);
    if (digitalReadFast(SD_CS_PIN))
    {
        // Permit a number of retries to enable the SD card
        int retries_ = 0;
        while (!SD.sdfs.begin(SdioConfig(FIFO_SDIO)))
        {
            if (++retries_ >= NUM_SD_RETRIES)
            {
                return false;
            }
        }
        return true;
    }
    return false;
}

bool beginSdfsSd()
{
    if (sdioEnabled)
    {
        // Ensure SDIO accesses are finished before continuing
        divMmcSpi.end();
        while (SD.sdfs.card()->isBusy()) { yield(); };
        sdioEnabled = false;
    }
    return !sdioEnabled;
}

bool beginDivMmcSd()
{
    if (!sdioEnabled)
    {
        // Close disk images
        divMmcHdf.end();
        divMmcSecondHdf.end();

        // Ensure sdfs accesses are finished before continuing
        while (SD.sdfs.card()->isBusy()) { yield(); };

        // Enable DivMMC over SDIO
        divMmcSpi.begin(SD.sdfs.card(), divMmcSdReadOnly);
        sdioEnabled = true;
    }
    return sdioEnabled;
}

uint16_t loadRomImage(const char* filename, char* ptr, const uint16_t size)
{
    uint16_t count = 0;
    File RomFile = SD.open(filename, FILE_READ);
    if (RomFile)
    {
        count = RomFile.readBytes(ptr, size);
        RomFile.close();
    }
    return count;
}

void loadSpectrumRomFile()
{
    // Attempt to load four 16KB ROM banks
    File RomFile = menuGetSpectrumRomFile();
    if (RomFile)
    {
        size_t count = RomFile.readBytes((char *)romArray[ROM_PAGE_ROM0], ROM_PAGE_SIZE);
        if (count > 0)
        {
            romArrayPresent |= BANK_ROM0;
            if (count >= ROM_PAGE_SIZE)
            {
                count = RomFile.readBytes((char *)romArray[ROM_PAGE_ROM1], ROM_PAGE_SIZE);
                if (count > 0)
                {
                    rom1Present = true;
                    romArrayPresent |= BANK_ROM1;
                    if (count >= ROM_PAGE_SIZE)
                    {
                        count = RomFile.readBytes((char *)romArray[ROM_PAGE_ROM2],
                            ROM_PAGE_SIZE);
                        if (count > 0)
                        {
                            rom23Present = true;
                            romArrayPresent |= (BANK_ROM2 | BANK_ROM3);
                            count = RomFile.readBytes((char *)romArray[ROM_PAGE_ROM3],
                                ROM_PAGE_SIZE);
                        }
                    }
                }
            }
        }
        RomFile.close();
    }
}

bool loadZXC2RomFile(File RomFile)
{
    // The ZXC2 cartridge is loaded into the DivMMC RAM area
    if (RomFile)
    {
        size_t count = RomFile.readBytes((char *)divMmcExtRamArray[0], RAM_PAGE_SIZE);
        if (count > 0)
        {
            zxC2Present = true;
            zxC2ShadowRom = (strncmp("SPECTRA_", RomFile.name(), 8) == 0);
            divMmcExtRamEnabled = false;
            romArrayPresent |= BANK_RAM;
            for (uint8_t i_ = 1; i_ < EXT_RAM_PAGE_COUNT; ++i_)
            {
                size_t blk_count_ = RomFile.readBytes((char *)divMmcExtRamArray[i_],
                    RAM_PAGE_SIZE);
                count += blk_count_;
                if (blk_count_ < RAM_PAGE_SIZE)
                {
                    break;
                }
            }
        }
        RomFile.close();
    }
    return zxC2Present;
}

void saveZXC3RomFile(const char* filePath)
{
    bool result = false;
    File saveFile = SD.open(filePath, FILE_WRITE_BEGIN);
    if (saveFile)
    {
        result = true;
        for (uint8_t i_ = 0; i_ < ZXC3_PAGE_COUNT; ++i_)
        {
            if (saveFile.write((uint8_t*)divMmcExtRamArray[i_], RAM_PAGE_SIZE) < RAM_PAGE_SIZE)
            {
                result = false;
                break;
            }
        }
        saveFile.close();
    }
    if (!result)
    {
        menuPrintDebug(false, F_CSTR("Failed to save ZXC3 '%s'"), filePath);
    }
}

void saveMldRomFile(const char* filePath)
{
    bool result = false;
    File saveFile = SD.open(filePath, FILE_WRITE_BEGIN);
    if (saveFile)
    {
        if (saveFile.write((uint8_t*)divMmcRamArray[0], (RAM_PAGE_COUNT * RAM_PAGE_SIZE)) >=
            (RAM_PAGE_COUNT * RAM_PAGE_SIZE))
        {
            if (saveFile.write((uint8_t*)divMmcExtRamArray[0],
                (EXT_RAM_PAGE_COUNT * RAM_PAGE_SIZE)) >= (EXT_RAM_PAGE_COUNT * RAM_PAGE_SIZE))
            {
                result = true;
            }
        }
        saveFile.close();
    }
    if (!result)
    {
        menuPrintDebug(false, F_CSTR("Failed to save MLD '%s'"), filePath);
    }
}

inline volatile uint8_t* mldGetSlotPtr(uint8_t page)
{
    page <<= 1;
    if (page >= RAM_PAGE_COUNT)
    {
        return divMmcExtRamArray[page - RAM_PAGE_COUNT];
    }
    return divMmcRamArray[page];
}

uint8_t mldFindHeaderSlot()
{
    for (uint8_t slot = 0; slot < mldSlotCount; ++slot)
    {
        volatile uint8_t* ptr = mldGetSlotPtr(slot);
        if (memcmp((const void*)&(ptr[MLD_SIGNATURE_OFFSET]), "MLD", 3) == 0)
        {
            return slot;
        }
    }
    return 0xFF;
}

bool mldRelocateToSlotZero(uint8_t headerSlot)
{
    // Read the MLD header directly from its ROM slot
    volatile uint8_t* ptr = mldGetSlotPtr(headerSlot);
    uint8_t baseSlot = ptr[MLD_HEADER_OFFSET];
    uint8_t requiredSectors = ptr[MLD_HEADER_OFFSET + 2];
    uint16_t tableOffset = ptr[MLD_HEADER_OFFSET + 7] |
        (ptr[MLD_HEADER_OFFSET + 8] << 8);
    uint16_t tableRowSize = ptr[MLD_HEADER_OFFSET + 9] |
        (ptr[MLD_HEADER_OFFSET + 10] << 8);
    uint16_t tableRows = ptr[MLD_HEADER_OFFSET + 11] |
        (ptr[MLD_HEADER_OFFSET + 12] << 8);
    uint8_t rowSlotOffset = ptr[MLD_HEADER_OFFSET + 13];
    uint8_t tableSlot = tableOffset / ROM_PAGE_SIZE;
    uint16_t tableSlotOffset = tableOffset & (ROM_PAGE_SIZE - 1);
    uint8_t saveSlot = mldSlotCount +
        ((requiredSectors + MLD_SECTORS_PER_SLOT - 1) / MLD_SECTORS_PER_SLOT);

    // The relocation table must fit in a loaded MLD slot
    if ((tableSlot >= mldSlotCount) || (saveSlot > MLD_MAX_SLOT_COUNT) ||
        (requiredSectors > MLD_MAX_SAVE_SECTOR_COUNT))
    {
        return false;
    }

    // Update the relocation table
    volatile uint8_t* tablePtr = mldGetSlotPtr(tableSlot);
    for (uint16_t row = 0; row < tableRows; ++row)
    {
        // Each row contains one slot reference to adjust by the header base slot
        uint32_t rowOffset = tableSlotOffset + ((uint32_t)row * tableRowSize) +
            rowSlotOffset;
        if (rowOffset >= ROM_PAGE_SIZE)
        {
            return false;
        }

        volatile uint8_t* slotPtr = &(tablePtr[rowOffset]);
        uint8_t oldValue = *slotPtr;
        uint8_t oldSlot = oldValue & 0x7F;
        *slotPtr = (oldValue & 0x80) | ((oldSlot - baseSlot) & 0x7F);
    }

    // Store the save slot sectors
    if (requiredSectors > 0)
    {
        uint8_t lastMldSaveSector = (MLD_SECTORS_PER_SLOT * saveSlot) - 1;
        for (uint8_t sector = 0; sector < requiredSectors; ++sector)
        {
            ptr[MLD_HEADER_OFFSET + 3 + sector] = lastMldSaveSector--;
        }
    }

    // Slot zero is now the base slot
    ptr[MLD_HEADER_OFFSET] = 0;
    return true;
}

bool loadMldRomFile(File RomFile)
{
    // The ZXC2 cartridge is loaded into the DivMMC RAM area
    if (RomFile)
    {
        size_t count = RomFile.readBytes((char *)divMmcRamArray[0],
            (RAM_PAGE_COUNT * RAM_PAGE_SIZE));
        if (count > 0)
        {
            mldPresent = true;
            divMmcExtRamEnabled = false;
            romArrayPresent |= BANK_RAM;
            for (uint8_t i_ = 0; i_ < EXT_RAM_PAGE_COUNT; ++i_)
            {
                size_t blk_count_ = RomFile.readBytes((char *)divMmcExtRamArray[i_],
                    RAM_PAGE_SIZE);
                count += blk_count_;
                if (blk_count_ < RAM_PAGE_SIZE)
                {
                    break;
                }
            }
        }
        if (mldPresent)
        {
            // Relocate MLD slots as necessary - assume full ROM images are already
            // laid out from slot zero
            mldSlotCount = (count + ROM_PAGE_SIZE - 1) / ROM_PAGE_SIZE;
            if (mldSlotCount < MLD_MAX_SLOT_COUNT)
            {
                uint8_t headerSlot = mldFindHeaderSlot();
                if ((headerSlot != 0xFF) && !mldRelocateToSlotZero(headerSlot))
                {
                    mldPresent = false;
                    mldSlotCount = 0;
                }
            }
        }
        RomFile.close();
    }
    return mldPresent;
}

bool loadSnapshotFile(File RomFile, bool isSnaFile)
{
    // Convert the Z80 snapshot into a loader ROM, in the DivMMC RAM area
    if (RomFile)
    {
        snaLoaderBanks = convertZ80toROM(RomFile, (uint8_t*)divMmcExtRamArray[0],
            (uint8_t*)divMmcExtRamArray[RAM_PAGE_COUNT], isSnaFile);
        if (snaLoaderBanks > 0)
        {
            snaLoaderBanks <<= 1;
            snaLoaderPresent = true;
            divMmcExtRamEnabled = false;
            romArrayPresent |= BANK_RAM;

            // Copy the loader for final stage into scratch RAM
            memcpy((void*)menuRamArray[2], (void*)divMmcExtRamArray[0], ROM_PAGE_SIZE);
        }
        RomFile.close();
    }
    return snaLoaderPresent;
}

bool loadMdrEmulatorFile(const char* fileName)
{
    bool result = false;
    File mdrFile = SD.open(fileName, FILE_READ);
    if (mdrFile)
    {
        uint16_t i_, j_;
        uint8_t buffer[0x21F];
        mdrMaxSector = 0;

        // The emulator can load microdrives with 180 used sectors,
        // in pages 1 to 6. Microdrive files may be sparse, so re-pack to fit
        // the emulator.
        uint8_t newSector = MDR_MAX_SECTOR;
        for (i_ = 2; i_ < 14; i_ += 2)
        {
            for (j_ = 0; j_ < 0x1E; ++j_)
            {
                uint8_t* ptr = (uint8_t*)divMmcExtRamArray[i_] + (j_ * 0x220);
                if (mdrFile.readBytes((char*)(ptr + 1), 0x21F) >= 0x21F)
                {
                    // Replace HDNUMB with new sector
                    if (ptr[2] > mdrMaxSector)
                    {
                        mdrMaxSector = ptr[2];
                    }
                    ptr[0] = 0x00;
                    ptr[2] = newSector--;

                    // Update HDCHK for the header
                    ptr[15] = 0;
                    for (uint16_t x = 1; x < 15; ++x)
                    {
                        uint16_t y = ptr[15] + ptr[x];
                        if (y >= 0x100)
                        {
                            ptr[15] += ptr[x] + 1;
                        } else {
                            ptr[15] += ptr[x];
                        }
                    }
                } else {
                    result = true;
                    break;
                }
            }
            if (result)
            {
                break;
            }
        }

        // Continue to load sectors that are not empty, into any blank sectors
        // that had been loaded from the file
        i_ = 2;
        j_ = 0;
        while (!result)
        {
            // Find an existing blank sector
            bool isUsed = false;
            uint8_t* ptr = (uint8_t*)divMmcExtRamArray[i_] + (j_ * 0x220);
            for (uint16_t k_ = 16; k_ < 0x220; ++k_)
            {
                if (ptr[k_] != 0x00)
                {
                    isUsed = true;
                    break;
                }
            }

            // Attempt to load a sector from file to replace this blank sector
            if (!isUsed)
            {
                if (mdrFile.readBytes((char*)buffer, 0x21F) >= 0x21F)
                {
                    // Test that the loaded sector is not blank itself
                    for (uint16_t k_ = 15; k_ < 0x21F; ++k_)
                    {
                        if (buffer[k_] != 0x00)
                        {
                            // Replace HDNUMB with existing new sector
                            if (buffer[1] > mdrMaxSector)
                            {
                                mdrMaxSector = buffer[1];
                            }
                            ptr[1] = buffer[0];
                            ptr[2] = newSector--;
                            memcpy(&(ptr[3]), &(buffer[2]), 0x21D);
                            isUsed = true;

                            // Update HDCHK for the header
                            ptr[15] = 0;
                            for (uint16_t x = 1; x < 15; ++x)
                            {
                                uint16_t y = ptr[15] + ptr[x];
                                if (y >= 0x100)
                                {
                                    ptr[15] += ptr[x] + 1;
                                } else {
                                    ptr[15] += ptr[x];
                                }
                            }
                            break;
                        }
                    }
                    if (!isUsed)
                    {
                        --j_;
                    }
                } else {
                    result = true;
                    break;
                }
            }

            // Increment to the next sector
            if (++j_ >= 0x1E)
            {
                j_ = 0;
                i_ += 2;
                if (i_ >= 14)
                {
                    // Detect if there are any more valid sectors in the file,
                    // that cannot be loaded into the emulator
                    result = true;
                    while (mdrFile.readBytes((char*)buffer, 0x21F) >= 0x21F)
                    {
                        for (uint16_t k_ = 15; k_ < 0x21F; ++k_)
                        {
                            if (buffer[k_] != 0x00)
                            {
                                // Fail to load if more data is found, to avoid
                                // corruption
                                result = false;
                                break;
                            }
                        }
                        if (!result)
                        {
                            break;
                        }
                    }
                    break;
                }
            }
        }
        mdrFile.close();
    }

    // Load the microdrive emulator ROM
    if (result &&
        (loadRomImage(MDR_EMULATOR_ROM_PATH, (char *)divMmcExtRamArray[0],
            ROM_PAGE_SIZE) > 0))
    {
        zxC2Present = true;
        zxC3Present = true;
        zxC2ShadowRom = true;
        divMmcExtRamEnabled = false;
        romArrayPresent |= BANK_RAM;
    }
    return zxC2Present;
}

void saveMdrEmulatorFile(const char* fileName)
{
    bool result = false;
    File mdrFile = SD.open(fileName, FILE_WRITE_BEGIN);
    if (mdrFile)
    {
        // Load the header from the first sector
        uint8_t buffer[0x21F];
        uint8_t sector = mdrMaxSector;
        uint8_t* ptr = (uint8_t*)divMmcExtRamArray[2];
        memcpy(buffer, ptr + 1, 15);
        result = true;

        // Generate empty sectors that had been truncated on load
        memset(&(buffer[0x0F]), 0x00, 0x210);
        while (sector > MDR_MAX_SECTOR)
        {
            // Replace HDNUMB with new sector
            buffer[1] = sector--;

            // Update HDCHK for the header
            buffer[14] = 0;
            for (uint16_t x = 0; x < 14; ++x)
            {
                uint16_t y = buffer[14] + buffer[x];
                if (y >= 0x100)
                {
                    buffer[14] += buffer[x] + 1;
                } else {
                    buffer[14] += buffer[x];
                }
            }

            // Write header and zero payload
            if (mdrFile.write(buffer, 0x21F) < 0x21F)
            {
                result = false;
                break;
            }
        }

        // Write out the existing sectors from pages 1 to 6
        for (uint8_t i_ = 2; i_ < 14; i_ += 2)
        {
            if (result)
            {
                for (uint8_t j_ = 0; j_ < 0x1E; ++j_)
                {
                    uint8_t* ptr = (uint8_t*)divMmcExtRamArray[i_] + (j_ * 0x220);
                    if (*ptr != 0xFF)
                    {
                        if (mdrFile.write(ptr + 1, 0x21F) >= 0x21F)
                        {
                            --sector;
                        } else {
                            result = false;
                            break;
                        }
                    }
                }
            } else {
                break;
            }
        }
        mdrFile.close();
    }
    if (!result)
    {
        menuPrintDebug(false, F_CSTR("Failed to save MDR '%s'"), fileName);
    }
}

bool loadForegroundRom()
{
    // Enable the DivMMC RAM
    divMmcExtRamEnabled = divMmcExtRamPresent;

    // Open and load the foreground ROM, if present
    rom_type_t romType;
    File RomFile = menuGetForegroundRomFile(&romType);
    switch (romType)
    {
        case TYPE_CART :
            // Interface 2 cartridge is ZXC2 with paging locked
            zxC2Lock = true;
        case TYPE_ZXC2 :
            return loadZXC2RomFile(RomFile);
        case TYPE_ZXC3 :
            if (loadZXC2RomFile(RomFile))
            {
                zxC3Present = true;
            } else {
                return false;
            }
            break;
        case TYPE_MLD :
            if (loadMldRomFile(RomFile))
            {
                zxC3Present = true;
            } else {
                return false;
            }
            break;
        case TYPE_Z80 :
        case TYPE_SNA :
            return loadSnapshotFile(RomFile, (romType != TYPE_Z80));
        default :
            break;
    }
    return true;
}

void initialiseRamBanks()
{
    romArrayPresent &= ~(BANK_RAM);
    memset((void*)divMmcRamArray, 0xFF, (RAM_PAGE_COUNT * RAM_PAGE_SIZE));
    memset((void*)divMmcExtRamArray, 0xFF, (EXT_RAM_PAGE_COUNT * RAM_PAGE_SIZE));
    memset((void*)&(romArray[ROM_PAGE_MF128][RAM_PAGE_SIZE]), 0xFF, RAM_PAGE_SIZE);
    memset((void*)menuRamArray, 0xFF, (MENU_PAGE_COUNT * RAM_PAGE_SIZE));
}

void performHardReset()
{
    // Disable the ESP-01S
    pinMode(ESP_ENABLE, OUTPUT);
    digitalWriteFast(ESP_ENABLE, 0);

    // Clear the UART
    wifiNtp.end();
    espUart.end();
    wifiNtpEnabled = false;

    // Perform reset into menu
    afterFirstReset = false;
    isDeviceDisabled = false;
    menuEnterOnReset = true;
    setState(STATE_RESET);
}

void handleStateResetEntry()
{
#ifdef DEBUG_OUTPUT
    if (afterFirstReset)
    {
        // Stop the trace
        debugTraceOn = false;

        // Dump the debug buffer
        Serial.printf("START\n");
        Serial.printf("Trace %db\n", debugBuffer.getSize());
        while (hasDebugData())
        {
            while ( (uint) Serial.availableForWrite() <  0x400);
            //Serial.printf("%02x:%02x%02x@%02x, ", readDebugData(), readDebugData(),
            //    readDebugData(), readDebugData());
            Serial.printf("%02x:%02x, ", readDebugData(), readDebugData());
        }
        Serial.printf("\nEND\n");
        while ( (uint) Serial.availableForWrite() <  0x400);

        // Dump the RAM banks
        /*for (uint8_t i_ = 0; i_ < RAM_PAGE_COUNT; ++i_)
        {
            for (uint16_t j_ = 0; j_ < RAM_PAGE_SIZE; j_ += 0x400)
            {
                while ( (uint) Serial.availableForWrite() <  0x400);
                Serial.write((char*)&divMmcRamArray[i_][j_], 0x400);
            }
        }
        for (uint8_t i_ = 0; i_ < EXT_RAM_PAGE_COUNT; ++i_)
        {
            for (uint16_t j_ = 0; j_ < RAM_PAGE_SIZE; j_ += 0x400)
            {
                while ( (uint) Serial.availableForWrite() <  0x400);
                Serial.write((char*)&divMmcExtRamArray[i_][j_], 0x400);
            }
        }*/

        // Reset the trace buffer
        debugBuffer.clear();
    }
#endif

    // Detect first reset, or button being pressed, for menu ROM
    bool isButtonHeld = false;
    if (!digitalReadFast(BUTTON_PIN))
    {
        isButtonHeld = true;
        menuEnterOnReset = true;

        // Wait for button release
        while (!digitalReadFast(BUTTON_PIN))
        {
            delay(75);
        }
    }

    // Reset the soft ROM detection state
    if (menuEnterOnReset || (stateActiveSlot >= 0))
    {
        // Reset and reload the configuration to enter the menu, or load state
        afterFirstReset = false;
        isDeviceDisabled = false;
    }
    if (!afterFirstReset)
    {
        zxC2Present = false;
        zxC3Present = false;
        mldPresent = false;
        mdrPresent = false;
        snaLoaderPresent = false;
        tzxPresent = false;
        menuClearConfiguration();

        // Load the ROMs
        loadRomSets = true;

        // Ensure the DivMMC is closed and idle
        divMmcHdf.reset(true);
        divMmcSecondHdf.reset(true);
        divMmcSpi.reset(true);

        // Clear the menu action
        menuResetAction();
    }

    // Initialise the RAM banks
    if (loadRomSets || !divMmcPreserveRam)
    {
        initialiseRamBanks();
    }

    // Update the RTC registers, if necessary
    rtcTeensy.updateRtc();

    // Initialise the device soft ROMs
    delay(250);
    if (loadRomSets)
    {
        // Reset the soft ROM state
        romArrayPresent = 0;
        rom1Present = false;
        rom23Present = false;
        loadRomSets = false;

        // Reset the "dynamic ROM" state
        zxC2Present = false;
        zxC3Present = false;
        mldPresent = false;
        zxC2ShadowRom = false;
        snaLoaderPresent = false;

        // Load configuration and soft ROMs
        if (!isDeviceDisabled)
        {
            // Load the built-in Interface 1 soft ROM
#ifdef ENABLE_BUILTIN_ROM_IF1
            memcpy((void *)romArray[ROM_PAGE_IF1], BUILTIN_ROM_IF1, BUILTIN_ROM_IF1_SIZE);
            romArrayPresent |= BANK_IF1;
#endif

            // Detect the SD card
            if (!sdCardPresent)
            {
                sdCardPresent = beginSdCard();
            }

            // Load ROMs from the SD card
            if (sdCardPresent && beginSdfsSd())
            {
                // Load initial configuration
                if (!afterFirstReset)
                {
                    // NOTE: Restore the active save slot, if loading
                    int8_t prevStateActiveSlot = stateActiveSlot;
                    menuLoadConfiguration(0);
                    if (prevStateActiveSlot >= 0)
                    {
                        stateActiveSlot = prevStateActiveSlot;
                    }
                }

                // Load Interface 1 ROM
                if (loadRomImage(IF1_ROM_PATH, (char *)romArray[ROM_PAGE_IF1],
                    ROM_PAGE_SIZE) > 0)
                {
                    romArrayPresent |= BANK_IF1;
                } else if ((romArrayPresent & BANK_IF1) == 0)
                {
                    interface1Present = false;
                }

                // Load DivMMC Esxdos ROM
                if (loadRomImage(ESXMMC_BIN_PATH, (char *)romArray[ROM_PAGE_DIVMMC],
                    RAM_PAGE_SIZE) > 0)
                {
                    romArrayPresent |= BANK_DIVMMC;
                } else {
                    divMmcRomPresent = false;
                }

                // Load Multiface 128 ROM
                if (loadRomImage(MF128_ROM_PATH, (char *)romArray[ROM_PAGE_MF128],
                    RAM_PAGE_SIZE) > 0)
                {
                    romArrayPresent |= BANK_MF128;
                } else {
                    mf128Present = false;
                }

                // Load ZX LPrint III ROM
                if (loadRomImage(LPRINT_ROM_PATH, (char *)lprintRom,
                    LPRINT_ROM_SIZE) > 0)
                {
                    romArrayPresent |= BANK_LPRINT;
                } else {
                    lprintPresent = false;
                }

                // Load menu ROM
                if (loadRomImage(MENU_ROM_PATH, (char *)romArray[ROM_PAGE_MENU],
                    RAM_PAGE_SIZE) > 0)
                {
                    romArrayPresent |= BANK_MENU;

                    // Initialise the menu
                    menuInitialise(romArray[ROM_PAGE_MENU], menuRamArray[0]);
                } else {
                    menuEnableInGame = false;
                }

                // Attempt to restore directly into any active saved state
                if (!isButtonHeld && (stateActiveSlot >= 0) &&
                    !digitalReadFast(ROMCS_IN_PIN))
                {
                    stateStartLoad = stateLoadOnStartup();
                }

                // Load Spectrum ROM
                loadSpectrumRomFile();
                if (!stateStartLoad)
                {
                    // Load the configured foreground ROM
                    if (!loadForegroundRom() && afterFirstReset)
                    {
                        menuEnterOnReset = true;
                        afterFirstReset = false;
                        handleStateResetEntry();
                        return;
                    }

                    // Page in the menu ROM on startup
                    if ((menuEnterOnReset || (!afterFirstReset && bootIntoMenu)) &&
                        !digitalReadFast(ROMCS_IN_PIN) &&
                        ((romArrayPresent & BANK_MENU) != 0))
                    {
                        menuPrevRomPtr = romArray[ROM_ROM0];
                        menuPrevRomSelected = romSelected;
                        PAGE_IN_ROM(ROM_MENU);
                        menuRamPtr = menuRamArray[0];
                    }
                }
            } else if (!isButtonHeld)
            {
                // Button without SD card disables the built-in Interface 1 soft ROM
                interface1Present = ((romArrayPresent & BANK_IF1) != 0);
            }
        }
    } else {
        // Unlock 128K paging when ROM present
        rom1Present = ((romArrayPresent & BANK_ROM1) != 0);
    }
}

void handleStateResetMenu()
{
    // Perform the menu action
    menuPerformAction();

    // Perform a full reset
    handleStateResetEntry();
}

void handleWarmStateReset()
{
    if (IS_ROM_PRIORITY(ROM_MENU))
    {
        // Ensure menu actions are performed on reset from menu
        if (globalState != STATE_RESET_MENU)
        {
            menuResetAction();
            setState(STATE_RESET_MENU);
        }
    } else if ((romArrayPresent & BANK_RAM) != 0)
    {
        // Reload the menu after ZXC2 cartridge
        menuEnterOnReset = true;
    } else {
        // Preserve DivMMC RAM when present
        divMmcPreserveRam = divMmcPresent;
    }
}

void handleStateReset()
{
    // Delay to allow reset to take effect
    delay(250);

    // Enable the ESP-01S
    pinMode(ESP_ENABLE, INPUT_PULLUP);

    // Blink the LED
    digitalWriteFast(LED_PIN, 1);

    // Clear any pending NMI
    nmiPending = false;
    nmiRomTarget = ROM_ROM0;
    digitalWriteFast(NMI_PIN, 0);

    // Handle actions before warm reset
    if (afterFirstReset)
    {
        handleWarmStateReset();
    }

    // Reset the UART state, and clear buffers
    httpStopServer();
    if (!wifiNtpEnabled)
    {
        // NOTE: wifiNtpEnabled persists UART across reset
        espUart.end();
    }

    // Stop the tape
    if (tzxEnabled)
    {
        tzxPlayer.end();
    }

    // Stop the disk
    if (dskEnabled)
    {
        dskController.end();
    }

    // Stop the printer
    if (printerEnabled || lprintEnabled)
    {
        printerPort.end();
    }

    // Reset the banking state
    romPaged = 0x01;
    interface1Enabled = false;
    divMmcEnabled = false;
    divMmcRomEnabled = false;
    divMmcToggle = false;
    divMmcConMem = false;
    divMmcAutoMap = false;
    divMmcMapRam = false;
    divMmcRamBank = 0;
    divMmcRamPtr = divMmcRamArray[0];
    divMmcRamBankThree = false;
    mf128Enabled = false;
    mf128ActiveNMI = false;
    menuTriggerNMI = false;
    zxC2Lock = false;
    zxC2RomBank = 0x00;
    zxC2ShadowRom = false;
    zxC3Write = false;
    zxC3EraseBusy = false;
    zxC3FlashState = ZXC3_FLASH_IDLE;
    zxC3FlashSetup = false;
    zxC3WriteTrigState = TRIGGER_READY;
    mldCmdLocked = false;
    mldCmdDisabled = false;
    mldCurrentSlot = 1;
    mldPreviousSlot = 1;
    mldClearCommand();
    mldCmdData1 = 0;
    mldCmdData2 = 0;
    mldEepProgram = MLD_EEP_IDLE;
    mldEepSector = 0;
    mldEepProgramRemaining = 0;
    mdrEnabled = false;
    uartEnabled = false;
    tzxEnabled = false;
    dskEnabled = false;
    modemEnabled = false;
    printerEnabled = false;
    printerStrobe = true;
    lprintEnabled = false;
    romSelected = ROM_ROM0;
    romArraySelected = BANK_ROM0;
    spectrumBorder = 0x00;
    spectrumBankM = 0x00;
    spectrumBank678 = 0x00;
    spectrumAyReg = 0x00;
    stateStartLoad = false;

    // Reset the USB detection state
    mousePresent = false;
    joystickPresent = false;

    // Reset the SD SPI state
    resetSdSpi();

    // Reset the menu and ZXC3 buffer
    menuBuffer.clear();
    zxC3EraseBuffer.clear();

    // Perform specific actions
    switch (globalState)
    {
        case STATE_RESET_MENU :
            handleStateResetMenu();
            break;
        default :
            handleStateResetEntry();
            break;
    }

    // Detect if RTC is set
    if (!rtcHasTime && (year() >= 2026))
    {
        rtcHasTime = true;
    }

    // Start to get time over WiFi, when not already sync'd
    // NOTE: wifiNtpEnabled persists UART across reset
    espUart.begin(0, 0);
    if (wifiNtpPresent && !rtcHasTime && !wifiNtpEnabled)
    {
        wifiNtp.begin(&espUart);
        wifiNtpEnabled = true;
    }

    // Flag that reset has occurred
    afterFirstReset = true;
    menuEnterOnReset = false;

    // Populate the menu when active
    if (IS_ROM_PAGED(ROM_MENU))
    {
        menuBeginMain();
    } else {
        // Enable DSK, MDR and TZX peripherals
        if (dskPresent && beginSdfsSd())
        {
            dskEnabled = true;
            dskController.begin(menuGetFdcFdaPath(), dskEnableDriveB,
                menuGetFdcFdbPath());
        }
        if (mdrPresent && beginSdfsSd() &&
            loadMdrEmulatorFile(menuGetBrowserPath()))
        {
            mdrEnabled = true;
        }
        if (tzxPresent && beginSdfsSd())
        {
            if (tzxPlayer.begin(menuGetTapeFileName(), divMmcExtRamArray[0], 0))
            {
                tzxEnabled = true;
                divMmcExtRamEnabled = false;
                if (!tzxPlayer.isStreamingFile())
                {
                    menuClearTapeFileName();
                }
            } else {
                tzxEnabled = false;
            }
        }

        // Load Genie 128 disassembler into the Multiface 128
        if (mf128Present && mf128LoadGenie)
        {
            loadRomImage(GENIE128_ROM_PATH,
                (char*)&(romArray[ROM_PAGE_MF128][RAM_PAGE_SIZE]),
                RAM_PAGE_SIZE);
        }

        // Enable the DivMMC
        if (divMmcPresent)
        {
            divMmcEnabled = true;
            divMmcRomEnabled = divMmcRomPresent;
            char* sdaPath = menuGetDivMmcSdaPath();
            char* sdbPath = menuGetDivMmcSdbPath();
            if (sdaPath == 0)
            {
                divMmcDriveSlot[0] = DIVMMC_NONE;
            } else if (stricmp("/", sdaPath) != 0)
            {
                divMmcDriveSlot[0] = (divMmcHdf.begin(sdaPath) ?
                    DIVMMC_HDF_A : DIVMMC_NONE);
            } else {
                divMmcDriveSlot[0] = DIVMMC_SDHC;
            }
            if (sdbPath == 0)
            {
                divMmcDriveSlot[1] = DIVMMC_NONE;
            } else if (stricmp("/", sdbPath) != 0)
            {
                divMmcDriveSlot[1] = (divMmcSecondHdf.begin(sdbPath) ?
                    DIVMMC_HDF_B : DIVMMC_NONE);
            } else {
                divMmcDriveSlot[1] = ((divMmcDriveSlot[0] != DIVMMC_SDHC) ?
                    DIVMMC_SDHC : DIVMMC_NONE);
            }
        } else {
            divMmcExtRamEnabled = false;
        }

        // Enable the Interface 1 when DivMMC is not enabled
        if (!divMmcEnabled && interface1Present)
        {
            interface1Enabled = true;
        }

        // If UART is present and ready, then enable
        if (uartPresent)
        {
            // Load the modem ROM, if possible
            if (modemPresent)
            {
                divMmcExtRamEnabled = false;
                if (((romArrayPresent & BANK_RAM) == 0) &&
                    (loadRomImage(MODEM_ROM_PATH, (char*)divMmcExtRamArray[0],
                        RAM_PAGE_SIZE) >= RAM_PAGE_SIZE))
                {
                    romArrayPresent |= BANK_RAM;
                    PAGE_IN_ROM(ROM_MODEM);
                }
            }

            // Wait for WiFi NTP before enabling UART
            if (!wifiNtpEnabled)
            {
                espUart.end();
                if (modemPresent)
                {
                    espUart.begin(0, menuGetModemUrl());
                    modemOnReset = true;
                    modemEnabled = true;
                } else {
                    espUart.begin(0, 0);
                    uartEnabled = true;
                }
            }
        }

        // Indicate if TZX files can be loaded into DivMMC RAM
        tzxPresent = !divMmcExtRamEnabled;

        // Page in the ZXC2 cartridge, or snapshot loader ROM
        if (stateStartLoad)
        {
            PAGE_IN_ROM(ROM_SNA);
            stateStartLoad = false;
        } else if (zxC2Present)
        {
            if (!zxC2ShadowRom)
            {
                PAGE_IN_ROM(ROM_ZXC2);
            }
        } else if (snaLoaderPresent)
        {
            PAGE_IN_ROM(ROM_SNA);
        } else if (mldPresent)
        {
            PAGE_IN_ROM(ROM_MLD);
        }

        // If printer port is present, then enable
        if (printerPresent)
        {
            printerPort.begin();
            if (lprintPresent)
            {
                lprintEnabled = true;
            } else {
                printerEnabled = true;
            }
        }

        // If USB mouse/gamepad is present, then enable
        if (usbPresent)
        {
            // Enable the USB host
            if (!usbEnabled)
            {
                usbHost.begin();
                usbKeyboard.attachPress(usbKeyboardPressed);
                usbKeyboard.attachRelease(usbKeyboardReleased);
                usbEnabled = true;

                // Delay to allow USB devices to initialise
                delay(250);
            }
        }
    }

    // Enable the soft ROM, if present
    delay(250);
    if (romArrayPresent != 0)
    {
        updateRomIndex(true);
        setState(STATE_ROM_ENABLE);
    } else {
        setState(STATE_ROM_DISABLE);
    }
}

FASTRUN void loop()
{
    // Detect reset entry, and perform actions in reset
    if (isGlobalStateReset())
    {
        handleStateReset();
    }

    // Run actions (eg. SD SPI) on regular ticks
    uint32_t cycle_ = ARM_DWT_CYCCNT;
    if ((cycle_ - globalCycleCount) >= TICK_CYCCNT)
    {
        globalCycleCount = cycle_;

        // Perform tick updates at TICK_FREQ
        sdSpiOnTick();
        espUart.onTick();
        tzxPlayer.onTick();
        mldPulseOnTick(cycle_);
        zxC3OnTick();
        printerPort.onTick();
        if (wifiNtpEnabled && wifiNtp.onTick())
        {
            // Update the RTC now time has been received
            wifiNtpEnabled = false;
            rtcTeensy.setAscTime(wifiNtp.getAscTime(), wifiNtpTz);
            rtcHasTime = true;
            menuRedraw = true;
            wifiNtp.end();

            // Enable the UART when not in menu, now time is updated
            if (uartPresent && (IS_ROM_PAGED(ROM_MENU) == 0))
            {
                espUart.end();
                if (modemPresent)
                {
                    espUart.begin(0, menuGetModemUrl());
                    modemOnReset = true;
                    modemEnabled = true;
                } else {
                    espUart.begin(0, 0);
                    uartEnabled = true;
                }
            }
        }

        // Trigger NMI to enter the menu, when requested
        if (menuTriggerNMI && (divMmcDrive == DIVMMC_NONE))
        {
            // Prepare the in-game menu
            menuBeginInGame();

            // Trigger NMI to page in the menu ROM
            nmiRomTarget = ROM_MENU;
            nmiPending = true;
            menuTriggerNMI = false;
            digitalWriteFast(NMI_PIN, 1);
        }

        // Trigger exit from menu with NMI, when requested
        if (menuTriggerExitNMI && IS_ROM_PAGED(ROM_MENU))
        {
            menuSelected = false;
            menuTriggerExitNMI = false;
            nmiRomTarget = (divMmcRomEnabled ? ROM_DIVMMC : ROM_MF128);
            menuBuffer.write(MENU_ROM_CMD_IN_GAME_EXIT);
        }

        // Perform menu actions
        if (menuSelected)
        {
            menuSelected = false;
            if (menuPerformSelection(menuSelectedIndex))
            {
                bool exitMenu = true;
                switch (menuGetMenuAction())
                {
                    case MENU_ACTION_IN_GAME_EXIT_BASIC :
                        menuInGameExitBasic();
                        nmiRomTarget = ROM_ROM0;
                        break;
                    case MENU_ACTION_IN_GAME_EXIT_TAPE :
                        if (tzxEnabled)
                        {
                            if (tzxPlayer.isTapePaused())
                            {
                                tzxPlayer.unpause();
                            } else {
                                tzxPlayer.play();
                            }
                        }
                        nmiRomTarget = ROM_ROM0;
                        break;
                    case MENU_ACTION_IN_GAME_SAVE_STATE :
                        exitMenu = false;
                        if (!stateBeginSave(stateSaveSlot))
                        {
                            menuRedraw = true;
                            menuBuffer.write(MENU_ROM_CMD_STATE_FAILED);
                        }
                        break;
                    case MENU_ACTION_IN_GAME_APPLY_POK :
                        exitMenu = false;
                        stateActiveSlot = -1;
                        if (!stateBeginSave(STATE_POKE_SLOT))
                        {
                            pokeFinishApply();
                            menuRedraw = true;
                            menuBuffer.write(MENU_ROM_CMD_STATE_FAILED);
                        }
                        break;
                    case MENU_ACTION_IN_GAME_EXIT :
                        nmiRomTarget = ROM_ROM0;
                        break;
                    case MENU_ACTION_IN_GAME_NMI :
                        nmiRomTarget = ROM_ROM3;
                        break;
                    case MENU_ACTION_IN_GAME_MF128 :
                        nmiRomTarget = ROM_MF128;
                        break;
                    case MENU_ACTION_IN_GAME_DIVMMC :
                        nmiRomTarget = ROM_DIVMMC;
                        break;
                    case MENU_ACTION_IN_GAME_RESET :
                        // Hard reset into the main menu
                        stateActiveSlot = -1;
                        performHardReset();
                        break;
                    case MENU_ACTION_LOAD_STATE_SLOT :
                        // Reset to load the active state slot
                        setState(STATE_RESET_MENU);
                        break;
                    default :
                        // The menu needs the Spectrum in reset to access the SD card,
                        // reload ROMs, update FW etc.
                        stateActiveSlot = -1;
                        setState(STATE_RESET_MENU);
                        break;
                }

                // Update ROM indexes, and exit the menu
                if (exitMenu && !isGlobalStateReset())
                {
                    updateRomIndex(true);
                    menuBuffer.write(MENU_ROM_CMD_IN_GAME_EXIT);
                }
            } else {
                // Refresh the menu, by default
                menuRedraw = true;

                // Perform in-game actions
                switch (menuGetMenuAction())
                {
                    case MENU_ACTION_SELECT_LOAD_SLOT :
                        // The menu ROM is waiting for the preview command
                        menuGenerate();
                        menuBuffer.write(MENU_ROM_CMD_STATE_PREVIEW);
                        menuRedraw = false;
                        break;
                    case MENU_ACTION_BROWSER_EXPAND :
                        // The menu ROM is waiting for the expanded-name command
                        menuGenerate();
                        menuBuffer.write(MENU_ROM_CMD_BROWSER_EXPAND);
                        menuRedraw = false;
                        break;
                    case MENU_ACTION_BROWSER_LOAD_TZX :
                        if (!divMmcExtRamEnabled && beginSdfsSd())
                        {
                            tzxEnabled = tzxPlayer.begin(menuGetTapeFileName(),
                                divMmcExtRamArray[0], 0);
                            if (tzxEnabled && !tzxPlayer.isStreamingFile())
                            {
                                menuClearTapeFileName();
                            }
                        }
                        break;
                    case MENU_ACTION_IN_GAME_EJECT_TAPE :
                        tzxPlayer.end();
                        menuClearTapeFileName();
                        tzxEnabled = false;
                        break;
                    case MENU_ACTION_BROWSER_LOAD_MF128 :
                        if (mf128Present && beginSdfsSd())
                        {
                            memset((void*)&(romArray[ROM_PAGE_MF128][RAM_PAGE_SIZE]),
                                0xFF, RAM_PAGE_SIZE);
                            loadRomImage(menuGetBrowserPath(),
                                (char*)&(romArray[ROM_PAGE_MF128][RAM_PAGE_SIZE]),
                                RAM_PAGE_SIZE);
                        }
                        break;
                    case MENU_ACTION_BROWSER_MOUNT_FDA :
                    case MENU_ACTION_BROWSER_MOUNT_FDB :
                    case MENU_ACTION_IN_GAME_UNMOUNT_FDA :
                    case MENU_ACTION_IN_GAME_UNMOUNT_FDB :
                        if (dskEnabled && !dskController.isMotorOn() && beginSdfsSd())
                        {
                            dskController.insertDisks(menuGetFdcFdaPath(),
                                menuGetFdcFdbPath());
                        }
                        break;
                    default :
                        break;
                }
            }
        }

        // Re-generate and re-draw the menu, when requested
        if (menuRedraw)
        {
            menuRedraw = false;
            if (IS_ROM_PAGED(ROM_MENU) && !nmiPending)
            {
                menuGenerate();
                menuBuffer.write(MENU_ROM_CMD_REDRAW);
            }
        }

        // Debounce the reset detection
        switch (resetTrigState)
        {
            case TRIGGER_HOLD :
                if (digitalReadFast(RESET_IN_PIN))
                {
                    resetTrigState = TRIGGER_DELAY;
                    resetTrigExitCount = TRIGGER_DELAY_CNT;
                } else {
                    --resetHardTrigCount;
                    if (resetHardTrigCount == 0)
                    {
                        performHardReset();
                        return;
                    }
                }
                break;
            case TRIGGER_DELAY :
                if (digitalReadFast(RESET_IN_PIN))
                {
                    --resetTrigExitCount;
                    if (resetTrigExitCount == 0)
                    {
                        resetTrigState = TRIGGER_READY;
                    }
                } else {
                    resetTrigState = TRIGGER_HOLD;
                    resetHardTrigCount = HARD_RESET_DELAY_CNT;
                }
                break;
            default :
                break;
        }

        // Debounce the button detection
        switch (buttonTrigState)
        {
            case TRIGGER_ACTIVE :
                if (!nmiPending && digitalReadFast(BUTTON_PIN))
                {
                    buttonTrigState = TRIGGER_HOLD;
                }
                break;
            case TRIGGER_HOLD :
                if (digitalReadFast(BUTTON_PIN))
                {
                    buttonTrigState = TRIGGER_DELAY;
                    buttonTrigExitCount = BUTTON_DELAY_CNT;
                }
                break;
            case TRIGGER_DELAY :
                if (digitalReadFast(BUTTON_PIN))
                {
                    --buttonTrigExitCount;
                    if (buttonTrigExitCount == 0)
                    {
                        buttonTrigState = TRIGGER_READY;
                    }
                } else {
                    buttonTrigExitCount = BUTTON_DELAY_CNT;
                }
                break;
            default :
                break;
        }
    }

    // Run HTTP server actions
    httpOnTick();

    // Run FDC actions
    dskController.onTick();

    // Run save/restore state actions
    stateOnTick();

    // Perform USB host functions
    if (usbEnabled)
    {
        usbHost.Task();
        if (usbMouse.available())
        {
            mouseX += (uint32_t)usbMouse.getMouseX();
            mouseY -= (uint32_t)usbMouse.getMouseY();
            mouseBtn = ~(usbMouse.getButtons());
            mousePresent = true;
            usbMouse.mouseDataClear();
        }
        if (usbJoystick.available())
        {
            uint8_t data = 0;
            uint32_t dpadButtons;
            uint32_t buttons = usbJoystick.getButtons();
            switch (usbJoystick.joystickType())
            {
                case JoystickController::PS3 :
                    // D-Pad
                    dpadButtons = (buttons & 0x00F0) >> 4;
                    if (dpadButtons != 0)
                    {
                        const static uint8_t dpadMap[16] = { 0x00,
                            0x08, 0x01, 0x09, 0x04, 0x0C, 0x05, 0x0D,
                            0x02, 0x0A, 0x03, 0x0B, 0x06, 0x0E, 0x07, 0x0F };
                        data |= dpadMap[dpadButtons];
                    }
                    if (gamepadButtons)
                    {
                        // NOTE: Button mapping is reversed
                        if (buttons & 0x8000)
                        {
                            data |= 0x10;
                        }
                        if (buttons & 0x4000)
                        {
                            data |= 0x20;
                        }
                        if (buttons & 0x2000)
                        {
                            data |= 0x40;
                        }
                        if (buttons & 0x1000)
                        {
                            data |= 0x80;
                        }
                    } else if (buttons & 0xF000)
                    {
                        data |= 0x10;
                    }
                    break;
                case JoystickController::PS4 :
                    // D-Pad
                    dpadButtons = (uint8_t)usbJoystick.getAxis(9);
                    if (dpadButtons < 0x08)
                    {
                        // PS4 D-Pad returns as compass points on Axis 9 eg.
                        // North, NE, East etc.
                        const static uint8_t dpadMap[8] = { 0x08, 0x09, 0x01,
                            0x05, 0x04, 0x06, 0x02, 0x0A };
                        data |= dpadMap[dpadButtons];
                    }
                    if (gamepadButtons)
                    {
                        data |= (buttons & 0x0F) << 4;
                    } else if (buttons & 0x0F)
                    {
                        data |= 0x10;
                    }
                    break;
                default :
                    // D-Pad
                    dpadButtons = (buttons & 0xF00) >> 8;
                    if (dpadButtons != 0)
                    {
                        const static uint8_t dpadMap[16] = { 0x00,
                            0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E,
                            0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F };
                        data |= dpadMap[dpadButtons];
                    }
                    if (gamepadButtons)
                    {
                        data |= (buttons & 0xF0);
                    } else if (buttons & 0xF0)
                    {
                        data |= 0x10;
                    }
                    break;
            }
#ifdef ENABLE_JOYSTICK_DEBUG
            if (menuIsDebugging() && (joystickData != data))
            {
                menuRedraw = menuPrintDebug(true, F_CSTR("joystickData %0d"), data);
            }
#endif
            joystickData = data;
            joystickPresent = true;
            usbJoystick.joystickDataClear();
        }
    }
}

void usbKeyboardPressed(int key)
{
    uint8_t data = joystickData;
    switch (key)
    {
        case 'q' :
        case KEYD_UP :
            data |= 0x08;
            break;
        case 'a' :
        case KEYD_DOWN :
            data |= 0x04;
            break;
        case 'o' :
        case KEYD_LEFT :
            data |= 0x02;
            break;
        case 'p' :
        case KEYD_RIGHT :
            data |= 0x01;
            break;
        case ' ' :
        case 'm' :
        case '\n' :
            data |= 0x10;
            break;
        case 'n' :
            data |= 0x20;
            break;
    }
    joystickData = data;
    joystickPresent = true;
}

void usbKeyboardReleased(int key)
{
    uint8_t data = joystickData;
    switch (key)
    {
        case 'q' :
        case KEYD_UP :
            data &= ~(0x08);
            break;
        case 'a' :
        case KEYD_DOWN :
            data &= ~(0x04);
            break;
        case 'o' :
        case KEYD_LEFT :
            data &= ~(0x02);
            break;
        case 'p' :
        case KEYD_RIGHT :
            data &= ~(0x01);
            break;
        case ' ' :
        case 'm' :
        case '\n' :
            data &= ~(0x10);
            break;
        case 'n' :
            data &= ~(0x20);
            break;
    }
    joystickData = data;
    joystickPresent = true;
}

inline void mldClearCommand()
{
    mldPulseState = MLD_PULSE_IDLE;
    mldCmdOpcode = 0;
    mldCmdRepeat = 0;
}

inline void mldClearEepProgram()
{
    mldEepProgram = MLD_EEP_IDLE;
    mldEepSector = 0;
    mldEepProgramRemaining = 0;
    zxC3FlashState = ZXC3_FLASH_IDLE;
    zxC3FlashSetup = false;
    zxC3Write = false;
}

void mldPulseOnTick(uint32_t cycle)
{
    if (mldPresent && (mldPulseState != MLD_PULSE_IDLE))
    {
        if (mldCmdDisabled || (mldEepProgram != MLD_EEP_IDLE))
        {
            mldClearCommand();
        } else {
            uint32_t timeout;
            if ((mldPulseState == MLD_PULSE_FAST) ||
                (((mldPulseState == MLD_PULSE_DATA1) ||
                    (mldPulseState == MLD_PULSE_DATA2)) && (mldCmdRepeat == 0)))
            {
                timeout = MLD_SPECIAL_TIMEOUT_CNT;
            } else {
                timeout = MLD_PULSE_TIMEOUT_CNT;
            }
            if ((cycle - mldPulseCycle) >= timeout)
            {
                switch (mldPulseState)
                {
                    case MLD_PULSE_COMMAND :
                        if ((mldCmdRepeat >= 40) && (mldCmdRepeat <= 49))
                        {
                            if (!mldCmdLocked || (mldCmdRepeat == 46))
                            {
                                mldCmdOpcode = mldCmdRepeat;
                                mldCmdRepeat = 0;
                                mldPulseState = MLD_PULSE_DATA1;
                                mldPulseCycle = cycle;
                            } else {
                                mldClearCommand();
                            }
                        } else {
                            uint8_t command = mldCmdRepeat;
                            mldClearCommand();
                            if ((command != 0) && !mldCmdLocked)
                            {
                                mldRunCommand(command);
                                updateRomIndex(true);
                            }
                        }
                        break;
                    case MLD_PULSE_DATA1 :
                        mldCmdData1 = mldCmdRepeat;
                        mldCmdRepeat = 0;
                        mldPulseState = MLD_PULSE_DATA2;
                        mldPulseCycle = cycle;
                        break;
                    case MLD_PULSE_DATA2 :
                        mldCmdData2 = mldCmdRepeat;
                        mldCmdRepeat = 0;
                        mldPulseState = MLD_PULSE_FAST;
                        mldPulseCycle = cycle;
                        break;
                    default :
                        mldClearCommand();
                        break;
                }
            }
        }
    }
}

void mldRunCommand(uint8_t command)
{
    if (!mldCmdLocked || (command == 46))
    {
        if ((command >= 1) && (command <= 32))
        {
            // Bank select command
            updateMldSlotPtr(command);
        } else {
            switch (command)
            {
                case 33 :
                    // Page out command
                    updateMldSlotPtr(33);
                    break;
                case 34 :
                    // Page out and lock command
                    updateMldSlotPtr(33);
                    mldCmdLocked = true;
                    break;
                case 36 :
                    // Reset spectrum command
                    mldClearEepProgram();
                    setState(STATE_RESET);
                    break;
                case 37 :
                    // Trigger NMI command
                    if (!nmiPending)
                    {
                        nmiPending = true;
                        digitalWriteFast(NMI_PIN, 1);
                    }
                    break;
                case 39 :
                    break;
                case 40 :
                    // "Fast" command
                    updateMldSlotPtr(mldCmdData1);
                    if ((mldCmdData2 & 0x08) != 0)
                    {
                        mldCmdDisabled = true;
                        mldCmdLocked = false;
                        mldClearEepProgram();
                    } else if ((mldCmdData2 & 0x04) != 0)
                    {
                        mldCmdLocked = true;
                    } else {
                        mldCmdLocked = false;
                    }
                    if ((mldCmdData2 & 0x02) != 0)
                    {
                        if (!nmiPending)
                        {
                            nmiPending = true;
                            digitalWriteFast(NMI_PIN, 1);
                        }
                    }
                    if ((mldCmdData2 & 0x01) != 0)
                    {
                        mldClearEepProgram();
                        setState(STATE_RESET);
                    }
                    break;
                case 46 :
                    // Dandanator lock command
                    if (mldCmdData1 == mldCmdData2)
                    {
                        if (mldCmdData1 == 1)
                        {
                            mldCmdLocked = true;
                        } else if (mldCmdData1 == 16)
                        {
                            mldCmdLocked = false;
                            mldCmdDisabled = false;
                            mldClearEepProgram();
                        } else if (mldCmdData1 == 31)
                        {
                            mldCmdDisabled = true;
                            mldCmdLocked = false;
                            mldClearEepProgram();
                        }
                    }
                    break;
                case 48 :
                {
                    // Flash program command
                    if (mldCmdData2 < MLD_SECTOR_COUNT)
                    {
                        mldEepProgram = MLD_EEP_IDLE;
                        if (mldCmdData1 == 1)
                        {
                            mldEepProgram = MLD_EEP_PROGRAM_BYTE;
                            mldEepProgramRemaining = 1;
                        } else if (mldCmdData1 == 16)
                        {
                            mldEepProgram = MLD_EEP_ERASE_SECTOR;
                            mldEepProgramRemaining = 0;
                        } else if (mldCmdData1 == 32)
                        {
                            mldEepProgram = MLD_EEP_PROGRAM_SECTOR;
                            mldEepProgramRemaining = 4096;
                        }
                        if (mldEepProgram != MLD_EEP_IDLE)
                        {
                            zxC3Write = true;
                            mldEepSector = mldCmdData2;
                            updateMldSlotPtr((mldEepSector >> 2) + 1);
                        } else {
                            mldClearEepProgram();
                        }
                    } else {
                        mldClearEepProgram();
                    }
                    break;
                }
                default :
                    break;
            }
        }
    }
}

void updateMldSlotPtr(uint8_t slot)
{
    if (slot == 35)
    {
        slot = mldPreviousSlot;
    }
    if ((slot >= 1) && (slot <= MLD_MAX_SLOT_COUNT))
    {
        uint8_t page = (slot - 1) << 1;
        if (page < MLD_MAX_PAGE_COUNT)
        {
            if (slot != mldCurrentSlot)
            {
                mldPreviousSlot = mldCurrentSlot;
                mldCurrentSlot = slot;
            }
            zxC2RomBank = page;
            PAGE_IN_ROM(ROM_MLD);
        }
    } else if (slot == 33)
    {
        if (mldCurrentSlot != 33)
        {
            mldPreviousSlot = mldCurrentSlot;
            mldCurrentSlot = 33;
        }
        PAGE_OUT_ROM(ROM_MLD);
    }
    updateRomIndex(true);
}

inline void updateRomPtr(bool pageNow)
{
    // Enable soft ROM when page is present
    if ((romArrayPresent & romArraySelected) != 0)
    {
        switch (romSelected)
        {
            case ROM_DIVMMC :
                if (divMmcMapRam && !divMmcConMem)
                {
                    romPtr = divMmcRamArray[3];
                } else {
                    //romPtr = romArray[ROM_PAGE_DIVMMC + (romSelected - ROM_DIVMMC)];
                    romPtr = romArray[ROM_PAGE_DIVMMC];
                }
                break;
            case ROM_LPRINT :
                // Special handling for 2KB ROM
                romPtr = lprintRom;
                break;
            case ROM_MODEM :
            case ROM_ZXC2 :
                romPtr = divMmcExtRamArray[zxC2RomBank];
                break;
            case ROM_MLD :
                if (zxC2RomBank >= RAM_PAGE_COUNT)
                {
                    romPtr = divMmcExtRamArray[(zxC2RomBank - RAM_PAGE_COUNT)];
                } else {
                    romPtr = divMmcRamArray[zxC2RomBank];
                }
                break;
            case ROM_SNA :
                if (snaLoaderBanks > 0)
                {
                    romPtr = divMmcExtRamArray[zxC2RomBank];
                } else {
                    romPtr = menuRamArray[2];
                }
                break;
            case ROM_MENU :
                romPtr = romArray[ROM_PAGE_MENU];
                break;
            default :
                // 16KB ROMs are two ROM pages
                romPtr = romArray[(romSelected * 2)];
                break;
        }
        if (!romEnabled)
        {
            // Page in the soft ROM
            if (pageNow)
            {
                disableInternalRom();
            } else {
                romCsEnable = true;
            }
        }
    } else if (romEnabled)
    {
        // Fall back to internal ROM
        if (pageNow)
        {
            enableInternalRom();
        } else {
            romCsDisable = true;
        }
    }
    if (pageNow)
    {
        divMmcUpdateInterfaceOne();
    }
}

inline void updateRomIndex(bool pageNow)
{
    // Determine which ROM is currently paged
    if (IS_ROM_PRIORITY(ROM_DIVMMC))
    {
        if (IS_ROM_PRIORITY(ROM_ZXC2))
        {
            if (IS_ROM_PRIORITY(ROM_SNA))
            {
                if (IS_ROM_PRIORITY(ROM_MENU))
                {
                    romArraySelected = BANK_MENU;
                    romSelected = ROM_MENU;
                } else {
                    romArraySelected = BANK_RAM;
                    romSelected = ROM_SNA;
                }
            } else {
                romArraySelected = BANK_RAM;
                if (IS_ROM_PRIORITY(ROM_MLD))
                {
                    romSelected = ROM_MLD;
                } else {
                    romSelected = ROM_ZXC2;
                }
            }
        } else {
            if (IS_ROM_PRIORITY(ROM_LPRINT))
            {
                if (IS_ROM_PRIORITY(ROM_MODEM))
                {
                    romArraySelected = BANK_RAM;
                    romSelected = ROM_MODEM;
                } else {
                    romArraySelected = BANK_LPRINT;
                    romSelected = ROM_LPRINT;
                }
            } else {
                romArraySelected = BANK_DIVMMC;
                romSelected = ROM_DIVMMC;
            }
        }
    } else {
        if (IS_ROM_PRIORITY(ROM_ROM3))
        {
            if (IS_ROM_PRIORITY(ROM_IF1))
            {
                if (IS_ROM_PRIORITY(ROM_MF128))
                {
                    romArraySelected = BANK_MF128;
                    romSelected = ROM_MF128;
                } else {
                    romArraySelected = BANK_IF1;
                    romSelected = ROM_IF1;
                }
            } else {
                romArraySelected = BANK_ROM3;
                romSelected = ROM_ROM3;
            }
        } else {
            if (IS_ROM_PRIORITY(ROM_ROM1))
            {
                if (IS_ROM_PRIORITY(ROM_ROM2))
                {
                    romArraySelected = BANK_ROM2;
                    romSelected = ROM_ROM2;
                } else {
                    romArraySelected = BANK_ROM1;
                    romSelected = ROM_ROM1;
                }
            } else {
                romArraySelected = BANK_ROM0;
                romSelected = ROM_ROM0;
            }
        }
    }

    // Enable soft ROM when page is present
    updateRomPtr(pageNow);
}

inline void writePagedRomData(uint16_t address)
{
    if (romEnabled)
    {
        // Transfer soft ROM data to the bus
        writeData(romPtr[address]);
    }
}

inline void writeZXC2RomData(uint16_t address)
{
    if (romEnabled)
    {
        // Transfer soft ROM data to the bus
        writeData(zxC3EraseBusy ? 0x08 : romPtr[address]);
    }
}

inline void writeDivMmcRomData(uint16_t address)
{
    if (address >= RAM_PAGE_SIZE)
    {
        // Tranfer DivMMC RAM data to the bus
        writeData(divMmcRamPtr[address & (RAM_PAGE_SIZE - 1)]);
    } else {
        // Transfer soft ROM data to the bus
        writeData(romPtr[address]);
    }
}

inline void writeLprintRomData(uint16_t address)
{
    writeData(romPtr[address & (LPRINT_ROM_SIZE - 1)]);
}

inline void writeMenuRomData(uint16_t address)
{
    if (address >= RAM_PAGE_SIZE)
    {
        // Tranfer menu RAM data to the bus
        writeData(menuRamPtr[address & (RAM_PAGE_SIZE - 1)]);
    } else {
        // Transfer soft ROM data to the bus
        writeData(romPtr[address]);
    }
}

inline void writeRomData(uint16_t address)
{
    switch (romSelected)
    {
        case ROM_DIVMMC :
            writeDivMmcRomData(address);
            break;
        case ROM_LPRINT :
            writeLprintRomData(address);
            break;
        case ROM_ZXC2 :
        case ROM_MLD :
            writeZXC2RomData(address);
            break;
        case ROM_MENU :
            writeMenuRomData(address);
            break;
        default :
            writePagedRomData(address);
            break;
    }
}

FASTRUN void isrFastGpios()
{
    uint32_t status = GPIO6_ISR & GPIO6_IMR;
    if (status)
    {
        GPIO6_ISR = status;
        if (status & WR_PIN_BITMASK)
        {
            isrWrEvent();
        } else {
            isrRdEvent();
        }

        // Re-sync regular ticks after ISR
        globalCycleCount = (ARM_DWT_CYCCNT - TICK_CYCCNT);
    }
    status = GPIO9_ISR & GPIO9_IMR;
    if (status)
    {
        GPIO9_ISR = status;
        if (status & ROMCS_IN_PIN_BITMASK)
        {
            isrRdEvent();
        }
        if (status & CORE_PIN33_BITMASK)
        {
            isrPinButton();
        }
        if (status & CORE_PIN2_BITMASK)
        {
            isrPinReset();
        }
    }
    asm volatile ("dsb":::"memory");
}

FASTRUN void isrPinReset()
{
    // Perform entry to reset when not debouncing reset
    if ((resetTrigState == TRIGGER_READY) && !digitalReadFast(RESET_IN_PIN))
    {
        setState(STATE_RESET);
    }
}

FASTRUN void isrPinButton()
{
    // Perform NMI when not debouncing button
    if ((buttonTrigState == TRIGGER_READY) && !digitalReadFast(BUTTON_PIN))
    {
        buttonTrigState = TRIGGER_ACTIVE;

        // Pause the tape
        bool wasTapePlaying = (tzxEnabled && tzxPlayer.isTapePlaying());
        if (wasTapePlaying)
        {
            tzxPlayer.pause();
        }

        // Perform NMI when not already handling previous NMI
        if (!isGlobalStateReset() && !nmiPending &&
            !menuTriggerNMI && !menuTriggerExitNMI)
        {
            if (IS_ROM_PAGED(ROM_MENU))
            {
                if (menuIsInGameMenu())
                {
                    menuTriggerExitNMI = true;
                }
            } else if (menuEnableInGame)
            {
                menuTriggerNMI = true;
            } else if (!wasTapePlaying)
            {
                nmiPending = true;
                digitalWriteFast(NMI_PIN, 1);
            }
        }
    }
}

FASTRUN void isrWrEvent()
{
    // Start of write access
    uint32_t gpioSix = (*(volatile uint32_t *)IMXRT_GPIO6_ADDRESS);
    if ((gpioSix & ROM_ADDRESS_MASK) == 0x00000000)
    {
        uint16_t address = decodeAddress(gpioSix);
        uint8_t data = readData();

        // Perform MLD, or ZXC2 address based paging
        if (mldPresent)
        {
            // Handle Dandanator command modes, when not programming
            if ((mldEepProgram == MLD_EEP_IDLE) && !mldCmdDisabled &&
                (zxC3FlashState != ZXC3_FLASH_WRITE))
            {
                bool hasCommand = false;
                switch (address)
                {
                    case 0x0001 :
                        mldPulseState = MLD_PULSE_IDLE;
                        if (data < 40)
                        {
                            if ((data != 0) && !mldCmdLocked)
                            {
                                if (mldCmdOpcode != data)
                                {
                                    mldCmdOpcode = data;
                                    mldCmdRepeat = 1;
                                } else if (mldCmdRepeat < 0xFF)
                                {
                                    ++mldCmdRepeat;
                                }
                                if (mldCmdRepeat == data)
                                {
                                    hasCommand = true;
                                }
                            } else {
                                mldCmdOpcode = 0;
                                mldCmdRepeat = 0;
                            }
                        } else {
                            if (!mldCmdLocked || (data == 46))
                            {
                                mldCmdOpcode = data;
                            } else {
                                mldCmdOpcode = 0;
                            }
                            mldCmdRepeat = 0;
                        }
                        break;
                    case 0x0002 :
                        mldPulseState = MLD_PULSE_IDLE;
                        mldCmdData1 = data;
                        break;
                    case 0x0003 :
                        mldPulseState = MLD_PULSE_IDLE;
                        mldCmdData2 = data;
                        break;
                    default :
                        if ((mldCmdOpcode >= 40) &&
                            (address == 0x0000) &&
                            ((mldPulseState == MLD_PULSE_IDLE) ||
                                (mldPulseState == MLD_PULSE_FAST)))
                        {
                            hasCommand = true;
                        } else {
                            switch (mldPulseState)
                            {
                                case MLD_PULSE_IDLE :
                                    mldCmdOpcode = 0;
                                    mldCmdRepeat = 1;
                                    mldPulseState = MLD_PULSE_COMMAND;
                                    mldPulseCycle = ARM_DWT_CYCCNT;
                                    break;
                                case MLD_PULSE_COMMAND :
                                case MLD_PULSE_DATA1 :
                                case MLD_PULSE_DATA2 :
                                    if (mldCmdRepeat < 0xFF)
                                    {
                                        ++mldCmdRepeat;
                                    }
                                    mldPulseCycle = ARM_DWT_CYCCNT;
                                    break;
                                default :
                                    break;
                            }
                        }
                        break;
                }
                if (hasCommand)
                {
                    mldRunCommand(mldCmdOpcode);
                    mldClearCommand();
                    return;
                }
            }
        } else if (zxC2Present && !zxC2Lock && !nmiPending &&
            (!zxC2ShadowRom || IS_ROM_PAGED(ROM_ZXC2)) &&
            IS_ROM_HIGHEST(ROM_ZXC2) &&
            ((address & 0xffc0) == 0x3fc0))
        {
            if (zxC3Present)
            {
                zxC3Write = ((address & 0x08) != 0);
                zxC2RomBank = ((address & 0x07) << 1);
            } else {
                zxC2RomBank = ((address & 0x0f) << 1);
            }
            if ((address & 0x10) == 0)
            {
                PAGE_IN_ROM(ROM_ZXC2);
            } else {
                PAGE_OUT_ROM(ROM_ZXC2);
            }
            zxC2Lock = ((address & 0x20) != 0);
            updateRomIndex(true);
        }

        switch (romSelected)
        {
            case ROM_MF128 :
                // Perform Multiface 128 RAM write
                if (address >= RAM_PAGE_SIZE)
                {
                    romPtr[address] = data;
                }
                break;
            case ROM_DIVMMC :
                // Perform DivMMC RAM write
                if ((address >= RAM_PAGE_SIZE) &&
                    (!divMmcMapRam || divMmcConMem || !divMmcRamBankThree))
                {
                    divMmcRamPtr[address & (RAM_PAGE_SIZE - 1)] = data;
                }
                break;
            case ROM_ZXC2 :
            case ROM_MLD :
                if (zxC3Write)
                {
                    bool mldEepActive = (mldEepProgram != MLD_EEP_IDLE);
                    switch (zxC3FlashState)
                    {
                        case ZXC3_FLASH_UNLOCK :
                            if (((zxC2RomBank == 0) || mldEepActive) &&
                                (address == 0x2AAA) && (data == 0x55))
                            {
                                zxC3FlashState = ZXC3_FLASH_CMD;
                            } else if (mldEepActive)
                            {
                                mldClearEepProgram();
                            }
                            break;
                        case ZXC3_FLASH_CMD :
                            zxC3FlashState = ZXC3_FLASH_IDLE;
                            switch (data)
                            {
                                case 0x10 :
                                    // Chip erase
                                    if (!mldEepActive && (zxC2RomBank == 2) &&
                                        (address == 0x1555) && zxC3FlashSetup)
                                    {
                                        zxC3EraseBuffer.write(0xFF);
                                        zxC3EraseBusy = true;
                                        zxC3WriteTrigState = TRIGGER_ACTIVE;
                                    }
                                    zxC3FlashSetup = false;
                                    if (mldEepActive)
                                    {
                                        mldClearEepProgram();
                                    }
                                    break;
                                case 0x30 :
                                    // Sector erase
                                    if (zxC3FlashSetup &&
                                        (!mldEepActive ||
                                            (mldEepProgram == MLD_EEP_ERASE_SECTOR)))
                                    {
                                        // Dandandator 512KB flash uses 4KB sectors,
                                        // ZXC3 128KB flash uses 16KB sectors
                                        zxC3EraseBuffer.write(mldPresent ?
                                            ((zxC2RomBank << 1) | (address >> 12)) :
                                            zxC2RomBank);
                                        zxC3EraseBusy = true;
                                        zxC3WriteTrigState = TRIGGER_ACTIVE;
                                        if (mldEepActive)
                                        {
                                            mldClearEepProgram();
                                        }
                                    } else if (mldEepActive)
                                    {
                                        mldClearEepProgram();
                                    }
                                    zxC3FlashSetup = false;
                                    break;
                                case 0x80 :
                                    // Setup command
                                    if (((zxC2RomBank == 2) || mldEepActive) &&
                                        (address == 0x1555) &&
                                        (!mldEepActive ||
                                            (mldEepProgram == MLD_EEP_ERASE_SECTOR)))
                                    {
                                        zxC3FlashSetup = true;
                                    } else if (mldEepActive)
                                    {
                                        mldClearEepProgram();
                                    }
                                    break;
                                case 0xA0 :
                                    // Program byte
                                    if (((zxC2RomBank == 2) || mldEepActive) &&
                                        (address == 0x1555) &&
                                        (!mldEepActive ||
                                            (mldEepProgram == MLD_EEP_PROGRAM_BYTE) ||
                                            (mldEepProgram == MLD_EEP_PROGRAM_SECTOR)))
                                    {
                                        zxC3FlashState = ZXC3_FLASH_WRITE;
                                    } else if (mldEepActive)
                                    {
                                        mldClearEepProgram();
                                    }
                                    break;
                                default :
                                    zxC3FlashSetup = false;
                                    if (mldEepActive)
                                    {
                                        mldClearEepProgram();
                                    }
                                    break;
                            }
                            break;
                        case ZXC3_FLASH_WRITE :
                            // Program byte
                            romPtr[address] = data;
                            zxC3FlashState = ZXC3_FLASH_IDLE;
                            zxC3WriteTrigState = TRIGGER_ACTIVE;
                            if (mldEepActive)
                            {
                                if ((mldEepProgram == MLD_EEP_PROGRAM_SECTOR) &&
                                    (mldEepProgramRemaining > 1))
                                {
                                    --mldEepProgramRemaining;
                                } else {
                                    mldClearEepProgram();
                                }
                            }
                            break;
                        default :
                            if (((zxC2RomBank == 2) || mldEepActive) &&
                                (address == 0x1555) && (data == 0xAA))
                            {
                                zxC3FlashState = ZXC3_FLASH_UNLOCK;
                            } else if (mldEepActive)
                            {
                                mldClearEepProgram();
                            }
                            break;
                    }
                }
                break;
            case ROM_SNA :
                // Detect snapshot loader paging
                if (address == 0x3FFF)
                {
                    if (snaLoaderBanks > 0)
                    {
                        zxC2RomBank += 2;
                        if (zxC2RomBank >= snaLoaderBanks)
                        {
                            zxC2RomBank = 0;
                            snaLoaderBanks = 0;
                            stateLoaderFinished(false);
                        }
                    } else {
                        PAGE_OUT_ROM(ROM_SNA);
                        stateLoaderFinished(true);
                        snaLoaderPresent = false;
                    }
                    updateRomIndex(true);
                }
                break;
            case ROM_MENU :
                // Perform menu RAM write
                if (address >= RAM_PAGE_SIZE)
                {
                    menuRamPtr[address & (RAM_PAGE_SIZE - 1)] = data;
                }
                break;
            default :
                break;
        }
    } else if ((gpioSix & IOREQ_PIN_BITMASK) == 0x00000000)
    {
        // Perform I/O write access
        uint8_t port_ = decodeLowAddress(gpioSix);
        if ((port_ & 0x02) == 0)
        {
            // Perform I/O 0x1ffd, 0x3ffd or 0x7ffd write access
            if ((gpioSix & A15_PIN_BITMASK) == 0x00000000)
            {
                bool isPort7F = ((gpioSix & A14_PIN_BITMASK) != 0x0);
                uint8_t data = readData();
                if (!rom1Present)
                {
                    // Record 0x7ffd write access for Internal ROMs
                    if (!IS_ROM_PAGED(ROM_MENU))
                    {
                        spectrumBankM = data;
                    }
                } else if (!rom23Present || isPort7F)
                {
                    // Detect 0x7ffd write access for 128k ROMs
                    if (!IS_ROM_PAGED(ROM_MENU))
                    {
                        spectrumBankM = data;
                    }
                    if ((data & 0x20) != 0)
                    {
                        rom1Present = false;
                    }
                    rom1Paged = ((data & 0x10) != 0);
                    romPaged &= ~(0x0F);
                    if (rom23Paged)
                    {
                        romPaged |= (rom1Paged ? 0x08 : 0x04);
                    } else {
                        romPaged |= (rom1Paged ? 0x02 : 0x01);
                    }
                    updateRomIndex(true);
                }
                if (isPort7F)
                {
                    if (divMmcEnabled && IS_ROM_PAGED(ROM_DIVMMC) &&
                        ((data & 0x10) == 0x0))
                    {
                        // Detect 0x7ffd write access to disable DivMMC
                        divMmcToggle = true;
                    }
                } else if ((gpioSix & A13_PIN_BITMASK) != 0x0)
                {
                    if (dskEnabled && ((gpioSix & A12_PIN_BITMASK) != 0x0))
                    {
                        // Detect 0x3ffd write access for disk
                        dskController.writeData(Dsk765ZXTeensy::WRITE_DATA,
                            data);
                    }
                } else if ((gpioSix & A12_PIN_BITMASK) != 0x0)
                {
                    // Detect 0x1ffd write access for +3 ROMs
                    if (!IS_ROM_PAGED(ROM_MENU))
                    {
                        spectrumBank678 = data;
                    }
                    if (rom1Present && rom23Present)
                    {
                        rom23Paged = ((data & 0x04) != 0);
                        romPaged &= ~(0x0F);
                        if (rom23Paged)
                        {
                            romPaged |= (rom1Paged ? 0x08 : 0x04);
                        } else {
                            romPaged |= (rom1Paged ? 0x02 : 0x01);
                        }
                        updateRomIndex(true);
                    }
                    if (dskEnabled)
                    {
                        dskController.setMotor((data & 0x08) != 0);
                    }
                    if (printerEnabled)
                    {
                        if ((data & 0x10) == 0)
                        {
                            if (!printerStrobe)
                            {
                                printerPort.writeData(printerByte);
                                printerStrobe = true;
                            }
                        } else {
                            printerStrobe = false;
                        }
                    }
                } else if (printerEnabled)
                {
                    // Detect 0x0ffd write access for Centronics printer
                    printerByte = data;
                }
            } else if (((gpioSix & A14_PIN_BITMASK) != 0x0) &&
                !IS_ROM_PAGED(ROM_MENU))
            {
                // Capture 0xfffd last selected AY register
                spectrumAyReg = readData();
            }
        } else {
            switch (port_)
            {
                case 0x3b :
                    {
                        uint8_t highPort = decodeHighAddress(gpioSix);
                        if (divMmcEnabled && ((highPort & 0xf0) == 0x70))
                        {
                            rtcTeensy.write((highPort & 0x0f), readData());
                        } else if (uartEnabled)
                        {
                            switch (highPort)
                            {
                                case 0x13 :
                                    espUart.writeData(UartZXTeensy::UART_WRITE, readData());
                                    break;
                                case 0x14 :
                                    espUart.writeData(UartZXTeensy::UART_SET_BAUD, readData());
                                    break;
                            }
                        }
                    }
                    break;
                case 0x3f :
                    mf128ActiveNMI = false;
                    if (mf128Enabled)
                    {
                        mf128Enabled = false;
                        divMmcEnabled = divMmcPresent;
                        divMmcRomEnabled = (divMmcEnabled && divMmcRomPresent);
                        interface1Enabled = (interface1Present && !divMmcEnabled);
                    }
                    break;
                case 0x7b :
                    if (lprintEnabled)
                    {
                        printerByte = readData();
                        printerPort.writeData(printerByte);
                    }
                    break;
                case 0x7f :
                    if (modemEnabled)
                    {
                        espUart.writeData(UartZXTeensy::UART_WRITE, readData());
                    }
                    break;
                case 0xbf :
                    if (IS_ROM_PAGED(ROM_MENU))
                    {
                        uint8_t data = readData();
                        if (data < MENU_PAGE_COUNT)
                        {
                            menuRamPtr = menuRamArray[data];
                        } else {
                            // Menu scratch RAM page 4 is "paged out" to capture
                            // PC in stack in peripheral RAM
                            switch (menuPrevRomSelected)
                            {
                                case ROM_DIVMMC :
                                    menuRamPtr = divMmcRamPtr;
                                    break;
                                case ROM_LPRINT :
                                case ROM_MENU:
                                    break;
                                default :
                                    menuRamPtr = &(menuPrevRomPtr[RAM_PAGE_SIZE]);
                                    break;
                            }
                        }
                    } else {
                        mf128ActiveNMI = false;
                    }
                    break;
                case 0xe3 :
                    if (isDivMmcSelected())
                    {
                        // DivMMC control
                        uint8_t data = readData();
                        divMmcConMem = ((data & 0x80) != 0);
                        if ((data & 0x40) != 0)
                        {
                            divMmcMapRam = true;
                        }
                        if (divMmcConMem || divMmcAutoMap)
                        {
                            PAGE_IN_ROM(ROM_DIVMMC);
                        } else {
                            PAGE_OUT_ROM(ROM_DIVMMC);
                        }

                        // DivMMC RAM banking
                        data &= (EXT_RAM_PAGE_COUNT + RAM_PAGE_COUNT - 1);
                        if (divMmcExtRamEnabled && (data >= RAM_PAGE_COUNT))
                        {
                            divMmcRamBank = data;
                            divMmcRamPtr = divMmcExtRamArray[(data - RAM_PAGE_COUNT)];
                            divMmcRamBankThree = false;
                        } else {
                            divMmcRamBank = data & (RAM_PAGE_COUNT - 1);
                            divMmcRamPtr = divMmcRamArray[divMmcRamBank];
                            divMmcRamBankThree = ((divMmcRamBank == 0x03) ? true : false);
                        }
                        updateRomIndex(true);
                    }
                    break;
                case 0xe7 :
                    if (isDivMmcSelected())
                    {
                        // DivMMC card select
                        uint8_t data = readData();
                        writeSdSpiWriteBuffer(SD_SPI_SELECT, data);
                        if ((data & 0x01) != 0)
                        {
                            digitalWriteFast(LED_PIN, 1);
                        } else {
                            digitalWriteFast(LED_PIN, 0);
                        }
                    }
                    break;
                case 0xeb :
                    if (IS_ROM_PAGED(ROM_MENU))
                    {
                        uint8_t data = readData();
                        if (isStateSaveActive() && ((data & 0x80) != 0))
                        {
                            stateSaveBlock(data);
                        } else if (!menuSelected)
                        {
                            menuSelected = true;
                            menuSelectedIndex = data;
                        }
                    } else if (isDivMmcSelected())
                    {
                        // DivMMC write
                        writeSdSpiWriteBuffer(SD_SPI_WRITE, readData());
                    }
                    break;
                case 0xfb :
                    if (lprintEnabled)
                    {
                        printerByte = readData();
                    }
                    break;
                case 0xfe :
                    if (!IS_ROM_PAGED(ROM_MENU))
                    {
                        // Capture last border colour
                        spectrumBorder = readData() & 0x07;
                    }
                    break;
                case 0xff :
                    if (modemEnabled)
                    {
                        if (modemOnReset)
                        {
                            modemOnReset = false;
                        } else {
                            uint8_t data = readData();
                            if ((data & 0x20) == 0)
                            {
                                PAGE_IN_ROM(ROM_MODEM);
                            } else {
                                PAGE_OUT_ROM(ROM_MODEM);
                            }
                            modemOnReset = ((data & 0x40) != 0);
                            updateRomIndex(true);
                        }
                    }
                    break;
            }
        }
    }
}

FASTRUN void isrRdEvent()
{
    uint32_t gpioSix = (*(volatile uint32_t *)IMXRT_GPIO6_ADDRESS);
    if ((gpioSix & RD_PIN_BITMASK) != 0)
    {
        if (busRdActive)
        {
            // End of read access
            disableData();
            busRdActive = false;

            // Enable or disable the soft ROM
            if (romCsEnable)
            {
                // Soft ROM is being enabled
                disableInternalRom();
                divMmcUpdateInterfaceOne();
                romCsEnable = false;
            } else if (romCsDisable)
            {
                // Internal ROM is being enabled
                enableInternalRom();
                divMmcUpdateInterfaceOne();
                romCsDisable = false;
            }
        }
    } else if ((gpioSix & ROM_ADDRESS_MASK) == 0x00000000)
    {
        // Perform ROM read access
        uint32_t gpioNine = (*(volatile uint32_t *)IMXRT_GPIO9_ADDRESS);
        if ((gpioNine & ROMCS_IN_PIN_BITMASK) != 0)
        {
            // External ROM is active
            disableData();
        } else if (!busRdActive)
        {
            busRdActive = true;
            uint16_t address = decodeAddress(gpioSix);

            // Perform ZXC2 address based paging
            if (zxC2Present && !zxC2Lock && !nmiPending &&
                (!zxC2ShadowRom || IS_ROM_PAGED(ROM_ZXC2)) &&
                IS_ROM_HIGHEST(ROM_ZXC2) &&
                ((address & 0xffc0) == 0x3fc0))
            {
                if (zxC3Present)
                {
                    zxC3Write = ((address & 0x08) != 0);
                    zxC2RomBank = ((address & 0x07) << 1);
                } else {
                    zxC2RomBank = ((address & 0x0f) << 1);
                }
                if ((address & 0x10) == 0)
                {
                    PAGE_IN_ROM(ROM_ZXC2);
                } else {
                    PAGE_OUT_ROM(ROM_ZXC2);
                }
                zxC2Lock = ((address & 0x20) != 0);
                updateRomIndex(true);
            }

            // Detect M1 cycle for ROM paging
            if ((gpioNine & M1_PIN_BITMASK) != 0)
            {
                // Non-M1 cycle - write ROM data to bus
                writeRomData(address);
            } else if (address == 0x66)
            {
                // Send the NMI to the menu ROM, or Multiface 128
                if (nmiPending)
                {
                    if (nmiRomTarget == ROM_MENU)
                    {
                        // Send the NMI to the menu, and set scratch RAM
                        // to store existing Spectrum state
                        menuRamPtr = menuRamArray[1];

                        // Directly page in the menu from any ROM, as highest
                        // priority
                        menuPrevRomPtr = romPtr;
                        menuPrevRomSelected = romSelected;
                        PAGE_IN_ROM(ROM_MENU);
                        romSelected = ROM_MENU;
                        romArraySelected = BANK_MENU;
                        updateRomPtr(true);
                    } else if (mf128Present && !mf128ActiveNMI &&
                        (nmiRomTarget != ROM_DIVMMC) && (nmiRomTarget != ROM_ROM3) &&
                        ((romArraySelected & (BANK_ROM0 | BANK_ROM1 | BANK_ROM3 |
                            BANK_MF128)) != 0))
                    {
                        // Send the NMI to the Multiface 128
                        mf128Enabled = true;
                        mf128ActiveNMI = true;
                        divMmcEnabled = false;
                        divMmcRomEnabled = false;
                        interface1Enabled = interface1Present;

                        // Directly page in the Multiface 128 from ROM 0/1/3
                        PAGE_IN_ROM(ROM_MF128);
                        romSelected = ROM_MF128;
                        romArraySelected = BANK_MF128;
                        updateRomPtr(true);
                    }
                }

                // Write ROM data to bus
                writeRomData(0x66);

                // Send the NMI to the DivMMC, if not Multiface 128
                if (divMmcRomEnabled && !mf128ActiveNMI && (nmiRomTarget != ROM_ROM3) &&
                    ((romArraySelected & (BANK_ROM0 | BANK_ROM1 | BANK_ROM3)) != 0))
                {
                    divMmcAutoMap = true;

                    // Directly page in the DivMMC from ROM 0/1/3
                    PAGE_IN_ROM(ROM_DIVMMC);
                    romSelected = ROM_DIVMMC;
                    romArraySelected = BANK_DIVMMC;
                    updateRomPtr(false);
                }

                // Release NMI on entry to interrupt handler
                nmiPending = false;
                nmiRomTarget = ROM_ROM0;
                digitalWriteFast(NMI_PIN, 0);
            } else {
                switch (romSelected)
                {
                    case ROM_ROM0 :
                    case ROM_ROM1 :
                    case ROM_ROM3 :
                        // Detect M1 cycle for Multiface 128 paging
                        if (mf128ActiveNMI && (address == 0x67))
                        {
                            // Directly page in the Multiface 128 from ROM 0/1/3
                            PAGE_IN_ROM(ROM_MF128);
                            romSelected = ROM_MF128;
                            romArraySelected = BANK_MF128;
                            updateRomPtr(true);
                        }

                        if (divMmcRomEnabled)
                        {
                            // Detect M1 cycle for DivMMC paging
                            if ((address & 0xff00) == 0x3d00)
                            {
                                divMmcAutoMap = true;

                                // Directly page in the DivMMC from ROM 0/1/3
                                PAGE_IN_ROM(ROM_DIVMMC);
                                romSelected = ROM_DIVMMC;
                                romArraySelected = BANK_DIVMMC;
                                updateRomPtr(true);

                                // Write ROM data to bus
                                writeDivMmcRomData(address);
                            } else {
                                // Write ROM data to bus
                                writePagedRomData(address);
                            }

                            // Detect post-M1 cycle for DivMMC paging
                            if ((address == 0x00) || (address == 0x08) ||
                                (address == 0x38) || (address == 0x4c6) ||
                                (address == 0x562))
                            {
                                divMmcAutoMap = true;

                                // Directly page in the DivMMC from ROM 0/1/3
                                PAGE_IN_ROM(ROM_DIVMMC);
                                romSelected = ROM_DIVMMC;
                                romArraySelected = BANK_DIVMMC;
                                updateRomPtr(false);
                            }
                        } else {
                            // Write ROM data to bus
                            writePagedRomData(address);

                            // Detect post-M1 cycle for Interface 1 paging
                            if ((interface1Enabled || zxC2ShadowRom) &&
                                ((address == 0x08) || (address == 0x1708)))
                            {
                                if (zxC2ShadowRom)
                                {
                                    // Directly page in the shadow ROM from ROM 0/1/3
                                    PAGE_IN_ROM(ROM_ZXC2);
                                    romSelected = ROM_ZXC2;
                                    romArraySelected = BANK_RAM;
                                } else {
                                    // Directly page in the Interface 1 from ROM 0/1/3
                                    PAGE_IN_ROM(ROM_IF1);
                                    romSelected = ROM_IF1;
                                    romArraySelected = BANK_IF1;
                                }
                                updateRomPtr(false);
                            }

                            // Detect LD-BYTEs to start tape
                            if (tzxEnabled && (address == 0x56c))
                            {
                                tzxPlayer.play();
                            }
                        }
                        break;
                    case ROM_IF1 :
                        // Write ROM data to bus
                        writePagedRomData(address);

                        // Detect post-M1 cycle for Interface 1 paging
                        if (address == 0x700)
                        {
                            PAGE_OUT_ROM(ROM_IF1);
                            updateRomIndex(false);
                        }
                        break;
                    case ROM_MF128 :
                        // Write ROM data to bus
                        writePagedRomData(address);

                        // Detect post-M1 cycle for Interface 1 paging
                        if (interface1Enabled &&
                            ((address == 0x08) || (address == 0x1708)))
                        {
                            // M1 cycle for Interface 1 paging
                            // NOTE: ROM_MF128 is higher priority than ROM_IF1,
                            // so do NOT page now
                            PAGE_IN_ROM(ROM_IF1);
                        }
                        break;
                    case ROM_DIVMMC :
                        // Detect M1 cycle for DivMMC paging
                        if (divMmcRomEnabled && ((address & 0xff00) == 0x3d00))
                        {
                            divMmcAutoMap = true;
                        }

                        // Write ROM data to bus
                        writeDivMmcRomData(address);

                        // Detect post-M1 cycle for DivMMC paging
                        // NOTE: Avoid paging out on MAPRAM to allow DivMMC
                        // loaded ROM images to behave correctly
                        if (!divMmcMapRam && ((address & 0xfff8) == 0x1ff8))
                        {
                            divMmcAutoMap = false;
                            if (!divMmcConMem)
                            {
                                PAGE_OUT_ROM(ROM_DIVMMC);
                            }
                            updateRomIndex(false);

                            // Disable the DivMMC, and enable the Interface 1
                            if (divMmcToggle)
                            {
                                divMmcToggle = false;
                                divMmcEnabled = false;
                                divMmcRomEnabled = false;
                                interface1Enabled = interface1Present;
                            }
                        }
                        break;
                    case ROM_LPRINT :
                        // Write ROM data to bus
                        writeLprintRomData(address);
                        break;
                    case ROM_ZXC2 :
                        // Write ROM data to bus
                        writeZXC2RomData(address);

                        // Detect post-M1 cycle for shadow ROM paging
                        if (zxC2ShadowRom && (address == 0x700))
                        {
                            PAGE_OUT_ROM(ROM_ZXC2);
                            updateRomIndex(false);
                        }
                        break;
                    case ROM_MLD :
                        // Write ROM data to bus
                        writeZXC2RomData(address);
                        break;
                    case ROM_MENU :
                        // Write ROM data to bus
                        writeMenuRomData(address);

                        // Detect post-M1 cycle for menu paging
                        if (address == 0x003B)
                        {
                            PAGE_OUT_ROM(ROM_MENU);
                            updateRomIndex(false);

                            // Trigger NMI directly for given target
                            if (nmiRomTarget != ROM_ROM0)
                            {
                                nmiPending = true;
                                digitalWriteFast(NMI_PIN, 1);
                            }
                        }
                        break;
                    default :
                        // Write ROM data to bus
                        writePagedRomData(address);
                        break;
                }
            }

            // Detect snapshot loader paging
            if (IS_ROM_PAGED(ROM_SNA) && (address == 0x3FFF))
            {
                if (snaLoaderBanks > 0)
                {
                    zxC2RomBank += 2;
                    if (zxC2RomBank >= snaLoaderBanks)
                    {
                        zxC2RomBank = 0;
                        snaLoaderBanks = 0;
                        stateLoaderFinished(false);
                    }
                    updateRomPtr(false);
                } else {
                    PAGE_OUT_ROM(ROM_SNA);
                    stateLoaderFinished(true);
                    snaLoaderPresent = false;
                    updateRomIndex(false);
                }
            }

#ifdef DEBUG_OUTPUT
            // Debug instruction tracing
            if ((gpioNine & M1_PIN_BITMASK) == 0)
            {
                //traceDebug(address);
            }
#endif
        }
    } else if (!busRdActive && ((gpioSix & IOREQ_PIN_BITMASK) == 0x00000000))
    {
        // Perform I/O read access
        uint8_t port = decodeLowAddress(gpioSix);
        busRdActive = true;
        switch (port)
        {
            case 0x1f :
                if (joystickPresent)
                {
                    writeData(joystickData);
                }
                break;
            case 0x3b :
                {
                    uint8_t highPort = decodeHighAddress(gpioSix);
                    if (divMmcEnabled && ((highPort & 0xf0) == 0x70))
                    {
                        writeData(rtcTeensy.read(highPort & 0x0f));
                    } else if (uartEnabled)
                    {
                        switch (highPort)
                        {
                            case 0x13 :
                                writeData(espUart.getUartStatusByte());
                                break;
                            case 0x14 :
                                writeData(espUart.hasReadData() ? espUart.readData() : 0x00);
                                break;
                        }
                    }
                }
                break;
            case 0x3f :
                if (mf128Enabled)
                {
                    writeData(((spectrumBankM & 0x08) != 0) ? 0x80 : 0x00);
                }
                if (IS_ROM_PAGED(ROM_MF128))
                {
                    PAGE_OUT_ROM(ROM_MF128);
                    updateRomIndex(true);
                }
                break;
            case 0x7b :
                if (lprintEnabled)
                {
                    writeData(printerPort.getBusy() ? 0xC0 : 0x40);
                    PAGE_OUT_ROM(ROM_LPRINT);
                    updateRomIndex(true);
                }
                break;
            case 0x7f :
                if (modemEnabled)
                {
                    writeData(espUart.hasReadData() ? espUart.readData() : 0x00);
                }
                break;
            case 0xbf :
                if (mf128Enabled)
                {
                    writeData(((spectrumBankM & 0x08) != 0) ? 0x80 : 0x00);
                    if (IS_ROM_PAGED(ROM_MF128) == 0)
                    {
                        PAGE_IN_ROM(ROM_MF128);
                        updateRomIndex(true);
                    }
                }
                break;
            case 0xdf :
                if (mousePresent)
                {
                    switch (decodeHighAddress(gpioSix))
                    {
                        case 0xfa :
                            writeData(mouseBtn);
                            break;
                        case 0xfb :
                            writeData(mouseX >> 1);
                            break;
                        case 0xff :
                            writeData(mouseY >> 1);
                            break;
                    }
                }
                break;
            case 0xeb :
                if (IS_ROM_PAGED(ROM_MENU))
                {
                    writeData(menuBuffer.canRead() ? menuBuffer.readRaw() :
                        MENU_ROM_CMD_IDLE);
                } else if (isDivMmcSelected())
                {
                    // Transfer SD SPI read data to bus
                    uint8_t data;
                    switch (divMmcDrive)
                    {
                        case DIVMMC_SDHC :
                            data = divMmcSpi.readData();
                            break;
                        case DIVMMC_HDF_A :
                            data = divMmcHdf.readData();
                            break;
                        case DIVMMC_HDF_B :
                            data = divMmcSecondHdf.readData();
                            break;
                        default :
                            data = 0xFF;
                            break;
                    }
                    writeData(data);
                }
                break;
            case 0xfb :
                if (lprintEnabled)
                {
                    writeData(printerPort.getBusy() ? 0xC0 : 0x40);
                    PAGE_IN_ROM(ROM_LPRINT);
                    updateRomIndex(true);
                }
                break;
            case 0xfd :
                switch (decodeHighAddress(gpioSix))
                {
                    case 0x0f :
                        if (printerEnabled)
                        {
                            writeData(printerPort.getBusy() ? 0x01 : 0x00);
                        }
                        break;
                    case 0x2f :
                        if (dskEnabled)
                        {
                            writeData(dskController.getStatusByte());
                        }
                        break;
                    case 0x3f :
                        if (dskEnabled)
                        {
                            writeData(dskController.readData());
                        }
                        break;
                }
                break;
            case 0xfe :
                if (tzxEnabled && tzxPlayer.isTapePlaying())
                {
                    writeData(tzxPlayer.getTapeByte());
                }
                break;
            case 0xff :
                if (modemEnabled)
                {
                    writeData(modemOnReset ? 0x00 : espUart.getModemStatusByte());
                }
                break;
        }
    }
}
