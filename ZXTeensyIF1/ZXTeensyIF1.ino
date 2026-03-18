
#define ZXTEENSY_VERSION "20260318"
#define ENABLE_BUILTIN_ROM_IF1
//define DEBUG_OUTPUT

#define ESXMMC_BIN_PATH ((const char*)F("/ZXTEENSY/esxmmc.bin"))
#define MF128_ROM_PATH ((const char*)F("/ZXTEENSY/mf128.rom"))
#define IF1_ROM_PATH ((const char*)F("/ZXTEENSY/if1.rom"))
#define MENU_ROM_PATH ((const char*)F("/ZXTEENSY/menu.rom"))
#define MDR_EMULATOR_ROM_PATH ((const char*)F("/ZXTEENSY/SPECTRA_IF1_ED2_ME_ROM_Formatted.bin"))

#include <SD.h>
#include <SdFat.h>
#include "USBHost_t36.h"
#include "if1-2_rom.h"
#include "RingBuffer.h"
#include "SdHdfZXTeensy.h"
#include "SdSdioZXTeensy.h"
#include "UartZXTeensy.h"
#include "RtcZXTeensy.h"
#include "EspNtpZXTeensy.h"
#include "TzxPlayerZXTeensy.h"
#include "Dsk765ZXTeensy.h"

// Run the Teensy 4.1 with slight overclock at 816MHz
// Tick the SD and Serial at 14MHz
#define TEENSY_CLK_FREQ 816000000ULL
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

extern "C" volatile uint32_t systick_millis_count;
extern "C" uint32_t set_arm_clock(uint32_t frequency);

static const char PROGMEM VERSION_STR[9] = ZXTEENSY_VERSION;
static const size_t MAX_PATH = 256;

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
    MENU_ACTION_BROWSER_OPEN,
    MENU_ACTION_BROWSER_OPEN_ZXC2,
    MENU_ACTION_BROWSER_OPEN_HDF,
    MENU_ACTION_BROWSER_OPEN_DSK,
    MENU_ACTION_BROWSER_LOAD_CART,
    MENU_ACTION_BROWSER_LOAD_ZXC2,
    MENU_ACTION_BROWSER_LOAD_ZXC3,
    MENU_ACTION_BROWSER_LOAD_Z80,
    MENU_ACTION_BROWSER_LOAD_TZX,
    MENU_ACTION_BROWSER_LOAD_MDR,
    MENU_ACTION_BROWSER_MOUNT_SDA,
    MENU_ACTION_BROWSER_MOUNT_SDB,
    MENU_ACTION_BROWSER_MOUNT_FDA,
    MENU_ACTION_BROWSER_MOUNT_FDB,
    MENU_ACTION_START_SERVER,
    MENU_ACTION_STOP_SERVER
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
    BANK_DIVMMC = 0x0020,
    BANK_MF128  = 0x0040,
    BANK_RAM    = 0x0080
} bank_select_t;

typedef enum {
    // 16KB ROMs
    ROM_ROM0,
    ROM_ROM1,
    ROM_ROM2,
    ROM_ROM3,
    ROM_IF1,
    ROM_MF128, // Upper ROM_MF128 is MF128 RAM
    // 8KB ROMs
    ROM_DIVMMC,
    // "Dynamic ROMs" that use DivMMC RAM
    ROM_ZXC2,
    ROM_SNA,
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
    ROM_PAGE_COUNT
} rom_page_t;

typedef enum {
    TYPE_ROM,
    TYPE_ZXC2,
    TYPE_CART,
    TYPE_Z80,
    TYPE_SNA
} rom_type_t;

typedef enum {
    SD_SPI_WRITE,
    SD_SPI_READ,
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
static const uint8_t NUM_SD_RETRIES = 3;

// Global state
volatile bool bootIntoMenu = false;
volatile bool afterFirstReset = false;
volatile bool isDeviceDisabled = false;
volatile bool loadRomSets = false;
volatile bool sdCardPresent = false;
volatile bool divMmcCardPresent = false;
volatile run_state_t globalState = STATE_RESET;
volatile bool busRdActive = false;
volatile bool nmiPending = false;

// Reset and NMI debouncing
volatile trigger_state_t resetTrigState = TRIGGER_ACTIVE;
volatile uint32_t resetTrigExitCount = TRIGGER_DELAY_CNT;
volatile uint32_t resetHardTrigCount = HARD_RESET_DELAY_CNT;
volatile trigger_state_t buttonTrigState = TRIGGER_READY;
volatile uint32_t buttonTrigExitCount = 0;

// ROM banking
static const uint16_t RAM_PAGE_SIZE = 0x2000;
static const uint16_t ROM_PAGE_SIZE = (RAM_PAGE_SIZE * 2);
volatile rom_select_t romSelected = ROM_ROM0;
volatile bank_select_t romArraySelected = BANK_ROM0;
volatile uint8_t romArray[ROM_PAGE_COUNT][RAM_PAGE_SIZE] __attribute__((aligned(16)));
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
volatile bool divMmcEnabled = false;
volatile bool divMmcRomEnabled = false;
volatile bool divMmcToggle = false;
volatile bool divMmcPaged = false;
volatile bool divMmcAutoMap = false;
volatile bool divMmcConMem = false;
volatile bool divMmcMapRam = false;
volatile bool divMmcRamBankThree = false;
volatile bool divMmcPreserveRam = false;
volatile bool divMmcExtRamEnabled = false;
volatile uint8_t* divMmcRamPtr;
const uint16_t PAGE_BANK_DIVMMC = (BANK_ROM0 | BANK_ROM1 | BANK_ROM3);

// Multiface 128
volatile bool mf128Present = false;
volatile bool mf128Enabled = false;
volatile bool mf128Paged = false;
volatile bool mf128VideoRam = false;
volatile bool mf128ActiveNMI = false;

// Interface 1
volatile bool interface1Present = false;
volatile bool interface1Enabled = false;
volatile bool interface1Paged = false;
volatile bool interface1Removed = false;
const uint16_t PAGE_BANK_MF128_IF1 = (BANK_ROM0 | BANK_ROM1 | BANK_ROM3 | BANK_MF128);

// ZXC2 cartridge (reuses divMmcExtRamArray)
volatile bool zxC2Present = false;
volatile bool zxC2Paged = false;
volatile bool zxC2Lock = false;
volatile bool zxC2ShadowRom = false;
volatile uint8_t zxC2BankPtr = 0x00;

// ZXC3 flash cartridge
static const uint16_t ZXC3_PAGE_COUNT = 16;
volatile bool zxC3Present = false;
volatile bool zxC3Write = false;
volatile zxc3_flash_state_t zxC3FlashState = ZXC3_FLASH_IDLE;
volatile bool zxC3FlashSetup = false;
volatile trigger_state_t zxC3WriteTrigState = TRIGGER_READY;
volatile uint32_t zxC3WriteTrigExitCount = 0;
volatile trigger_state_t zxC3EraseTrigState = TRIGGER_READY;
volatile uint32_t zxC3EraseTrigExitCount = 0;
DMAMEM RingBuffer<EXT_RAM_PAGE_COUNT> zxC3EraseBuffer;

// Microdrive emulator
static const uint8_t MDR_MAX_SECTOR = 0xB4;
volatile bool mdrPresent = false;
volatile bool mdrEnabled = false;
volatile uint8_t mdrMaxSector = 0;

// Z80 snapshot loader banking
volatile bool snaLoaderPresent = false;
volatile bool snaLoaderPaged = false;
volatile uint8_t snaLoaderBanks = 0;

// Boot menu ROM (reuses divMmcRamArray)
volatile bool menuEnterOnReset = false;
volatile bool menuPaged = false;
volatile bool menuSelected = false;
volatile uint8_t menuSelectedIndex = 0;

// DivMMC SPI/SD
static const size_t READ_BUFFER_SIZE = 1024;
static const size_t WRITE_BUFFER_SIZE = 16;
RingBuffer<READ_BUFFER_SIZE> sdSpiReadBuffer;
RingBuffer<WRITE_BUFFER_SIZE> sdSpiWriteBuffer;
RingBuffer<WRITE_BUFFER_SIZE> sdSpiFlagsBuffer;
SdSdioZXTeensy<READ_BUFFER_SIZE> divMmcSpi;
SdHdfZXTeensy<READ_BUFFER_SIZE> divMmcHdf;
SdHdfZXTeensy<READ_BUFFER_SIZE> divMmcSecondHdf;
volatile divmmc_spi_t divMmcDrive = DIVMMC_NONE;
volatile divmmc_spi_t divMmcDriveSlot[2];

// MB03+ UART
static const size_t UART_BUFFER_SIZE = 4096;
volatile bool uartPresent = false;
volatile bool uartEnabled = false;
UartZXTeensy espUart;
DMAMEM uint8_t uartBuffer[UART_BUFFER_SIZE];

// RTC module
volatile bool wifiNtpPresent = false;
volatile bool wifiNtpEnabled = false;
volatile bool wifiNtpHasTime = false;
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

// u765 disk controller
Dsk765ZXTeensy dskController;
volatile bool dskPresent = false;
volatile bool dskEnabled = false;

// SPI and UART tick cycle counter
volatile uint32_t globalCycleCount;

// Optimised ISR functions
FASTRUN void isrFastGpios() __attribute__((hot, optimize("O3")));
FASTRUN void isrRdEvent() __attribute__((hot, optimize("O3")));
FASTRUN void isrWrEvent() __attribute__((hot, optimize("O3")));

// Optimised task loop functions
FASTRUN void loop() __attribute__((hot, optimize("O3")));
inline void sdSpiOnTick() __attribute__((always_inline, hot, optimize("O3")));

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

inline __attribute__((always_inline)) void flushSdSpiBuffers()
{
    sdSpiReadBuffer.clear();
    sdSpiFlagsBuffer.clear();
    sdSpiWriteBuffer.clear();
}

// NOTE: onTick is main loop, so optimize
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
                        divMmcSpi.performTick(true, data);
                        break;
                    case DIVMMC_HDF_A :
                        divMmcHdf.performTick(true, data);
                        break;
                    case DIVMMC_HDF_B :
                        divMmcSecondHdf.performTick(true, data);
                        break;
                    default :
                        break;
                }
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

inline __attribute__((always_inline)) void updateRomIndex(bool pageNow)
{
    // Determine which ROM is currently paged
    if (menuPaged)
    {
        romSelected = ROM_MENU;
        romArraySelected = BANK_RAM;
    } else if (zxC2Paged)
    {
        romSelected = ROM_ZXC2;
        romArraySelected = BANK_RAM;
    } else if (snaLoaderPaged)
    {
        romSelected = ROM_SNA;
        romArraySelected = BANK_RAM;
    } else if (mf128Paged)
    {
        romSelected = ROM_MF128;
        romArraySelected = BANK_MF128;
    } else if (divMmcPaged)
    {
        romSelected = ROM_DIVMMC;
        romArraySelected = BANK_DIVMMC;
    } else if (interface1Paged)
    {
        romSelected = ROM_IF1;
        romArraySelected = BANK_IF1;
    } else if (rom23Paged)
    {
        if (rom1Paged)
        {
            romArraySelected = BANK_ROM3;
            romSelected = ROM_ROM3;
        } else {
            romArraySelected = BANK_ROM2;
            romSelected = ROM_ROM2;
        }
    } else if (rom1Paged)
    {
        romArraySelected = BANK_ROM1;
        romSelected = ROM_ROM1;
    } else {
        romArraySelected = BANK_ROM0;
        romSelected = ROM_ROM0;
    }

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
                    // 8KB ROM
                    romPtr = romArray[ROM_PAGE_DIVMMC];
                }
                break;
            case ROM_ZXC2 :
            case ROM_SNA :
                romPtr = divMmcExtRamArray[zxC2BankPtr];
                break;
            case ROM_MENU :
                romPtr = divMmcRamArray[0];
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
#ifdef DEBUG_OUTPUT
    Serial.begin(115200);
#endif
    if (CrashReport)
    {
#ifndef DEBUG_OUTPUT
        Serial.begin(115200);
#endif
        Serial.print(CrashReport);
    }

    // Configure UART
    Serial8.addMemoryForRead(uartBuffer, UART_BUFFER_SIZE);

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

bool detectSdCard()
{
    pinMode(SD_CS_PIN, INPUT_PULLDOWN);
    delayMicroseconds(5);
    return digitalReadFast(SD_CS_PIN);
}

bool beginSdfsSd()
{
    if (divMmcCardPresent)
    {
        divMmcSpi.end();
        divMmcCardPresent = false;
    }
    if (!sdCardPresent)
    {
        if (detectSdCard())
        {
            uint8_t retries_ = 0;
            while (!SD.sdfs.begin(SdioConfig(FIFO_SDIO)))
            {
                if (++retries_ > NUM_SD_RETRIES)
                {
                    return false;
                }
            }
            sdCardPresent = true;
        } else {
            return false;
        }
    }
    return true;
}

bool beginDivMmcSd()
{
    if (sdCardPresent)
    {
        divMmcHdf.end();
        divMmcSecondHdf.end();
        SD.sdfs.end();
        sdCardPresent = false;
    }
    if (!divMmcCardPresent)
    {
        if (detectSdCard() && SD.sdfs.cardBegin(SdioConfig(FIFO_SDIO)))
        {
            divMmcSpi.begin(&sdSpiReadBuffer, SD.sdfs.card());
        } else {
            return false;
        }
        divMmcCardPresent = true;
    }
    return true;
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
    // Reset the Spectrum ROM state
    romArrayPresent &= ~(BANK_ROM0 | BANK_ROM1 | BANK_ROM2 | BANK_ROM3);
    rom1Present = false;
    rom23Present = false;

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
            zxC2ShadowRom = (strstr(RomFile.name(), "SPECTRA_") != 0);
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
    File saveFile = SD.open(filePath, FILE_WRITE_BEGIN);
    if (saveFile)
    {
        for (uint8_t i_ = 0; i_ < ZXC3_PAGE_COUNT; ++i_)
        {
            if (saveFile.write((uint8_t*)divMmcExtRamArray[i_], RAM_PAGE_SIZE) < RAM_PAGE_SIZE)
            {
                break;
            }
        }
        saveFile.close();
    }
}

bool loadSnapshotFile(File RomFile, bool isSnaFile)
{
    // Convert the Z80 snapshot into a loader ROM, in the DivMMC RAM area
    if (RomFile)
    {
        snaLoaderBanks = convertZ80toROM(RomFile, (uint8_t*)divMmcExtRamArray[0],
            (uint8_t*)divMmcExtRamArray[(EXT_RAM_PAGE_COUNT - RAM_PAGE_COUNT)],
            isSnaFile);
        if (snaLoaderBanks > 0)
        {
            snaLoaderBanks <<= 1;
            snaLoaderPresent = true;
            divMmcExtRamEnabled = false;
            romArrayPresent |= BANK_RAM;
        }
        RomFile.close();
    }
    return snaLoaderPresent;
}

bool loadTzxPlayerFile(const char* fileName, size_t* count)
{
    // Reset the tape player state
    tzxPresent = false;

    // The tape is loaded in the DivMMC RAM area
    File tzxFile = SD.open(fileName, FILE_READ);
    if (tzxFile)
    {
        *count = tzxFile.readBytes((char *)divMmcExtRamArray[0], RAM_PAGE_SIZE);
        if (*count > 0)
        {
            tzxPresent = true;
            divMmcExtRamEnabled = false;
            for (uint8_t i_ = 1; i_ < EXT_RAM_PAGE_COUNT; ++i_)
            {
                size_t blk_count_ = tzxFile.readBytes((char *)divMmcExtRamArray[i_],
                    RAM_PAGE_SIZE);
                *count += blk_count_;
                if (blk_count_ < RAM_PAGE_SIZE)
                {
                    break;
                }
            }
        }
        tzxFile.close();
    } else {
        *count = 0;
    }
    return (*count > 0);
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
    File mdrFile = SD.open(fileName, FILE_WRITE_BEGIN);
    if (mdrFile)
    {
        // Load the header from the first sector
        uint8_t buffer[0x21F];
        uint8_t sector = mdrMaxSector;
        uint8_t* ptr = (uint8_t*)divMmcExtRamArray[2];
        memcpy(buffer, ptr + 1, 15);

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
            mdrFile.write(buffer, 0x21F);
        }

        // Write out the existing sectors from pages 1 to 6
        for (uint8_t i_ = 2; i_ < 14; i_ += 2)
        {
            for (uint8_t j_ = 0; j_ < 0x1E; ++j_)
            {
                uint8_t* ptr = (uint8_t*)divMmcExtRamArray[i_] + (j_ * 0x220);
                if (*ptr != 0xFF)
                {
                    mdrFile.write(ptr + 1, 0x21F);
                    --sector;
                }
            }
        }
        mdrFile.close();
    }
}

bool loadForegroundRom()
{
    // Reset the ZXC2 cartridge state
    romArrayPresent &= ~(BANK_RAM);
    zxC2Present = false;
    zxC3Present = false;
    zxC2ShadowRom = false;
    snaLoaderPresent = false;

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
    divMmcExtRamEnabled = true;
    romArrayPresent &= ~(BANK_RAM);
    memset((void*)divMmcRamArray, 0xFF, (RAM_PAGE_COUNT * RAM_PAGE_SIZE));
    memset((void*)divMmcExtRamArray, 0xFF, (EXT_RAM_PAGE_COUNT * RAM_PAGE_SIZE));
    memset((void*)&(romArray[ROM_PAGE_MF128][RAM_PAGE_SIZE]), 0xFF, RAM_PAGE_SIZE);
}

void performHardReset()
{
    // Disable the ESP-01S
    pinMode(ESP_ENABLE, OUTPUT);
    digitalWriteFast(ESP_ENABLE, 0);

    // Clear the UART
    espUart.end();

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
    if (menuEnterOnReset)
    {
        // Reset and reload the configuration to enter the menu
        afterFirstReset = false;
        isDeviceDisabled = false;
    }
    if (!afterFirstReset)
    {
        romArrayPresent = 0;
        rom1Present = false;
        rom23Present = false;
        interface1Present = false;
        divMmcPresent = false;
        mf128Present = false;
        zxC2Present = false;
        zxC3Present = false;
        snaLoaderPresent = false;
        tzxPresent = false;
        mdrPresent = false;

        // Re-initialise RAM, and load the ROMs
        loadRomSets = true;
        divMmcPreserveRam = false;

        // Ensure the DivMMC images are closed
        if (sdCardPresent)
        {
            divMmcHdf.end();
            divMmcSecondHdf.end();
        }

        // Clear the menu action
        menuResetAction();
    }

    // Initialise the RAM banks
    if (!divMmcPreserveRam)
    {
        initialiseRamBanks();
    }

    // Update the RTC registers, if necessary
    rtcTeensy.updateRtc();

    // Initialise the device
    delay(250);
    if (!isDeviceDisabled)
    {
        // Load the built-in Interface 1 soft ROM
#ifdef ENABLE_BUILTIN_ROM_IF1
        memcpy((void *)romArray[ROM_PAGE_IF1], BUILTIN_ROM_IF1, BUILTIN_ROM_IF1_SIZE);
        romArrayPresent |= BANK_IF1;
#endif

        // Load ROMs from the SD card
        if (loadRomSets)
        {
            loadRomSets = false;
            if (beginSdfsSd())
            {
                // Load DivMMC Esxdos ROM
                if (loadRomImage(ESXMMC_BIN_PATH, (char *)romArray[ROM_PAGE_DIVMMC],
                    RAM_PAGE_SIZE) > 0)
                {
                    romArrayPresent |= BANK_DIVMMC;
                }

                // Load Multiface 128 ROM
                if (loadRomImage(MF128_ROM_PATH, (char *)romArray[ROM_PAGE_MF128],
                    RAM_PAGE_SIZE) > 0)
                {
                    romArrayPresent |= BANK_MF128;
                }

                // Load Interface 1 ROM
                if (loadRomImage(IF1_ROM_PATH, (char *)romArray[ROM_PAGE_IF1],
                    ROM_PAGE_SIZE) > 0)
                {
                    romArrayPresent |= BANK_IF1;
                }

                // Load configuration
                if (!afterFirstReset)
                {
                    menuLoadConfiguration(0);
                }

                // Load Spectrum ROM
                loadSpectrumRomFile();

                // Load foreground ROM
                if (!loadForegroundRom())
                {
                    if (afterFirstReset)
                    {
                        afterFirstReset = false;
                        handleStateResetEntry();
                        return;
                    }
                }

                // Load menu ROM into the DivMMC RAM area
                if ((menuEnterOnReset || (!afterFirstReset && bootIntoMenu)) &&
                    !digitalReadFast(ROMCS_IN_PIN) &&
                    (loadRomImage(MENU_ROM_PATH, (char *)divMmcRamArray[0],
                        RAM_PAGE_SIZE) > 0))
                {
                    menuPaged = true;
                    romArrayPresent |= BANK_RAM;
                }
            } else if (!isButtonHeld)
            {
                // Button without SD card disables the built-in Interface 1 soft ROM
                interface1Present = ((romArrayPresent & BANK_IF1) != 0);
            }
        }
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
    if (menuPaged)
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
    digitalWriteFast(NMI_PIN, 0);

    // Handle actions before warm reset
    if (afterFirstReset)
    {
        handleWarmStateReset();
    }

    // Reset the UART state, and clear buffers of any idle data
    httpStopServer();
    if (uartEnabled)
    {
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

    // Reset the banking state
    menuPaged = false;
    menuSelected = false;
    rom1Paged = false;
    rom23Paged = false;
    interface1Paged = false;
    interface1Enabled = false;
    divMmcPaged = false;
    divMmcEnabled = false;
    divMmcToggle = false;
    divMmcConMem = false;
    divMmcAutoMap = false;
    divMmcMapRam = false;
    divMmcRamPtr = divMmcRamArray[0];
    mf128Paged = false;
    mf128Enabled = false;
    mf128VideoRam = false;
    mf128ActiveNMI = false;
    zxC2Paged = false;
    zxC2Lock = false;
    zxC2BankPtr = 0x00;
    zxC2ShadowRom = false;
    zxC3Write = false;
    zxC3FlashState = ZXC3_FLASH_IDLE;
    zxC3FlashSetup = false;
    zxC3WriteTrigState = TRIGGER_READY;
    mdrEnabled = false;
    snaLoaderPaged = false;
    uartEnabled = false;
    tzxEnabled = false;
    dskEnabled = false;
    romSelected = ROM_ROM0;
    romArraySelected = BANK_ROM0;

    // Reset the USB detection state
    mousePresent = false;
    joystickPresent = false;

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

    // If UART is required, then initialise
    if (uartPresent)
    {
        // Start the UART
        espUart.begin(0);

        // Disable time over WiFi, in case already started
        if (!wifiNtpPresent)
        {
            wifiNtp.end();
        } else if (!wifiNtpHasTime)
        {
            // Start to get time over WiFi, when not already sync'd
            if (year() >= 2026)
            {
                wifiNtpHasTime = true;
            } else {
                wifiNtp.begin(&espUart);
                wifiNtpEnabled = true;
            }
        }
    }

    // Flag that reset has occurred
    afterFirstReset = true;
    menuEnterOnReset = false;

    // Populate the menu when active
    if (menuPaged)
    {
        menuInitialise(divMmcRamArray[0]);
    } else {
        // Page in the ZXC2 cartridge, or snapshot loader ROM
        if (zxC2Present)
        {
            zxC2Paged = !zxC2ShadowRom;
        } else if (snaLoaderPresent)
        {
            snaLoaderPaged = true;
        }

        // Enable DSK, MDR and TZX peripherals, and prevent direct SD card access
        bool hasSdAccess = false;
        if (dskPresent && beginSdfsSd())
        {
            dskEnabled = true;
            hasSdAccess = true;
            dskController.begin(menuGetFdcFdaPath(), menuGetFdcFdbPath());
        }
        if (mdrPresent && beginSdfsSd() &&
            loadMdrEmulatorFile(menuGetBrowserPath()))
        {
            mdrEnabled = true;
            hasSdAccess = true;
        }
        if (tzxPresent && beginSdfsSd())
        {
            size_t size;
            if (loadTzxPlayerFile(menuGetBrowserPath(), &size))
            {
                tzxEnabled = true;
                tzxPlayer.begin(divMmcExtRamArray[0], size);
            }
        }

        // Enable the DivMMC
        if (divMmcPresent)
        {
            // The Interface 1 can be enabled by switching back
            // into 128k mode (".128") or enabling the Multiface 128
            divMmcEnabled = true;
            char* sdaPath = menuGetDivMmcSdaPath();
            char* sdbPath = menuGetDivMmcSdbPath();
            if ((sdaPath == 0) && (sdbPath != 0))
            {
                divMmcDriveSlot[0] = (hasSdAccess ? DIVMMC_NONE : DIVMMC_SDHC);
                if (divMmcSecondHdf.begin(&sdSpiReadBuffer, sdbPath))
                {
                    divMmcDriveSlot[1] = DIVMMC_HDF_B;
                } else {
                    divMmcDriveSlot[1] = DIVMMC_NONE;
                }
            } else {
                if ((sdbPath != 0) &&
                    divMmcSecondHdf.begin(&sdSpiReadBuffer, sdbPath))
                {
                    divMmcDriveSlot[1] = DIVMMC_HDF_B;
                } else {
                    divMmcDriveSlot[1] = DIVMMC_NONE;
                }
                if ((sdaPath != 0) &&
                    divMmcHdf.begin(&sdSpiReadBuffer, sdaPath))
                {
                    divMmcDriveSlot[0] = DIVMMC_HDF_A;
                } else {
                    divMmcDriveSlot[0] = (hasSdAccess ? DIVMMC_NONE : DIVMMC_SDHC);
                }
            }
        }

        // Enable the Interface 1 when DivMMC is not enabled
        if (!divMmcEnabled && interface1Present)
        {
            interface1Enabled = true;
        }

        // If UART is present and ready, then enable
        if (uartPresent && !wifiNtpEnabled)
        {
            espUart.flush();
            uartEnabled = true;
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

    // Enable the ROM, if present
    delay(250);
    if (romArrayPresent != 0)
    {
        updateRomIndex(true);
        setState(STATE_ROM_ENABLE);
    } else {
        // Disable the soft ROM
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
        if (wifiNtpEnabled)
        {
            if (wifiNtp.onTick())
            {
                wifiNtpEnabled = false;
                rtcTeensy.setAscTime(wifiNtp.getAscTime(), wifiNtpTz);
                wifiNtpHasTime = true;

                // Enable the UART, now time is updated
                espUart.flush();
                uartEnabled = true;

                // Trigger NMI in menu to redraw
                if (menuPaged && !nmiPending)
                {
                    menuGenerate();
                    nmiPending = true;
                    digitalWriteFast(NMI_PIN, 1);
                }
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
                    buttonTrigExitCount = TRIGGER_DELAY_CNT;
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
                    buttonTrigExitCount = TRIGGER_DELAY_CNT;
                }
                break;
            default :
                break;
        }

        // Perform menu actions
        if (menuSelected)
        {
            menuSelected = false;
            if (menuPerformSelection(menuSelectedIndex))
            {
                // The menu needs the Spectrum in reset to access the SD card,
                // reload ROMs, update FW etc.
                setState(STATE_RESET_MENU);
            }
        }

        // Perform ZXC3 flash actions
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
                                zxC3EraseTrigState = TRIGGER_READY;
                                if (zxC3EraseBuffer.canRead())
                                {
                                    uint8_t sector = zxC3EraseBuffer.readRaw();
                                    if (sector < 0xFF)
                                    {
                                        memset((void*)&(divMmcExtRamArray[sector][1]),
                                            0xFF, (ROM_PAGE_SIZE - 1));
                                        *divMmcExtRamArray[sector] = 0xFF;
                                    } else {
                                        memset((void*)&(divMmcExtRamArray[0][1]),
                                            0xFF, ((ZXC3_PAGE_COUNT * RAM_PAGE_SIZE) - 1));
                                        *divMmcExtRamArray[0] = 0xFF;
                                    }
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
                            if (mdrEnabled)
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

    // Run HTTP server actions
    httpRunServer();

    // Run FDC actions
    dskController.onTick();

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
            uint32_t buttons = usbJoystick.getButtons();
            switch (usbJoystick.joystickType())
            {
                case JoystickController::PS4 :
                    break;
                case JoystickController::PS3 :
                    break;
                default :
                    // D-Pad
                    uint32_t dpadButtons = (buttons & 0xF00) >> 8;
                    if (dpadButtons != 0)
                    {
                        const static uint8_t dpadMap[16] = { 0x00,
                            0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E,
                            0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F };
                        data |= dpadMap[dpadButtons];
                    }

                    // A and X as Fire 1
                    if (buttons & 0x30)
                    {
                        data |= 0x10;
                    }

                    // B and Y as Fire 2
                    if (buttons & 0xc0)
                    {
                        data |= 0x20;
                    }
                    break;
            }
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

inline __attribute__((always_inline)) void writeRomData(uint16_t address)
{
    if ((romSelected == ROM_DIVMMC) && (address >= RAM_PAGE_SIZE))
    {
        // Tranfer DivMMC RAM data to the bus
        writeData(divMmcRamPtr[address & (RAM_PAGE_SIZE - 1)]);
    } else if (romEnabled)
    {
        // Transfer soft ROM data to the bus
        writeData(romPtr[address]);
    }
}

inline __attribute__((always_inline)) void writePagedRomData(uint16_t address)
{
    if (romEnabled)
    {
        // Transfer soft ROM data to the bus
        writeData(romPtr[address]);
    }
}

inline __attribute__((always_inline)) void writeDivMmcRomData(uint16_t address)
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

        // Perform NMI when not already handling previous NMI
        if (!isGlobalStateReset() && !menuPaged && !nmiPending && !mf128ActiveNMI)
        {
            nmiPending = true;
            digitalWriteFast(NMI_PIN, 1);
        }
    }
}

FASTRUN void isrWrEvent()
{
    if (globalState == STATE_ROM_ENABLE)
    {
        // Start of write access
        uint32_t gpioSix = (*(volatile uint32_t *)IMXRT_GPIO6_ADDRESS);
        if ((gpioSix & ROM_ADDRESS_MASK) == 0x00000000)
        {
            uint16_t address = decodeAddress(gpioSix);

            // Perform ZXC2 address based paging
            if (zxC2Present && !zxC2Lock &&
                (!zxC2ShadowRom || zxC2Paged) &&
                ((address & 0xffc0) == 0x3fc0))
            {
                if (zxC3Present)
                {
                    zxC3Write = ((address & 0x08) != 0);
                    zxC2BankPtr = ((address & 0x07) << 1);
                } else {
                    zxC2BankPtr = ((address & 0x0f) << 1);
                }
                zxC2Paged = ((address & 0x10) == 0);
                zxC2Lock = ((address & 0x20) != 0);
                updateRomIndex(true);
            }

            switch (romSelected)
            {
                case ROM_MF128 :
                    // Perform Multiface 128 RAM write
                    if (address >= RAM_PAGE_SIZE)
                    {
                        romPtr[address] = readData();
                    }
                    break;
                case ROM_DIVMMC :
                    // Perform DivMMC RAM write
                    if ((address >= RAM_PAGE_SIZE) &&
                        (!divMmcMapRam || divMmcConMem || !divMmcRamBankThree))
                    {
                        divMmcRamPtr[address & (RAM_PAGE_SIZE - 1)] = readData();
                    }
                    break;
                case ROM_ZXC2 :
                    if (zxC3Write)
                    {
                        uint8_t data = readData();
                        switch (zxC3FlashState)
                        {
                            case ZXC3_FLASH_UNLOCK :
                                if ((zxC2BankPtr == 0) && (address == 0x2AAA) && (data == 0x55))
                                {
                                    zxC3FlashState = ZXC3_FLASH_CMD;
                                } else {
                                    zxC3FlashState = ZXC3_FLASH_IDLE;
                                    zxC3FlashSetup = false;
                                }
                                break;
                            case ZXC3_FLASH_CMD :
                                zxC3FlashState = ZXC3_FLASH_IDLE;
                                switch (data)
                                {
                                    case 0x10 :
                                        // Chip erase
                                        if ((zxC2BankPtr == 2) && (address == 0x1555) &&
                                            zxC3FlashSetup)
                                        {
                                            for (uint8_t i = 0; i < ZXC3_PAGE_COUNT; ++i)
                                            {
                                                *divMmcExtRamArray[i] = 0x08;
                                            }
                                            zxC3EraseBuffer.write(0xFF);
                                            zxC3WriteTrigState = TRIGGER_ACTIVE;
                                        }
                                        zxC3FlashSetup = false;
                                        break;
                                    case 0x30 :
                                        // Sector erase
                                        if (zxC3FlashSetup)
                                        {
                                            *divMmcExtRamArray[zxC2BankPtr] = 0x08;
                                            zxC3EraseBuffer.write(zxC2BankPtr);
                                            zxC3WriteTrigState = TRIGGER_ACTIVE;
                                        }
                                        zxC3FlashSetup = false;
                                        break;
                                    case 0x80 :
                                        // Setup command
                                        if ((zxC2BankPtr == 2) && (address == 0x1555))
                                        {
                                            zxC3FlashSetup = true;
                                        }
                                        break;
                                    case 0xA0 :
                                        // Program byte
                                        if ((zxC2BankPtr == 2) && (address == 0x1555))
                                        {
                                            zxC3FlashState = ZXC3_FLASH_WRITE;
                                        }
                                        break;
                                    default :
                                        zxC3FlashSetup = false;
                                        break;
                                }
                                break;
                            case ZXC3_FLASH_WRITE :
                                // Program byte
                                romPtr[address] = data;
                                zxC3FlashState = ZXC3_FLASH_IDLE;
                                zxC3WriteTrigState = TRIGGER_ACTIVE;
                                break;
                            default :
                                if ((zxC2BankPtr == 2) && (address == 0x1555) && (data == 0xAA))
                                {
                                    zxC3FlashState = ZXC3_FLASH_UNLOCK;
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
                            zxC2BankPtr += 2;
                            if (zxC2BankPtr >= snaLoaderBanks)
                            {
                                zxC2BankPtr = 0;
                                snaLoaderBanks = 0;
                            }
                        } else {
                            snaLoaderPaged = false;
                        }
                        updateRomIndex(true);
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
                    if (rom1Present && (!rom23Present || isPort7F))
                    {
                        // Detect 0x7ffd write access for 128k ROMs
                        if ((data & 0x20) != 0)
                        {
                            rom1Present = false;
                        }
                        rom1Paged = ((data & 0x10) != 0);
                        mf128VideoRam = ((data & 0x08) != 0);
                        updateRomIndex(true);
                    }
                    if (isPort7F)
                    {
                        if (interface1Present && divMmcEnabled &&
                            divMmcPaged && ((data & 0x10) == 0x0))
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
                        if (rom1Present && rom23Present)
                        {
                            rom23Paged = ((data & 0x04) != 0);
                            updateRomIndex(true);
                        }
                        if (dskEnabled)
                        {
                            dskController.setMotor((data & 0x08) != 0);
                        }
                    }
                }
            } else {
                switch (port_)
                {
                    case 0x3f :
                        mf128ActiveNMI = false;
                        if (mf128Enabled)
                        {
                            mf128Enabled = false;
                            divMmcEnabled = divMmcPresent;
                            interface1Enabled = (interface1Present && !divMmcEnabled);
                        }
                        break;
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
                    case 0xbf :
                        mf128ActiveNMI = false;
                        break;
                    case 0xe3 :
                        if (isDivMmcSelected())
                        {
                            // DivMMC control
                            uint8_t data = readData();
                            if ((data & 0x80) != 0)
                            {
                                divMmcConMem = 1;
                                divMmcPaged = true;
                            } else {
                                divMmcConMem = 0;
                                if (!divMmcAutoMap)
                                {
                                    divMmcPaged = false;
                                }
                            }
                            if ((data & 0x40) != 0)
                            {
                                divMmcMapRam = true;
                            }

                            // DivMMC RAM banking
                            data &= (EXT_RAM_PAGE_COUNT + RAM_PAGE_COUNT - 1);
                            if (divMmcExtRamEnabled && (data >= RAM_PAGE_COUNT))
                            {
                                divMmcRamPtr = divMmcExtRamArray[(data - RAM_PAGE_COUNT)];
                                divMmcRamBankThree = false;
                            } else {
                                divMmcRamPtr = divMmcRamArray[data & (RAM_PAGE_COUNT - 1)];
                                divMmcRamBankThree = ((data == 0x03) ? 1 : 0);
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
                        if (menuPaged)
                        {
                            if (!menuSelected)
                            {
                                menuSelected = true;
                                menuSelectedIndex = readData();
                            }
                        } else if (isDivMmcSelected())
                        {
                            // DivMMC write
                            writeSdSpiWriteBuffer(SD_SPI_WRITE, readData());
                        }
                        break;
                }
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
    } else if (globalState == STATE_ROM_ENABLE)
    {
        if ((gpioSix & ROM_ADDRESS_MASK) == 0x00000000)
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
                if (zxC2Present && !zxC2Lock &&
                    (!zxC2ShadowRom || zxC2Paged) &&
                    ((address & 0xffc0) == 0x3fc0))
                {
                    if (zxC3Present)
                    {
                        zxC3Write = ((address & 0x08) != 0);
                        zxC2BankPtr = ((address & 0x07) << 1);
                    } else {
                        zxC2BankPtr = ((address & 0x0f) << 1);
                    }
                    zxC2Paged = ((address & 0x10) == 0);
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
                    // Send the NMI to the Multiface 128
                    if (mf128Present && nmiPending && !mf128ActiveNMI &&
                        ((romArraySelected & PAGE_BANK_MF128_IF1) != 0))
                    {
                        mf128ActiveNMI = true;
                        mf128Paged = true;
                        mf128Enabled = true;
                        divMmcEnabled = false;
                        interface1Enabled = interface1Present;
                        updateRomIndex(true);
                    }

                    // Write ROM data to bus
                    writeRomData(address);

                    // Send the NMI to the DivMMC, if not Multiface 128
                    if (divMmcEnabled && !mf128ActiveNMI &&
                        ((romArraySelected & PAGE_BANK_DIVMMC) != 0))
                    {
                        divMmcPaged = true;
                        divMmcAutoMap = true;
                        updateRomIndex(false);
                    }

                    // Release NMI on entry to interrupt handler
                    nmiPending = false;
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
                                mf128Paged = true;
                                updateRomIndex(true);
                            }

                            if (divMmcEnabled && divMmcRomEnabled && !mf128Paged)
                            {
                                // Detect M1 cycle for DivMMC paging
                                if ((address & 0xff00) == 0x3d00)
                                {
                                    divMmcPaged = true;
                                    divMmcAutoMap = true;
                                    updateRomIndex(true);
                                }

                                // Write ROM data to bus
                                writePagedRomData(address);

                                // Detect post-M1 cycle for DivMMC paging
                                if ((address == 0x00) || (address == 0x08) ||
                                    (address == 0x38) || (address == 0x4c6) ||
                                    (address == 0x562))
                                {
                                    divMmcPaged = true;
                                    divMmcAutoMap = true;
                                    updateRomIndex(false);
                                }
                            } else {
                                // Write ROM data to bus
                                writePagedRomData(address);

                                // Detect M1 cycle for Interface 1 paging
                                if ((interface1Enabled || zxC2ShadowRom) &&
                                    ((address == 0x08) || (address == 0x1708)))
                                {
                                    // M1 cycle for Interface 1 paging
                                    if (zxC2ShadowRom)
                                    {
                                        zxC2Paged = true;
                                    } else {
                                        interface1Paged = true;
                                    }
                                    updateRomIndex(false);
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
                                interface1Paged = false;
                                updateRomIndex(false);
                            }
                            break;
                        case ROM_MF128 :
                            // Detect M1 cycle for Multiface 128 paging
                            if (mf128ActiveNMI && (address == 0x67))
                            {
                                mf128Paged = true;
                                updateRomIndex(true);
                            }

                            // Write ROM data to bus
                            writePagedRomData(address);

                            // Detect M1 cycle for Interface 1 paging
                            if (interface1Enabled &&
                                ((address == 0x08) || (address == 0x1708)))
                            {
                                // M1 cycle for Interface 1 paging
                                interface1Paged = true;
                                updateRomIndex(false);
                            }
                            break;
                        case ROM_DIVMMC :
                            // Write ROM data to bus
                            writeDivMmcRomData(address);

                            // Detect post-M1 cycle for DivMMC paging
                            // NOTE: Avoid paging out on MAPRAM to allow DivMMC
                            // loaded ROM images to behave correctly
                            if (!divMmcMapRam && ((address & 0xfff8) == 0x1ff8))
                            {
                                divMmcPaged = false;
                                divMmcAutoMap = false;
                                updateRomIndex(false);

                                // Disable the DivMMC, and enable the Interface 1
                                if (divMmcToggle)
                                {
                                    divMmcToggle = false;
                                    divMmcEnabled = false;
                                    interface1Enabled = interface1Present;
                                }
                            }
                            break;
                        case ROM_ZXC2 :
                            // Write ROM data to bus
                            writePagedRomData(address);

                            // Detect post-M1 cycle for Interface 1 paging
                            if (zxC2ShadowRom && (address == 0x700))
                            {
                                zxC2Paged = false;
                                updateRomIndex(false);
                            }
                            break;
                        default :
                            // Write ROM data to bus
                            writePagedRomData(address);
                            break;
                    }
                }

                // Detect snapshot loader paging
                if (snaLoaderPaged && (address == 0x3FFF))
                {
                    if (snaLoaderBanks > 0)
                    {
                        zxC2BankPtr += 2;
                        if (zxC2BankPtr >= snaLoaderBanks)
                        {
                            zxC2BankPtr = 0;
                            snaLoaderBanks = 0;
                        }
                    } else {
                        snaLoaderPaged = false;
                    }
                    updateRomIndex(true);
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
                                    writeData(espUart.getStatusByte());
                                    break;
                                case 0x14 :
                                    writeData(espUart.hasReadData() ? espUart.readData() : 0x00);
                                    break;
                            }
                        }
                    }
                    break;
                case 0x3f :
                    if (mf128Paged)
                    {
                        mf128Paged = false;
                        updateRomIndex(true);
                    }
                    if (mf128Enabled)
                    {
                        writeData(mf128VideoRam ? 0x80 : 0x00);
                    }
                    break;
                case 0xbf :
                    if (mf128Enabled)
                    {
                        if (!mf128Paged)
                        {
                            mf128Paged = true;
                            updateRomIndex(true);
                        }
                        writeData(mf128VideoRam ? 0x80 : 0x00);
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
                    if (isDivMmcSelected())
                    {
                        // Transfer SD SPI read data to bus
                        if (sdSpiReadBuffer.canRead())
                        {
                            writeData(sdSpiReadBuffer.readRaw());
                            if (!sdSpiReadBuffer.canRead())
                            {
                                writeSdSpiWriteBuffer(SD_SPI_READ, 0xff);
                            }
                        } else {
                            writeData(0xff);
                            writeSdSpiWriteBuffer(SD_SPI_READ, 0xff);
                        }
                    }
                    break;
                case 0xfd :
                    if (dskEnabled)
                    {
                        switch (decodeHighAddress(gpioSix))
                        {
                            case 0x2f :
                                writeData(dskController.getStatusByte());
                                break;
                            case 0x3f :
                                writeData(dskController.readData());
                                break;
                        }
                    }
                case 0xfe :
                    if (tzxEnabled && tzxPlayer.isTapePlaying())
                    {
                        writeData(tzxPlayer.getTapeByte());
                    }
                    break;
            }
        }
    }
}
