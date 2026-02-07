
// Must be set in SdFat/src/SdFatConfig.h
#define SPI_DRIVER_SELECT 2

#define ZXTEENSY_VERSION "20260207"
#define ENABLE_BUILTIN_ROM_IF1
//#define DEBUG_OUTPUT

#include <SD.h>
#include <SdFat.h>
#include "if1-2_rom.h"
#include "RingBuffer.h"
#include "SdSpiZXTeensy.h"
#include "UartZXTeensy.h"

// Run the Teensy 4.1 with slight overclock at 816MHz
// Run the SD card at ~7MHz (at 816MHz, SD_TICK_CYCCNT = 58)
#define TEENSY_CLK_FREQ 816000000ULL
#define SD_CLK_FREQ 7000000ULL
#define SD_TICK_CYCCNT ((TEENSY_CLK_FREQ / SD_CLK_FREQ) / 2)
#define FAST_SD_CLK_FREQ 24000000ULL
#define FAST_SD_TICK_CYCCNT ((TEENSY_CLK_FREQ / FAST_SD_CLK_FREQ) / 2)

// Allow ~350ms for reset/button to debounce (at 816MHz, TRIGGER_DELAY_CNT = 0x4B22E9)
#define TRIGGER_DELAY_MS 350
#define TRIGGER_DELAY_CNT ((TRIGGER_DELAY_MS * TEENSY_CLK_FREQ) / (SD_TICK_CYCCNT * 1000))

extern "C" uint32_t set_arm_clock(uint32_t frequency);

const char PROGMEM VERSION_STR[9] = ZXTEENSY_VERSION;

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
    ROM_ROM0,
    ROM_ROM1,
    ROM_ROM2,
    ROM_ROM3,
    ROM_IF1,
    ROM_DIVMMC,
    ROM_MF128,
    // "ROMs" below use DivMMC RAM
    ROM_ZXC2,   // ROM_PAGE_COUNT
    ROM_MENU
} rom_index_t;

typedef enum {
    TYPE_ROM,
    TYPE_ZXC2,
    TYPE_IF2
} rom_type_t;

// I/O pin assignments
const uint8_t LED_PIN = 13;
const uint8_t DATA_DIS_PIN = 29;
const uint8_t DATA_OUT_PIN = 36;  // 1 = output, 0 = input
const uint8_t RESET_PIN = 31;
const uint8_t RESET_IN_PIN = 2;
const uint8_t BUTTON_PIN = 33;
const uint8_t ROMCS_PIN = 37;
const uint8_t ROMCS_IN_PIN = 3;
const uint8_t IF1_DIS_PIN = 5;
const uint8_t NMI_PIN = 30;
const uint8_t RD_PIN = 1;
const uint8_t WR_PIN = 0;
const uint8_t MREQ_PIN = 24;
const uint8_t IOREQ_PIN = 25;
const uint8_t M1_PIN = 4;
const uint8_t ESP_ENABLE = 28;

const uint8_t INPUT_PINS[] = {
    RESET_IN_PIN, MREQ_PIN, RD_PIN, IOREQ_PIN, WR_PIN, M1_PIN, ROMCS_IN_PIN,
    19, 18, 14, 15, 40, 41, 17, 16, 22, 23, 20, 21, 38, 39, 26, 27, // Address bus
    BUTTON_PIN, ESP_ENABLE
};

const uint32_t RD_PIN_BITMASK = CORE_PIN1_BITMASK;
const uint32_t M1_PIN_BITMASK = CORE_PIN4_BITMASK;
const uint32_t IOREQ_PIN_BITMASK = CORE_PIN25_BITMASK;
const uint32_t MREQ_PIN_BITMASK = CORE_PIN24_BITMASK;
const uint32_t A15_PIN_BITMASK = CORE_PIN27_BITMASK;
const uint32_t A14_PIN_BITMASK = CORE_PIN26_BITMASK;
const uint32_t A13_PIN_BITMASK = CORE_PIN39_BITMASK;
const uint32_t A12_PIN_BITMASK = CORE_PIN38_BITMASK;
const uint32_t ROMCS_IN_PIN_BITMASK = CORE_PIN3_BITMASK;

const uint8_t DATA_PINS[] = { 6, 7, 8, 9, 10, 11, 12, 32 };

const uint32_t GPIO7_DATA_MASK = (CORE_PIN6_BITMASK | CORE_PIN7_BITMASK |
    CORE_PIN8_BITMASK | CORE_PIN9_BITMASK |
    CORE_PIN10_BITMASK | CORE_PIN11_BITMASK |
    CORE_PIN12_BITMASK | CORE_PIN32_BITMASK);

const uint32_t DATA_OUT_PIN_BITMASK = CORE_PIN36_BITMASK;

const uint8_t OUTPUT_PINS[] = {
    LED_PIN, ROMCS_PIN, NMI_PIN, IF1_DIS_PIN
};

const uint8_t SD_CS_PIN = 46;
const uint8_t SD_OUT_PIN = 45;
const uint8_t SD_CLK_PIN = 44;
const uint8_t SD_IN_PIN = 43;

// Mask for A15, A14, ^RD, and ^MREQ
const uint32_t ROM_READ_MASK = (A15_PIN_BITMASK | A14_PIN_BITMASK |
    RD_PIN_BITMASK | MREQ_PIN_BITMASK);

// Mask for ^RD and ^IOREQ
const uint32_t IO_READ_MASK = (IOREQ_PIN_BITMASK | RD_PIN_BITMASK);

// Mask for A15, A14, A13 and ^MREQ
const uint32_t DIVMMC_RAM_WRITE_MASK = (A15_PIN_BITMASK | A14_PIN_BITMASK |
    A13_PIN_BITMASK | MREQ_PIN_BITMASK);

// Number of SD retries
const uint8_t NUM_SD_RETRIES = 5;

// Global state
volatile bool bootIntoMenu = false;
volatile bool afterFirstReset = false;
volatile bool isDeviceDisabled = false;
volatile bool sdCardPresent = false;
volatile run_state_t globalState = STATE_RESET;
volatile bool busRdActive = false;
volatile bool nmiPending = false;

// Reset and NMI debouncing
volatile trigger_state_t resetTrigState = TRIGGER_ACTIVE;
volatile uint32_t resetTrigExitCount = TRIGGER_DELAY_CNT;
volatile trigger_state_t buttonTrigState = TRIGGER_READY;
volatile uint32_t buttonTrigExitCount = 0;

// ROM banking
const rom_index_t ROM_PAGE_COUNT = ROM_ZXC2;
const uint16_t ROM_PAGE_SIZE = 0x4000;
volatile rom_index_t romSelected = ROM_ROM0;
volatile bank_select_t romArraySelected = BANK_ROM0;
volatile uint8_t romArray[ROM_PAGE_COUNT][ROM_PAGE_SIZE] __attribute__((aligned(16)));
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
const uint16_t RAM_PAGE_COUNT = 16;
const uint16_t EXT_RAM_PAGE_COUNT = 48;
const uint16_t RAM_PAGE_SIZE = 0x2000;
volatile uint8_t divMmcRamArray[RAM_PAGE_COUNT][RAM_PAGE_SIZE] __attribute__((aligned(16)));
volatile DMAMEM uint8_t divMmcExtRamArray[EXT_RAM_PAGE_COUNT][RAM_PAGE_SIZE] __attribute__((aligned(16)));
volatile bool divMmcPresent = false;
volatile bool divMmcPaged = false;
volatile bool divMmcAutoMap = false;
volatile bool divMmcConMem = false;
volatile bool divMmcMapRam = false;
volatile bool divMmcRamBankThree = false;
volatile uint8_t* divMmcRamPtr;
volatile bool divMmcRemoval = false;
const uint16_t PAGE_BANK_DIVMMC = (BANK_ROM0 | BANK_ROM1 | BANK_ROM3);

// Multiface 128
volatile bool mf128Present = false;
volatile bool mf128Paged = false;
volatile bool mf128VideoRam = false;
volatile bool mf128ActiveNMI = false;

// Interface 1
volatile bool interface1Present = false;
volatile bool interface1Paged = false;
volatile bool interface1Removed = false;
const uint16_t PAGE_BANK_MF128_IF1 = (BANK_ROM0 | BANK_ROM1 | BANK_ROM3 | BANK_MF128);

// ZXC2 cartridge (reuses divMmcRamArray)
volatile bool zxC2Present = false;
volatile bool zxC2Paged = false;
volatile bool zxC2Lock = false;
volatile uint8_t zxC2BankPtr = 0x00;

// Boot menu ROM
volatile bool menuPaged = false;
volatile bool menuSelected = false;
volatile uint8_t menuSelectedIndex = 0;

// DivMMC SPI/SD
SdSpiZXTeensy divMmcSpi(FAST_SD_TICK_CYCCNT);

// MB03+ UART
volatile bool uartPresent = false;
UartZXTeensy espUart;

// SPI and UART tick cycle counter
volatile uint32_t globalCycleCount;

#ifdef DEBUG_OUTPUT

// Debug data buffer
const uint16_t DEBUG_BUFFER_SIZE = 128;
RingBuffer<DEBUG_BUFFER_SIZE> debugBuffer;

inline __attribute__((always_inline)) void writeDebugData(uint8_t data)
{
    debugBuffer.write(data);
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

#endif

inline __attribute__((always_inline)) void writeData(uint8_t data)
{
    // Output D[7:0] to GPIO2/7
    uint32_t gpioSeven = ((data & 0x07) | ((data & 0x38) << 7) | ((data & 0xc0) << 10));
    CORE_PIN34_PORTSET = DATA_OUT_PIN_BITMASK;
    CORE_PIN10_DDRREG |= GPIO7_DATA_MASK;
    CORE_PIN10_PORTSET = gpioSeven & GPIO7_DATA_MASK;
    CORE_PIN10_PORTCLEAR = (~gpioSeven) & GPIO7_DATA_MASK;
}

inline __attribute__((always_inline)) uint8_t readData()
{
    // Decode D[7:0] from GPIO2/7
    uint32_t gpioSeven = (*(volatile uint32_t *)IMXRT_GPIO7_ADDRESS);
    uint32_t data = ((gpioSeven & 0x07) | ((gpioSeven & 0x1c00) >> 7) | ((gpioSeven & 0x30000) >> 10));
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

inline __attribute__((always_inline)) void performOnClock()
{
    uint32_t cycle_ = ARM_DWT_CYCCNT;
    if ((cycle_ - globalCycleCount) >= SD_TICK_CYCCNT)
    {
        globalCycleCount = cycle_;

        // Perform SPI and UART on clock ticks
        divMmcSpi.onTick();
        espUart.onTick();

        // Debounce the reset detection
        switch (resetTrigState)
        {
            case TRIGGER_HOLD :
                if (digitalReadFast(RESET_IN_PIN))
                {
                    resetTrigState = TRIGGER_DELAY;
                    resetTrigExitCount = TRIGGER_DELAY_CNT;
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
                    resetTrigExitCount = TRIGGER_DELAY_CNT;
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
    }
}

inline __attribute__((always_inline)) bool isGlobalStateReset()
{
    return ((globalState & 0x02) != 0);
}

inline __attribute__((always_inline)) bool isDivMmcSelected()
{
    return (divMmcPresent && ((romArraySelected & (BANK_MF128 | BANK_IF1)) == 0));
}

inline __attribute__((always_inline)) void divMmcUpdateInterfaceOne()
{
    if ((!interface1Present && !interface1Removed) || isDivMmcSelected())
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
            digitalWriteFast(DATA_DIS_PIN, 0);
            digitalWriteFast(RESET_PIN, 0);
            digitalWriteFast(LED_PIN, 1);
            break;
        case STATE_ROM_DISABLE :
            resetTrigState = TRIGGER_HOLD;
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
                    romPtr = romArray[ROM_DIVMMC];
                }
                break;
            case ROM_ZXC2 :
                romPtr = divMmcExtRamArray[zxC2BankPtr];
                break;
            case ROM_MENU :
                romPtr = divMmcRamArray[0];
                break;
            default :
                romPtr = romArray[romSelected];
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

    // Configure SD card I/Os
    pinMode(SD_IN_PIN, INPUT_PULLUP); // 43
    pinMode(42, INPUT);
    pinMode(47, INPUT);
    pinMode(SD_CS_PIN, OUTPUT);

    // Set the SD CS pin to deselect card
    digitalWriteFast(SD_CS_PIN, 1);

    // Configure SD CLK and MISO to high speed
    pinMode(SD_OUT_PIN, OUTPUT); // 45
    CORE_PIN45_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);
    pinMode(SD_CLK_PIN, OUTPUT); // 44
    CORE_PIN44_PADCONFIG |= IOMUXC_PAD_SRE | IOMUXC_PAD_SPEED(3);

    // Force the spectrum into reset
    globalCycleCount = ARM_DWT_CYCCNT;
    setState(STATE_RESET);

    // Configure UART, and USB serial debug
#ifdef DEBUG_OUTPUT
    Serial.begin(115200);
#endif

    // Setup RD, WR, ROMCS, reset and button ISRs
    attachInterrupt(digitalPinToInterrupt(RD_PIN), isrRdEvent, CHANGE);
    attachInterrupt(digitalPinToInterrupt(WR_PIN), isrWrEvent, FALLING);
    attachInterrupt(digitalPinToInterrupt(ROMCS_IN_PIN), isrRdEvent, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RESET_IN_PIN), isrPinReset, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isrPinButton, FALLING);

    // Setup UART ISRs
    attachInterruptVector(IRQ_LPUART5, isrUartEvent);
    NVIC_ENABLE_IRQ(IRQ_LPUART5);

    // TODO: set HW ints as high priority, otherwise ethernet int timer causes misses
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 16);
    NVIC_SET_PRIORITY(IRQ_LPUART5, 64);
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

void loadSpectrumRomFile(File RomFile)
{
    // Reset the Spectrum ROM and ZXC2 state
    romArrayPresent &= ~(BANK_ROM0 | BANK_ROM1 | BANK_ROM2 | BANK_ROM3 | BANK_RAM);
    rom1Present = false;
    rom23Present = false;
    zxC2Present = false;

    // Attempt to load four 16KB ROM banks
    size_t count = RomFile.readBytes((char *)romArray[ROM_ROM0], ROM_PAGE_SIZE);
    if (count > 0)
    {
        romArrayPresent |= BANK_ROM0;
        if (count >= ROM_PAGE_SIZE)
        {
            count = RomFile.readBytes((char *)romArray[ROM_ROM1], ROM_PAGE_SIZE);
            if (count > 0)
            {
                rom1Present = true;
                romArrayPresent |= BANK_ROM1;
                if (count >= ROM_PAGE_SIZE)
                {
                    count = RomFile.readBytes((char *)romArray[ROM_ROM2], ROM_PAGE_SIZE);
                    if (count > 0)
                    {
                        rom23Present = true;
                        romArrayPresent |= (BANK_ROM2 | BANK_ROM3);
                        count = RomFile.readBytes((char *)romArray[ROM_ROM3], ROM_PAGE_SIZE);
                    }
                }
            }
        }
    }
}

void loadZXC2RomFile(File RomFile)
{
    // Reset the ZXC2 state only, as can page into the Spectrum ROM
    romArrayPresent &= ~(BANK_RAM);
    zxC2Present = false;

    // The ZXC2 cartridge is loaded into the DivMMC RAM area
    size_t count = RomFile.readBytes((char *)divMmcExtRamArray[0], RAM_PAGE_SIZE);
    if (count > 0)
    {
        zxC2Present = true;
        romArrayPresent |= BANK_RAM;
        for (uint8_t i_ = 1; i_ < EXT_RAM_PAGE_COUNT; ++i_)
        {
            size_t blk_count_ = RomFile.readBytes((char *)divMmcExtRamArray[i_], RAM_PAGE_SIZE);
            count += blk_count_;
            if (blk_count_ < RAM_PAGE_SIZE)
            {
                break;
            }
        }
    }
}

void loadForegroundRom()
{
    rom_type_t romType;
    File RomFile = menuGetRomFile(&romType);
    if (RomFile)
    {
        switch (romType)
        {
            case TYPE_IF2 :
                zxC2Lock = true;
            case TYPE_ZXC2 :
                loadZXC2RomFile(RomFile);
                break;
            default :
                loadSpectrumRomFile(RomFile);
                break;
        }
        RomFile.close();
    }
}

void handleStateResetEntry()
{
#ifdef DEBUG_OUTPUT
    if (afterFirstReset)
    {
        // Dump the debug buffer
        while (hasDebugData())
        {
            while ( (uint) Serial.availableForWrite() <  0x400);
            Serial.printf("%02x%02x\n", readDebugData(), readDebugData());
        }
        Serial.printf("END\n");

        // Dump the RAM banks
        for (uint8_t i_ = 0; i_ < RAM_PAGE_COUNT; ++i_)
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
        }
    }
#endif

    // Detect button being pressed for menu ROM
    bool isButtonHeld = false;
    if (!digitalReadFast(BUTTON_PIN))
    {
        // Re-initialise into the menu ROM
        isButtonHeld = true;
        afterFirstReset = false;
        isDeviceDisabled = false;

        // Close the SD card to reload the system
        if (sdCardPresent)
        {
            SD.sdfs.end();
            sdCardPresent = false;
        }

        // Wait for button release
        while (!digitalReadFast(BUTTON_PIN))
        {
            delay(75);
        }
    }

    // Perform first reset initialisation
    if (!afterFirstReset)
    {
        // Initialise the RAM banks
        for (uint8_t i_ = 0; i_ < RAM_PAGE_COUNT; ++i_)
        {
            for (uint16_t j_ = 0; j_ < RAM_PAGE_SIZE; ++j_)
            {
                divMmcRamArray[i_][j_] = 0xff;
            }
        }
        for (uint8_t i_ = 0; i_ < EXT_RAM_PAGE_COUNT; ++i_)
        {
            for (uint16_t j_ = 0; j_ < RAM_PAGE_SIZE; ++j_)
            {
                divMmcExtRamArray[i_][j_] = 0xff;
            }
        }
        for (uint16_t j_ = RAM_PAGE_SIZE; j_ < ROM_PAGE_SIZE; ++j_)
        {
            romArray[ROM_DIVMMC][j_] = 0xff;
            romArray[ROM_MF128][j_] = 0xff;
        }

        // Reset the soft ROM detection state
        romArrayPresent = 0;
        rom1Present = false;
        rom23Present = false;
        interface1Present = false;
        divMmcPresent = false;
        mf128Present = false;
        zxC2Present = false;
    }

    // Reset the UART state, and clear buffers of any idle data
    espUart.end();

    // Initialise the SD card
    delay(250);
    if (!isDeviceDisabled && !sdCardPresent)
    {
        // Wait for any previous SD accesses to finish, and clear buffers of any
        // idle state
        divMmcSpi.flush();

        // Detect the SD card
        pinMode(SD_CS_PIN, INPUT_PULLDOWN);
        if (digitalReadFast(SD_CS_PIN))
        {
            uint8_t retries_ = 0;
            sdCardPresent = true;
            pinMode(SD_CS_PIN, OUTPUT);
            digitalWriteFast(SD_CS_PIN, 1);
            digitalWriteFast(SD_OUT_PIN, 1);
            digitalWriteFast(SD_CLK_PIN, 0);

            // Attempt to initialise the SD card
            while (!SD.sdfs.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI,
                SD_SCK_MHZ(1), &divMmcSpi)))
            {
                ++retries_;
                delay(5);
                if (retries_ > NUM_SD_RETRIES)
                {
                    sdCardPresent = false;
                    break;
                }
            }
        }
    }

    // Load the built-in Interface 1 soft ROM
#ifdef ENABLE_BUILTIN_ROM_IF1
    memcpy((void *)romArray[ROM_IF1], BUILTIN_ROM_IF1, BUILTIN_ROM_IF1_SIZE);
    romArrayPresent |= BANK_IF1;
#endif

    // Load ROMs from the SD card
    if (sdCardPresent)
    {
        // Load device ROMs
        if (!afterFirstReset)
        {
            // Load DivMMC Esxdos ROM
            if (loadRomImage("esxmmc.bin", (char *)romArray[ROM_DIVMMC], RAM_PAGE_SIZE) > 0)
            {
                romArrayPresent |= BANK_DIVMMC;
            }

            // Load Multiface 128 ROM
            if (loadRomImage("mf128.rom", (char *)romArray[ROM_MF128], RAM_PAGE_SIZE) > 0)
            {
                romArrayPresent |= BANK_MF128;
            }

            // Load Interface 1 ROM
            if (loadRomImage("if1.rom", (char *)romArray[ROM_IF1], ROM_PAGE_SIZE) > 0)
            {
                romArrayPresent |= BANK_IF1;
            }
        }

        // Load configuration
        menuLoadConfiguration();

        // Load foreground ROM
        loadForegroundRom();

        // Load menu ROM into the DivMMC RAM area
        if ((isButtonHeld || (!afterFirstReset && bootIntoMenu)) &&
            !digitalReadFast(ROMCS_IN_PIN) &&
            (loadRomImage("menu.rom", (char *)divMmcRamArray[0], RAM_PAGE_SIZE) > 0))
        {
            menuPaged = true;
            romArrayPresent |= BANK_RAM;
        }
    } else if (!isButtonHeld)
    {
        // Button without SD card disables the built-in Interface 1 soft ROM
        interface1Present = true;
    }

    // First reset completed
    afterFirstReset = true;
}

void handleStateResetMenu()
{
    // Perform the menu action
    menuPerformAction();

    // Close the SD card to reload the system
    SD.sdfs.end();
    sdCardPresent = false;

    // Wait for any previous SD accesses to finish
    divMmcSpi.flush();

    // Perform a full reset
    handleStateResetEntry();
}

void handleStateReset()
{
    // Reset the banking state
    menuPaged = false;
    menuSelected = false;
    rom1Paged = false;
    rom23Paged = false;
    interface1Paged = false;
    interface1Removed = false;
    divMmcRemoval = false;
    divMmcPaged = false;
    divMmcConMem = false;
    divMmcAutoMap = false;
    divMmcMapRam = false;
    divMmcRamPtr = divMmcRamArray[0];
    mf128Paged = false;
    mf128VideoRam = false;
    mf128ActiveNMI = false;
    zxC2Paged = false;
    zxC2Lock = false;
    zxC2BankPtr = 0x00;
    romSelected = ROM_ROM0;
    romArraySelected = BANK_ROM0;

    // Clear any pending NMI
    nmiPending = false;
    digitalWriteFast(NMI_PIN, 0);

    // Blink the LED
    delay(150);
    digitalWriteFast(LED_PIN, 1);

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

    // Populate the menu when active
    if (menuPaged)
    {
        generateMenu(divMmcRamArray[0]);
    } else {
        // If ZXC2 cartridge is present, then page in immediately
        if (zxC2Present)
        {
            zxC2Paged = true;
        }

        // If DivMMC is present, then disable Interface 1
        if (divMmcPresent)
        {
            // The Interface 1 can be enabled by switching back
            // into 128k mode (".128")
            if (interface1Present)
            {
                interface1Removed = true;
                interface1Present = false;
            }

            // Close the SD card to hand to the DivMMC
            SD.sdfs.end();
            sdCardPresent = false;

            // Wait for any previous SD accesses to finish
            divMmcSpi.flush();
        }

        // If UART is present, then enable Serial8
        if (uartPresent)
        {
            espUart.begin(0);
        }
    }

    // Enable the ROM, if present
    delay(100);
    if (romArrayPresent != 0)
    {
        updateRomIndex(true);
        setState(STATE_ROM_ENABLE);
    } else {
        // Disable the soft ROM
        setState(STATE_ROM_DISABLE);
    }
}

void loop()
{
    // Detect reset entry, and perform actions in reset
    if (isGlobalStateReset())
    {
        handleStateReset();
    }

    // Run actions (eg. SD SPI) on regular ticks
    performOnClock();
}

inline __attribute__((always_inline)) void writeRomData(uint16_t address)
{
    if (digitalReadFast(ROMCS_IN_PIN))
    {
        // External ROM is active late
        disableData();
        busRdActive = false;
    } else if ((romSelected == ROM_DIVMMC) && (address >= RAM_PAGE_SIZE))
    {
        // Tranfer DivMMC RAM data to the bus
        writeData(divMmcRamPtr[address & (RAM_PAGE_SIZE - 1)]);
    } else if (romEnabled)
    {
        // Transfer soft ROM data to the bus
        writeData(romPtr[address]);
    }
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

FASTRUN void isrUartEvent()
{
    espUart.isrUartEvent();
}

FASTRUN void isrWrEvent()
{
    if (globalState == STATE_ROM_ENABLE)
    {
        // Start of write access
        uint32_t gpioSix = (*(volatile uint32_t *)IMXRT_GPIO6_ADDRESS);
        if ((gpioSix & DIVMMC_RAM_WRITE_MASK) == A13_PIN_BITMASK)
        {
            if (romSelected == ROM_MF128)
            {
                // Perform Multiface 128 RAM write
                uint8_t data = readData();
                uint16_t address = (0x2000 | decodeRamAddress(gpioSix));
                romPtr[address] = data;
            } else if ((romSelected == ROM_DIVMMC) &&
                (!divMmcMapRam || divMmcConMem || !divMmcRamBankThree))
            {
                // Perform DivMMC RAM write
                uint8_t data = readData();
                uint16_t address = decodeRamAddress(gpioSix);
                divMmcRamPtr[address] = data;
            }
        } else if ((gpioSix & IOREQ_PIN_BITMASK) == 0x00000000)
        {
            // Perform I/O write access
            uint8_t port_ = decodeLowAddress(gpioSix);
            if ((port_ & 0x02) == 0)
            {
                if ((gpioSix & A15_PIN_BITMASK) == 0x00000000)
                {
                    // Perform I/O 0x1ffd or 0x7ffd write access
                    bool isPort7F = ((gpioSix & A14_PIN_BITMASK) != 0x0);
                    uint8_t data = readData();
                    if (rom1Present)
                    {
                        if (!rom23Present || isPort7F)
                        {
                            // Detect 0x7ffd write access for 128k ROMs
                            if (mf128Present)
                            {
                                mf128VideoRam = ((data & 0x08) != 0);
                            }
                            if ((data & 0x20) != 0)
                            {
                                rom1Present = false;
                            }
                            rom1Paged = ((data & 0x10) != 0);
                        }
                        if (rom23Present && !isPort7F &&
                            ((gpioSix & A13_PIN_BITMASK) == 0x0) &&
                            ((gpioSix & A12_PIN_BITMASK) != 0x0))
                        {
                            // Detect 0x1ffd write access for +3 ROMs
                            rom23Paged = ((data & 0x04) != 0);
                        }
                        updateRomIndex(true);
                    }

                    // Detect 0x7ffd write access to disable DivMMC
                    if (isPort7F && interface1Removed &&
                        divMmcPaged && ((data & 0x10) == 0x0))
                    {
                        interface1Removed = false;
                        divMmcRemoval = true;
                    }
                }
            } else if (mf128ActiveNMI && ((port_ == 0x3f) || (port_ == 0xbf)))
            {
                mf128ActiveNMI = false;
                updateRomIndex(true);
            } else if (menuPaged && (port_ == 0xeb))
            {
                if (!menuSelected)
                {
                    menuSelected = true;
                    menuSelectedIndex = readData();
                }
            } else if (uartPresent && (port_ == 0x3b))
            {
                switch (decodeHighAddress(gpioSix))
                {
                    case 0x13 :
                        espUart.writeData(UartZXTeensy::UART_WRITE, readData());
                        break;
                    case 0x14 :
                        espUart.writeData(UartZXTeensy::UART_SET_BAUD, readData());
                        break;
                }
            } else if (isDivMmcSelected())
            {
                switch (port_)
                {
                    case 0xe7 : // DivMMC card select
                        if ((readData() & 0x01) != 0)
                        {
                            divMmcSpi.writeData(SdSpiZXTeensy::SD_SPI_DISABLE, 0xff);
                            digitalWriteFast(LED_PIN, 1);
                        } else {
                            divMmcSpi.writeData(SdSpiZXTeensy::SD_SPI_ENABLE, 0xff);
                            digitalWriteFast(LED_PIN, 0);
                        }
                        break;
                    case 0xeb : // DivMMC write
                        divMmcSpi.writeData(SdSpiZXTeensy::SD_SPI_WRITE, readData());
                        break;
                    case 0xe3 : // DivMMC control
                        {
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
                            if (!zxC2Present && (data >= RAM_PAGE_COUNT))
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
        if ((gpioSix & ROM_READ_MASK) == 0x00000000)
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
                    ((address & 0xffc0) == 0x3fc0))
                {
                    zxC2BankPtr = ((address & 0x0f) << 1);
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
                    if (nmiPending)
                    {
                        // Send the NMI to the Multiface 128
                        if (mf128Present && !mf128ActiveNMI &&
                            ((romArraySelected & PAGE_BANK_MF128_IF1) != 0))
                        {
                            mf128ActiveNMI = true;
                            mf128Paged = true;
                            updateRomIndex(true);
                        }

                        // Write ROM data to bus
                        writeRomData(address);

                        // Send the NMI to the DivMMC, if not Multiface 128
                        if (divMmcPresent && !mf128ActiveNMI)
                        {
                            divMmcPaged = true;
                            divMmcAutoMap = true;
                            updateRomIndex(false);
                        }

                        // Release NMI on entry to interrupt handler
                        nmiPending = false;
                        digitalWriteFast(NMI_PIN, 0);
                    } else {
                        // No pending NMI - write ROM data to bus
                        writeRomData(address);
                    }
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

                            if (divMmcPresent && !mf128Paged)
                            {
                                // Detect M1 cycle for DivMMC paging
                                if ((address & 0xff00) == 0x3d00)
                                {
                                    divMmcPaged = true;
                                    divMmcAutoMap = true;
                                    updateRomIndex(true);
                                }

                                // Write ROM data to bus
                                writeRomData(address);

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
                                writeRomData(address);

                                // Detect M1 cycle for Interface 1 paging
                                if (interface1Present &&
                                    ((address == 0x08) || (address == 0x1708)))
                                {
                                    // M1 cycle for Interface 1 paging
                                    interface1Paged = true;
                                    updateRomIndex(false);
                                }
                            }
                            break;
                        case ROM_IF1 :
                            // Write ROM data to bus
                            writeRomData(address);

                            // Detect post-M1 cycle for Interface 1 paging
                            if (address == 0x700)
                            {
                                interface1Paged = false;
                                updateRomIndex(false);
                            }
                            break;
                        case ROM_DIVMMC :
                            // Write ROM data to bus
                            writeRomData(address);

                            // Detect post-M1 cycle for DivMMC paging
                            // NOTE: Avoid paging out on MAPRAM to allow DivMMC
                            // loaded ROM images to behave correctly
                            if (!divMmcMapRam && ((address & 0xfff8) == 0x1ff8))
                            {
                                divMmcPaged = false;
                                divMmcAutoMap = false;
                                updateRomIndex(false);

                                // Disable the DivMMC, and enable the Interface 1
                                if (divMmcRemoval)
                                {
                                    divMmcRemoval = false;
                                    divMmcPresent = false;
                                    interface1Present = true;
                                }
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
                            writeRomData(address);

                            // Detect M1 cycle for Interface 1 paging
                            if (interface1Present &&
                                ((address == 0x08) || (address == 0x1708)))
                            {
                                // M1 cycle for Interface 1 paging
                                interface1Paged = true;
                                updateRomIndex(false);
                            }
                            break;
                        default :
                            // Write ROM data to bus
                            writeRomData(address);
                            break;
                    }
                }
            }
        } else if (!busRdActive && ((gpioSix & IO_READ_MASK) == 0x00000000))
        {
            // Perform I/O read access
            uint8_t port = decodeLowAddress(gpioSix);
            busRdActive = true;
            switch (port)
            {
                case 0xeb :
                    if (isDivMmcSelected())
                    {
                        // Transfer SD SPI read data to bus
                        if (divMmcSpi.hasReadData())
                        {
                            writeData(divMmcSpi.readData());
                            if (!divMmcSpi.hasReadData())
                            {
                                divMmcSpi.writeData(SdSpiZXTeensy::SD_SPI_READ, 0xff);
                            }
                        } else {
                            writeData(0xff);
                            divMmcSpi.writeData(SdSpiZXTeensy::SD_SPI_READ, 0xff);
                        }
                    }
                    break;
                case 0x3f :
                    if (mf128Present)
                    {
                        if (mf128Paged)
                        {
                            mf128Paged = false;
                            updateRomIndex(true);
                        }
                        writeData(mf128VideoRam ? 0x80 : 0x00);
                    }
                    break;
                case 0xbf :
                    if (mf128Present)
                    {
                        if (!mf128Paged)
                        {
                            mf128Paged = true;
                            updateRomIndex(true);
                        }
                        writeData(mf128VideoRam ? 0x80 : 0x00);
                    }
                    break;
                case 0x3b :
                    if (uartPresent)
                    {
                        switch (decodeHighAddress(gpioSix))
                        {
                            case 0x13 :
                                writeData(espUart.getStatusByte());
                                break;
                            case 0x14 :
                                writeData(espUart.hasReadData() ? espUart.readData() : 0x00);
                                break;
                        }
                    }
                    break;
            }
        }
    }
}
