
// Must be set in SdFat/src/SdFatConfig.h
#define SPI_DRIVER_SELECT 2

//#define ENABLE_ALLRAM
#define ENABLE_BUILTIN_ROM_IF1
//#define DEBUG_OUTPUT

#include <SD.h>
#include <SPI.h>
#include <SdFat.h>
#include "if1-2_rom.h"

// Run the Teensy 4.1 with slight overclock at 816MHz
// Run the SD card at ~7MHz (at 816MHz, SD_TICK_CYCCNT = 58)
#define TEENSY_CLK_FREQ 816000000ULL
#define SD_CLK_FREQ 7000000ULL
#define SD_TICK_CYCCNT ((TEENSY_CLK_FREQ / SD_CLK_FREQ) / 2)

// Allow ~225ms for reset to debounce (at 816MHz, RESET_DELAY_CNT = 0x304D4D)
#define RESET_DELAY_MS 225
#define RESET_DELAY_CNT ((RESET_DELAY_MS * TEENSY_CLK_FREQ) / (SD_TICK_CYCCNT * 1000))

extern "C" uint32_t set_arm_clock(uint32_t frequency);

typedef enum {
    STATE_RESET,
    STATE_ROM_ENABLE,
    STATE_ROM_DISABLE
} run_state_t;

typedef enum {
    RESET_ACTIVE,
    RESET_HOLD,
    RESET_DELAY,
    RESET_READY
} reset_state_t;

typedef enum {
    SD_SPI_WRITE,
    SD_SPI_READ,
    SD_SPI_ENABLE,
    SD_SPI_DISABLE
} sd_spi_action_t;

typedef enum {
    BANK_ROM0   = 0x0001,
    BANK_ROM1   = 0x0002,
    BANK_ROM2   = 0x0004,
    BANK_ROM3   = 0x0008,
    BANK_ALLRAM = 0x0010,
    BANK_IF1    = 0x0020,
    BANK_DIVMMC = 0x0040,
    BANK_MF128  = 0x0080,
    BANK_ZXC2   = 0x0100,
    BANK_MENU   = 0x0200
} bank_select_t;

typedef enum {
    ROM_ROM0,
    ROM_ROM1,
    ROM_ROM2,
    ROM_ROM3,
    ROM_IF1,
    ROM_DIVMMC,
    ROM_MF128,
    ROM_MENU,
    ROM_PAGE_COUNT
} rom_index_t;

// I/O pin assignments
const uint8_t LED_PIN = 13;
const uint8_t DATA_DIS_PIN = 29;
const uint8_t DATA_OUT_PIN = 34;  // 1 = output, 0 = input
const uint8_t RESET_PIN = 31;
const uint8_t RESET_IN_PIN = 2;
const uint8_t BUTTON_PIN = 33;
const uint8_t ROMCS_PIN = 35;
const uint8_t ROMCS_IN_PIN = 3;
const uint8_t IF1_DIS_PIN = 5;
const uint8_t NMI_PIN = 30;
const uint8_t RD_PIN = 1;
const uint8_t WR_PIN = 0;
const uint8_t MREQ_PIN = 24;
const uint8_t IOREQ_PIN = 25;
const uint8_t M1_PIN = 4;
const uint8_t NC_IN_A_PIN = 28;
const uint8_t NC_IN_B_PIN = 36;
const uint8_t NC_IN_C_PIN = 37;

const uint8_t INPUT_PINS[] = {
    RESET_IN_PIN, MREQ_PIN, RD_PIN, IOREQ_PIN, WR_PIN, M1_PIN, ROMCS_IN_PIN,
    19, 18, 14, 15, 40, 41, 17, 16, 22, 23, 20, 21, 38, 39, 26, 27, // Address bus
    BUTTON_PIN, NC_IN_A_PIN, NC_IN_B_PIN, NC_IN_C_PIN
};

const uint32_t IOREQ_PIN_BITMASK = CORE_PIN25_BITMASK;
const uint32_t MREQ_PIN_BITMASK = CORE_PIN24_BITMASK;
const uint32_t A15_PIN_BITMASK = CORE_PIN27_BITMASK;
const uint32_t A14_PIN_BITMASK = CORE_PIN26_BITMASK;
const uint32_t A13_PIN_BITMASK = CORE_PIN39_BITMASK;
const uint32_t A12_PIN_BITMASK = CORE_PIN38_BITMASK;

const uint8_t DATA_PINS[] = { 6, 7, 8, 9, 10, 11, 12, 32 };

const uint32_t GPIO7_DATA_MASK = (CORE_PIN6_BITMASK | CORE_PIN7_BITMASK |
    CORE_PIN8_BITMASK | CORE_PIN9_BITMASK |
    CORE_PIN10_BITMASK | CORE_PIN11_BITMASK |
    CORE_PIN12_BITMASK | CORE_PIN32_BITMASK);
    
const uint32_t DATA_OUT_PIN_BITMASK = CORE_PIN34_BITMASK;

const uint8_t OUTPUT_PINS[] = {
    LED_PIN, ROMCS_PIN, NMI_PIN, IF1_DIS_PIN
};

const uint8_t SD_CS_PIN = 46;
const uint8_t SD_OUT_PIN = 45;
const uint8_t SD_CLK_PIN = 44;
const uint8_t SD_IN_PIN = 43;

// Mask for A15, A14, ^RD, and ^MREQ
const uint32_t ROM_READ_MASK = (A15_PIN_BITMASK | A14_PIN_BITMASK |
    CORE_PIN1_BITMASK | MREQ_PIN_BITMASK);

// Mask for ^RD and ^IOREQ
const uint32_t IO_READ_MASK = (IOREQ_PIN_BITMASK | CORE_PIN1_BITMASK);

// Mask for A15, A14, A13 and ^MREQ
const uint32_t DIVMMC_RAM_WRITE_MASK = (A15_PIN_BITMASK | A14_PIN_BITMASK |
    A13_PIN_BITMASK | MREQ_PIN_BITMASK);

// Number of SD retries
const uint8_t NUM_SD_RETRIES = 3;

// Global state
volatile bool sd_card_present = false;
volatile bool afterFirstReset = 0x00;
volatile reset_state_t resetState = RESET_ACTIVE;
volatile uint32_t resetExitCount = RESET_DELAY_CNT;
volatile run_state_t globalState = STATE_RESET;
volatile bool busRdActive = false;
volatile bool nmiPending = false;

// ROM banking
const uint16_t ROM_PAGE_SIZE = 0x4000;
volatile bank_select_t romSelected = BANK_ROM0;
volatile uint8_t romArray[ROM_PAGE_COUNT][ROM_PAGE_SIZE];
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

#ifdef ENABLE_ALLRAM
volatile bool allRamPaged = false;
volatile DMAMEM uint8_t allRamArray[ROM_PAGE_SIZE];
#endif

// Debug menu ROM
volatile bool menuPaged = false;

// Interface 1
volatile bool interface1Present = false;
volatile bool interface1Paged = false;
const uint16_t PAGE_BANK_MF128_IF1 = (BANK_ROM0 | BANK_ROM1 | BANK_ROM3 | BANK_MF128);

// DivMMC with 2 x 256KB of RAM
const uint16_t RAM_PAGE_COUNT = 32;
const uint16_t RAM_PAGE_SIZE = 0x2000;
volatile uint8_t divMmcRamArray[RAM_PAGE_COUNT][RAM_PAGE_SIZE];
volatile DMAMEM uint8_t divMmcHighRamArray[RAM_PAGE_COUNT][RAM_PAGE_SIZE];
volatile bool divMmcPresent = false;
volatile bool divMmcPaged = false;
volatile bool divMmcAutoMap = false;
volatile bool divMmcConMem = false;
volatile bool divMmcMapRam = false;
volatile bool divMmcRamBankThree = false;
volatile uint8_t* divMmcRamPtr = divMmcRamArray[0];
volatile bool divMmcRemoval = false;
const uint16_t PAGE_BANK_DIVMMC = (BANK_ROM0 | BANK_ROM1 | BANK_ROM3 | BANK_ALLRAM);

// DivMMC SPI
volatile uint8_t sdSpiTxData = 0xff;
volatile uint8_t sdSpiRxData = 0xff;
volatile bool sdSpiRxNotTx = false;
volatile uint8_t sdSpiTick = 0;

// Multiface 128
volatile bool mf128Present = false;
volatile bool mf128Paged = false;
volatile bool mf128VideoRam = false;
volatile bool mf128ActiveNMI = false;

// ZXC2 cartridge (reuses divMmcRamArray)
volatile bool zxC2Present = false;
volatile bool zxC2Paged = false;
volatile bool zxC2Lock = false;
volatile uint8_t zxC2BankPtr = 0x00;

// DivMMC SD buffer
typedef enum {
    SD_BUFFER_READ = 0,
    SD_BUFFER_WRITE = 1,
    SD_BUFFER_FLAGS = 2
} sd_spi_buffer;
const uint8_t SPI_BUFFER_SIZE = 8;
volatile uint8_t sdSpiDataBuffer[3][SPI_BUFFER_SIZE];
volatile uint8_t sdSpiReadDataWritePtr = 0;
volatile uint8_t sdSpiReadDataReadPtr = 0;
volatile uint8_t sdSpiWriteDataWritePtr = 0;
volatile uint8_t sdSpiWriteDataReadPtr = 0;
volatile uint8_t sdSpiCount = 0;

// DivMMC SPI tick every SD_TICK_CYCCNT count
volatile uint32_t cycleCount;

#ifdef DEBUG_OUTPUT

// Debug data buffer
const uint16_t DEBUG_BUFFER_SIZE = 128;
volatile uint8_t debugDataBuffer[DEBUG_BUFFER_SIZE];
volatile uint16_t debugWritePtr = 0;
volatile uint16_t debugReadPtr = 0;

inline __attribute__((always_inline)) void writeDebugData(uint8_t data_)
{
    debugDataBuffer[debugWritePtr] = data_;
    debugWritePtr = (debugWritePtr + 1) & (DEBUG_BUFFER_SIZE - 1);
    if (!hasDebugData())
    {
        debugReadPtr = (debugReadPtr + 1) & (DEBUG_BUFFER_SIZE - 1);
    }
}

inline __attribute__((always_inline)) uint8_t readDebugData()
{
    uint8_t data_;
    if (hasDebugData())
    {
        data_ = debugDataBuffer[debugReadPtr];
        debugReadPtr = (debugReadPtr + 1) & (DEBUG_BUFFER_SIZE - 1);
    } else {
        data_ = 0xff;
    }
    return data_;
}

inline __attribute__((always_inline)) bool hasDebugData()
{
    return (debugReadPtr != debugWritePtr);
}

#endif

inline __attribute__((always_inline)) void writeDivMmcReadData(uint8_t data_)
{
    sdSpiDataBuffer[SD_BUFFER_READ][sdSpiReadDataWritePtr] = data_;
    sdSpiReadDataWritePtr = (sdSpiReadDataWritePtr + 1) & (SPI_BUFFER_SIZE - 1);
}

inline __attribute__((always_inline)) uint8_t readDivMmcReadData()
{
    uint8_t data_ = sdSpiDataBuffer[SD_BUFFER_READ][sdSpiReadDataReadPtr];
    sdSpiReadDataReadPtr = (sdSpiReadDataReadPtr + 1) & (SPI_BUFFER_SIZE - 1);
    return data_;
}

inline __attribute__((always_inline)) bool hasDivMmcReadData()
{
    return (sdSpiReadDataReadPtr != sdSpiReadDataWritePtr);
}

inline __attribute__((always_inline)) void writeDivMmcWriteData(sd_spi_action_t action_, uint8_t data_)
{
    sdSpiDataBuffer[SD_BUFFER_WRITE][sdSpiWriteDataWritePtr] = data_;
    sdSpiDataBuffer[SD_BUFFER_FLAGS][sdSpiWriteDataWritePtr] = (uint8_t)action_;
    sdSpiWriteDataWritePtr = (sdSpiWriteDataWritePtr + 1) & (SPI_BUFFER_SIZE - 1);
}

inline __attribute__((always_inline)) sd_spi_action_t readDivMmcWriteData()
{
    sd_spi_action_t action_ = (sd_spi_action_t)sdSpiDataBuffer[SD_BUFFER_FLAGS][sdSpiWriteDataReadPtr];
    sdSpiRxNotTx = (action_ == SD_SPI_READ);
    sdSpiTxData = sdSpiDataBuffer[SD_BUFFER_WRITE][sdSpiWriteDataReadPtr];
    sdSpiWriteDataReadPtr = (sdSpiWriteDataReadPtr + 1) & (SPI_BUFFER_SIZE - 1);
    return action_;
}

inline __attribute__((always_inline)) bool hasDivMmcWriteData()
{
    return (sdSpiWriteDataReadPtr != sdSpiWriteDataWritePtr);
}

inline __attribute__((always_inline)) void writeData(uint8_t data_)
{
    // Output D[7:0] to GPIO2/7
    uint32_t regbits_ = ((data_ & 0x07) | ((data_ & 0x38) << 7) | ((data_ & 0xc0) << 10));
    CORE_PIN34_PORTSET = DATA_OUT_PIN_BITMASK;
    CORE_PIN10_DDRREG |= GPIO7_DATA_MASK;
    CORE_PIN10_PORTSET = regbits_ & GPIO7_DATA_MASK;
    CORE_PIN10_PORTCLEAR = (~regbits_) & GPIO7_DATA_MASK;
}

inline __attribute__((always_inline)) uint8_t readData()
{
    // Decode D[7:0] from GPIO2/7
    uint32_t regbits_ = (*(volatile uint32_t *)IMXRT_GPIO7_ADDRESS);
    uint32_t tmp_ = ((regbits_ & 0x07) | ((regbits_ & 0x1c00) >> 7) | ((regbits_ & 0x30000) >> 10));
    return tmp_;
}

inline __attribute__((always_inline)) void disableData()
{
    CORE_PIN10_DDRREG &= ~GPIO7_DATA_MASK;
    CORE_PIN34_PORTCLEAR = DATA_OUT_PIN_BITMASK;
}

inline __attribute__((always_inline)) uint16_t decodeAddress(uint32_t gpio_6_)
{
    // Decode A[15:0] from GPIO1/7
    return (gpio_6_ >> 16);
}

inline __attribute__((always_inline)) uint16_t decodeRamAddress(uint32_t gpio_6_)
{
    // Decode A[12:0] from GPIO1/6
    return ((gpio_6_ & 0x1fff0000) >> 16);
}

inline __attribute__((always_inline)) uint8_t decodeLowAddress(uint32_t gpio_6_)
{
    // Decode A[7:0] from GPIO
    return ((gpio_6_ & 0x00ff0000) >> 16);
}

inline __attribute__((always_inline)) void performSdSpi()
{
    // Perform SD SPI accesses on clock edges
    if (sdSpiTick == 0x00)
    {
        // SPI is idle, so wait for write data
        if (hasDivMmcWriteData())
        {
            switch (readDivMmcWriteData())
            {
                case SD_SPI_ENABLE :
                    CORE_PIN46_PORTCLEAR = CORE_PIN46_BITMASK;
                    break;
                case SD_SPI_DISABLE :
                    CORE_PIN46_PORTSET = CORE_PIN46_BITMASK;
                    break;
                default :
                    // Drive new SPI access
                    if (sdSpiTxData & 0x80)
                    {
                        CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                    } else {
                        CORE_PIN45_PORTCLEAR = CORE_PIN45_BITMASK;
                    }
                    sdSpiTxData <<= 1;
                    sdSpiTick = 1;
                    break;
            }
        }
    } else if (sdSpiTick & 0x0f)
    {
        // SPI is being driven, so toggle clock and data
        if (sdSpiTick & 0x01)
        {
            // Capture read data on rising edges
            if (sdSpiRxNotTx)
            {
                if (CORE_PIN43_PINREG & CORE_PIN43_BITMASK)
                {
                    sdSpiRxData = (sdSpiRxData << 1) | 0x01;
                } else {
                    sdSpiRxData <<= 1;
                }
                if (sdSpiTick == 0x0f)
                {
                    // Write read data into buffer
                    writeDivMmcReadData(sdSpiRxData);
                }
            }
        } else if (!sdSpiRxNotTx)
        {
            // Drive write data on falling edges
            if (sdSpiTxData & 0x80)
            {
                CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
            } else {
                CORE_PIN45_PORTCLEAR = CORE_PIN45_BITMASK;
            }
            sdSpiTxData <<= 1;
        }
        CORE_PIN44_PORTTOGGLE = CORE_PIN44_BITMASK;
        ++sdSpiTick;
    } else {
        // SPI access has just finished, so test for write data
        // before falling idle, otherwise continue
        ++sdSpiCount;
        if (hasDivMmcWriteData())
        {
            switch (readDivMmcWriteData())
            {
                case SD_SPI_ENABLE :
                    CORE_PIN46_PORTCLEAR = CORE_PIN46_BITMASK;
                    CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                    sdSpiTick = 0;
                    break;
                case SD_SPI_DISABLE :
                    CORE_PIN46_PORTSET = CORE_PIN46_BITMASK;
                    CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                    sdSpiTick = 0;
                    break;
                default :
                    // Drive new SPI access on this falling edge
                    if (sdSpiTxData & 0x80)
                    {
                        CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                    } else {
                        CORE_PIN45_PORTCLEAR = CORE_PIN45_BITMASK;
                    }
                    sdSpiTxData <<= 1;
                    sdSpiTick = 1;
                    break;
            }
        } else {
            CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
            sdSpiTick = 0;
        }
        CORE_PIN44_PORTCLEAR = CORE_PIN44_BITMASK;
    }
}

inline __attribute__((always_inline)) void performOnClock()
{
    uint32_t cycle_ = ARM_DWT_CYCCNT;
    if ((cycle_ - cycleCount) >= SD_TICK_CYCCNT)
    {
        cycleCount = cycle_;

        // Perform SD SPI accesses on clock edges
        if ((globalState == STATE_RESET) || (sdSpiTick != 0) ||
            hasDivMmcWriteData())
        {
            performSdSpi();
        }

        // Debounce the reset detection
        switch (resetState)
        {
            case RESET_HOLD :
                if (digitalReadFast(RESET_IN_PIN))
                {
                    resetState = RESET_DELAY;
                    resetExitCount = RESET_DELAY_CNT;
                }
                break;
            case RESET_DELAY :
                if (digitalReadFast(RESET_IN_PIN))
                {
                    --resetExitCount;
                    if (resetExitCount == 0)
                    {
                        resetState = RESET_READY;
                    }
                } else {
                    resetExitCount = RESET_DELAY_CNT;
                }
                break;
            default :
                break;
        }
    }
}

void setState(run_state_t state_)
{
    switch (state_)
    {
        case STATE_RESET :
            resetState = RESET_ACTIVE;
            digitalWriteFast(RESET_PIN, 1);
            digitalWriteFast(ROMCS_PIN, 1);
            disableData();
            break;
        case STATE_ROM_ENABLE :
            resetState = RESET_HOLD;
            digitalWriteFast(DATA_DIS_PIN, 0);
            digitalWriteFast(RESET_PIN, 0);
            break;
        case STATE_ROM_DISABLE :
            resetState = RESET_HOLD;
            digitalWriteFast(DATA_DIS_PIN, 1);
            digitalWriteFast(ROMCS_PIN, 0);
            digitalWriteFast(RESET_PIN, 0);
            break;
    }
    globalState = state_;
}

inline __attribute__((always_inline)) bool isDivMmcSelected()
{
    return (divMmcPresent && ((romSelected & (BANK_MF128 | BANK_IF1)) == 0));
}

inline __attribute__((always_inline)) void updateRomIndex(bool page_now_)
{
    // Determine which ROM is currently paged
    if (menuPaged)
    {
        romSelected = BANK_MENU;
    } else if (mf128Paged)
    {
        romSelected = BANK_MF128;
    } else if (divMmcPaged)
    {
        romSelected = BANK_DIVMMC;
    } else if (zxC2Paged)
    {
        romSelected = BANK_ZXC2;
    } else if (interface1Paged)
    {
        romSelected = BANK_IF1;
#ifdef ENABLE_ALLRAM
    } else if (allRamPaged)
    {
        romSelected = BANK_ALLRAM;
#endif
    } else if (rom23Paged)
    {
        romSelected = rom1Paged ? BANK_ROM3 : BANK_ROM2;
    } else {
        romSelected = rom1Paged ? BANK_ROM1 : BANK_ROM0;
    }

    // Enable soft ROM when page is present
    if ((romArrayPresent & romSelected) != 0)
    {
        switch (romSelected)
        {
            case BANK_ROM0 :
                romPtr = romArray[ROM_ROM0];
                break;
            case BANK_ROM1 :
                romPtr = romArray[ROM_ROM1];
                break;
            case BANK_ROM2 :
                romPtr = romArray[ROM_ROM2];
                break;
            case BANK_ROM3 :
                romPtr = romArray[ROM_ROM3];
                break;
#ifdef ENABLE_ALLRAM
            case BANK_ALLRAM :
                romPtr = allRamArray;
                break;
#endif
            case BANK_IF1 :
                romPtr = romArray[ROM_IF1];
                break;
            case BANK_DIVMMC :
                if (divMmcMapRam && !divMmcConMem)
                {
                    romPtr = divMmcRamArray[3];
                } else {
                    romPtr = romArray[ROM_DIVMMC];
                }
                break;
            case BANK_MF128 :
                romPtr = romArray[ROM_MF128];
                break;
            case BANK_ZXC2 :
                romPtr = divMmcHighRamArray[zxC2BankPtr];
                break;
            default :
                romPtr = romArray[ROM_MENU];
                break;
        }
        if (!romEnabled)
        {
            // Page in the soft ROM
            if (page_now_)
            {
                digitalWriteFast(ROMCS_PIN, 1);
                romEnabled = true;
            } else {
                romCsEnable = true;
            }
        }
    } else if (romEnabled)
    {
        // Fall back to real ROM
        if (page_now_)
        {
            digitalWriteFast(ROMCS_PIN, 0);
            romEnabled = false;
        } else {
            romCsDisable = true;
        }
    }
    if (page_now_)
    {
        if (isDivMmcSelected())
        {
            digitalWriteFast(IF1_DIS_PIN, 1);
        } else {
            digitalWriteFast(IF1_DIS_PIN, 0);
        }
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
    cycleCount = ARM_DWT_CYCCNT;
    setState(STATE_RESET);

    // Start Serial debug
#ifdef DEBUG_OUTPUT
    Serial.begin(115200);
#endif

    // Setup RD, WR, ROMCS, reset and button ISRs
    attachInterrupt(digitalPinToInterrupt(RD_PIN), isr_rd_event, CHANGE);
    attachInterrupt(digitalPinToInterrupt(WR_PIN), isr_wr_event, FALLING);
    attachInterrupt(digitalPinToInterrupt(ROMCS_IN_PIN), isr_rd_event, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RESET_IN_PIN), isr_reset, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr_button, FALLING);

    // TODO: set HW ints as high priority, otherwise ethernet int timer causes misses
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 16);
}

class SdSpiZXTeensy : public SdSpiSoftDriver {
    public:
        /** Initialize the SPI bus. */
        void begin()
        {
            // Do nothing
        }

        /** Receive a byte.
        *
        * \return The byte.
        */
        uint8_t receive()
        {
            writeDivMmcWriteData(SD_SPI_READ, 0xff);
            while (!hasDivMmcReadData()) {
                performOnClock();
            };
            return readDivMmcReadData();
        }

        /** Send a byte.
        *
        * \param[in] data Byte to send
        */
        void send(uint8_t data)
        {
            uint8_t count_ = sdSpiCount;
            writeDivMmcWriteData(SD_SPI_WRITE, data);
            while (count_ == sdSpiCount) {
                performOnClock();
            };
        }
};

SdSpiZXTeensy divMmcSpi;

bool loadSpectrumRomImage()
{
    bool result_ = false;
    File RomFile = SD.open("plus3.rom", FILE_READ);
    if (!RomFile)
    {
        RomFile = SD.open("plus2.rom", FILE_READ);
    }
    if (!RomFile)
    {
        RomFile = SD.open("48.rom", FILE_READ);
    }
    if (RomFile)
    {
        size_t count_ = RomFile.readBytes((char *)romArray[ROM_ROM0], ROM_PAGE_SIZE);
        if (count_ > 0)
        {
            result_ = true;
            romArrayPresent |= BANK_ROM0;
            if (count_ >= ROM_PAGE_SIZE)
            {
                count_ = RomFile.readBytes((char *)romArray[ROM_ROM1], ROM_PAGE_SIZE);
                if (count_ > 0)
                {
                    rom1Present = true;
                    romArrayPresent |= BANK_ROM1;
                    if (count_ >= ROM_PAGE_SIZE)
                    {
                        count_ = RomFile.readBytes((char *)romArray[ROM_ROM2], ROM_PAGE_SIZE);
                        if (count_ > 0)
                        {
                            rom23Present = true;
                            romArrayPresent |= (BANK_ROM2 | BANK_ROM3 | BANK_ALLRAM);
                            count_ = RomFile.readBytes((char *)romArray[ROM_ROM3], ROM_PAGE_SIZE);
                        }
                    }
                }
            }
        }
        RomFile.close();
    }
    return result_;
}

uint32_t loadZXC2RomImage(const char* filename)
{
    // The ZXC2 cartridge is loaded into the DivMMC high RAM area
    size_t count_ = 0;
    File RomFile = SD.open(filename, FILE_READ);
    if (RomFile)
    {
        count_ = RomFile.readBytes((char *)divMmcHighRamArray[0], RAM_PAGE_SIZE);
        if (count_ > 0)
        {
            for (uint8_t i_ = 1; i_ < RAM_PAGE_COUNT; ++i_)
            {
                size_t blk_count_ = RomFile.readBytes((char *)divMmcHighRamArray[i_], RAM_PAGE_SIZE);
                count_ += blk_count_;
                if (blk_count_ < RAM_PAGE_SIZE)
                {
                    break;
                }
            }
        }
        RomFile.close();
    }
    return count_;
}

uint16_t loadRomImage(const char* filename, const rom_index_t romIndex, const uint16_t size)
{
    uint16_t count_ = 0;
    File RomFile = SD.open(filename, FILE_READ);
    if (RomFile)
    {
        count_ = RomFile.readBytes((char *)romArray[romIndex], size);
        RomFile.close();
    }
    return count_;
}

void handleStateReset()
{
    // Wait for any previous SD accesses to finish
    while (sdSpiTick != 0) { };
    sdSpiReadDataReadPtr = sdSpiReadDataWritePtr;
    sdSpiWriteDataReadPtr = sdSpiWriteDataWritePtr;

    // Indicate in reset with LED
    digitalWriteFast(LED_PIN, 0);
    delay(100);
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
        for (uint8_t i_ = 0; i_ < RAM_PAGE_COUNT; ++i_)
        {
            for (uint16_t j_ = 0; j_ < RAM_PAGE_SIZE; j_ += 0x400)
            {
                while ( (uint) Serial.availableForWrite() <  0x400);
                Serial.write((char*)&divMmcHighRamArray[i_][j_], 0x400);
            }
        }
    }
#endif
    // Initialise the RAM banks
    while (!digitalReadFast(BUTTON_PIN))
    {
        afterFirstReset = false;
        delay(5);
    }
    if (!afterFirstReset)
    {
        afterFirstReset = true;
        sd_card_present = false;
        for (uint8_t i_ = 0; i_ < RAM_PAGE_COUNT; ++i_)
        {
            for (uint16_t j_ = 0; j_ < RAM_PAGE_SIZE; ++j_)
            {
                divMmcRamArray[i_][j_] = 0xff;
                divMmcHighRamArray[i_][j_] = 0xff;
            }
        }
        for (uint16_t j_ = RAM_PAGE_SIZE; j_ < ROM_PAGE_SIZE; ++j_)
        {
            romArray[ROM_DIVMMC][j_] = 0xff;
            romArray[ROM_MF128][j_] = 0xff;
        }
    }

    // Initialise the SD card
    if (!sd_card_present)
    {
        pinMode(SD_CS_PIN, INPUT_PULLDOWN);
        if (digitalReadFast(SD_CS_PIN))
        {
            uint8_t retries_ = 0;
            sd_card_present = true;
            pinMode(SD_CS_PIN, OUTPUT);
            digitalWriteFast(SD_CS_PIN, 1);
            digitalWriteFast(SD_OUT_PIN, 1);
            digitalWriteFast(SD_CLK_PIN, 0);
            while (!SD.sdfs.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(1), &divMmcSpi)))
            {
                ++retries_;
                if (retries_ > NUM_SD_RETRIES)
                {
                    sd_card_present = false;
                    break;
                }
            }
        }
    }

    // Reset the soft ROM detection state
    romArrayPresent = 0;
    rom1Present = false;
    rom23Present = false;
    interface1Present = false;
    divMmcPresent = false;
    mf128Present = false;
    zxC2Present = false;

    // Reset the banking state
    nmiPending = false;
    menuPaged = false;
    rom1Paged = false;
    rom23Paged = false;
#ifdef ENABLE_ALLRAM
    allRamPaged = false;
#endif
    interface1Paged = false;
    divMmcPaged = false;
    divMmcConMem = false;
    divMmcAutoMap = false;
    divMmcMapRam = false;
    divMmcRemoval = false;
    divMmcRamPtr = divMmcRamArray[0];
    mf128Paged = false;
    mf128VideoRam = false;
    mf128ActiveNMI = false;
    zxC2Paged = false;
    zxC2Lock = false;
    zxC2BankPtr = 0x00;
    romEnabled = false;
    romSelected = BANK_ROM0;
    digitalWriteFast(ROMCS_PIN, 0);
    digitalWriteFast(IF1_DIS_PIN, 0);
    digitalWriteFast(NMI_PIN, 0);
    delay(100);

    // Load the built-in Interface 1 soft ROM
#ifdef ENABLE_BUILTIN_ROM_IF1
    memcpy((void *)romArray[ROM_IF1], BUILTIN_ROM_IF1, BUILTIN_ROM_IF1_SIZE);
    interface1Present = true;
    romArrayPresent |= BANK_IF1;
#endif

    // Load ROMs from the SD card
    if (sd_card_present)
    {
        // Load spectrum ROM
        if (!loadSpectrumRomImage())
        {
            // Enable ROM 1 bank detection if no soft ROM is loaded
            rom1Present = true;
        }

        // Load ZXC2 cartridge ROM
        if (loadZXC2RomImage("zxc2.rom") > 0)
        {
            zxC2Present = true;
            romArrayPresent |= BANK_ZXC2;
        }

        // Load interface 1 ROM
        if (loadRomImage("if1.rom", ROM_IF1, ROM_PAGE_SIZE) > 0)
        {
            interface1Present = true;
            romArrayPresent |= BANK_IF1;
        }

        // Load menu ROM
        if (loadRomImage("menu.rom", ROM_MENU, ROM_PAGE_SIZE) > 0)
        {
            if (!digitalReadFast(BUTTON_PIN))
            {
                menuPaged = true;
            }
            romArrayPresent |= BANK_MENU;
        }

        // Load DivMMC Esxdos ROM
        if (loadRomImage("esxmmc.bin", ROM_DIVMMC, RAM_PAGE_SIZE) > 0)
        {
            divMmcPresent = true;
            romArrayPresent |= BANK_DIVMMC;
        }

        // Load Multiface 128 ROM
        if (loadRomImage("mf128.rom", ROM_MF128, RAM_PAGE_SIZE) > 0)
        {
            mf128Present = true;
            romArrayPresent |= BANK_MF128;
        }
    }

    // If ZXC2 cartridge is present, then disable DivMMC
    if (zxC2Present)
    {
        divMmcPresent = false;
        zxC2Paged = true;
    }

    // If DivMMC is present, then disable Interface 1
    if (divMmcPresent)
    {
        // The Interface 1 can be enabled by switching back
        // into 128k mode (".128")
        interface1Present = false;

        // Close the SD card to hand to the DivMMC
        SD.sdfs.end();
        sd_card_present = false;
    }

    // Enable the ROM, if present
    if (romArrayPresent != 0)
    {
        updateRomIndex(true);
        setState(STATE_ROM_ENABLE);
        digitalWriteFast(LED_PIN, 1);
    } else {
        // Disable the soft ROM
        setState(STATE_ROM_DISABLE);
        digitalWriteFast(LED_PIN, 0);
    }
}

void loop()
{
    // Detect reset entry, and handle
    if (globalState == STATE_RESET)
    {
        handleStateReset();
    }

    // Run SD SPI on regular ticks
    performOnClock();
}

inline __attribute__((always_inline)) void writeRomData(uint16_t address_)
{
    // Transfer soft ROM, or DivMMC RAM, data to the bus
    if (digitalReadFast(ROMCS_IN_PIN))
    {
        disableData();
        busRdActive = false;
    } else if ((romSelected == BANK_DIVMMC) && (address_ >= RAM_PAGE_SIZE))
    {
        address_ &= (RAM_PAGE_SIZE - 1);
        uint8_t data_ = divMmcRamPtr[address_];
        writeData(data_);
    } else if (romEnabled)
    {
        writeData(romPtr[address_]);
    }
}

FASTRUN void isr_reset()
{
    // Perform entry to reset when not debouncing reset
    if ((resetState == RESET_READY) && !digitalReadFast(RESET_IN_PIN))
    {
        setState(STATE_RESET);
    }
}

FASTRUN void isr_button()
{
    // Perform NMI when not already handling previous NMI
    if ((globalState != STATE_RESET) && !nmiPending &&
        !mf128ActiveNMI && !divMmcPaged &&
        !digitalReadFast(BUTTON_PIN))
    {
        nmiPending = true;
        digitalWriteFast(NMI_PIN, 1);
    }
}

FASTRUN void isr_wr_event()
{
    if (globalState == STATE_ROM_ENABLE)
    {
        // Start of write access
        uint32_t gpio_6_ = (*(volatile uint32_t *)IMXRT_GPIO6_ADDRESS);
        if ((gpio_6_ & DIVMMC_RAM_WRITE_MASK) == A13_PIN_BITMASK)
        {
#ifdef ENABLE_ALLRAM
            if (romSelected == BANK_ALLRAM)
            {
                // Perform AllRam RAM write
                uint8_t data_ = readData();
                uint16_t address_ = decodeAddress(gpio_6_);
                romPtr[address_] = data_;
            } else 
#endif
            if (romSelected == BANK_MF128)
            {
                // Perform Multiface 128 RAM write
                uint8_t data_ = readData();
                uint16_t address_ = (0x2000 | decodeRamAddress(gpio_6_));
                romPtr[address_] = data_;
            } else if ((romSelected == BANK_DIVMMC) &&
                (!divMmcMapRam || divMmcConMem || !divMmcRamBankThree))
            {
                // Perform DivMMC RAM write
                uint8_t data_ = readData();
                uint16_t address_ = decodeRamAddress(gpio_6_);
                divMmcRamPtr[address_] = data_;
            }
        } else if ((gpio_6_ & IOREQ_PIN_BITMASK) == 0x00000000)
        {
            // Perform I/O write access
            uint8_t port_ = decodeLowAddress(gpio_6_);
            if ((port_ & 0x02) == 0)
            {
                if ((gpio_6_ & A15_PIN_BITMASK) == 0x00000000)
                {
                    // Perform IO 'h1ffd or 'h7ffd write access
                    bool is_7f_ = ((gpio_6_ & A14_PIN_BITMASK) != 0x0);
                    uint8_t data_ = readData();
                    if (rom1Present)
                    {
                        if (!rom23Present || is_7f_)
                        {
                            // Detect 'h7ffd write access for 128k ROMs
                            if (mf128Present)
                            {
                                mf128VideoRam = ((data_ & 0x08) != 0);
                            }
                            if ((data_ & 0x20) != 0)
                            {
                                rom1Present = false;
                            }
                            rom1Paged = ((data_ & 0x10) != 0);
                        }
                        if (rom23Present && !is_7f_ &&
                            ((gpio_6_ & A13_PIN_BITMASK) == 0x0) &&
                            ((gpio_6_ & A12_PIN_BITMASK) != 0x0))
                        {
                            // Detect 'h1ffd write access for +3 ROMs
                            rom23Paged = ((data_ & 0x04) != 0);
#ifdef ENABLE_ALLRAM
                            allRamPaged = ((data_ & 0x01) != 0);
#endif
                        }
                        updateRomIndex(true);
                    }

                    // Detect 'h7ffd write access to disable DivMMC
                    if (is_7f_ && divMmcPaged &&
                        ((data_ & 0x10) == 0x0))
                    {
                        divMmcRemoval = true;
                    }
                }
            } else if (mf128ActiveNMI && ((port_ == 0x3f) || (port_ == 0xbf)))
            {
                mf128ActiveNMI = false;
                updateRomIndex(true);
            } else if (isDivMmcSelected())
            {
                switch (port_)
                {
                    case 0xe7 : // DivMMC card select
                        if ((readData() & 0x1) != 0)
                        {
                            writeDivMmcWriteData(SD_SPI_DISABLE, 0xff);
                            digitalWriteFast(LED_PIN, 1);
                        } else {
                            writeDivMmcWriteData(SD_SPI_ENABLE, 0xff);
                            digitalWriteFast(LED_PIN, 0);
                        }
                        break;
                    case 0xeb : // DivMMC write
                        if (romSelected == BANK_DIVMMC)
                        {
                            writeDivMmcWriteData(SD_SPI_WRITE, readData());
                        }
                        break;
                    case 0xe3 : // DivMMC control
                        {
                            uint8_t data_ = readData();
                            if ((data_ & 0x80) != 0)
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
                            if ((data_ & 0x40) != 0)
                            {
                                divMmcMapRam = true;
                            }
                            if ((data_ & 0x20) != 0)
                            {
                                divMmcRamPtr = divMmcHighRamArray[(data_ & (RAM_PAGE_COUNT - 1))];
                                divMmcRamBankThree = false;
                            } else {
                                data_ &= (RAM_PAGE_COUNT - 1);
                                divMmcRamPtr = divMmcRamArray[data_];
                                if (data_ == 0x03)
                                {
                                    divMmcRamBankThree = true;
                                } else {
                                    divMmcRamBankThree = false;
                                }
                            }
                            updateRomIndex(true);
                        }
                        break;
                }
            }
        }
    }
}

FASTRUN void isr_rd_event()
{
    if (digitalReadFast(RD_PIN))
    {
        if (busRdActive)
        {
            // End of read access
            disableData();
            busRdActive = false;
            if (romCsEnable)
            {
                digitalWriteFast(ROMCS_PIN, 1);
                romEnabled = true;
                romCsEnable = false;
                if (isDivMmcSelected())
                {
                    digitalWriteFast(IF1_DIS_PIN, 1);
                } else {
                    digitalWriteFast(IF1_DIS_PIN, 0);
                }
            } else if (romCsDisable)
            {
                digitalWriteFast(ROMCS_PIN, 0);
                romEnabled = false;
                romCsDisable = false;
                if (isDivMmcSelected())
                {
                    digitalWriteFast(IF1_DIS_PIN, 1);
                } else {
                    digitalWriteFast(IF1_DIS_PIN, 0);
                }
            }
        }
    } else if (globalState == STATE_ROM_ENABLE)
    {
        if (!busRdActive)
        {
            // Start of read access
            uint32_t gpio_6_ = (*(volatile uint32_t *)IMXRT_GPIO6_ADDRESS);
            if ((gpio_6_ & ROM_READ_MASK) == 0x00000000)
            {
                // Perform ROM read access
                if (digitalReadFast(ROMCS_IN_PIN))
                {
                    disableData();
                } else {
                    busRdActive = true;
                    uint16_t address_ = decodeAddress(gpio_6_);
                    if (zxC2Present && !zxC2Lock && 
                        ((address_ & 0xffc0) == 0x3fc0))
                    {
                        zxC2BankPtr = ((address_ & 0x0f) << 1);
                        zxC2Paged = ((address_ & 0x10) == 0);
                        zxC2Lock = ((address_ & 0x20) != 0);
                        updateRomIndex(true);
                    }
                    if (digitalReadFast(M1_PIN))
                    {
                        // Write ROM data to bus
                        writeRomData(address_);
                    } else if (address_ == 0x66)
                    {
                        // Send the NMI to the Multiface 128
                        if (nmiPending && mf128Present && !divMmcPaged && !mf128ActiveNMI &&
                            ((romSelected & PAGE_BANK_MF128_IF1) != 0))
                        {
                            mf128ActiveNMI = true;
                            mf128Paged = true;
                            updateRomIndex(true);
                        }

                        // Write ROM data to bus
                        writeRomData(address_);

                        // Release NMI on entry to interrupt handler
                        if (nmiPending)
                        {
                            // Send the NMI to the DivMMC
                            if (divMmcPresent && !divMmcPaged && !mf128ActiveNMI &&
                                ((romSelected & PAGE_BANK_DIVMMC) != 0))
                            {
                                divMmcPaged = true;
                                divMmcAutoMap = true;
                                updateRomIndex(false);
                            }

                            nmiPending = false;
                            digitalWriteFast(NMI_PIN, 0);
                        }
                    } else if (divMmcPresent && !mf128Paged)
                    {
                        // Detect M1 cycle for DivMMC paging
                        if (((romSelected & PAGE_BANK_DIVMMC) != 0) &&
                            ((address_ & 0xff00) == 0x3d00))
                        {
                            divMmcPaged = true;
                            divMmcAutoMap = true;
                            updateRomIndex(true);
                        }

                        // Write ROM data to bus
                        writeRomData(address_);

                        // Detect post-M1 cycle for DivMMC paging
                        if (romSelected == BANK_DIVMMC)
                        {
                            if (!divMmcMapRam && ((address_ & 0xfff8) == 0x1ff8))
                            {
                                divMmcPaged = false;
                                divMmcAutoMap = false;
                                updateRomIndex(false);

                                // Disable the DivMMC, and enable the Interface 1
                                if (divMmcRemoval == true)
                                {
                                    divMmcRemoval = false;
                                    divMmcPresent = false;
                                    if ((romArrayPresent & BANK_IF1) != 0)
                                    {
                                        interface1Present = true;
                                    }
                                }
                            }
                        } else if (((romSelected & PAGE_BANK_DIVMMC) != 0) &&
                            ((address_ == 0x00) || (address_ == 0x08) ||
                            (address_ == 0x38) || (address_ == 0x4c6) ||
                            (address_ == 0x562)))
                        {
                            divMmcPaged = true;
                            divMmcAutoMap = true;
                            updateRomIndex(false);
                        }
                    } else {
                        // Detect M1 cycle for Multiface 128 paging
                        if (mf128ActiveNMI && (address_ == 0x67))
                        {
                            mf128Paged = true;
                            updateRomIndex(true);
                        }

                        // Detect M1 cycle for Interface 1 paging
                        if (interface1Present)
                        {
                            if (((romSelected & PAGE_BANK_MF128_IF1) != 0) &&
                                ((address_ == 0x08) || (address_ == 0x1708)))
                            {
                                interface1Paged = true;
                                updateRomIndex(true);
                            }

                            // Write ROM data to bus
                            writeRomData(address_);

                            // Detect post-M1 cycle for Interface 1 paging
                            if ((romSelected == BANK_IF1) && (address_ == 0x700))
                            {
                                interface1Paged = false;
                                updateRomIndex(false);
                            }
                        } else {
                            // Write ROM data to bus
                            writeRomData(address_);
                        }
                    }
                }
            } else if ((gpio_6_ & IO_READ_MASK) == 0x00000000)
            {
                // Perform IO read access
                uint8_t port_ = decodeLowAddress(gpio_6_);
                busRdActive = true;
                switch (port_)
                {
                    case 0xeb :
                        if (isDivMmcSelected())
                        {
                            // Transfer SD SPI read data to bus
                            if (hasDivMmcReadData())
                            {
                                writeData(readDivMmcReadData());
                                if (!hasDivMmcReadData())
                                {
                                    writeDivMmcWriteData(SD_SPI_READ, 0xff);
                                }
                            } else {
                                writeData(0xff);
                                writeDivMmcWriteData(SD_SPI_READ, 0xff);
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
                }
            }
        }
    }
}
