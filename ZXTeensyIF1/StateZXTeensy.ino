#include <SD.h>
#include <SdFat.h>

// State files are standard, uncompressed Z80 v3 snapshots. This lets startup
// restoration reuse the existing snapshot loader and keeps the on-card format
// useful outside ZXTeensy.
static const uint32_t STATE_48_FILE_SIZE = 49248;
static const uint32_t STATE_128_FILE_SIZE = 131183;
static const uint8_t STATE_SIGNAL_FINISH = 0xC0;
static const uint16_t STATE_DEVICE_VERSION = 1;

static const char* const STATE_FILE_NAMES[] = {
    "STATE.Z80", "DIVRAM.BIN", "DIVEXT.BIN", "MFRAM.BIN", "DEVICE.BIN"
};
static const uint8_t STATE_FILE_COUNT =
    sizeof(STATE_FILE_NAMES) / sizeof(STATE_FILE_NAMES[0]);

typedef struct __attribute__((packed)) {
    // Saved-record metadata.
    char magic[4];
    uint16_t version;
    uint16_t size;

    // Configuration normally loaded from ZXTEENSY.cfg.
    uint8_t divMmcPresent;
    uint8_t divMmcExtRamPresent;
    uint8_t divMmcRomPresent;
    uint8_t interface1Present;
    uint8_t mf128Present;
    uint8_t uartPresent;
    uint8_t usbPresent;
    uint8_t gamepadButtons;
    uint8_t wifiNtpPresent;
    uint8_t wifiNtpTz;
    uint8_t dskPresent;
    uint8_t modemPresent;
    uint8_t printerPresent;
    uint8_t lprintPresent;
    uint8_t bootIntoMenu;
    uint8_t menuEnableInGame;
    cfg_data_t cfgData;

    // Runtime device state.
    uint32_t romPaged;
    uint16_t romArrayPresent;
    uint8_t divMmcRamBank;
    uint8_t mf128VideoRam;
    uint8_t spectrumPort1ffd;
    uint8_t spectrumBorder;
    uint8_t rom1Paged;
    uint8_t rom23Paged;
    uint8_t interface1Enabled;
    uint8_t divMmcEnabled;
    uint8_t divMmcRomEnabled;
    uint8_t divMmcToggle;
    uint8_t divMmcAutoMap;
    uint8_t divMmcConMem;
    uint8_t divMmcMapRam;
    uint8_t divMmcRamBankThree;
    uint8_t divMmcExtRamEnabled;
    uint8_t mf128Enabled;
    uint8_t mf128ActiveNMI;
    uint8_t zxC2Present;
    uint8_t zxC3Present;
    uint8_t zxC2Lock;
    uint8_t zxC2ShadowRom;
    uint8_t zxC2BankPtr;
    uint8_t zxC3Write;
    uint8_t zxC3FlashState;
    uint8_t zxC3FlashSetup;
    uint8_t mdrPresent;
    uint8_t mdrEnabled;
    uint8_t mdrMaxSector;
    uint8_t tzxPresent;
    uint8_t tzxEnabled;
    uint8_t dskEnabled;
    uint8_t printerEnabled;
    uint8_t lprintEnabled;
    uint8_t uartEnabled;
    uint8_t modemEnabled;
    uint8_t modemOnReset;
    uint8_t usbEnabled;
    uint8_t printerStrobe;
    uint8_t printerByte;
} state_device_data_t;

File stateSaveFile;
volatile bool stateSaveActive = false;
volatile bool stateSaveBlockPending = false;
volatile uint8_t stateSaveBlockValue = 0;
bool stateSave128 = false;
uint8_t stateSaveExpectedBlock = 0;
state_device_data_t stateRestoreDevice;
volatile bool stateLoadActive = false;
volatile bool stateLoadFinalStage = false;

uint16_t stateReadWord(const volatile uint8_t* data, uint16_t offset)
{
    return data[offset] | (data[offset + 1] << 8);
}

bool stateWriteBytes(const void* data, size_t size)
{
    return stateSaveFile && (stateSaveFile.write((const uint8_t*)data, size) == size);
}

uint8_t stateRegisterByte(uint16_t base, uint8_t offset)
{
    return menuRamArray[1][(base + offset) & (RAM_PAGE_SIZE - 1)];
}

bool stateWriteZ80Header()
{
    // MEM_SPR points below the 16 saved AY words. The Z80 register frame starts
    // 32 bytes above it: IY, IX, BC', DE', HL', AF', BC, DE, RF, IF, HL, AF.
    uint16_t frame = (stateReadWord(menuRamArray[1], Z80_AY_REGS) + 0x20) &
        (RAM_PAGE_SIZE - 1);
    uint16_t pc = stateReadWord(menuRamArray[1], Z80_PC);
    uint16_t sp = stateReadWord(menuRamArray[1], Z80_SP) + 2;
    uint8_t header[87] = {};

    header[0] = stateRegisterByte(frame, 23); // A
    header[1] = stateRegisterByte(frame, 22); // F
    header[2] = stateRegisterByte(frame, 12); // C
    header[3] = stateRegisterByte(frame, 13); // B
    header[4] = stateRegisterByte(frame, 20); // L
    header[5] = stateRegisterByte(frame, 21); // H
    // PC is zero in the base header for a v2/v3 snapshot.
    header[8] = sp;
    header[9] = sp >> 8;
    header[10] = stateRegisterByte(frame, 19); // I
    header[11] = stateRegisterByte(frame, 17); // R
    header[12] = ((spectrumBorder & 0x07) << 1) | ((header[11] >> 7) & 0x01);
    header[13] = stateRegisterByte(frame, 14); // E
    header[14] = stateRegisterByte(frame, 15); // D
    header[15] = stateRegisterByte(frame, 4);  // C'
    header[16] = stateRegisterByte(frame, 5);  // B'
    header[17] = stateRegisterByte(frame, 6);  // E'
    header[18] = stateRegisterByte(frame, 7);  // D'
    header[19] = stateRegisterByte(frame, 8);  // L'
    header[20] = stateRegisterByte(frame, 9);  // H'
    header[21] = stateRegisterByte(frame, 11); // A'
    header[22] = stateRegisterByte(frame, 10); // F'
    header[23] = stateRegisterByte(frame, 0);  // IYL
    header[24] = stateRegisterByte(frame, 1);  // IYH
    header[25] = stateRegisterByte(frame, 2);  // IXL
    header[26] = stateRegisterByte(frame, 3);  // IXH
    bool interruptsEnabled = (stateRegisterByte(frame, 18) & 0x04) != 0;
    header[27] = interruptsEnabled ? 1 : 0;
    header[28] = interruptsEnabled ? 1 : 0;
    header[29] = ((menuRamArray[1][Z80_IM2] & 0x01) != 0) ? 2 : 1;

    // Z80 v3 additional header.
    header[30] = 55;
    header[31] = 0;
    header[32] = pc;
    header[33] = pc >> 8;
    header[34] = stateSave128 ? 4 : 0; // 128K or 48K Spectrum
    header[35] = mf128VideoRam;
    header[36] = 0;                    // Interface 1 ROM not paged
    header[37] = 0;                    // hardware modification
    header[38] = 0;                    // selected AY register
    uint16_t ayStack = stateReadWord(menuRamArray[1], Z80_AY_REGS);
    for (uint8_t reg = 0; reg < 16; ++reg)
    {
        // Registers were pushed 15..0, so register 0 is the first stack word.
        header[39 + reg] = menuRamArray[1][(ayStack + (reg * 2) + 1) &
            (RAM_PAGE_SIZE - 1)];
    }
    header[86] = spectrumPort1ffd;
    return stateWriteBytes(header, sizeof(header));
}

bool statePrepareDirectories(uint8_t slot, char* path)
{
    if (!SD.exists("/ZXTEENSY") && !SD.mkdir("/ZXTEENSY"))
    {
        return false;
    }
    if (!SD.exists(STATE_ROOT_PATH) && !SD.mkdir(STATE_ROOT_PATH))
    {
        return false;
    }
    char slotPath[MAX_PATH];
    snprintf(slotPath, MAX_PATH, "%s/%d", STATE_ROOT_PATH, slot);
    if (!SD.exists(slotPath) && !SD.mkdir(slotPath))
    {
        return false;
    }
    snprintf(path, MAX_PATH, "%s/STATE.Z80", slotPath);
    return true;
}

void stateBuildSlotPath(char* path, uint8_t slot, const char* filename)
{
    snprintf(path, MAX_PATH, "%s/%d/%s", STATE_ROOT_PATH, slot, filename);
}

bool stateWriteFile(uint8_t slot, const char* filename,
    const volatile void* data, size_t size)
{
    bool success = false;
    char path[MAX_PATH];
    stateBuildSlotPath(path, slot, filename);
    File file = SD.open(path, FILE_WRITE_BEGIN);
    if (file)
    {
        if (file.write((const uint8_t*)data, size) >= size)
        {
            success = true;
        }
        file.close();
    }
    if (!success)
    {
        SD.remove(path);
    }
    return success;
}

bool stateReadFile(uint8_t slot, const char* filename,
    volatile void* data, size_t size)
{
    bool success = false;
    char path[MAX_PATH];
    stateBuildSlotPath(path, slot, filename);
    File file = SD.open(path, FILE_READ);
    if (file)
    {
        if (file.read((uint8_t*)data, size) >= size)
        {
            success = true;
        }
        file.close();
    }
    return success;
}

uint8_t stateGetDivMmcRamBank()
{
    for (uint8_t bank = 0; bank < RAM_PAGE_COUNT; ++bank)
    {
        if (divMmcRamPtr == divMmcRamArray[bank])
        {
            return bank;
        }
    }
    for (uint8_t bank = 0; bank < EXT_RAM_PAGE_COUNT; ++bank)
    {
        if (divMmcRamPtr == divMmcExtRamArray[bank])
        {
            return RAM_PAGE_COUNT + bank;
        }
    }
    return 0;
}

void stateCaptureDeviceData(void* data)
{
    state_device_data_t* state = (state_device_data_t*)data;
    memset(state, 0, sizeof(*state));
    memcpy(state->magic, "ZXST", 4);
    state->version = STATE_DEVICE_VERSION;
    state->size = sizeof(*state);
    state->romPaged = romPaged & ~((1UL << ROM_MENU) | (1UL << ROM_SNA));
    state->romArrayPresent = romArrayPresent;
    state->divMmcRamBank = stateGetDivMmcRamBank();
    state->mf128VideoRam = mf128VideoRam;
    state->spectrumPort1ffd = spectrumPort1ffd;
    state->spectrumBorder = spectrumBorder;
    state->rom1Paged = rom1Paged;
    state->rom23Paged = rom23Paged;
    state->interface1Enabled = interface1Enabled;
    state->divMmcEnabled = divMmcEnabled;
    state->divMmcRomEnabled = divMmcRomEnabled;
    state->divMmcToggle = divMmcToggle;
    state->divMmcAutoMap = divMmcAutoMap;
    state->divMmcConMem = divMmcConMem;
    state->divMmcMapRam = divMmcMapRam;
    state->divMmcRamBankThree = divMmcRamBankThree;
    state->divMmcExtRamEnabled = divMmcExtRamEnabled;
    state->mf128Enabled = mf128Enabled;
    state->mf128ActiveNMI = mf128ActiveNMI;
    state->zxC2Present = zxC2Present;
    state->zxC3Present = zxC3Present;
    state->zxC2Lock = zxC2Lock;
    state->zxC2ShadowRom = zxC2ShadowRom;
    state->zxC2BankPtr = zxC2BankPtr;
    state->zxC3Write = zxC3Write;
    state->zxC3FlashState = zxC3FlashState;
    state->zxC3FlashSetup = zxC3FlashSetup;
    state->mdrPresent = mdrPresent;
    state->mdrEnabled = mdrEnabled;
    state->mdrMaxSector = mdrMaxSector;
    state->tzxPresent = tzxPresent;
    state->tzxEnabled = tzxEnabled;
    state->dskEnabled = dskEnabled;
    state->printerEnabled = printerEnabled;
    state->lprintEnabled = lprintEnabled;
    state->uartEnabled = uartEnabled;
    state->modemEnabled = modemEnabled;
    state->modemOnReset = modemOnReset;
    state->usbEnabled = usbEnabled;
    state->printerStrobe = printerStrobe;
    state->printerByte = printerByte;
    state->divMmcPresent = divMmcPresent;
    state->divMmcExtRamPresent = divMmcExtRamPresent;
    state->divMmcRomPresent = divMmcRomPresent;
    state->interface1Present = interface1Present;
    state->mf128Present = mf128Present;
    state->uartPresent = uartPresent;
    state->usbPresent = usbPresent;
    state->gamepadButtons = gamepadButtons;
    state->wifiNtpPresent = wifiNtpPresent;
    state->wifiNtpTz = wifiNtpTz;
    state->dskPresent = dskPresent;
    state->modemPresent = modemPresent;
    state->printerPresent = printerPresent;
    state->lprintPresent = lprintPresent;
    state->bootIntoMenu = bootIntoMenu;
    state->menuEnableInGame = menuEnableInGame;
    memcpy(&state->cfgData, &cfgData, sizeof(state->cfgData));
}

void stateCloseOtherHandles()
{
    // Ensure files are closed, ready to save state
    httpStopServer();
    if (printerEnabled || lprintEnabled)
    {
        printerPort.end();
    }
    if (dskEnabled)
    {
        dskController.end();
    }
    divMmcHdf.end();
    divMmcSecondHdf.end();
    beginSdfsSd();
}

bool stateBeginSave(uint8_t slot)
{
    if (!stateSaveActive && (slot < STATE_SLOT_COUNT))
    {
        // Detect 128K mode, and if the paging is locked or +3 AllRam
        stateSave128 = ((romArrayPresent & BANK_ROM1) != 0);
        if (stateSave128 && (((mf128VideoRam & 0x20) != 0) ||
            ((spectrumPort1ffd & 0x01) != 0)))
        {
            return false;
        }

        // Update the menu configuration, and then prepare to save state
        menuSaveConfiguration();
        stateCloseOtherHandles();
        char path[MAX_PATH];
        if (statePrepareDirectories(slot, path))
        {
            stateSaveFile = SD.open(path, FILE_WRITE_BEGIN);
            if (stateSaveFile)
            {
                if (stateWriteZ80Header())
                {
                    stateSaveExpectedBlock = 0;
                    stateSaveBlockPending = false;
                    stateSaveActive = true;
                    stateSaveSlot = slot;
                    menuBuffer.write(stateSave128 ? MENU_ROM_CMD_STATE_CAPTURE_128 :
                        MENU_ROM_CMD_STATE_CAPTURE_48);
                    return true;
                } else {
                    // File header write failed
                    stateSaveFile.close();
                    stateSaveActive = false;
                    SD.remove(path);
                }
            }
        }
    }
    stateResumeClosedDevices();
    return false;
}

bool isStateSaveActive()
{
    return stateSaveActive;
}

void stateSaveBlock(uint8_t block)
{
    if (stateSaveActive && !stateSaveBlockPending)
    {
        stateSaveBlockValue = block;
        stateSaveBlockPending = true;
    }
}

uint8_t statePageForBlock(uint8_t block)
{
    static const uint8_t pages48[3] = { 8, 4, 5 };
    return stateSave128 ? (3 + block) : pages48[block];
}

void stateResumeClosedDevices()
{
    if (dskEnabled)
    {
        dskController.begin(menuGetFdcFdaPath(), menuGetFdcFdbPath());
    }
    if (printerEnabled || lprintEnabled)
    {
        printerPort.begin();
    }
}

void stateDeleteSlot()
{
    char path[MAX_PATH];
    for (uint8_t index = 0; index < STATE_FILE_COUNT; ++index)
    {
        stateBuildSlotPath(path, stateSaveSlot, STATE_FILE_NAMES[index]);
        if (SD.exists(path))
        {
            SD.remove(path);
        }
    }
}

void stateFinishSave(bool success)
{
    stateSaveFile.close();
    if (success)
    {
        char path[MAX_PATH];
        stateBuildSlotPath(path, stateSaveSlot, "STATE.Z80");
        File verifyFile = SD.open(path, FILE_READ);
        if (verifyFile)
        {
            uint32_t expectedSize = stateSave128 ? STATE_128_FILE_SIZE : STATE_48_FILE_SIZE;
            success = (verifyFile.size() == expectedSize);
            verifyFile.close();
        } else {
            success = false;
        }
    }
    if (success)
    {
        // Save the DivMMC RAM, ROM banking and peripheral state
        stateCaptureDeviceData(&stateRestoreDevice);
        success = (stateWriteFile(stateSaveSlot, "DIVRAM.BIN",
                divMmcRamArray, (RAM_PAGE_COUNT * RAM_PAGE_SIZE)) &&
            stateWriteFile(stateSaveSlot, "DIVEXT.BIN",
                divMmcExtRamArray, (EXT_RAM_PAGE_COUNT * RAM_PAGE_SIZE)) &&
            stateWriteFile(stateSaveSlot, "MFRAM.BIN",
                &romArray[ROM_PAGE_MF128][RAM_PAGE_SIZE], RAM_PAGE_SIZE) &&
            stateWriteFile(stateSaveSlot, "DEVICE.BIN",
                &stateRestoreDevice, sizeof(stateRestoreDevice)));
    }
    if (success)
    {
        // Store the saved slot to be loaded at boot
        stateActiveSlot = stateSaveSlot;
        menuConfigChanged = true;
        menuSaveConfiguration();
    } else {
        // Ensure the active slot is cleared on failure
        menuPrintDebug(false, "Failed to save state %d", stateActiveSlot);
        stateActiveSlot = -1;
        stateDeleteSlot();
    }

    // Finished saving the state
    stateSaveActive = false;
    stateResumeClosedDevices();
    menuBuffer.write(success ? MENU_ROM_CMD_STATE_COMPLETE : MENU_ROM_CMD_STATE_FAILED);
    menuRedraw = true;
}

void stateApplyConfiguration()
{
    // Restore the configuration
    divMmcPresent = stateRestoreDevice.divMmcPresent;
    divMmcExtRamPresent = stateRestoreDevice.divMmcExtRamPresent;
    divMmcRomPresent = stateRestoreDevice.divMmcRomPresent;
    interface1Present = stateRestoreDevice.interface1Present;
    mf128Present = stateRestoreDevice.mf128Present;
    uartPresent = stateRestoreDevice.uartPresent;
    usbPresent = stateRestoreDevice.usbPresent;
    gamepadButtons = stateRestoreDevice.gamepadButtons;
    wifiNtpPresent = stateRestoreDevice.wifiNtpPresent;
    wifiNtpTz = stateRestoreDevice.wifiNtpTz;
    dskPresent = stateRestoreDevice.dskPresent;
    modemPresent = stateRestoreDevice.modemPresent;
    printerPresent = stateRestoreDevice.printerPresent;
    lprintPresent = stateRestoreDevice.lprintPresent;
    bootIntoMenu = stateRestoreDevice.bootIntoMenu;
    menuEnableInGame = stateRestoreDevice.menuEnableInGame;
    memcpy(&cfgData, &stateRestoreDevice.cfgData, sizeof(cfgData));
    cfgData.cfgName[MAX_PATH - 1] = 0;
    cfgData.romName[MAX_PATH - 1] = 0;
    cfgData.divMmcSdaPath[MAX_PATH - 1] = 0;
    cfgData.divMmcSdbPath[MAX_PATH - 1] = 0;
    cfgData.dskFdaPath[MAX_PATH - 1] = 0;
    cfgData.dskFdbPath[MAX_PATH - 1] = 0;
    cfgData.modemUrl[MAX_PATH - 1] = 0;
}

void stateApplyDeviceData()
{
    // Restore the enable flags
    dskEnabled = stateRestoreDevice.dskEnabled;
    printerEnabled = stateRestoreDevice.printerEnabled;
    lprintEnabled = stateRestoreDevice.lprintEnabled;
    uartEnabled = stateRestoreDevice.uartEnabled;
    modemEnabled = stateRestoreDevice.modemEnabled;
    modemOnReset = stateRestoreDevice.modemOnReset;
    usbEnabled = stateRestoreDevice.usbEnabled;
    printerStrobe = stateRestoreDevice.printerStrobe;
    printerByte = stateRestoreDevice.printerByte;
    snaLoaderPresent = false;

    // Restore ROM banking and peripheral state
    romArrayPresent = (romArrayPresent & ~BANK_RAM) |
        (stateRestoreDevice.romArrayPresent & BANK_RAM);
    romPaged = stateRestoreDevice.romPaged;
    rom1Paged = stateRestoreDevice.rom1Paged;
    rom23Paged = stateRestoreDevice.rom23Paged;
    spectrumPort1ffd = stateRestoreDevice.spectrumPort1ffd;
    spectrumBorder = stateRestoreDevice.spectrumBorder;
    interface1Enabled = stateRestoreDevice.interface1Enabled;
    divMmcEnabled = stateRestoreDevice.divMmcEnabled;
    divMmcRomEnabled = stateRestoreDevice.divMmcRomEnabled;
    divMmcToggle = stateRestoreDevice.divMmcToggle;
    divMmcConMem = stateRestoreDevice.divMmcConMem;
    divMmcAutoMap = stateRestoreDevice.divMmcAutoMap;
    divMmcMapRam = stateRestoreDevice.divMmcMapRam;
    divMmcExtRamEnabled = stateRestoreDevice.divMmcExtRamEnabled;
    mf128Enabled = stateRestoreDevice.mf128Enabled;
    mf128VideoRam = stateRestoreDevice.mf128VideoRam;
    mf128ActiveNMI = stateRestoreDevice.mf128ActiveNMI;
    zxC2Present = stateRestoreDevice.zxC2Present;
    zxC3Present = stateRestoreDevice.zxC3Present;
    zxC2Lock = stateRestoreDevice.zxC2Lock;
    zxC2ShadowRom = stateRestoreDevice.zxC2ShadowRom;
    zxC2BankPtr = stateRestoreDevice.zxC2BankPtr;
    zxC3Write = stateRestoreDevice.zxC3Write;
    zxC3FlashState = (zxc3_flash_state_t)stateRestoreDevice.zxC3FlashState;
    zxC3FlashSetup = stateRestoreDevice.zxC3FlashSetup;
    mdrPresent = stateRestoreDevice.mdrPresent;
    mdrEnabled = stateRestoreDevice.mdrEnabled;
    mdrMaxSector = stateRestoreDevice.mdrMaxSector;
    tzxPresent = stateRestoreDevice.tzxPresent;
    tzxEnabled = false;

    // Restore the DivMMC RAM pointer
    uint8_t bank = stateRestoreDevice.divMmcRamBank;
    divMmcRamBankThree = (bank == 3);
    if ((bank >= RAM_PAGE_COUNT) &&
        (bank < (RAM_PAGE_COUNT + EXT_RAM_PAGE_COUNT)))
    {
        divMmcRamPtr = divMmcExtRamArray[bank - RAM_PAGE_COUNT];
    } else {
        divMmcRamPtr = divMmcRamArray[bank & (RAM_PAGE_COUNT - 1)];
    }

    // Page in ROMs
    updateRomIndex(true);
}

bool stateReadDeviceData(uint8_t slot)
{
    // Validate the header of the "DEVICE.BIN" image
    if (stateReadFile(slot, "DEVICE.BIN", &stateRestoreDevice,
        sizeof(stateRestoreDevice)))
    {
        return (memcmp(stateRestoreDevice.magic, "ZXST", 4) == 0) &&
            (stateRestoreDevice.version == STATE_DEVICE_VERSION) &&
            (stateRestoreDevice.size == sizeof(stateRestoreDevice));
    }
    return false;
}

void stateLoaderFinished()
{
    if (stateLoadActive)
    {
        stateLoadFinalStage = true;
    }
}

bool stateLoadOnStartup()
{
    bool restored = false;
    if ((stateActiveSlot >= 0) && (stateActiveSlot < STATE_SLOT_COUNT) &&
        stateReadDeviceData(stateActiveSlot))
    {
        char statePath[MAX_PATH];
        stateBuildSlotPath(statePath, stateActiveSlot, "STATE.Z80");
        File stateFile = SD.open(statePath, FILE_READ);
        if (loadSnapshotFile(stateFile, false) &&
            stateReadFile(stateActiveSlot, "DIVRAM.BIN", divMmcRamArray,
                (RAM_PAGE_COUNT * RAM_PAGE_SIZE)) &&
            stateReadFile(stateActiveSlot, "MFRAM.BIN",
                &romArray[ROM_PAGE_MF128][RAM_PAGE_SIZE], RAM_PAGE_SIZE))
        {
            stateApplyConfiguration();
            stateLoadActive = true;
            restored = true;
        }
    }
    if (!restored)
    {
        stateActiveSlot = -1;
    }
    return restored;
}

void stateOnTick()
{
    if (stateLoadFinalStage)
    {
        bool restored = false;
        stateLoadFinalStage = false;

        // Restore the DivMMC RAM used by the Z80 loader, then restore ROM
        // banking and peripheral state
        if (beginSdfsSd() &&
            stateReadFile(stateActiveSlot, "DIVEXT.BIN", divMmcExtRamArray,
                (EXT_RAM_PAGE_COUNT * RAM_PAGE_SIZE)))
        {
            stateApplyDeviceData();
            restored = true;
        }

        // Clear the restore slot on success
        if (restored)
        {
            stateActiveSlot = -1;
            menuConfigChanged = true;
            menuSaveConfiguration();
        }
        stateLoadActive = false;
    }

    if (stateSaveActive && stateSaveBlockPending)
    {
        stateSaveBlockPending = false;
        uint8_t block = stateSaveBlockValue;
        if (block < STATE_SIGNAL_FINISH)
        {
            block &= 0x3F;
            if (block != stateSaveExpectedBlock)
            {
                stateFinishSave(false);
            } else {
                uint8_t blockHeader[3] = { 0xFF, 0xFF, statePageForBlock(block) };
                if (!stateWriteBytes(blockHeader, sizeof(blockHeader)))
                {
                    stateFinishSave(false);
                    return;
                }

                // The in-game menu replaced the visible screen. Patch bank 5 from the
                // copy made by the NMI entry code before writing the bank.
                bool isBank5 = stateSave128 ? (block == 5) : (block == 0);
                if (isBank5)
                {
                    memcpy((void*)menuRamArray[2], (void*)&menuRamArray[1][0x500], 0x1B00);
                }
                if (stateWriteBytes((const void*)menuRamArray[2], RAM_PAGE_SIZE) &&
                    stateWriteBytes((const void*)menuRamArray[3], RAM_PAGE_SIZE))
                {
                    ++stateSaveExpectedBlock;
                    menuBuffer.write(MENU_ROM_CMD_STATE_BLOCK_DONE);
                } else {
                    stateFinishSave(false);
                }
            }
        } else {
            // Ensure all 16K banks have been received.
            stateFinishSave(stateSaveExpectedBlock == (stateSave128 ? 8 : 3));
        }
    }
}
