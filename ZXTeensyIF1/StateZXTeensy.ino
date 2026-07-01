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
    uint8_t spectrumBankM;
    uint8_t spectrumBank678;
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
    uint8_t divMmcRamBank;
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
    uint8_t mousePresent;
    uint8_t joystickPresent;
    uint8_t printerStrobe;
    uint8_t printerByte;
    uint8_t divMmcSpiSdIdle;
    uint8_t divMmcHdfSdIdle;
    uint8_t divMmcSecondHdfSdIdle;
    uint32_t mouseX;
    uint32_t mouseY;
    uint32_t tapePosition;
    uint32_t tapeLength;
} state_device_data_t;

File stateSaveFile;
volatile bool stateSaveActive = false;
volatile bool stateSaveBlockPending = false;
volatile uint8_t stateSaveBlockValue = 0;
bool stateSave128 = false;
uint8_t stateSaveExpectedBlock = 0;
DMAMEM state_device_data_t stateRestoreDevice;
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
    uint16_t frame = (stateReadWord(menuRamArray[1], MEM_SPR) + 0x20) &
        (RAM_PAGE_SIZE - 1);
    uint16_t pc = stateReadWord(menuRamArray[1], MEM_PC);
    uint16_t sp = stateReadWord(menuRamArray[1], MEM_SP2) + 2;
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
    header[29] = ((menuRamArray[1][MEM_IM2] & 0x01) != 0) ? 2 : 1;

    // Z80 v3 additional header.
    header[30] = 55;
    header[31] = 0;
    header[32] = pc;
    header[33] = pc >> 8;
    header[34] = (stateSave128 ? 4 : 0); // 128K or 48K Spectrum
    header[35] = spectrumBankM;
    header[36] = (IS_ROM_PAGED(ROM_IF1) ? 0xFF : 0x00); // Interface 1 ROM paged
    header[37] = 0; // hardware modification
    header[38] = spectrumAyReg; // selected AY register
    uint16_t ayStack = stateReadWord(menuRamArray[1], MEM_SPR) &
        (RAM_PAGE_SIZE - 1);
    for (uint8_t reg = 0; reg < 16; ++reg)
    {
        // Registers were pushed 15..0, so register 0 is the first stack word.
        header[39 + reg] = menuRamArray[1][(ayStack + (reg * 2) + 1)];
    }
    header[86] = spectrumBank678;
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

bool stateReadDivMmcExtRam(uint8_t slot)
{
    // NOTE: The Z80 loader resides in the first 128K of divMmcExtRamArray,
    // and will be replaced in the final loader stage
    static const size_t preservedSize = (RAM_PAGE_COUNT * RAM_PAGE_SIZE);
    static const size_t restoreSize = (RAM_PAGE_SIZE *
        (EXT_RAM_PAGE_COUNT - RAM_PAGE_COUNT));

    bool success = false;
    char path[MAX_PATH];
    stateBuildSlotPath(path, slot, "DIVEXT.BIN");
    File file = SD.open(path, FILE_READ);
    if (file)
    {
        if (file.seek(preservedSize, SeekSet) &&
            (file.read((uint8_t*)divMmcExtRamArray[RAM_PAGE_COUNT],
                restoreSize) >= restoreSize))
        {
            success = true;
        }
        file.close();
    }
    return success;
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
    state->spectrumBankM = spectrumBankM;
    state->spectrumBank678 = spectrumBank678;
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
    state->divMmcRamBank = divMmcRamBank;
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
    state->mousePresent = mousePresent;
    state->joystickPresent = joystickPresent;
    state->printerStrobe = printerStrobe;
    state->printerByte = printerByte;
    state->divMmcSpiSdIdle = divMmcSpi.getSdIdle();
    state->divMmcHdfSdIdle = divMmcHdf.getSdIdle();
    state->divMmcSecondHdfSdIdle = divMmcSecondHdf.getSdIdle();
    state->mouseX = mouseX;
    state->mouseY = mouseY;
    size_t tapeLength;
    state->tapePosition = tzxPlayer.getPosition(&tapeLength);
    state->tapeLength = tapeLength;
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
        // The menu ROM probes whether 0x7FFD changes the RAM at 0xC000, to
        // determine if a 48K or 128K snapshot is needed
        uint8_t stateMode = menuRamArray[0][MEM_MODE];
        stateSave128 = (stateMode == Z80_MODE_128);
        if ((spectrumBank678 & 0x01) != 0)
        {
            // +3 All-Ram mode is not supported
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
    // Return Z80 file "page" for the given block number
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
    cfgData.romName[MAX_PATH - 1] = 0;
    cfgData.divMmcSdaPath[MAX_PATH - 1] = 0;
    cfgData.divMmcSdbPath[MAX_PATH - 1] = 0;
    cfgData.dskFdaPath[MAX_PATH - 1] = 0;
    cfgData.dskFdbPath[MAX_PATH - 1] = 0;
    cfgData.modemUrl[MAX_PATH - 1] = 0;

    // Clear the configuration name as this changes configuration
    cfgData.cfgName[0] = 0;
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
    mousePresent = stateRestoreDevice.mousePresent;
    joystickPresent = stateRestoreDevice.joystickPresent;
    mouseX = stateRestoreDevice.mouseX;
    mouseY = stateRestoreDevice.mouseY;
    printerStrobe = stateRestoreDevice.printerStrobe;
    printerByte = stateRestoreDevice.printerByte;
    divMmcSpi.setSdIdle(stateRestoreDevice.divMmcSpiSdIdle);
    divMmcHdf.setSdIdle(stateRestoreDevice.divMmcHdfSdIdle);
    divMmcSecondHdf.setSdIdle(stateRestoreDevice.divMmcSecondHdfSdIdle);

    // Restore peripheral stated
    rom1Paged = stateRestoreDevice.rom1Paged;
    rom23Paged = stateRestoreDevice.rom23Paged;
    spectrumBank678 = stateRestoreDevice.spectrumBank678;
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
    spectrumBankM = stateRestoreDevice.spectrumBankM;
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
    if (stateRestoreDevice.tzxEnabled && (stateRestoreDevice.tapeLength > 0))
    {
        tzxPlayer.begin(divMmcExtRamArray[0], stateRestoreDevice.tapeLength);
        tzxPlayer.setPosition(stateRestoreDevice.tapePosition);
        tzxEnabled = true;
    } else {
        tzxEnabled = false;
    }

    // Restore the DivMMC RAM pointer
    divMmcRamBank = stateRestoreDevice.divMmcRamBank;
    if (divMmcExtRamEnabled && (divMmcRamBank >= RAM_PAGE_COUNT))
    {
        divMmcRamPtr = divMmcExtRamArray[(divMmcRamBank - RAM_PAGE_COUNT)];
        divMmcRamBankThree = false;
    } else {
        divMmcRamPtr = divMmcRamArray[divMmcRamBank & (RAM_PAGE_COUNT - 1)];
        divMmcRamBankThree = ((divMmcRamBank == 0x03) ? true : false);
    }
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

void stateLoaderFinished(bool onPageOut)
{
    if (stateLoadActive)
    {
        if (onPageOut)
        {
            // Restore final ROM banking state
            // Snapshot loader is being paged out, as fully completed
            romArrayPresent = (romArrayPresent & ~BANK_RAM) |
                (stateRestoreDevice.romArrayPresent & BANK_RAM);
            romPaged = stateRestoreDevice.romPaged;

            // Snapshot loader completed
            stateLoadActive = false;
        } else {
            // Loader is entering final stage
            stateLoadFinalStage = true;
        }
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
                &romArray[ROM_PAGE_MF128][RAM_PAGE_SIZE], RAM_PAGE_SIZE) &&
            stateReadDivMmcExtRam(stateActiveSlot))
        {
            // Apply the saved configuration
            stateApplyConfiguration();
            stateLoadActive = true;
            restored = true;

            // Copy the loader into scratch RAM
            memcpy((void*)menuRamArray[2], (void*)divMmcExtRamArray[0], ROM_PAGE_SIZE);
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

        // Restore the DivMMC RAM used by the Z80 loader
        if (beginSdfsSd() &&
            stateReadFile(stateActiveSlot, "DIVEXT.BIN", divMmcExtRamArray,
                (RAM_PAGE_COUNT * RAM_PAGE_SIZE)))
        {
            restored = true;
        }

        // Update the quick save slot on success
        if (restored)
        {
            stateSaveSlot = stateActiveSlot;
        }

        // Clear the restore saved state slot
        stateActiveSlot = -1;
        menuConfigChanged = true;

        // Restore the banking and peripheral state
        if (restored)
        {
            menuSaveConfiguration();
            stateApplyDeviceData();
        } else {
            // Return to menu on final stage failure
            setState(STATE_RESET_MENU);
        }
    }

    if (stateSaveBlockPending)
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
                uint8_t page = statePageForBlock(block);
                uint8_t blockHeader[3] = { 0xFF, 0xFF, page };
                if (!stateWriteBytes(blockHeader, sizeof(blockHeader)))
                {
                    stateFinishSave(false);
                    return;
                }

                // The in-game menu replaced the visible screen - patch "page 8"
                // with address 0x4000 with the shadow copy from scratch RAM
                if (page == 8)
                {
                    memcpy((void*)menuRamArray[2], (void*)&menuRamArray[1][0x500], 0x1B00);
                }
                if (stateWriteBytes((const void*)menuRamArray[2], ROM_PAGE_SIZE))
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
