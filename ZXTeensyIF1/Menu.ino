
#define FLASH_FILENAME "ZXTEENSY.HEX"
#define INTERNAL_ROM_NAME ":INTERNAL"
#define ROM_NAME_LEN 32

typedef enum {
    MENU_ACTION_REFRESH,
    MENU_ACTION_LOAD_ROM,
    MENU_ACTION_LOAD_ZXC2,
    MENU_ACTION_UPDATE_FW
} menu_action_t;

typedef struct {
    char divMmcPresent;
    char interface1Present;
    char mf128Present;
    char bootIntoMenu;
    char romName[(ROM_NAME_LEN + 1)];
} cfg_data_t;

uint8_t menuLine = 0;
uint8_t menuTotalLines = 0;
uint8_t menuPage = 0;
char* menuPtr = 0;
char* menuSettingsPtr = 0;
bool menuConfigChanged = false;
bool menuConfigReload = true;
bool menuHasUpdateFw = false;

// Configuration data
DMAMEM cfg_data_t cfgData;

// "Restart" is always index 0
// "Use internal ROM" is always index (menuRomListIndex - 1)
volatile uint8_t menuRomListIndex = 0;
volatile menu_action_t menuAction = MENU_ACTION_REFRESH;

void generateMenu(volatile uint8_t* romPtr)
{
    // Build the menu
    menuPtr = (char*)romPtr + RAM_PAGE_SIZE;
    char* endPtr = menuPtr + RAM_PAGE_SIZE - 36;
    char* ptr = menuAddSetting(menuPtr, "Save and Restart", 0);
    ptr = menuAddSetting(ptr, "Disable and Restart", 0);

    // Add firmware update option, if available
    File fwUpdateFile = SD.open(FLASH_FILENAME, FILE_READ);
    if (fwUpdateFile)
    {
        menuHasUpdateFw = true;
        fwUpdateFile.close();
        ptr = menuAddSetting(ptr, "Update firmware and Restart", 0);
    } else {
        menuHasUpdateFw = false;
        ptr = menuAddSetting(ptr, "", 0);
    }

    // Add settings menu, that is refreshed on MENU_ACTION_REFRESH
    menuSettingsPtr = ptr;
    ptr = menuGenerateSettings(ptr);

    // Add disable option

    // List ROM files
    ptr = menuAddSetting(ptr, "Internal ROM",
        (stricmp(cfgData.romName, INTERNAL_ROM_NAME) == 0));
    menuRomListIndex = menuTotalLines;
    File romDirectory = SD.open("ROMS", FILE_READ);
    if (romDirectory)
    {
        if (romDirectory.isDirectory())
        {
            while (true)
            {
                File entry = romDirectory.openNextFile();
                if (entry)
                {
                    if (!entry.isDirectory())
                    {
                        ptr = menuAddFile(ptr, entry.name());
                    }
                    entry.close();
                } else {
                    // End of listing
                    break;
                }

                // End of menu check
                if ((ptr > endPtr) || (menuTotalLines == 0xFF))
                {
                    break;
                }
            }
        }
        romDirectory.close();
    }

    // Write the menu dimensions
    uint16_t address = ((romPtr[0x0FFC] << 8) + romPtr[0x0FFB]);
    romPtr[address] = (menuTotalLines - 1);
    address = ((romPtr[0x0FFF] << 8) + romPtr[0x0FFE]);
    romPtr[address] = (menuPage + 1);
}

char* menuGenerateSettings(char* ptr)
{
    // Add settings menu as first options
    ptr = menuAddSetting(ptr, "Boot into Menu", bootIntoMenu);
    if ((romArrayPresent & BANK_DIVMMC) != 0)
    {
        ptr = menuAddSetting(ptr, "Enable DivMMC", divMmcPresent);
    } else {
        ptr = menuAddSetting(ptr, "DivMMC not present", 0);
    }
    if ((romArrayPresent & BANK_IF1) != 0)
    {
        ptr = menuAddSetting(ptr, "Enable Interface 1", interface1Present);
    } else {
        ptr = menuAddSetting(ptr, "Interface 1 not present", 0);
    }
    if ((romArrayPresent & BANK_MF128) != 0)
    {
        ptr = menuAddSetting(ptr, "Enable Multiface 128", mf128Present);
    } else {
        ptr = menuAddSetting(ptr, "Multiface 128 not present", 0);
    }
    return ptr;
}

char* menuAddSetting(char* ptr, const char* label, bool checked)
{
    *ptr++ = (checked ? 27 : 26);
    unsigned int len = strlen(label);
    ptr = strncpy(ptr, label, len) + len;
    if (menuLine < 21)
    {
        *ptr = 10;
        ++menuLine;
    } else {
        menuLine = 0;
        ++menuPage;
    }
    ++menuTotalLines;
    return (ptr+1);
}

char* menuAddFile(char* ptr, const char* filename)
{
    *ptr++ = ((stricmp(filename, cfgData.romName) == 0) ? 27 : 26);
    unsigned int len = strlen(filename);

    // Find the file extension
    bool hasIcon = false;
    char *fileext = strrchr(filename, '.');
    if (fileext != 0)
    {
        if (stricmp(fileext + 1, "bin") == 0)
        {
            len = (fileext - filename);
            hasIcon = true;
        } else if (stricmp(fileext + 1, "rom") == 0)
        {
            len = (fileext - filename);
        }
    }

    // Truncate the file name
    if (len > 32)
    {
        len = 32;
    }
    strncpy(ptr, filename, len);
    ptr += len;

    // Add icon
    if (hasIcon)
    {
        *ptr++ = 9;
        *ptr++ = 28;
        *ptr++ = 29;
    }

    // Add new line, and update menu dimensions
    if (menuLine < 20)
    {
        *ptr = 10;
        ++menuLine;
    } else {
        *ptr = 0;
        menuLine = 0;
        ++menuPage;
    }
    ++menuTotalLines;
    return (ptr+1);
}

bool menuPerformSelection(uint8_t index)
{
    if (index >= menuTotalLines)
    {
        afterFirstReset = false;
        menuAction = MENU_ACTION_LOAD_ROM;
        return true;
    } else if (index >= menuRomListIndex)
    {
        // Update the ROM name for the selection, and reset
        switch (updateRomName(index - menuRomListIndex))
        {
            case TYPE_ROM :
                menuAction = MENU_ACTION_LOAD_ROM;
                menuConfigChanged = true;
                break;
            default :
                menuAction = MENU_ACTION_LOAD_ZXC2;
                break;
        }
        return true;
    } else {
        switch (index)
        {
            case 0 :
                // Reload the existing ROM name, and reset
                menuAction = MENU_ACTION_LOAD_ROM;
                return true;
            case 1 :
                // Temporarily disable device, and reset
                menuAction = MENU_ACTION_LOAD_ROM;
                isDeviceDisabled = true;
                return true;
            case 2 :
                // Perform firmware update, if available
                if (menuHasUpdateFw)
                {
                    menuAction = MENU_ACTION_UPDATE_FW;
                    return true;
                }
                break;
            case 3 :
                bootIntoMenu = !bootIntoMenu;
                menuConfigChanged = true;
                break;
            case 4 :
                if ((romArrayPresent & BANK_DIVMMC) != 0)
                {
                    divMmcPresent = !divMmcPresent;
                    menuConfigChanged = true;
                }
                break;
            case 5 :
                if ((romArrayPresent & BANK_IF1) != 0)
                {
                    interface1Present = !interface1Present;
                    menuConfigChanged = true;
                }
                break;
            case 6 :
                if ((romArrayPresent & BANK_MF128) != 0)
                {
                    mf128Present = !mf128Present;
                    menuConfigChanged = true;
                }
                break;
            default :
                // Load internal ROM name
                if (index >= (menuRomListIndex - 1))
                {
                    strncpy(cfgData.romName, INTERNAL_ROM_NAME, ROM_NAME_LEN);
                    cfgData.romName[ROM_NAME_LEN] = 0;
                    menuAction = MENU_ACTION_LOAD_ROM;
                    menuConfigChanged = true;
                    return true;
                }
                break;
        }
    }

    // Refresh the settings menu
    menuAction = MENU_ACTION_REFRESH;
    menuGenerateSettings(menuSettingsPtr);
    return false;
}

void menuPerformAction()
{
    switch (menuAction)
    {
        case MENU_ACTION_UPDATE_FW :
            // Flash the firmware update
            flashUpdate(FLASH_FILENAME);
            break;
        case MENU_ACTION_LOAD_ZXC2 :
            // Load new ROM, without changing the configuration
            menuConfigReload = false;
            break;
        default :
            // Save the configuration to load new ROM
            menuSaveConfiguration();
            menuConfigReload = true;
            break;
    }
}

rom_type_t getRomType(const char* fileName)
{
    char *fileext = strrchr(fileName, '.');
    if (fileext != 0)
    {
        if (stricmp(fileext + 1, "bin") == 0)
        {
            return TYPE_ZXC2;
        } else if (stricmp(fileext + 1, "rom") == 0)
        {
            return TYPE_ROM;
        }
    }
    return TYPE_IF2;
}

rom_type_t updateRomName(uint8_t fileIndex)
{
    rom_type_t romType = TYPE_ROM;
    File romDirectory = SD.open("ROMS", FILE_READ);
    if (romDirectory)
    {
        if (romDirectory.isDirectory())
        {
            uint8_t index = 0;
            while (true)
            {
                File entry = romDirectory.openNextFile();
                if (entry)
                {
                    if (!entry.isDirectory())
                    {
                        // Find ROM at matching list index
                        if (index == fileIndex)
                        {
                            strncpy(cfgData.romName, entry.name(), ROM_NAME_LEN);
                            romType = getRomType(entry.name());
                            entry.close();
                            break;
                        } else {
                            ++index;
                        }
                    }
                    entry.close();
                } else {
                    // End of listing
                    strncpy(cfgData.romName, INTERNAL_ROM_NAME, ROM_NAME_LEN);
                    break;
                }
            }
        }
        romDirectory.close();
    }
    return romType;
}

File menuGetFile(rom_type_t* romType)
{
    if (stricmp(cfgData.romName, INTERNAL_ROM_NAME) != 0)
    {
        File romDirectory = SD.open("ROMS", FILE_READ);
        if (romDirectory)
        {
            if (romDirectory.isDirectory())
            {
                while (true)
                {
                    File entry = romDirectory.openNextFile();
                    if (entry)
                    {
                        if (!entry.isDirectory())
                        {
                            if (stricmp(entry.name(), cfgData.romName) == 0)
                            {
                                *romType = getRomType(entry.name());
                                romDirectory.close();
                                return entry;
                            }
                        }
                        entry.close();
                    } else {
                        // End of listing
                        break;
                    }
                }
            }
            romDirectory.close();
        }
    }

    // Return closed File
    return File();
}

void menuClearConfiguration()
{
    divMmcPresent = false;
    interface1Present = false;
    mf128Present = false;
    bootIntoMenu = true;
    strncpy(cfgData.romName, INTERNAL_ROM_NAME, ROM_NAME_LEN);
    cfgData.romName[ROM_NAME_LEN] = 0;
    menuConfigChanged = true;
}

void menuLoadConfiguration()
{
    // Load the configuration from the SD card, as required
    if (menuConfigReload)
    {
        File cfgFile = SD.open("ZXTEENSY.CFG", FILE_READ);
        if (cfgFile)
        {
            if (cfgFile.readBytes((char*)&cfgData, sizeof(cfgData)) >= 0)
            {
                if ((romArrayPresent & BANK_DIVMMC) != 0)
                {
                    divMmcPresent = cfgData.divMmcPresent;
                }
                if ((romArrayPresent & BANK_IF1) != 0)
                {
                    interface1Present = cfgData.interface1Present;
                }
                if ((romArrayPresent & BANK_MF128) != 0)
                {
                    mf128Present = cfgData.mf128Present;
                }
                bootIntoMenu = cfgData.bootIntoMenu;
                cfgData.romName[ROM_NAME_LEN] = 0;
            }
            cfgFile.close();
        }
    }

    // Always reload the configuration on normal reset
    menuConfigReload = true;
}

void menuSaveConfiguration()
{
    // Save the configuration to SD card, if anything has changed
    if (menuConfigChanged)
    {
        menuConfigChanged = false;
        File cfgFile = SD.open("ZXTEENSY.CFG", FILE_WRITE_BEGIN);
        if (cfgFile)
        {
            cfgData.divMmcPresent = divMmcPresent;
            cfgData.interface1Present = interface1Present;
            cfgData.mf128Present = mf128Present;
            cfgData.bootIntoMenu = bootIntoMenu;
            cfgData.romName[ROM_NAME_LEN] = 0;
            cfgFile.write((char*)&cfgData, sizeof(cfgData));
            cfgFile.close();
        }
    }
}
