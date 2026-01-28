
#define INTERNAL_ROM_NAME ":INTERNAL"
#define ROM_NAME_LEN 32

uint8_t menuLine = 0;
uint8_t menuTotalLines = 0;
uint8_t menuPage = 0;
char* menuPtr = 0;
bool menuConfigChanged = false;

typedef struct {
    char divMmcPresent;
    char interface1Present;
    char mf128Present;
    char romName[(ROM_NAME_LEN + 1)];
} cfg_data_t;

// Configuration data
DMAMEM cfg_data_t cfgData;

// "Restart" is always index 0
// "Use internal ROM" is always index (menuRomListIndex - 1)
volatile uint8_t menuRomListIndex = 0;

void generateMenu(volatile uint8_t* romPtr)
{
    // Add settings menu
    menuPtr = (char*)romPtr + RAM_PAGE_SIZE;
    char* endPtr = menuPtr + RAM_PAGE_SIZE - 36;
    char* ptr = generateMenuSettings(menuPtr);

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

char* generateMenuSettings(char* ptr)
{
    // Add settings menu as first options
    ptr = menuAddSetting(ptr, "Restart", 0);
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
    if (index >= menuRomListIndex)
    {
        // Update the ROM name for the selection
        updateRomName(index - menuRomListIndex);
        menuConfigChanged = true;
        return true;
    } else {
        switch (index)
        {
            case 0 :
                // Reload the existing ROM name
                return true;
            case 1 :
                if ((romArrayPresent & BANK_DIVMMC) != 0)
                {
                    divMmcPresent = !divMmcPresent;
                    menuConfigChanged = true;
                }
                break;
            case 2 :
                if ((romArrayPresent & BANK_IF1) != 0)
                {
                    interface1Present = !interface1Present;
                    menuConfigChanged = true;
                }
                break;
            case 3 :
                if ((romArrayPresent & BANK_MF128) != 0)
                {
                    mf128Present = !mf128Present;
                    menuConfigChanged = true;
                }
                break;
            default :
                // Load internal ROM name
                strncpy(cfgData.romName, INTERNAL_ROM_NAME, ROM_NAME_LEN);
                cfgData.romName[ROM_NAME_LEN] = 0;
                menuConfigChanged = true;
                return true;
        }
    }

    // Update settings menu
    generateMenuSettings(menuPtr);
    return false;
}

void menuPerformAction()
{
    // Save the configuration to load new ROM
    menuSaveConfiguration();
}

void updateRomName(uint8_t fileIndex)
{
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
}

File menuGetFile(bool* isZXC2Rom)
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
                                char *fileext = strrchr(entry.name(), '.');
                                if ((fileext != 0) && (stricmp(fileext + 1, "bin") == 0))
                                {
                                    *isZXC2Rom = true;
                                } else {
                                    *isZXC2Rom = false;
                                }
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

void menuLoadConfiguration()
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
            cfgData.romName[ROM_NAME_LEN] = 0;
        }
        cfgFile.close();
    }
}

void menuSaveConfiguration()
{
    if (menuConfigChanged)
    {
        // Save the configuration
        menuConfigChanged = false;
        File cfgFile = SD.open("ZXTEENSY.CFG", FILE_WRITE_BEGIN);
        if (cfgFile)
        {
            cfgData.divMmcPresent = divMmcPresent;
            cfgData.interface1Present = interface1Present;
            cfgData.mf128Present = mf128Present;
            cfgData.romName[ROM_NAME_LEN] = 0;
            cfgFile.write((char*)&cfgData, sizeof(cfgData));
            cfgFile.close();
        }
    }
}
