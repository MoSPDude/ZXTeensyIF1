
uint8_t menuLine = 0;
uint8_t menuTotalLines = 0;
uint8_t menuPage = 0;
char* menuPtr = 0;
uint8_t menuLastRomIndex = 0;
bool menuSettingsChanged = false;

// "Use last ROM" is always index 0
// "Use internal ROM" is always index (MENU_ENTRIES - 1)
#define MENU_ENTRIES 5

void generateMenu(volatile uint8_t* romPtr)
{
    // Add settings menu
    menuPtr = (char*)romPtr + 0x2000;
    char* endPtr = menuPtr + 0x2000 - 36;
    char* ptr = generateMenuSettings(menuPtr);

    // Add save settings
    ptr = menuAddSetting(ptr, "Use internal ROM", 0);
    // MENU_ENTRIES == 5

    // List ROM files
    File romDirectory = SD.open("ROMS", FILE_READ);
    if (romDirectory)
    {
        if (romDirectory.isDirectory())
        {
            File entry = romDirectory.openNextFile();
            while (true)
            {
                if (entry)
                {
                    File nextEntry = romDirectory.openNextFile();
                    if (!entry.isDirectory())
                    {
                        ptr = menuAddFile(ptr, entry.name(), !nextEntry);
                    }
                    entry.close();
                    entry = nextEntry;
                } else {
                    // End of listing
                    break;
                }

                // End of menu check
                if ((ptr > endPtr) || (menuTotalLines == 0xFF))
                {
                    if (entry)
                    {
                        entry.close();
                    }
                    break;
                }
            }
        }
        romDirectory.close();
    }

    // Write the menu dimensions
    uint16_t address_ = ((romPtr[0x0FFC] << 8) + romPtr[0x0FFB]);
    romPtr[address_] = (menuTotalLines - 1);
    address_ = ((romPtr[0x0FFF] << 8) + romPtr[0x0FFE]);
    romPtr[address_] = (menuPage + 1);
}

char* generateMenuSettings(char* ptr)
{
    // Add settings menu as first options
    ptr = menuAddSetting(ptr, "Use last ROM", 0);
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

char* menuAddFile(char* ptr, const char* filename, bool lastFile)
{
    *ptr++ = ((menuTotalLines != menuLastRomIndex) ? (lastFile ? 31 : 30) : 27);
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
    switch (index)
    {
        case 0 :
            // Index 0 is the last loaded ROM
            return true;
        case 1 :
            if ((romArrayPresent & BANK_DIVMMC) != 0)
            {
                divMmcPresent = !divMmcPresent;
                menuSettingsChanged = true;
            }
            break;
        case 2 :
            if ((romArrayPresent & BANK_IF1) != 0)
            {
                interface1Present = !interface1Present;
                menuSettingsChanged = true;
            }
            break;
        case 3 :
            if ((romArrayPresent & BANK_MF128) != 0)
            {
                mf128Present = !mf128Present;
                menuSettingsChanged = true;
            }
            break;
        default :
            // Load ROM, either internal or from SD
            return true;
    }

    // Update settings menu
    generateMenuSettings(menuPtr);
    return false;
}

File menuGetFile(uint8_t index, bool* isZXC2Rom)
{
    // Index 0 is the last loaded ROM
    if (index == 0)
    {
        index = menuLastRomIndex;
    }

    // List the ROM files
    if (index >= MENU_ENTRIES)
    {
        File romDirectory = SD.open("ROMS", FILE_READ);
        if (romDirectory)
        {
            if (romDirectory.isDirectory())
            {
                uint8_t count = 0;
                index -= MENU_ENTRIES;
                while (true)
                {
                    File entry = romDirectory.openNextFile();
                    if (entry)
                    {
                        if (!entry.isDirectory())
                        {
                            // Find ROM at matching list index
                            if (index == count)
                            {
                                char *fileext = strrchr(entry.name(), '.');
                                if ((fileext != 0) && (stricmp(fileext + 1, "bin") == 0))
                                {
                                    *isZXC2Rom = true;
                                } else {
                                    *isZXC2Rom = false;
                                }
                                return entry;
                            }
                            ++count;
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
        char buf[6];
        if (cfgFile.readBytes(buf, 6) >= 6)
        {
            if ((romArrayPresent & BANK_DIVMMC) != 0)
            {
                divMmcPresent = (buf[0] == '1') ? true : false;
            }
            if ((romArrayPresent & BANK_IF1) != 0)
            {
                interface1Present = (buf[1] == '1') ? true : false;
            }
            if ((romArrayPresent & BANK_MF128) != 0)
            {
                mf128Present = (buf[2] == '1') ? true : false;
            }
            menuLastRomIndex = (uint8_t)strtol(&buf[3], 0, 16);
        }
        cfgFile.close();
    }
}

void menuSaveConfiguration(uint8_t index)
{
    if (menuSettingsChanged || (index != 0))
    {
        // Index 0 is the last loaded ROM
        menuSettingsChanged = false;
        if (index == 0)
        {
            index = menuLastRomIndex;
        }

        // Save the configuration
        File cfgFile = SD.open("ZXTEENSY.CFG", FILE_WRITE_BEGIN);
        if (cfgFile)
        {
            char buf[6];
            buf[0] = (divMmcPresent ? '1' : '0');
            buf[1] = (interface1Present ? '1' : '0');
            buf[2] = (mf128Present ? '1' : '0');
            ltoa(index, &buf[3], 16);
            cfgFile.write(buf, 6);
            cfgFile.close();
        }

        // Update the last loaded ROM
        menuLastRomIndex = index;
    }
}
