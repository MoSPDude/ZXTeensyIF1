
#define FLASH_FILENAME "ZXTEENSY.HEX"
#define INTERNAL_ROM_NAME ":INTERNAL"
#define ROM_NAME_LEN 32
#define MAX_PATH 256

typedef enum {
    MENU_TYPE_SETTINGS,
    MENU_TYPE_BROWSER
} menu_type_t;

typedef struct {
    menu_action_t action;
    uint8_t index;
    const char* ptr;
} menu_entry_t;

typedef struct {
    char divMmcPresent;
    char interface1Present;
    char mf128Present;
    char uartPresent;
    char usbPresent;
    char bootIntoMenu;
    char romName[(ROM_NAME_LEN + 1)];
} cfg_data_t;

// Configuration data
DMAMEM cfg_data_t cfgData;
bool menuConfigChanged = false;
bool menuConfigReload = true;
bool menuHasUpdateFw = false;

// Menu structure
char* menuPtr;
char* menuEndPtr;
uint8_t menuEntries;
menu_type_t menuCurrent;
DMAMEM menu_entry_t menu[255];
volatile menu_action_t menuAction = MENU_ACTION_SETTING;

// Menu file browser
char browserPath[MAX_PATH];

// Menu page creation
uint8_t menuPage;
uint8_t menuPageLine;

void menuInsertEntry(menu_action_t action, uint8_t index, const char* ptr)
{
    menu[menuEntries].action = action;
    menu[menuEntries].index = index;
    menu[menuEntries].ptr = ptr;
    ++menuEntries;
}

char* menuAddSetting(menu_action_t action, uint8_t index, char* ptr, const char* label, 
    bool checked)
{
    menuInsertEntry(action, index, ptr);
    *ptr++ = (checked ? 27 : 24);
    unsigned int len = strlen(label);
    ptr = strncpy(ptr, label, len) + len;

    // Add new line, and update menu dimensions
    if (menuPageLine < 20)
    {
        *ptr = 10;
        ++menuPageLine;
    } else {
        menuPageLine = 0;
        ++menuPage;
    }
    return (ptr+1);
}

inline __attribute__((always_inline)) char* menuInsertSpacer(char* ptr)
{
    return menuAddSetting(MENU_ACTION_SETTING, 2, ptr, "", 0);
}

char* menuAddFile(uint8_t index, char* ptr, const char* filename)
{
    const char* startPtr = ptr;
    *ptr++ = ((stricmp(filename, cfgData.romName) == 0) ? 27 : 24);
    unsigned int len = strlen(filename);

    // Find the file extension
    uint8_t icon = 2;
    char *fileext = strrchr(filename, '.');
    menu_action_t action = MENU_ACTION_LOAD_CART;
    if (fileext != 0)
    {
        if (stricmp(fileext + 1, "rom") == 0)
        {
            action = MENU_ACTION_LOAD_ROM;
            len = (fileext - filename);
            icon = 0;
        } else if (stricmp(fileext + 1, "bin") == 0)
        {
            len = (fileext - filename);
            icon = 1;
        }
    }

    // Add menu entry
    menuInsertEntry(action, index, startPtr);

    // Truncate the file name
    if (len > 32)
    {
        len = 32;
    }
    strncpy(ptr, filename, len);
    ptr += len;

    // Add icon
    switch (icon)
    {
        case 1 :
            *ptr++ = 9;
            *ptr++ = 28;
            *ptr++ = 29;
            break;
        case 2 :
            *ptr++ = 9;
            *ptr++ = 30;
            *ptr++ = 31;
            break;
        default :
            break;
    }

    // Add new line, and update menu dimensions
    if (menuPageLine < 20)
    {
        *ptr = 10;
        ++menuPageLine;
    } else {
        *ptr = 0;
        menuPageLine = 0;
        ++menuPage;
    }
    return (ptr+1);
}

char* menuAddBrowser(uint8_t index, char* ptr, File entry)
{
    const char* startPtr = ptr;
    const char* filename = entry.name();
    unsigned int len = strlen(filename);
    *ptr++ = (entry.isDirectory() ? 25 : 24);

    // Find the file extension
    uint8_t icon = 0;
    if (!entry.isDirectory())
    {
        char *fileext = strrchr(filename, '.');
        icon = 2;
        if (fileext != 0)
        {
            if (stricmp(fileext + 1, "rom") == 0)
            {
                len = (fileext - filename);
            } else if (stricmp(fileext + 1, "bin") == 0)
            {
                len = (fileext - filename);
                icon = 1;
            }
        }
    }

    // Add menu entry
    menuInsertEntry((entry.isDirectory() ? MENU_ACTION_BROWSER_CD :
        MENU_ACTION_BROWSER_OPEN), index, startPtr);

    // Truncate the file name
    if (len > 32)
    {
        len = 32;
    }
    strncpy(ptr, filename, len);
    ptr += len;

    // Add icon
    switch (icon)
    {
        case 1 :
            *ptr++ = 9;
            *ptr++ = 28;
            *ptr++ = 29;
            break;
        case 2 :
            *ptr++ = 9;
            *ptr++ = 30;
            *ptr++ = 31;
            break;
        default :
            break;
    }

    // Add new line, and update menu dimensions
    if (menuPageLine < 20)
    {
        *ptr = 10;
        ++menuPageLine;
    } else {
        *ptr = 0;
        menuPageLine = 0;
        ++menuPage;
    }
    return (ptr+1);
}

void menuGenerateBrowser(char* ptr)
{
    File directory = SD.open(browserPath, FILE_READ);
    if (directory)
    {
        if (directory.isDirectory())
        {
            uint8_t index = 0;
            ptr = menuAddSetting(MENU_ACTION_BROWSER_CD, 0xFF, ptr, "..", 0);
            while (true)
            {
                File entry = directory.openNextFile();
                if (entry)
                {
                    ptr = menuAddBrowser(index, ptr, entry);
                    entry.close();
                    ++index;
                } else {
                    // End of listing
                    break;
                }

                // End of menu check
                if ((ptr > menuEndPtr) || (menuEntries == 254))
                {
                    break;
                }
            }
        }
        directory.close();
    }

    // End of menu
    *(ptr-1) = 0;
}

void menuGenerateSettings(char* ptr)
{
    ptr = menuAddSetting(MENU_ACTION_SETTING, 0, ptr, "Save and Restart", 0);
    ptr = menuAddSetting(MENU_ACTION_SETTING, 1, ptr, "Disable and Restart", 0);
    ptr = menuAddSetting(MENU_ACTION_SETTING, 0xFE, ptr, "Browse SD card", 0);
    ptr = menuInsertSpacer(ptr);

    // Add firmware update option, if available
    File fwUpdateFile = SD.open(FLASH_FILENAME, FILE_READ);
    if (fwUpdateFile)
    {
        menuHasUpdateFw = true;
        fwUpdateFile.close();
        ptr = menuAddSetting(MENU_ACTION_UPDATE_FW, 0, ptr, "Update firmware and Restart", 0);
    } else {
        menuHasUpdateFw = false;
    }

    // Add settings menu
    ptr = menuAddSetting(MENU_ACTION_SETTING, 3, ptr, "Boot into Menu", bootIntoMenu);
    if ((romArrayPresent & BANK_DIVMMC) != 0)
    {
        ptr = menuAddSetting(MENU_ACTION_SETTING, 4, ptr, "Enable DivMMC", divMmcPresent);
    }
    if ((romArrayPresent & BANK_IF1) != 0)
    {
        ptr = menuAddSetting(MENU_ACTION_SETTING, 5, ptr, "Enable Interface 1", interface1Present);
    }
    if ((romArrayPresent & BANK_MF128) != 0)
    {
        ptr = menuAddSetting(MENU_ACTION_SETTING, 6, ptr, "Enable Multiface 128", mf128Present);
    }
    ptr = menuAddSetting(MENU_ACTION_SETTING, 7, ptr, "Enable ESP-01S UART", uartPresent);
    ptr = menuAddSetting(MENU_ACTION_SETTING, 8, ptr, "Enable Kempston USB mouse/gamepad", usbPresent);
    ptr = menuInsertSpacer(ptr);

    // List ROM files
    ptr = menuAddSetting(MENU_ACTION_SETTING, 0xFF, ptr, "Internal ROM",
        (stricmp(cfgData.romName, INTERNAL_ROM_NAME) == 0));
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
                        ptr = menuAddFile(index, ptr, entry.name());
                    }
                    entry.close();
                    ++index;
                } else {
                    // End of listing
                    break;
                }

                // End of menu check
                if ((ptr > menuEndPtr) || (menuEntries == 255))
                {
                    break;
                }
            }
        }
        romDirectory.close();
    }

    // End of menu
    *(ptr-1) = 0;
}

void menuGenerate()
{
    // Reset the menu dimensions
    menuEntries = 0;
    menuPage = 0;
    menuPageLine = 0;

    // Build the menu
    char* textPtr = menuPtr + RAM_PAGE_SIZE;
    switch (menuCurrent)
    {
        case MENU_TYPE_SETTINGS :
            menuGenerateSettings(textPtr);
            break;
        case MENU_TYPE_BROWSER :
            menuGenerateBrowser(textPtr);
            break;
    }

    // Store the menu dimensions
    uint16_t address = ((menuPtr[0x11FC] << 8) + menuPtr[0x11FB]);
    menuPtr[address] = (menuEntries - 1);
    address = ((menuPtr[0x11FF] << 8) + menuPtr[0x11FE]);
    menuPtr[address] = (menuPage + 1);
}

void menuInitialise(volatile uint8_t* romPtr)
{
    // Build the menu
    menuPtr = (char*)romPtr;
    menuEndPtr = menuPtr + ROM_PAGE_SIZE - 36;

    // Store the version information
    uint16_t address = ((menuPtr[0x11F9] << 8) + menuPtr[0x11F8]);
    strncpy((char*)&menuPtr[address], VERSION_STR, 9);

    // Generate the menu
    menuCurrent = MENU_TYPE_SETTINGS;
    menuGenerate();
}

bool menuPerformSelection(uint8_t index)
{
    if (index >= menuEntries)
    {
        // Fully reset on out-of-bounds index error
        afterFirstReset = false;
        menuAction = MENU_ACTION_LOAD_ROM;
        return true;
    }

    menuAction = menu[index].action;
    uint8_t entryIndex = menu[index].index;
    switch (menuAction)
    {
        case MENU_ACTION_SETTING :
            switch (entryIndex)
            {
                case 0 :
                    // Reload the existing ROM name, and reset
                    menuAction = MENU_ACTION_LOAD_ROM;
                    return true;
                case 1 :
                    // Temporarily disable device, and fully reset
                    isDeviceDisabled = true;
                    afterFirstReset = false;
                    menuAction = MENU_ACTION_LOAD_ROM;
                    return true;
                case 2 :
                    // Spacer line
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
                case 7 :
                    uartPresent = !uartPresent;
                    menuConfigChanged = true;
                    break;
                case 8 :
                    usbPresent = !usbPresent;
                    menuConfigChanged = true;
                    break;
                case 0xFE :
                    // Start file browser
                    strncpy(browserPath, "/", MAX_PATH);
                    menuCurrent = MENU_TYPE_BROWSER;
                    break;
                case 0xFF :
                    // Load internal ROM name
                    strncpy(cfgData.romName, INTERNAL_ROM_NAME, ROM_NAME_LEN);
                    cfgData.romName[ROM_NAME_LEN] = 0;
                    menuAction = MENU_ACTION_LOAD_ROM;
                    menuConfigChanged = true;
                    return true;
                default :
                    break;
            }
        case MENU_ACTION_UPDATE_FW :
            // Perform firmware update, if available
            if (menuHasUpdateFw)
            {
                menuAction = MENU_ACTION_UPDATE_FW;
                return true;
            }
            break;
        case MENU_ACTION_LOAD_ROM :
            menuConfigChanged = true;
            updateRomName(entryIndex);
            return true;
        case MENU_ACTION_LOAD_CART :
            updateRomName(entryIndex);
            return true;
        case MENU_ACTION_BROWSER_CD :
            if (!updateBrowserPath(entryIndex))
            {
                menuCurrent = MENU_TYPE_SETTINGS;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN :
            updateBrowserPath(entryIndex);
            // TODO:
            menuCurrent = MENU_TYPE_SETTINGS;
            break;
    }

    // Refresh the menu
    menuGenerate();
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
        case MENU_ACTION_LOAD_CART :
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
    return TYPE_CART;
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
                    // Find ROM at matching directory index
                    if (!entry.isDirectory() && (index == fileIndex))
                    {
                        strncpy(cfgData.romName, entry.name(), ROM_NAME_LEN);
                        entry.close();
                        break;
                    }
                    entry.close();
                    ++index;
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

bool updateBrowserPath(uint8_t fileIndex)
{
    File directory = SD.open(browserPath, FILE_READ);
    if (directory)
    {
        if (directory.isDirectory())
        {
            if (fileIndex != 0xFF)
            {
                uint8_t index = 0;
                while (true)
                {
                    File entry = directory.openNextFile();
                    if (entry)
                    {
                        // Find ROM at matching directory index
                        if (index == fileIndex)
                        {
                            size_t pathLen = strlen(browserPath);
                            if ((pathLen + strlen(entry.name())) < (MAX_PATH - 2))
                            {
                                if (pathLen > 1)
                                {
                                    strcat(browserPath, "/");
                                }
                                strcat(browserPath, entry.name());
                            }
                            entry.close();
                            break;
                        }
                        entry.close();
                        ++index;
                    }
                }
            } else {
                char *fileext = strrchr(browserPath, '/');
                if (fileext != 0)
                {
                    if (fileext != browserPath)
                    {
                        // Remove last directory
                        *fileext = 0;
                    } else if (strlen(browserPath) > 1)
                    {
                        // Return to root directory
                        *(fileext+1) = 0;
                    } else {
                        // Exit the browser
                        directory.close();
                        return false;
                    }
                }
            }
        }
        directory.close();
    }
    return true;
}

File menuGetRomFile(rom_type_t* romType)
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
    *romType = TYPE_ROM;
    return File();
}

void menuClearConfiguration()
{
    memset(&cfgData, 0, sizeof(cfgData));
    divMmcPresent = false;
    interface1Present = false;
    mf128Present = false;
    uartPresent = false;
    usbPresent = false;
    bootIntoMenu = true;
    cfgData.bootIntoMenu = bootIntoMenu;
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
            menuClearConfiguration();
            if (cfgFile.readBytes((char*)&cfgData, sizeof(cfgData)) > 0)
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
                uartPresent = cfgData.uartPresent;
                usbPresent = cfgData.usbPresent;
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
            cfgData.uartPresent = uartPresent;
            cfgData.usbPresent = usbPresent;
            cfgData.bootIntoMenu = bootIntoMenu;
            cfgData.romName[ROM_NAME_LEN] = 0;
            cfgFile.write((char*)&cfgData, sizeof(cfgData));
            cfgFile.close();
        }
    }
}
