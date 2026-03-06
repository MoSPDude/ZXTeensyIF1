
#define FLASH_FILENAME "ZXTEENSY.HEX"
#define CFG_FILENAME ((const char*)F("ZXTEENSY.CFG"))
#define INTERNAL_ROM_NAME ((const char*)F(":INTERNAL"))
#define ROM_NAME_LEN 32
#define MAX_PATH 256

#define NETMAN_PATH "/netman.z80"
#define RTC_SETUP_PATH "/rtc_setup.z80"

#include "StringsZXTeensy.h"

static const uint8_t CHAR_BORDER = 20;
static const uint8_t CHAR_DIR = 21;
static const uint8_t CHAR_TICK = 22;
static const uint8_t CHAR_DSK = 23;
static const uint8_t CHAR_ZXC_L = 24;
static const uint8_t CHAR_ZXC_R = 25;
static const uint8_t CHAR_IF2_L = 26;
static const uint8_t CHAR_IF2_R = 27;
static const uint8_t CHAR_Z80_L = 28;
static const uint8_t CHAR_Z80_R = 29;
static const uint8_t CHAR_TZX_L = 30;
static const uint8_t CHAR_TZX_R = 31;

typedef enum {
    MENU_TYPE_SETTINGS,
    MENU_TYPE_LOAD_ROM,
    MENU_TYPE_NTP_TZ,
    MENU_TYPE_BROWSER,
    MENU_TYPE_BROWSER_OPEN
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
    char wifiNtpPresent;
    char wifiNtpTz;
    char bootIntoMenu;
    char romName[(ROM_NAME_LEN + 1)];
} cfg_data_t;

typedef enum {
    SETTING_ACTION_RESTART,
    SETTING_ACTION_DISABLE,
    SETTING_ACTION_NO_OP,
    SETTING_ACTION_TOGGLE_MENU,
    SETTING_ACTION_TOGGLE_DIVMMC,
    SETTING_ACTION_TOGGLE_IF1,
    SETTING_ACTION_TOGGLE_MF128,
    SETTING_ACTION_TOGGLE_USB,
    SETTING_ACTION_TOGGLE_UART,
    SETTING_ACTION_TOGGLE_NTP,
    SETTING_ACTION_OPEN_ROMS = 0xFC,
    SETTING_ACTION_OPEN_NTP_TZ = 0xFD,
    SETTING_ACTION_OPEN_BROWSER = 0xFE,
    SETTING_ACTION_INTERNAL_ROM = 0xFF
} settings_menu_action_t;

// Configuration data
DMAMEM cfg_data_t cfgData;
bool menuConfigChanged = false;
bool menuConfigReload = true;
bool menuHasUpdateFw = false;

// Menu structure
DMAMEM char* menuPtr;
DMAMEM char* menuEndPtr;
DMAMEM uint8_t menuEntries;
DMAMEM menu_type_t menuCurrent;
DMAMEM menu_entry_t menu[255];
volatile DMAMEM menu_action_t menuAction;

// Menu file browser
DMAMEM char browserPath[MAX_PATH];

// Menu page creation
DMAMEM uint8_t menuPage;
DMAMEM uint8_t menuPageLine;

void menuInsertEntry(menu_action_t action, uint8_t index, const char* ptr)
{
    menu[menuEntries].action = action;
    menu[menuEntries].index = index;

    // The label pointer will be zero if the label has been truncated, or
    // cannot be used (eg. non-printable characters)
    menu[menuEntries].ptr = ptr;
    ++menuEntries;
}

char* menuInsertSetting(menu_action_t action, uint8_t index, char* ptr, const char* label,
    bool checked)
{
    menuInsertEntry(action, index, 0);
    *ptr++ = (checked ? CHAR_TICK : CHAR_BORDER);
    unsigned int len = strlen(label);
    if (len > 35)
    {
        for (size_t i = 0; i < 34; ++i)
        {
            *ptr++ = (label[i] >= 128) ? '?' : label[i];
        }
        *ptr++ = '>';
    } else {
        for (size_t i = 0; i < len; ++i)
        {
            if (label[i] >= 128)
            {
                *ptr++ = '?';
            } else {
                *ptr++ = label[i];
            }
        }
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

inline __attribute__((always_inline)) char* menuInsertSpacer(char* ptr)
{
    return menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
        ptr, "", 0);
}

char* menuInsertFile(menu_action_t action, icon_type_t icon, uint8_t index, char* ptr,
    const char* filename)
{
    // Truncate the file name
    // NOTE: labelPtr will be zero if the name is truncated, or contains non-printable
    // characters
    unsigned int len = strlen(filename);
    const char* labelPtr;
    if (len > 32)
    {
        labelPtr = 0;
        for (size_t i = 0; i < 31; ++i)
        {
            *ptr++ = (filename[i] >= 128) ? '?' : filename[i];
        }
        *ptr++ = '>';
    } else {
        labelPtr = ptr;
        for (size_t i = 0; i < len; ++i)
        {
            if (filename[i] >= 128)
            {
                *ptr++ = '?';
                labelPtr = 0;
            } else {
                *ptr++ = filename[i];
            }
        }
    }

    // Add menu entry
    menuInsertEntry(action, index, labelPtr);

    // Add icon
    switch (icon)
    {
        case ICON_TYPE_DSK :
            *ptr++ = 9;
            *ptr++ = ' ';
            *ptr++ = CHAR_DSK;
            break;
        case ICON_TYPE_ZXC2 :
            *ptr++ = 9;
            *ptr++ = CHAR_ZXC_L;
            *ptr++ = CHAR_ZXC_R;
            break;
        case ICON_TYPE_CART :
            *ptr++ = 9;
            *ptr++ = CHAR_IF2_L;
            *ptr++ = CHAR_IF2_R;
            break;
        case ICON_TYPE_Z80 :
            *ptr++ = 9;
            *ptr++ = CHAR_Z80_L;
            *ptr++ = CHAR_Z80_R;
            break;
        case ICON_TYPE_TZX :
            *ptr++ = 9;
            *ptr++ = CHAR_TZX_L;
            *ptr++ = CHAR_TZX_R;
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

char* menuAddRomFile(uint8_t index, char* ptr, const char* filename)
{
    // Add check mark against active ROM
    *ptr++ = ((stricmp(filename, cfgData.romName) == 0) ? CHAR_TICK : CHAR_BORDER);

    // Find the file extension
    icon_type_t icon = ICON_TYPE_CART;
    char *fileext = strrchr(filename, '.');
    menu_action_t action = MENU_ACTION_LOAD_CART;
    if (fileext != 0)
    {
        if (stricmp(fileext + 1, "rom") == 0)
        {
            action = MENU_ACTION_LOAD_ROM;
            icon = ICON_TYPE_NONE;
        } else if (stricmp(fileext + 1, "bin") == 0)
        {
            icon = ICON_TYPE_ZXC2;
        }
    }

    // Insert the menu entry
    return menuInsertFile(action, icon, index, ptr, filename);
}

char* menuAddBrowserFile(uint8_t index, char* ptr, File entry)
{
    // Add directory icons, and find the file extension
    icon_type_t icon;
    menu_action_t action;
    const char* filename = entry.name();
    if (entry.isDirectory())
    {
        *ptr++ = CHAR_DIR;
        icon = ICON_TYPE_NONE;
        action = MENU_ACTION_BROWSER_CD;
    } else {
        *ptr++ = CHAR_BORDER;
        char *fileext = strrchr(filename, '.');
        if (fileext != 0)
        {
            if (stricmp(fileext + 1, "rom") == 0)
            {
                icon = ICON_TYPE_CART;
                action = MENU_ACTION_BROWSER_LOAD_CART;
            } else if (stricmp(fileext + 1, "bin") == 0)
            {
                icon = ICON_TYPE_ZXC2;
                action = MENU_ACTION_BROWSER_LOAD_ZXC2;
            } else if ((stricmp(fileext + 1, "z80") == 0) ||
                (stricmp(fileext + 1, "sna") == 0))
            {
                icon = ICON_TYPE_Z80;
                action = MENU_ACTION_BROWSER_LOAD_Z80;
            } else if ((stricmp(fileext + 1, "dsk") == 0) ||
                (stricmp(fileext + 1, "hdf") == 0))
            {
                icon = ICON_TYPE_DSK;
                action = MENU_ACTION_BROWSER_LOAD_DSK;
            } else if ((stricmp(fileext + 1, "tap") == 0) ||
                (stricmp(fileext + 1, "tzx") == 0))
            {
                icon = ICON_TYPE_TZX;
                action = MENU_ACTION_BROWSER_LOAD_TZX;
            } else {
                icon = ICON_TYPE_NONE;
                action = MENU_ACTION_BROWSER_OPEN;
            }
        } else {
            icon = ICON_TYPE_NONE;
            action = MENU_ACTION_BROWSER_OPEN;
        }
    }

    // Insert the menu entry
    return menuInsertFile(action, icon, index, ptr, filename);
}

char* menuGenerateNtpTz(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 0, ptr, MENU_STRINGS[STRING_MINUS_12_00_HOURS], (wifiNtpTz == 0));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 4, ptr, MENU_STRINGS[STRING_MINUS_11_00_HOURS], (wifiNtpTz == 4));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 8, ptr, MENU_STRINGS[STRING_MINUS_10_00_HOURS], (wifiNtpTz == 8));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 10, ptr, MENU_STRINGS[STRING_MINUS_9_30_HOURS], (wifiNtpTz == 10));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 12, ptr, MENU_STRINGS[STRING_MINUS_9_00_HOURS], (wifiNtpTz == 12));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 16, ptr, MENU_STRINGS[STRING_MINUS_8_00_HOURS], (wifiNtpTz == 16));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 20, ptr, MENU_STRINGS[STRING_MINUS_7_00_HOURS], (wifiNtpTz == 20));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 24, ptr, MENU_STRINGS[STRING_MINUS_6_00_HOURS], (wifiNtpTz == 24));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 28, ptr, MENU_STRINGS[STRING_MINUS_5_00_HOURS], (wifiNtpTz == 28));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 32, ptr, MENU_STRINGS[STRING_MINUS_4_00_HOURS], (wifiNtpTz == 32));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 36, ptr, MENU_STRINGS[STRING_MINUS_3_00_HOURS], (wifiNtpTz == 36));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 40, ptr, MENU_STRINGS[STRING_MINUS_2_00_HOURS], (wifiNtpTz == 40));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 44, ptr, MENU_STRINGS[STRING_MINUS_1_00_HOURS], (wifiNtpTz == 44));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 48, ptr, MENU_STRINGS[STRING_PLUS_0_00_HOURS], (wifiNtpTz == 48));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 52, ptr, MENU_STRINGS[STRING_PLUS_1_00_HOURS], (wifiNtpTz == 52));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 56, ptr, MENU_STRINGS[STRING_PLUS_2_00_HOURS], (wifiNtpTz == 56));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 60, ptr, MENU_STRINGS[STRING_PLUS_3_00_HOURS], (wifiNtpTz == 60));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 64, ptr, MENU_STRINGS[STRING_PLUS_4_00_HOURS], (wifiNtpTz == 64));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 68, ptr, MENU_STRINGS[STRING_PLUS_5_00_HOURS], (wifiNtpTz == 68));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 72, ptr, MENU_STRINGS[STRING_PLUS_6_00_HOURS], (wifiNtpTz == 72));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 74, ptr, MENU_STRINGS[STRING_PLUS_6_30_HOURS], (wifiNtpTz == 74));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 76, ptr, MENU_STRINGS[STRING_PLUS_7_00_HOURS], (wifiNtpTz == 76));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 80, ptr, MENU_STRINGS[STRING_PLUS_8_00_HOURS], (wifiNtpTz == 80));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 82, ptr, MENU_STRINGS[STRING_PLUS_8_30_HOURS], (wifiNtpTz == 82));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 84, ptr, MENU_STRINGS[STRING_PLUS_9_00_HOURS], (wifiNtpTz == 84));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 88, ptr, MENU_STRINGS[STRING_PLUS_9_00_HOURS], (wifiNtpTz == 88));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 90, ptr, MENU_STRINGS[STRING_PLUS_10_00_HOURS], (wifiNtpTz == 90));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 92, ptr, MENU_STRINGS[STRING_PLUS_10_30_HOURS], (wifiNtpTz == 92));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 94, ptr, MENU_STRINGS[STRING_PLUS_11_00_HOURS], (wifiNtpTz == 94));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 96, ptr, MENU_STRINGS[STRING_PLUS_12_00_HOURS], (wifiNtpTz == 96));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 99, ptr, MENU_STRINGS[STRING_PLUS_12_45_HOURS], (wifiNtpTz == 99));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 100, ptr, MENU_STRINGS[STRING_PLUS_13_00_HOURS], (wifiNtpTz == 100));
    ptr = menuInsertSetting(MENU_ACTION_NTP_TZ, 104, ptr, MENU_STRINGS[STRING_PLUS_14_00_HOURS], (wifiNtpTz == 104));
    return ptr;
}

char* menuGenerateBrowserOpen(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, 0xFF, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_CART, 0, ptr, MENU_STRINGS[STRING_LOAD_ROM], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_ZXC2, 0, ptr, MENU_STRINGS[STRING_LOAD_ZXC2], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_Z80, 0, ptr, MENU_STRINGS[STRING_LOAD_Z80], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_TZX, 0, ptr, MENU_STRINGS[STRING_LOAD_TZX], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_DSK, 0, ptr, MENU_STRINGS[STRING_LOAD_DSK], 0);
    return ptr;
}

char* menuGenerateBrowser(char* ptr)
{
    File directory = SD.open(browserPath, FILE_READ);
    if (directory)
    {
        if (directory.isDirectory())
        {
            uint8_t index = 0;
            ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, 0xFF, ptr, "..", 0);
            while (true)
            {
                File entry = directory.openNextFile();
                if (entry)
                {
                    ptr = menuAddBrowserFile(index, ptr, entry);
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
    return ptr;
}

char* menuGenerateLoadRom(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_INTERNAL_ROM,
        ptr, MENU_STRINGS[STRING_INTERNAL_ROM], (stricmp(cfgData.romName, INTERNAL_ROM_NAME) == 0));
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
                        ptr = menuAddRomFile(index, ptr, entry.name());
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
    return ptr;
}

char* menuGenerateSettings(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_RESTART,
        ptr, MENU_STRINGS[STRING_RESTART], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_DISABLE,
        ptr, MENU_STRINGS[STRING_DISABLE], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_BROWSER,
        ptr, MENU_STRINGS[STRING_OPEN_BROWSER], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_ROMS,
        ptr, MENU_STRINGS[STRING_OPEN_ROMS], 0);
    ptr = menuInsertSpacer(ptr);

    // Add firmware update option, if available
    File tmpFile = SD.open(FLASH_FILENAME, FILE_READ);
    if (tmpFile)
    {
        menuHasUpdateFw = true;
        tmpFile.close();
        ptr = menuInsertSetting(MENU_ACTION_UPDATE_FW, 0, ptr,
            MENU_STRINGS[STRING_UPDATE_FW], 0);
    } else {
        menuHasUpdateFw = false;
    }

    // Add settings menu
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_MENU,
        ptr, MENU_STRINGS[STRING_BOOT_MENU], bootIntoMenu);
    if ((romArrayPresent & BANK_DIVMMC) != 0)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_DIVMMC,
            ptr, MENU_STRINGS[STRING_ENABLE_DIVMMC], divMmcPresent);
    }
    if ((romArrayPresent & BANK_IF1) != 0)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_IF1,
            ptr, MENU_STRINGS[STRING_ENABLE_IF1], interface1Present);
    }
    if ((romArrayPresent & BANK_MF128) != 0)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_MF128,
            ptr, MENU_STRINGS[STRING_ENABLE_MF128], mf128Present);
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_USB, ptr,
        MENU_STRINGS[STRING_ENABLE_USB], usbPresent);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_UART, ptr,
        MENU_STRINGS[STRING_ENABLE_UART], uartPresent);
    if (uartPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_NTP,
            ptr, MENU_STRINGS[STRING_UPDATE_RTC_WIFI_NTP], wifiNtpPresent);
        if (wifiNtpHasTime)
        {
            struct tm buf;
            time_t timeNow = now();
            char timeStr[36];
            timeStr[0] = ' ';
            timeStr[1] = '>';
            timeStr[2] = ' ';
            strftime(&(timeStr[3]), 32, "%a %b %e %H:%M:%S %Y",
                gmtime_r(&timeNow, &buf));
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
                ptr, timeStr, 0);
        } else {
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
                ptr, MENU_STRINGS[STRING_WIFI_NTP_NOT_READY], 0);
        }
        if (wifiNtpPresent)
        {
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_NTP_TZ,
                ptr, MENU_STRINGS[STRING_SET_NTP_TZ], 0);
        }
    }

    // Add tools
    bool hasToolSpacer = false;
    tmpFile = SD.open(RTC_SETUP_PATH, FILE_READ);
    if (tmpFile)
    {
        hasToolSpacer = true;
        ptr = menuInsertSpacer(ptr);
        ptr = menuInsertSetting(MENU_ACTION_LOAD_RTC_SETUP, 0, ptr, 
            MENU_STRINGS[STRING_LOAD_RTC_CONFIG], 0);
        tmpFile.close();
    }
    tmpFile = SD.open(NETMAN_PATH, FILE_READ);
    if (tmpFile)
    {
        if (!hasToolSpacer)
        {
            ptr = menuInsertSpacer(ptr);
        }
        ptr = menuInsertSetting(MENU_ACTION_LOAD_NETMAN, 0, ptr, 
            MENU_STRINGS[STRING_LOAD_NETMAN], 0);
        tmpFile.close();
    }
    return ptr;
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
            textPtr = menuGenerateSettings(textPtr);
            break;
        case MENU_TYPE_LOAD_ROM :
            textPtr = menuGenerateLoadRom(textPtr);
            break;
        case MENU_TYPE_NTP_TZ :
            textPtr = menuGenerateNtpTz(textPtr);
            break;
        case MENU_TYPE_BROWSER :
            textPtr = menuGenerateBrowser(textPtr);
            break;
        case MENU_TYPE_BROWSER_OPEN :
            textPtr = menuGenerateBrowserOpen(textPtr);
            break;
    }

    // End of menu
    *(textPtr - 1) = 0;

    // Store the menu dimensions
    uint16_t address = ((menuPtr[0x11FC] << 8) + menuPtr[0x11FB]);
    menuPtr[address] = (menuEntries - 1);
    address = ((menuPtr[0x11FF] << 8) + menuPtr[0x11FE]);
    menuPtr[address] = ((menuPageLine != 0) ? (menuPage + 1) : menuPage);
}

void menuResetAction()
{
    // Reset the menu actions
    menuCurrent = MENU_TYPE_SETTINGS;
    menuAction = MENU_ACTION_SETTING;
}

void menuInitialise(volatile uint8_t* romPtr)
{
    // Store the menu pointers
    // NOTE: Allow for ROM_NAME_LEN, left icon, tab, 2 x right icons, and new-line
    menuPtr = (char*)romPtr;
    menuEndPtr = menuPtr + ROM_PAGE_SIZE - (ROM_NAME_LEN + 5);

    // Store the version information
    uint16_t address = ((menuPtr[0x11F9] << 8) + menuPtr[0x11F8]);
    strncpy((char*)&menuPtr[address], VERSION_STR, 9);

    // Generate the menu
    menuResetAction();
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
    char* entryPtr = (char*)menu[index].ptr;
    switch (menuAction)
    {
        case MENU_ACTION_TOP_MENU :
            menuCurrent = MENU_TYPE_SETTINGS;
            break;
        case MENU_ACTION_SETTING :
            switch (entryIndex)
            {
                case SETTING_ACTION_RESTART :
                    // Reload the existing ROM name, and reset
                    menuAction = MENU_ACTION_LOAD_ROM;
                    return true;
                case SETTING_ACTION_DISABLE :
                    // Temporarily disable device, and fully reset
                    isDeviceDisabled = true;
                    afterFirstReset = false;
                    menuAction = MENU_ACTION_LOAD_ROM;
                    return true;
                case SETTING_ACTION_NO_OP :
                    // Spacer line
                    break;
                case SETTING_ACTION_TOGGLE_MENU :
                    bootIntoMenu = !bootIntoMenu;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_DIVMMC :
                    if ((romArrayPresent & BANK_DIVMMC) != 0)
                    {
                        divMmcPresent = !divMmcPresent;
                        menuConfigChanged = true;
                    }
                    break;
                case SETTING_ACTION_TOGGLE_IF1 :
                    if ((romArrayPresent & BANK_IF1) != 0)
                    {
                        interface1Present = !interface1Present;
                        menuConfigChanged = true;
                    }
                    break;
                case SETTING_ACTION_TOGGLE_MF128 :
                    if ((romArrayPresent & BANK_MF128) != 0)
                    {
                        mf128Present = !mf128Present;
                        menuConfigChanged = true;
                    }
                    break;
                case SETTING_ACTION_TOGGLE_USB :
                    usbPresent = !usbPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_UART :
                    uartPresent = !uartPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_NTP :
                    wifiNtpPresent = !wifiNtpPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_OPEN_ROMS :
                    // Start ROM browser
                    menuCurrent = MENU_TYPE_LOAD_ROM;
                    break;
                case SETTING_ACTION_OPEN_NTP_TZ :
                    // Start WiFi NTP timezone selector
                    menuCurrent = MENU_TYPE_NTP_TZ;
                    break;
                case SETTING_ACTION_OPEN_BROWSER :
                    // Start file browser
                    strncpy(browserPath, "/", MAX_PATH);
                    menuCurrent = MENU_TYPE_BROWSER;
                    break;
                case SETTING_ACTION_INTERNAL_ROM :
                    // Load internal ROM name
                    strncpy(cfgData.romName, INTERNAL_ROM_NAME, ROM_NAME_LEN);
                    cfgData.romName[ROM_NAME_LEN] = 0;
                    menuAction = MENU_ACTION_LOAD_ROM;
                    menuConfigChanged = true;
                    return true;
                default :
                    break;
            }
            break;
        case MENU_ACTION_LOAD_ROM :
            menuConfigChanged = true;
            updateRomName(entryIndex, entryPtr);
            return true;
        case MENU_ACTION_LOAD_CART :
            updateRomName(entryIndex, entryPtr);
            return true;
        case MENU_ACTION_UPDATE_FW :
            // Perform firmware update, if available
            if (menuHasUpdateFw)
            {
                menuAction = MENU_ACTION_UPDATE_FW;
                return true;
            }
            break;
        case MENU_ACTION_NTP_TZ :
            wifiNtpTz = entryIndex;
            menuConfigChanged = true;
            menuCurrent = MENU_TYPE_SETTINGS;
            break;
        case MENU_ACTION_LOAD_NETMAN :
            strncpy(browserPath, NETMAN_PATH, MAX_PATH);
            return true;
        case MENU_ACTION_LOAD_RTC_SETUP :
            strncpy(browserPath, RTC_SETUP_PATH, MAX_PATH);
            return true;
        case MENU_ACTION_BROWSER_CD :
            if (updateBrowserPath(entryIndex, entryPtr))
            {
                menuCurrent = MENU_TYPE_BROWSER;
            } else {
                menuCurrent = MENU_TYPE_SETTINGS;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN :
            if (updateBrowserPath(entryIndex, entryPtr))
            {
                menuCurrent = MENU_TYPE_BROWSER_OPEN;
            } else {
                menuCurrent = MENU_TYPE_SETTINGS;
            }
            break;
        case MENU_ACTION_BROWSER_LOAD_CART :
        case MENU_ACTION_BROWSER_LOAD_ZXC2 :
        case MENU_ACTION_BROWSER_LOAD_Z80 :
        case MENU_ACTION_BROWSER_LOAD_TZX :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                updateBrowserPath(entryIndex, entryPtr))
            {
                return true;
            } else {
                menuCurrent = MENU_TYPE_SETTINGS;
            }
            break;
        case MENU_ACTION_BROWSER_LOAD_DSK :
            // TODO: Disk emulation?
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
        case MENU_ACTION_BROWSER_LOAD_CART :
        case MENU_ACTION_BROWSER_LOAD_ZXC2 :
        case MENU_ACTION_BROWSER_LOAD_Z80 :
            // Load new cartridge, with DivMMC disabled
            divMmcPresent = false;
            menuConfigReload = false;
            break;
        case MENU_ACTION_BROWSER_LOAD_TZX :
            // Load new tape, with DivMMC disabled
            tzxPresent = true;
            divMmcPresent = false;
            menuConfigReload = false;
            break;
        case MENU_ACTION_LOAD_NETMAN :
        case MENU_ACTION_LOAD_RTC_SETUP :
            // Load tools, with DivMMC and UART enabled
            divMmcPresent = true;
            uartPresent = true;
            wifiNtpPresent = false;
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
        } else if (stricmp(fileext + 1, "z80") == 0)
        {
            return TYPE_Z80;
        } else if (stricmp(fileext + 1, "sna") == 0)
        {
            return TYPE_SNA;
        }
    }
    return TYPE_CART;
}

void updateRomName(uint8_t fileIndex, char* filename)
{
    // Attempt to use the menu label as file name directly
    if (filename != 0)
    {
        cfgData.romName[ROM_NAME_LEN] = 0;
        for (size_t i = 0; i < ROM_NAME_LEN; ++i)
        {
            if (*filename < ' ')
            {
                cfgData.romName[i] = 0;
                return;
            } else if (*filename >= 128)
            {
                break;
            } else {
                cfgData.romName[i] = *filename++;
            }
        }
    }

    // Iterate the directory to find the file name
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

bool updateBrowserPath(uint8_t fileIndex, char* filename)
{
    // Attempt to use the menu label as file name directly
    if (filename != 0)
    {
        char tmpName[(ROM_NAME_LEN + 1)];
        tmpName[ROM_NAME_LEN] = 0;
        for (size_t i = 0; i < ROM_NAME_LEN; ++i)
        {
            if (*filename < ' ')
            {
                // Create new path
                tmpName[i] = 0;
                size_t pathLen = strlen(browserPath);
                if ((pathLen + strlen(tmpName)) < (MAX_PATH - 2))
                {
                    if (pathLen > 1)
                    {
                        strcat(browserPath, "/");
                    }
                    strcat(browserPath, tmpName);
                }
                return true;
            } else if (*filename >= 128)
            {
                break;
            } else {
                tmpName[i] = *filename++;
            }
        }
    }

    // Iterate the directory to find the file name
    File directory = SD.open(browserPath, FILE_READ);
    if (directory)
    {
        if (directory.isDirectory() && (fileIndex != 0xFF))
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
                        // Create new path
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
        directory.close();
    } else {
        // Fall back to root if the existing path is no longer found
        strcpy(browserPath, "/");
    }
    return true;
}

File menuGetForegroundRomFile(rom_type_t* romType)
{
    if (stricmp(cfgData.romName, INTERNAL_ROM_NAME) != 0)
    {
        // Attempt to open the ROM file directly
        if (strlen(cfgData.romName) < ROM_NAME_LEN)
        {
            strcpy(browserPath, "ROMS/");
            strcat(browserPath, cfgData.romName);
            File entry = SD.open(browserPath, FILE_READ);
            if (entry)
            {
                if (!entry.isDirectory())
                {
                    *romType = getRomType(cfgData.romName);
                    return entry;
                }
                entry.close();
            }
        }

        // Iterate the directory to find the partial file name
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

char* menuGetBrowserPath()
{
    return browserPath;
}

File menuGetBrowserRomFile()
{
    File entry = SD.open(browserPath, FILE_READ);
    if (entry)
    {
        if (!entry.isDirectory())
        {
            return entry;
        }
        entry.close();
    }

    // Return closed File
    return File();
}

File menuGetBrowserZ80File(rom_type_t* romType)
{
    File entry = menuGetBrowserRomFile();
    if (entry)
    {
        *romType = ((getRomType(entry.name()) != TYPE_SNA) ? TYPE_Z80 : TYPE_SNA);
    }
    return entry;
}

File menuGetRomFile(rom_type_t* romType)
{
    switch (menuAction)
    {
        case MENU_ACTION_BROWSER_LOAD_ZXC2 :
            *romType = TYPE_ZXC2;
            return menuGetBrowserRomFile();
        case MENU_ACTION_BROWSER_LOAD_CART :
            *romType = TYPE_CART;
            return menuGetBrowserRomFile();
        case MENU_ACTION_LOAD_NETMAN :
        case MENU_ACTION_LOAD_RTC_SETUP :
        case MENU_ACTION_BROWSER_LOAD_Z80 :
            return menuGetBrowserZ80File(romType);
        default :
            return menuGetForegroundRomFile(romType);
    }
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
    wifiNtpPresent = false;
    wifiNtpTz = 48;
    strncpy(cfgData.romName, INTERNAL_ROM_NAME, ROM_NAME_LEN);
    cfgData.romName[ROM_NAME_LEN] = 0;
    menuConfigChanged = true;
}

void menuLoadConfiguration()
{
    // Load the configuration from the SD card, as required
    if (menuConfigReload)
    {
        File cfgFile = SD.open(CFG_FILENAME, FILE_READ);
        if (cfgFile)
        {
            menuClearConfiguration();
            if (cfgFile.readBytes((char*)&cfgData, sizeof(cfgData)) >= sizeof(cfgData))
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
                wifiNtpPresent = cfgData.wifiNtpPresent;
                wifiNtpTz = cfgData.wifiNtpTz;
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
        File cfgFile = SD.open(CFG_FILENAME, FILE_WRITE_BEGIN);
        if (cfgFile)
        {
            cfgData.divMmcPresent = divMmcPresent;
            cfgData.interface1Present = interface1Present;
            cfgData.mf128Present = mf128Present;
            cfgData.uartPresent = uartPresent;
            cfgData.usbPresent = usbPresent;
            cfgData.wifiNtpPresent = wifiNtpPresent;
            cfgData.wifiNtpTz = wifiNtpTz;
            cfgData.bootIntoMenu = bootIntoMenu;
            cfgData.romName[ROM_NAME_LEN] = 0;
            cfgFile.write((char*)&cfgData, sizeof(cfgData));
            cfgFile.close();
        }
    }
}
