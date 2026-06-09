
#include "StringsZXTeensy.h"
#include "DefinesZXTeensy.h"

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

static const uint16_t MENU_VERSION_STR = 0x11F8;
static const uint16_t MENU_NUM_ENTRIES = 0x11FB;
static const uint16_t MENU_NUM_PAGES = 0x11FE;

static const uint16_t Z80_BANK1 = 0x4FF;
static const uint16_t Z80_IM2 = 0x4FE;
static const uint16_t Z80_PC = 0x4FC;
static const uint16_t Z80_SP = 0x4FA;
static const uint16_t Z80_REGS = 0x4F8;

typedef enum {
    Z80_REG_IY = 0,
    Z80_REG_IX = 2,
    Z80_REG_BC2 = 4,
    Z80_REG_DE2 = 6,
    Z80_REG_HL2 = 8,
    Z80_REG_AF2 = 10,
    Z80_REG_BC = 12,
    Z80_REG_DE = 14,
    Z80_REG_RF = 16,
    Z80_REG_IF = 18,
    Z80_REG_HL = 20,
    Z80_REG_AF = 24
} z80_regs_offset_t;

extern float tempmonGetTemp(void);

typedef enum {
    MENU_TYPE_MAIN,
    MENU_TYPE_SETTINGS,
    MENU_TYPE_LOAD_ROM,
    MENU_TYPE_NTP_TZ,
    MENU_TYPE_BROWSER,
    MENU_TYPE_BROWSER_OPEN,
    MENU_TYPE_BROWSER_OPEN_ZXC2,
    MENU_TYPE_BROWSER_MOUNT_HDF,
    MENU_TYPE_BROWSER_MOUNT_DSK,
    MENU_TYPE_HTTP_SERVER,
    MENU_TYPE_IN_GAME,
    MENU_TYPE_TAPE_BROWSER,
    MENU_TYPE_DEBUG
} menu_type_t;

typedef struct {
    menu_action_t action;
    uint8_t index;
    const char* ptr;
} menu_entry_t;

typedef struct {
    char cfgName[MAX_PATH];
    char romName[MAX_PATH];
    char divMmcSdaPath[MAX_PATH];
    char divMmcSdbPath[MAX_PATH];
    char dskFdaPath[MAX_PATH];
    char dskFdbPath[MAX_PATH];
    char modemUrl[MAX_PATH];
} cfg_data_t;

typedef enum {
    SETTING_ACTION_RESTART,
    SETTING_ACTION_DISABLE,
    SETTING_ACTION_NO_OP,
    SETTING_ACTION_TOGGLE_MENU,
    SETTING_ACTION_TOGGLE_MENU_IN_GAME,
    SETTING_ACTION_TOGGLE_DIVMMC,
    SETTING_ACTION_TOGGLE_DIVMMC_RAM,
    SETTING_ACTION_TOGGLE_DIVMMC_ROM,
    SETTING_ACTION_TOGGLE_IF1,
    SETTING_ACTION_TOGGLE_MF128,
    SETTING_ACTION_TOGGLE_USB,
    SETTING_ACTION_TOGGLE_FIRE_BUTTONS,
    SETTING_ACTION_TOGGLE_UART,
    SETTING_ACTION_TOGGLE_MODEM,
    SETTING_ACTION_TOGGLE_NTP,
    SETTING_ACTION_TOGGLE_FDC,
    SETTING_ACTION_TOGGLE_PRINTER,
    SETTING_ACTION_TOGGLE_LPRINT,
    SETTING_ACTION_CLEAR_PRINTER,
    SETTING_ACTION_UNMOUNT_SDA,
    SETTING_ACTION_UNMOUNT_SDB,
    SETTING_ACTION_UNMOUNT_FDA,
    SETTING_ACTION_UNMOUNT_FDB,
    SETTING_ACTION_IN_GAME_TOGGLE_IF1,
    SETTING_ACTION_OPEN_DEBUG = 0xF8,
    SETTING_ACTION_OPEN_TAPE_BROWSER = 0xF9,
    SETTING_ACTION_OPEN_SERVER = 0xFA,
    SETTING_ACTION_OPEN_ROMS = 0xFB,
    SETTING_ACTION_OPEN_NTP_TZ = 0xFC,
    SETTING_ACTION_OPEN_BROWSER = 0xFD,
    SETTING_ACTION_OPEN_SETTINGS = 0xFE,
    SETTING_ACTION_INTERNAL_ROM = 0xFF
} settings_menu_action_t;

// Configuration data
cfg_data_t cfgData;
bool menuConfigChanged = false;
bool menuHasUpdateFw = false;
bool menuHasMdrEmu = false;

// Menu structure
char* menuPtr;
char* menuTxtPtr;
char* menuEndPtr;
uint8_t menuEntries;
menu_type_t menuCurrent;
menu_type_t menuTopMenu;
menu_entry_t menu[255];
volatile menu_action_t menuAction;

// Menu file browser
char menuFileName[MAX_PATH];
char browserPath[MAX_PATH];

// Menu page creation
uint8_t menuPage;
uint8_t menuPageLine;

// Debug menu text
static const size_t MENU_DEBUG_SIZE = (21 * MENU_TXT_LEN * 2);
char menuDebugBuffer[MENU_DEBUG_SIZE];
volatile size_t menuDebugIndex = 0;
volatile bool menuHasDebug = false;

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
    if (len > MENU_TXT_LEN)
    {
        for (size_t i = 0; i < (MENU_TXT_LEN - 1); ++i)
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
    return (ptr + 1);
}

inline __attribute__((always_inline)) char* menuInsertSpacer(char* ptr)
{
    return menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
        ptr, "", 0);
}

char* menuInsertInGameStatus(char* ptr)
{
    // Show tape position
    char label[(MENU_STR_LEN + 1)];
    if (tzxEnabled)
    {
        size_t size;
        size_t pos = tzxPlayer.getPosition(&size);
        double temp = pos;
        temp = (temp * 100) / size;
        int a = temp;
        temp *= 100;
        int b = (int)(temp) % 100;
        if (snprintf(label, (MENU_STR_LEN + 1), " > Tape: %d.%02d%%", a, b) >= (MENU_STR_LEN + 1))
        {
            label[MENU_STR_LEN] = 0;
        }
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
            ptr, label, 0);
    }

    // Show current ROMs
    strcpy(label, " > ROM:         ");
    if ((romArrayPresent & BANK_ROM0) != 0)
    {
        if (IS_ROM_PAGED(ROM_ROM3))
        {
            label[8] = '3';
        } else if (IS_ROM_PAGED(ROM_ROM2))
        {
            label[8] = '2';
        } else if (IS_ROM_PAGED(ROM_ROM1))
        {
            label[8] = '1';
        } else if (IS_ROM_PAGED(ROM_ROM0))
        {
            label[8] = '0';
        }
    }
    if (IS_ROM_PAGED(ROM_IF1))
    {
        label[9] = 'I';
    } else if (interface1Enabled)
    {
        label[9] = 'i';
    }
    if (IS_ROM_PAGED(ROM_MF128))
    {
        label[10] = 'M';
    } else if (mf128Present)
    {
        label[10] = 'm';
    }
    if (IS_ROM_PAGED(ROM_DIVMMC))
    {
        label[10] = 'D';
    } else if (divMmcRomEnabled)
    {
        label[11] = 'd';
    }
    if (IS_ROM_PAGED(ROM_LPRINT))
    {
        label[12] = 'L';
    } else if (lprintEnabled)
    {
        label[12] = 'l';
    }
    if (IS_ROM_PAGED(ROM_MODEM))
    {
        label[13] = 'V';
    } else if (modemPresent)
    {
        label[13] = 'v';
    }
    if (IS_ROM_PAGED(ROM_ZXC2))
    {
        label[14] = 'Z';
    } else if (zxC2Present)
    {
        label[14] = 'z';
    }
    if (IS_ROM_PAGED(ROM_SNA))
    {
        label[15] = 'S';
    } else if (snaLoaderPresent)
    {
        label[15] = 's';
    }
    return menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
        ptr, label, 0);
}

char* menuInsertClockTime(char* ptr)
{
    struct tm buf;
    time_t timeNow = now();
    char label[(MENU_STR_LEN + 1)];
    label[0] = ' ';
    label[1] = '>';
    label[2] = ' ';
    strftime(&(label[3]), 32, "%a %b %e %H:%M:%S %Y",
        gmtime_r(&timeNow, &buf));
    return menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
        ptr, label, 0);
}

char* menuInsertStatus(char* ptr)
{
    char label[(MENU_STR_LEN + 1)];
    double temp = tempmonGetTemp();
    int a = temp;
    temp *= 100;
    int b = (int)(temp) % 100;
    if (snprintf(label, (MENU_STR_LEN + 1), " > %d.%02d degC", a, b) >= (MENU_STR_LEN + 1))
    {
        label[MENU_STR_LEN] = 0;
    }
    return menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
        ptr, label, 0);
}

char* menuInsertFile(menu_action_t action, icon_type_t icon, uint8_t index, char* ptr,
    const char* filename)
{
    // Truncate the file name
    // NOTE: labelPtr will be zero if the name is truncated, or contains non-printable
    // characters
    unsigned int len = strlen(filename);
    const char* labelPtr;
    if (len > ROM_NAME_LEN)
    {
        labelPtr = 0;
        for (size_t i = 0; i < (ROM_NAME_LEN - 1); ++i)
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
    return (ptr + 1);
}

char* menuAddLoadRomFile(uint8_t index, char* ptr, const char* filename)
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

char* menuAddMainFile(uint8_t index, char* ptr, const char* filename)
{
    char name[MAX_PATH];
    strncpy(name, filename, MAX_PATH);
    name[(MAX_PATH - 1)] = 0;
    char *fileext = strrchr(name, '.');
    if ((fileext != 0) && (stricmp(fileext + 1, "cfg") == 0))
    {
        // Remove the file extension
        *fileext = 0;

        // Add check mark against active configuration
        *ptr++ = ((stricmp(filename, cfgData.cfgName) == 0) ? CHAR_TICK : CHAR_BORDER);

        // Insert the menu entry
        ptr = menuInsertFile(MENU_ACTION_LOAD_CFG, ICON_TYPE_NONE, index, ptr, name);
    }
    return ptr;
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
                action = MENU_ACTION_BROWSER_OPEN_ZXC2;
            } else if ((stricmp(fileext + 1, "z80") == 0) ||
                (stricmp(fileext + 1, "sna") == 0))
            {
                icon = ICON_TYPE_Z80;
                action = MENU_ACTION_BROWSER_LOAD_Z80;
            } else if (((romArrayPresent & BANK_DIVMMC) != 0) &&
                ((stricmp(fileext + 1, "img") == 0) ||
                    (stricmp(fileext + 1, "hdf") == 0)))
            {
                icon = ICON_TYPE_DSK;
                action = MENU_ACTION_BROWSER_OPEN_HDF;
            } else if (stricmp(fileext + 1, "dsk") == 0)
            {
                icon = ICON_TYPE_DSK;
                action = MENU_ACTION_BROWSER_OPEN_DSK;
            } else if (menuHasMdrEmu &&
                (stricmp(fileext + 1, "mdr") == 0))
            {
                icon = ICON_TYPE_DSK;
                action = MENU_ACTION_BROWSER_LOAD_MDR;
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

char* menuGenerateTapeBrowser(char* ptr)
{
    uint8_t count = tzxPlayer.tapeMarkCount;
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    for (uint8_t index = 0; index < count; ++index)
    {
        char* markName = tzxPlayer.tapeMarkName[index];
        ptr = menuInsertSetting(MENU_ACTION_IN_GAME_SEEK_TAPE, index, ptr,
            markName, 0);
    }
    return ptr;
}

char* menuGenerateInGame(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_IN_GAME_EXIT, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    if (tzxEnabled)
    {
        ptr = menuInsertSetting(MENU_ACTION_IN_GAME_EXIT_TAPE, 0, ptr,
            MENU_STRINGS[STRING_IN_GAME_EXIT_TAPE], 0);
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_TAPE_BROWSER,
            ptr, MENU_STRINGS[STRING_OPEN_TAPE_BROWSER], 0);
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_BROWSER,
        ptr, MENU_STRINGS[STRING_OPEN_BROWSER], 0);
    ptr = menuInsertSpacer(ptr);
    bool needSpacer = false;
    if (interface1Present && divMmcPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_IN_GAME_TOGGLE_IF1, 
            ptr, MENU_STRINGS[STRING_ENABLE_IF1], interface1Enabled);
        needSpacer = true;
    }
    if (mf128Present && ((romArrayPresent & BANK_MF128) != 0))
    {
        ptr = menuInsertSetting(MENU_ACTION_IN_GAME_MF128, 0, ptr,
            MENU_STRINGS[STRING_IN_GAME_MF128], 0);
        needSpacer = true;
    }
    if (divMmcRomEnabled && ((romArrayPresent & BANK_DIVMMC) != 0))
    {
        ptr = menuInsertSetting(MENU_ACTION_IN_GAME_DIVMMC, 0, ptr,
            MENU_STRINGS[STRING_IN_GAME_DIVMMC], 0);
        needSpacer = true;
    }
    if (needSpacer)
    {
        ptr = menuInsertSpacer(ptr);
    }
    ptr = menuInsertSetting(MENU_ACTION_IN_GAME_EXIT_BASIC, 0, ptr,
        MENU_STRINGS[STRING_IN_GAME_EXIT_BASIC], 0);
    ptr = menuInsertSetting(MENU_ACTION_IN_GAME_RESET, 0, ptr,
        MENU_STRINGS[STRING_IN_GAME_RESET], 0);
    ptr = menuInsertSetting(MENU_ACTION_IN_GAME_HARD_RESET, 0, ptr,
        MENU_STRINGS[STRING_IN_GAME_HARD_RESET], 0);

    // Add debug and status
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_DEBUG,
        ptr, (menuHasDebug ? MENU_STRINGS[STRING_OPEN_DEBUG] : ""), 0);
    ptr = menuInsertStatus(ptr);
    if (rtcHasTime)
    {
        ptr = menuInsertClockTime(ptr);
    } else {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
            ptr, (wifiNtpEnabled ? MENU_STRINGS[STRING_WIFI_NTP_WAITING] :
                MENU_STRINGS[STRING_RTC_NOT_SET]), 0);
    }
    ptr = menuInsertInGameStatus(ptr);
    return ptr;
}

char* menuGenerateDebug(char* ptr)
{
    size_t i = 0;
    menuDebugBuffer[(MENU_DEBUG_SIZE - 1)] = 0;
    while ((i < MENU_DEBUG_SIZE) && (menuDebugBuffer[i] < ' '))
    {
        ++i;
    }
    while ((menuEntries < 255) && (i < MENU_DEBUG_SIZE))
    {
        menuInsertEntry(MENU_ACTION_TOP_MENU, 0, 0);
        *ptr++ = CHAR_BORDER;
        for (size_t j = 0; j < (ROM_NAME_LEN + 3); ++j)
        {
            if (menuDebugBuffer[i] >= ' ')
            {
                *ptr++ = menuDebugBuffer[i++];
            } else {
                ++i;
                break;
            }
        }
        while ((i < MENU_DEBUG_SIZE) && (menuDebugBuffer[i] < ' '))
        {
            ++i;
        }
        if (menuPageLine < 20)
        {
            *ptr++ = 10;
            ++menuPageLine;
        } else {
            *ptr = 0;
            menuPageLine = 0;
            ++menuPage;
        }
    }
    if (menuEntries == 0)
    {
        ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr,
            MENU_STRINGS[STRING_CANCEL], 0);
    }
    return ptr;
}

char* menuGenerateHttpServer(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_START_SERVER, 0, ptr, MENU_STRINGS[STRING_START_HTTP], 0);
    if (httpServerStatus != "")
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP, ptr, httpServerStatus.c_str(), 0);
    }
    ptr = menuInsertSetting(MENU_ACTION_STOP_SERVER, 0, ptr, MENU_STRINGS[STRING_STOP_HTTP], 0);
    return ptr;
}

char* menuGenerateNtpTz(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_SETTINGS, ptr, MENU_STRINGS[STRING_CANCEL], 0);
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

char* menuGenerateBrowserOpenZXC2(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, 0xFF, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_CART, 0, ptr, MENU_STRINGS[STRING_LOAD_ROM], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_ZXC2, 0, ptr, MENU_STRINGS[STRING_LOAD_ZXC2], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_ZXC3, 0, ptr, MENU_STRINGS[STRING_LOAD_ZXC3], 0);
    return ptr;
}

char* menuGenerateBrowserMountDsk(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, 0xFF, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_FDA, 0, ptr, MENU_STRINGS[STRING_MOUNT_FDA], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_FDB, 0, ptr, MENU_STRINGS[STRING_MOUNT_FDB], 0);
    return ptr;
}

char* menuGenerateBrowserMountHdf(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, 0xFF, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_SDA, 0, ptr, MENU_STRINGS[STRING_MOUNT_SDA], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_SDB, 0, ptr, MENU_STRINGS[STRING_MOUNT_SDB], 0);
    return ptr;
}

char* menuGenerateBrowserOpen(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, 0xFF, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_CART, 0, ptr, MENU_STRINGS[STRING_LOAD_ROM], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_ZXC2, 0, ptr, MENU_STRINGS[STRING_LOAD_ZXC2], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_ZXC3, 0, ptr, MENU_STRINGS[STRING_LOAD_ZXC3], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_Z80, 0, ptr, MENU_STRINGS[STRING_LOAD_Z80], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_TZX, 0, ptr, MENU_STRINGS[STRING_LOAD_TZX], 0);
    if (menuHasMdrEmu)
    {
        ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_MDR, 0, ptr, MENU_STRINGS[STRING_LOAD_MDR], 0);
    }
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_FDA, 0, ptr, MENU_STRINGS[STRING_MOUNT_FDA], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_FDB, 0, ptr, MENU_STRINGS[STRING_MOUNT_FDB], 0);
    if ((romArrayPresent & BANK_DIVMMC) != 0)
    {
        ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_SDA, 0, ptr, MENU_STRINGS[STRING_MOUNT_SDA], 0);
        ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_SDB, 0, ptr, MENU_STRINGS[STRING_MOUNT_SDB], 0);
    }
    return ptr;
}

char* menuGenerateBrowser(char* ptr)
{
    if (beginSdfsSd())
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
                    if ((ptr > menuEndPtr) || (menuEntries == 255))
                    {
                        break;
                    }
                }
            }
            directory.close();
        }
    } else {
        ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr,
            MENU_STRINGS[STRING_CANCEL], 0);
    }
    return ptr;
}

char* menuGenerateLoadRom(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_INTERNAL_ROM,
        ptr, MENU_STRINGS[STRING_INTERNAL_ROM], (cfgData.romName[0] == 0));
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
                        ptr = menuAddLoadRomFile(index, ptr, entry.name());
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
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);

    // Add firmware update option, if available
    File tmpFile = SD.open(FWUPDATE_HEX_PATH, FILE_READ);
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
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_MENU_IN_GAME,
        ptr, MENU_STRINGS[STRING_ENABLE_MENU_IN_GAME], menuEnableInGame);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_DIVMMC,
        ptr, MENU_STRINGS[STRING_ENABLE_DIVMMC], divMmcPresent);
    if (divMmcPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_DIVMMC_RAM,
            ptr, MENU_STRINGS[STRING_ENABLE_DIVMMC_RAM], divMmcExtRamPresent);
        if ((romArrayPresent & BANK_DIVMMC) != 0)
        {
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_DIVMMC_ROM,
                ptr, MENU_STRINGS[STRING_ENABLE_DIVMMC_ROM], divMmcRomPresent);
        }
        char label[(MENU_STR_LEN + 1)];
        char* tmpPath = menuGetDivMmcSdaPath();
        if (tmpPath != 0)
        {
            tmpFile = SD.open(tmpPath, FILE_READ);
            if (tmpFile)
            {
                if (snprintf(label, (MENU_STR_LEN + 1), " > sda: %s", tmpFile.name()) >= (MENU_STR_LEN + 1))
                {
                    label[MENU_STR_LEN] = 0;
                }
                ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_SDA,
                    ptr, label, 0);
                tmpFile.close();
            } else {
                cfgData.divMmcSdaPath[0] = 0;
            }
        }
        tmpPath = menuGetDivMmcSdbPath();
        if (tmpPath != 0)
        {
            tmpFile = SD.open(tmpPath, FILE_READ);
            if (tmpFile)
            {
                if (snprintf(label, (MENU_STR_LEN + 1), " > sdb: %s", tmpFile.name()) >= (MENU_STR_LEN + 1))
                {
                    label[MENU_STR_LEN] = 0;
                }
                ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_SDB,
                    ptr, label, 0);
                tmpFile.close();
            } else {
                cfgData.divMmcSdbPath[0] = 0;
            }
        }
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_FDC,
        ptr, MENU_STRINGS[STRING_ENABLE_FDC], dskPresent);
    if (dskPresent)
    {
        char label[(MENU_STR_LEN + 1)];
        char* tmpPath = menuGetFdcFdaPath();
        if (tmpPath != 0)
        {
            tmpFile = SD.open(tmpPath, FILE_READ);
            if (tmpFile)
            {
                if (snprintf(label, (MENU_STR_LEN + 1), " > A: %s", tmpFile.name()) >= (MENU_STR_LEN + 1))
                {
                    label[MENU_STR_LEN] = 0;
                }
                ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_FDA,
                    ptr, label, 0);
                tmpFile.close();
            } else {
                cfgData.dskFdaPath[0] = 0;
            }
        }
        tmpPath = menuGetFdcFdbPath();
        if (tmpPath != 0)
        {
            tmpFile = SD.open(tmpPath, FILE_READ);
            if (tmpFile)
            {
                if (snprintf(label, (MENU_STR_LEN + 1), " > B: %s", tmpFile.name()) >= (MENU_STR_LEN + 1))
                {
                    label[MENU_STR_LEN] = 0;
                }
                ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_FDB,
                    ptr, label, 0);
                tmpFile.close();
            } else {
                cfgData.dskFdbPath[0] = 0;
            }
        }
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
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_PRINTER,
        ptr, MENU_STRINGS[STRING_ENABLE_PRINTER], printerPresent);
    if (printerPresent)
    {
        if ((romArrayPresent & BANK_LPRINT) != 0)
        {
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_LPRINT,
                ptr, MENU_STRINGS[STRING_ENABLE_LPRINT], lprintPresent);
        }
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_CLEAR_PRINTER,
            ptr, MENU_STRINGS[STRING_CLEAR_PRINTER], 0);
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_USB, ptr,
        MENU_STRINGS[STRING_ENABLE_USB], usbPresent);
    if (usbPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_FIRE_BUTTONS,
            ptr, MENU_STRINGS[STRING_ENABLE_FIRE_BUTTONS], gamepadButtons);
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_UART, ptr,
        MENU_STRINGS[STRING_ENABLE_UART], uartPresent);
    if (uartPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_MODEM,
            ptr, MENU_STRINGS[STRING_ENABLE_MODEM], modemPresent);
        if (modemPresent)
        {
            char label[(MENU_STR_LEN + 1)];
            if (snprintf(label, (MENU_STR_LEN + 1), " > %s", cfgData.modemUrl) >= (MENU_STR_LEN + 1))
            {
                label[MENU_STR_LEN] = 0;
            }
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
                ptr, label, 0);
        }
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_NTP,
        ptr, MENU_STRINGS[STRING_UPDATE_RTC_WIFI_NTP], wifiNtpPresent);
    if (wifiNtpPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_NTP_TZ,
            ptr, MENU_STRINGS[STRING_SET_NTP_TZ], 0);
    }
    if (rtcHasTime)
    {
        ptr = menuInsertClockTime(ptr);
    } else if (wifiNtpPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
            ptr, (wifiNtpEnabled ? MENU_STRINGS[STRING_WIFI_NTP_WAITING] :
                MENU_STRINGS[STRING_RTC_NOT_SET]), 0);
    }

    // Add tools
    bool hasToolSpacer = false;
    tmpFile = SD.open(RTC_SETUP_Z80_PATH, FILE_READ);
    if (tmpFile)
    {
        hasToolSpacer = true;
        ptr = menuInsertSpacer(ptr);
        ptr = menuInsertSetting(MENU_ACTION_LOAD_RTC_SETUP, 0, ptr,
            MENU_STRINGS[STRING_LOAD_RTC_CONFIG], 0);
        tmpFile.close();
    }
    tmpFile = SD.open(NETMAN_Z80_PATH, FILE_READ);
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

char* menuGenerateMain(char* ptr)
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

    // List stored configurations for quick selection
    File cfgDirectory = SD.open("/ZXTEENSY/CONFIGS", FILE_READ);
    if (cfgDirectory)
    {
        if (cfgDirectory.isDirectory())
        {
            uint8_t index = 0;
            while (true)
            {
                File entry = cfgDirectory.openNextFile();
                if (entry)
                {
                    if (!entry.isDirectory())
                    {
                        ptr = menuAddMainFile(index, ptr, entry.name());
                    }
                    entry.close();
                    ++index;
                } else {
                    // End of listing
                    break;
                }

                // End of menu check
                if ((ptr > menuEndPtr) || (menuEntries == 252))
                {
                    break;
                }
            }
            if (index > 0)
            {
                ptr = menuInsertSpacer(ptr);
            }
        }
        cfgDirectory.close();
    }

    // Add settings and HTTP server
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_SETTINGS,
        ptr, MENU_STRINGS[STRING_OPEN_SETTINGS], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_SERVER,
        ptr, MENU_STRINGS[STRING_OPEN_HTTP_SERVER], 0);

    // Add debug and status
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_DEBUG,
        ptr, (menuHasDebug ? MENU_STRINGS[STRING_OPEN_DEBUG] : ""), 0);
    ptr = menuInsertStatus(ptr);
    if (rtcHasTime)
    {
        ptr = menuInsertClockTime(ptr);
    } else {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
            ptr, (wifiNtpEnabled ? MENU_STRINGS[STRING_WIFI_NTP_WAITING] :
                MENU_STRINGS[STRING_RTC_NOT_SET]), 0);
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
    char* textPtr = menuTxtPtr;
    switch (menuCurrent)
    {
        case MENU_TYPE_MAIN :
            textPtr = menuGenerateMain(textPtr);
            break;
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
        case MENU_TYPE_BROWSER_OPEN_ZXC2 :
            textPtr = menuGenerateBrowserOpenZXC2(textPtr);
            break;
        case MENU_TYPE_BROWSER_MOUNT_HDF :
            textPtr = menuGenerateBrowserMountHdf(textPtr);
            break;
        case MENU_TYPE_BROWSER_MOUNT_DSK :
            textPtr = menuGenerateBrowserMountDsk(textPtr);
            break;
        case MENU_TYPE_HTTP_SERVER :
            textPtr = menuGenerateHttpServer(textPtr);
            break;
        case MENU_TYPE_IN_GAME :
            textPtr = menuGenerateInGame(textPtr);
            break;
        case MENU_TYPE_TAPE_BROWSER :
            textPtr = menuGenerateTapeBrowser(textPtr);
            break;
        case MENU_TYPE_DEBUG :
            textPtr = menuGenerateDebug(textPtr);
            break;
    }

    // End of menu
    *(textPtr - 1) = 0;

    // Store the menu dimensions
    uint16_t address = ((menuPtr[(MENU_NUM_ENTRIES + 1)] << 8) + menuPtr[MENU_NUM_ENTRIES]);
    menuPtr[address] = (menuEntries - 1);
    address = ((menuPtr[(MENU_NUM_PAGES + 1)] << 8) + menuPtr[MENU_NUM_PAGES]);
    menuPtr[address] = ((menuPageLine != 0) ? (menuPage + 1) : menuPage);
}

void menuResetAction()
{
    // Reset the menu actions
    menuCurrent = MENU_TYPE_MAIN;
    menuTopMenu = MENU_TYPE_MAIN;
    menuAction = MENU_ACTION_LOAD_ROM;
}

void menuBeginInGame()
{
    // Reset the menu actions
    menuCurrent = MENU_TYPE_IN_GAME;
    menuTopMenu = MENU_TYPE_IN_GAME;
    menuAction = MENU_ACTION_LOAD_ROM;

    // Generate the menu
    menuGenerate();
}

void menuBeginMain()
{
    // Reset the menu actions
    menuResetAction();

    // Generate the menu
    menuGenerate();
}

void menuInitialise(volatile uint8_t* romPtr, volatile uint8_t* ramPtr)
{
    // Store the menu pointers
    menuPtr = (char*)romPtr;
    menuTxtPtr = (char*)ramPtr;
    menuEndPtr = menuTxtPtr + RAM_PAGE_SIZE - MENU_STR_LEN;

    // Store the version information
    uint16_t address = ((menuPtr[(MENU_VERSION_STR + 1)] << 8) + menuPtr[MENU_VERSION_STR]);
    strncpy((char*)&menuPtr[address], VERSION_STR, 9);

    // Check for microdrive emulator ROM
    File mdrRomFile = SD.open(MDR_EMULATOR_ROM_PATH, FILE_READ);
    if (mdrRomFile)
    {
        menuHasMdrEmu = true;
        mdrRomFile.close();
    } else {
        menuHasMdrEmu = false;
    }
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
            menuCurrent = menuTopMenu;
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
                case SETTING_ACTION_TOGGLE_MENU_IN_GAME :
                    menuEnableInGame = !menuEnableInGame;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_DIVMMC :
                    divMmcPresent = !divMmcPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_DIVMMC_RAM :
                    divMmcExtRamPresent = !divMmcExtRamPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_DIVMMC_ROM :
                    if ((romArrayPresent & BANK_DIVMMC) != 0)
                    {
                        divMmcRomPresent = !divMmcRomPresent;
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
                case SETTING_ACTION_TOGGLE_FIRE_BUTTONS :
                    gamepadButtons = !gamepadButtons;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_UART :
                    uartPresent = !uartPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_MODEM :
                    modemPresent = !modemPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_NTP :
                    wifiNtpPresent = !wifiNtpPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_FDC :
                    dskPresent = !dskPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_PRINTER :
                    printerPresent = !printerPresent;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_TOGGLE_LPRINT :
                    if ((romArrayPresent & BANK_LPRINT) != 0)
                    {
                        lprintPresent = !lprintPresent;
                        menuConfigChanged = true;
                    }
                    break;
                case SETTING_ACTION_CLEAR_PRINTER :
                    menuClearPrinterFile();
                    break;
                case SETTING_ACTION_UNMOUNT_SDA :
                    cfgData.divMmcSdaPath[0] = 0;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_UNMOUNT_SDB :
                    cfgData.divMmcSdbPath[0] = 0;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_UNMOUNT_FDA :
                    cfgData.dskFdaPath[0] = 0;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_UNMOUNT_FDB :
                    cfgData.dskFdbPath[0] = 0;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_IN_GAME_TOGGLE_IF1 :
                    // Toggle between DivMMC and Interface 1
                    if (interface1Enabled)
                    {
                        divMmcEnabled = divMmcPresent;
                        interface1Enabled = (interface1Present && !divMmcEnabled);
                    } else {
                        interface1Enabled = interface1Present;
                        divMmcEnabled = (divMmcPresent && !interface1Enabled);
                    }
                    divMmcRomEnabled = (divMmcEnabled && divMmcRomPresent);
                    break;
                case SETTING_ACTION_OPEN_DEBUG :
                    // Open debug menu
                    menuCurrent = MENU_TYPE_DEBUG;
                    break;
                case SETTING_ACTION_OPEN_TAPE_BROWSER :
                    // Open tape browser
                    tzxPlayer.scanTape();
                    menuCurrent = MENU_TYPE_TAPE_BROWSER;
                    break;
                case SETTING_ACTION_OPEN_SERVER :
                    // Configure HTTP server
                    menuCurrent = MENU_TYPE_HTTP_SERVER;
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
                case SETTING_ACTION_OPEN_SETTINGS :
                    menuCurrent = MENU_TYPE_SETTINGS;
                    break;
                case SETTING_ACTION_INTERNAL_ROM :
                    // Load internal ROM name
                    if (cfgData.romName[0] == 0)
                    {
                        menuAction = MENU_ACTION_LOAD_ROM;
                        return true;
                    } else {
                        cfgData.romName[0] = 0;
                        menuConfigChanged = true;
                    }
                    break;
                default :
                    break;
            }
            break;
        case MENU_ACTION_LOAD_ROM :
            menuUpdateRomFileName(entryIndex, entryPtr);
            if (stricmp(cfgData.romName, menuFileName) == 0)
            {
                return true;
            } else {
                strncpy(cfgData.romName, menuFileName, MAX_PATH);
                cfgData.romName[(MAX_PATH - 1)] = 0;
                menuConfigChanged = true;
            }
            break;
        case MENU_ACTION_LOAD_CART :
            menuUpdateRomFileName(entryIndex, entryPtr);
            return true;
        case MENU_ACTION_LOAD_CFG :
            menuUpdateCfgFileName(entryIndex, entryPtr);
            if (stricmp(cfgData.cfgName, menuFileName) == 0)
            {
                return true;
            } else {
                strncpy(cfgData.cfgName, menuFileName, MAX_PATH);
                cfgData.cfgName[(MAX_PATH - 1)] = 0;
                menuLoadConfiguration(cfgData.cfgName);
                menuSaveConfiguration();
            }
            break;
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
            strncpy(browserPath, NETMAN_Z80_PATH, MAX_PATH);
            return true;
        case MENU_ACTION_LOAD_RTC_SETUP :
            strncpy(browserPath, RTC_SETUP_Z80_PATH, MAX_PATH);
            return true;
        case MENU_ACTION_BROWSER_CD :
            if (updateBrowserPath(entryIndex, entryPtr))
            {
                menuCurrent = MENU_TYPE_BROWSER;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN :
            if (updateBrowserPath(entryIndex, entryPtr))
            {
                menuCurrent = MENU_TYPE_BROWSER_OPEN;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN_ZXC2 :
            if (updateBrowserPath(entryIndex, entryPtr))
            {
                menuCurrent = MENU_TYPE_BROWSER_OPEN_ZXC2;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN_HDF :
            if (updateBrowserPath(entryIndex, entryPtr))
            {
                menuCurrent = MENU_TYPE_BROWSER_MOUNT_HDF;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN_DSK :
            if (updateBrowserPath(entryIndex, entryPtr))
            {
                menuCurrent = MENU_TYPE_BROWSER_MOUNT_DSK;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_LOAD_Z80 :
        case MENU_ACTION_BROWSER_LOAD_MDR :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                updateBrowserPath(entryIndex, entryPtr))
            {
                return true;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_LOAD_TZX :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                updateBrowserPath(entryIndex, entryPtr))
            {
                if (menuTopMenu != MENU_TYPE_MAIN)
                {
                    menuCurrent = menuTopMenu;
                } else {
                    return true;
                }
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_LOAD_CART :
        case MENU_ACTION_BROWSER_LOAD_ZXC2 :
        case MENU_ACTION_BROWSER_LOAD_ZXC3 :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                (menuCurrent == MENU_TYPE_BROWSER_OPEN_ZXC2) ||
                updateBrowserPath(entryIndex, entryPtr))
            {
                return true;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_MOUNT_SDA :
        case MENU_ACTION_BROWSER_MOUNT_SDB :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                (menuCurrent == MENU_TYPE_BROWSER_MOUNT_HDF) ||
                updateBrowserPath(entryIndex, entryPtr))
            {
                strncpy(((menuAction == MENU_ACTION_BROWSER_MOUNT_SDB) ?
                    cfgData.divMmcSdbPath : cfgData.divMmcSdaPath),
                    browserPath, MAX_PATH);
                divMmcPresent = true;
                menuConfigChanged = true;
            }
            menuCurrent = menuTopMenu;
            break;
        case MENU_ACTION_BROWSER_MOUNT_FDA :
        case MENU_ACTION_BROWSER_MOUNT_FDB :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                (menuCurrent == MENU_TYPE_BROWSER_MOUNT_DSK) ||
                updateBrowserPath(entryIndex, entryPtr))
            {
                strncpy(((menuAction == MENU_ACTION_BROWSER_MOUNT_FDB) ?
                    cfgData.dskFdbPath : cfgData.dskFdaPath),
                    browserPath, MAX_PATH);
                if (menuTopMenu == MENU_TYPE_MAIN)
                {
                    dskPresent = true;
                    menuConfigChanged = true;
                }
            }
            menuCurrent = menuTopMenu;
            break;
        case MENU_ACTION_START_SERVER :
            httpStartServer();
            break;
        case MENU_ACTION_STOP_SERVER :
            httpStopServer();
            break;
        case MENU_ACTION_IN_GAME_SEEK_TAPE :
            tzxPlayer.seek(entryIndex);
            menuCurrent = menuTopMenu;
            break;
        case MENU_ACTION_IN_GAME_EXIT :
        case MENU_ACTION_IN_GAME_EXIT_TAPE :
        case MENU_ACTION_IN_GAME_EXIT_BASIC :
        case MENU_ACTION_IN_GAME_MF128 :
        case MENU_ACTION_IN_GAME_DIVMMC :
        case MENU_ACTION_IN_GAME_RESET :
        case MENU_ACTION_IN_GAME_HARD_RESET :
            // These actions require additional NMI or reset handling
            return true;
    }

    // Clear the configuration name when changed
    if (menuConfigChanged)
    {
        cfgData.cfgName[0] = 0;
    }
    return false;
}

inline menu_action_t menuGetInGameAction()
{
    // Return MENU_ACTION_TOP_MENU as "No-Op", to either redraw or reset the
    // Spectrum on the main menu
    return ((menuTopMenu != MENU_TYPE_MAIN) ? menuAction : MENU_ACTION_TOP_MENU);
}

void menuPerformAction()
{
    // Save the configuration (if needed), and reload ROMs
    menuSaveConfiguration();
    loadRomSets = true;

    // Perform the menu action
    switch (menuAction)
    {
        case MENU_ACTION_UPDATE_FW :
            // Flash the firmware update
            flashUpdate(FWUPDATE_HEX_PATH);
            break;
        case MENU_ACTION_LOAD_CART :
        case MENU_ACTION_BROWSER_LOAD_CART :
        case MENU_ACTION_BROWSER_LOAD_ZXC2 :
        case MENU_ACTION_BROWSER_LOAD_ZXC3 :
        case MENU_ACTION_BROWSER_LOAD_Z80 :
            // Load new cartridge - with DivMMC, modem and LPRINT III disabled
            divMmcPresent = false;
            modemPresent = false;
            lprintPresent = false;
            break;
        case MENU_ACTION_BROWSER_LOAD_TZX :
            tzxPresent = true;
            break;
        case MENU_ACTION_BROWSER_LOAD_MDR :
            // Load new MDR image - with DivMMC and Interface 1 disabled
            mdrPresent = true;
            divMmcPresent = false;
            interface1Present = false;
            break;
        case MENU_ACTION_LOAD_NETMAN :
        case MENU_ACTION_LOAD_RTC_SETUP :
            // Load tools - with DivMMC and UART enabled
            uartPresent = true;
            divMmcPresent = true;
            modemPresent = false;
            lprintPresent = false;
            wifiNtpPresent = false;
            break;
        default :
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

bool menuUpdateFileName(uint8_t fileIndex, char* filename, const char* dirname)
{
    // Attempt to use the menu label as file name directly
    menuFileName[ROM_NAME_LEN] = 0;
    if (filename != 0)
    {
        for (size_t i = 0; i < ROM_NAME_LEN; ++i)
        {
            if (*filename < ' ')
            {
                menuFileName[i] = 0;
                return true;
            } else if (*filename >= 128)
            {
                break;
            } else {
                menuFileName[i] = *filename++;
            }
        }
    }

    // Iterate the directory to find the file name
    bool result = false;
    File directory = SD.open(dirname, FILE_READ);
    if (directory)
    {
        if (directory.isDirectory())
        {
            uint8_t index = 0;
            while (true)
            {
                File entry = directory.openNextFile();
                if (entry)
                {
                    // Find ROM at matching directory index
                    if (!entry.isDirectory() && (index == fileIndex))
                    {
                        result = true;
                        strncpy(menuFileName, entry.name(), MAX_PATH);
                        entry.close();
                        break;
                    }
                    entry.close();
                    ++index;
                } else {
                    // End of listing
                    menuFileName[0] = 0;
                    result = false;
                    break;
                }
            }
        }
        directory.close();
    }
    return result;
}

void menuUpdateRomFileName(uint8_t fileIndex, char* filename)
{
    menuUpdateFileName(fileIndex, filename, "/ROMS");
}

void menuUpdateCfgFileName(uint8_t fileIndex, char* filename)
{
    if (menuUpdateFileName(fileIndex, filename, "/ZXTEENSY/CONFIGS"))
    {
        strcat(menuFileName, ".cfg");
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
                    *(fileext + 1) = 0;
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

char* menuGetBrowserPath()
{
    return browserPath;
}

char* menuGetDivMmcSdaPath()
{
    return ((strlen(cfgData.divMmcSdaPath) > 0) ? cfgData.divMmcSdaPath : 0);
}

char* menuGetDivMmcSdbPath()
{
    return ((strlen(cfgData.divMmcSdbPath) > 0) ? cfgData.divMmcSdbPath : 0);
}

char* menuGetFdcFdaPath()
{
    return ((strlen(cfgData.dskFdaPath) > 0) ? cfgData.dskFdaPath : 0);
}

char* menuGetFdcFdbPath()
{
    return ((strlen(cfgData.dskFdbPath) > 0) ? cfgData.dskFdbPath : 0);
}

char* menuGetModemUrl()
{
    return ((strlen(cfgData.modemUrl) > 0) ? cfgData.modemUrl : 0);
}

File menuGetMenuRomFile(const char* cfgRomName, rom_type_t* romType)
{
    if (cfgRomName[0] != 0)
    {
        char romPath[MAX_PATH];
        if (snprintf(romPath, MAX_PATH, "/ROMS/%s", cfgRomName) >= (int)MAX_PATH)
        {
            romPath[(MAX_PATH - 1)] = 0;
        }
        File entry = SD.open(romPath, FILE_READ);
        if (entry)
        {
            if (!entry.isDirectory())
            {
                *romType = getRomType(cfgRomName);
                return entry;
            }
            entry.close();
        }
    }

    // Return closed File
    *romType = TYPE_ROM;
    return File();
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

File menuGetForegroundRomFile(rom_type_t* romType)
{
    switch (menuAction)
    {
        case MENU_ACTION_BROWSER_LOAD_ZXC2 :
            *romType = TYPE_ZXC2;
            return menuGetBrowserRomFile();
        case MENU_ACTION_BROWSER_LOAD_ZXC3 :
            *romType = TYPE_ZXC3;
            return menuGetBrowserRomFile();
        case MENU_ACTION_BROWSER_LOAD_CART :
            *romType = TYPE_CART;
            return menuGetBrowserRomFile();
        case MENU_ACTION_LOAD_NETMAN :
        case MENU_ACTION_LOAD_RTC_SETUP :
        case MENU_ACTION_BROWSER_LOAD_Z80 :
            return menuGetBrowserZ80File(romType);
        case MENU_ACTION_LOAD_CART :
            return menuGetMenuRomFile(menuFileName, romType);
        default :
            break;
    }

    // Return closed File
    *romType = TYPE_ROM;
    return File();
}

File menuGetSpectrumRomFile()
{
    rom_type_t romType;
    return menuGetMenuRomFile(cfgData.romName, &romType);
}

void menuInGameExitBasic()
{
    // Return to REPORT-D in 48K ROM
    menuRamArray[1][(Z80_PC + 1)] = 0x0D;
    menuRamArray[1][Z80_PC] = 0x00;

    // Set IY to 0x5C3A for BASIC
    uint16_t baseAddress = (RAM_PAGE_SIZE - 1) & 
        ((menuRamArray[1][(Z80_REGS + 1)] << 8) + menuRamArray[1][Z80_REGS]);
    uint16_t address = (baseAddress + Z80_REG_IY);
    menuRamArray[1][(address + 1)] = 0x5C;
    menuRamArray[1][address] = 0x3A;

    // Restore BASIC with IM1, enable interrupts, and I to 0x3F
     address = (baseAddress + Z80_REG_IF);
    menuRamArray[1][(address + 1)] = 0x3F;
    menuRamArray[1][address] |= 0x02;
    menuRamArray[1][Z80_IM2] = 0x02;

    // Set to bank in ROM1/3
    menuRamArray[1][Z80_BANK1] |= 0x10;

    // Page out into 48K ROM
    romPaged = (1UL << (ROM_MENU));
    if ((romArrayPresent & BANK_ROM3) != 0)
    {
        PAGE_IN_ROM(ROM_ROM3);
    } else if ((romArrayPresent & BANK_ROM1) != 0)
    {
        PAGE_IN_ROM(ROM_ROM1);
    } else {
        PAGE_IN_ROM(ROM_ROM0);
    }
}

void menuClearConfiguration()
{
    divMmcPresent = false;
    divMmcExtRamPresent = true;
    divMmcRomPresent = false;
    interface1Present = false;
    mf128Present = false;
    uartPresent = false;
    usbPresent = false;
    gamepadButtons = false;
    wifiNtpPresent = false;
    wifiNtpTz = 48;
    dskPresent = false;
    modemPresent = false;
    printerPresent = false;
    lprintPresent = false;
    bootIntoMenu = true;
    menuEnableInGame = true;
    memset(&cfgData, 0, sizeof(cfgData));
    strcpy(cfgData.modemUrl, MODEM_URL_PATH);
}

void menuLoadConfiguration(const char* cfgCfgName)
{
    // Load the configuration from the SD card
    size_t count = 0;
    char cfgPath[MAX_PATH];
    if (cfgCfgName != 0)
    {
        if (snprintf(cfgPath, MAX_PATH, "/ZXTEENSY/CONFIGS/%s", cfgCfgName) >= (int)MAX_PATH)
        {
            cfgPath[(MAX_PATH - 1)] = 0;
        }
    } else {
        menuClearConfiguration();
        strncpy(cfgPath, ZXTEENSY_CFG_PATH, MAX_PATH);
    }
    File cfgFile = SD.open(cfgPath, FILE_READ);
    if (cfgFile)
    {
        // Read the configuration from file
        String cfgStr;
        while (cfgFile.available())
        {
            cfgStr = cfgFile.readStringUntil('\n');
            const char* cfgPtr = cfgStr.c_str();
            switch (*cfgPtr)
            {
                case 0 :
                    // Empty line
                    break;
                case 'd' :
                    if (strncmp("divMmcPresent = ", cfgPtr, 16) == 0)
                    {
                        divMmcPresent = ((cfgPtr[16] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("divMmcExtRamPresent = ", cfgPtr, 22) == 0)
                    {
                        divMmcExtRamPresent = ((cfgPtr[22] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("divMmcRomPresent = ", cfgPtr, 19) == 0)
                    {
                        divMmcRomPresent = ((cfgPtr[19] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("divMmcSdaPath = ", cfgPtr, 16) == 0)
                    {
                        strncpy(cfgData.divMmcSdaPath, &(cfgPtr[16]), MAX_PATH);
                        cfgData.divMmcSdaPath[(MAX_PATH - 1)] = 0;
                        ++count;
                    } else if (strncmp("divMmcSdbPath = ", cfgPtr, 16) == 0)
                    {
                        strncpy(cfgData.divMmcSdbPath, &(cfgPtr[16]), MAX_PATH);
                        cfgData.divMmcSdbPath[(MAX_PATH - 1)] = 0;
                        ++count;
                    } else if (strncmp("dskPresent = ", cfgPtr, 13) == 0)
                    {
                        dskPresent = ((cfgPtr[13] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("dskFdaPath = ", cfgPtr, 13) == 0)
                    {
                        strncpy(cfgData.dskFdaPath, &(cfgPtr[13]), MAX_PATH);
                        cfgData.dskFdaPath[(MAX_PATH - 1)] = 0;
                        ++count;
                    } else if (strncmp("dskFdbPath = ", cfgPtr, 13) == 0)
                    {
                        strncpy(cfgData.dskFdbPath, &(cfgPtr[13]), MAX_PATH);
                        cfgData.dskFdbPath[(MAX_PATH - 1)] = 0;
                        ++count;
                    }
                    break;
                case 'm' :
                    if (strncmp("menuEnableInGame = ", cfgPtr, 19) == 0)
                    {
                        menuEnableInGame = ((cfgPtr[19] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("mf128Present = ", cfgPtr, 15) == 0)
                    {
                        mf128Present = ((cfgPtr[15] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("modemPresent = ", cfgPtr, 15) == 0)
                    {
                        modemPresent = ((cfgPtr[15] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("modemUrl = ", cfgPtr, 11) == 0)
                    {
                        strncpy(cfgData.modemUrl, &(cfgPtr[11]), MAX_PATH);
                        cfgData.modemUrl[(MAX_PATH - 1)] = 0;
                        ++count;
                    }
                    break;
                case 'u' :
                    if (strncmp("uartPresent = ", cfgPtr, 14) == 0)
                    {
                        uartPresent = ((cfgPtr[14] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("usbPresent = ", cfgPtr, 13) == 0)
                    {
                        usbPresent = ((cfgPtr[13] == '1') ? true : false);
                        ++count;
                    }
                    break;
                case 'w' :
                    if (strncmp("wifiNtpPresent = ", cfgPtr, 17) == 0)
                    {
                        wifiNtpPresent = ((cfgPtr[17] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("wifiNtpTz = ", cfgPtr, 12) == 0)
                    {
                        wifiNtpTz = atol(&(cfgPtr[12]));
                        ++count;
                    }
                    break;
                default :
                    if (strncmp("bootIntoMenu = ", cfgPtr, 15) == 0)
                    {
                        bootIntoMenu = ((cfgPtr[15] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("gamepadButtons = ", cfgPtr, 17) == 0)
                    {
                        gamepadButtons = ((cfgPtr[17] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("interface1Present = ", cfgPtr, 20) == 0)
                    {
                        interface1Present = ((cfgPtr[20] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("printerPresent = ", cfgPtr, 17) == 0)
                    {
                        printerPresent = ((cfgPtr[17] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("lprintPresent = ", cfgPtr, 16) == 0)
                    {
                        lprintPresent = ((cfgPtr[16] == '1') ? true : false);
                        ++count;
                    } else if (strncmp("romName = ", cfgPtr, 10) == 0)
                    {
                        strncpy(cfgData.romName, &(cfgPtr[10]), MAX_PATH);
                        cfgData.romName[(MAX_PATH - 1)] = 0;
                        ++count;
                    } else if ((cfgCfgName == 0) &&
                        (strncmp("cfgName = ", cfgPtr, 10) == 0))
                    {
                        strncpy(cfgData.cfgName, &(cfgPtr[10]), MAX_PATH);
                        cfgData.cfgName[(MAX_PATH - 1)] = 0;
                        ++count;
                    }
                    break;
            }
        }
        cfgFile.close();
    }
    if (count > 0)
    {
        menuConfigChanged = ((cfgCfgName != 0) ? true : false);
    } else {
        menuConfigChanged = true;
        menuSaveConfiguration();
    }
}

void menuSaveConfiguration()
{
    // Save the configuration to SD card, if anything has changed
    if (menuConfigChanged)
    {
        menuConfigChanged = false;
        File cfgFile = SD.open(ZXTEENSY_CFG_PATH, FILE_WRITE_BEGIN);
        if (cfgFile)
        {
            // Update the configuration
            cfgData.romName[(MAX_PATH - 1)] = 0;
            cfgData.cfgName[(MAX_PATH - 1)] = 0;
            cfgData.divMmcSdaPath[(MAX_PATH - 1)] = 0;
            cfgData.divMmcSdbPath[(MAX_PATH - 1)] = 0;
            cfgData.dskFdaPath[(MAX_PATH - 1)] = 0;
            cfgData.dskFdbPath[(MAX_PATH - 1)] = 0;
            cfgData.modemUrl[(MAX_PATH - 1)] = 0;

            // Write configuration to file
            cfgFile.truncate();
            cfgFile.printf("divMmcPresent = %0d\n", divMmcPresent);
            cfgFile.printf("divMmcExtRamPresent = %0d\n", divMmcExtRamPresent);
            cfgFile.printf("divMmcRomPresent = %0d\n", divMmcRomPresent);
            cfgFile.printf("interface1Present = %0d\n", interface1Present);
            cfgFile.printf("mf128Present = %0d\n", mf128Present);
            cfgFile.printf("uartPresent = %0d\n", uartPresent);
            cfgFile.printf("usbPresent = %0d\n", usbPresent);
            cfgFile.printf("gamepadButtons = %0d\n", gamepadButtons);
            cfgFile.printf("wifiNtpPresent = %0d\n", wifiNtpPresent);
            cfgFile.printf("wifiNtpTz = %0d\n", wifiNtpTz);
            cfgFile.printf("dskPresent = %0d\n", dskPresent);
            cfgFile.printf("modemPresent = %0d\n", modemPresent);
            cfgFile.printf("printerPresent = %0d\n", printerPresent);
            cfgFile.printf("lprintPresent = %0d\n", lprintPresent);
            cfgFile.printf("bootIntoMenu = %0d\n", bootIntoMenu);
            cfgFile.printf("menuEnableInGame = %0d\n", menuEnableInGame);
            cfgFile.printf("cfgName = %s\n", cfgData.cfgName);
            cfgFile.printf("romName = %s\n", cfgData.romName);
            cfgFile.printf("divMmcSdaPath = %s\n", cfgData.divMmcSdaPath);
            cfgFile.printf("divMmcSdbPath = %s\n", cfgData.divMmcSdbPath);
            cfgFile.printf("dskFdaPath = %s\n", cfgData.dskFdaPath);
            cfgFile.printf("dskFdbPath = %s\n", cfgData.dskFdbPath);
            cfgFile.printf("modemUrl = %s\n", cfgData.modemUrl);
            cfgFile.close();
        }
    }
}

void menuClearPrinterFile()
{
    File outFile = SD.open(PRINTER_OUT_PATH, FILE_WRITE_BEGIN);
    if (outFile)
    {
        outFile.seek(0, SeekSet);
        outFile.truncate();
        outFile.close();
    }
}

inline bool menuIsDebugging()
{
    return (menuCurrent == MENU_TYPE_DEBUG);
}

inline void menuClearDebug()
{
    menuDebugIndex = 0;
    memset(menuDebugBuffer, 0, MENU_DEBUG_SIZE);
    menuHasDebug = false;
}

bool menuPrintDebug(bool clearDebug, const char *fmt, ...)
{
    if (clearDebug)
    {
        menuClearDebug();
    }
    if (menuDebugIndex < MENU_DEBUG_SIZE)
    {
        va_list ap;
        va_start(ap, fmt);
        char* ptr = &(menuDebugBuffer[menuDebugIndex]);
        size_t count = vsnprintf(ptr, (MENU_DEBUG_SIZE - menuDebugIndex), fmt, ap);
        if ((menuDebugIndex + count + 1) >= MENU_DEBUG_SIZE)
        {
            menuDebugBuffer[(MENU_DEBUG_SIZE - 1)] = 0;
            menuDebugIndex = MENU_DEBUG_SIZE;
        } else {
            menuDebugIndex += (strlen(ptr) + 1);
        }
        va_end(ap);
    }
    menuHasDebug = true;
    return (menuCurrent == MENU_TYPE_DEBUG);
}
