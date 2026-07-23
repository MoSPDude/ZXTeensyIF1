
#include "StringsZXTeensy.h"
#include "DefinesZXTeensy.h"

static const uint8_t CHAR_BORDER = 19;
static const uint8_t CHAR_DIR = 20;
static const uint8_t CHAR_TICK = 21;
// NOTE: The characters below are hard coded values in strings
static const uint8_t CHAR_EJECT = 22;
static const uint8_t CHAR_DSK = 23;
static const uint8_t CHAR_ZXC_L = 24;
static const uint8_t CHAR_ZXC_R = 25;
static const uint8_t CHAR_IF2_L = 26;
static const uint8_t CHAR_IF2_R = 27;
static const uint8_t CHAR_Z80_L = 28;
static const uint8_t CHAR_Z80_R = 29;
static const uint8_t CHAR_TZX_L = 30;
static const uint8_t CHAR_TZX_R = 31;

static const uint16_t MENU_TEXT_WIDTHS = 0x11F5;
static const uint16_t MENU_VERSION_STR = 0x11F8;
static const uint16_t MENU_NUM_ENTRIES = 0x11FB;
static const uint16_t MENU_NUM_PAGES = 0x11FE;

static const uint16_t MEM_BANK1 = 0x4FF;
static const uint16_t MEM_IM2 = 0x4FE;
static const uint16_t MEM_PC = 0x4FC;
static const uint16_t MEM_SPR = 0x4FA;
static const uint16_t MEM_UNUSED = 0x4F9;
static const uint16_t MEM_BORDER = 0x4F8;
static const uint16_t MEM_SP2 = 0x4F6;
static const uint16_t MEM_MODE = 0x1FF8;
static const uint16_t MEM_POS = 0x1FFD;
static const uint8_t Z80_MODE_48 = 0;
static const uint8_t Z80_MODE_128 = 1;
static const uint8_t Z80_MODE_UNKNOWN = 2;

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
    Z80_REG_AF = 22
} z80_regs_offset_t;

extern float tempmonGetTemp(void);

typedef enum {
    MENU_TYPE_MAIN,
    MENU_TYPE_SETTINGS,
    MENU_TYPE_CONFIGS,
    MENU_TYPE_LOAD_ROM,
    MENU_TYPE_NTP_TZ,
    MENU_TYPE_BROWSER,
    MENU_TYPE_BROWSER_EXPAND,
    MENU_TYPE_BROWSER_OPEN,
    MENU_TYPE_BROWSER_OPEN_ROM,
    MENU_TYPE_BROWSER_MOUNT_HDF,
    MENU_TYPE_BROWSER_MOUNT_DSK,
    MENU_TYPE_POK_BROWSER,
    MENU_TYPE_HTTP_SERVER,
    MENU_TYPE_IN_GAME,
    MENU_TYPE_TAPE_BROWSER,
    MENU_TYPE_SAVE_STATE_SLOT,
    MENU_TYPE_LOAD_STATE_SLOT,
    MENU_TYPE_PREVIEW_STATE_SLOT,
    MENU_TYPE_IN_GAME_SETTINGS,
    MENU_TYPE_DEBUG
} menu_type_t;

typedef struct {
    menu_action_t action;
    uint32_t index;
} menu_entry_t;

typedef struct {
    char cfgName[MAX_PATH];
    char romName[MAX_PATH];
    char divMmcSdaPath[MAX_PATH];
    char divMmcSdbPath[MAX_PATH];
    char dskFdaPath[MAX_PATH];
    char dskFdbPath[MAX_PATH];
    char modemUrl[MAX_PATH];
    char tapeFileName[MAX_PATH];
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
    SETTING_ACTION_TOGGLE_DIVMMC_LOCK_SD,
    SETTING_ACTION_TOGGLE_IF1,
    SETTING_ACTION_TOGGLE_MF128,
    SETTING_ACTION_TOGGLE_GENIE128,
    SETTING_ACTION_TOGGLE_USB,
    SETTING_ACTION_TOGGLE_FIRE_BUTTONS,
    SETTING_ACTION_TOGGLE_UART,
    SETTING_ACTION_TOGGLE_MODEM,
    SETTING_ACTION_TOGGLE_NTP,
    SETTING_ACTION_TOGGLE_FDC,
    SETTING_ACTION_TOGGLE_FDC_FDB,
    SETTING_ACTION_TOGGLE_PRINTER,
    SETTING_ACTION_TOGGLE_LPRINT,
    SETTING_ACTION_CLEAR_PRINTER,
    SETTING_ACTION_MOUNT_SD_SDA,
    SETTING_ACTION_UNMOUNT_SDA,
    SETTING_ACTION_MOUNT_SD_SDB,
    SETTING_ACTION_UNMOUNT_SDB,
    SETTING_ACTION_UNMOUNT_FDA,
    SETTING_ACTION_UNMOUNT_FDB,
    SETTING_ACTION_IN_GAME_TOGGLE_IF1,
    SETTING_ACTION_IN_GAME_TOGGLE_PRINTER,
    SETTING_ACTION_IN_GAME_TOGGLE_USB,
    SETTING_ACTION_OPEN_DEBUG = 0xF4,
    SETTING_ACTION_OPEN_IN_GAME_SETTINGS = 0xF5,
    SETTING_ACTION_OPEN_SAVE_STATE_SLOT = 0xF6,
    SETTING_ACTION_OPEN_LOAD_STATE_SLOT = 0xF7,
    SETTING_ACTION_OPEN_TAPE_BROWSER = 0xF8,
    SETTING_ACTION_OPEN_SERVER = 0xF9,
    SETTING_ACTION_OPEN_ROMS = 0xFA,
    SETTING_ACTION_OPEN_NTP_TZ = 0xFB,
    SETTING_ACTION_OPEN_BROWSER = 0xFC,
    SETTING_ACTION_OPEN_CONFIGS = 0xFD,
    SETTING_ACTION_OPEN_SETTINGS = 0xFE,
    SETTING_ACTION_INTERNAL_ROM = 0xFF
} settings_menu_action_t;

// Configuration data
DMAMEM cfg_data_t cfgData;
bool menuConfigChanged = false;
bool menuHasUpdateFw = false;
bool menuHasMdrEmu = false;

// Menu structure
char* menuPtr;
char* menuTxtPtr;
char* menuEndPtr;
uint8_t* menuTextWidths;
uint8_t menuEntries;
menu_type_t menuCurrent;
menu_type_t menuTopMenu;
menu_entry_t menu[255];
volatile menu_action_t menuAction;

// Menu file browser
static const uint8_t BROWSER_ENTRY_LIMIT = 254;
static const uint8_t BROWSER_SORT_ENTRY_LIMIT = 255;
static const uint32_t BROWSER_PARENT_INDEX = 0xFFFFFFFFUL;
static const size_t BROWSER_PIXEL_WIDTH = 247 - (3 * 8);
char menuBrowserPath[MAX_PATH];
uint32_t menuBrowserStartIndex = 0;
uint32_t menuBrowserExpandedIndex = BROWSER_PARENT_INDEX;
menu_action_t menuBrowserExpandedAction = MENU_ACTION_TOP_MENU;
uint8_t menuBrowserExpandedMenuIndex = 0;
uint8_t menuBrowserExpandedLineCount = 1;
uint8_t menuBrowserExpandedMenuLine = 0;

// Menu TZX and POK listings
char menuDynamicList[255][MENU_STR_LEN];

// Menu saved state preview
uint8_t menuPreviewSlot = 0;

typedef struct {
    uint32_t dirIndex;
    char sortKey[ROM_NAME_LEN];
    bool isDirectory;
} browser_sort_entry_t;

typedef struct {
    uint8_t start;
    uint8_t count;
    bool hasPrevious;
    bool hasNext;
} browser_page_batch_t;

// File browser large directory handling
browser_sort_entry_t menuBrowserLowerBound;
browser_sort_entry_t menuBrowserNextLowerBound;
bool menuBrowserHasLowerBound = false;
bool menuBrowserHasNextLowerBound = false;
bool menuBrowserPreviousPage = false;

bool menuSetFileSortEntry(browser_sort_entry_t* sortEntry, FsFile* entry,
    char* displayName);
int menuCompareBrowserFiles(const browser_sort_entry_t* first,
    const browser_sort_entry_t* second);
void menuSortBrowserFiles(browser_sort_entry_t* entries, uint8_t count);
void menuInsertSortedBrowserEntry(browser_sort_entry_t* entries, uint8_t* count,
    uint8_t limit, const browser_sort_entry_t* candidate);
void menuInsertPreviousBrowserEntry(browser_sort_entry_t* entries,
    uint8_t* count, uint8_t limit, const browser_sort_entry_t* candidate);
uint8_t menuCollectForwardBrowserEntries(FsFile& directory,
    const browser_sort_entry_t* afterEntry, browser_sort_entry_t* entries,
    uint8_t limit, char* displayName);
uint8_t menuCollectPreviousBrowserEntries(FsFile& directory,
    const browser_sort_entry_t* beforeEntry, browser_sort_entry_t* entries,
    uint8_t limit, char* displayName);
void menuResetBrowserPage();
menu_action_t menuGetBrowserFileAction(const char* filename,
    bool isDirectory, icon_type_t* icon);
bool menuSetBrowserExpandedAction(uint32_t fileIndex);
size_t menuGetSettingTextLength(const char* label);
size_t menuGetFileIconLength(icon_type_t icon);
uint8_t menuGetCollapsedBrowserFileMetrics(size_t length, const char* filename,
    size_t* textLength);
uint8_t menuGetExpandedBrowserFileMetrics(size_t length, const char* filename,
    size_t* textLength);
bool menuGetBrowserFileMetrics(FsFile& directory,
    const browser_sort_entry_t* browserEntry, char* displayName,
    bool allowExpanded, size_t* textLength, uint8_t* entryCount);
bool menuReserveBrowserBatchSpace(size_t* used, uint16_t* entries,
    size_t textLength, uint8_t entryCount);
bool menuReserveBrowserPageControls(size_t* used, uint16_t* entries,
    bool hasPrevious, bool hasNext);
browser_page_batch_t menuBatchForwardBrowserPage(FsFile& directory,
    browser_sort_entry_t* entries, uint8_t count, char* displayName,
    char* ptr, bool hasPrevious, bool allowExpanded);
browser_page_batch_t menuBatchPreviousBrowserPage(FsFile& directory,
    browser_sort_entry_t* entries, uint8_t count, char* displayName,
    char* ptr, bool allowExpanded);

// Menu page creation
static const uint8_t MENU_PAGE_ENTRY_COUNT = 21;
uint8_t menuPage;
uint8_t menuPageLine;
uint8_t menuRenderSourceEntry;
bool menuRenderSkipEnabled;
uint8_t menuRenderSkipStart;
uint8_t menuRenderSkipEnd;

// Debug menu text
static const size_t MENU_DEBUG_SIZE = (21 * MENU_TXT_LEN * 2);
char menuDebugBuffer[MENU_DEBUG_SIZE];
volatile size_t menuDebugIndex = 0;
volatile bool menuHasDebug = false;

bool menuSkipNextEntry()
{
    uint8_t entryIndex = menuRenderSourceEntry++;
    return (menuRenderSkipEnabled && (entryIndex >= menuRenderSkipStart) &&
        (entryIndex < menuRenderSkipEnd));
}

void menuInsertEntry(menu_action_t action, uint32_t index)
{
    menu[menuEntries].action = action;
    menu[menuEntries].index = index;
    ++menuEntries;
}

char* menuInsertLineEnd(char* ptr)
{
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

char* menuInsertFileIcon(icon_type_t icon, char* ptr)
{
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
    return ptr;
}

char* menuInsertSetting(menu_action_t action, uint32_t index, char* ptr, const char* label,
    bool checked)
{
    if (!menuSkipNextEntry())
    {
        menuInsertEntry(action, index);
        *ptr++ = (checked ? CHAR_TICK : CHAR_BORDER);
        unsigned int len = strlen(label);
        if (len > MENU_TXT_LEN)
        {
            for (size_t i = 0; i < (MENU_TXT_LEN - 1); ++i)
            {
                *ptr++ = ((label[i] >= 128) ? '?' : label[i]);
            }
            *ptr++ = '>';
        } else {
            for (size_t i = 0; i < len; ++i)
            {
                *ptr++ = ((label[i] >= 128) ? '?' : label[i]);
            }
        }

        // Add new line, and update menu dimensions
        ptr = menuInsertLineEnd(ptr);
    }
    return ptr;
}

inline __attribute__((always_inline)) char* menuInsertSpacer(char* ptr)
{
    return menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
        ptr, "", 0);
}

char* menuInsertEject(menu_action_t action, uint32_t index, char* ptr, const char* drive,
    const char* path)
{
    char label[(MENU_STR_LEN + 1)];
    if (path != 0)
    {
        // Parse filename from the full path
        const char* name = strrchr(path, '/');
        name = ((name != 0) ? (name + 1) : path);
        if (snprintf(label, (MENU_STR_LEN + 1), "  \x16  %s %s",
            drive, name) >= (MENU_STR_LEN + 1))
        {
            label[MENU_STR_LEN] = 0;
        }
    } else {
        // Drive is empty
        action = MENU_ACTION_SETTING;
        index = SETTING_ACTION_NO_OP;
        if (snprintf(label, (MENU_STR_LEN + 1), "  \x17  %s %s",
            drive, MENU_STRINGS[STRING_EMPTY_FDD]) >= (MENU_STR_LEN + 1))
        {
            label[MENU_STR_LEN] = 0;
        }
    }
    return menuInsertSetting(action, index, ptr, label, false);
}

char* menuInsertInGameStatus(char* ptr)
{
    // Show current ROMs
    char label[(MENU_STR_LEN + 1)];
    if (snprintf(label, (MENU_STR_LEN + 1), " > ROM: --------- RAM: %d%s",
        (spectrumBankM & 0x07), ((spectrumBankM & 0x20) ? " LOCK" : ""))
        >= (MENU_STR_LEN + 1))
    {
        label[MENU_STR_LEN] = 0;
    }
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
    if (IS_ROM_PAGED(ROM_MLD))
    {
        label[15] = 'D';
    } else if (mldPresent)
    {
        label[15] = 'd';
    }
    if (IS_ROM_PAGED(ROM_SNA))
    {
        label[16] = 'S';
    } else if (snaLoaderPresent)
    {
        label[16] = 's';
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
        ptr, label, 0);

    // Show current RAM
    if (zxC2Present || divMmcPresent)
    {
        char zxc2Label[(MENU_STR_LEN + 1)], divMmcLabel[(MENU_STR_LEN + 1)];
        if (zxC2Present)
        {
            if (snprintf(zxc2Label, (MENU_STR_LEN + 1), " \x18\x19%s: %d%s",
                zxC3Present ? "3" : "2", (zxC2RomBank >> 1),
                (zxC2Lock ? " LOCK" : "")) >= (MENU_STR_LEN + 1))
            {
                zxc2Label[MENU_STR_LEN] = 0;
            }
        } else {
            zxc2Label[0] = 0;
        }
        if (divMmcPresent)
        {
            if (snprintf(divMmcLabel, (MENU_STR_LEN + 1), " DivMMC: %d%s", divMmcRamBank,
                (divMmcMapRam ? " MAPRAM" : "")) >= (MENU_STR_LEN + 1))
            {
                divMmcLabel[MENU_STR_LEN] = 0;
            }
        } else {
            divMmcLabel[0] = 0;
        }
        if (snprintf(label, (MENU_STR_LEN + 1), " >%s%s", zxc2Label, divMmcLabel)
            >= (MENU_STR_LEN + 1))
        {
            label[MENU_STR_LEN] = 0;
        }
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
            ptr, label, 0);
    }
    return ptr;
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

char* menuInsertFile(menu_action_t action, icon_type_t icon, uint32_t index, char* ptr,
    const char* filename, bool canExpand)
{
    // Truncate the file name
    size_t i = 0;
    size_t width = 8;
    while (filename[i] != 0)
    {
        char c = ((filename[i] >= 128) ? '?' : filename[i]);
        width += *(menuTextWidths + (c - ' '));
        if (width < BROWSER_PIXEL_WIDTH)
        {
            *ptr++ = c;
            ++i;
        } else {
            *ptr++ = '>';
            if (canExpand)
            {
                action = MENU_ACTION_BROWSER_EXPAND;
            }
            break;
        }
    }

    // Add menu entry
    menuInsertEntry(action, index);

    // Add icon
    ptr = menuInsertFileIcon(icon, ptr);

    // Add new line, and update menu dimensions
    return menuInsertLineEnd(ptr);
}

char* menuInsertExpandedFile(menu_action_t action, icon_type_t icon, uint32_t index,
    char* ptr, const char* filename)
{
    size_t i = 0;
    bool firstLine = true;
    do
    {
        if (menuEntries < 255)
        {
            // Truncate the lines
            size_t width = 8;
            if (!firstLine)
            {
                *ptr++ = CHAR_BORDER;
                *ptr++ = ' ';
                *ptr++ = '>';
                *ptr++ = ' ';
                width += (3 * 8);
            }
            while (filename[i] != 0)
            {
                char c = ((filename[i] >= 128) ? '?' : filename[i]);
                width += *(menuTextWidths + (c - ' '));
                if (width < BROWSER_PIXEL_WIDTH)
                {
                    *ptr++ = c;
                    ++i;
                } else {
                    break;
                }
            }

            // Add menu entry
            menuInsertEntry(action, index);

            // Add icon on first line
            if (firstLine)
            {
                ptr = menuInsertFileIcon(icon, ptr);
                firstLine = false;
            }

            // Add new line, and update menu dimensions
            ptr = menuInsertLineEnd(ptr);
        } else {
            break;
        }
    } while ((filename[i] != 0) && (ptr <= menuEndPtr));
    return ptr;
}

char* menuAddBrowserFile(uint32_t index, char* ptr, const char* filename, bool isDirectory)
{
    if ((menuBrowserExpandedIndex == index) || !menuSkipNextEntry())
    {
        // Add directory icon for directories
        *ptr++ = (isDirectory ? CHAR_DIR : CHAR_BORDER);

        // Find the file extension and action
        icon_type_t icon;
        menu_action_t action = menuGetBrowserFileAction(filename, isDirectory, &icon);

        // Insert the menu entry
        if (menuBrowserExpandedIndex == index)
        {
            ptr = menuInsertExpandedFile(action, icon, index, ptr, filename);
        } else {
            ptr = menuInsertFile(action, icon, index, ptr, filename, true);
        }
    }
    return ptr;
}

char* menuAddLoadRomFile(uint32_t index, char* ptr, const char* filename)
{
    // Add check mark against active ROM
    *ptr++ = ((stricmp(filename, cfgData.romName) == 0) ? CHAR_TICK : CHAR_BORDER);

    // Find the file extension and action
    icon_type_t icon = ICON_TYPE_CART;
    const char *fileext = strrchr(filename, '.');
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
    return menuInsertFile(action, icon, index, ptr, filename, false);
}

char* menuAddConfigurationFile(uint32_t index, char* ptr, const char* filename)
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
        ptr = menuInsertFile(MENU_ACTION_LOAD_CFG, ICON_TYPE_NONE, index, ptr,
            name, false);
    }
    return ptr;
}

menu_action_t menuGetBrowserFileAction(const char* filename, bool isDirectory,
    icon_type_t* icon)
{
    // Add directory icons, and find the file extension
    if (isDirectory)
    {
        *icon = ICON_TYPE_NONE;
        return MENU_ACTION_BROWSER_CD;
    }

    menu_action_t action;
    const char *fileext = strrchr(filename, '.');
    if (fileext != 0)
    {
        if (stricmp(fileext + 1, "rom") == 0)
        {
            // Open ROM type selector, as ".rom" could be various types
            *icon = ICON_TYPE_CART;
            action = MENU_ACTION_BROWSER_OPEN_ROM;
        } else if (stricmp(fileext + 1, "bin") == 0)
        {
            // Open ROM type selector, as ".bin" could be various types
            *icon = ICON_TYPE_ZXC2;
            action = MENU_ACTION_BROWSER_OPEN_ROM;
        } else if (stricmp(fileext + 1, "mld") == 0)
        {
            *icon = ICON_TYPE_CART;
            action = MENU_ACTION_BROWSER_LOAD_MLD;
        } else if ((stricmp(fileext + 1, "z80") == 0) ||
            (stricmp(fileext + 1, "sna") == 0))
        {
            *icon = ICON_TYPE_Z80;
            action = MENU_ACTION_BROWSER_LOAD_Z80;
        } else if (((romArrayPresent & BANK_DIVMMC) != 0) &&
            ((stricmp(fileext + 1, "img") == 0) ||
                (stricmp(fileext + 1, "hdf") == 0)))
        {
            *icon = ICON_TYPE_DSK;
            action = MENU_ACTION_BROWSER_OPEN_HDF;
        } else if (stricmp(fileext + 1, "dsk") == 0)
        {
            *icon = ICON_TYPE_DSK;
            action = MENU_ACTION_BROWSER_OPEN_DSK;
        } else if (menuHasMdrEmu &&
            (stricmp(fileext + 1, "mdr") == 0))
        {
            *icon = ICON_TYPE_DSK;
            action = MENU_ACTION_BROWSER_LOAD_MDR;
        } else if ((stricmp(fileext + 1, "tap") == 0) ||
            (stricmp(fileext + 1, "tzx") == 0))
        {
            *icon = ICON_TYPE_TZX;
            action = MENU_ACTION_BROWSER_LOAD_TZX;
        } else if ((menuTopMenu == MENU_TYPE_IN_GAME) &&
            (stricmp(fileext + 1, "pok") == 0))
        {
            *icon = ICON_TYPE_NONE;
            action = MENU_ACTION_BROWSER_OPEN_POK;
        } else {
            *icon = ICON_TYPE_NONE;
            action = MENU_ACTION_BROWSER_OPEN;
        }
    } else {
        *icon = ICON_TYPE_NONE;
        action = MENU_ACTION_BROWSER_OPEN;
    }

    return action;
}

char* menuGenerateTapeBrowser(char* ptr)
{
    uint8_t count = ((tzxPlayer.tapeMarkCount >= 254) ?
        254 : tzxPlayer.tapeMarkCount);
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    for (uint8_t index = 0; index < count; ++index)
    {
        ptr = menuInsertSetting(MENU_ACTION_IN_GAME_SEEK_TAPE, index, ptr,
            menuDynamicList[index], 0);
    }
    return ptr;
}

char* menuGenerateSelectStateSlot(char* ptr, bool loadNotSave)
{
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    for (int8_t slot = 0; slot < STATE_POKE_SLOT; ++slot)
    {
        if (!loadNotSave || stateReadDeviceData(slot))
        {
            char label[MENU_STR_LEN + 1];
            if (snprintf(label, (MENU_STR_LEN + 1), "%s %d",
                (loadNotSave ? MENU_STRINGS[STRINGS_LOAD_STATE] :
                    ((slot == stateSaveSlot) ? MENU_STRINGS[STRINGS_SAVE_STATE] :
                        MENU_STRINGS[STRINGS_SELECT_STATE])),
                slot) >= (MENU_STR_LEN + 1))
            {
                label[MENU_STR_LEN] = 0;
            }
            ptr = menuInsertSetting((loadNotSave ?
                    MENU_ACTION_SELECT_LOAD_SLOT : MENU_ACTION_SELECT_SAVE_SLOT),
                slot, ptr, label, (slot == stateSaveSlot));
        }
    }
    return ptr;
}

char* menuGeneratePreviewStateSlot(char* ptr)
{
    // This menu is hidden behind the full-screen preview. The menu ROM
    // selects entry 0 for any direction, or entry 1 to confirm the load.
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_LOAD_STATE_SLOT,
        ptr, MENU_STRINGS[STRING_CANCEL], 0);
    return menuInsertSetting(MENU_ACTION_LOAD_STATE_SLOT, menuPreviewSlot, ptr,
        MENU_STRINGS[STRINGS_LOAD_STATE], 0);
}

char* menuGeneratePokBrowser(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, BROWSER_PARENT_INDEX,
        ptr, MENU_STRINGS[STRING_CANCEL], 0);
    if (pokeHasSelectedTrainers())
    {
        ptr = menuInsertSetting(MENU_ACTION_IN_GAME_APPLY_POK, 0, ptr,
            MENU_STRINGS[STRING_IN_GAME_APPLY_POK], 0);
    } else {
        ptr = menuInsertSpacer(ptr);
    }

    uint8_t count = pokeGetTrainerCount();
    for (uint8_t index = 0; index < count; ++index)
    {
        ptr = menuInsertSetting(MENU_ACTION_POK_TOGGLE_TRAINER, index, ptr,
            menuDynamicList[index], pokeIsTrainerSelected(index));
        if ((ptr > menuEndPtr) || (menuEntries == 255))
        {
            break;
        }
    }
    return ptr;
}

char* menuGenerateInGameSettings(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    if (interface1Present && divMmcPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_IN_GAME_TOGGLE_IF1,
            ptr, MENU_STRINGS[STRING_ENABLE_IF1], interface1Enabled);
    }
    if (printerPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_IN_GAME_TOGGLE_PRINTER,
            ptr, MENU_STRINGS[STRING_ENABLE_PRINTER], (printerEnabled || lprintEnabled));
        if (printerEnabled || lprintEnabled)
        {
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_CLEAR_PRINTER,
                ptr, MENU_STRINGS[STRING_CLEAR_PRINTER], 0);
        }
    }
    if (usbPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_IN_GAME_TOGGLE_USB, ptr,
            MENU_STRINGS[STRING_ENABLE_USB], usbEnabled);
        if (usbEnabled)
        {
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_FIRE_BUTTONS,
                ptr, MENU_STRINGS[STRING_ENABLE_FIRE_BUTTONS], gamepadButtons);
        }
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
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_BROWSER,
        ptr, MENU_STRINGS[STRING_OPEN_BROWSER], 0);
    char label[(MENU_STR_LEN + 1)];
    int8_t slot = ((stateSaveSlot >= 0) ? stateSaveSlot : 0);
    if (snprintf(label, (MENU_STR_LEN + 1), "%s %d", MENU_STRINGS[STRINGS_SAVE_STATE],
        slot) >= (MENU_STR_LEN + 1))
    {
        label[MENU_STR_LEN] = 0;
    }
    ptr = menuInsertSetting(MENU_ACTION_IN_GAME_SAVE_STATE, slot, ptr, label,
        (slot == stateActiveSlot));
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_SAVE_STATE_SLOT,
        ptr, MENU_STRINGS[STRING_OPEN_SAVE_STATE_SLOT], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_LOAD_STATE_SLOT,
        ptr, MENU_STRINGS[STRING_OPEN_LOAD_STATE_SLOT], 0);

    // Add tape and disk options
    if (tzxPresent)
    {
        if (tzxEnabled)
        {
            size_t position;
            size_t length = tzxPlayer.getPosition(&position);
            double temp = position;
            temp = (temp * 100) / length;
            int a = temp;
            temp *= 100;
            int b = (int)(temp) % 100;
            if (snprintf(label, (MENU_STR_LEN + 1), "  \x16  Tape %d.%02d%%", a, b) >=
                (MENU_STR_LEN + 1))
            {
                label[MENU_STR_LEN] = 0;
            }
            ptr = menuInsertSetting(MENU_ACTION_IN_GAME_EJECT_TAPE, 0,
                ptr, label, 0);
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_TAPE_BROWSER,
                ptr, MENU_STRINGS[STRING_OPEN_TAPE_BROWSER], 0);
        } else {
            if (snprintf(label, (MENU_STR_LEN + 1), " \x1E\x1F Tape %s",
                MENU_STRINGS[STRING_EMPTY_FDD]) >= (MENU_STR_LEN + 1))
            {
                label[MENU_STR_LEN] = 0;
            }
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP,
                ptr, label, 0);
        }
    }
    if (dskPresent)
    {
        ptr = menuInsertEject(MENU_ACTION_IN_GAME_UNMOUNT_FDA, 0, ptr,
            "A:", menuGetFdcFdaPath());
        if (dskEnableDriveB)
        {
            ptr = menuInsertEject(MENU_ACTION_IN_GAME_UNMOUNT_FDB, 0, ptr,
                "B:", menuGetFdcFdbPath());
        }
    }

    // Add peripheral and reset options
    ptr = menuInsertSpacer(ptr);
    ptr = menuInsertSetting(MENU_ACTION_IN_GAME_EXIT_BASIC, 0, ptr,
        MENU_STRINGS[STRING_IN_GAME_EXIT_BASIC], 0);
    ptr = menuInsertSetting(MENU_ACTION_IN_GAME_RESET, 0, ptr,
        MENU_STRINGS[STRING_IN_GAME_RESET], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_IN_GAME_SETTINGS,
        ptr, MENU_STRINGS[STRING_IN_GAME_SETTINGS], 0);

    // Add NMI options
    ptr = menuInsertSpacer(ptr);
    ptr = menuInsertSetting(MENU_ACTION_IN_GAME_NMI, 0, ptr,
        MENU_STRINGS[STRING_IN_GAME_NMI], 0);
    if (mf128Present && ((romArrayPresent & BANK_MF128) != 0) &&
        (menuPrevRomSelected <= ROM_MF128))
    {
        ptr = menuInsertSetting(MENU_ACTION_IN_GAME_MF128, 0, ptr,
            MENU_STRINGS[STRING_IN_GAME_MF128], 0);
    }
    if (divMmcRomEnabled && ((romArrayPresent & BANK_DIVMMC) != 0) &&
        (menuPrevRomSelected <= ROM_DIVMMC))
    {
        ptr = menuInsertSetting(MENU_ACTION_IN_GAME_DIVMMC, 0, ptr,
            MENU_STRINGS[STRING_IN_GAME_DIVMMC], 0);
    }

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
    while (menuEntries < 255)
    {
        while ((i < MENU_DEBUG_SIZE) && (menuDebugBuffer[i] < ' '))
        {
            ++i;
        }
        if (i < MENU_DEBUG_SIZE)
        {
            menuInsertEntry(MENU_ACTION_TOP_MENU, 0);
            *ptr++ = CHAR_BORDER;
            for (size_t j = 0; j < (ROM_NAME_LEN + 3); ++j)
            {
                if (menuDebugBuffer[i] >= ' ')
                {
                    *ptr++ = ((menuDebugBuffer[i] >= 128) ? '?' : menuDebugBuffer[i]);
                    ++i;
                } else {
                    ++i;
                    break;
                }
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
        } else {
            break;
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
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_NO_OP, ptr,
            httpServerStatus.c_str(), 0);
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

char* menuGenerateBrowserOpenRom(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, BROWSER_PARENT_INDEX,
        ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_CART, 0, ptr, MENU_STRINGS[STRING_LOAD_ROM], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_ZXC2, 0, ptr, MENU_STRINGS[STRING_LOAD_ZXC2], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_ZXC3, 0, ptr, MENU_STRINGS[STRING_LOAD_ZXC3], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_MLD, 0, ptr, MENU_STRINGS[STRING_LOAD_MLD], 0);
    if ((menuTopMenu != MENU_TYPE_MAIN) && mf128Present)
    {
        ptr = menuInsertSetting(MENU_ACTION_BROWSER_LOAD_MF128, 0, ptr, MENU_STRINGS[STRING_LOAD_MF128], 0);
    }
    return ptr;
}

char* menuGenerateBrowserMountDsk(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, BROWSER_PARENT_INDEX,
        ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_FDA, 0, ptr, MENU_STRINGS[STRING_MOUNT_FDA], 0);
    if (dskEnableDriveB)
    {
        ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_FDB, 0, ptr, MENU_STRINGS[STRING_MOUNT_FDB], 0);
    }
    return ptr;
}

char* menuGenerateBrowserMountHdf(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD, BROWSER_PARENT_INDEX,
        ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_SDA, 0, ptr, MENU_STRINGS[STRING_MOUNT_SDA], 0);
    ptr = menuInsertSetting(MENU_ACTION_BROWSER_MOUNT_SDB, 0, ptr, MENU_STRINGS[STRING_MOUNT_SDB], 0);
    return ptr;
}

char* menuGenerateBrowserOpen(char* ptr)
{
    ptr = menuGenerateBrowserOpenRom(ptr);
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

bool menuSetFileSortEntry(browser_sort_entry_t* sortEntry, FsFile* entry,
    char* displayName)
{
    if (!entry->getName(displayName, MAX_PATH))
    {
        return false;
    }

    sortEntry->dirIndex = entry->dirIndex();
    sortEntry->isDirectory = entry->isDirectory();
    memset(sortEntry->sortKey, 0, sizeof(sortEntry->sortKey));
    for (size_t index = 0;
        (index < sizeof(sortEntry->sortKey)) && (displayName[index] != 0);
        ++index)
    {
        char value = displayName[index];
        if ((value >= 'a') && (value <= 'z'))
        {
            value -= ('a' - 'A');
        }
        sortEntry->sortKey[index] = value;
    }
    return true;
}

int menuCompareBrowserFiles(const browser_sort_entry_t* first,
    const browser_sort_entry_t* second)
{
    // Keep directories together at the start of the listing
    if (first->isDirectory != second->isDirectory)
    {
        return first->isDirectory ? -1 : 1;
    }

    int result = memcmp(first->sortKey, second->sortKey,
        sizeof(first->sortKey));
    if (result != 0)
    {
        return result;
    }

    // Entries with matching visible file names retain directory order
    if (first->dirIndex != second->dirIndex)
    {
        return ((first->dirIndex < second->dirIndex) ? -1 : 1);
    }
    return 0;
}

void menuSortBrowserFiles(browser_sort_entry_t* entries, uint8_t count)
{
    // Insertion sort is stable and only moves the compact metadata
    for (uint8_t index = 1; index < count; ++index)
    {
        browser_sort_entry_t entry = entries[index];
        uint8_t position = index;
        while ((position > 0) &&
            (menuCompareBrowserFiles(&entry, &(entries[position - 1])) < 0))
        {
            entries[position] = entries[position - 1];
            --position;
        }
        entries[position] = entry;
    }
}

void menuInsertSortedBrowserEntry(browser_sort_entry_t* entries, uint8_t* count,
    uint8_t limit, const browser_sort_entry_t* candidate)
{
    if (limit == 0)
    {
        return;
    }

    uint8_t position;
    if (*count < limit)
    {
        position = *count;
        ++(*count);
    } else {
        if (menuCompareBrowserFiles(candidate, &(entries[limit - 1])) >= 0)
        {
            return;
        }
        position = (limit - 1);
    }

    while ((position > 0) &&
        (menuCompareBrowserFiles(candidate, &(entries[position - 1])) < 0))
    {
        entries[position] = entries[position - 1];
        --position;
    }
    entries[position] = *candidate;
}

void menuInsertPreviousBrowserEntry(browser_sort_entry_t* entries,
    uint8_t* count, uint8_t limit, const browser_sort_entry_t* candidate)
{
    if (limit == 0)
    {
        return;
    }

    uint8_t position;
    if (*count < limit)
    {
        position = *count;
        ++(*count);
    } else {
        if (menuCompareBrowserFiles(candidate, &(entries[0])) <= 0)
        {
            return;
        }
        for (position = 1; position < limit; ++position)
        {
            entries[position - 1] = entries[position];
        }
        position = (limit - 1);
    }

    while ((position > 0) &&
        (menuCompareBrowserFiles(candidate, &(entries[position - 1])) < 0))
    {
        entries[position] = entries[position - 1];
        --position;
    }
    entries[position] = *candidate;
}

uint8_t menuCollectForwardBrowserEntries(FsFile& directory,
    const browser_sort_entry_t* afterEntry, browser_sort_entry_t* entries,
    uint8_t limit, char* displayName)
{
    uint8_t count = 0;
    if (limit == 0)
    {
        return count;
    }

    // Iterate the directory, and sort into a limited window - keeping track of
    // the entry just before, and just after
    while (true)
    {
        FsFile entry = directory.openNextFile(O_RDONLY);
        if (entry)
        {
            browser_sort_entry_t candidate;
            if (menuSetFileSortEntry(&candidate, &entry,
                displayName))
            {
                int compareAfter = ((afterEntry == 0) ? 1 :
                    menuCompareBrowserFiles(&candidate, afterEntry));
                if (compareAfter > 0)
                {
                    menuInsertSortedBrowserEntry(entries, &count, limit,
                        &candidate);
                }
            }
            entry.close();
        } else {
            break;
        }
    }
    return count;
}

uint8_t menuCollectPreviousBrowserEntries(FsFile& directory,
    const browser_sort_entry_t* beforeEntry, browser_sort_entry_t* entries,
    uint8_t limit, char* displayName)
{
    uint8_t count = 0;
    if ((limit == 0) || (beforeEntry == 0))
    {
        return count;
    }

    while (true)
    {
        FsFile entry = directory.openNextFile(O_RDONLY);
        if (entry)
        {
            browser_sort_entry_t candidate;
            if (menuSetFileSortEntry(&candidate, &entry, displayName) &&
                (menuCompareBrowserFiles(&candidate, beforeEntry) <= 0))
            {
                menuInsertPreviousBrowserEntry(entries, &count, limit,
                    &candidate);
            }
            entry.close();
        } else {
            break;
        }
    }
    return count;
}

void menuResetBrowserPage()
{
    menuBrowserStartIndex = 0;
    menuBrowserHasLowerBound = false;
    menuBrowserHasNextLowerBound = false;
    menuBrowserPreviousPage = false;
    menuBrowserExpandedIndex = BROWSER_PARENT_INDEX;
    menuBrowserExpandedAction = MENU_ACTION_TOP_MENU;
    menuBrowserExpandedMenuIndex = 0;
    menuBrowserExpandedLineCount = 1;
    menuBrowserExpandedMenuLine = 0;
}

bool menuSetBrowserExpandedAction(uint32_t fileIndex)
{
    bool result = false;
    FsFile directory = SD.sdfs.open(menuBrowserPath, O_RDONLY);
    if (directory)
    {
        if (directory.isDirectory())
        {
            FsFile entry;
            if (entry.open(&directory, fileIndex, O_RDONLY))
            {
                char name[MAX_PATH];
                if (entry.getName(name, MAX_PATH))
                {
                    icon_type_t icon;
                    size_t textLength;
                    menuBrowserExpandedAction =
                        menuGetBrowserFileAction(name, entry.isDirectory(), &icon);
                    menuBrowserExpandedLineCount =
                        menuGetExpandedBrowserFileMetrics(0, name, &textLength);
                    result = true;
                }
                entry.close();
            }
        }
        directory.close();
    }
    return result;
}

uint8_t menuGetCollapsedBrowserFileMetrics(size_t length, const char* filename,
    size_t* textLength)
{
    size_t i = 0;
    size_t width = 8;
    ++length;
    while (filename[i] != 0)
    {
        char c = ((filename[i] >= 128) ? '?' : filename[i]);
        width += *(menuTextWidths + (c - ' '));
        ++length;
        if (width < BROWSER_PIXEL_WIDTH)
        {
            ++i;
        } else {
            break;
        }
    }

    // Provide text length, and number of lines
    *textLength = length + 1;
    return 1;
}

uint8_t menuGetExpandedBrowserFileMetrics(size_t length, const char* filename,
    size_t* textLength)
{
    size_t i = 0;
    uint8_t lines = 0;
    bool firstLine = true;
    do
    {
        size_t width = 8;
        if (firstLine)
        {
            ++length;
            firstLine = false;
        } else {
            length += 4;
            width += (3 * 8);
        }
        while (filename[i] != 0)
        {
            char c = ((filename[i] >= 128) ? '?' : filename[i]);
            width += *(menuTextWidths + (c - ' '));
            if (width < BROWSER_PIXEL_WIDTH)
            {
                ++length;
                ++i;
            } else {
                break;
            }
        }
        ++length;
        ++lines;
    } while (filename[i] != 0);

    // Provide text length, and number of lines
    *textLength = length;
    return lines;
}

size_t menuGetFileIconLength(icon_type_t icon)
{
    switch (icon)
    {
        case ICON_TYPE_DSK :
        case ICON_TYPE_ZXC2 :
        case ICON_TYPE_CART :
        case ICON_TYPE_Z80 :
        case ICON_TYPE_TZX :
            return 3;
        default :
            return 0;
    }
}

bool menuGetBrowserFileMetrics(FsFile& directory,
    const browser_sort_entry_t* browserEntry, char* displayName,
    bool allowExpanded, size_t* textLength, uint8_t* entryCount)
{
    // Measure the exact text bytes and action entries that this browser item
    // will consume before writing it into the menu RAM page.
    bool result = false;
    FsFile entry;
    if (entry.open(&directory, browserEntry->dirIndex, O_RDONLY))
    {
        if (entry.getName(displayName, MAX_PATH))
        {
            icon_type_t icon;
            menuGetBrowserFileAction(displayName, browserEntry->isDirectory,
                &icon);
            size_t length = menuGetFileIconLength(icon);
            if (allowExpanded &&
                (menuBrowserExpandedIndex == browserEntry->dirIndex))
            {
                *entryCount = menuGetExpandedBrowserFileMetrics(length, displayName,
                    textLength);
            } else {
                *entryCount = menuGetCollapsedBrowserFileMetrics(length, displayName,
                    textLength);
            }
            result = true;
        }
        entry.close();
    }
    return result;
}

bool menuReserveBrowserBatchSpace(size_t* used, uint16_t* entries,
    size_t textLength, uint8_t entryCount)
{
    if (((*entries) + entryCount) > 255)
    {
        return false;
    }
    if (((*used) + textLength) > (size_t)(menuEndPtr - menuTxtPtr))
    {
        return false;
    }

    *entries += entryCount;
    *used += textLength;
    return true;
}

size_t menuGetSettingTextLength(const char* label)
{
    size_t length = strlen(label);
    if (length > MENU_TXT_LEN)
    {
        length = MENU_TXT_LEN;
    }
    return (length + 2);
}

bool menuReserveBrowserPageControls(size_t* used, uint16_t* entries,
    bool hasPrevious, bool hasNext)
{
    // Reserve the page controls up front so the file batch leaves room for
    // the previous and duplicated next-page entries.
    if (hasPrevious &&
        !menuReserveBrowserBatchSpace(used, entries,
            menuGetSettingTextLength(MENU_STRINGS[STRING_PREVIOUS_PAGE]), 1))
    {
        return false;
    }

    if (hasNext)
    {
        size_t nextLength =
            menuGetSettingTextLength(MENU_STRINGS[STRING_NEXT_PAGE]);
        if (!menuReserveBrowserBatchSpace(used, entries, nextLength, 1) ||
            !menuReserveBrowserBatchSpace(used, entries, nextLength, 1))
        {
            return false;
        }
    }
    return true;
}

browser_page_batch_t menuBatchForwardBrowserPage(FsFile& directory,
    browser_sort_entry_t* entries, uint8_t count, char* displayName,
    char* ptr, bool hasPrevious, bool allowExpanded)
{
    browser_page_batch_t batch = {0, 0, hasPrevious, false};
    size_t noNextUsed = (size_t)(ptr - menuTxtPtr);
    size_t withNextUsed = noNextUsed;
    uint16_t noNextEntries = menuEntries;
    uint16_t withNextEntries = menuEntries;
    bool noNextCanFit = menuReserveBrowserPageControls(&noNextUsed,
        &noNextEntries, hasPrevious, false);
    bool withNextCanFit = menuReserveBrowserPageControls(&withNextUsed,
        &withNextEntries, hasPrevious, true);
    uint8_t noNextCount = 0;
    uint8_t withNextCount = 0;

    if (!noNextCanFit && !withNextCanFit)
    {
        return batch;
    }

    // Build both possible forward batches in one directory pass: one that
    // assumes no next page exists, and one that reserves room for it.
    for (uint8_t index = 0; index < count; ++index)
    {
        size_t textLength = 0;
        uint8_t entryCount = 0;
        bool haveMetrics = menuGetBrowserFileMetrics(directory,
            &(entries[index]), displayName, allowExpanded, &textLength,
            &entryCount);

        if (noNextCanFit)
        {
            if (!haveMetrics ||
                menuReserveBrowserBatchSpace(&noNextUsed, &noNextEntries,
                    textLength, entryCount))
            {
                ++noNextCount;
            } else {
                noNextCanFit = false;
            }
        }

        if (withNextCanFit)
        {
            if (!haveMetrics ||
                menuReserveBrowserBatchSpace(&withNextUsed, &withNextEntries,
                    textLength, entryCount))
            {
                ++withNextCount;
            } else {
                withNextCanFit = false;
            }
        }

        if (!noNextCanFit && !withNextCanFit)
        {
            break;
        }
    }

    batch.hasNext = ((noNextCount < count) ||
        (count == BROWSER_SORT_ENTRY_LIMIT));
    batch.count = (batch.hasNext ? withNextCount : noNextCount);
    if (batch.count == 0)
    {
        batch.hasNext = false;
    }
    return batch;
}

browser_page_batch_t menuBatchPreviousBrowserPage(FsFile& directory,
    browser_sort_entry_t* entries, uint8_t count, char* displayName,
    char* ptr, bool allowExpanded)
{
    browser_page_batch_t batch = {0, 0, false, false};
    bool hasNext = (count > 0);
    size_t noPreviousUsed = (size_t)(ptr - menuTxtPtr);
    size_t withPreviousUsed = noPreviousUsed;
    uint16_t noPreviousEntries = menuEntries;
    uint16_t withPreviousEntries = menuEntries;
    bool noPreviousCanFit = menuReserveBrowserPageControls(&noPreviousUsed,
        &noPreviousEntries, false, hasNext);
    bool withPreviousCanFit = menuReserveBrowserPageControls(&withPreviousUsed,
        &withPreviousEntries, true, hasNext);
    uint8_t noPreviousStart = count;
    uint8_t withPreviousStart = count;

    if (!noPreviousCanFit && !withPreviousCanFit)
    {
        return batch;
    }

    // Build both possible previous batches from the end of the collected
    // window, keeping the newest entries visible while preserving controls.
    for (uint8_t index = count; index > 0; --index)
    {
        size_t textLength = 0;
        uint8_t entryCount = 0;
        bool haveMetrics = menuGetBrowserFileMetrics(directory,
            &(entries[index - 1]), displayName, allowExpanded, &textLength,
            &entryCount);

        if (noPreviousCanFit)
        {
            if (!haveMetrics ||
                menuReserveBrowserBatchSpace(&noPreviousUsed,
                    &noPreviousEntries, textLength, entryCount))
            {
                noPreviousStart = index - 1;
            } else {
                noPreviousCanFit = false;
            }
        }

        if (withPreviousCanFit)
        {
            if (!haveMetrics ||
                menuReserveBrowserBatchSpace(&withPreviousUsed,
                    &withPreviousEntries, textLength, entryCount))
            {
                withPreviousStart = index - 1;
            } else {
                withPreviousCanFit = false;
            }
        }

        if (!noPreviousCanFit && !withPreviousCanFit)
        {
            break;
        }
    }

    batch.hasPrevious = ((noPreviousStart > 0) ||
        (count == BROWSER_SORT_ENTRY_LIMIT));
    batch.start = (batch.hasPrevious ? withPreviousStart : noPreviousStart);
    batch.count = count - batch.start;
    batch.hasPrevious = (batch.start > 0);
    batch.hasNext = (batch.count > 0);
    return batch;
}

char* menuGenerateBrowser(char* ptr)
{
    browser_sort_entry_t browserSortEntries[BROWSER_SORT_ENTRY_LIMIT];
    char browserDisplayName[MAX_PATH];

    if (beginSdfsSd())
    {
        // Open the directory
        FsFile directory = SD.sdfs.open(menuBrowserPath, O_RDONLY);
        if (!directory)
        {
            // If the stored path has gone away, recover to the root browser.
            strcpy(menuBrowserPath, "/");
            menuResetBrowserPage();
            directory = SD.sdfs.open(menuBrowserPath, O_RDONLY);
        }

        // Scan the directory to find the sorted window to render
        if (directory)
        {
            uint8_t count = 0;
            browser_page_batch_t batch = {0, 0, false, false};
            bool renderView = false;
            if (directory.isDirectory())
            {
                // Expanded filenames are drawn over the current listing, so
                // do not advance or rewind the browser cursors for that view.
                bool allowExpanded = (menuCurrent == MENU_TYPE_BROWSER_EXPAND);
                bool updatePageState = !allowExpanded;
                renderView = true;
                ptr = menuInsertSetting(MENU_ACTION_BROWSER_CD,
                    BROWSER_PARENT_INDEX, ptr, "..", 0);
                if (menuBrowserPreviousPage && menuBrowserHasLowerBound)
                {
                    // Moving backwards collects entries up to the current lower
                    // bound, then keeps the last batch that fits the menu page.
                    browser_sort_entry_t nextLowerBound =
                        menuBrowserLowerBound;
                    count = menuCollectPreviousBrowserEntries(directory,
                        &nextLowerBound, browserSortEntries,
                        BROWSER_SORT_ENTRY_LIMIT, browserDisplayName);

                    batch = menuBatchPreviousBrowserPage(directory,
                        browserSortEntries, count, browserDisplayName,
                        ptr, allowExpanded);

                    if (updatePageState)
                    {
                        // Shift the lower bound to the entry immediately before
                        // this batch, or clear it when we reached the first page.
                        if (batch.hasPrevious)
                        {
                            menuBrowserLowerBound =
                                browserSortEntries[batch.start - 1];
                            menuBrowserHasLowerBound = true;
                        } else {
                            menuBrowserHasLowerBound = false;
                            menuBrowserStartIndex = 0;
                        }
                        menuBrowserNextLowerBound = nextLowerBound;
                        menuBrowserHasNextLowerBound = batch.hasNext;
                    }
                } else {
                    // Moving forwards collects entries after the current lower
                    // bound, then keeps the first batch that fits the menu page.
                    batch.hasPrevious = menuBrowserHasLowerBound;
                    count = menuCollectForwardBrowserEntries(directory,
                        (menuBrowserHasLowerBound ?
                            &menuBrowserLowerBound : 0),
                        browserSortEntries, BROWSER_SORT_ENTRY_LIMIT,
                        browserDisplayName);

                    batch = menuBatchForwardBrowserPage(directory,
                        browserSortEntries, count, browserDisplayName, ptr,
                        batch.hasPrevious, allowExpanded);

                    if (updatePageState)
                    {
                        // The last rendered entry becomes the cursor for the
                        // next forward batch.
                        if (batch.hasNext && (batch.count > 0))
                        {
                            menuBrowserNextLowerBound =
                                browserSortEntries[batch.count - 1];
                            menuBrowserHasNextLowerBound = true;
                        } else {
                            menuBrowserHasNextLowerBound = false;
                        }
                    }
                }
                menuBrowserPreviousPage = false;

                // Add previous and next page selections before the filenames
                // so they have stable action indexes on the first menu page.
                if (batch.hasPrevious)
                {
                    ptr = menuInsertSetting(MENU_ACTION_BROWSER_PAGE, 0, ptr,
                        MENU_STRINGS[STRING_PREVIOUS_PAGE], 0);
                }
                if (batch.hasNext)
                {
                    ptr = menuInsertSetting(MENU_ACTION_BROWSER_PAGE, 1, ptr,
                        MENU_STRINGS[STRING_NEXT_PAGE], 0);
                }
            } else {
                ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr,
                    MENU_STRINGS[STRING_CANCEL], 0);
            }

            // Close the directory before re-opening for render
            directory.close();

            // Render the file names in the sorted window
            if (renderView && (batch.count > 0))
            {
                directory = SD.sdfs.open(menuBrowserPath, O_RDONLY);
                if (directory)
                {
                    for (uint8_t index = 0; index < batch.count; ++index)
                    {
                        const browser_sort_entry_t* browserEntry =
                            &(browserSortEntries[index + batch.start]);
                        FsFile entry;
                        if (entry.open(&directory, browserEntry->dirIndex, O_RDONLY))
                        {
                            if (entry.getName(browserDisplayName, MAX_PATH))
                            {
                                ptr = menuAddBrowserFile(browserEntry->dirIndex, ptr,
                                    browserDisplayName, browserEntry->isDirectory);
                            }
                            entry.close();
                        }
                        if ((ptr > menuEndPtr) || (menuEntries == 255))
                        {
                            break;
                        }
                    }
                    if (batch.hasNext && (menuEntries < 255) &&
                        (ptr <= menuEndPtr))
                    {
                        // Duplicate "next page" after the filenames so paging
                        // remains reachable at the bottom of the listing.
                        ptr = menuInsertSetting(MENU_ACTION_BROWSER_PAGE, 1, ptr,
                            MENU_STRINGS[STRING_NEXT_PAGE], 0);
                    }
                    directory.close();
                }
            }
        } else {
            ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr,
                MENU_STRINGS[STRING_CANCEL], 0);
        }
    } else {
        ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr,
            MENU_STRINGS[STRING_CANCEL], 0);
    }
    return ptr;
}

char* menuGenerateBrowserExpand(char* ptr)
{
    uint8_t shiftRows = 0;
    if ((menuBrowserExpandedMenuLine + menuBrowserExpandedLineCount) >
        MENU_PAGE_ENTRY_COUNT)
    {
        shiftRows = (menuBrowserExpandedMenuLine +
            menuBrowserExpandedLineCount) - MENU_PAGE_ENTRY_COUNT;
        if (shiftRows > menuBrowserExpandedMenuLine)
        {
            shiftRows = menuBrowserExpandedMenuLine;
        }
    }

    menuRenderSkipEnabled = (shiftRows > 0);
    menuRenderSkipStart = menuBrowserExpandedMenuIndex - shiftRows;
    menuRenderSkipEnd = menuBrowserExpandedMenuIndex;
    menuTxtPtr[MEM_POS] = menuBrowserExpandedMenuLine - shiftRows;

    ptr = menuGenerateBrowser(ptr);
    menuRenderSkipEnabled = false;

    // The listing above is displayed, but the ROM modal sends only entry 0
    // to collapse or entry 1 to perform the original browser action.
    menuEntries = 0;
    menuInsertEntry(MENU_ACTION_BROWSER_EXPAND, menuBrowserExpandedIndex);
    menuInsertEntry(menuBrowserExpandedAction, menuBrowserExpandedIndex);
    return ptr;
}

char* menuGenerateLoadRom(char* ptr)
{
    browser_sort_entry_t romSortEntries[BROWSER_ENTRY_LIMIT];
    char romDisplayName[MAX_PATH];

    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_INTERNAL_ROM,
        ptr, MENU_STRINGS[STRING_INTERNAL_ROM], (cfgData.romName[0] == 0));
    FsFile romDirectory = SD.sdfs.open("ROMS", O_RDONLY);
    if (romDirectory)
    {
        if (romDirectory.isDirectory())
        {
            uint8_t count = 0;
            while (count < (BROWSER_ENTRY_LIMIT - 1))
            {
                FsFile entry = romDirectory.openNextFile(O_RDONLY);
                if (entry)
                {
                    if (!entry.isDirectory())
                    {
                        browser_sort_entry_t* romEntry = &(romSortEntries[count]);
                        if (menuSetFileSortEntry(romEntry, &entry,
                            romDisplayName))
                        {
                            ++count;
                        }
                    }
                    entry.close();
                } else {
                    // End of listing
                    break;
                }
            }

            menuSortBrowserFiles(romSortEntries, count);
            for (uint8_t index = 0; index < count; ++index)
            {
                const browser_sort_entry_t* romEntry = &(romSortEntries[index]);
                FsFile entry;
                if (entry.open(&romDirectory, romEntry->dirIndex, O_RDONLY))
                {
                    if (entry.getName(romDisplayName, MAX_PATH))
                    {
                        ptr = menuAddLoadRomFile(romEntry->dirIndex, ptr,
                            romDisplayName);
                    }
                    entry.close();
                }
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
    // Add settings menu
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);
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
        bool isSdSda = false;
        char* tmpPath = menuGetDivMmcSdaPath();
        if (tmpPath != 0)
        {
            if (stricmp("/", tmpPath) != 0)
            {
                File tmpFile = SD.open(tmpPath, FILE_READ);
                if (tmpFile)
                {
                    ptr = menuInsertEject(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_SDA,
                        ptr, "sda", tmpPath);
                    tmpFile.close();
                } else {
                    cfgData.divMmcSdaPath[0] = 0;
                }
            } else {
                ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_DIVMMC_LOCK_SD,
                    ptr, MENU_STRINGS[STRING_DIVMMC_LOCK_SD], divMmcSdReadOnly);
                ptr = menuInsertEject(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_SDA,
                    ptr, "sda", MENU_STRINGS[STRING_SD_CARD]);
                isSdSda = true;
            }
        }
        if (menuGetDivMmcSdaPath() == 0)
        {
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_MOUNT_SD_SDA,
                ptr, MENU_STRINGS[STRING_MOUNT_SD_SDA], 0);
        }
        tmpPath = menuGetDivMmcSdbPath();
        if (tmpPath != 0)
        {
            if (stricmp("/", tmpPath) != 0)
            {
                File tmpFile = SD.open(tmpPath, FILE_READ);
                if (tmpFile)
                {
                    ptr = menuInsertEject(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_SDB,
                        ptr, "sdb", tmpPath);
                    tmpFile.close();
                } else {
                    cfgData.divMmcSdbPath[0] = 0;
                }
            } else if (isSdSda)
            {
                cfgData.divMmcSdbPath[0] = 0;
            } else {
                ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_DIVMMC_LOCK_SD,
                    ptr, MENU_STRINGS[STRING_DIVMMC_LOCK_SD], divMmcSdReadOnly);
                ptr = menuInsertEject(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_SDB,
                    ptr, "sdb", MENU_STRINGS[STRING_SD_CARD]);
            }
        }
        if (!isSdSda && (menuGetDivMmcSdbPath() == 0))
        {
            ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_MOUNT_SD_SDB,
                ptr, MENU_STRINGS[STRING_MOUNT_SD_SDB], 0);
        }
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_FDC,
        ptr, MENU_STRINGS[STRING_ENABLE_FDC], dskPresent);
    if (dskPresent)
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_FDC_FDB,
            ptr, MENU_STRINGS[STRING_ENABLE_FDC_FDB], dskEnableDriveB);
        ptr = menuInsertEject(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_FDA, ptr,
            "A:", menuGetFdcFdaPath());
        if (dskEnableDriveB)
        {
            ptr = menuInsertEject(MENU_ACTION_SETTING, SETTING_ACTION_UNMOUNT_FDB,
                ptr, "B:", menuGetFdcFdbPath());
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
        if (mf128Present)
        {
            File tmpFile = SD.open(GENIE128_ROM_PATH, FILE_READ);
            if (tmpFile)
            {
                ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_GENIE128,
                    ptr, MENU_STRINGS[STRING_ENABLE_GENIE128], mf128LoadGenie);
                tmpFile.close();
            }
        }
    }
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_PRINTER,
        ptr, MENU_STRINGS[STRING_ENABLE_PRINTER], printerPresent);
    if (printerPresent && ((romArrayPresent & BANK_LPRINT) != 0))
    {
        ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_TOGGLE_LPRINT,
            ptr, MENU_STRINGS[STRING_ENABLE_LPRINT], lprintPresent);
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
            if (snprintf(label, (MENU_STR_LEN + 1), " > %s", cfgData.modemUrl) >=
                (MENU_STR_LEN + 1))
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
    File toolFile = SD.open(RTC_SETUP_Z80_PATH, FILE_READ);
    if (toolFile)
    {
        hasToolSpacer = true;
        ptr = menuInsertSpacer(ptr);
        ptr = menuInsertSetting(MENU_ACTION_LOAD_RTC_SETUP, 0, ptr,
            MENU_STRINGS[STRING_LOAD_RTC_CONFIG], 0);
        toolFile.close();
    }
    toolFile = SD.open(NETMAN_Z80_PATH, FILE_READ);
    if (toolFile)
    {
        if (!hasToolSpacer)
        {
            ptr = menuInsertSpacer(ptr);
        }
        ptr = menuInsertSetting(MENU_ACTION_LOAD_NETMAN, 0, ptr,
            MENU_STRINGS[STRING_LOAD_NETMAN], 0);
        toolFile.close();
    }
    return ptr;
}

char* menuGenerateConfigurations(char* ptr)
{
    browser_sort_entry_t cfgSortEntries[BROWSER_ENTRY_LIMIT];
    char cfgDisplayName[MAX_PATH];
    ptr = menuInsertSetting(MENU_ACTION_TOP_MENU, 0, ptr, MENU_STRINGS[STRING_CANCEL], 0);

    // List stored configurations for quick selection
    FsFile cfgDirectory = SD.sdfs.open("/ZXTEENSY/CONFIGS", O_RDONLY);
    if (cfgDirectory)
    {
        if (cfgDirectory.isDirectory())
        {
            uint8_t count = 0;
            while (count < BROWSER_ENTRY_LIMIT)
            {
                FsFile entry = cfgDirectory.openNextFile(O_RDONLY);
                if (entry)
                {
                    if (!entry.isDirectory())
                    {
                        browser_sort_entry_t* cfgEntry =
                            &(cfgSortEntries[count]);
                        if (menuSetFileSortEntry(cfgEntry, &entry,
                            cfgDisplayName))
                        {
                            char* fileext = strrchr(cfgDisplayName, '.');
                            if ((fileext != 0) &&
                                (stricmp(fileext + 1, "cfg") == 0))
                            {
                                ++count;
                            }
                        }
                    }
                    entry.close();
                } else {
                    // End of listing
                    break;
                }
            }

            menuSortBrowserFiles(cfgSortEntries, count);
            for (uint8_t index = 0; index < count; ++index)
            {
                const browser_sort_entry_t* cfgEntry =
                    &(cfgSortEntries[index]);
                FsFile entry;
                if (entry.open(&cfgDirectory, cfgEntry->dirIndex, O_RDONLY))
                {
                    if (entry.getName(cfgDisplayName, MAX_PATH))
                    {
                        ptr = menuAddConfigurationFile(cfgEntry->dirIndex, ptr,
                            cfgDisplayName);
                    }
                    entry.close();
                }

                if ((ptr > menuEndPtr) || (menuEntries == 255))
                {
                    break;
                }
            }
        }
        cfgDirectory.close();
    }
    return ptr;
}

char* menuGenerateMain(char* ptr)
{
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_RESTART,
        ptr, MENU_STRINGS[STRING_RESTART], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_BROWSER,
        ptr, MENU_STRINGS[STRING_OPEN_BROWSER], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_ROMS,
        ptr, MENU_STRINGS[STRING_OPEN_ROMS], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_LOAD_STATE_SLOT,
        ptr, MENU_STRINGS[STRING_OPEN_LOAD_STATE_SLOT], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_SERVER,
        ptr, MENU_STRINGS[STRING_OPEN_HTTP_SERVER], 0);
    ptr = menuInsertSpacer(ptr);

    // Add settings and HTTP server
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_CONFIGS,
        ptr, MENU_STRINGS[STRING_OPEN_CONFIGS], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_OPEN_SETTINGS,
        ptr, MENU_STRINGS[STRING_OPEN_SETTINGS], 0);
    ptr = menuInsertSetting(MENU_ACTION_SETTING, SETTING_ACTION_DISABLE,
        ptr, MENU_STRINGS[STRING_DISABLE], 0);

    // Add firmware update option, if available
    File tmpFile = SD.open(FWUPDATE_HEX_PATH, FILE_READ);
    if (tmpFile)
    {
        menuHasUpdateFw = true;
        tmpFile.close();
        ptr = menuInsertSpacer(ptr);
        ptr = menuInsertSetting(MENU_ACTION_UPDATE_FW, 0, ptr,
            MENU_STRINGS[STRING_UPDATE_FW], 0);
    } else {
        menuHasUpdateFw = false;
    }

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
    menuRenderSourceEntry = 0;
    menuRenderSkipEnabled = false;

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
        case MENU_TYPE_CONFIGS :
            textPtr = menuGenerateConfigurations(textPtr);
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
        case MENU_TYPE_BROWSER_EXPAND :
            textPtr = menuGenerateBrowserExpand(textPtr);
            break;
        case MENU_TYPE_BROWSER_OPEN :
            textPtr = menuGenerateBrowserOpen(textPtr);
            break;
        case MENU_TYPE_BROWSER_OPEN_ROM :
            textPtr = menuGenerateBrowserOpenRom(textPtr);
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
        case MENU_TYPE_POK_BROWSER :
            textPtr = menuGeneratePokBrowser(textPtr);
            break;
        case MENU_TYPE_SAVE_STATE_SLOT :
            textPtr = menuGenerateSelectStateSlot(textPtr, false);
            break;
        case MENU_TYPE_LOAD_STATE_SLOT :
            textPtr = menuGenerateSelectStateSlot(textPtr, true);
            break;
        case MENU_TYPE_PREVIEW_STATE_SLOT :
            textPtr = menuGeneratePreviewStateSlot(textPtr);
            break;
        case MENU_TYPE_IN_GAME_SETTINGS :
            textPtr = menuGenerateInGameSettings(textPtr);
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

    // Store border colour and bank selection
    menuRamArray[1][MEM_BORDER] = spectrumBorder;
    menuRamArray[1][MEM_BANK1] = spectrumBankM;

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

    // Store the variable width font width table
    address = ((menuPtr[(MENU_TEXT_WIDTHS + 1)] << 8) + menuPtr[MENU_TEXT_WIDTHS]);
    menuTextWidths = (uint8_t*)&(menuPtr[address]);

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
    uint32_t entryIndex = menu[index].index;

    // Perform large directory filename expansion handling
    if (menuCurrent == MENU_TYPE_BROWSER_EXPAND)
    {
        menuTxtPtr[MEM_POS] = menuBrowserExpandedMenuLine;
        if (index == 1)
        {
            // Handle action when expanded filename is selected
            menuBrowserExpandedIndex = BROWSER_PARENT_INDEX;
            menuBrowserExpandedAction = MENU_ACTION_TOP_MENU;
            menuCurrent = MENU_TYPE_BROWSER;
        }
    }

    // Perform menu action
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
                case SETTING_ACTION_TOGGLE_DIVMMC_LOCK_SD :
                    divMmcSdReadOnly = !divMmcSdReadOnly;
                    menuConfigChanged = true;
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
                case SETTING_ACTION_TOGGLE_GENIE128 :
                    mf128LoadGenie = !mf128LoadGenie;
                    menuConfigChanged = true;
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
                case SETTING_ACTION_TOGGLE_FDC_FDB :
                    dskEnableDriveB = !dskEnableDriveB;
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
                    printerPort.end();
                    break;
                case SETTING_ACTION_MOUNT_SD_SDA :
                    strcpy(cfgData.divMmcSdaPath, "/");
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_UNMOUNT_SDA :
                    cfgData.divMmcSdaPath[0] = 0;
                    menuConfigChanged = true;
                    break;
                case SETTING_ACTION_MOUNT_SD_SDB :
                    strcpy(cfgData.divMmcSdbPath, "/");
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
                case SETTING_ACTION_IN_GAME_TOGGLE_PRINTER :
                    // Toggle active Centronics printer
                    if (printerEnabled || lprintEnabled)
                    {
                        printerEnabled = false;
                        lprintEnabled = false;
                    } else if (lprintPresent)
                    {
                        lprintEnabled = true;
                    } else {
                        printerEnabled = true;
                    }
                    break;
                case SETTING_ACTION_IN_GAME_TOGGLE_USB :
                    // Toggle active USB
                    usbEnabled = !usbEnabled;
                    break;
                case SETTING_ACTION_OPEN_DEBUG :
                    // Open debug menu
                    menuCurrent = MENU_TYPE_DEBUG;
                    break;
                case SETTING_ACTION_OPEN_TAPE_BROWSER :
                    // Open tape browser
                    tzxPlayer.scanTape(menuDynamicList);
                    menuCurrent = MENU_TYPE_TAPE_BROWSER;
                    break;
                case SETTING_ACTION_OPEN_SAVE_STATE_SLOT :
                    // Open the state slot selector
                    menuCurrent = MENU_TYPE_SAVE_STATE_SLOT;
                    break;
                case SETTING_ACTION_OPEN_LOAD_STATE_SLOT :
                    // Open the state slot selector
                    menuCurrent = MENU_TYPE_LOAD_STATE_SLOT;
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
                    strncpy(menuBrowserPath, "/", MAX_PATH);
                    menuResetBrowserPage();
                    menuCurrent = MENU_TYPE_BROWSER;
                    break;
                case SETTING_ACTION_OPEN_CONFIGS :
                    menuCurrent = MENU_TYPE_CONFIGS;
                    break;
                case SETTING_ACTION_OPEN_SETTINGS :
                    menuCurrent = MENU_TYPE_SETTINGS;
                    break;
                case SETTING_ACTION_OPEN_IN_GAME_SETTINGS :
                    menuCurrent = MENU_TYPE_IN_GAME_SETTINGS;
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
            menuUpdateRomFileName(entryIndex);
            if (stricmp(cfgData.romName, menuBrowserPath) == 0)
            {
                return true;
            } else {
                strncpy(cfgData.romName, menuBrowserPath, MAX_PATH);
                cfgData.romName[(MAX_PATH - 1)] = 0;
                menuConfigChanged = true;
            }
            break;
        case MENU_ACTION_LOAD_CART :
            menuUpdateRomFileName(entryIndex);
            return true;
        case MENU_ACTION_LOAD_CFG :
            menuUpdateCfgFileName(entryIndex);
            if (stricmp(cfgData.cfgName, menuBrowserPath) == 0)
            {
                return true;
            } else {
                strncpy(cfgData.cfgName, menuBrowserPath, MAX_PATH);
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
            wifiNtpTz = (uint8_t)entryIndex;
            menuConfigChanged = true;
            menuCurrent = MENU_TYPE_SETTINGS;
            break;
        case MENU_ACTION_LOAD_NETMAN :
            strncpy(menuBrowserPath, NETMAN_Z80_PATH, MAX_PATH);
            return true;
        case MENU_ACTION_LOAD_RTC_SETUP :
            strncpy(menuBrowserPath, RTC_SETUP_Z80_PATH, MAX_PATH);
            return true;
        case MENU_ACTION_BROWSER_PAGE :
            if (entryIndex != 0)
            {
                if (menuBrowserHasNextLowerBound)
                {
                    menuBrowserLowerBound = menuBrowserNextLowerBound;
                    ++menuBrowserStartIndex;
                    menuBrowserHasLowerBound = true;
                    menuBrowserPreviousPage = false;
                } else {
                    menuResetBrowserPage();
                }
            } else {
                if (menuBrowserHasLowerBound)
                {
                    if (menuBrowserStartIndex > 0)
                    {
                        --menuBrowserStartIndex;
                    }
                    menuBrowserPreviousPage = true;
                } else {
                    menuResetBrowserPage();
                }
            }
            menuCurrent = MENU_TYPE_BROWSER;
            break;
        case MENU_ACTION_BROWSER_EXPAND :
            if (menuBrowserExpandedIndex != entryIndex)
            {
                if (menuSetBrowserExpandedAction(entryIndex))
                {
                    menuBrowserExpandedIndex = entryIndex;
                    menuBrowserExpandedMenuIndex = index;
                    menuBrowserExpandedMenuLine = index % MENU_PAGE_ENTRY_COUNT;
                    menuCurrent = MENU_TYPE_BROWSER_EXPAND;
                    return false;
                }
            } else {
                menuBrowserExpandedIndex = BROWSER_PARENT_INDEX;
                menuBrowserExpandedAction = MENU_ACTION_TOP_MENU;
            }
            menuCurrent = MENU_TYPE_BROWSER;
            break;
        case MENU_ACTION_BROWSER_CD :
            if (updateBrowserPath(entryIndex))
            {
                menuCurrent = MENU_TYPE_BROWSER;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN :
            if (updateBrowserPath(entryIndex))
            {
                menuCurrent = MENU_TYPE_BROWSER_OPEN;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN_ROM :
            if (updateBrowserPath(entryIndex))
            {
                menuCurrent = MENU_TYPE_BROWSER_OPEN_ROM;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN_HDF :
            if (updateBrowserPath(entryIndex))
            {
                menuCurrent = MENU_TYPE_BROWSER_MOUNT_HDF;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN_DSK :
            if (updateBrowserPath(entryIndex))
            {
                menuCurrent = MENU_TYPE_BROWSER_MOUNT_DSK;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_OPEN_POK :
            if (updateBrowserPath(entryIndex))
            {
                if (pokeOpenFile(menuBrowserPath, menuDynamicList))
                {
                    menuCurrent = MENU_TYPE_POK_BROWSER;
                } else {
                    updateBrowserPath(BROWSER_PARENT_INDEX);
                    menuCurrent = MENU_TYPE_BROWSER;
                }
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_LOAD_Z80 :
        case MENU_ACTION_BROWSER_LOAD_MDR :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                updateBrowserPath(entryIndex))
            {
                return true;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_LOAD_TZX :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                updateBrowserPath(entryIndex))
            {
                strncpy(cfgData.tapeFileName, menuBrowserPath, MAX_PATH);
                cfgData.tapeFileName[(MAX_PATH - 1)] = 0;
                if (menuTopMenu != MENU_TYPE_MAIN)
                {
                    menuCurrent = menuTopMenu;
                    return false;
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
        case MENU_ACTION_BROWSER_LOAD_MLD :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                (menuCurrent == MENU_TYPE_BROWSER_OPEN_ROM) ||
                updateBrowserPath(entryIndex))
            {
                return true;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_LOAD_MF128 :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                (menuCurrent == MENU_TYPE_BROWSER_OPEN_ROM) ||
                updateBrowserPath(entryIndex))
            {
                menuCurrent = menuTopMenu;
                return false;
            } else {
                menuCurrent = menuTopMenu;
            }
            break;
        case MENU_ACTION_BROWSER_MOUNT_SDA :
        case MENU_ACTION_BROWSER_MOUNT_SDB :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                (menuCurrent == MENU_TYPE_BROWSER_MOUNT_HDF) ||
                updateBrowserPath(entryIndex))
            {
                strncpy(((menuAction == MENU_ACTION_BROWSER_MOUNT_SDB) ?
                    cfgData.divMmcSdbPath : cfgData.divMmcSdaPath),
                    menuBrowserPath, MAX_PATH);
                divMmcPresent = true;
                menuConfigChanged = true;
            }
            menuCurrent = menuTopMenu;
            break;
        case MENU_ACTION_BROWSER_MOUNT_FDA :
        case MENU_ACTION_BROWSER_MOUNT_FDB :
            if ((menuCurrent == MENU_TYPE_BROWSER_OPEN) ||
                (menuCurrent == MENU_TYPE_BROWSER_MOUNT_DSK) ||
                updateBrowserPath(entryIndex))
            {
                strncpy(((menuAction == MENU_ACTION_BROWSER_MOUNT_FDB) ?
                    cfgData.dskFdbPath : cfgData.dskFdaPath),
                    menuBrowserPath, MAX_PATH);
                if (menuTopMenu != MENU_TYPE_MAIN)
                {
                    menuCurrent = menuTopMenu;
                    return false;
                } else {
                    dskPresent = true;
                    if (menuAction == MENU_ACTION_BROWSER_MOUNT_FDB)
                    {
                        dskEnableDriveB = true;
                    }
                    menuConfigChanged = true;
                }
            }
            menuCurrent = menuTopMenu;
            break;
        case MENU_ACTION_IN_GAME_UNMOUNT_FDA :
            cfgData.dskFdaPath[0] = 0;
            menuCurrent = menuTopMenu;
            return false;
        case MENU_ACTION_IN_GAME_UNMOUNT_FDB :
            cfgData.dskFdbPath[0] = 0;
            menuCurrent = menuTopMenu;
            return false;
        case MENU_ACTION_IN_GAME_EJECT_TAPE :
            menuCurrent = menuTopMenu;
            return false;
        case MENU_ACTION_START_SERVER :
            httpStartServer();
            break;
        case MENU_ACTION_STOP_SERVER :
            httpStopServer();
            break;
        case MENU_ACTION_IN_GAME_SEEK_TAPE :
            tzxPlayer.seek((uint8_t)entryIndex);
            menuCurrent = menuTopMenu;
            break;
        case MENU_ACTION_POK_TOGGLE_TRAINER :
            pokeToggleTrainer((uint8_t)entryIndex);
            break;
        case MENU_ACTION_SELECT_SAVE_SLOT :
            if (stateSaveSlot != entryIndex)
            {
                stateSaveSlot = entryIndex;
                menuConfigChanged = true;
            } else {
                menuAction = MENU_ACTION_IN_GAME_SAVE_STATE;
                return true;
            }
            break;
        case MENU_ACTION_SELECT_LOAD_SLOT :
            // The first selection previews the saved active screen.
            if (statePreparePreview(entryIndex))
            {
                menuPreviewSlot = entryIndex;
                menuCurrent = MENU_TYPE_PREVIEW_STATE_SLOT;
                return false;
            }
            break;
        case MENU_ACTION_LOAD_STATE_SLOT :
            // A second selection from the preview loads the slot.
            // Clear the configuration name as this changes configuration.
            cfgData.cfgName[0] = 0;
            stateActiveSlot = (int8_t)entryIndex;
            menuConfigChanged = true;
            menuSaveConfiguration();
            return true;
        case MENU_ACTION_IN_GAME_EXIT :
        case MENU_ACTION_IN_GAME_EXIT_TAPE :
        case MENU_ACTION_IN_GAME_EXIT_BASIC :
        case MENU_ACTION_IN_GAME_NMI :
        case MENU_ACTION_IN_GAME_MF128 :
        case MENU_ACTION_IN_GAME_DIVMMC :
        case MENU_ACTION_IN_GAME_RESET :
            // These actions require additional NMI or reset handling
            return true;
        case MENU_ACTION_IN_GAME_SAVE_STATE :
            stateSaveSlot = entryIndex;
            return true;
        case MENU_ACTION_IN_GAME_APPLY_POK :
            if (pokeRequestApply())
            {
                menuCurrent = menuTopMenu;
                return true;
            }
            break;
    }

    // Clear the configuration name when changed
    if (menuConfigChanged)
    {
        cfgData.cfgName[0] = 0;
    }

    // Clear the action as "No-Op", to redraw the main screen
    menuAction = MENU_ACTION_TOP_MENU;
    return false;
}

inline bool menuIsInGameMenu()
{
    return (menuTopMenu != MENU_TYPE_MAIN);
}

inline menu_action_t menuGetMenuAction()
{
    return menuAction;
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
            // Disable the ESP-01S
            pinMode(ESP_ENABLE, OUTPUT);
            digitalWriteFast(ESP_ENABLE, 0);

            // Flash the firmware update
            flashUpdate(FWUPDATE_HEX_PATH);
            break;
        case MENU_ACTION_LOAD_CART :
        case MENU_ACTION_BROWSER_LOAD_CART :
        case MENU_ACTION_BROWSER_LOAD_ZXC2 :
        case MENU_ACTION_BROWSER_LOAD_ZXC3 :
        case MENU_ACTION_BROWSER_LOAD_MLD :
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
        } else if (stricmp(fileext + 1, "mld") == 0)
        {
            return TYPE_MLD;
        }
    }
    return TYPE_CART;
}

bool menuUpdateFileName(uint32_t fileIndex, const char* dirname)
{
    menuBrowserPath[0] = 0;
    bool result = false;
    FsFile directory = SD.sdfs.open(dirname, O_RDONLY);
    if (directory)
    {
        if (directory.isDirectory())
        {
            FsFile entry;
            if (entry.open(&directory, fileIndex, O_RDONLY))
            {
                if (!entry.isDirectory() &&
                    entry.getName(menuBrowserPath, MAX_PATH))
                {
                    result = true;
                }
                entry.close();
            }
        }
        directory.close();
    }
    return result;
}

void menuUpdateRomFileName(uint32_t fileIndex)
{
    menuUpdateFileName(fileIndex, "/ROMS");
}

void menuUpdateCfgFileName(uint32_t fileIndex)
{
    menuUpdateFileName(fileIndex, "/ZXTEENSY/CONFIGS");
}

bool updateBrowserPath(uint32_t fileIndex)
{
    bool resetBrowserStartIndex = false;
    FsFile directory = SD.sdfs.open(menuBrowserPath, O_RDONLY);
    if (directory)
    {
        bool wasDirectory = directory.isDirectory();
        if (wasDirectory && (fileIndex != BROWSER_PARENT_INDEX))
        {
            FsFile entry;
            if (entry.open(&directory, fileIndex, O_RDONLY))
            {
                bool entryIsDirectory = entry.isDirectory();
                char name[MAX_PATH];
                if (entry.getName(name, MAX_PATH))
                {
                    size_t pathLen = strlen(menuBrowserPath);
                    if ((pathLen + strlen(name)) < (MAX_PATH - 2))
                    {
                        if (pathLen > 1)
                        {
                            strcat(menuBrowserPath, "/");
                        }
                        strcat(menuBrowserPath, name);
                        resetBrowserStartIndex = entryIsDirectory;
                    }
                }
                entry.close();
            }
        } else {
            char *fileext = strrchr(menuBrowserPath, '/');
            if (fileext != 0)
            {
                if (fileext != menuBrowserPath)
                {
                    // Remove last directory
                    *fileext = 0;
                    resetBrowserStartIndex = wasDirectory;
                } else if (strlen(menuBrowserPath) > 1)
                {
                    // Return to root directory
                    *(fileext + 1) = 0;
                    resetBrowserStartIndex = wasDirectory;
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
        strcpy(menuBrowserPath, "/");
        resetBrowserStartIndex = true;
    }
    if (resetBrowserStartIndex)
    {
        menuResetBrowserPage();
    }
    return true;
}

char* menuGetBrowserPath()
{
    return menuBrowserPath;
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

char* menuGetTapeFileName()
{
    return ((strlen(cfgData.tapeFileName) > 0) ? cfgData.tapeFileName : 0);
}

void menuClearTapeFileName()
{
    cfgData.tapeFileName[0] = 0;
}

File menuGetMenuRomFile(char* romName, rom_type_t* romType, bool updatePath)
{
    if (romName[0] != 0)
    {
        char romPath[MAX_PATH];
        if (snprintf(romPath, MAX_PATH, "/ROMS/%s", romName) >= (int)MAX_PATH)
        {
            romPath[(MAX_PATH - 1)] = 0;
        }
        File entry = SD.open(romPath, FILE_READ);
        if (entry)
        {
            if (!entry.isDirectory())
            {
                *romType = getRomType(romName);
                if (updatePath)
                {
                    strncpy(romName, romPath, MAX_PATH);
                }
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
    File entry = SD.open(menuBrowserPath, FILE_READ);
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
        case MENU_ACTION_BROWSER_LOAD_MLD :
            *romType = TYPE_MLD;
            return menuGetBrowserRomFile();
        case MENU_ACTION_BROWSER_LOAD_CART :
            *romType = TYPE_CART;
            return menuGetBrowserRomFile();
        case MENU_ACTION_LOAD_NETMAN :
        case MENU_ACTION_LOAD_RTC_SETUP :
        case MENU_ACTION_BROWSER_LOAD_Z80 :
            return menuGetBrowserZ80File(romType);
        case MENU_ACTION_LOAD_CART :
            return menuGetMenuRomFile(menuBrowserPath, romType, true);
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
    return menuGetMenuRomFile(cfgData.romName, &romType, false);
}

void menuInGameExitBasic()
{
    // Return to REPORT-D in 48K ROM
    menuRamArray[1][(MEM_PC + 1)] = 0x0D;
    menuRamArray[1][MEM_PC] = 0x00;

    // Ensure SP is in RAM, and not peripheral RAM
    if (menuRamArray[1][(MEM_SP2 + 1)] <= 0x3F)
    {
        menuRamArray[1][(MEM_SP2 + 1)] = 0x7F;
        menuRamArray[1][MEM_SP2] = 0x00;
    }

    // Set IY to 0x5C3A for BASIC
    uint16_t baseAddress = (RAM_PAGE_SIZE - 1) & (0x20 +
        (menuRamArray[1][(MEM_SPR + 1)] << 8) + menuRamArray[1][MEM_SPR]);
    uint16_t address = (baseAddress + Z80_REG_IY);
    menuRamArray[1][(address + 1)] = 0x5C;
    menuRamArray[1][address] = 0x3A;

    // Restore BASIC with IM1, enable interrupts, and I to 0x3F
    address = (baseAddress + Z80_REG_IF);
    menuRamArray[1][(address + 1)] = 0x3F;
    menuRamArray[1][address] |= 0x02;
    menuRamArray[1][MEM_IM2] = 0x02;

    // Set to bank in ROM1/3
    menuRamArray[1][MEM_BANK1] |= 0x10;

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
    stateActiveSlot = -1;
    divMmcPresent = false;
    divMmcExtRamPresent = true;
    divMmcSdReadOnly = true;
    divMmcRomPresent = false;
    interface1Present = false;
    mf128Present = false;
    mf128LoadGenie = false;
    uartPresent = false;
    usbPresent = false;
    gamepadButtons = false;
    wifiNtpPresent = false;
    wifiNtpTz = 48;
    dskPresent = false;
    dskEnableDriveB = false;
    modemPresent = false;
    printerPresent = false;
    lprintPresent = false;
    bootIntoMenu = true;
    menuEnableInGame = true;
    memset(&cfgData, 0, sizeof(cfgData));
    strcpy(cfgData.divMmcSdaPath, "/");
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
                    } else if (strncmp("divMmcSdReadOnly = ", cfgPtr, 19) == 0)
                    {
                        divMmcSdReadOnly = ((cfgPtr[19] == '1') ? true : false);
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
                    } else if (strncmp("dskEnableDriveB = ", cfgPtr, 18) == 0)
                    {
                        dskEnableDriveB = ((cfgPtr[18] == '1') ? true : false);
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
                    } else if (strncmp("mf128LoadGenie = ", cfgPtr, 17) == 0)
                    {
                        mf128LoadGenie = ((cfgPtr[17] == '1') ? true : false);
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
                case 's' :
                    if ((cfgCfgName == 0) &&
                        (strncmp("stateActiveSlot = ", cfgPtr, 18) == 0))
                    {
                        long slot = atol(&(cfgPtr[18]));
                        stateActiveSlot = ((slot >= 0) &&
                            (slot < STATE_SLOT_COUNT)) ? slot : -1;
                        ++count;
                    } else if (strncmp("stateSaveSlot = ", cfgPtr, 16) == 0)
                    {
                        long slot = atol(&(cfgPtr[16]));
                        stateSaveSlot = ((slot >= 0) &&
                            (slot < STATE_POKE_SLOT)) ? slot : 0;
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
            cfgData.tapeFileName[(MAX_PATH - 1)] = 0;

            // Write configuration to file
            cfgFile.truncate();
            cfgFile.printf("stateActiveSlot = %0d\n", stateActiveSlot);
            cfgFile.printf("stateSaveSlot = %0d\n", stateSaveSlot);
            cfgFile.printf("divMmcPresent = %0d\n", divMmcPresent);
            cfgFile.printf("divMmcExtRamPresent = %0d\n", divMmcExtRamPresent);
            cfgFile.printf("divMmcSdReadOnly = %0d\n", divMmcSdReadOnly);
            cfgFile.printf("divMmcRomPresent = %0d\n", divMmcRomPresent);
            cfgFile.printf("interface1Present = %0d\n", interface1Present);
            cfgFile.printf("mf128Present = %0d\n", mf128Present);
            cfgFile.printf("mf128LoadGenie = %0d\n", mf128LoadGenie);
            cfgFile.printf("uartPresent = %0d\n", uartPresent);
            cfgFile.printf("usbPresent = %0d\n", usbPresent);
            cfgFile.printf("gamepadButtons = %0d\n", gamepadButtons);
            cfgFile.printf("wifiNtpPresent = %0d\n", wifiNtpPresent);
            cfgFile.printf("wifiNtpTz = %0d\n", wifiNtpTz);
            cfgFile.printf("dskPresent = %0d\n", dskPresent);
            cfgFile.printf("dskEnableDriveB = %0d\n", dskEnableDriveB);
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

bool menuVaPrintDebug(bool clearDebug, const char *fmt, va_list ap)
{
    if (clearDebug)
    {
        menuClearDebug();
    }
    if (menuDebugIndex < MENU_DEBUG_SIZE)
    {
        char* ptr = &(menuDebugBuffer[menuDebugIndex]);
        size_t count = vsnprintf(ptr, (MENU_DEBUG_SIZE - menuDebugIndex), fmt, ap);
        if ((menuDebugIndex + count + 1) >= MENU_DEBUG_SIZE)
        {
            menuDebugBuffer[(MENU_DEBUG_SIZE - 1)] = 0;
            menuDebugIndex = MENU_DEBUG_SIZE;
        } else {
            menuDebugIndex += (strlen(ptr) + 1);
        }
    }
    menuHasDebug = true;
    return (menuCurrent == MENU_TYPE_DEBUG);
}

bool menuPrintDebug(bool clearDebug, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    bool result_ = menuVaPrintDebug(clearDebug, fmt, ap);
    va_end(ap);
    return result_;
}
