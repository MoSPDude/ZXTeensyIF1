
#include <SD.h>

static const uint8_t POKE_MAX_TRAINERS = 253;
static const uint16_t POKE_MAX_RECORDS = 256;
static const size_t POKE_SELECTION_SIZE = ((POKE_MAX_TRAINERS + 7) / 8);

typedef struct {
    uint16_t address;
    uint8_t value;
    uint8_t trainer;
    uint8_t bank;
    uint8_t sourceBank;
    uint8_t selected;
} poke_record_t;

DMAMEM poke_record_t pokeRecords[POKE_MAX_RECORDS];
uint8_t pokeTrainerSelected[POKE_SELECTION_SIZE];
uint8_t pokeTrainerUnsupported[POKE_SELECTION_SIZE];
uint8_t pokeTrainerCount = 0;
uint16_t pokeRecordCount = 0;
uint8_t pokeSelectedBankMask = 0;
bool pokeApplyRequested = false;

bool pokeGetBit(const uint8_t* bits, uint8_t index)
{
    return ((bits[index >> 3] >> (index & 0x07)) & 0x01) != 0;
}

void pokeToggleBit(uint8_t* bits, uint8_t index)
{
    bits[index >> 3] ^= (1 << (index & 0x07));
}

void pokeSetBit(uint8_t* bits, uint8_t index)
{
    bits[index >> 3] |= (1 << (index & 0x07));
}

bool pokeReadLine(File& file, char* line)
{
    size_t length = file.readBytesUntil('\n', line, (MAX_PATH - 1));
    if (length > 0)
    {
        line[length] = 0;
        return true;
    }
    return false;
}

void pokeTrimRight(char* value)
{
    size_t length = strlen(value);
    while ((length > 0) &&
        ((value[length - 1] == ' ') || (value[length - 1] == '\t') ||
            (value[length - 1] == '\r')))
    {
        value[--length] = 0;
    }
}

bool pokeParseNumber(char** text, uint32_t maximum, uint32_t* result)
{
    char* ptr = *text;
    while ((*ptr == ' ') || (*ptr == '\t'))
    {
        ++ptr;
    }
    if ((*ptr < '0') || (*ptr > '9'))
    {
        return false;
    }

    char* end;
    unsigned long value = strtoul(ptr, &end, 10);
    if ((end == ptr) || (value > maximum))
    {
        return false;
    }
    *text = end;
    *result = value;
    return true;
}

bool pokeParseRecord(char* line, uint8_t* bank, uint16_t* address,
    uint16_t* value, uint8_t* original)
{
    char* ptr = line + 1;
    uint32_t parsedBank;
    uint32_t parsedAddress;
    uint32_t parsedValue;
    uint32_t parsedOriginal;
    if (!pokeParseNumber(&ptr, 8, &parsedBank) ||
        !pokeParseNumber(&ptr, 65535, &parsedAddress) ||
        !pokeParseNumber(&ptr, 256, &parsedValue) ||
        !pokeParseNumber(&ptr, 255, &parsedOriginal))
    {
        return false;
    }
    while ((*ptr == ' ') || (*ptr == '\t'))
    {
        ++ptr;
    }
    if ((*ptr != 0) || ((parsedBank == 8) && (parsedAddress < 0x4000)))
    {
        return false;
    }

    *bank = parsedBank;
    *address = parsedAddress;
    *value = parsedValue;
    *original = parsedOriginal;
    return true;
}

bool pokeParseFile(const char* pokeFilePath, char pokeTrainerNames[][MENU_STR_LEN])
{
    File file = SD.open(pokeFilePath, FILE_READ);
    if (!file)
    {
        menuPrintDebug(false, F_CSTR("Failed to open POK '%s'"), pokeFilePath);
        return false;
    }

    bool success = false;
    bool trainerOpen = false;
    bool foundEnd = false;
    bool errorReported = false;
    int16_t trainer = -1;
    uint16_t recordCount = 0;
    uint32_t lineNumber = 0;
    char line[MAX_PATH];
    while (true)
    {
        // Load the next line to end of file
        bool lineResult = pokeReadLine(file, line);
        if (lineResult)
        {
            ++lineNumber;
            pokeTrimRight(line);
            if (line[0] == 0)
            {
                continue;
            }
        } else {
            break;
        }

        // Parse the current line
        if (line[0] == 'N')
        {
            if (trainerOpen || ((trainer + 1) >= POKE_MAX_TRAINERS))
            {
                menuPrintDebug(false, F_CSTR("Invalid POK trainer at line %d"),
                    lineNumber);
                errorReported = true;
                break;
            }
            char* name = line + 1;
            while ((*name == ' ') || (*name == '\t'))
            {
                ++name;
            }
            if (*name == 0)
            {
                menuPrintDebug(false, F_CSTR("Invalid POK name at line %d"),
                    lineNumber);
                errorReported = true;
                break;
            }
            ++trainer;
            if (snprintf(pokeTrainerNames[trainer], MENU_STR_LEN, "%s", name) >=
                MENU_STR_LEN)
            {
                pokeTrainerNames[trainer][(MENU_STR_LEN - 2)] = '>';
                pokeTrainerNames[trainer][(MENU_STR_LEN - 1)] = 0;
            }
            trainerOpen = true;
        } else if ((line[0] == 'M') || (line[0] == 'Z'))
        {
            uint8_t bank;
            uint16_t address;
            uint16_t value;
            uint8_t original;
            if (!trainerOpen ||
                !pokeParseRecord(line, &bank, &address, &value, &original))
            {
                menuPrintDebug(false, F_CSTR("Invalid POK record at line %d"),
                    lineNumber);
                errorReported = true;
                break;
            }
            if (recordCount >= POKE_MAX_RECORDS)
            {
                menuPrintDebug(false, F_CSTR("Too many POK records '%s'"),
                    pokeFilePath);
                errorReported = true;
                break;
            }

            poke_record_t* record = &(pokeRecords[recordCount++]);
            record->selected = 0x00;
            record->trainer = trainer;
            record->bank = bank;
            record->sourceBank = bank;
            record->address = address;
            if (value > 0xFF)
            {
                pokeSetBit(pokeTrainerUnsupported, trainer);
                record->value = 0xFF;
            } else {
                record->value = value;
            }
            if (line[0] == 'Z')
            {
                trainerOpen = false;
            }
        } else if (line[0] == 'Y')
        {
            char* ptr = line + 1;
            while ((*ptr == ' ') || (*ptr == '\t'))
            {
                ++ptr;
            }
            if (trainerOpen || (trainer < 0) || (*ptr != 0))
            {
                menuPrintDebug(false, F_CSTR("Invalid POK end at line %d"),
                    lineNumber);
                errorReported = true;
                break;
            }
            foundEnd = true;
            success = true;
            break;
        } else {
            menuPrintDebug(false, F_CSTR("Unknown POK line %d"), lineNumber);
            errorReported = true;
            break;
        }
    }
    file.close();

    if (!foundEnd && !errorReported)
    {
        menuPrintDebug(false, F_CSTR("POK missing end marker '%s'"),
            pokeFilePath);
    }
    if (success)
    {
        pokeTrainerCount = trainer + 1;
        pokeRecordCount = recordCount;
    } else {
        pokeTrainerCount = 0;
        pokeRecordCount = 0;
    }
    return success;
}

bool pokeOpenFile(const char* path, char pokeTrainerNames[][MENU_STR_LEN])
{
    pokeTrainerCount = 0;
    pokeRecordCount = 0;
    pokeSelectedBankMask = 0;
    pokeApplyRequested = false;
    memset(pokeTrainerSelected, 0, sizeof(pokeTrainerSelected));
    memset(pokeTrainerUnsupported, 0, sizeof(pokeTrainerUnsupported));
    return pokeParseFile(path, pokeTrainerNames);
}

uint8_t pokeGetTrainerCount()
{
    return pokeTrainerCount;
}

bool pokeIsTrainerSelected(uint8_t trainer)
{
    return (trainer < pokeTrainerCount) &&
        pokeGetBit(pokeTrainerSelected, trainer);
}

bool pokeHasSelectedTrainers()
{
    for (uint8_t index = 0; index < sizeof(pokeTrainerSelected); ++index)
    {
        if (pokeTrainerSelected[index] != 0)
        {
            return true;
        }
    }
    return false;
}

bool pokeToggleTrainer(uint8_t trainer)
{
    if ((trainer < pokeTrainerCount) &&
        !pokeGetBit(pokeTrainerUnsupported, trainer))
    {
        pokeToggleBit(pokeTrainerSelected, trainer);
        return true;
    }
    return false;
}

bool pokeRequestApply()
{
    if (pokeHasSelectedTrainers())
    {
        pokeApplyRequested = true;
        return true;
    }
    return false;
}

bool pokeIsApplyRequested()
{
    return pokeApplyRequested;
}

inline int8_t pokeGetTargetBank(uint8_t bank, uint16_t address, bool state128,
    uint8_t currentBank)
{
    if (bank >= 8)
    {
        if (address < 0x8000)
        {
            return 5;
        }
        if (address < 0xC000)
        {
            return 2;
        }
        return state128 ? (currentBank & 0x07) : 0;
    }
    return state128 ? bank : -1;
}

bool pokePrepareApply(bool state128, uint8_t currentBank)
{
    uint8_t bankMask = 0;
    for (uint16_t index = 0; index < pokeRecordCount; ++index)
    {
        poke_record_t* record = &(pokeRecords[index]);
        if (pokeIsTrainerSelected(record->trainer))
        {
            int8_t bank = pokeGetTargetBank(record->sourceBank, record->address,
                state128, currentBank);
            if (bank >= 0)
            {
                record->selected = 0xFF;
                record->bank = bank;
                bankMask |= (1 << bank);
            } else {
                menuPrintDebug(false, F_CSTR("POK bank %d invalid for 48K"),
                    record->sourceBank);
                return false;
            }
        }
    }
    pokeSelectedBankMask = bankMask;
    return true;
}

void pokeApplyBank(uint8_t bank, volatile uint8_t* data)
{
    if ((pokeSelectedBankMask & (1 << bank)) != 0)
    {
        for (uint16_t index = 0; index < pokeRecordCount; ++index)
        {
            poke_record_t* record = &(pokeRecords[index]);
            if ((record->selected != 0x00) && (record->bank == bank))
            {
                data[record->address & (ROM_PAGE_SIZE - 1)] = record->value;
                record->selected = 0x00;
            }
        }
    }
}

void pokeFinishApply()
{
    pokeApplyRequested = false;
    pokeSelectedBankMask = 0;
}
