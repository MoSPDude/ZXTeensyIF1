
#include <cstdio>
#include "TzxPlayerZXTeensy.h"

extern bool menuPrintDebug(bool clearDebug, const char *fmt, ...);

void TzxPlayerZXTeensy::sendStopCommand()
{
    uint8_t command[8];
    command[0] = BLOCK_STOP;
    command[1] = 0;
    command[2] = 0;
    command[3] = 0;
    command[4] = 0;
    command[5] = 0;
    command[6] = 0;
    command[7] = 0;
    dataBuffer.writeBlock(command, 8);
}

void TzxPlayerZXTeensy::sendPulseCommand(uint16_t length)
{
    uint8_t command[8];
    command[0] = BLOCK_SYNC;
    command[1] = length & 0xFF;
    command[2] = (length & 0xFF00) >> 8;
    command[3] = command[1];
    command[4] = command[2];
    command[5] = 0x01;
    command[6] = 0x00;
    command[7] = 0x01;
    dataBuffer.writeBlock(command, 8);
}

void TzxPlayerZXTeensy::sendPulseSeqCommand(uint8_t numPulses, uint16_t firstLength)
{
    uint8_t command[10];
    command[0] = BLOCK_PULSES;
    command[1] = 0;
    command[2] = 0;
    command[3] = 0;
    command[4] = 0;
    command[5] = numPulses;
    command[6] = 0;
    command[7] = 0;
    command[8] = firstLength & 0xFF;
    command[9] = (firstLength & 0xFF00) >> 8;
    dataBuffer.writeBlock(command, 10);
}

void TzxPlayerZXTeensy::sendPauseCommand(uint16_t durationMs)
{
    // Pulse width is 1ms
    uint8_t command[8];
    uint16_t numBytes = durationMs / 8;
    uint16_t numFinalBits = durationMs % 8;
    if (numFinalBits > 0)
    {
        ++numBytes;
    } else {
        numFinalBits = 8;
    }
    command[0] = BLOCK_PAUSE;
    command[1] = 0xAC;
    command[2] = 0x0D;
    command[3] = command[1];
    command[4] = command[2];
    command[5] = numBytes & 0xFF;
    command[6] = (numBytes & 0xFF00) >> 8;
    command[7] = numFinalBits;
    dataBuffer.writeBlock(command, 8);
}

void TzxPlayerZXTeensy::sendPilotCommand(uint16_t numPulses, uint16_t pulseLength)
{
    uint8_t command[8];
    uint16_t numBytes = numPulses / 8;
    uint16_t numFinalBits = numPulses % 8;
    if (numFinalBits > 0)
    {
        ++numBytes;
    } else {
        numFinalBits = 8;
    }
    command[0] = BLOCK_PILOT;
    command[1] = pulseLength & 0xFF;
    command[2] = (pulseLength & 0xFF00) >> 8;
    command[3] = command[1];
    command[4] = command[2];
    command[5] = numBytes & 0xFF;
    command[6] = (numBytes & 0xFF00) >> 8;
    command[7] = numFinalBits;
    dataBuffer.writeBlock(command, 8);
}

void TzxPlayerZXTeensy::sendSyncCommand(uint16_t firstLength, uint16_t secondLength)
{
    uint8_t command[8];
    command[0] = BLOCK_SYNC;
    command[1] = firstLength & 0xFF;
    command[2] = (firstLength & 0xFF00) >> 8;
    command[3] = secondLength & 0xFF;
    command[4] = (secondLength & 0xFF00) >> 8;
    command[5] = 0x01;
    command[6] = 0x00;
    command[7] = 0x02;
    dataBuffer.writeBlock(command, 8);
}

void TzxPlayerZXTeensy::sendDataCommand(uint16_t zeroLength, uint16_t oneLength,
    uint16_t numBytes, uint8_t numFinalBits, uint8_t firstByte)
{
    uint8_t command[9];
    command[0] = BLOCK_DATA;
    command[1] = zeroLength & 0xFF;
    command[2] = (zeroLength & 0xFF00) >> 8;
    command[3] = oneLength & 0xFF;
    command[4] = (oneLength & 0xFF00) >> 8;
    command[5] = numBytes & 0xFF;
    command[6] = (numBytes & 0xFF00) >> 8;
    command[7] = numFinalBits;
    command[8] = firstByte;
    dataBuffer.writeBlock(command, 9);
}

void TzxPlayerZXTeensy::sendSamplesCommand(uint16_t pulseLength,
    uint16_t numBytes, uint8_t numFinalBits, uint8_t firstByte)
{
    uint8_t command[9];
    command[0] = BLOCK_SAMPLES;
    command[1] = pulseLength & 0xFF;
    command[2] = (pulseLength & 0xFF00) >> 8;
    command[3] = command[1];
    command[4] = command[2];
    command[5] = numBytes & 0xFF;
    command[6] = (numBytes & 0xFF00) >> 8;
    command[7] = numFinalBits;
    command[8] = firstByte;
    dataBuffer.writeBlock(command, 9);
}

void TzxPlayerZXTeensy::insertStandardSpeedBlock(uint16_t numBytes, uint8_t flag)
{
    sendPilotCommand(((flag & 0x80) ? 3223 : 8063), 2168);
    sendSyncCommand(667, 735);
    sendDataCommand(855, 1710, numBytes, 8, flag);
}

void TzxPlayerZXTeensy::insertPauseBlock(uint16_t durationMs)
{
    if (durationMs > 0)
    {
        sendPulseCommand(0xDAC);
        if (durationMs > 1)
        {
            sendPauseCommand(durationMs - 1);
        }
    }
}

bool TzxPlayerZXTeensy::runTape()
{
    uint32_t elapsed = (ARM_DWT_CYCCNT - edgeCycleCount);
    while (elapsed >= pulseDuration)
    {
        edgeCycleCount += pulseDuration;
        if (currentBlock <= BLOCK_STOP)
        {
            currentLevel = !currentLevel;
        }
        if (doublePulse)
        {
            doublePulse = false;
        } else if (pulseShiftCount < 7)
        {
            pulseData <<= 1;
            ++pulseShiftCount;
            doublePulse = (currentBlock == BLOCK_DATA);
        } else if (numBytes > 0)
        {
            if (!runTapeNextByte())
            {
                return false;
            }
        } else if (!runTapeNextBlock())
        {
            return false;
        }
        elapsed -= pulseDuration;
        if (currentBlock != BLOCK_SAMPLES)
        {
            uint32_t newDuration = (((pulseData & 0x80) != 0) ?
                oneDuration : zeroDuration) * TAPE_DELAY_CNT;
            pulseDuration = newDuration;
        } else {
            currentLevel = ((pulseData & 0x80) != 0);
            pulseDuration = zeroDuration;
        }
    }
    return true;
}

bool TzxPlayerZXTeensy::startTape()
{
    if (runTapeNextBlock())
    {
        uint32_t newDuration = (((pulseData & 0x80) != 0) ?
            oneDuration : zeroDuration) * TAPE_DELAY_CNT;
        pulseDuration = newDuration;
    } else {
        return false;
    }
    return true;
}

void TzxPlayerZXTeensy::bufferTape()
{
    if ((dataBlockSize > 0) &&
        (dataBuffer.getFree() > (TAPE_BUFFER_SIZE + COMMAND_SIZE)))
    {
        uint16_t count = truncateTapeLength(
            (dataBlockSize > TAPE_BUFFER_SIZE) ?
                TAPE_BUFFER_SIZE : dataBlockSize);
        if (count > 0)
        {
            insertTapeData(count);
            dataBlockSize -= count;
        } else {
            dataBlockSize = 0;
        }
        if (dataBlockSize == 0)
        {
            insertPauseBlock(pauseAfterBlock);
            pauseAfterBlock = 0;
        }
    }
    if ((dataBlockSize == 0) &&
        (dataBuffer.getFree() > (TAPE_BUFFER_SIZE + COMMAND_SIZE)))
    {
        if (!tapeBufferStarted)
        {
            insertPauseBlock(1000);
            tapeBufferStarted = true;
        }

        if (loadFromTape())
        {
            // Start the tape running, if not already
            if (tapeBufferAutoPlay)
            {
                tapeBufferAutoPlay = false;
                if (!isPlaying && startTape())
                {
                    edgeCycleCount = ARM_DWT_CYCCNT;
                    isPlaying = true;
                }
            }
        } else {
            tapeBufferEnded = true;
            isBuffering = false;
        }
    }
}

bool TzxPlayerZXTeensy::loadStandardSpeedBlock()
{
    dataBlockSize = readTapeWord();
    size_t count = truncateTapeLength(
        (dataBlockSize > TAPE_BUFFER_SIZE) ?
            TAPE_BUFFER_SIZE : dataBlockSize);
    if (count > 0)
    {
        insertStandardSpeedBlock(dataBlockSize, readTapeByte());
        insertTapeData(count - 1);
        dataBlockSize -= count;
        if (dataBlockSize == 0)
        {
            insertPauseBlock(pauseAfterBlock);
            pauseAfterBlock = 0;
        }
        return true;
    }
    return false;
}

bool TzxPlayerZXTeensy::loadTurboSpeedBlock()
{
    uint16_t pilotLength = readTapeWord();
    uint16_t firstLength = readTapeWord();
    uint16_t secondLength = readTapeWord();
    uint16_t zeroLength = readTapeWord();
    uint16_t oneLength = readTapeWord();
    uint16_t pilotNumPulses = readTapeWord();
    uint8_t numFinalBits = readTapeByte();
    pauseAfterBlock = readTapeWord();
    dataBlockSize = readTapeWord();
    dataBlockSize |= (readTapeByte() << 16);
    size_t count = truncateTapeLength(
        (dataBlockSize > TAPE_BUFFER_SIZE) ?
            TAPE_BUFFER_SIZE : dataBlockSize);
    if (count > 0)
    {
        sendPilotCommand(pilotNumPulses, pilotLength);
        sendSyncCommand(firstLength, secondLength);
        sendDataCommand(zeroLength, oneLength, dataBlockSize, numFinalBits, readTapeByte());
        insertTapeData(count - 1);
        dataBlockSize -= count;
        if (dataBlockSize == 0)
        {
            insertPauseBlock(pauseAfterBlock);
            pauseAfterBlock = 0;
        }
        return true;
    }
    return false;
}

bool TzxPlayerZXTeensy::loadPureDataBlock()
{
    uint16_t zeroLength = readTapeWord();
    uint16_t oneLength = readTapeWord();
    uint8_t numFinalBits = readTapeByte();
    pauseAfterBlock = readTapeWord();
    dataBlockSize = readTapeWord();
    dataBlockSize |= (readTapeByte() << 16);
    size_t count = truncateTapeLength(
        (dataBlockSize > TAPE_BUFFER_SIZE) ?
            TAPE_BUFFER_SIZE : dataBlockSize);
    if (count > 0)
    {
        sendDataCommand(zeroLength, oneLength, dataBlockSize, numFinalBits, readTapeByte());
        insertTapeData(count - 1);
        dataBlockSize -= count;
        if (dataBlockSize == 0)
        {
            insertPauseBlock(pauseAfterBlock);
            pauseAfterBlock = 0;
        }
        return true;
    }
    return false;
}

bool TzxPlayerZXTeensy::loadPulseSequenceBlock()
{
    uint8_t length = readTapeByte();
    dataBlockSize = (length * 2);
    size_t count = truncateTapeLength(
        (dataBlockSize > TAPE_BUFFER_SIZE) ?
            TAPE_BUFFER_SIZE : dataBlockSize);
    if (count > 1)
    {
        sendPulseSeqCommand(length, readTapeWord());
        insertTapeData(count - 2);
        dataBlockSize -= count;
        return true;
    }
    return false;
}

bool TzxPlayerZXTeensy::loadDirectRecordingBlock()
{
    uint16_t pulseLength = readTapeWord();
    pauseAfterBlock = readTapeWord();
    uint8_t numFinalBits = readTapeByte();
    dataBlockSize = readTapeWord();
    dataBlockSize |= (readTapeByte() << 16);
    size_t count = truncateTapeLength(
        (dataBlockSize > TAPE_BUFFER_SIZE) ?
            TAPE_BUFFER_SIZE : dataBlockSize);
    if (count > 0)
    {
        sendSamplesCommand(pulseLength, dataBlockSize, numFinalBits, readTapeByte());
        insertTapeData(count - 1);
        dataBlockSize -= count;
        if (dataBlockSize == 0)
        {
            insertPauseBlock(pauseAfterBlock);
            pauseAfterBlock = 0;
        }
        return true;
    }
    return false;
}

bool TzxPlayerZXTeensy::loadFromTape()
{
    if (isTzxTapeFile)
    {
        if (tapePosition < tapeLength)
        {
            uint8_t blockId = readTapeByte();
            switch (blockId)
            {
                case 0x10 :
                    // Load Standard Speed Block
                    if (hasTapeLength(4))
                    {
                        pauseAfterBlock = readTapeWord();
                        return loadStandardSpeedBlock();
                    }
                    break;
                case 0x11 :
                    // Load Turbo Speed Block
                    if (hasTapeLength(0x12))
                    {
                        return loadTurboSpeedBlock();
                    }
                    break;
                case 0x12 :
                    // Load Pure Tone Block
                    if (hasTapeLength(4))
                    {
                        uint16_t pulseLength = readTapeWord();
                        sendPilotCommand(readTapeWord(), pulseLength);
                        return true;
                    }
                    break;
                case 0x13 :
                    // Load Pulse Sequence
                    if (hasTapeLength(1))
                    {
                        return loadPulseSequenceBlock();
                    }
                case 0x14 :
                    // Load Pure Data Block
                    if (hasTapeLength(0x0A))
                    {
                        return loadPureDataBlock();
                    }
                    break;
                case 0x15 :
                    // Load Direct Recording Block
                    if (hasTapeLength(8))
                    {
                        return loadDirectRecordingBlock();
                    }
                    break;
                case 0x18 :
                case 0x19 :
                    // CSW Recording, Generalized Data
                    if (hasTapeLength(4))
                    {
                        uint32_t length = readTapeWord();
                        length = truncateTapeLength(length | (readTapeWord() << 16));
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x20 :
                    // Load Insert Pause Block
                    if (hasTapeLength(2))
                    {
                        uint16_t durationMs = readTapeWord();
                        if (durationMs > 0)
                        {
                            insertPauseBlock(durationMs);
                        } else {
                            sendStopCommand();
                        }
                        return true;
                    }
                    break;
                case 0x21 :
                    // Group Start
                    if (hasTapeLength(1))
                    {
                        uint8_t length = truncateTapeLength(readTapeByte());
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x22 :
                    // Group End
                    return loadFromTape();
                case 0x23 :
                    // Jump To Block
                    if (hasTapeLength(2))
                    {
                        // Jump relative to the next block
                        int16_t target = (int16_t)readTapeWord();
                        jumpRelative(target - 1);
                        return loadFromTape();
                    }
                    break;
                case 0x24 :
                    // Loop Start
                    if (hasTapeLength(2) && (tapeStackCount < TAPE_STACK_SIZE))
                    {
                        tapeStack[tapeStackCount].argument = readTapeWord();
                        tapeStack[tapeStackCount].position = tapePosition;
                        ++tapeStackCount;
                        return loadFromTape();
                    }
                    break;
                case 0x25 :
                    // Loop End
                    if (tapeStackCount > 0)
                    {
                        if (tapeStack[(tapeStackCount - 1)].argument > 0)
                        {
                            tapePosition = tapeStack[(tapeStackCount - 1)].position;
                            --tapeStack[(tapeStackCount - 1)].argument;
                        } else {
                            --tapeStackCount;
                        }
                    }
                    return loadFromTape();
                case 0x26 :
                    // Call Sequence
                    if (hasTapeLength(2) && (tapeStackCount < TAPE_STACK_SIZE))
                    {
                        uint32_t length = readTapeWord();
                        if (hasTapeLength(2 * length))
                        {
                            if (length > 0)
                            {
                                tapeStack[tapeStackCount].argument = 1;
                                tapeStack[tapeStackCount].position = tapePosition - 3;
                                ++tapeStackCount;
                                int16_t target = (int16_t)readTapeWord();
                                ignoreTapeData(2 * (length - 1));

                                // Jump relative to the block AFTER the
                                // Call Sequence
                                jumpRelative(target - 1);
                            }
                            return loadFromTape();
                        }
                    }
                    break;
                case 0x27 :
                    // Return From Sequence
                    if (tapeStackCount > 0)
                    {
                        tapePosition = tapeStack[(tapeStackCount - 1)].position;
                        if (readTapeByte() == 0x26)
                        {
                            uint32_t length = readTapeWord();
                            if (hasTapeLength(2 * length))
                            {
                                size_t retPosition = tapePosition + (2 * length);
                                uint16_t call = tapeStack[(tapeStackCount - 1)].argument;
                                if (call < length)
                                {
                                    ignoreTapeData(2 * call);
                                    int16_t target = (int16_t)readTapeWord();
                                    ++tapeStack[(tapeStackCount - 1)].argument;

                                    // Jump relative to the block AFTER the
                                    // Call Sequence
                                    tapePosition = retPosition;
                                    jumpRelative(target - 1);
                                } else {
                                    tapePosition = retPosition;
                                    --tapeStackCount;
                                }
                                return loadFromTape();
                            }
                        }
                    }
                    break;
                case 0x28 :
                    // Select
                    if (hasTapeLength(2))
                    {
                        uint16_t length = readTapeWord();
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x2A :
                    // Load Stop Tape if 48K Block
                    if (hasTapeLength(4))
                    {
                        ignoreTapeData(4);
                        return loadFromTape();
                    }
                    break;
                case 0x2B :
                    // Set Signal Level
                    if (hasTapeLength(5))
                    {
                        ignoreTapeData(5);
                        return loadFromTape();
                    }
                    break;
                case 0x30 :
                    // Load Text Description
                    if (hasTapeLength(1))
                    {
                        uint8_t length = truncateTapeLength(readTapeByte());
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x31 :
                    // Load Message
                    if (hasTapeLength(2))
                    {
                        ignoreTapeData(1);
                        uint8_t length = truncateTapeLength(readTapeByte());
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x32 :
                    // Archive Info
                    if (hasTapeLength(2))
                    {
                        uint16_t length = truncateTapeLength(readTapeWord());
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x33 :
                    // Hardware Type
                    if (hasTapeLength(1))
                    {
                        uint16_t length = readTapeByte();
                        length = truncateTapeLength(3 * length);
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x35 :
                    // Custom Info
                    if (hasTapeLength(14))
                    {
                        ignoreTapeData(10);
                        uint32_t length = readTapeWord();
                        length = truncateTapeLength(length | (readTapeWord() << 16));
                        ignoreTapeData(length);
                    }
                    break;
                case 0x5A :
                    // Glue Block
                    if (hasTapeLength(8))
                    {
                        ignoreTapeData(8);
                        return loadFromTape();
                    }
                    break;
                default :
                    menuPrintDebug(false, F_CSTR("tzxPlayer %d unknown ID %d"),
                        (tapePosition - 1), blockId);
                    tapePosition = tapeLength;
                    break;
            }
        }
    } else if (hasTapeLength(2))
    {
        pauseAfterBlock = 1000;
        return loadStandardSpeedBlock();
    }
    tapePosition = tapeLength;
    return false;
}

uint32_t TzxPlayerZXTeensy::doScanTape(bool seekBlock, bool seekRelative,
    uint32_t blockNum, char (*tapeMarkNames)[MENU_STR_LEN])
{
    uint8_t index = 0;
    uint32_t blockCount = 0;
    size_t currentTapePosition = tapePosition;
    if (!seekRelative)
    {
        tapePosition = (isTzxTapeFile ? 0x0A : 0x00);
    }
    while ((tapePosition < tapeLength) && (index < 255))
    {
        uint8_t blockId;
        size_t blockPosition = tapePosition;
        if (isTzxTapeFile)
        {
            blockId = readTapeByte();
        } else {
            blockId = 0x10;
        }
        switch (blockId)
        {
            case 0x10 :
                // Standard Speed Block
                if (hasTapeLength(4))
                {
                    if (isTzxTapeFile)
                    {
                        ignoreTapeData(2);
                    }
                    uint16_t count = truncateTapeLength(readTapeWord());
                    size_t nextPosition = tapePosition + count;
                    if (count > 0)
                    {
                        char tmpName[MAX_PATH];
                        char blockName[MAX_PATH];
                        uint8_t flag = readTapeByte();
                        if (flag & 0x80)
                        {
                            // Subtract 2 for flag and checksum bytes
                            snprintf(blockName, MAX_PATH, "Standard Data %db",
                                ((count > 2) ? (count - 2) : 0));
                        } else if (hasTapeLength(0x12))
                        {
                            // Decode tape header
                            flag = readTapeByte();
                            switch (flag)
                            {
                                case 0 :
                                    strcpy(blockName, "Program: ");
                                    break;
                                case 1 :
                                    strcpy(blockName, "Num. array: ");
                                    break;
                                case 2 :
                                    strcpy(blockName, "Chr. array: ");
                                    break;
                                default :
                                    strcpy(blockName, "Bytes: ");
                                    break;
                            }
                            memcpy(tmpName, (void *)&(tapeBuffer[tapePosition]), 10);
                            tmpName[10] = 0;
                            int i = 9;
                            while (i >= 0)
                            {
                                if (tmpName[i] == ' ')
                                {
                                    tmpName[i] = 0;
                                    --i;
                                } else {
                                    break;
                                }
                            }
                            while (i >= 0)
                            {
                                if ((tmpName[i] < ' ') || (tmpName[i] >= 128))
                                {
                                    tmpName[i] = '?';
                                }
                                --i;
                            }
                            strcat(blockName, tmpName);
                            ignoreTapeData(10);
                            uint16_t length = readTapeWord();
                            uint16_t param1 = readTapeWord();
                            ignoreTapeData(2);
                            switch (flag)
                            {
                                case 0 :
                                    if (param1 < 0x8000)
                                    {
                                        snprintf(tmpName, MAX_PATH, " LINE %0d", param1);
                                        strcat(blockName, tmpName);
                                    }
                                    break;
                                case 1 :
                                case 2 :
                                    strcpy(tmpName, ((flag == 2) ? " DATA ?$()" : " DATA ?()"));
                                    tmpName[6] = 0x60 + (param1 & 0x1F);
                                    strcat(blockName, tmpName);
                                    break;
                                case 3 :
                                    if ((param1 == 0x4000) && (length == 6912))
                                    {
                                        strcat(blockName, " SCREEN$");
                                    } else {
                                        snprintf(tmpName, MAX_PATH, " CODE %d,%d", param1, length);
                                        strcat(blockName, tmpName);
                                    }
                                    break;
                                default :
                                    break;
                            }
                        } else {
                            strcpy(blockName, "Header");
                        }

                        // Store block
                        if (tapeMarkNames != 0)
                        {
                            tapeMarkPosition[index] = blockPosition;
                            strncpy(tapeMarkNames[index], blockName, MENU_STR_LEN);
                            tapeMarkNames[index][(MENU_STR_LEN - 1)] = 0;
                            ++index;
                        }
                    }
                    tapePosition = nextPosition;
                }
                break;
            case 0x11 :
                // Turbo Speed Block
                if (hasTapeLength(0x12))
                {
                    ignoreTapeData(0x0F);
                    uint32_t length = readTapeWord();
                    length |= (readTapeByte() << 16);
                    ignoreTapeData(length);

                    // Store block
                    if (tapeMarkNames != 0)
                    {
                        if (snprintf(tapeMarkNames[index], MENU_STR_LEN,
                            "Turbo Data %db", (int)length) >= MENU_STR_LEN)
                        {
                            tapeMarkNames[index][(MENU_STR_LEN - 1)] = 0;
                        }
                        tapeMarkPosition[index] = blockPosition;
                        ++index;
                    }
                }
                break;
            case 0x12 :
                // Pure Tone Block
                ignoreTapeData(4);
                break;
            case 0x13 :
                // Pulse Sequence
                if (hasTapeLength(1))
                {
                    uint32_t length = readTapeByte() * 2;
                    ignoreTapeData(length);
                }
                break;
            case 0x14 :
                // Pure Data Block
                if (hasTapeLength(0x0A))
                {
                    ignoreTapeData(7);
                    uint32_t length = readTapeWord();
                    length |= (readTapeByte() << 16);
                    ignoreTapeData(length);
                }
                break;
            case 0x15 :
                // Direct Recording Block
                if (hasTapeLength(8))
                {
                    ignoreTapeData(5);
                    uint32_t length = readTapeWord();
                    length |= (readTapeByte() << 16);
                    ignoreTapeData(length);
                }
                break;
            case 0x18 :
            case 0x19 :
                // CSW Recording, Generalized Data
                if (hasTapeLength(4))
                {
                    uint32_t length = readTapeWord();
                    length = truncateTapeLength(length | (readTapeWord() << 16));
                    ignoreTapeData(length);
                    if (tapeMarkNames != 0)
                    {
                        if (snprintf(tapeMarkNames[index], MENU_STR_LEN,
                            "UNSUPPORTED ID %d", blockId) >= MENU_STR_LEN)
                        {
                            tapeMarkNames[index][(MENU_STR_LEN - 1)] = 0;
                        }
                        tapeMarkPosition[index] = blockPosition;
                        ++index;
                    }
                }
                break;
            case 0x20 :
                // Insert Pause Block
                if (hasTapeLength(2))
                {
                    uint16_t durationMs = readTapeWord();
                    if (tapeMarkNames != 0)
                    {
                        if (durationMs == 0)
                        {
                            strcpy(tapeMarkNames[index], "Stop Tape");
                        } else if (snprintf(tapeMarkNames[index], MENU_STR_LEN,
                            "Pause %d ms", durationMs) >= MENU_STR_LEN)
                        {
                            tapeMarkNames[index][(MENU_STR_LEN - 1)] = 0;
                        }
                        tapeMarkPosition[index] = blockPosition;
                        ++index;
                    }
                }
                break;
            case 0x21 :
            case 0x30 :
                // Group Start, Text Description
                if (hasTapeLength(1))
                {
                    uint8_t length = truncateTapeLength(readTapeByte());
                    if (tapeMarkNames != 0)
                    {
                        uint8_t size = ((length >= MENU_STR_LEN) ?
                            (MENU_STR_LEN - 1) : length);
                        memcpy(tapeMarkNames[index],
                            (void *)&(tapeBuffer[tapePosition]), size);
                        tapeMarkNames[index][size] = 0;
                        tapeMarkPosition[index] = blockPosition;
                        ++index;
                    }
                    ignoreTapeData(length);
                }
                break;
            case 0x23 :
            case 0x24 :
                // Jump To Block, Loop Start
                ignoreTapeData(2);
                break;
            case 0x26:
                // Call Sequence
                if (hasTapeLength(2))
                {
                    uint32_t length = truncateTapeLength(readTapeWord() * 2);
                    ignoreTapeData(length);
                }
                break;
            case 0x22 :
            case 0x25 :
            case 0x27 :
                // Group End, Loop End, Return From Sequence
                break;
            case 0x28 :
                // Select
                if (hasTapeLength(2))
                {
                    uint16_t length = readTapeWord();
                    ignoreTapeData(length);
                    if (tapeMarkNames != 0)
                    {
                        if (snprintf(tapeMarkNames[index], MENU_STR_LEN,
                            "UNSUPPORTED ID %d", blockId) >= MENU_STR_LEN)
                        {
                            tapeMarkNames[index][(MENU_STR_LEN - 1)] = 0;
                        }
                        tapeMarkPosition[index] = blockPosition;
                        ++index;
                    }
                }
                break;
            case 0x2A :
                // Stop Tape if 48K Block
                ignoreTapeData(4);
                break;
            case 0x2B :
                // Set Signal Level
                ignoreTapeData(5);
                break;
            case 0x31 :
                // Message
                if (hasTapeLength(2))
                {
                    ignoreTapeData(1);
                    uint8_t length = truncateTapeLength(readTapeByte());
                    if (tapeMarkNames != 0)
                    {
                        uint8_t size = ((length >= MENU_STR_LEN) ?
                            (MENU_STR_LEN - 1) : length);
                        memcpy(tapeMarkNames[index],
                            (void *)&(tapeBuffer[tapePosition]), size);
                        tapeMarkNames[index][size] = 0;
                        tapeMarkPosition[index] = blockPosition;
                        ++index;
                    }
                    ignoreTapeData(length);
                }
                break;
            case 0x32 :
                // Archive Info
                if (hasTapeLength(2))
                {
                    uint16_t length = truncateTapeLength(readTapeWord());
                    ignoreTapeData(length);
                }
                break;
            case 0x33 :
                // Hardware Type
                if (hasTapeLength(1))
                {
                    uint16_t length = readTapeByte();
                    length = truncateTapeLength(3 * length);
                    ignoreTapeData(length);
                }
                break;
            case 0x35 :
                // Custom Info
                if (hasTapeLength(14))
                {
                    if (tapeMarkNames != 0)
                    {
                        memcpy(tapeMarkNames[index],
                            (void *)&(tapeBuffer[tapePosition]), 10);
                        tapeMarkNames[index][10] = 0;
                        tapeMarkPosition[index] = blockPosition;
                        ++index;
                    }
                    ignoreTapeData(10);
                    uint32_t length = readTapeWord();
                    length = truncateTapeLength(length | (readTapeWord() << 16));
                    ignoreTapeData(length);
                }
            case 0x5A :
                // Glue Block
                ignoreTapeData(8);
                break;
            default :
                menuPrintDebug(false, F_CSTR("tzxPlayer %d unknown ID %d"),
                    (tapePosition - 1), blockId);
                tapePosition = tapeLength;
                break;
        }

        // Determine if complete scan action
        if (seekBlock)
        {
            ++blockCount;
            if (blockCount >= blockNum)
            {
                return blockCount;
            }
        } else if ((tapeMarkNames == 0) && (tapePosition > currentTapePosition))
        {
            break;
        } else {
            ++blockCount;
        }
    }

    // Store markers, and restore tape position
    if (!seekBlock)
    {
        tapeMarkCount = index;
        tapePosition = currentTapePosition;
    }
    return blockCount;
}

void TzxPlayerZXTeensy::jumpRelative(int16_t target)
{
    if (target > 0)
    {
        doScanTape(true, true, target, 0);
    } else if (target < 0)
    {
        uint32_t block = doScanTape(false, false, 0, 0);
        doScanTape(true, false, block + target, 0);
    }
}

void TzxPlayerZXTeensy::scanTape(char (*tapeMarkNames)[MENU_STR_LEN])
{
    doScanTape(false, false, 0, tapeMarkNames);
}

void TzxPlayerZXTeensy::seek(uint8_t index)
{
    if (index < tapeMarkCount)
    {
        isPlaying = false;
        isBuffering = false;
        tapeBufferEnded = false;
        dataBlockSize = 0;
        currentBlock = BLOCK_IDLE;
        dataBuffer.clear();
        tapePosition = tapeMarkPosition[index];
    }
}

void TzxPlayerZXTeensy::begin(volatile uint8_t* buffer, size_t size)
{
    // Ensure the player is reset
    end();

    // Store the tape buffer
    tapeBuffer = buffer;
    tapeLength = size;
    if (memcmp((void*)tapeBuffer, "ZXTape!", 7) == 0)
    {
        tapePosition = 0x0A;
        isTzxTapeFile = true;
    } else {
        tapePosition = 0;
        isTzxTapeFile = false;
    }

    // Enable the player
    enabled = true;
}

void TzxPlayerZXTeensy::end()
{
    isPlaying = false;
    isPaused = false;
    isBuffering = false;
    tapeBufferEnded = false;
    dataBlockSize = 0;
    currentBlock = BLOCK_IDLE;
    dataBuffer.clear();
    enabled = false;
    tapeMarkCount = 0;
    tapeStackCount = 0;
}
