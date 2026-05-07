
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
        if (currentBlock != BLOCK_PAUSE)
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
        uint32_t newDuration = (((pulseData & 0x80) != 0) ?
            oneDuration : zeroDuration) * TAPE_DELAY_CNT;
        pulseDuration = newDuration;
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

bool TzxPlayerZXTeensy::loadFromTape()
{
    if (isTzxTapeFile)
    {
        if (truncateTapeLength(1) >= 1)
        {
            uint8_t blockId = readTapeByte();
            switch (blockId)
            {
                case 0x10 :
                    // Load Standard Speed Block
                    if (truncateTapeLength(4) >= 4)
                    {
                        pauseAfterBlock = readTapeWord();
                        return loadStandardSpeedBlock();
                    }
                    break;
                case 0x11 :
                    // Load Turbo Speed Block
                    if (truncateTapeLength(0x12) >= 0x12)
                    {
                        return loadTurboSpeedBlock();
                    }
                    break;
                case 0x12 :
                    // Load Pure Tone Block
                    if (truncateTapeLength(4) >= 4)
                    {
                        uint16_t pulseLength = readTapeWord();
                        sendPilotCommand(readTapeWord(), pulseLength);
                        return true;
                    }
                    break;
                case 0x13 :
                    // Load Pulse Sequence
                    if (truncateTapeLength(1) >= 1)
                    {
                        return loadPulseSequenceBlock();
                    }
                case 0x14 :
                    // Load Pure Data Block
                    if (truncateTapeLength(0x10) >= 0x10)
                    {
                        return loadPureDataBlock();
                    }
                    break;
                case 0x20 :
                    // Load Insert Pause Block
                    if (truncateTapeLength(2) >= 2)
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
                    if (truncateTapeLength(1) >= 1)
                    {
                        uint8_t length = truncateTapeLength(readTapeByte());
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x22 :
                    // Group End
                    return loadFromTape();
                case 0x2A :
                    // Load Stop Tape if 48K Block
                    if (truncateTapeLength(4) >= 4)
                    {
                        ignoreTapeData(4);
                        return loadFromTape();
                    }
                    break;
                case 0x30 :
                    // Load Text Description
                    if (truncateTapeLength(1) >= 1)
                    {
                        uint8_t length = truncateTapeLength(readTapeByte());
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x31 :
                    // Load Message
                    if (truncateTapeLength(1) >= 1)
                    {
                        ignoreTapeData(1);
                        uint8_t length = truncateTapeLength(readTapeByte());
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x32 :
                    // Archive info
                    if (truncateTapeLength(2) >= 2)
                    {
                        uint16_t length = truncateTapeLength(readTapeWord());
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                case 0x33 :
                    // Hardware type
                    if (truncateTapeLength(1) >= 1)
                    {
                        uint16_t length = readTapeByte();
                        length = truncateTapeLength(3 * length);
                        ignoreTapeData(length);
                        return loadFromTape();
                    }
                    break;
                default :
                    menuPrintDebug(false, "tzxPlayer unknown ID 0x%h", blockId);
                    break;
            }
        }
    } else if (truncateTapeLength(2) >= 2)
    {
        pauseAfterBlock = 1000;
        return loadStandardSpeedBlock();
    }
    return false;
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
    dataBlockSize = 0;
    currentBlock = BLOCK_IDLE;
    dataBuffer.clear();
    enabled = false;
}
