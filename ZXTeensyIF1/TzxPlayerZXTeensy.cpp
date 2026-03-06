
#include "TzxPlayerZXTeensy.h"

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
    sendPulseCommand(0xDAC);
    if (durationMs > 1)
    {
        sendPauseCommand(durationMs - 1);
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
            oneDuration : zeroDuration) * TICK_DELAY_CNT;
        pulseDuration = newDuration;
    }
    return true;
}

bool TzxPlayerZXTeensy::startTape()
{
    if (runTapeNextBlock())
    {
        uint32_t newDuration = (((pulseData & 0x80) != 0) ?
            oneDuration : zeroDuration) * TICK_DELAY_CNT;
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
            insertPauseBlock(1000);
        }
    }
    if ((dataBlockSize == 0) &&
        (dataBuffer.getFree() > (TAPE_BUFFER_SIZE + COMMAND_SIZE)))
    {
        uint16_t count = 0;
        if (truncateTapeLength(2) >= 2)
        {
            dataBlockSize = readTapeWord();
            count = truncateTapeLength(
                (dataBlockSize > TAPE_BUFFER_SIZE) ?
                    TAPE_BUFFER_SIZE : dataBlockSize);
            if (count > 0)
            {
                if (!tapeBufferStarted)
                {
                    insertPauseBlock(1000);
                    tapeBufferStarted = true;
                }
                insertStandardSpeedBlock(dataBlockSize, readTapeByte());
                insertTapeData(count - 1);
                dataBlockSize -= count;
                if (dataBlockSize == 0)
                {
                    insertPauseBlock(1000);
                }

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
            }
        }
        if (count == 0)
        {
            tapeBufferEnded = true;
            isBuffering = false;
        }
    }
}

void TzxPlayerZXTeensy::onTick()
{
    if (isBuffering)
    {
        bufferTape();
    }
    if (isPlaying)
    {
        isPlaying = runTape();
    }
}

void TzxPlayerZXTeensy::begin(volatile uint8_t* buffer, size_t size)
{
    // Store the tape buffer
    tapeBuffer = buffer;
    tapeLength = size;
    tapePosition = 0;

    // Reset the player state
    end();

    // Enable the tape
    enabled = true;
}

void TzxPlayerZXTeensy::end()
{
    isPlaying = false;
    isBuffering = false;
    dataBlockSize = 0;
    currentBlock = BLOCK_IDLE;
    dataBuffer.clear();
    enabled = false;
}