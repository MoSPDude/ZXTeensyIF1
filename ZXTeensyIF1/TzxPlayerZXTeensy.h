
#ifndef TZX_PLAYER_ZX_TEENSY_H
#define TZX_PLAYER_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "RingBuffer.h"
#include "DefinesZXTeensy.h"

//#define DEBUG_TAPE_OUTPUT

class TzxPlayerZXTeensy
{
    public :
        typedef enum {
            BLOCK_IDLE,
            BLOCK_PILOT,
            BLOCK_SYNC,
            BLOCK_DATA,
            BLOCK_PULSES,
            BLOCK_PAUSE,
            BLOCK_STOP
        } block_type_t;

    protected :
        static const size_t DATA_BUFFER_SIZE = 1024;
        static const size_t TAPE_BUFFER_SIZE = 128;
        static const size_t COMMAND_SIZE = 64;
        RingBuffer<DATA_BUFFER_SIZE> dataBuffer;
        volatile bool enabled;
        volatile bool isBuffering;
        volatile bool isPlaying;
        volatile bool currentLevel;
        volatile block_type_t currentBlock;
        volatile uint16_t zeroDuration;
        volatile uint16_t oneDuration;
        volatile uint16_t numBytes;
        volatile uint8_t numFinalBits;

        volatile bool doublePulse;
        volatile uint8_t pulseData;
        volatile int pulseShiftCount;
        volatile uint32_t pulseDuration;
        volatile uint32_t edgeCycleCount;

        volatile bool isTzxTapeFile;
        volatile uint8_t* tapeBuffer;
        volatile size_t tapePosition;
        volatile size_t tapeLength;
        volatile uint32_t dataBlockSize;
        volatile uint16_t pauseAfterBlock;
        volatile bool tapeBufferStarted;
        volatile bool tapeBufferEnded;
        volatile bool tapeBufferAutoPlay;

        void sendStopCommand();
        void sendPulseCommand(uint16_t length);
        void sendPulseSeqCommand(uint8_t numPulses, uint16_t firstLength);
        void sendPauseCommand(uint16_t durationMs);
        void sendPilotCommand(uint16_t numPulses, uint16_t pulseLength);
        void sendSyncCommand(uint16_t firstLength, uint16_t secondLength);
        void sendDataCommand(uint16_t zeroLength, uint16_t oneLength,
            uint16_t numBytes, uint8_t numFinalBits, uint8_t firstByte);

        void insertStandardSpeedBlock(uint16_t numBytes, uint8_t flag);
        void insertPauseBlock(uint16_t durationMs);

        bool loadStandardSpeedBlock();
        bool loadTurboSpeedBlock();
        bool loadPureDataBlock();
        bool loadPulseSequenceBlock();
        bool loadFromTape();

        inline bool runTapeNextByte() __attribute__((always_inline, optimize("O3")))
        {
            switch (currentBlock)
            {
                case BLOCK_SYNC :
                    pulseData = 0x40;
                    break;
                case BLOCK_DATA :
                    doublePulse = true;
                    if (dataBuffer.canRead())
                    {
                        pulseData = dataBuffer.readRaw();
                    } else {
                        pulseData = 0xAA;
                        pulseShiftCount = 0;
                        return false;
                    }
                    break;
                case BLOCK_PULSES :
                    if (dataBuffer.canRead())
                    {
                        pulseData = 0x40;
                        zeroDuration = dataBuffer.readRaw();
                        zeroDuration |= (dataBuffer.readRaw() << 8);
                        if (dataBuffer.canRead() && (numBytes > 1))
                        {
                            pulseShiftCount = 6;
                            oneDuration = dataBuffer.readRaw();
                            oneDuration |= (dataBuffer.readRaw() << 8);
                            --numBytes;
                        } else {
                            pulseShiftCount = 7;
                        }
                    } else {
                        return false;
                    }
                    break;
                case BLOCK_PAUSE :
                    currentLevel = false;
                    pulseData = 0x00;
                    break;
                case BLOCK_STOP :
                    return false;
                default :
                    pulseData = 0xAA;
                    break;
            }
            --numBytes;
            if (currentBlock != BLOCK_PULSES)
            {
                if (numBytes > 0)
                {
                    pulseShiftCount = 0;
                } else {
                    pulseShiftCount = (8 - numFinalBits);
                }
            }
            return true;
        }

        inline bool runTapeNextBlock() __attribute__((always_inline, optimize("O3")))
        {
            if (dataBuffer.canRead())
            {
                currentBlock = (block_type_t)dataBuffer.readRaw();
                zeroDuration = dataBuffer.readRaw();
                zeroDuration |= (dataBuffer.readRaw() << 8);
                oneDuration = dataBuffer.readRaw();
                oneDuration |= (dataBuffer.readRaw() << 8);
                numBytes = dataBuffer.readRaw();
                numBytes |= (dataBuffer.readRaw() << 8);
                numFinalBits = dataBuffer.readRaw();
            } else {
                currentBlock = BLOCK_IDLE;
            }
            if (currentBlock != BLOCK_IDLE)
            {
                return runTapeNextByte();
            }
            return false;
        }

        // NOTE: runTape is called from ISR, so optimize
        bool runTape() __attribute__((hot, optimize("O3")));

        bool startTape();
        void bufferTape();

        inline __attribute__((always_inline)) uint8_t readTapeByte()
        {
            uint8_t data = tapeBuffer[tapePosition];
            tapePosition += 1;
            return data;
        }

        inline __attribute__((always_inline)) uint16_t readTapeWord()
        {
            uint16_t data = tapeBuffer[tapePosition];
            data |= (tapeBuffer[(tapePosition + 1)] << 8);
            tapePosition += 2;
            return data;
        }

        inline __attribute__((always_inline)) void insertTapeData(size_t size)
        {
            dataBuffer.writeBlock((uint8_t*)&(tapeBuffer[tapePosition]), size);
            tapePosition += size;
        }

        inline __attribute__((always_inline)) size_t truncateTapeLength(size_t size)
        {
            if (size > (tapeLength - tapePosition))
            {
                size = (tapeLength - tapePosition);
            }
            return size;
        }

        inline __attribute__((always_inline)) void ignoreTapeData(size_t size)
        {
            tapePosition += size;
        }

    public :
        constexpr TzxPlayerZXTeensy() : enabled(false), isBuffering(false), isPlaying(false),
            currentLevel(false), currentBlock(BLOCK_IDLE),
            zeroDuration(0), oneDuration(0), numBytes(0), numFinalBits(0),
            doublePulse(false), pulseData(0xAA), pulseShiftCount(0),
            pulseDuration(0), edgeCycleCount(0), isTzxTapeFile(false),
            tapeBuffer(0), tapePosition(0), tapeLength(0), dataBlockSize(0), pauseAfterBlock(0),
            tapeBufferStarted(false), tapeBufferEnded(false), tapeBufferAutoPlay(false)
        {
        }

        void begin(volatile uint8_t* buffer, size_t size);
        void end();

        // NOTE: onTick is main loop, so optimize
        inline void onTick() __attribute__((always_inline, hot, optimize("O3")))
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

        inline __attribute__((always_inline)) void play()
        {
            if (enabled && !isPlaying)
            {
                if (startTape())
                {
                    edgeCycleCount = ARM_DWT_CYCCNT;
                    isPlaying = true;
                } else if (!tapeBufferEnded)
                {
                    isBuffering = true;
                    tapeBufferAutoPlay = true;
                }
            }
        }

        inline __attribute__((always_inline)) bool isTapePlaying()
        {
            return (isPlaying);
        }

        inline __attribute__((always_inline)) uint8_t getTapeByte()
        {
            if ((currentBlock != BLOCK_PAUSE) &&
                ((ARM_DWT_CYCCNT - edgeCycleCount) >= pulseDuration))
            {
                return (currentLevel ? 0xBF : 0xFF);
            }
            return (currentLevel ? 0xFF : 0xBF);
        }
};

#endif
