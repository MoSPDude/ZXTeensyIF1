
#ifndef TZX_PLAYER_ZX_TEENSY_H
#define TZX_PLAYER_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "WString.h"
#include "RingBuffer.h"
#include "DefinesZXTeensy.h"
#include <SD.h>
#include <SdFat.h>

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
            // NOTE: These blocks require special handling in runTape and getTapeByte
            BLOCK_STOP,
            BLOCK_PAUSE,
            BLOCK_LOW,
            BLOCK_HIGH,
            BLOCK_SAMPLES
        } block_type_t;

    protected :
        static const size_t DATA_BUFFER_SIZE = 1024;
        static const size_t TAPE_BUFFER_SIZE = 128;
        static const size_t COMMAND_SIZE = 64;
        RingBuffer<DATA_BUFFER_SIZE> dataBuffer;
        volatile bool enabled;
        volatile bool isBuffering;
        volatile bool isPlaying;
        volatile bool isPaused;
        volatile bool currentLevel;
        volatile block_type_t currentBlock;
        volatile uint16_t zeroDuration;
        volatile uint16_t oneDuration;
        volatile uint32_t numBytes;
        volatile uint8_t numFinalBits;

        volatile bool doublePulse;
        volatile uint8_t pulseData;
        volatile int pulseShiftCount;
        volatile uint32_t pulseDuration;
        volatile uint32_t edgeCycleCount;

        static const size_t BUFFER_SIZE = (0x2000 * 48);
        static const size_t FILE_BUFFER_SIZE = (0x2000 * 4);
        static const size_t FILE_BUFFER_MARK = (FILE_BUFFER_SIZE - DATA_BUFFER_SIZE);
        File tapeFile;
        bool isStreamed;
        volatile bool isTzxTapeFile;
        volatile uint8_t* tapeBuffer;
        volatile size_t bufferPosition;
        volatile size_t tapeFillPosition;
        volatile size_t bufferFillPosition;
        volatile size_t tapePosition;
        volatile size_t tapeLength;
        volatile uint32_t dataBlockSize;
        volatile uint16_t pauseAfterBlock;
        volatile bool tapeBufferStarted;
        volatile bool tapeBufferEnded;
        volatile bool tapeBufferAutoPlay;

        typedef struct {
            size_t position;
            uint16_t argument;
        } tape_stack_t;

        static const size_t TAPE_STACK_SIZE = 8;
        tape_stack_t tapeStack[TAPE_STACK_SIZE];
        uint8_t tapeStackCount;

        bool stopOn48k;

    public :
        size_t tapeMarkPosition[255];
        uint8_t tapeMarkCount;

    protected :
        void sendStopCommand();
        void sendLevelCommand(uint8_t level);
        void sendPulseCommand(uint16_t length);
        void sendPulseSeqCommand(uint8_t numPulses, uint16_t firstLength);
        void sendPauseCommand(uint16_t durationMs);
        void sendPilotCommand(uint16_t numPulses, uint16_t pulseLength);
        void sendSyncCommand(uint16_t firstLength, uint16_t secondLength);
        void sendDataCommand(uint16_t zeroLength, uint16_t oneLength,
            uint32_t numBytes, uint8_t numFinalBits, uint8_t firstByte);
        void sendSamplesCommand(uint16_t pulseLength, uint32_t numBytes,
            uint8_t numFinalBits, uint8_t firstByte);

        void insertStandardSpeedBlock(uint32_t numBytes, uint8_t flag);
        void insertPauseBlock(uint16_t durationMs);

        bool loadStandardSpeedBlock();
        bool loadTurboSpeedBlock();
        bool loadPureDataBlock();
        bool loadPulseSequenceBlock();
        bool loadDirectRecordingBlock();
        bool loadFromTape();
        void manageTape(bool reload);

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
                case BLOCK_SAMPLES :
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
                    pulseData = 0x00;
                    break;
                case BLOCK_LOW :
                    currentLevel = false;
                    return runTapeNextBlock();
                case BLOCK_HIGH :
                    currentLevel = true;
                    return runTapeNextBlock();
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

        inline bool runTapeNextBlock() __attribute__((optimize("O3")))
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
                numBytes |= (dataBuffer.readRaw() << 16);
                numBytes |= (dataBuffer.readRaw() << 24);
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
        void jumpRelative(int16_t target);
        bool decodeBasicHeader(char* blockName);
        uint32_t doScanTape(bool seekBlock, bool seekRelative,
            uint32_t blockNum, char (*tapeMarkNames)[MENU_STR_LEN]);

        inline __attribute__((always_inline)) uint8_t readTapeByte()
        {
            uint8_t data = *(getTapeBufferPtr());
            tapePosition += 1;
            return data;
        }

        inline __attribute__((always_inline)) uint16_t readTapeWord()
        {
            volatile uint8_t* ptr = getTapeBufferPtr();
            uint16_t data = *ptr++;
            data |= (*ptr << 8);
            tapePosition += 2;
            return data;
        }

        inline __attribute__((always_inline)) void insertTapeData(size_t size)
        {
            dataBuffer.writeBlock((uint8_t*)getTapeBufferPtr(), size);
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

        inline __attribute__((always_inline)) bool hasTapeLength(size_t size)
        {
            return ((tapePosition + size) <= tapeLength);
        }

        inline __attribute__((always_inline)) void ignoreTapeData(size_t size)
        {
            tapePosition += size;
        }

        inline __attribute__((always_inline)) volatile uint8_t* getTapeBufferPtr()
        {
            return tapeBuffer + (tapePosition - bufferPosition);
        }

    public :
        constexpr TzxPlayerZXTeensy() : enabled(false), isBuffering(false), isPlaying(false),
            isPaused(false), currentLevel(false), currentBlock(BLOCK_IDLE),
            zeroDuration(0), oneDuration(0), numBytes(0), numFinalBits(0),
            doublePulse(false), pulseData(0xAA), pulseShiftCount(0),
            pulseDuration(0), edgeCycleCount(0), tapeFile(), isStreamed(false),
            isTzxTapeFile(false), tapeBuffer(0), bufferPosition(0), tapeFillPosition(0),
            bufferFillPosition(0), tapePosition(0), tapeLength(0), dataBlockSize(0),
            pauseAfterBlock(0), tapeBufferStarted(false), tapeBufferEnded(false),
            tapeBufferAutoPlay(false), tapeStack{}, tapeStackCount(0), stopOn48k(false),
            tapeMarkPosition{}, tapeMarkCount(0)
        {
        }

        void scanTape(char (*tapeMarkNames)[MENU_STR_LEN]);
        void seek(uint8_t index);
        bool begin(char* fileName, volatile uint8_t* buffer, size_t length, bool is48k);
        bool reopen(char* fileName);
        void close();
        void end();

        // NOTE: onTick is main loop, so optimize
        inline void onTick() __attribute__((always_inline, hot, optimize("O3")))
        {
            if (isBuffering)
            {
                bufferTape();
            }
            if (isPlaying && !isPaused)
            {
                isPlaying = runTape();
            }
        }

        inline void play() __attribute__((always_inline, hot, optimize("O3")))
        {
            if (enabled && !isPlaying)
            {
                if (startTape())
                {
                    edgeCycleCount = ARM_DWT_CYCCNT;
                    isPlaying = true;
                    isPaused = false;
                } else if (!tapeBufferEnded)
                {
                    isBuffering = true;
                    tapeBufferAutoPlay = true;
                }
            }
        }

        inline void pause() __attribute__((always_inline, hot, optimize("O3")))
        {
            if (enabled && isPlaying && !isPaused)
            {
                isPaused = true;
            }
        }

        inline __attribute__((always_inline)) void unpause()
        {
            if (enabled && isPlaying && isPaused)
            {
                edgeCycleCount = ARM_DWT_CYCCNT;
                pulseDuration = TEENSY_CLK_FREQ;
                isPaused = false;
            }
        }

        inline __attribute__((always_inline)) bool isTapePlaying()
        {
            return (isPlaying && !isPaused);
        }

        inline __attribute__((always_inline)) bool isTapePaused()
        {
            return isPaused;
        }

        inline uint8_t getTapeByte() __attribute__((always_inline, hot, optimize("O3")))
        {
            // Detect if an edge has been missed
            if ((currentBlock <= BLOCK_STOP) &&
                ((ARM_DWT_CYCCNT - edgeCycleCount) >= pulseDuration))
            {
                return (currentLevel ? 0xFF : 0xBF);
            }

            // NOTE: TZX "low" means pull-ups active ie. bit high
            return (currentLevel ? 0xBF : 0xFF);
        }

        inline size_t getPosition(size_t* position)
        {
            *position = tapePosition;
            return tapeLength;
        }

        inline size_t savePositionState(size_t* position, size_t* tapeBufferPosition,
                size_t* fillPosition, size_t* tapeBufferFillPosition)
        {
            *position = tapePosition;
            *tapeBufferPosition = bufferPosition;
            *fillPosition = tapeFillPosition;
            *tapeBufferFillPosition = bufferFillPosition;
            return tapeLength;
        }

        inline void restorePositionState(size_t position, size_t tapeBufferPosition,
            size_t fillPosition, size_t tapeBufferFillPosition)
        {
            isPlaying = false;
            isPaused = false;
            isBuffering = false;
            tapeBufferEnded = false;
            dataBlockSize = 0;
            currentBlock = BLOCK_IDLE;
            dataBuffer.clear();
            tapePosition = position;
            bufferPosition = tapeBufferPosition;
            tapeFillPosition = fillPosition;
            bufferFillPosition = tapeBufferFillPosition;
        }

        inline bool isStreamingFile()
        {
            return isStreamed;
        }
};

#endif
