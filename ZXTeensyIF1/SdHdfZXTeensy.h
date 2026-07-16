
#ifndef SD_HDF_ZX_TEENSY_H
#define SD_HDF_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "RingBuffer.h"
#include "usb_serial.h"
#include "DefinesZXTeensy.h"
#include <SD.h>
#include <SdFat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//define DEBUG_HDF_OUTPUT

template <size_t READ_BUFFER_SIZE> class SdHdfZXTeensy
{
    public :

        typedef enum {
            STATE_IDLE      = 0x00,
            STATE_CMD_ARG4  = 0x01,
            STATE_CMD_ARG3  = 0x02,
            STATE_CMD_ARG2  = 0x03,
            STATE_CMD_ARG1  = 0x04,
            STATE_CMD_CRC   = 0x05,
            STATE_DATA      = 0x08
        } state_t;

        typedef enum {
            CMD_IDLE            = 0,
            CMD_SEND_OP_COND    = 8,
            CMD_SEND_CSD        = 9,
            CMD_SEND_CID        = 10,
            CMD_SET_BLOCKLEN    = 16,
            CMD_READ_SINGLE_BLOCK = 17,
            CMD_WRITE_BLOCK     = 24,
            APP_SEND_OP_COND    = 41,
            CMD_APP_CMD         = 55,
            CMD_READ_OCR        = 58
        } command_t;

    protected :
        static const uint16_t MAX_IMAGE_SEGMENTS = 1000;
        static const uint16_t INVALID_SEGMENT = 0xFFFF;

        RingBuffer<READ_BUFFER_SIZE> sdSpiReadBuffer;

        File sdCard;
        char sdCardPath[MAX_PATH];
        size_t sdCardSegmentBaseLength;
        bool validSdCard;
        bool readOnlySdCard;
        bool isSdIdle;
        uint8_t currentState;
        uint8_t currentCommand;
        bool commandAppCmd;
        uint32_t commandArgument;
        bool dataActive;
        uint32_t dataRegister;
        uint32_t dataIndex;
        volatile uint8_t dataBuffer[528] __attribute__((aligned(16)));

        size_t sectorOffset;
        uint32_t* segmentSectors;
        uint16_t segmentCapacity;
        uint16_t numSegments;
        uint16_t currentSegment;
        uint32_t currentSegmentStart;
        uint32_t currentSegmentSectors;
        size_t currentSegmentOffset;
        uint32_t currentSector;
        uint32_t numSectors;

        inline __attribute__((always_inline)) void writeReadData(uint8_t data)
        {
            sdSpiReadBuffer.write(data);
        }

        inline __attribute__((always_inline)) void writeStatusData()
        {
            writeReadData(isSdIdle ? 0x01 : 0x00);
        }

        inline __attribute__((always_inline)) void writeErrorData(uint8_t error)
        {
            writeReadData(error | (isSdIdle ? 0x01 : 0x00));
        }

        bool allocateSegmentSectors(uint16_t capacity)
        {
            if ((capacity > 0) && (capacity <= MAX_IMAGE_SEGMENTS))
            {
                segmentSectors = (uint32_t*)malloc(
                    sizeof(uint32_t) * (capacity + 1));
                if (segmentSectors != 0)
                {
                    segmentCapacity = capacity;
                    return true;
                }
            }
            segmentCapacity = 0;
            return false;
        }

        void freeSegmentSectors()
        {
            if (segmentSectors != 0)
            {
                free(segmentSectors);
                segmentSectors = 0;
            }
            segmentCapacity = 0;
        }

        size_t findSegmentPathBaseLength() const
        {
            const char* extension = 0;
            size_t pathLength = strlen(sdCardPath);
            const char* p = sdCardPath + pathLength;
            while (p != sdCardPath)
            {
                --p;
                if ((*p == '/') || (*p == '\\'))
                {
                    break;
                }
                if (*p == '.')
                {
                    extension = p;
                    break;
                }
            }
            return ((extension != 0) ?
                (size_t)(extension - sdCardPath) : pathLength);
        }

        bool buildSegmentPath(uint16_t index, char* path) const
        {
            if (index > 0)
            {
                memcpy(path, sdCardPath, sdCardSegmentBaseLength);
                snprintf(&(path[sdCardSegmentBaseLength]),
                    (MAX_PATH - sdCardSegmentBaseLength), ".%03u",
                    (unsigned int)index);
                return true;
            } else {
                strncpy(path, sdCardPath, MAX_PATH);
                path[(MAX_PATH - 1)] = 0;
                return (path[0] != 0);
            }
        }

        File openSegmentFile(uint16_t index)
        {
            char segmentPath[MAX_PATH];
            if (!buildSegmentPath(index, segmentPath))
            {
                return File();
            }

            File file;
            if (!readOnlySdCard)
            {
                file = SD.open(segmentPath, FILE_WRITE_BEGIN);
                if (!file)
                {
                    readOnlySdCard = true;
                    file = SD.open(segmentPath, FILE_READ);
                }
            } else {
                file = SD.open(segmentPath, FILE_READ);
            }
            return file;
        }

        bool appendSegmentSectors(uint64_t sectorCount)
        {
            if ((segmentSectors != 0) && (numSegments < segmentCapacity) &&
                (sectorCount > 0))
            {
                uint64_t remainingSectors = 0xFFFFFFFFULL - numSectors;
                if (sectorCount > remainingSectors)
                {
                    sectorCount = remainingSectors;
                }
                if (sectorCount == 0)
                {
                    return false;
                }

                ++numSegments;
                numSectors += (uint32_t)sectorCount;
                segmentSectors[numSegments] = numSectors;
                return (numSectors < 0xFFFFFFFFUL);
            }
            return false;
        }

        bool segmentFileHasSectors(uint16_t index) const
        {
            char segmentPath[MAX_PATH];
            if (buildSegmentPath(index, segmentPath))
            {
                File segmentFile = SD.open(segmentPath, FILE_READ);
                if (segmentFile)
                {
                    bool result = (segmentFile.size() >= 512);
                    segmentFile.close();
                    return result;
                }
            }
            return false;
        }

        uint16_t countAdditionalSegments()
        {
            // Test for a single file image, with no additional segments
            if (!segmentFileHasSectors(1))
            {
                return 0;
            }

            // Perform a sparse search to find the last available segment file
            uint16_t low = 1;
            uint16_t high = 2;
            const uint16_t maxAdditionalSegment = MAX_IMAGE_SEGMENTS - 1;
            while ((high <= maxAdditionalSegment) &&
                segmentFileHasSectors(high))
            {
                low = high;
                if (high >= maxAdditionalSegment)
                {
                    return maxAdditionalSegment;
                }
                high = ((high > (maxAdditionalSegment >> 1)) ?
                    maxAdditionalSegment : (high << 1));
            }

            uint16_t firstMissingLow = low + 1;
            uint16_t firstMissingHigh = high;
            while (firstMissingLow < firstMissingHigh)
            {
                uint16_t index = firstMissingLow +
                    ((firstMissingHigh - firstMissingLow) >> 1);
                if (segmentFileHasSectors(index))
                {
                    firstMissingLow = index + 1;
                } else {
                    firstMissingHigh = index;
                }
            }
            return firstMissingLow - 1;
        }

        void discoverAdditionalSegments()
        {
            // Open and append each segment files capacity information
            for (uint16_t index = 1; index < segmentCapacity; ++index)
            {
                char segmentPath[MAX_PATH];
                if (buildSegmentPath(index, segmentPath))
                {
                    File segmentFile = SD.open(segmentPath, FILE_READ);
                    if (segmentFile)
                    {
                        uint64_t sectorCount = segmentFile.size() / 512;
                        segmentFile.close();
                        if (!appendSegmentSectors(sectorCount))
                        {
                            break;
                        }
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }
        }

        bool findSegmentForSector(uint32_t sector, uint16_t* segmentIndex,
            uint32_t* segmentStart) const
        {
            // Perform a binary search to find the segment that contains the
            // required sector
            if (segmentSectors == 0)
            {
                return false;
            }

            uint16_t low = 0;
            uint16_t high = numSegments;
            if ((currentSegment != INVALID_SEGMENT) &&
                (currentSegment < numSegments))
            {
                uint32_t currentEnd =
                    currentSegmentStart + currentSegmentSectors;
                if (sector < currentSegmentStart)
                {
                    high = currentSegment;
                } else if (sector >= currentEnd)
                {
                    low = currentSegment + 1;
                } else {
                    *segmentIndex = currentSegment;
                    *segmentStart = currentSegmentStart;
                    return true;
                }
            }

            while (low < high)
            {
                uint16_t index = low + ((high - low) >> 1);
                uint32_t start = segmentSectors[index];
                uint32_t end = segmentSectors[index + 1];
                if (sector < start)
                {
                    high = index;
                } else if (sector >= end)
                {
                    low = index + 1;
                } else {
                    *segmentIndex = index;
                    *segmentStart = start;
                    return true;
                }
            }
            return false;
        }

        bool currentSegmentContains(uint32_t sector) const
        {
            return (validSdCard && (currentSegment != INVALID_SEGMENT) &&
                (sector >= currentSegmentStart) &&
                ((sector - currentSegmentStart) < currentSegmentSectors));
        }

        bool openSdCard(uint32_t sector)
        {
            // Ensure the sector is within the SD card
            if ((segmentSectors == 0) || (sector >= numSectors))
            {
                return false;
            }

            // Open the segment file that contains the sector, as required
            if (!currentSegmentContains(sector))
            {
                uint16_t segmentIndex = INVALID_SEGMENT;
                uint32_t segmentStart = 0;
                if (findSegmentForSector(sector, &segmentIndex, &segmentStart))
                {
                    if (sdCard)
                    {
                        sdCard.close();
                    }
                    sdCard = openSegmentFile(segmentIndex);
                    currentSector = numSectors;
                    if (sdCard)
                    {
                        validSdCard = true;
                        currentSegment = segmentIndex;
                        currentSegmentStart = segmentStart;
                        currentSegmentSectors =
                            segmentSectors[segmentIndex + 1] - segmentSectors[segmentIndex];
                        currentSegmentOffset = ((segmentIndex == 0) ? sectorOffset : 0);
                    } else {
                        validSdCard = false;
                        currentSegment = INVALID_SEGMENT;
                        currentSegmentStart = 0;
                        currentSegmentSectors = 0;
                        currentSegmentOffset = 0;
                    }
                } else {
                    validSdCard = false;
                }
            }
            return validSdCard;
        }

        bool seekSdCardSector(uint32_t sector)
        {
            if (openSdCard(sector))
            {
                if (sector != currentSector)
                {
                    size_t offset = currentSegmentOffset +
                        (((size_t)(sector - currentSegmentStart)) * 512);
                    if (!sdCard.seek(offset, SeekSet))
                    {
                        return false;
                    }
                    currentSector = sector;
                }
                return true;
            }
            return false;
        }

        bool processData(uint8_t data)
        {
            if (currentCommand == CMD_WRITE_BLOCK)
            {
                if (dataActive)
                {
                    dataBuffer[dataIndex] = data;
                    ++dataIndex;
                    if (dataIndex >= 0x202)
                    {
                        if (readOnlySdCard)
                        {
                            // Card is locked
                            writeReadData(0x0D);
                        } else {
                            // Consume the CRC bytes, then send data response
                            writeReadData(0x05);
                            writeReadData(0x00);
                            writeReadData(0x00);

                            // Write the data
                            sdCard.write((uint8_t*)dataBuffer, 512);
                            ++currentSector;
                        }
                        currentState = STATE_IDLE;
                        dataActive = false;
                    }
                } else if (data == 0xFE)
                {
                    dataActive = true;
                    dataIndex = 0;
                }
            }
            return false;
        }

        void executeCommand()
        {
            if (commandAppCmd)
            {
                commandAppCmd = false;
                switch (currentCommand)
                {
                    case APP_SEND_OP_COND :
                        isSdIdle = false;
                        writeStatusData();
                        break;
                    default :
                        break;
                }
            } else {
                switch (currentCommand)
                {
                    case CMD_READ_SINGLE_BLOCK :
                        {
                            if ((commandArgument < numSectors) &&
                                seekSdCardSector(commandArgument))
                            {
                                writeStatusData();
                                writeReadData(0xFF);
                                sdCard.read((uint8_t*)dataBuffer, 512);
                                ++currentSector;
                                sdSpiReadBuffer.writeBlockWithToken(0xFE,
                                    (uint8_t*)dataBuffer, 512);

                                // Generate the two CRC bytes
                                writeReadData(0x00);
                                writeReadData(0x00);
                            } else {
                                // Parameter error
                                writeErrorData(0x40);
                            }
                        }
                        break;
                    case CMD_WRITE_BLOCK :
                        {
                            if ((commandArgument < numSectors) &&
                                seekSdCardSector(commandArgument))
                            {
                                writeStatusData();
                                writeReadData(0xFF);
                                currentState = STATE_DATA;
                                return;
                            } else {
                                // Parameter error
                                writeErrorData(0x40);
                            }
                        }
                        break;
                    case CMD_IDLE :
                        isSdIdle = true;
                        writeStatusData();
                        break;
                    case CMD_SEND_OP_COND :
                        // Return the command argument (0x000001AA)
                        writeStatusData();
                        writeReadData(commandArgument >> 24);
                        writeReadData(commandArgument >> 16);
                        writeReadData(commandArgument >> 8);
                        writeReadData(commandArgument);
                        break;
                    case CMD_SEND_CSD :
                        {
                            // Return 16 bytes of CID/CSD as data packet
                            uint8_t cid[16] = { 0x40, 0x0E, 0x00, 0x32,
                                0x5B, 0x59, 0x00, 0x00,
                                0x1D, 0xA7, 0x7F, 0x80,
                                0x0A, 0x40, 0x00, 0x00 };
                            uint32_t cardSize = (numSectors >= 1024) ?
                                ((numSectors >> 10) - 1) : 0;
                            cid[7] = (cid[7] & 0xC0) | ((cardSize >> 16) & 0x3F);
                            cid[8] = ((cardSize >> 8) & 0xFF);
                            cid[9] = (cardSize & 0xFF);
                            writeStatusData();
                            sdSpiReadBuffer.writeBlockWithToken(0xFE, cid, 16);

                            // Generate the two CRC bytes
                            writeReadData(0x00);
                            writeReadData(0x00);
                        }
                        break;
                    case CMD_SEND_CID :
                        {
                            // Return 16 bytes of CID/CSD as data packet
                            uint8_t cid[16] = { 0x27, 'Z', 'X', 'T',
                                'N', 'S', 'Y', ' ',
                                0x60, 0xDA, 0x6C, 0x7F,
                                0xB3, 0x01, 0x92, 0x00 };
                            writeStatusData();
                            sdSpiReadBuffer.writeBlockWithToken(0xFE, cid, 16);

                            // Generate the two CRC bytes
                            writeReadData(0x00);
                            writeReadData(0x00);
                        }
                        break;
                    case CMD_SET_BLOCKLEN :
                        if (commandArgument == 512)
                        {
                            writeStatusData();
                        } else {
                            // Parameter error
                            writeErrorData(0x40);
                        }
                        break;
                    case CMD_APP_CMD :
                        commandAppCmd = true;
                        writeStatusData();
                        break;
                    case CMD_READ_OCR :
                        {
                            // Return the OCR register
                            uint32_t ocr = 0xC0FF8000;
                            writeStatusData();
                            writeReadData(ocr >> 24);
                            writeReadData(ocr >> 16);
                            writeReadData(ocr >> 8);
                            writeReadData(ocr);
                        }
                        break;
                    default :
                        break;
                }
            }

            // Return to idle after internally completing the command
            currentState = STATE_IDLE;
        }

    public :
        constexpr SdHdfZXTeensy() : sdCard(), sdCardPath(),
            sdCardSegmentBaseLength(0), validSdCard(false), readOnlySdCard(false),
            isSdIdle(true), currentState(STATE_IDLE), currentCommand(CMD_IDLE),
            commandAppCmd(false), commandArgument(0), dataActive(false),
            dataRegister(0), dataIndex(0), dataBuffer(), sectorOffset(0),
            segmentSectors(0), segmentCapacity(0), numSegments(0),
            currentSegment(INVALID_SEGMENT), currentSegmentStart(0),
            currentSegmentSectors(0), currentSegmentOffset(0),
            currentSector(0), numSectors(0)
        {
            sdCardPath[0] = 0;
        }

        inline __attribute__((always_inline)) void performTick(uint8_t data)
        {
            switch (currentState)
            {
                case STATE_IDLE :
                    if ((data & 0xC0) == 0x40)
                    {
                        currentCommand = data & 0x3F;
                        ++currentState;
                    }
                    break;
                case STATE_CMD_ARG4 :
                    commandArgument = ((uint32_t)data << 24);
                    ++currentState;
                    break;
                case STATE_CMD_ARG3 :
                    commandArgument |= ((uint32_t)data << 16);
                    ++currentState;
                    break;
                case STATE_CMD_ARG2 :
                    commandArgument |= ((uint32_t)data << 8);
                    ++currentState;
                    break;
                case STATE_CMD_ARG1 :
                    commandArgument |= data;
                    ++currentState;
                    break;
                case STATE_CMD_CRC :
                    // Consume the command CRC byte, and add one byte spacer
                    writeReadData(0xFF);
                    executeCommand();
                    break;
                case STATE_DATA :
                    processData(data);
                    break;
            }
        }

        bool begin(const char* filename)
        {
#ifdef DEBUG_HDF_OUTPUT
            Serial.begin(115200);
#endif

            // Close the previous image
            end();
            freeSegmentSectors();
            numSectors = 0;
            numSegments = 0;
            currentSector = 0;
            sectorOffset = 0;
            sdCardSegmentBaseLength = 0;
            readOnlySdCard = false;

            // Open the new image
            if ((filename == 0) || (strlen(filename) >= MAX_PATH))
            {
                sdCardPath[0] = 0;
                return false;
            }
            strcpy(sdCardPath, filename);
            sdCardSegmentBaseLength = findSegmentPathBaseLength();
            if (sdCardSegmentBaseLength > (MAX_PATH - 5))
            {
                sdCardPath[0] = 0;
                sdCardSegmentBaseLength = 0;
                return false;
            }

            // Load the image header
            sdCard = openSegmentFile(0);
            if (sdCard)
            {
                validSdCard = true;
                currentSegment = 0;
                currentSegmentStart = 0;
                currentSegmentOffset = 0;

                // Read the image header
                uint8_t header[16];
                if (sdCard.read(header, 16) >= 16)
                {
                    if (memcmp(header, "RS-IDE", 6) == 0)
                    {
                        // Find start of the data
                        sectorOffset = ((header[10] << 8) | header[9]);
                    } else {
                        // Assume as disk image
                        sectorOffset = 0;
                    }

                    // Determine the card size
                    if (sdCard.size() >= sectorOffset)
                    {
                        uint64_t baseSectorCount =
                            (sdCard.size() - sectorOffset) / 512;
                        if (baseSectorCount > 0)
                        {
                            uint16_t totalSegments =
                                countAdditionalSegments() + 1;
                            if (allocateSegmentSectors(totalSegments))
                            {
                                segmentSectors[0] = 0;
                                if (appendSegmentSectors(baseSectorCount))
                                {
                                    discoverAdditionalSegments();
                                    currentSegmentSectors =
                                        segmentSectors[1] - segmentSectors[0];
                                    currentSegmentOffset = sectorOffset;
                                    currentSector = numSectors;
                                    return true;
                                }
                            }
                        }
                    }
                }
                sdCard.close();
            }

            // The SD card image failed to load
            validSdCard = false;
            sdCardPath[0] = 0;
            sdCardSegmentBaseLength = 0;
            freeSegmentSectors();
            return false;
        }

        void end()
        {
            if (sdCard)
            {
                sdCard.close();
                sdCard = File();
            }
            validSdCard = false;
            currentSegment = INVALID_SEGMENT;
            currentSegmentStart = 0;
            currentSegmentSectors = 0;
            currentSegmentOffset = 0;
        }

        void reset(bool setSdIdle)
        {
            // NOTE: Do NOT set isSdIdle as the DivMMC can warm reset
            sdSpiReadBuffer.clear();
            currentState = SdHdfZXTeensy::STATE_IDLE;
            currentCommand = SdHdfZXTeensy::CMD_IDLE;
            commandAppCmd = false;
            commandArgument = 0;
            dataActive = false;
            dataRegister = 0;
            dataIndex = 0;
            end();
            if (setSdIdle)
            {
                isSdIdle = true;
            }
        }

        inline uint8_t readData() __attribute__((always_inline, hot, optimize("O3")))
        {
            return (sdSpiReadBuffer.canRead() ? sdSpiReadBuffer.readRaw() : 0xFF);
        }

        inline bool getSdIdle() const
        {
            return isSdIdle;
        }

        inline void setSdIdle(bool idle)
        {
            isSdIdle = idle;
        }
};

#endif
