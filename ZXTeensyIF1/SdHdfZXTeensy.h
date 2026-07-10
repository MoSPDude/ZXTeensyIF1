
#ifndef SD_HDF_ZX_TEENSY_H
#define SD_HDF_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "RingBuffer.h"
#include "usb_serial.h"
#include "DefinesZXTeensy.h"
#include <SD.h>
#include <SdFat.h>

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
            CMD_READ_SINGLE_BLOCK = 17,
            CMD_WRITE_BLOCK     = 24,
            APP_SEND_OP_COND    = 41,
            CMD_APP_CMD         = 55,
            CMD_READ_OCR        = 58
        } command_t;

    protected :
        RingBuffer<READ_BUFFER_SIZE> sdSpiReadBuffer;

        File sdCard;
        char sdCardPath[MAX_PATH];
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

        uint64_t sectorOffset;
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

        inline __attribute__((always_inline)) void openSdCard()
        {
            if (!validSdCard)
            {
                if (!readOnlySdCard)
                {
                    sdCard = SD.open(sdCardPath, FILE_WRITE_BEGIN);
                    if (!sdCard)
                    {
                        readOnlySdCard = true;
                        sdCard = SD.open(sdCardPath, FILE_READ);
                    }
                } else {
                    sdCard = SD.open(sdCardPath, FILE_READ);
                }
                if (sdCard)
                {
                    currentSector = numSectors;
                    validSdCard = true;
                }
            }
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
                            openSdCard();
                            if (sdCard && (commandArgument < numSectors))
                            {
                                writeStatusData();
                                writeReadData(0xFF);
                                if (commandArgument != currentSector)
                                {
                                    uint64_t offset = (commandArgument * 512) + sectorOffset;
                                    sdCard.seek(offset, SeekSet);
                                    currentSector = commandArgument;
                                }
                                sdCard.read((uint8_t*)dataBuffer, 512);
                                ++currentSector;
                                writeReadData(0xFE);
                                sdSpiReadBuffer.writeBlock((uint8_t*)dataBuffer, 512);

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
                            openSdCard();
                            if (sdCard && (commandArgument < numSectors))
                            {
                                writeStatusData();
                                writeReadData(0xFF);
                                if (commandArgument != currentSector)
                                {
                                    uint64_t offset = (commandArgument * 512) + sectorOffset;
                                    sdCard.seek(offset, SeekSet);
                                    currentSector = commandArgument;
                                }
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
                            uint32_t cardSize = (numSectors >> 10) - 1;
                            cid[8] = ((cardSize & 0xFF) >> 8);
                            cid[9] = (cardSize & 0xFF);
                            writeStatusData();
                            writeReadData(0xFE);
                            for (int i = 0; i < 16; ++i)
                            {
                                writeReadData(cid[i]);
                            }

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
                            writeReadData(0xFE);
                            for (int i = 0; i < 16; ++i)
                            {
                                writeReadData(cid[i]);
                            }

                            // Generate the two CRC bytes
                            writeReadData(0x00);
                            writeReadData(0x00);
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
        constexpr SdHdfZXTeensy() : sdCard(), sdCardPath(), validSdCard(false),
            readOnlySdCard(false), isSdIdle(true), currentState(STATE_IDLE),
            currentCommand(CMD_IDLE), commandAppCmd(false), commandArgument(0),
            dataActive(false), dataRegister(0), dataIndex(0), dataBuffer(),
            sectorOffset(0), currentSector(0), numSectors(0)
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
                    commandArgument = (data << 24);
                    ++currentState;
                    break;
                case STATE_CMD_ARG3 :
                    commandArgument |= (data << 16);
                    ++currentState;
                    break;
                case STATE_CMD_ARG2 :
                    commandArgument |= (data << 8);
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

            // Load the image header
            numSectors = 0;
            currentSector = 0;
            readOnlySdCard = false;
            strcpy(sdCardPath, filename);
            openSdCard();
            if (sdCard)
            {
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
                    sdCard.seek(0, SeekEnd);
                    uint64_t cardsize = ((sdCard.size() - sectorOffset) / 512);
                    numSectors = cardsize;
                    currentSector = numSectors;
                    return true;
                }
                sdCard.close();
            }
            validSdCard = false;
            sdCardPath[0] = 0;
            return false;
        }

        void end()
        {
            if (validSdCard)
            {
                sdCard.close();
                sdCard = File();
                validSdCard = false;
            }
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
