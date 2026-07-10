
#ifndef SD_SDIO_ZX_TEENSY_H
#define SD_SDIO_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "RingBuffer.h"
#include "usb_serial.h"
#include <SD.h>
#include <SdFat.h>

//define DEBUG_HDF_OUTPUT

template <size_t READ_BUFFER_SIZE> class SdSdioZXTeensy
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
        RingBuffer<READ_BUFFER_SIZE> sdSpiReadBuffer;

        SdioCard* sdioCard;
        bool isSdIdle;
        uint8_t currentState;
        uint8_t currentCommand;
        bool commandAppCmd;
        uint32_t commandArgument;
        bool dataActive;
        uint32_t dataRegister;
        uint32_t dataIndex;
        volatile uint8_t dataBuffer[528] __attribute__((aligned(16)));

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
                        // Consume the CRC bytes, then send data response
                        writeReadData(0x05);
                        writeReadData(0x00);
                        writeReadData(0x00);

                        // Write the data
                        sdioCard->writeSector(currentSector, (uint8_t*)dataBuffer);
                        ++currentSector;
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
                            if (commandArgument < numSectors)
                            {
                                writeStatusData();
                                writeReadData(0xFF);
                                currentSector = commandArgument;
                                sdioCard->readSector(currentSector, (uint8_t*)dataBuffer);
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
                            if (commandArgument < numSectors)
                            {
                                writeStatusData();
                                writeReadData(0xFF);
                                currentSector = commandArgument;
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
                    case CMD_SEND_CID :
                        {
                            // Return 16 bytes of CID/CSD as data packet
                            uint8_t cid[16];
                            if (currentCommand == CMD_SEND_CSD)
                            {
                                sdioCard->readCSD((csd_t*)cid);
                            } else {
                                sdioCard->readCID((cid_t*)cid);
                            }
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
                            uint32_t ocr;
                            sdioCard->readOCR(&ocr);
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
        constexpr SdSdioZXTeensy() : sdioCard(0), isSdIdle(true), currentState(STATE_IDLE),
            currentCommand(CMD_IDLE), commandAppCmd(false), commandArgument(0),
            dataActive(false), dataRegister(0), dataIndex(0), dataBuffer(),
            currentSector(0), numSectors(0)
        {
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

        void begin(SdCard* card)
        {
#ifdef DEBUG_HDF_OUTPUT
            Serial.begin(115200);
#endif

            sdioCard = static_cast<SdioCard*>(card);
            if (sdioCard != 0)
            {
                numSectors = sdioCard->sectorCount();
                while (sdioCard->isBusy()) { yield(); };
            } else {
                numSectors = 0;
            }
        }

        void end()
        {
            sdioCard = 0;
        }

        void reset(bool setSdIdle)
        {
            // NOTE: Do NOT set isSdIdle as the DivMMC can warm reset
            sdSpiReadBuffer.clear();
            currentState = SdSdioZXTeensy::STATE_IDLE;
            currentCommand = SdSdioZXTeensy::CMD_IDLE;
            commandAppCmd = false;
            commandArgument = 0;
            dataActive = false;
            dataRegister = 0;
            dataIndex = 0;
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
