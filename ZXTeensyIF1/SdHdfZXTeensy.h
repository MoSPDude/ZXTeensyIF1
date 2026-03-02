//
// Modified from the Teensyduino Core Library
// Fixed SPI to SDHC bridge, with integrated RingBuffer for ZXTeensyIF1
//
/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2019 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/**
 * Copyright (c) 2011-2021 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef SD_HDF_ZX_TEENSY_H
#define SD_HDF_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "RingBuffer.h"
#include "usb_serial.h"
#include <SD.h>
#include <SdFat.h>

//define DEBUG_HDF_OUTPUT

class SdHdfZXTeensy
{
    public :

        typedef enum {
            STATE_IDLE      = 0x00,
            STATE_CMD_ARG4  = 0x01,
            STATE_CMD_ARG3  = 0x02,
            STATE_CMD_ARG2  = 0x03,
            STATE_CMD_ARG1  = 0x04,
            STATE_CMD_CRC   = 0x05,
            STATE_EXECUTE   = 0x08,
            STATE_DATA      = 0x10
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
        static const size_t READ_BUFFER_SIZE = 4096;
        RingBuffer<READ_BUFFER_SIZE>* sdSpiReadBuffer;

        File sdCard;
        bool cardSelected;
        bool isSdIdle;
        uint8_t currentState;
        uint8_t currentCommand;
        bool commandAppCmd;
        uint32_t commandArgument;
        bool dataActive;
        uint32_t dataRegister;
        uint32_t dataIndex;
        uint8_t dataBuffer[514];

        uint32_t sectorOffset;
        uint32_t currentSector;
        uint32_t numSectors;
        bool hasValidPath;
        char sdCardPath[256];

        inline __attribute__((always_inline)) void writeReadData(uint8_t data)
        {
            sdSpiReadBuffer->write(data);
        }

        inline __attribute__((always_inline)) void writeStatusData()
        {
            writeReadData(isSdIdle ? 0x01 : 0x00);
        }

        inline __attribute__((always_inline)) void openSdCard()
        {
            if (!sdCard)
            {
                sdCard = SD.open(sdCardPath, FILE_WRITE_BEGIN);
#ifdef DEBUG_HDF_OUTPUT
                if (!sdCard)
                {
                    Serial.printf("fail\n");
                } else {
                    Serial.printf("open\n");
                }
#endif
            }
#ifdef DEBUG_HDF_OUTPUT
            else {
                Serial.printf("ready\n");
            }
#endif
        }

        bool processData(bool hasData, uint8_t data)
        {
            if ((currentCommand == CMD_WRITE_BLOCK) && hasData)
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

                        // Write the data
                        sdCard.write(dataBuffer, 512);
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

        void collectResponse()
        {
            //
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
                            if (sdCard)
                            {
                                writeStatusData();
                                writeReadData(0xFF);
                                if (commandArgument != currentSector)
                                {
                                    uint32_t offset = (commandArgument * 512) + sectorOffset;
                                    sdCard.seek(offset, SeekSet);
                                    currentSector = commandArgument;
                                }
                                sdCard.read(dataBuffer, 512);
                                ++currentSector;
                                writeReadData(0xFE);
                                sdSpiReadBuffer->writeBlock(dataBuffer, 512);

                                // Generate the two CRC bytes
                                writeReadData(0x00);
                                writeReadData(0x00);
                            } else {
                                // TODO: Error
                            }
                        }
                        break;
                    case CMD_WRITE_BLOCK :
                        {
                            openSdCard();
                            if (sdCard)
                            {
                                writeStatusData();
                                writeReadData(0xFF);
                                if (commandArgument != currentSector)
                                {
                                    uint32_t offset = (commandArgument * 512) + sectorOffset;
                                    sdCard.seek(offset, SeekSet);
                                    currentSector = commandArgument;
                                }
                                currentState = STATE_DATA;
                                return;
                            } else {
                                // TODO: Error
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
        constexpr SdHdfZXTeensy() : sdSpiReadBuffer(0), sdCard(), cardSelected(false),
            isSdIdle(true), currentState(STATE_IDLE),
            currentCommand(CMD_IDLE), commandAppCmd(false), commandArgument(0),
            dataActive(false), dataRegister(0), dataIndex(0), dataBuffer(),
            sectorOffset(0), currentSector(0), numSectors(0), hasValidPath(false),
            sdCardPath()
        {
            memset(sdCardPath, 0, 256);
        }

        void begin(RingBuffer<READ_BUFFER_SIZE>* readBuffer, const char* filename);
        void end(void);

        inline __attribute__((always_inline)) void performTick(bool hasData, uint8_t data)
        {
            if (hasData || (currentState & (STATE_EXECUTE | STATE_DATA)))
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
                    case STATE_EXECUTE :
                        collectResponse();
                        break;
                    case STATE_DATA :
                        processData(hasData, data);
                        break;
                }
            }
        }
};

#endif
