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

#ifndef SD_SDHC_ZX_TEENSY_H
#define SD_SDHC_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "RingBuffer.h"
#include "usb_serial.h"
#include <SdFat.h>

//define DEBUG_SDHC_OUTPUT

#define FIFO_WML 16

#define MAKE_REG_MASK(m,s) (((uint32_t)(((uint32_t)(m) << s))))
#define MAKE_REG_SET(x,m,s) (((uint32_t)(((uint32_t)(x) & m) << s)))
#define SDHC_XFERTYP_CMDINX(n)  MAKE_REG_SET(n,0x3F,24) //(uint32_t)(((n) & 0x3F)<<24)// Command Index
#define SDHC_XFERTYP_RSPTYP(n)  MAKE_REG_SET(n,0x3,16) //(uint32_t)(((n) & 0x3)<<16)  // Response Type Select
#define SDHC_XFERTYP_DPSEL          0x00200000
#define SDHC_XFERTYP_CICEN          0x00100000
#define SDHC_XFERTYP_CCCEN          0x00080000

#define SDHC_SYSCTL_INITA           0x08000000

#define SDHC_IRQSTAT_CMD_ERROR      0x11300000
#define SDHC_IRQSTAT_DATA_ERROR     0x000f0000
#define SDHC_IRQSTAT_DINT           0x00000004
#define SDHC_IRQSTAT_CC             0x00000001
#define SDHC_IRQSTAT_TC             0x00000002

#define SDHC_PRSSTAT_SDSTB          0x00000008
#define SDHC_PRSSTAT_BREN           0x00000800
#define SDHC_PRSSTAT_BWEN           0x00000400

#define SDHC_PROCTL_CREQ            0x00020000
#define SDHC_PROCTL_SABGREQ         0x00010000

class SdSdhcZXTeensy
{
    public :
        typedef enum {
            SD_SPI_WRITE,
            SD_SPI_READ,
            SD_SPI_ENABLE,
            SD_SPI_DISABLE
        } sd_spi_action_t;

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
        static const size_t READ_BUFFER_SIZE = 1024;
        static const size_t WRITE_BUFFER_SIZE = 8;

        const uint32_t CMD_RESP_NONE = SDHC_XFERTYP_RSPTYP(0);

        const uint32_t CMD_RESP_R1 = SDHC_XFERTYP_CICEN | SDHC_XFERTYP_CCCEN |
                                     SDHC_XFERTYP_RSPTYP(2);

        const uint32_t CMD_RESP_R1b = SDHC_XFERTYP_CICEN | SDHC_XFERTYP_CCCEN |
                                      SDHC_XFERTYP_RSPTYP(3);

        const uint32_t CMD_RESP_R2 = SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(1);

        const uint32_t CMD_RESP_R3 = SDHC_XFERTYP_RSPTYP(2);

        const uint32_t CMD_RESP_R6 = CMD_RESP_R1;

        const uint32_t CMD_RESP_R7 = CMD_RESP_R1;

        RingBuffer<READ_BUFFER_SIZE> sdSpiReadBuffer;
        RingBuffer<WRITE_BUFFER_SIZE> sdSpiWriteBuffer;
        RingBuffer<WRITE_BUFFER_SIZE> sdSpiFlagsBuffer;

        SdCard* sdCard;
        bool cardSelected;
        bool isActive;
        bool isSdIdle;
        uint8_t currentState;
        uint8_t currentCommand;
        bool commandAppCmd;
        uint32_t commandArgument;
        bool dataActive;
        uint32_t dataRegister;
        uint32_t dataIndex;
        uint8_t dmaBuffer[512];

        inline __attribute__((always_inline)) void writeReadData(uint8_t data)
        {
#ifdef DEBUG_SDHC_OUTPUT
            Serial.println(data, HEX);
#endif
            sdSpiReadBuffer.write(data);
        }

        inline __attribute__((always_inline)) void writeStatusData()
        {
            writeReadData(isSdIdle ? 0x01 : 0x00);
        }

        inline __attribute__((always_inline)) sd_spi_action_t readWriteData(uint8_t* data)
        {
            sd_spi_action_t spiAction = (sd_spi_action_t)sdSpiFlagsBuffer.readRaw();
            *data = sdSpiWriteBuffer.readRaw();
            return spiAction;
        }

        inline __attribute__((always_inline)) bool hasWriteData()
        {
            return sdSpiWriteBuffer.canRead();
        }

        bool processData(bool hasData, uint8_t data)
        {
            if (currentCommand != CMD_WRITE_BLOCK)
            {
                dataRegister = USDHC1_INT_STATUS;
                USDHC1_INT_STATUS = dataRegister;
                if (dataRegister & SDHC_IRQSTAT_TC)
                {
                    writeReadData(0xFE);
                    sdSpiReadBuffer.writeBlock(dmaBuffer, 512);

                    // Generate the two CRC bytes
                    writeReadData(0x00);
                    writeReadData(0x00);
                    currentState = STATE_IDLE;
                    dataActive = false;
                    return true;
                }
            } else if (hasData)
            {
                if (dataActive)
                {
                    if (dataIndex >= 0x202)
                    {
                        dataRegister = USDHC1_INT_STATUS;
                        USDHC1_INT_STATUS = dataRegister;
                        if (dataRegister & SDHC_IRQSTAT_TC)
                        {
                            currentState = STATE_IDLE;
                            dataActive = false;
                            return true;
                        } else {
                            // Send busy response
                            writeReadData(0x00);
                        }
                    } else {
                        switch (dataIndex & 0x00000003)
                        {
                            case 0 :
                                dataRegister = data;
                                break;
                            case 1 :
                                dataRegister |= (data << 8);
                                break;
                            case 2 :
                                dataRegister |= (data << 16);
                                break;
                            case 3 :
                                dataRegister |= (data << 24);
                                break;
                        }
                        ++dataIndex;
                        if (dataIndex <= 0x200)
                        {
                            if ((dataIndex & 0x00000003) == 0)
                            {
                                USDHC1_DATA_BUFF_ACC_PORT = dataRegister;
                            }
                        } else if (dataIndex >= 0x202)
                        {
                            // Consume the CRC bytes, then send data response
                            writeReadData(0x05);
                        }
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
            uint32_t status = USDHC1_INT_STATUS;
            if (status & (SDHC_IRQSTAT_CC | SDHC_IRQSTAT_CMD_ERROR))
            {
                USDHC1_INT_STATUS = status;
                if (status & SDHC_IRQSTAT_CMD_ERROR)
                {
                    // TODO: Command error
#ifdef DEBUG_SDHC_OUTPUT
                    Serial.println(0xFF, HEX);
#endif
                    writeReadData(isSdIdle ? 0x7F : 0x7E);
                    currentState = STATE_IDLE;
                } else {
#ifdef DEBUG_SDHC_OUTPUT
                    Serial.println(USDHC1_CMD_RSP0, HEX);
                    Serial.println(USDHC1_CMD_RSP1, HEX);
                    Serial.println(USDHC1_CMD_RSP2, HEX);
                    Serial.println(USDHC1_CMD_RSP3, HEX);
#endif
                    switch (currentCommand)
                    {
                        case CMD_READ_SINGLE_BLOCK :
                        case CMD_WRITE_BLOCK :
                            USDHC1_PROT_CTRL &= ~SDHC_PROCTL_SABGREQ;
                            USDHC1_PROT_CTRL |= SDHC_PROCTL_CREQ;
                            USDHC1_PROT_CTRL |= SDHC_PROCTL_SABGREQ;
                            writeStatusData();
                            writeReadData(0xFF);
                            currentState = STATE_DATA;
#ifdef DEBUG_SDHC_OUTPUT
                            Serial.println("RR");
#endif
                            break;
                        default :
                            writeStatusData();
                            currentState = STATE_IDLE;
                            break;
                    }
                }
            }
        }

        void executeCommand()
        {

#ifdef DEBUG_SDHC_OUTPUT
            Serial.println(currentCommand, HEX);
            Serial.println(commandArgument, HEX);
#endif
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
                            // Send real command to the SDHC controller
                            uint32_t xfertype = SDHC_XFERTYP_CMDINX(CMD_READ_SINGLE_BLOCK) |
                                SDHC_XFERTYP_DPSEL | CMD_RESP_R1;
                            USDHC1_DS_ADDR  = (uint32_t)dmaBuffer;
                            USDHC1_BLK_ATT = 0x00010200;
                            USDHC1_MIX_CTRL &= 0xFFFFFF00;
                            USDHC1_MIX_CTRL |= 0x00000011;
                            USDHC1_PROT_CTRL |= SDHC_PROCTL_SABGREQ;
                            USDHC1_CMD_ARG = commandArgument;
                            USDHC1_CMD_XFR_TYP = xfertype;
                            currentState = STATE_EXECUTE;
                        }
                        return;
                    case CMD_WRITE_BLOCK :
                        {
                            // Send real command to the SDHC controller
                            uint32_t xfertype = SDHC_XFERTYP_CMDINX(CMD_WRITE_BLOCK) |
                                SDHC_XFERTYP_DPSEL | CMD_RESP_R1;
                            USDHC1_BLK_ATT = 0x00010200;
                            USDHC1_MIX_CTRL &= 0xFFFFFF00;
                            USDHC1_PROT_CTRL &= ~SDHC_PROCTL_SABGREQ;
                            USDHC1_CMD_ARG = commandArgument;
                            USDHC1_CMD_XFR_TYP = xfertype;
                            currentState = STATE_EXECUTE;
                        }
                        return;
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
                                sdCard->readCSD((csd_t*)cid);
                            } else {
                                sdCard->readCID((cid_t*)cid);
                            }
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
                            uint32_t ocr;
                            sdCard->readOCR(&ocr);
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

        inline __attribute__((always_inline)) void performTick(bool hasData, uint8_t data)
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

    public :
        constexpr SdSdhcZXTeensy() : sdCard(0), cardSelected(false),
            isActive(false), isSdIdle(true), currentState(STATE_IDLE),
            currentCommand(CMD_IDLE), commandAppCmd(false), commandArgument(0),
            dataActive(false), dataRegister(0), dataIndex(0), dmaBuffer()
        {
        }

        void begin(SdCard* card);
        void end(void);

        inline __attribute__((always_inline)) uint8_t readData()
        {
            return sdSpiReadBuffer.readRaw();
        }

        inline __attribute__((always_inline)) bool hasReadData()
        {
            return sdSpiReadBuffer.canRead();
        }

        inline __attribute__((always_inline)) void writeData(sd_spi_action_t spiAction, uint8_t data)
        {
            sdSpiFlagsBuffer.write((uint8_t)spiAction);
            sdSpiWriteBuffer.write(data);
        }

        inline __attribute__((always_inline)) void flush()
        {
            sdSpiReadBuffer.clear();
            sdSpiWriteBuffer.clear();
            sdSpiFlagsBuffer.clear();
        }

        inline __attribute__((always_inline)) void onTick()
        {
            if (hasWriteData())
            {
                uint8_t data;
                switch (readWriteData(&data))
                {
                    case SD_SPI_ENABLE :
                        cardSelected = true;
                        break;
                    case SD_SPI_DISABLE :
                        cardSelected = false;
                        break;
                    default :
                        if (cardSelected)
                        {
                            performTick(true, data);
                        }
                        break;
                }
            } else if (currentState & (STATE_EXECUTE | STATE_DATA))
            {
                performTick(false, 0);
            }
        }
};

#endif
