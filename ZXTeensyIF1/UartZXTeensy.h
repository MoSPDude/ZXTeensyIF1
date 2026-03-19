//
// Modified from the Teensyduino Core Library
// Fixed to UART5, 8N1 format, and integrated RingBuffer for ZXTeensyIF1
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

#ifndef UART_ZX_TEENSY_H
#define UART_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "RingBuffer.h"
#include "usb_serial.h"
#include <HardwareSerial.h>

#define UART_CLOCK 24000000

class UartZXTeensy
{
    public :
        typedef enum {
            UART_WRITE,
            UART_SET_BAUD
        } uart_action_t;

    protected :
        // NOTE: TX_BUFFER_SIZE needs to be large enough for EspNtpZXTeensy
        static const size_t RX_BUFFER_SIZE = 64;
        static const size_t TX_BUFFER_SIZE = 64;
        RingBuffer<RX_BUFFER_SIZE> uartReadBuffer;
        RingBuffer<TX_BUFFER_SIZE> uartWriteBuffer;
        RingBuffer<TX_BUFFER_SIZE> uartFlagsBuffer;
        bool enabled;
        bool isPassthrough;

        inline __attribute__((always_inline)) uart_action_t readWriteData(uint8_t* data)
        {
            uart_action_t action = (uart_action_t)uartFlagsBuffer.readRaw();
            *data = uartWriteBuffer.readRaw();
            return action;
        }

        inline __attribute__((always_inline)) bool hasWriteData()
        {
            return uartWriteBuffer.canRead();
        }

    public :
        constexpr UartZXTeensy() : enabled(false), isPassthrough(false)
        {
        }

        // Open Serial8, and start the port
        void begin(uint8_t baud, const char* modemUrl);

        // Flush and close the port
        void end(void);

        // Wait for response
        bool espWaitFor(const char *token, uint32_t timeout);

        inline __attribute__((always_inline)) uint8_t readData()
        {
            return uartReadBuffer.readRaw();
        }

        inline __attribute__((always_inline)) bool hasReadData()
        {
            return uartReadBuffer.canRead();
        }

        inline __attribute__((always_inline)) void writeData(uart_action_t action, uint8_t data)
        {
            uartFlagsBuffer.write((uint8_t)action);
            uartWriteBuffer.write(data);
        }

        inline uint8_t getUartStatusByte() __attribute__((always_inline, hot, optimize("O3")))
        {
            size_t count = uartReadBuffer.getSize();
            uint8_t status = (count != 0) ? 0x01 : 0x00;
            if (hasWriteData())
            {
                status |= 0x02;
            }
            if (count >= (RX_BUFFER_SIZE - 1))
            {
                status |= 0x1C;
            } else if (count >= (RX_BUFFER_SIZE / 2))
            {
                status |= 0x18;
            } else if (count >= 256)
            {
                status |= 0x08;
            }
            return status;
        }

        inline uint8_t getModemStatusByte() __attribute__((always_inline, hot, optimize("O3")))
        {
            uint8_t status = (hasReadData() ? 0x82 : 0x80);
            if (!hasWriteData())
            {
                status |= 0x05;
            }
            return status;
        }

        // NOTE: onTick is main loop, so optimize
        inline void onTick() __attribute__((always_inline, hot, optimize("O3")))
        {
            if (enabled)
            {
                if (hasWriteData() && Serial8.availableForWrite())
                {
                    uint8_t data;
                    switch (readWriteData(&data))
                    {
                        case UART_SET_BAUD :
                            end();
                            begin(data, 0);
                            break;
                        case UART_WRITE :
                            Serial8.write(data);
                            break;
                    }
                }
                while (Serial8.available() && uartReadBuffer.canWrite())
                {
                    uartReadBuffer.write(Serial8.read());
                }
            }
        }

        inline __attribute__((always_inline)) void flush()
        {
            uartReadBuffer.clear();
            uartWriteBuffer.clear();
            uartFlagsBuffer.clear();
        }
};

#endif
