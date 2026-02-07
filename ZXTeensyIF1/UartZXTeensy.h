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

#define UART_CLOCK 24000000

class UartZXTeensy
{
    public :
        typedef enum {
            UART_WRITE,
            UART_SET_BAUD
        } uart_action_t;

    protected :
        static const size_t UART_RX_BUFFER_SIZE = 2048;
        static const size_t UART_TX_BUFFER_SIZE = 8;
        RingBuffer<UART_RX_BUFFER_SIZE> uartReadBuffer;
        RingBuffer<UART_TX_BUFFER_SIZE> uartWriteBuffer;
        RingBuffer<UART_TX_BUFFER_SIZE> uartFlagsBuffer;
        RingBuffer<UART_TX_BUFFER_SIZE> uartTxDataBuffer;
        bool enabled;
        bool isTransmitting;

        constexpr int calculateBestDiv(const uint32_t baud, int& bestosr)
        {
            float base = (float)UART_CLOCK / (float)baud;
            float besterr = 1e20;
            int bestdiv = 1;
            bestosr = 4;
            for (int osr=4; osr <= 32; osr++) {
                float div = base / (float)osr;
                int divint = (int)(div + 0.5f);
                if (divint < 1) divint = 1;
                else if (divint > 8191) divint = 8191;
                float err = ((float)divint - div) / div;
                if (err < 0.0f) err = -err;
                if (err <= besterr) {
                    besterr = err;
                    bestdiv = divint;
                    bestosr = osr;
                }
            }
            return bestdiv;
        }

        inline __attribute__((always_inline)) uart_action_t readWriteData(uint8_t* data)
        {
            uart_action_t action = (uart_action_t)uartFlagsBuffer.readRaw();
            *data = uartWriteBuffer.readRaw();
            return action;
        }

    public :
        constexpr UartZXTeensy() : enabled(false), isTransmitting(false)
        {
        }

        void begin(uint8_t baud);

        void end(void);

        inline __attribute__((always_inline)) void isrUartEvent()
        {
            // See if we have stuff to read in.
            // Todo - Check idle.
            IMXRT_LPUART_t *port = (IMXRT_LPUART_t *)IMXRT_LPUART5_ADDRESS;
            if (port->STAT & (LPUART_STAT_RDRF | LPUART_STAT_IDLE))
            {
                // See how many bytes or pending
                uint8_t avail = (port->WATER >> 24) & 0x7;
                while (avail > 0)
                {
                    uint8_t data = port->DATA;
                    uartReadBuffer.write(data);
                    --avail;
                }

                // If it was an idle status clear the idle
                if (port->STAT & LPUART_STAT_IDLE)
                {
                    port->STAT |= LPUART_STAT_IDLE;    // writing a 1 to idle should clear it.
                }
            }

            // See if we are transmitting and room in buffer.
            uint32_t ctrl = port->CTRL;
            if ((ctrl & LPUART_CTRL_TIE) && (port->STAT & LPUART_STAT_TDRE))
            {
                if (uartTxDataBuffer.canRead())
                {
                    isTransmitting = true;
                    do {
                        port->DATA = uartTxDataBuffer.readRaw();
                    } while (uartTxDataBuffer.canRead() && (((port->WATER >> 8) & 0x7) < 4));
                }
                if (!uartTxDataBuffer.canRead())
                {
                    port->CTRL &= ~LPUART_CTRL_TIE;
                    port->CTRL |= LPUART_CTRL_TCIE; // Actually wondering if we can just leave this one on...
                }
            }

            if ((ctrl & LPUART_CTRL_TCIE) && (port->STAT & LPUART_STAT_TC))
            {
                isTransmitting = false;
                port->CTRL &= ~LPUART_CTRL_TCIE;
            }
        }

        inline __attribute__((always_inline)) void sendData(uint8_t data)
        {
            IMXRT_LPUART_t *port = (IMXRT_LPUART_t *)IMXRT_LPUART5_ADDRESS;
            isTransmitting = true;
            uartTxDataBuffer.write(data);
            __disable_irq();
            port->CTRL |= LPUART_CTRL_TIE;
            __enable_irq();
        }

        inline __attribute__((always_inline)) uint8_t readData()
        {
            return uartReadBuffer.readRaw();
        }

        inline __attribute__((always_inline)) bool hasReadData()
        {
            return uartReadBuffer.canRead();
        }

        inline __attribute__((always_inline)) bool hasWriteData()
        {
            return uartWriteBuffer.canRead();
        }

        inline __attribute__((always_inline)) void writeData(uart_action_t action, uint8_t data)
        {
            uartFlagsBuffer.write((uint8_t)action);
            uartWriteBuffer.write(data);
        }

        inline __attribute__((always_inline)) uint8_t getStatusByte()
        {
            uint16_t count = uartReadBuffer.getSize();
            uint8_t status = (count != 0) ? 0x01 : 0x00;
            if (hasWriteData())
            {
                status |= 0x02;
            }
            if (count >= (UART_RX_BUFFER_SIZE - 1))
            {
                status |= 0x1C;
            } else if (count >= (UART_RX_BUFFER_SIZE / 2))
            {
                status |= 0x18;
            } else if (count >= 256)
            {
                status |= 0x08;
            }
            return status;
        }

        inline __attribute__((always_inline)) void onTick()
        {
            if (enabled && hasWriteData() && !uartTxDataBuffer.canRead())
            {
                uint8_t data;
                switch (readWriteData(&data))
                {
                    case UART_SET_BAUD :
                        end();
                        begin(data);
                        break;
                    case UART_WRITE :
                        sendData(data);
                        break;
                }
            }
        }
};

#endif
