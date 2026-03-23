
#ifndef UART_ZX_TEENSY_H
#define UART_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "RingBuffer.h"
#include "usb_serial.h"
#include <HardwareSerial.h>
#include "DefinesZXTeensy.h"

class UartZXTeensy
{
    public :
        typedef enum {
            UART_WRITE,
            UART_SET_BAUD
        } uart_action_t;

    protected :
        // NOTE: TX_BUFFER_SIZE needs to be large enough for EspNtpZXTeensy
        static const size_t TX_BUFFER_SIZE = 64;
        RingBuffer<TX_BUFFER_SIZE> uartWriteBuffer;
        RingBuffer<TX_BUFFER_SIZE> uartFlagsBuffer;

        // Occasionally, receive 2 x MTU into buffer, so size over (2 * 1500)
        static const size_t RX_BUFFER_SIZE = 3072;
        RingBuffer<RX_BUFFER_SIZE> uartReadBuffer __attribute__((aligned(16)));

        bool enabled;
        bool isModemPassthrough;
        volatile size_t readBytesAvailable;
        volatile uint32_t readByteCycle;

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
        constexpr UartZXTeensy() : enabled(false), isModemPassthrough(false),
            readBytesAvailable(0), readByteCycle(0)
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
            uint8_t status = (hasReadData() ? 0x01 : 0x00);
            if (hasWriteData())
            {
                status |= 0x02;
            }
            if (readBytesAvailable >= 2048)
            {
                status |= 0x1C;
            } else if (readBytesAvailable >= 1024)
            {
                status |= 0x18;
            } else if (readBytesAvailable >= 256)
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
                if (hasWriteData())
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
                size_t count = Serial8.available();
                if (isModemPassthrough)
                {
                    // Restrict data rate to byte per MODEM_DELAY_CNT
                    if ((ARM_DWT_CYCCNT - readByteCycle) >= MODEM_DELAY_CNT)
                    {
                        readByteCycle = ARM_DWT_CYCCNT;
                        if (count > 0)
                        {
                            uartReadBuffer.write(Serial8.read());
                            --count;
                        }
                    }
                } else {
                    while ((count > 0) && uartReadBuffer.canWrite())
                    {
                        uartReadBuffer.write(Serial8.read());
                        --count;
                    }
                }
                readBytesAvailable = count;
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
