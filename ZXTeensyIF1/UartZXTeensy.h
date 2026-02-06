
#ifndef UART_ZX_TEENSY_H
#define UART_ZX_TEENSY_H

#include "RingBuffer.h"
#include "HardwareSerialPublic.h"

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
        bool enabled;
        
        bool canReadHardwareSerial(HardwareSerialIMXRT* serial)
        {
            // NOTE: HACK to see if data in software buffer without disabling IRQs to
            // check the hardware buffer - gain public access to private class members
            HardwareSerialIMXRTPublic* hwSerial8 = (HardwareSerialIMXRTPublic*)serial;
            uint32_t head, tail;
            head = hwSerial8->rx_buffer_head_;
            tail = hwSerial8->rx_buffer_tail_;
            return (head != tail);
        }

        inline __attribute__((always_inline)) void writeReadData(uint8_t data)
        {
            uartReadBuffer.write(data);
        }

        inline __attribute__((always_inline)) uart_action_t readWriteData(uint8_t* data)
        {
            uart_action_t action = (uart_action_t)uartFlagsBuffer.readRaw();
            *data = uartWriteBuffer.readRaw();
            return action;
        }
        
        inline __attribute__((always_inline)) void performTick()
        {
            if (hasWriteData() && Serial8.availableForWrite())
            {
                uint8_t data;
                switch (readWriteData(&data))
                {
                    case UART_SET_BAUD :
                        int baud;
                        Serial8.end();
                        switch (data)
                        {
                            case 1 :
                                baud = 57600;
                                break;
                            case 2 :
                                baud = 38400;
                                break;
                            case 3 :
                                baud = 31250;
                                break;
                            case 4 :
                                baud = 19200;
                                break;
                            case 5 :
                                baud = 9600;
                                break;
                            case 6 :
                                baud = 4800;
                                break;
                            case 7 : 
                                baud = 2400;
                                break;
                            default :
                                baud = 115200;
                                break;
                        }
                        Serial8.begin(baud);
                        break;
                    case UART_WRITE :
                        Serial8.write(data);
                        break;
                }
            }

            if (!hasReadData() && canReadHardwareSerial(&Serial8))
            {
                uint8_t data = Serial8.read();
                writeReadData(data);
            }
        }
        
    public :
        constexpr UartZXTeensy() : enabled(false)
        {
        }
        
        inline __attribute__((always_inline)) void begin()
        {
            if (!enabled)
            {
                Serial8.begin(115200);
                enabled = true;
            }
        }
        
        inline __attribute__((always_inline)) void end()
        {
            if (enabled)
            {
                Serial8.end();
                uartReadBuffer.clear();
                uartWriteBuffer.clear();
                uartFlagsBuffer.clear();
                enabled = false;
            }
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

        inline __attribute__((always_inline)) void onTick()
        {
            if (enabled)
            {
                performTick();
            }
        }
};

#endif
