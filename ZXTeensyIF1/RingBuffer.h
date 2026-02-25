
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <cstddef>
#include <string.h>

template <size_t BUFFER_SIZE> class RingBuffer
{
    protected :
        volatile uint8_t buffer[BUFFER_SIZE];
        volatile uint16_t bufferHead;
        volatile uint16_t bufferTail;

    public :
        constexpr RingBuffer() : bufferHead(0), bufferTail(0)
        {
        }

        inline __attribute__((always_inline)) void clear()
        {
            bufferTail = bufferHead;
        }

        inline __attribute__((always_inline)) uint16_t getSize()
        {
            uint16_t head = bufferHead;
            uint16_t tail = bufferTail;
            if (head >= tail)
            {
                return (head - tail);
            }
            return (BUFFER_SIZE - tail - head);
        }

        inline __attribute__((always_inline)) void write(uint8_t data)
        {
            buffer[bufferHead] = data;
            bufferHead = (bufferHead + 1) % BUFFER_SIZE;
        }

        inline __attribute__((always_inline)) void writeBlock(uint8_t* data, size_t size)
        {
            if (size >= BUFFER_SIZE)
            {
                size = BUFFER_SIZE - 1;
            }
            uint16_t nextHead = (bufferHead + size) % BUFFER_SIZE;
            if (nextHead < bufferHead)
            {
                size_t partSize = (BUFFER_SIZE - bufferHead);
                memcpy((void*)&(buffer[bufferHead]), data, partSize);
                memcpy((void*)buffer, &(data[partSize]), (size - partSize));
            } else {
                memcpy((void*)&buffer[bufferHead], data, size);
            }
            bufferHead = nextHead;
        }

        inline __attribute__((always_inline)) uint8_t readRaw()
        {
            uint8_t data = buffer[bufferTail];
            bufferTail = (bufferTail + 1) % BUFFER_SIZE;
            return data;
        }

        inline __attribute__((always_inline)) bool canRead()
        {
            return (bufferHead != bufferTail);
        }

        inline __attribute__((always_inline)) bool canWrite()
        {
            uint8_t head = (bufferHead + 1) % BUFFER_SIZE;
            return (head != bufferTail);
        }

        inline bool read(uint8_t* data)
        {
            if (canRead())
            {
                *data = readRaw();
                return true;
            }
            return false;
        }
};

#endif
