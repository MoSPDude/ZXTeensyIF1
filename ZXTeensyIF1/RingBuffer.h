
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <cstddef>
#include <string.h>

template <size_t BUFFER_SIZE> class RingBuffer
{
    protected :
        volatile uint8_t buffer[BUFFER_SIZE];
        volatile size_t bufferHead;
        volatile size_t bufferTail;

    public :
        constexpr RingBuffer() : bufferHead(0), bufferTail(0)
        {
        }

        inline __attribute__((always_inline)) void clear()
        {
            bufferTail = bufferHead;
        }

        inline __attribute__((always_inline)) size_t getSize()
        {
            size_t head = bufferHead;
            size_t tail = bufferTail;
            if (head >= tail)
            {
                return (head - tail);
            }
            return (BUFFER_SIZE + head - tail);
        }

        inline __attribute__((always_inline)) size_t getFree()
        {
            return (BUFFER_SIZE - 1 - getSize());
        }

        inline __attribute__((always_inline)) void write(uint8_t data)
        {
            buffer[bufferHead] = data;
            bufferHead = (bufferHead + 1) % BUFFER_SIZE;
        }

        inline __attribute__((always_inline)) void writeBlock(uint8_t* data, size_t size)
        {
            size_t head = (bufferHead + size) % BUFFER_SIZE;
            if (head < bufferHead)
            {
                size_t partSize = (BUFFER_SIZE - bufferHead);
                memcpy((void*)&(buffer[bufferHead]), data, partSize);
                memcpy((void*)buffer, &(data[partSize]), (size - partSize));
            } else {
                memcpy((void*)&(buffer[bufferHead]), data, size);
            }
            bufferHead = head;
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
            size_t head = (bufferHead + 1) % BUFFER_SIZE;
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
