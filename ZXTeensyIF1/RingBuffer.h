
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

        template <size_t BLOCK_SIZE>
        inline __attribute__((always_inline)) void writeFixedBlock(
            const uint8_t (&data)[BLOCK_SIZE])
        {
            static_assert((BLOCK_SIZE > 0) && (BLOCK_SIZE <= BUFFER_SIZE),
                "Fixed block size must fit within the ring buffer");
            size_t head = bufferHead;
            for (size_t i = 0; i < BLOCK_SIZE; ++i)
            {
                buffer[head] = data[i];
                if (++head == BUFFER_SIZE)
                {
                    head = 0;
                }
            }
            bufferHead = head;
        }

        inline __attribute__((always_inline)) void writeBlockWithToken(uint8_t token,
            uint8_t* data, size_t size)
        {
            size_t tmpHead = (bufferHead + 1) % BUFFER_SIZE;
            size_t head = (tmpHead + size) % BUFFER_SIZE;
            buffer[bufferHead] = token;
            if (head < tmpHead)
            {
                size_t partSize = (BUFFER_SIZE - tmpHead);
                memcpy((void*)&(buffer[tmpHead]), data, partSize);
                memcpy((void*)buffer, &(data[partSize]), (size - partSize));
            } else {
                memcpy((void*)&(buffer[tmpHead]), data, size);
            }
            bufferHead = head;
        }

        inline __attribute__((always_inline)) uint8_t readRaw()
        {
            size_t tail = bufferTail;
            uint8_t data = buffer[tail];
            if (++tail == BUFFER_SIZE)
            {
                tail = 0;
            }
            bufferTail = tail;
            return data;
        }

        inline __attribute__((always_inline)) uint8_t peekRaw(size_t offset)
        {
            size_t index = bufferTail + offset;
            while (index >= BUFFER_SIZE)
            {
                index -= BUFFER_SIZE;
            }
            return buffer[index];
        }

        template <size_t BLOCK_SIZE>
        inline __attribute__((always_inline)) void readFixedBlock(
            uint8_t (&data)[BLOCK_SIZE])
        {
            peekFixedBlock(data);
            discardFixedBlock<BLOCK_SIZE>();
        }

        template <size_t BLOCK_SIZE>
        inline __attribute__((always_inline)) void peekFixedBlock(
            uint8_t (&data)[BLOCK_SIZE])
        {
            static_assert((BLOCK_SIZE > 0) && (BLOCK_SIZE <= BUFFER_SIZE),
                "Fixed block size must fit within the ring buffer");
            size_t tail = bufferTail;
            for (size_t i = 0; i < BLOCK_SIZE; ++i)
            {
                data[i] = buffer[tail];
                if (++tail == BUFFER_SIZE)
                {
                    tail = 0;
                }
            }
        }

        template <size_t BLOCK_SIZE>
        inline __attribute__((always_inline)) void discardFixedBlock()
        {
            static_assert((BLOCK_SIZE > 0) && (BLOCK_SIZE <= BUFFER_SIZE),
                "Fixed block size must fit within the ring buffer");
            size_t tail = bufferTail + BLOCK_SIZE;
            if (tail >= BUFFER_SIZE)
            {
                tail -= BUFFER_SIZE;
            }
            bufferTail = tail;
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
