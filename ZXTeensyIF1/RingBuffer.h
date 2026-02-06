
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

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

        inline __attribute__((always_inline)) void write(uint8_t data)
        {
            buffer[bufferHead] = data;
            bufferHead = (bufferHead + 1) % BUFFER_SIZE;;
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
