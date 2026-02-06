
#ifndef SD_SPI_ZX_TEENSY_H
#define SD_SPI_ZX_TEENSY_H

#include <SPI.h>
#include "RingBuffer.h"

extern volatile uint32_t globalCycleCount;

class SdSpiZXTeensy : public SdSpiSoftDriver
{
    public :
        typedef enum {
            SD_SPI_WRITE,
            SD_SPI_READ,
            SD_SPI_ENABLE,
            SD_SPI_DISABLE
        } sd_spi_action_t;

    protected :
        static const uint8_t SPI_BUFFER_SIZE = 8;
        const uint32_t FAST_TICK_CYCCNT;

        volatile uint8_t sdSpiTxData;
        volatile uint8_t sdSpiRxData;
        volatile bool sdSpiRxNotTx;
        volatile uint8_t sdSpiTick;
        volatile uint8_t sdSpiCount;
        RingBuffer<SPI_BUFFER_SIZE> sdSpiReadBuffer;
        RingBuffer<SPI_BUFFER_SIZE> sdSpiWriteBuffer;
        RingBuffer<SPI_BUFFER_SIZE> sdSpiFlagsBuffer;

        inline __attribute__((always_inline)) void writeReadData(uint8_t data)
        {
            sdSpiReadBuffer.write(data);
        }

        inline __attribute__((always_inline)) sd_spi_action_t readWriteData()
        {
            sd_spi_action_t spiAction = (sd_spi_action_t)sdSpiFlagsBuffer.readRaw();
            sdSpiRxNotTx = (spiAction == SD_SPI_READ);
            sdSpiTxData = sdSpiWriteBuffer.readRaw();
            return spiAction;
        }

        inline __attribute__((always_inline)) bool hasWriteData()
        {
            return sdSpiWriteBuffer.canRead();
        }

        inline __attribute__((always_inline)) void performTick()
        {
            // Perform SD SPI accesses on clock edges
            if (sdSpiTick == 0x00)
            {
                // SPI is idle, so wait for write data
                if (hasWriteData())
                {
                    switch (readWriteData())
                    {
                        case SD_SPI_ENABLE :
                            CORE_PIN46_PORTCLEAR = CORE_PIN46_BITMASK;
                            break;
                        case SD_SPI_DISABLE :
                            CORE_PIN46_PORTSET = CORE_PIN46_BITMASK;
                            break;
                        default :
                            // Drive new SPI access
                            if (sdSpiTxData & 0x80)
                            {
                                CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                            } else {
                                CORE_PIN45_PORTCLEAR = CORE_PIN45_BITMASK;
                            }
                            sdSpiTxData <<= 1;
                            sdSpiTick = 1;
                            break;
                    }
                }
            } else if (sdSpiTick & 0x0f)
            {
                // SPI is being driven, so toggle clock and data
                if (sdSpiTick & 0x01)
                {
                    // Capture read data on rising edges
                    if (sdSpiRxNotTx)
                    {
                        if (CORE_PIN43_PINREG & CORE_PIN43_BITMASK)
                        {
                            sdSpiRxData = (sdSpiRxData << 1) | 0x01;
                        } else {
                            sdSpiRxData <<= 1;
                        }
                        if (sdSpiTick == 0x0f)
                        {
                            // Write read data into buffer
                            writeReadData(sdSpiRxData);
                        }
                    }
                } else if (!sdSpiRxNotTx)
                {
                    // Drive write data on falling edges
                    if (sdSpiTxData & 0x80)
                    {
                        CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                    } else {
                        CORE_PIN45_PORTCLEAR = CORE_PIN45_BITMASK;
                    }
                    sdSpiTxData <<= 1;
                }
                CORE_PIN44_PORTTOGGLE = CORE_PIN44_BITMASK;
                ++sdSpiTick;
            } else {
                // SPI access has just finished, so test for write data
                // before falling idle, otherwise continue
                ++sdSpiCount;
                if (hasWriteData())
                {
                    switch (readWriteData())
                    {
                        case SD_SPI_ENABLE :
                            CORE_PIN46_PORTCLEAR = CORE_PIN46_BITMASK;
                            CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                            sdSpiTick = 0;
                            break;
                        case SD_SPI_DISABLE :
                            CORE_PIN46_PORTSET = CORE_PIN46_BITMASK;
                            CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                            sdSpiTick = 0;
                            break;
                        default :
                            // Drive new SPI access on this falling edge
                            if (sdSpiTxData & 0x80)
                            {
                                CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                            } else {
                                CORE_PIN45_PORTCLEAR = CORE_PIN45_BITMASK;
                            }
                            sdSpiTxData <<= 1;
                            sdSpiTick = 1;
                            break;
                    }
                } else {
                    CORE_PIN45_PORTSET = CORE_PIN45_BITMASK;
                    sdSpiTick = 0;
                }
                CORE_PIN44_PORTCLEAR = CORE_PIN44_BITMASK;
            }
        }

        inline __attribute__((always_inline)) void waitFastTick()
        {
            uint32_t cycle_ = ARM_DWT_CYCCNT;
            if ((cycle_ - globalCycleCount) >= FAST_TICK_CYCCNT)
            {
                globalCycleCount = cycle_;
                performTick();
            }
        }

        inline __attribute__((always_inline)) uint8_t readDataSync()
        {
            writeData(SD_SPI_READ, 0xff);
            while (!hasReadData()) {
                waitFastTick();
            };
            return readData();
        }

        inline __attribute__((always_inline)) void writeDataSync(uint8_t data)
        {
            uint8_t count = sdSpiCount;
            writeData(SD_SPI_WRITE, data);
            while (count == sdSpiCount) {
                waitFastTick();
            };
        }

    public :
        constexpr SdSpiZXTeensy(uint32_t FAST_TICK_CYCCNT_) : FAST_TICK_CYCCNT(FAST_TICK_CYCCNT_),
            sdSpiTxData(0xFF), sdSpiRxData(0xFF), sdSpiRxNotTx(0), sdSpiTick(0), sdSpiCount(0)
        {
        }

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
            while ((sdSpiTick != 0) || hasWriteData())
            {
                waitFastTick();
            }
            sdSpiReadBuffer.clear();
            sdSpiWriteBuffer.clear();
            sdSpiFlagsBuffer.clear();
        }

        inline __attribute__((always_inline)) void onTick()
        {
            if ((sdSpiTick != 0) || hasWriteData())
            {
                performTick();
            }
        }

        /** Initialize the SPI bus. */
        void begin()
        {
            // Do nothing
        }

        /** Receive a byte.
        *
        * \return The byte.
        */
        uint8_t receive()
        {
            return readDataSync();
        }

        /** Send a byte.
        *
        * \param[in] data Byte to send
        */
        void send(uint8_t data)
        {
            writeDataSync(data);
        }
};

#endif