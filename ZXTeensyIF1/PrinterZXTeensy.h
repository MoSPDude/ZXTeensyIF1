
#ifndef PRINTER_ZX_TEENSY_H
#define PRINTER_ZX_TEENSY_H

#include "RingBuffer.h"
#include "DefinesZXTeensy.h"
#include <SD.h>
#include <SdFat.h>

class PrinterZXTeensy
{
    protected :
        static const size_t PRINTER_BUFFER_SIZE = 32;
        RingBuffer<PRINTER_BUFFER_SIZE> printerBuffer;
        bool enabled;
        uint32_t printDelayCount;
        File outFile;
        
        inline bool openPrinter()
        {
            if (!outFile)
            {
                outFile = SD.open(PRINTER_OUT_PATH, FILE_WRITE);
                if (outFile)
                {
                    outFile.seek(0, SeekEnd);
                    enabled = true;
                } else {
                    enabled = false;
                    return false;
                }
            }
            return true;
        }

    public :
        PrinterZXTeensy() : enabled(false), printDelayCount(0)
        {
        }

        inline __attribute__((always_inline)) void begin()
        {
            //
        }

        inline void end()
        {
            onTick();
            if (outFile)
            {
                outFile.close();
            }
            enabled = false;
            printDelayCount = 0;
            printerBuffer.clear();
        }

        inline __attribute__((always_inline)) void writeData(uint8_t data)
        {
            printerBuffer.write(data);
        }

        inline __attribute__((always_inline)) bool getBusy()
        {
            return !printerBuffer.canWrite();
        }

        // NOTE: onTick is main loop, so optimize
        inline void onTick() __attribute__((always_inline, hot, optimize("O3")))
        {
            if (printerBuffer.canRead())
            {
                if (!enabled && !openPrinter())
                {
                    printerBuffer.clear();
                    return;
                }
                do {
                    outFile.write(printerBuffer.readRaw());
                } while (printerBuffer.canRead());
                printDelayCount = TRIGGER_DELAY_CNT;
            } else if (printDelayCount > 0)
            {
                // Delay to close and commit the file to SD
                if (--printDelayCount == 0)
                {
                    outFile.close();
                    enabled = false;
                }
            }
        }
};

#endif
