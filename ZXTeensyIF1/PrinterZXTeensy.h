
#ifndef PRINTER_ZX_TEENSY_H
#define PRINTER_ZX_TEENSY_H

#include "RingBuffer.h"
#include "DefinesZXTeensy.h"
#include <SD.h>
#include <SdFat.h>
#include <TimeLib.h>

class PrinterZXTeensy
{
    protected :
        static const size_t PRINTER_BUFFER_SIZE = 64;
        static const size_t PRINTER_PATH_SIZE = 40;
        static const uint16_t PRINTER_PATH_SUFFIX_MAX = 99;
        RingBuffer<PRINTER_BUFFER_SIZE> printerBuffer;
        bool enabled;
        uint32_t printDelayCount;
        File outFile;
        char outPath[PRINTER_PATH_SIZE];

        inline void resetOutputPath()
        {
            outPath[0] = 0;
        }

        inline bool buildOutputPath()
        {
            time_t timeNow = now();
            int yrs = year(timeNow);
            int mnts = month(timeNow);
            int days = day(timeNow);
            int hrs = hour(timeNow);
            int mins = minute(timeNow);
            int secs = second(timeNow);
            size_t pathLength = snprintf(outPath, PRINTER_PATH_SIZE, 
                "/printer-%04d%02d%02d-%02d%02d%02d.txt",
                yrs, mnts, days, hrs, mins, secs);
            if ((pathLength < 4) || (pathLength >= PRINTER_PATH_SIZE))
            {
                resetOutputPath();
                return false;
            }
            size_t extensionIndex = (size_t)(pathLength - 4);
            char* extension = &(outPath[extensionIndex]);
            if (!SD.exists(outPath))
            {
                return true;
            }
            extensionIndex = (PRINTER_PATH_SIZE - extensionIndex);
            for (uint16_t suffix = 1; suffix <= PRINTER_PATH_SUFFIX_MAX; ++suffix)
            {
                if (snprintf(extension, extensionIndex, "-%02u.txt", suffix) >= (int)extensionIndex)
                {
                    resetOutputPath();
                    return false;
                }
                if (!SD.exists(outPath))
                {
                    return true;
                }
            }
            resetOutputPath();
            return false;
        }

        inline bool openPrinter()
        {
            if (!outFile)
            {
                if (outPath[0] == 0 && !buildOutputPath())
                {
                    enabled = false;
                    return false;
                }
                outFile = SD.open(outPath, FILE_WRITE);
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
            resetOutputPath();
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
            resetOutputPath();
        }

        inline void clearOutput()
        {
            if (outFile)
            {
                outFile.close();
            }
            if (outPath[0] != 0)
            {
                File clearFile = SD.open(outPath, FILE_WRITE_BEGIN);
                if (clearFile)
                {
                    clearFile.seek(0, SeekSet);
                    clearFile.truncate();
                    clearFile.close();
                }
            }
            enabled = false;
            printDelayCount = 0;
            printerBuffer.clear();
            resetOutputPath();
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
                printDelayCount = TRIGGER_DELAY_MS;
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
