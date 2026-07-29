
#ifndef DSK_765_ZX_TEENSY_H
#define DSK_765_ZX_TEENSY_H

#include "imxrt.h"
#include "core_pins.h"
#include "RingBuffer.h"
#include "usb_serial.h"
#include <SD.h>
#include <SdFat.h>

extern "C" {
    #include "lib765/765i.h"
}

class Dsk765ZXTeensy
{
    public :
        typedef enum {
            WRITE_DATA,
            WRITE_MOTOR
        } flags_t;

    static const uint8_t LED_PIN = 13;

    protected :
        static const size_t BUFFER_SIZE = 32;
        static const size_t READ_BUFFER_SIZE = 32;
        RingBuffer<BUFFER_SIZE> wrFlagBuffer;
        RingBuffer<BUFFER_SIZE> writeBuffer;
        RingBuffer<READ_BUFFER_SIZE> rdStatusBuffer;
        RingBuffer<READ_BUFFER_SIZE> readBuffer;
        volatile uint8_t readDataStatus;
        volatile bool motorOn;
        volatile uint8_t statusRegister;
        FDC_PTR fdc;
        FDRV_PTR fdd_a;
        FDRV_PTR fdd_b;
        bool hasDriveB;

    public :
        constexpr Dsk765ZXTeensy() : readDataStatus(0xFF), motorOn(false),
            statusRegister(0), fdc(0), fdd_a(0), fdd_b(0), hasDriveB(false)
        {
        }

        void begin(const char* diskAPath, bool enableDriveB, const char* diskBPath)
        {
            // Ensure controller is reset
            end();

            // Create the FDC, and attach drives with disks
            fdc = fdc_new();
            fdd_a = fd_newdsk();
            fd_settype(fdd_a, FD_30);
            fd_setheads(fdd_a, 1);
            fd_setcyls(fdd_a, 42);
            if (diskAPath != 0)
            {
                fdd_setfilename(fdd_a, diskAPath);
            } else {
                fd_eject(fdd_a);
            }
            if (enableDriveB)
            {
                hasDriveB = true;
                fdd_b = fd_newdsk();
                fd_settype(fdd_b, FD_30);
                fd_setheads(fdd_b, 1);
                fd_setcyls(fdd_b, 42);
                if (diskBPath != 0)
                {
                    fdd_setfilename(fdd_b, diskBPath);
                } else {
                    fd_eject(fdd_b);
                }
            } else {
                fdd_b = fd_new();
            }
            fdc_reset(fdc);
            fdc_setisr(fdc, 0);
            fdc_setdrive(fdc, 0, fdd_a);
            fdc_setdrive(fdc, 1, fdd_b);
            fdc_setdrive(fdc, 2, fdd_a);
            fdc_setdrive(fdc, 3, fdd_b);
        }

        void end()
        {
            readDataStatus = 0xFF;
            motorOn = false;
            statusRegister = 0;
            wrFlagBuffer.clear();
            writeBuffer.clear();
            rdStatusBuffer.clear();
            readBuffer.clear();
            if (fdc != 0)
            {
                hasDriveB = false;
                fdc_destroy(&fdc);
                fd_destroy(&fdd_a);
                fd_destroy(&fdd_b);
                fdc = 0;
            }
        }

        void insertDisks(const char* diskAPath, const char* diskBPath)
        {
            if (diskAPath == 0)
            {
                fd_eject(fdd_a);
            } else if (stricmp(diskAPath, fdd_getfilename(fdd_a)) != 0)
            {
                fdd_setfilename(fdd_a, diskAPath);
            }
            if (hasDriveB)
            {
                if (diskBPath == 0)
                {
                    fd_eject(fdd_b);
                } else if (stricmp(diskBPath, fdd_getfilename(fdd_b)) != 0)
                {
                    fdd_setfilename(fdd_b, diskBPath);
                }
            }
        }

        // NOTE: onTick is main loop, so optimize
        inline void onTick() __attribute__((always_inline, hot, optimize("O3")))
        {
            if (fdc != 0)
            {
                // Send write data, or motor control
                if (wrFlagBuffer.canRead())
                {
                    uint8_t data = writeBuffer.readRaw();
                    switch (wrFlagBuffer.readRaw())
                    {
                        case WRITE_DATA :
                            statusRegister &= 0x7F;
                            fdc_write_data(fdc, data);
                            break;
                        case WRITE_MOTOR :
                            fdc_set_motor(fdc, data);

                            // Flush writes to the disk image when motor stops
                            if (!motorOn)
                            {
                                // NOTE: Modified fdd_dirty function to flush
                                // the file, and clear dirty flag
                                fd_dirty(fdd_a);
                                fd_dirty(fdd_b);
                            }
                            break;
                        default :
                            break;
                    }
                }

                // Fetch data from the FDC into the read buffer
                if (!rdStatusBuffer.canRead())
                {
                    statusRegister = fdc_read_ctrl(fdc);
                    if ((statusRegister & 0xC0) == 0xC0)
                    {
                        readBuffer.write(fdc_read_data(fdc));
                        rdStatusBuffer.write(fdc_read_ctrl(fdc));
                    } else {
                        readDataStatus = statusRegister;
                    }
                }
            }
        }

        inline __attribute__((always_inline)) void writeData(flags_t flag, uint8_t data)
        {
            writeBuffer.write(data);
            wrFlagBuffer.write(flag);
        }

        inline __attribute__((always_inline)) void setMotor(bool motor)
        {
            if (motorOn != motor)
            {
                writeData(WRITE_MOTOR, (motor ? 0x0F : 0x00));
                motorOn = motor;
                digitalWriteFast(LED_PIN, !motorOn);
            }
        }

        inline __attribute__((always_inline)) bool isMotorOn()
        {
            return motorOn;
        }

        inline __attribute__((always_inline)) int getCurCyl(bool driveB)
        {
            return (driveB ? fd_getcurcyl(fdd_b) : fd_getcurcyl(fdd_a));
        }

        inline __attribute__((always_inline)) uint8_t readData()
        {
            uint8_t data;
            if (rdStatusBuffer.canRead())
            {
                data = readBuffer.readRaw();
                statusRegister = rdStatusBuffer.readRaw();
            } else {
                // Unexpected read of FDC
                data = readDataStatus;
            }
            return data;
        }

        inline __attribute__((always_inline)) uint8_t getStatusByte()
        {
            return statusRegister;
        }

};

#endif
