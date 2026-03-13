
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
            WRITE_MOTOR,
            WRITE_EOE,
            WRITE_EOR
        } flags_t;

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
        FDRV_PTR fdd_x;

    public :
        constexpr Dsk765ZXTeensy() : readDataStatus(0xFF), motorOn(false), 
            statusRegister(0), fdc(0), fdd_a(0), fdd_b(0), fdd_x(0)
        {
        }

        void begin(const char* diskAPath, const char* diskBPath)
        {
            end();

            //
            fdc = fdc_new();
            fdd_x = fd_new();
            if (diskAPath != 0)
            {
                fdd_a = fd_newdsk();
                fd_settype(fdd_a, FD_30);
                fd_setheads(fdd_a, 1);
                fd_setcyls(fdd_a, 40);
                fdd_setfilename(fdd_a, diskAPath);
            } else {
                fdd_a = fd_new();
            }
            if (diskBPath != 0)
            {
                fdd_b = fd_new();
                /*fdd_b = fd_newdsk();
                fd_settype(fdd_b, FD_30);
                fd_setheads(fdd_b, 1);
                fd_setcyls(fdd_b, 40);
                fdd_setfilename(fdd_b, diskBPath);*/
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
            if (fdc != 0)
            {
                fdc_destroy(&fdc);
                fd_destroy(&fdd_a);
                fd_destroy(&fdd_b);
                fd_destroy(&fdd_x);
                fdc = 0;
            }
        }

        inline __attribute__((always_inline)) void onTick()
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
            }
        }

        inline __attribute__((always_inline)) uint8_t readData()
        {
            uint8_t data;
            if (rdStatusBuffer.canRead())
            {
                data = readBuffer.readRaw();
                statusRegister = rdStatusBuffer.readRaw();
            } else {
                // Unexpected read from FDC
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
