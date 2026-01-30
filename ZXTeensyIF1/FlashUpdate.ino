
// Adapted from:

//******************************************************************************
// FlasherX -- firmware OTA update via Intel Hex file over serial or SD stream
// https://github.com/joepasquariello/FlasherX
//******************************************************************************
//
// Based on Flasher3 (Teensy 3.x) and Flasher4 (Teensy 4.x) by Jon Zeeff
//
// Jon Zeeff 2016, 2019, 2020 This code is in the public domain.
// Please retain my name in distributed copies, and let me know about any bugs
//
// I, Jon Zeeff, give no warranty, expressed or implied for this software and/or
// documentation provided, including, without limitation, warranty of
// merchantability and fitness for a particular purpose.
//
// WARNING: You can brick your Teensy with incorrect flash erase/write, such as
// incorrect flash config (0x400-40F). This code may or may not prevent that.

// 10/09/22 (v2.3) JWP - option for reading hex file from serial or SD
//   - move hex file support functions to new file FXUtil.cpp
//   - update_firmware() now takes two Stream* arguments ("in" and "out")
//   - FlasherX.ino lets user choose between hex file via serial or SD
// 09/01/22 (v2.2) JWP - change FlashTxx from CPP to C file
//   - rename FlashTxx.cpp to FlashTxx.c (resolve link error when calling from C)
//   - FlasherX.ino place #include "FlashTxx.h" inside extern "C" block
// 01/07/22 (v2.1) JWP - use TD 1.56 core functions for T4x wait/write/erase
//   - FlashTxx.h update FLASH_SIZE for Teensy Micromod from 8 to 16 MB
//   - option to artificially increase code size via const array (in flash)
// 11/18/21 JWP - bug fix in file FlashTXX.cpp
//   - fix logic in while loop in flash_block_write() in FlashTXX
// 10/27/21 JWP - add support for Teensy Micromod
//   - define macros for TEENSY_MICROMOD w/ same values as for TEENSY40
//   - update FLASH_SIZE for T4.1 and TMM from 2MB to 8MB
// JWP - merge of Flasher3/4 and new features
//   - FLASH buffer dynamically sized from top of existing code to FLASH_RESERVE
//   - optional RAM buffer option for T4.x via macro RAM_BUFFER_SIZE > 0
//   - Stream* (USB or UART) and buffer addr/size set at run-time
//   - incorporate Frank Boesing's FlashKinetis routines for T3.x
//   - add support for Teensy 4.1 and Teensy LC
//    This code is released into the public domain.
// JWP - Joe Pasquariello - modifications for T3.5 and T3.6 in Dec 2020
//    This code is released into the public domain
// Deb Hollenback at GiftCoder -- Modifications for teensy 3.5/3/6
//    This code is released into the public domain.
//    see https://forum.pjrc.com/threads/43165-Over-the-Air-firmware-updates-changes-for-flashing-Teensy-3-5-amp-3-6
// Jon Zeeff modifications
//    see https://forum.pjrc.com/threads/29607-Over-the-air-updates
// Original by Niels A. Moseley, 2015.
//    This code is released into the public domain.
//    https://namoseley.wordpress.com/2015/02/04/freescale-kinetis-mk20dx-series-flash-erasing/

#include "Flash/FXUtil.h"   // read_ascii_line(), hex file support
#include "Flash/FXUtil.cpp"
extern "C" {
  #include "Flash/FlashTxx.h"   // TLC/T3x/T4x/TMM flash primitives
  #include "Flash/FlashTxx.c"
}

void flashUpdate(const char *fwFileName)
{
    // Detach all interrupts
    detachInterrupt(digitalPinToInterrupt(RD_PIN));
    detachInterrupt(digitalPinToInterrupt(WR_PIN));
    detachInterrupt(digitalPinToInterrupt(ROMCS_IN_PIN));
    detachInterrupt(digitalPinToInterrupt(RESET_IN_PIN));
    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));

    // Create flash buffer to hold new firmware
    uint32_t buffer_addr, buffer_size;
    if (firmware_buffer_init( &buffer_addr, &buffer_size ) == FLASH_BUFFER_TYPE) 
    {
       // read hex file, write new firmware to flash, clean up, reboot
       File hexFile = SD.open(fwFileName, FILE_READ);
       update_firmware(&hexFile, buffer_addr, buffer_size);

       // return from update_firmware() means error or user abort, so clean up
       firmware_buffer_free(buffer_addr, buffer_size);
    }
}
