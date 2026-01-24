
# ZXTeensyIF1

![image](./Images/pcb-v0.2.JPG "Prototype PCB")

A Teensy 4.1 powered DivMMC and ZX Interface 1 clone,

* ZX Interface 1
    * Requires the 9V and 5V power rails
    * Uses a MAX232 for the RS232 level shifting, so no +12V or -12V required
* DivMMC with 512KB RAM
    * Shares the Teensy main SD card
* Multiface 128 emulation
    * NMI button and soft ROM
* ZX Interface 2, and ZXC2 ROM emulation
    * Implements ZXC2 ROM banking
* Soft ROM emulation
    * Override the internal Spectrum ROM with ROMs from SD card
    * Supports 16KB (48K Spectrum), 32KB (128K Spectrum) and 64KB (+2A/+3 Spectrum) ROMs
        * The +2A/+3 soft ROM support requires banking changes (see below)
    * Provides the Interface 1 ROM, Multiface 128 ROM and DivMMC ROM
* External ROM support
    * ZX Interface 1 edge connector supports other ROM based hardware
    * eg. Retroleum diagnostic card, real ZX Interface 2 hardware etc.

The ZX Interface 1 v2 ROM is included in compiled firmware, and as noted on other
sources "Amstrad have kindly given their permission for the redistribution
of their copyrighted material but retain that copyright"

This is a project that I started with a view of wanting a DivMMC clone that works with a
ZX Interface 1 attached ZX Max 128 issue 3 - as I own a ZX Microdrive as well as a ZXPicoMD
(https://github.com/TomDDG/ZXPicoMD) ... and didn't want to keep disconnecting the ZX
Interface 1.

It borrows content, ideas and inspriation from,

* https://github.com/SensoriumEmbedded/TeensyROM
    * Initial code and parts for the Teensy 4.1 firmware
* https://github.com/liveboxandy/ZX-Interface-1-Recreated
    * Re-used the ZX Interface 1 board layout and schematic
* https://github.com/ZXSpectrumVault/rom-disassemblies
    * ZX Interface 1 v2 ROM disassembly
* https://spectrumcomputing.co.uk/pub/sinclair/technical-docs/ZXInterface1_Schematics.gif
    * ZX Interface 1 schematic
* https://github.com/TomDDG/ZXPicoIF2Lite
    * ROM menu ideas, and the idea of a soft ROM
* https://www.thingiverse.com/thing:6500064
    * Also from TomDDG, a replacement ZX Interface 1 case

## Current Status

First v0.2 PCBs have come back from PCBWay, and are being tested with my 48K Spectrum, and my
ZX Max 128 Issue 3. Some parts have come from a donor ZX Interface 1 that needed a new old-stock
LA15-312 ULA from eBay.

Removing the ZX Interface 1 edge connector that goes to the ZX Spectrum was an immense pain - so
might have to find other ideas. Even with gentle heat, I managed to deform and melt the plastic
riser block...

At the moment, the firmware looks for fixed ROM filenames on the SD card to load - it'd be nice to
create a menu like the ZXPicoIF2Lite has.

Otherwise, the soft ROM functions correctly - banking the DivMMC, Multiface 128, Interface 1 or
Spectrum soft ROMs as required. eg. when testing the external ROM support, the Retroleum card 
correctly sees the 128K soft ROM loaded on to my 48K Spectrum.

When the DivMMC is enabled, restarting the machine with ".128" will disable the DivMMC and enable
the Interface 1.

## Version History

### Hardware

* v0.3 PCB prototype
    * HAS NOT BEEN MANUFACTURED YET
    * Minor footprint fixes, and routing tweaks
    * No electrical or schematic changes
* v0.2 PCB prototype
    * First PCBs made, and tested
        * Microdrive, RS232, ZX Net and nROMCS on external edge connector working
        * Firmware updated for new pin layout - soft ROM all working
    * Need to revise some footprints
    * My PCBs had silkscreen that stated v0.1
* v0.1 veroboard prototype (not uploaded)
    * Teensy 4.1 and level shifter on Veroboard
        * Soft ROM and Multiface 128 behaviour working
    * Modded ZX Interface 1 to add "IORQ inhibit" (see below)

### Firmware

* First upload
    * Rough and ready for v0.2 PCB
    * Address lines now contiguous and in order, on GPIO6
    * Data lines in order, but not contiguous, on GPIO7
* Earlier prototypes (not uploaded)
    * Very rough and ready
    * Address and data organised to suit the veroboard

The file "if1-2_rom.h" contains the Sinclair ZX Interface 1 v2 (if1-2.rom) ROM in
hexadecimal format, as embedded in the compiled firmware.

## Building the firmware

* Setup the Arduino IDE 2.3.6 for the Teensy 4.1
* Modify "%LOCALAPPDATA%\Arduino15\packages\teensy\hardware\avr\1.59.0\libraries\SdFat\src\SdFatConfig.h",
  to define SPI_DRIVER_SELECT as 2
* Open and build ZXTeensyIF1.ino

## Loading the KiCad project

The PCB folder contains a KiCad 9.0.2 project, based on liveboxandy "ZX Interface 1 Recreated" KiCad
project. Without that project, I'd have had to spend considerable time getting the board outline and
aligning the sockets etc.

It uses,

* https://github.com/XenGi/teensy_library as teensy_library-master
* https://github.com/XenGi/teensy.pretty as teensy.pretty-master
    * Set XGENGI_TEENSY_LIBRARY path in Symbol Libraries etc.
* https://github.com/sparkfun/SparkFun-KiCad-Libraries
    * Set SPARKFUN_KICAD_LIBRARY path in Symbol Libraries etc.
* https://github.com/nosuz/kicad-symbols-footprints
    * Place as ./nosuz-kicad-symbols-footprints in the project directory

## DivMMC and ZX Interface 1 support

Technically, the DivMMC and ZX Interface 1 cannot be active simultaneously. The DivMMC ports and
ZX Interface 1 IO ports clash directly. (More info at
https://worldofspectrum.org/faq/reference/48kreference.htm#PortF7 )

To overcome this, the Teensy 4.1 drives the nIORQ of the Interface 1 ULA high when the DivMMC
is active.

### Prototyping notes

The veroboard prototype used the edge connector A4 (as N/C on 48k spectrums) to signal back into
the ZX Interface 1,

* Disconnect the base of Q11 from the Interface 1 ULA IC1 pin 10
    * It will probably be soldered directly onto the ULA pin!
* Cut the nIORQ trace near the Interface 1 ULA IC1 pin 10 - be very careful!
* Add a 1N4148 diode from edge connector A4 with cathode to IC1 pin 10
* Add a 1N4148 diode from edge connector A17 with cathode to IC1 pin 10
* Add a 6.8K resistor from IC1 pin 10 to ground (eg. IC1 pin 20)
* Connect the base of Q11 back to the edge connector A17 with wire

## ZX Max 128 Issue 3 +2A/+3 ROM banking

The ZX Max 128 Issue 3 needs a modification to support the +2A/+3 soft ROM.

Without it, accesses to the Secondary Memory Control register (0x1FFD) also affect the original
Memory Control register (0x7FFD) due to the partial decoding. (More info at
https://worldofspectrum.org/faq/reference/128kreference.htm )

The modification is shown at
https://github.com/DonSuperfo/ZX-Max-128/blob/main/Issue%203/Modify%20for%20%2B3%20ROM.pdf ,

* Replace R12 with a 1N4148 diode, with cathode to U9 pin 4
* Add a 1N4148 diode from U6 pin 12 with cathode to U9 pin 4
* Add a 10K resistor from U9 pin 4 to ground (eg. U8 pin 24)
