
# ZXTeensyIF1

![image](./Images/pcb-v0.2.JPG "Prototype PCB")

A Teensy 4.1 powered DivMMC and ZX Interface 1 clone for the ZX Spectrum 48K/128K/+2 (Grey) machines,

* ZX Interface 1
    * Requires the 9V and 5V power rails
    * Uses a MAX232 for the RS232 level shifting, so no +12V or -12V required
    * Supports 16KB shadow soft ROM
* DivMMC with 512KB RAM
    * Shares the Teensy main SD card
    * Supports HDF and IMG images
* Multiface 128 emulation
    * NMI button and soft ROM
* ZX Interface 2 emulation
* ZXC2 and ZXC3 with Flash ROM cartridge emulation
    * Implements ZXC2 ROM banking
    * Implements ZXC3 ROM banking with 128KB Flash ROM
    * Implements SPECTRA shadow ROM support
* ESP-01S module via TX8/RX8
    * Requires the 9V power rail for the 3.3V regulator
    * Available on ports 0x143B (5179) for RX and 0x133B (4923) for TX
    * https://www.specnext.com/the-next-on-the-network/
* Kempston USB mouse and gamepad
    * Not wired to the board - use a small lead that exits the case
    * eg. StarTech.com "6in USB 2.0 Cable - USB A Female to USB Motherboard 4 Pin Header F/F"
* RTC module from the Teensy
    * Accessed as a RTC-72421 on ports 0x7X3B
        * The time is only read or written when register 0xD sets HOLD to 1
    * Uses a minor bugfixed RTC.SYS from https://velesoft.speccy.cz/zx/rtcmodule/index.htm
        * Patched byte 6 from 0x0D to 0x7D
    * Not wired to the board - attach a coin cell battery to VBAT
        * See https://www.pjrc.com/store/teensy41.html#timing
* Single speed TZX and TAP playback
    * Playback locks the keyboard, as it overdrives the ULA
    * TAP loading is better done via the DivMMC
* Spectrum +3 uPD765a FDC emulation
    * Integrated lib765 v0.4.2 with additional bug fixes for "EXTENDED" disk loading
    * Requires Spectrum +2A/+3 soft ROM support (see below), or other compatible DOS
* Prism VTX5000 over ESP-01S
    * Creates a serial tunnel over WiFi to given URL
    * Default URL is '"glasstty.com",6502' - https://glasstty.com/telstar/
* Soft ROM emulation
    * Override the internal Spectrum ROM with ROMs from SD card
    * Supports 16KB (Spectrum 48K), 32KB (Spectrum 128K/+2 (Grey)) and 64KB (Spectrum +2A/+3) ROMs
        * The +2A/+3 soft ROM support requires port decoding changes (see below)
    * Provides the Interface 1 ROM, Multiface 128 ROM and DivMMC ROM
* Menu ROM derived from TomDDGs ZXPicoIF2Lite ROMExplorer
* Z80 snapshot loading from TomDDGs ZXPicoIF2Lite
    * Integrated "z80torom" for loading 'z80' and 'sna' files
* MDR microdrive loading with Paul Farrows SPECTRA Microdrive Emulator
    * http://www.fruitcake.plus.com/Sinclair/Interface2/Cartridges/Interface2_RC_New_Microdrive_Emulator.htm
    * Supports MDR images with up to 90KB of data
* Small HTTP server for WiFi file access over ESP-01S
    * Use HTTP PUT to send files eg. "curl -T FILENAME.ROM http://192.168.0.254/FILENAME.ROM"
    * Use web browser to list file, and download
* Nihirash's Network Manager for WiFi configuration
    * Small bugfix to clear the BASIC keypress on load
* Velesoft's RTC_SETUP for RTC configuration
    * Minor tweaks to preset the RTC port
* External ROM support
    * ZX Interface 1 edge connector supports other ROM based hardware
    * eg. Retroleum SMART card, real ZX Interface 2 hardware etc.

The ZX Interface 1 v2 ROM is included in compiled firmware, and as noted on other
sources "Amstrad have kindly given their permission for the redistribution
of their copyrighted material but retain that copyright"

This is a project that I started with a view of wanting a DivMMC clone that works with a
ZX Interface 1 attached to my ZX Max 128 - as I own a ZX Microdrive as well as a ZXPicoMD
(https://github.com/TomDDG/ZXPicoMD) ... and didn't want to keep disconnecting the ZX
Interface 1.

## Credits

Full credits to the following projects - this project gathers and integrates from
many various sources.

It borrows content, ideas and inspiration from,

* http://www.pjrc.com/teensy/
    * Teensyduino Core and SdFat libraries - provides all the Teensy 4.1 functionality
    * Reused parts of code for the RTC, SDHC and UART drivers
* https://github.com/SensoriumEmbedded/TeensyROM
    * Initial code and parts for the Teensy 4.1 firmware
* https://github.com/TomDDG/ZXPicoIF2Lite
    * ROM menu source code, Z80 snapshot loader, and the idea of a soft ROM
* http://www.fruitcake.plus.com/Sinclair/Interface2/Cartridges/Interface2_RC_Cartridges.htm
    * ZXC2 and ZXC3 banking, and Microdrive Emulator
* https://github.com/nihirash/netman-zx
    * WiFi Network Manager for the ESP-01S
* https://velesoft.speccy.cz/zx/rtcmodule/index.htm
    * RTC_SETUP for the RTC module
    * RTC.SYS for EsxDOS
* https://www.seasip.info/Unix/LibDsk/
    * lib765 FDC controller library
* https://github.com/joepasquariello/FlasherX
    * Teensy 4.x OTA upgrade library
* https://github.com/liveboxandy/ZX-Interface-1-Recreated
    * Re-used the ZX Interface 1 board layout and schematic
* https://github.com/ZXSpectrumVault/rom-disassemblies
    * ZX Interface 1 v2 ROM disassembly
* https://spectrumcomputing.co.uk/pub/sinclair/technical-docs/ZXInterface1_Schematics.gif
    * ZX Interface 1 schematic
* https://divide.speccy.cz/files/pgm_model.txt
    * DivIDE programming model
* https://www.thingiverse.com/thing:6500064
    * Also from TomDDG, a replacement ZX Interface 1 case

Without the above projects, this would not have been possible!

## Current Status

Updated v0.7 PCBs have come back from PCBWay, and now being tested.

The first v0.2 PCBs had come back from PCBWay, and been tested with my 48K Spectrum, and my
ZX Max 128 Issue 3. Some parts have come from a donor ZX Interface 1 that needed a new old-stock
LA15-312 ULA from eBay.

Removing the ZX Interface 1 edge connector that goes to the ZX Spectrum was an immense pain - so
might have to find other ideas. Even with gentle heat, I managed to deform and melt the plastic
riser block...

Otherwise, the soft ROM functions correctly - banking the DivMMC, Multiface 128, Interface 1 or
Spectrum soft ROMs as required. eg. when testing the external ROM support, the Retroleum SMART card
(https://blog.retroleum.co.uk/smart-card-for-zx-spectrum/) diagnostics correctly sees the 128K soft
ROM loaded on to my 48K Spectrum.

When the DivMMC is enabled, restarting the machine with ".128" (even on 48K Spectrums) will disable
the DivMMC and enable the Interface 1.

## SD Card Setup

* ROOT/
    * ZXTEENSY/
        * CONFIGS/
            * <NAME>.CFG (Configurations that appear as "NAME" in the Menu ROM for quick selection)
        * MENU.ROM
        * MF128.ROM (MD5SUM: ca8c9d97c8aedd718d1081fad2e3af8d)
        * ESXMMC.BIN (MD5SUM: fa50b0258e52b8d72bd83cc2fb6e1013)
        * SPECTRA_IF1_ED2_ME_ROM_Formatted.bin (MD5SUM: 052ad91ee822604960e8ca8d32a3ddb9)
        * IF1.ROM (Optional, MD5SUM: 31b704ae925305e74f50699271fddd9a)
        * VTX.ROM (MD5SUM: 12a62cb7ea7383f109c2711dfca99f5e)
        * netman.z80 (WiFi Network Manager snapshot)
        * rtc_setup.z80 (RTC Setup snapshot)
        * ZXTEENSY.CFG (Current configuration from Menu ROM)
    * ROMS/
        * (ZX Spectrum ROMs ending ".rom")
        * (Interface 2 and ZXC2 ROMs ending ".bin")
    * (Other ESXDOS files)
    * SYS/
        * RTC.SYS (Optional, for RTC access)
        * (Other ESXDOS files)
    * ZXTEENSY.HEX (Optional, firmware update)

To load the Menu ROM, either,
    * Use "Boot into Menu" to load on initial power on
    * or, Hold the button, and Reset
    * or, Hold Reset for longer than 5 seconds

Inside the Menu ROM, you can toggle the available Devices and settings - or choose
a ROM to load, or browse the SD card to load and mount files.

To firmware update, place the ZXTEENSY.HEX file on the root of the SD card and
select the option from the Menu ROM - then wait for the Spectrum to restart
(!! It will take a minute !!).

### Preparing the SD Card

ESXDOS has trouble loading if it is not "early" on the SD card,

* Format the SD card with FAT32
* Extract ESXDOS on computer, and transfer all directories and ESXMMC.BIN to the SD card
* Transfer the MF128.ROM, MENU.ROM and ZXTEENSY.CFG
* Create ROMS directory, and add other ROMs
* Add any other files

## Version History

### Hardware

* v0.7 PCB prototype
    * PCBs have been sent for manufacturing
    * Moved the ROMCS and DataDir output to pins 36 and 37, to free up pins 34 and 35
    * Added an ESP-01S header, and header for 3.3V regulator (eg. Pololu D24V5F3)
        * The ESP-01S can take over 300mA, so requires a separate regulator
        * The Pololu D24V5F3 is a 3.3V 500mA regulator module, available from The Pi Hut
        * I had to re-organise the left side of the board to make room
    * Connected RX8 pin 34 and TX8 pin 35, to the ESP-01S header
* v0.2 PCB prototype
    * First PCBs made, and tested
        * Microdrive, RS232, ZX Net and nROMCS on external edge connector working
        * Firmware updated for new pin layout - soft ROM all working
    * Need to revise some footprints
    * The PCBs had silkscreen that stated v0.1
* v0.1 veroboard prototype (not uploaded)
    * Prototype to test the initial idea
    * Teensy 4.1 and level shifters on Veroboard
        * Soft ROM and Multiface 128 behaviour working
    * Modded ZX Interface 1 to add "nIORQ inhibit" (see below)

### Firmware

* 20260316
    * Added lib765 and +3 DSK support
* 20260312
    * Updated for v0.7 PCB
    * Added ZXC3 and MDR emulator support

## Building the firmware

* Setup the Arduino IDE 2.3.6 for the Teensy 4.1
    * "Teensy (for Arduino IDE 2.0.4 or later)" v1.60.0
* Open ZXTeensyIF1\ZXTeensyIF1.ino
* Set Board to Teensy 4.1
* Set Optimize to "Smallest Code with LTO"
* Set CPU Speed to "816 MHz (overclock)"
* Verify and Upload

## Loading the KiCad project

The PCB folder contains a KiCad 9.0.2 project, based on liveboxandy "ZX Interface 1 Recreated" KiCad
7 project. Without that project, I'd have had to spend considerable time getting the board outline
correct, and aligning the sockets etc.

It uses,

* https://github.com/XenGi/teensy_library as teensy_library-master
* https://github.com/XenGi/teensy.pretty as teensy.pretty-master
    * Set XGENGI_TEENSY_LIBRARY path in Symbol Libraries etc.
* https://github.com/sparkfun/SparkFun-KiCad-Libraries
    * Set SPARKFUN_KICAD_LIBRARY path in Symbol Libraries etc.
* https://github.com/nosuz/kicad-symbols-footprints
    * Place as ./nosuz-kicad-symbols-footprints in the project directory

Freerouting (https://github.com/freerouting/freerouting) was used to perform the initial routing,
especially with getting the address and data lines out to the level shifters and the Interface 1
ULA.

## DivMMC and ZX Interface 1 support

Technically, the DivMMC and ZX Interface 1 cannot be active simultaneously. The DivMMC ports and
ZX Interface 1 I/O ports clash directly. (More info at
https://worldofspectrum.org/faq/reference/48kreference.htm#PortF7)

To overcome this, the Teensy drives the nIORQ of the Interface 1 ULA high when the DivMMC is
active. Also, only A3 and A4 have been wired to the Interface 1 ULA, as required for the port
decoding - which helped with the PCB routing. The Teensy provides all the ROM facilities for
the Interface 1 behaviour.

### Early prototype

The veroboard prototype used the edge connector A4 (as N/C on the Spectrum 48K) to signal back
into the Interface 1,

* Disconnect the base of Q11 from the Interface 1 ULA IC1 pin 10
    * It will probably be soldered directly onto the ULA pin!
* Cut the nIORQ trace near the Interface 1 ULA IC1 pin 10 - be very careful!
* Add a 1N4148 diode from edge connector A4 with cathode to IC1 pin 10
* Add a 1N4148 diode from edge connector A17 with cathode to IC1 pin 10
* Add a 6.8K resistor from IC1 pin 10 to ground (eg. IC1 pin 20)
* Connect the base of Q11 back to the edge connector A17 with wire

## +2A/+3 soft ROM on Spectrum 128K/+2 (Grey) machines

Spectrum 128K, +2 (Grey) and similar machines (eg. ZX Max 128 Issue 3) require a modification to
support the +2A/+3 soft ROM.

Without it, accesses to the Secondary Memory Control register (0x1FFD) also affect the original
Memory Control register (0x7FFD) due to the partial decoding. (More info at
https://worldofspectrum.org/faq/reference/128kreference.htm)

### Spectrum 128K/+2 (Grey) Memory Control port decoding

The BANK decoding is performed by a PAL10H8 chip, which can be swapped for a GAL16V8 to apply
the "Unrainer/IN 7FFD" fix - see https://spectrumforeveryone.com/technical/applying-the-unrainerin-7ffd-fix-to-128grey-2-machines/ and
Velesoft https://velesoft.speccy.cz/zx/umbrella/umbrella.htm for the original article and files.

Unrainer fixed GAL16V8s with socket and wire can be bought from the Retroleum shop at
https://retroleum.co.uk/.

But, by adding a "BANK = ... & ZA14" term, then the GAL will support the decoding
required for the +2A/+3 soft ROM.

I've placed the updated files in the GAL folder.

* Remove IC29 (Spectrum 128K) / IC7 (Grey +2) - and replace with a socket
    * *Take care of the orientation of the socket!* (The Grey +2 has it pointing downwards)
* On the underside, add a wire from CPU pin 28 to the GAL socket pin 11
* Program the GAL16V8 with GALNEW2A.JED, and fit into socket
    * Again, taking care with the orientation!

### ZX Max 128 Issue 3 Memory Control port decoding

The modification is shown at
https://github.com/DonSuperfo/ZX-Max-128/blob/main/Issue%203/Modify%20for%20%2B3%20ROM.pdf,

* Replace R12 with a 1N4148 diode, with cathode to U9 pin 4
* Add a 1N4148 diode from U6 pin 12 with cathode to U9 pin 4
* Add a 10K resistor from U9 pin 4 to ground (eg. U8 pin 24)
