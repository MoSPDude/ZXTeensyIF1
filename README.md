
# ZXTeensyIF1

![image](./Images/pcb-v0.7.JPG "Working PCB")

A Teensy 4.1 powered DivMMC and ZX Interface 1 clone for the ZX Spectrum 48K/128K/+2 (Grey) machines,

* ZX Interface 1
    * Requires the 9V and 5V power rails
    * Uses a MAX232 for the RS232 level shifting, so no +12V or -12V required
    * Supports 16KB shadow soft ROM
        * eg. Ian Collier's modified Interface 1 ROM from https://ftp.nvg.ntnu.no/pub/sinclair/roms/imc-i1rom.zip
            * The [imc-i1.rom](Extras/imc-i1.rom) here contains a copy with the "Parallel Printer" modification disabled
* DivMMC with 512KB RAM
    * Supports accessing the main SD card, HDF and IMG images
    * Large images over FAT32 file limit can be split into multiple files
        * eg. esximage.hdf, esximage.001, up to esximage.999
        * The maximum SD card image size is ~2TB
    * If you use images from https://esxdos.zxfiles.net/ - remember to hex edit "DMA=1" to "DMA=0"
* Multiface 128 emulation
    * NMI button and soft ROM
* ZX Interface 2 ROM cartridge emulation
    * It does not emulate the Joystick ports
* ZXC2 and ZXC3 with Flash ROM cartridge emulation
    * Implements ZXC2 ROM banking
    * Implements ZXC3 ROM banking with 128KB Flash ROM
    * Implements SPECTRA shadow ROM support
        * The ROM file needs to be named as "SPECTRA_\*.bin"
* Dandanator Mini with Flash ROM cartridge emulation
    * Implements slow pulsed banking, and fast command banking
    * Can load individual MLD files, or full 512KB generated ROM images
* ESP-01S module via TX8/RX8
    * Requires the 9V power rail for the 3.3V regulator
    * Available on ports 0x143B (5179) for RX and 0x133B (4923) for TX
    * https://www.specnext.com/the-next-on-the-network/
* USB mouse and gamepad Kempston interface
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
* Spectrum +3 and ZX LPRINT III Centronics printer
    * Writes output into a timestamped "printer-YYYYMMDD-HHMMSS.txt" file on SD card, adding a numeric suffix if that file already exists
* Soft ROM emulation
    * Override the internal Spectrum ROM with ROMs from SD card
    * Supports 16KB (Spectrum 48K), 32KB (Spectrum 128K/+2 (Grey)) and 64KB (Spectrum +2A/+3) ROMs
        * The +2A/+3 soft ROM support requires port decoding changes (see below)
    * Provides the Interface 1 ROM, Multiface 128 ROM and DivMMC ROM
* Menu ROM derived from TomDDGs ZXPicoIF2Lite ROMExplorer
* Z80 snapshot loading from TomDDGs ZXPicoIF2Lite
    * Integrated "z80torom" for loading 'z80' and 'sna' files
* Save and restore states
    * Uses the Z80 snapshot loader to restore state on power-on
    * Hold the button to disable the power-on restore
* MDR microdrive loading with Paul Farrows SPECTRA Microdrive Emulator
    * http://www.fruitcake.plus.com/Sinclair/Interface2/Cartridges/Interface2_RC_New_Microdrive_Emulator.htm
    * Supports MDR images with up to 90KB of data
* Small WebDAV class 1 HTTP server for WiFi file access over ESP-01S
    * Use WinSCP to send files, or HTTP PUT to send files eg. "curl -T FILENAME.ROM http://192.168.0.254/FILENAME.ROM"
        * Windows native support requires LOCK (and UNLOCK) which is Not Implemented
    * Use web browser to list files, and download
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

The ZXTEENSY directory contains the peripheral ROM images - the exact redistribution
status of these is unknown, but they have been gathered from sources from the Internet,
and provided for convenience and safe-keeping.

This is a project that I started with a view of wanting a DivMMC clone that works with a
ZX Interface 1 attached to my ZX Max 128 - as I own a ZX Microdrive as well as a ZXPicoMD
(https://github.com/TomDDG/ZXPicoMD) ... and didn't want to keep disconnecting the ZX
Interface 1.

> [!WARNING]
> I created this for myself, and it is distributed in the hope that it will be useful,
> but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
> FITNESS FOR A PARTICULAR PURPOSE.
>
> I take ZERO responsibility if it damages your hardware!

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
* http://www.dandare.es/Proyectos_Dandare/ZX_Dandanator%21_Mini_EN.html
    * Dandantor Mini slow banking from PIC source code
    * Also, https://github.com/chernandezba/zesarux/tree/main/src/storage for fast banking
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
* https://spectrumcomputing.co.uk/pub/sinclair/technical-docs/PrismVTX5000Modem_Schematics-Interface.bmp
    * Prism VTX5000 schematic
* https://zxpress.ru/chapters_images/periferiya/periferiya-33.png
    * ZX LPRINT III schematic
* https://divide.speccy.cz/files/pgm_model.txt
    * DivIDE programming model
* https://www.thingiverse.com/thing:6500064
    * Also from TomDDG, a replacement ZX Interface 1 case

Without the above projects, this would not have been possible!

## Construction

[Construction details are here](PCB/README.md)

## SD Card Setup

* ZXTEENSY/
    * CONFIGS/
        * \<NAME\>.CFG (Saved configurations that appear as "NAME" in the Menu ROM for quick selection)
    * STATE/\<SLOT\>/ (Save slots 0-14; slot 15 is reserved)
        * STATE.Z80 (Z80 v3 snapshot for the save state)
        * SCREEN.SCR (6912-byte active Spectrum screen for load preview)
        * (Additional saved state files for the save slot)
    * MENU.ROM
    * MF128.ROM (MD5SUM: ca8c9d97c8aedd718d1081fad2e3af8d)
    * GENIE128.ROM (MD5SUM: b942f2e658d15da9747791a15a9a110d)
        * Genie 128K disassembler Multiface 128 RAM image
    * ESXMMC.BIN (MD5SUM: fa50b0258e52b8d72bd83cc2fb6e1013)
    * SPECTRA_IF1_ED2_ME_ROM_Formatted.bin (MD5SUM: 052ad91ee822604960e8ca8d32a3ddb9)
    * IF1.ROM (Optional)
    * VTX.ROM (MD5SUM: 12a62cb7ea7383f109c2711dfca99f5e)
    * LPRINT32.ROM (MD5SUM: e85f4ccb4cc80aaa81cceffb3e064bf7)
    * netman.z80 (WiFi Network Manager snapshot)
    * rtc_setup.z80 (RTC Setup snapshot)
    * ZXTEENSY.CFG (Current configuration from Menu ROM)
* ROMS/
    * (ZX Spectrum ROMs ending ".rom")
    * (ZXC2 ROMs ending ".bin")
    * (Other ROM files)
* (Other ESXDOS files)
* SYS/
    * RTC.SYS (Optional, for RTC access)
    * (Other ESXDOS files)
* ZXTEENSY.HEX (Optional, firmware update)

## Using the Menu ROM

![image](./Images/menu-260322.JPG "Menu 20260322")

To load the Menu ROM, either,

* Use "Boot into Menu" to load on initial power on
* or, Hold the button, and Reset
* or, Hold Reset for longer than 5 seconds

Inside the Menu ROM you can,

* Restart with either the ZXTeensyIF1 enabled or disabled
    * Pressing Reset will also reset with the ZXTeensyIF1 enabled
* Browse the SD card to select files to load, or mount
    * Selecting known file extensions will either load or ask to mount
    * Selecting unknown file extensions will ask for an action
    * From the in-game browser, select a POK file and tick one or more trainers
      before selecting "Apply selected trainers"
* Browse the system Soft ROMs in the "ROMS/" directory to load, or set as Soft ROM
    * Double selecting a system Soft ROM will immediately reset into it
* Select a saved configuration to load
    * Double selecting a saved configuration will immediately reset into it
* Edit the configuration
    * Enable or disable the peripherals
    * Eject mounted disks by selecting them
    * Set to fetch time over the WiFI, and set the timezone
    * Load the RTC Setup or WiFi Network Manager snapshots
* Start or stop the HTTP server to transfer files over WiFi

To firmware update, place the ZXTEENSY.HEX file on the root of the SD card and
select the option from the Menu ROM - then wait for the Spectrum to reset
(!! It will take a minute !!).

In "Load ROMS", from the "ROMS/" directory,

| File extension | File type |
|----------------|-----------|
| .rom | System Soft ROM |
| .bin | ZXC2 cartridge |
| Other | ZX Interface 2 cartridge |

In "Browse SD card", in other directories,

| File extension | File type |
|----------------|-----------|
| .rom, .bin | ROM cartridge, or RAM image |
| .z80, .sna | Z80 snapshot |
| .hdf, .img | DivMMC SD card |
| .dsk | Spectrum +3 disk |
| .mdr | ZX Microdrive cartridge |
| .tap, .tzx | Audio cassette |
| .pok | POK trainers (in-game browser only) |
| Other | The menu will ask for an action |

POK trainers are applied by saving a temporary state to reserved slot 15,
patching the selected RAM banks, and immediately restoring that state. POK
trainers that require a prompted value (`256`) are not supported and cannot be
enabled.

### Preparing the SD Card

ESXDOS has trouble loading if it is not "early" on the SD card,

* Format the SD card with FAT32
* Extract ESXDOS on computer, and transfer all directories and ESXMMC.BIN to the SD card
* Transfer the "/ZXTEENSY" directory, with ROMs and CONFIGS
* Create system "/ROMS" directory, and add other ROMs
* Add any other files

## Building the firmware

* Setup the Arduino IDE 2.3.6 for the Teensy 4.1
    * "Teensy (for Arduino IDE 2.0.4 or later)" v1.60.0
* Open ZXTeensyIF1\ZXTeensyIF1.ino
* Set Board to Teensy 4.1
* Set Optimize to "Smallest Code with LTO"
* Set CPU Speed to "720 MHz (overclock)"
* Verify and Upload

## Prism VTX5000 support

The VTX5000 over ESP-01S uses the ESP8266 in UART-WiFi passthrough mode after establishing
a connection to a URL over TCP. If the VTX.ROM is provided, then it will load after reset.
The default URL is the TELSTAR Viewdata service at https://glasstty.com/telstar/ .

To access TELSTAR,
 * Press CAPS SHIFT + ENTER to open the menu
 * Press 0 to "Log ON or OFF"
 * Press 1 for "Manual Log ON"
 * Enter the ID "\*\*\*\*\*\*\*\*\*\*" ie. 10 asterixes
 * Read the welcome message, and press ENTER as the "#" key

To navigate, either press numbers to select items, or enter pages in the form of "*800#" :-
 * SYMBOL SHIFT is the "*" key
 * ENTER is the "#" key

## WebDAV class 1 support

The HTTP server only supports a single connection.

Windows 11 seems to still send a LOCK command when sending a file to the server,
though it is not presented in the server Allow list or required by WebDAV class 1.
It then ignores the "501 Not Implemented" or the "405 Method Not Allowed", and
fails to send the file.

WinSCP seems to work fine - you may need to change the preferences to ensure only
a single upload connection.

OpenAI Codex helped to provide the additional WebDAV class 1 implementation over
just the basic HTTP PUT and GET that I'd implemented.

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

* Replace R12 with a 1N4148 diode, with cathode to U9 pin 9
* Add a 1N4148 diode from U6 pin 12 with cathode to U9 pin 9
* Add a 10K resistor from U9 pin 9 to ground (eg. U8 pin 24)

## 3rd party Xbox One USB gamepads

You will need to find and add the USB PID:VID code for the controller to the USBHost_t36
library.

In the case of my "8BitDo M30 Wired Controller for Xbox One", under Windows 11
Device Manager, it was the "Xbox Peripherals > Xbox Gaming Device" *NOT* the
(virtual) HID device. Right click the device, select Properties, then Details and
select "Hardware IDs" to find it listed as "USB\VID_2DC8&PID_200A&..."

Once found, add the USB PID:VID code to %LOCALAPPDATA%\Arduino15\packages\teensy\hardware\avr\1.60.0\libraries\USBHost_t36\joystick.cpp,
under JoystickController::pid_vid_mapping ,

* 8BitDo M30 Wired Controller for Xbox One :- \{ 0x2dc8, 0x200a, XBOXONE, false \}

## ROM paging and banking

ROMs are paged in a priority order - if more than one ROM is paged, the highest active entry wins.
When that ROM pages out, it falls back to the next highest active entry. The base
Spectrum ROMs are mutually exclusive, and selected by the 128K/+2A/+3 Memory Control
registers - while add-on ROMs can sit above them until they are paged out.

The following ROMs are provided by the Teensy, listed in priority order,

| Peripheral / ROM | Pages in from | Pages out from | Banking notes |
| ---------------- | ------------- | -------------- | ------------- |
| Spectrum ROM 0 | On reset, or I/O port 0x7FFD bit 4 = 0, and I/O port 0x1FFD bit 2 = 0 | Replaced by another Spectrum ROM | 48K BASIC ROM, or 128K/+2 Editor ROM slot |
| Spectrum ROM 1 | I/O port 0x7FFD bit 4 = 1, and I/O port 0x1FFD bit 2 = 0 | Replaced by another Spectrum ROM | 128K/+2 BASIC ROM, or +2A/+3 Syntax ROM slot |
| Spectrum ROM 2 | I/O port 0x7FFD bit 4 = 0, and I/O port 0x1FFD bit 2 = 1 | Replaced by another Spectrum ROM | +2A/+3 +3DOS ROM slot |
| Spectrum ROM 3 | I/O port 0x7FFD bit 4 = 1, and I/O port 0x1FFD bit 2 = 1 | Replaced by another Spectrum ROM | +2A/+3 BASIC ROM slot |
| Interface 1 | Post-M1 access to address 0x0008 or 0x1708 from Spectrum ROM 0, 1 or 3, when Interface 1 is enabled and DivMMC is not | Post-M1 access to address 0x0700 | Emulates Interface 1 ROM paging while the Teensy inhibits the physical Interface 1 ULA I/O decode when DivMMC is active. |
| Multiface 128 | Multiface NMI path at address 0x0066/0x0067, or read from I/O port 0xXXBF | Read from I/O port 0xXX3F; write to I/O port 0xXX3F also disables Multiface mode | Higher priority than Interface 1, so Interface 1 may be marked paged underneath it and then become visible after Multiface pages out. |
| DivMMC | M1 access to addresses 0x3Dxx, post-M1 access to addresses 0x0000, 0x0008, 0x0038, 0x04C6 or 0x0562 from Spectrum ROM 0, 1 or 3; DivMMC NMI at address 0x0066; or I/O port 0xXXE3 with CONMEM/automap active | Post-M1 access to addresses 0x1FF8-0x1FFF unless MAPRAM is active; or I/O port 0xXXE3 when CONMEM and automap are clear | I/O port 0xXXE3 also selects the DivMMC RAM bank. Bit 7 is CONMEM, bit 6 latches MAPRAM, and the low bits select internal or extended RAM. MAPRAM serves RAM bank 3 instead of the ROM when CONMEM is clear. |
| LPRINT III | Read from I/O port 0xXXFB | Read from I/O port 0xXX7B | Printer data/strobe writes use I/O ports 0xXX7B and 0xXXFB; ROM paging itself is read-port driven. |
| VTX5000 modem | VTX5000 register write to I/O port 0xXXFF with bit 5 clear | VTX5000 register write to I/O port 0xXXFF with bit 5 set | The first register write after reset is ignored. |
| ZXC2/ZXC3 cartridge | ZXC2/ZXC3 address-banking access in addresses 0x3FC0-0x3FFF, or post-M1 access to address 0x0008/0x1708 when used as Shadow ROM | ZXC2/ZXC3 address-banking access in addresses 0x3FC0-0x3FFF, or post-M1 access to address 0x0700 when used as Shadow ROM | For ZXC2, address bits 0-3 select the 16KB bank and bit 4 pages in/out. ZXC3 uses bits 0-2 for the bank and bit 3 to enter flash-write handling. Bit 5 locks further ZXC2/ZXC3 paging. |
| Dandanator/MLD cartridge | Dandanator/MLD slow-pulse or fast command bank select | Dandanator/MLD page-out command | Commands 1-32 select slots, command 33 pages out, command 34 pages out and locks, and command 40 performs the fast bank/control command. |
| Snapshot/state loader | Snapshot/state loader start | Access to address 0x3FFF advances loader banks; the final access pages out | Used for generated snapshot/state loader ROMs. |
| Menu ROM | Menu NMI target at address 0x0066, from any current ROM | Post-M1 access to address 0x003B | Highest priority ROM. I/O port 0xXXBF selects the active menu RAM page, and I/O port 0xXXEB is used for menu commands and selections. |

## I/O port mapping

The following ports are decoded by the Teensy,

| Port | R/W | Function | Comments |
| ---- | --- | -------- | -------- |
| 0xXX1F | R | Kempston joystick | |
| 0x133B | R | UART status | |
| 0x133B | W | UART TX data | |
| 0x143B | R | UART RX data | |
| 0x143B | W | UART baud register | |
| 0x7N3B | R/W | DivMMC RTC registers 0 - 15 | N is the register address |
| 0xXX3F | R | Multiface 128 page out | Returns bit 7 as the Current Screen |
| 0xXX3F | W | Multiface 128 disable | |
| 0xXX7B | R | LPRINT III page out | Returns status |
| 0xXX7B | W | LPRINT III data with strobe | |
| 0xXX7F | R/W | VTX5000 data | |
| 0xXXBF | R | Multiface 128 page in | Returns bit 7 as the Current Screen |
| 0xXXBF | W | Multiface 128 NMI clear | Menu ROM RAM banking |
| 0xFADF | R | Kempston mouse buttons | |
| 0xFBDF | R | Kempston mouse X axis | |
| 0xFFDF | R | Kempston mouse Y axis | |
| 0xXXE3 | W | DivMMC control register | |
| 0xXXE7 | W | DivMMC SPI select | |
| 0xXXEB | R/W | DivMMC SPI data | Menu ROM read command, write selected option |
| 0xXXFB | R | LPRINT III page in | Returns status |
| 0xXXFB | W | LPRINT III data, clear strobe | |
| 0x0FFD | R | +3 Centronics busy status | |
| 0x0FFD (0b0000XXXX_XXXXXX0X) | W | +3 Centronics data | |
| 0x1FFD (0b0001XXXX_XXXXXX0X) | W | +2A / +3 Secondary Memory Control | ROM paging, +3 FDC motor and +3 Centronics strobe |
| 0x2FFD | R | +3 FDC status register | |
| 0x3FFD | R | +3 FDC data | |
| 0x3FFD (0b0011XXXX_XXXXXX0X) | W | +3 FDC data | |
| 0x7FFD (0b01XXXXXX_XXXXXX0X) | W | 128K Memory Control | Current Screen, and ROM paging |
| 0xXXFE | R | Spectrum ULA | Drives EAR input for tape |
| 0xXXFF | R/W | VTX5000 register | Ignore first write access, controls ROM paging |

The following ports are decoded by the Interface 1 ULA when not inhibited by the Teensy,

| Port | R/W | Function | Comments |
| ---- | --- | -------- | -------- |
| 0xXXE7 (0bXXX00XXX) | R/W | Interface 1 microdrive data | Halts Z80 on read |
| 0xXXEF (0bXXX01XXX) | R/W | Interface 1 control register | |
| 0xXXF7 (0bXXX10XXX) | R/W | Interface 1 RS232/network data | |
