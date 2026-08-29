
# ZXTeensyIF1 Construction

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

## Bill of Materials

| Reference | Qty | Value | Comments |
| --- | --- | --- | --- |
| C1, C2, C7, C8 | 4 | 1uF | Use 0.1uF if U8 is MAX232A |
| C3 | 1 | 100pF | |
| C4 | 1 | 22pF | |
| C5 | 1 | 47nF | |
| C6, C13 | 2 | 47uF 16V | |
| C9, C10, C11, C12 | 4 | 100nF | |
| D1, D2, D3, D9, D11 | 5 | 1N4148 | |
| D4 | 1 | 1N4004 | or 1N4001 etc. |
| D8 | 1 | BZX79CV4V3 | |
| J1 | 1 | ESP-01S | Optional, for WiFi |
| J3 | 1 | Polou D24V5F3 | Optional, fit if ESP-01S is fitted |
| L1 | 1 | 22uH 0.41A | |
| Q1, Q2 | 2 | BC213C | |
| Q3, Q6, Q9 | 3 | BC184C | |
| Q4, Q5, Q7, Q8 | 4 | PN2222A | |
| Q10, Q11 | 2 | ZTX313 | |
| R1, R3, R5, R11, R12, R14, R24, R25, R31 | 9 | 1K0 | |
| R2, R10, R17, R18 | 4 | 6K8 | |
| R4, R6, R8, R13 | 4 | 3K9 | |
| R7 | 1 | 47R | |
| R9, R16, R21, R26, R27, R33, R36 | 7 | 10K | |
| R15 | 1 | 2K2 | |
| R22, R23, R34 | 3 | 330R | |
| R30 | 1 | 270R | |
| R35 | 1 | 47K | |
| SK1 | 1 | DE9_Receptacle | |
| SK2, SK3 | 2 | 3.5mm Mono Socket | |
| SK5 | 1 | ZX Spectrum connector | Difficult - see notes |
| SW1 | 1 | PTH_RA_h7.5mm | 6 x 6mm x 8mm Right Angle |
| U1 | 1 | LA15-302 | ZX Interface 1 ULA |
| U2 | 1 | Teensy4.1 | Teensy 4.1 No Ethernet |
| U3, U5, U6, U7 | 4 | SN74LVC245APW | |
| U4 | 1 | 74HC04 | |
| U8 | 1 | MAX232 | Alternative is MAX232A |
| X1 | 1 | 8Mhz crystal | |

## Notes

In my build, the ULA, 74HC04, BZX79CV4V3, 8 MHz crystal, 3K9 resistors, inductor
and all connectors came from a donor ZX Interface 1 board.

Removing the ZX Interface 1 edge connector that goes to the ZX Spectrum was an immense pain - so
might have to find other ideas. Even with gentle heat, I managed to deform and melt the plastic
riser block...

## Hardware revisions

* v0.7 PCB prototype
    * PCBs have been returned from PCBWay
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

## DivMMC and ZX Interface 1 support

Technically, the DivMMC and ZX Interface 1 cannot be active simultaneously. The DivMMC ports and
ZX Interface 1 I/O ports clash directly. (More info at below)

To overcome this, the Teensy drives the nIORQ of the Interface 1 ULA high when the DivMMC is
active. Also, only A3 and A4 have been wired to the Interface 1 ULA, as required for the port
decoding - which helped with the PCB routing. The Teensy provides all the ROM facilities for
the Interface 1 behaviour.
