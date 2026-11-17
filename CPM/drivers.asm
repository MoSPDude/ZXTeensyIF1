; Bank 2 hardware support. Original fonts are supplied locally by patch_ems.py.
    DEVICE ZXSPECTRUM128
    INCLUDE "layout.inc"
    ORG 0
    DS $100,0
    JP initialise
    JP console_status
    JP console_input
    JP console_output
    JP disk_read
    JP disk_write
    JP terminal_reset
    JP set_border
    JP xbios_dispatch
    JP timer_service
    JP disk_transfer
    JP clock_service
    JP floppy_select
    JP floppy_external_error
    JP floppy_external_message

initialise:
    XOR A
    CALL keyboard_initialise
    CALL floppy_initialise
    CALL terminal_initialise
    LD A,1                       ; motor waits may yield after initial setup
    LD (IRQ_READY),A
    LD HL,banner
    CALL print_text
    LD HL,one_drive
    LD A,(FLOPPY_COUNT)
    CP 2
    JR NZ,print_text
    LD HL,two_drives
print_text:
.print:
    LD A,(HL)
    OR A
    RET Z
    INC HL
    LD C,A
    CALL console_output
    JR .print
banner:
    DB "CP/M Plus - ZXTeensyIF1",13,10
    DB "DivMMC AllRAM / +3 FDC",13,10
    DB "61K TPA, ",0
one_drive: DB "1 floppy drive",13,10,0
two_drives: DB "2 floppy drives",13,10,0

console_status:
    CALL keyboard_peek
    LD A,(pending_valid)
    OR A
    RET Z
    LD A,$FF
    RET
console_input:
    DI
    XOR A
    LD (KEY_WAITING),A
    CALL keyboard_peek
    LD A,(pending_valid)
    OR A
    JR NZ,.ready
    LD A,1
    LD (KEY_WAITING),A
    EI
    HALT
    JR console_input
.ready:
    JP keyboard_read

console_output:
    PUSH BC
    PUSH DE
    PUSH HL
    LD A,$FF
    LD (SCREEN_FUNCTION),A
    CALL SCREEN_GATE
    POP HL
    POP DE
    POP BC
    RET
terminal_initialise:
    LD A,$FD
    JR terminal_service
terminal_reset:
    LD A,$C2
    JR terminal_service
set_border:
    LD A,$CE
terminal_service:
    LD (SCREEN_FUNCTION),A
    JP SCREEN_GATE

    INCLUDE "floppy.asm"
    INCLUDE "keyboard.asm"
    INCLUDE "timer.asm"
    INCLUDE "xbios.asm"
    ASSERT $ <= $18A4
    DS $18A4-$,0
    INCBIN "format_specs.bin"
    DS $1926-$,0
    INCBIN "format_dpb.bin"
    DS $1B37-$,0
    INCBIN "format_math.bin"
    ASSERT $ < $2000
    INCLUDE "keyboard_data.asm"
    INCLUDE "floppy_data.asm"
    DS $3000-$,0
    INCLUDE "floppy_xbios.asm"
    ASSERT $ < $3C00
