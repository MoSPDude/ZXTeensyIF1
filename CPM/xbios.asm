; Banked implementation of the original +3 extended BIOS jumpblock.
; The common gate supplies a nested frame; screen callbacks may call us again.
xbios_dispatch:
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    LD A,(XBIOS_FUNCTION)
    SUB $80
    JR C,.unsupported
    LD B,0
.divide:
    SUB 3
    JR C,.index
    INC B
    JR .divide
.index:
    CP $FD
    JR NZ,.unsupported
    LD A,B
    CP 37
    JR NC,.unsupported
    ADD A,A
    LD L,A
    LD H,0
    LD DE,xbios_table
    ADD HL,DE
    LD E,(HL)
    INC HL
    LD D,(HL)
    EX DE,HL
    JR .invoke
.unsupported:
    LD HL,xbios_unsupported
.invoke:
    LD (xbios_call+1),HL
    POP HL
    POP DE
    POP BC
    POP AF
xbios_call:
    JP 0
xbios_unsupported:
    LD HL,0
    XOR A
    RET
xbios_terminal:
    PUSH AF
    LD A,(XBIOS_FUNCTION)
    LD (SCREEN_FUNCTION),A
    POP AF
    JP SCREEN_GATE
xbios_version:
    LD A,3
    LD BC,XBIOS_VERSION
    LD HL,XBIOS_VERSION
    RET
xbios_info:
    LD A,(FLOPPY_COUNT)
    DEC A
    NEG                           ; 00 = one drive, FF = two
    LD BC,$0800                   ; 128K Spectrum; no serial backend
    LD HL,BUFFER_TABLE
    RET
xbios_screen:
    PUSH AF
    LD A,$E9
    LD (SCREEN_FUNCTION),A
    POP AF
    JP SCREEN_GATE
xbios_find:
    PUSH DE
    LD HL,FID_NAME
    LD B,8
.copy:
    LD A,(DE)
    LD (HL),A
    INC HL
    INC DE
    DJNZ .copy
    POP DE
    LD HL,HOST_FIND
    JP HOST_GATE
xbios_table:
    DW floppy_dd_init,floppy_dd_setup,floppy_dd_read,floppy_dd_write
    DW floppy_dd_check,floppy_dd_format,floppy_dd_login,$1926
    DW floppy_drive_status,floppy_dd_read_id,floppy_dd_l_dpb,floppy_dd_l_xdpb
    DW floppy_motor_start,floppy_dd_timeout,floppy_motor_stop
    DW floppy_dd_low_read,floppy_dd_low_write,floppy_dd_seek
    DUP 3
    DW xbios_unsupported
    EDUP
    DUP 7
    DW xbios_terminal
    EDUP
    DW km_set_expand,km_set_key,km_get,km_put,km_set_speed
    DW xbios_version,xbios_info,xbios_screen,xbios_find
