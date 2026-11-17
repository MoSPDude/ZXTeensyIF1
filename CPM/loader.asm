    DEVICE ZXSPECTRUM128
    INCLUDE "layout.inc"
    INCLUDE "segments.inc"
    ORG $B000
    DI
    LD SP,$AF00
    LD A,7
.clear_bank:
    PUSH AF
    CALL select_staging_bank
    LD HL,0
    LD DE,1
    LD BC,$3FFF
    LD (HL),0
    LDIR
    POP AF
    DEC A
    JP P,.clear_bank

    LD IX,segments
.segment:
    LD A,(IX+0)
    CP $FF
    JR Z,.finish
    CALL select_staging_bank
    LD L,(IX+1)
    LD H,(IX+2)
    LD E,(IX+3)
    LD D,(IX+4)
    CALL unpack
    LD DE,5
    ADD IX,DE
    JR .segment
.finish:
    LD BC,$7FFD
    LD A,8
    OUT (C),A                     ; common Spectrum RAM 0 at C000
    LD HL,$C000
    LD DE,$C001
    LD BC,$3FFF
    LD (HL),0
    LDIR
    LD HL,$8000                   ; original resident BDOS + our common BIOS
    LD DE,BDOS_BASE
    LD BC,$0C00
    LDIR
    XOR A
    CALL SELECT_BANK
    JP BIOS_BASE

; Literals 1..126; 127 introduces a run; bit 7 introduces a 12-bit
; backwards reference. Length ten uses an extra byte (maximum 265).
; Staged source bytes wrap from FFFF to the second chunk at 4000.
unpack:
    CALL fetch
    OR A
    RET Z
    CP 127
    JR C,.literal_start
    JR NZ,.match
    CALL fetch
    INC A
    LD B,A                        ; zero means a 256-byte run to DJNZ
    CALL fetch
.repeat:
    LD (DE),A
    INC DE
    DJNZ .repeat
    JR unpack
.literal_start:
    LD B,A
.literal:
    CALL fetch
    LD (DE),A
    INC DE
    DJNZ .literal
    JR unpack
.match:
    LD C,A
    AND $70
    RRCA
    RRCA
    RRCA
    RRCA
    ADD A,3
    LD B,A
    LD A,C
    AND 15
    LD (match_high),A
    CALL fetch
    LD (match_low),A
    LD A,B
    CP 10
    LD C,B
    LD B,0
    JR NZ,.copy
    CALL fetch
    ADD A,10
    LD C,A
    JR NC,.copy
    INC B
.copy:
    PUSH HL
    PUSH DE
    LD HL,(match_low)
    INC HL
    EX DE,HL
    OR A
    SBC HL,DE
    POP DE
    LDIR
    POP HL
    JR unpack
match_low: DB 0
match_high: DB 0
fetch:
    LD A,(HL)
    INC HL
    PUSH AF
    LD A,H
    OR L
    JR NZ,.done
    LD H,$40
.done:
    POP AF
    RET

select_staging_bank:
    PUSH BC
    LD E,A
    AND 1
    RLCA
    RLCA
    RLCA
    RLCA
    OR $0C                        ; source remains Spectrum RAM 4 at C000
    LD BC,$7FFD
    OUT (C),A
    LD A,E
    AND 4
    LD BC,$1FFD
    OUT (C),A
    LD A,E
    AND 2
    RLCA
    RLCA
    LD BC,$00E3
    OUT (C),A                     ; refresh both 8K halves after 7FFD
    POP BC
    RET

segments:
    SEGMENTS
    ASSERT $ <= $BF00
