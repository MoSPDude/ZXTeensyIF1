; New Spectrum backend for the locally supplied PCW8256 GIOS #8 PRL.
; The patcher assembles this at two page origins to build relocation bits.
    DEVICE ZXSPECTRUM128
    IFNDEF PATCH_DELTA
PATCH_DELTA EQU 0
    ENDIF
userf           EQU $0D88+PATCH_DELTA
viewport        EQU $0D5E+PATCH_DELTA
set_colour      EQU $0D8B+PATCH_DELTA
select_pen      EQU $0D98+PATCH_DELTA
select_paper    EQU $0DA3+PATCH_DELTA
set_mode        EQU $0DAE+PATCH_DELTA
set_mask        EQU $0DCA+PATCH_DELTA
plot_masked     EQU $0DDB+PATCH_DELTA
clear_screen    EQU $10F5+PATCH_DELTA
colours         EQU $1190+PATCH_DELTA
input_modes     EQU $0D0B+PATCH_DELTA
driver_start    EQU $0100+PATCH_DELTA
graphics_state  EQU $1180+PATCH_DELTA
line_mask       EQU $118A+PATCH_DELTA
mode_target     EQU $0DE8+PATCH_DELTA
KERNEL_FRAME    EQU $2E00
KERNEL_RESULTS  EQU $2E10
KERNEL_RETURN   EQU $2E12
KERNEL_STACK    EQU $2FF0

    ORG $11A0+PATCH_DELTA
initialise:
    ; The loaded GIOS remains in shared RAM while its raster code runs in D7.
    LD A,HIGH initialise
    CP $80
    JR NC,.visible
    OR A
    RET
.visible:
    LD HL,($0001)
    LD DE,$0057
    ADD HL,DE
    LD DE,userf
    LD BC,3
    LDIR
    CALL userf
    DW $00E3
    CP 3                        ; Spectrum +3 / ZXTeensyIF1 XBIOS
    JR Z,.spectrum
    OR A
    RET
.spectrum:
    LD A,($F9FA)                ; this port's common memory starts at 4000
    CP $40
    JR Z,.memory_map
    OR A
    RET
.memory_map:
    CALL userf
    DW $00BF
    LD A,E
    LD B,0
    CP 31
    JR Z,.save_mode
    INC B
    CP 79
    JR NZ,.save_mode
    INC B
.save_mode:
    LD A,B
    LD (saved_mode),A
    CALL userf
    DW $00C5
    LD (saved_status),A
    XOR A
    CALL userf
    DW $00C8
    LD HL,graphics_sequence
    CALL print_sequence
    LD BC,0
    LD DE,$1F17                 ; 32 columns, 24 rows; inclusive bounds
    CALL viewport
    XOR A
    LD B,A
    CALL set_colour
    LD A,1
    LD B,A
    CALL set_colour
    XOR A
    CALL select_paper
    LD A,1
    CALL select_pen
    LD HL,$FFFF
    CALL set_mask
    LD A,1
    CALL set_mode
    CALL install_kernel
    SCF
    RET

begin_graphics:
    XOR A
    LD HL,191
    JP clear_screen

close_graphics:
    LD HL,restore_sequence
    CALL print_sequence
    LD A,(saved_mode)
    LD E,A
    CALL conout
    LD A,(saved_status)
    CALL userf
    DW $00C8
    LD HL,close_sequence
    JP print_sequence

print_sequence:
    ; Counted strings permit the NUL parameter in ESC 3,0.
    LD B,(HL)
    INC HL
.next:
    LD E,(HL)
    CALL conout
    INC HL
    DJNZ .next
    RET
conout:
    PUSH BC
    PUSH DE
    PUSH HL
    LD C,2
    CALL $0005
    POP HL
    POP DE
    POP BC
    RET

graphics_sequence: DB 5,$1B,"3",0,$1B,"f"
restore_sequence:  DB 2,$1B,"3"
close_sequence:    DB 6,$1B,"E",$1B,"H",$1B,"e"
saved_mode:        DB 1
saved_status:      DB $FF

pixel_address:
    ; DE=x, HL=y in physical top-down pixels. Return HL=bitmap address,
    ; B=bit mask. Spectrum rows are interleaved; x and y were clipped by GIOS.
    LD A,E
    AND 7
    LD C,A
    LD A,$80
    INC C
.mask:
    DEC C
    JR Z,.address
    RRCA
    JR .mask
.address:
    LD B,A
    LD A,L
    AND 7
    OR $C0                      ; Spectrum RAM 7 is mapped at C000
    LD H,A
    LD A,L
    RRCA
    RRCA
    RRCA
    AND $18
    OR H
    LD H,A
    LD A,L
    AND $38
    RLCA
    RLCA
    LD L,A
    LD A,E
    RRCA
    RRCA
    RRCA
    AND $1F
    OR L
    LD L,A
    RET

horizontal_span:
    ; D7 raster call: IX=left, DE=right, IY=y. The polygon filler supplies
    ; its pattern and writing mode. Use the original masked pixel operation
    ; so transparent, replace, XOR and erase have identical semantics.
    PUSH DE
    POP HL
    PUSH IX
    POP DE
    BIT 7,D
    JR Z,.left_positive
    LD DE,0
.left_positive:
    LD A,D
    OR A
    RET NZ                       ; left is beyond the right edge
    BIT 7,H
    RET NZ                       ; right is left of the display
    LD A,H
    OR A
    JR Z,.right_inside
    LD HL,255
.right_inside:
    OR A
    SBC HL,DE
    RET C
    INC HL
    LD B,H
    LD C,L
    PUSH IY
    POP HL
    LD A,H
    OR A
    RET NZ
    LD A,L
    CP 192
    RET NC
.next:
    PUSH BC
    PUSH DE
    PUSH HL
    CALL plot_masked
    POP HL
    POP DE
    POP BC
    INC DE
    DEC BC
    LD A,B
    OR C
    JR NZ,.next
    RET

fill_screen:
    ; D7 raster call for clear workstation; bank 7 is visible at C000.
    LD HL,colours
    OR A
    JR Z,.colour
    INC HL
.colour:
    LD A,(HL)
    LD HL,$C000
    LD DE,$C001
    LD BC,6143
    LD (HL),A
    LDIR
    LD HL,$D800
    LD DE,$D801
    LD BC,767
    LD (HL),7                   ; white ink on black paper, no flash
    LDIR
    RET

keyboard_initialise:
    LD HL,$0101
    LD (input_modes),HL
    LD (input_modes+2),HL
    LD HL,1                     ; one keyboard locator; no PCW mouse RSX
    RET

keyboard_poll:
    ; The Spectrum's translated cursor controls are BS/FF/VT/LF. Avoid the
    ; PCW raw-key tables and mouse RSX. Other characters terminate a request.
    PUSH DE
    PUSH HL
    LD C,6
    LD E,$FF
    CALL $0005
    POP HL
    POP DE
    OR A
    RET Z
    LD IX,0
    LD IY,0
    CP 8
    JR Z,.left
    CP 12
    JR Z,.right
    CP 11
    JR Z,.up
    CP 10
    JR Z,.down
    OR A
    SCF
    RET
.left:
    DEC IX
    JR .motion
.right:
    INC IX
    JR .motion
.up:
    INC IY
    JR .motion
.down:
    DEC IY
.motion:
    XOR A                       ; Z: movement; carry: event available
    SCF
    RET

save_mapping:
    ; Keep the motor and other 1FFD bits. Derive the page bits from the
    ; logical bank, just as the BIOS does, without changing its shadows.
    LD A,($FF00)
    AND 1
    RLCA
    RLCA
    RLCA
    RLCA
    OR 8
    LD (saved_port7),A
    LD A,($FF25)
    AND $FB
    LD HL,$FF00
    BIT 2,(HL)
    JR Z,.port1
    OR 4
.port1:
    LD (saved_port1),A
    LD A,(HL)
    AND 2
    RLCA
    RLCA
    LD (saved_e3),A
    RET

    MACRO MAP_D7
    LD BC,$7FFD
    LD A,(saved_port7)
    OR $10
    OUT (C),A
    LD BC,$1FFD
    LD A,(saved_port1)
    OR 4
    OUT (C),A
    LD BC,$00E3
    LD A,8
    OUT (C),A                    ; E3 last: refresh both AllRAM halves
    ENDM

    MACRO RESTORE_MAPPING
    LD BC,$7FFD
    LD A,(saved_port7)
    OUT (C),A
    LD BC,$1FFD
    LD A,(saved_port1)
    OUT (C),A
    LD BC,$00E3
    LD A,(saved_e3)
    OUT (C),A
    ENDM

install_kernel:
    ; One copy of the relocated GIOS plus a font cache at open workstation.
    ; Execution stays in shared RAM 0; no stack access while low RAM is hidden.
    PUSH IX
    LD A,I
    DI
    PUSH AF
    POP BC
    LD A,C
    LD (install_iff),A
    CALL save_mapping
    MAP_D7
    LD HL,driver_start
    LD DE,$0100
    LD BC,backend_end-driver_start
    LDIR
    ; Undo the loaded GIOS's page relocation in the D7 copy. The patcher
    ; appends this bitmap to the executable body as non-relocatable data.
    LD HL,backend_end
    LD DE,$0100
    LD IX,(backend_end-driver_start+7)/8
.byte:
    LD B,(HL)
    INC HL
    LD C,8
.bit:
    SLA B
    JR NC,.next
    LD A,(DE)
    SUB HIGH driver_start-1
    LD (DE),A
.next:
    INC DE
    DEC C
    JR NZ,.bit
    DEC IX
    LD A,IXH
    OR IXL
    JR NZ,.byte
    ; D3 and D7 differ only in E3. Read the native 8x8 font once; no screen
    ; window, application RAM, or D6 backup page is involved.
    LD HL,$3000
    LD IX,2048
.font:
    LD BC,$00E3
    XOR A
    OUT (C),A                    ; terminal AllRAM D3
    LD E,(HL)
    LD A,8
    OUT (C),A                    ; raster AllRAM D7
    LD (HL),E
    INC HL
    DEC IX
    LD A,IXH
    OR IXL
    JR NZ,.font
    RESTORE_MAPPING
    POP IX
    LD A,(install_iff)
    AND 4
    RET Z
    EI
    RET

direct_gate:
    ; BC=callback, AF/DE/HL/IX/IY as for SCR RUN ROUTINE, but only a small
    ; register/state block is marshalled. The caller's stack can be anywhere
    ; in the TPA, including low RAM or the C000 bank that will be hidden.
    LD (saved_sp),SP
    PUSH AF
    LD A,I
    DI
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    PUSH IX
    PUSH IY
    LD HL,0
    ADD HL,SP
    LD DE,incoming_frame
    LD BC,14
    LDIR
    CALL save_mapping
    MAP_D7
    LD HL,incoming_frame
    LD DE,KERNEL_FRAME
    LD BC,14
    LDIR
    LD A,(KERNEL_FRAME+9)         ; callback address -> normalized D7 address
    SUB HIGH driver_start-1
    LD (KERNEL_FRAME+9),A
    LD HL,graphics_state
    LD DE,$1180
    LD BC,32
    LDIR
    LD A,($118D)                ; foreground/background pointers -> D7 data
    SUB HIGH driver_start-1
    LD ($118D),A
    LD A,($118F)
    SUB HIGH driver_start-1
    LD ($118F),A
    LD HL,(mode_target)         ; original GIOS self-modifies this jump
    LD A,H
    SUB HIGH driver_start-1
    LD H,A
    LD ($0DE8),HL
    LD HL,direct_return
    LD (KERNEL_RETURN),HL
    JP kernel_entry-PATCH_DELTA

kernel_entry:
    ; This entry is executed only in the normalized D7 copy. Code, data and
    ; private stack remain below 4000 while Spectrum bank 7 replaces RAM 0.
    LD BC,$7FFD
    LD A,$1F
    OUT (C),A
    LD BC,$00E3
    LD A,8
    OUT (C),A
    LD SP,KERNEL_FRAME
    POP IY
    POP IX
    POP HL
    POP DE
    POP BC
    POP AF                      ; saved IFF2; original AF follows it
    POP AF
    LD SP,KERNEL_STACK
    CALL .call_bc
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    PUSH IX
    PUSH IY
    LD (KERNEL_RESULTS),SP
    LD BC,$7FFD
    LD A,$18
    OUT (C),A                   ; restore RAM 0, keeping AllRAM D7 selected
    LD BC,$00E3
    LD A,8
    OUT (C),A
    LD HL,(KERNEL_RETURN)
    JP (HL)
.call_bc:
    PUSH BC
    RET

direct_return:
    LD HL,(KERNEL_RESULTS)
    LD DE,returned_frame
    LD BC,12
    LDIR
    LD HL,($118A)               ; return the rotated line/fill mask
    LD (line_mask),HL
    RESTORE_MAPPING
    LD SP,returned_frame
    POP IY
    POP IX
    POP HL
    POP DE
    POP BC
    LD A,(incoming_frame+10)    ; original IFF2, before DI
    AND 4
    JR Z,.disabled
    POP AF
    LD SP,(saved_sp)
    EI
    RET
.disabled:
    POP AF
    LD SP,(saved_sp)
    RET

saved_sp:       DW 0
saved_port7:    DB 0
saved_port1:    DB 0
saved_e3:       DB 0
install_iff:    DB 0
incoming_frame: DS 14,0
returned_frame: DS 12,0
backend_end:
    ASSERT backend_end-PATCH_DELTA < KERNEL_FRAME
