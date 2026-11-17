; Run the user's original +3 terminal in a spare AllRAM page. The patcher
; extracts its code, messages and fonts locally; none are distributed here.
    DEVICE ZXSPECTRUM128
    INCLUDE "layout.inc"
    ORG 0
    DS $38,0
    JP $FA6C
    DS $54-$,0
border_output:
    OR 8
    OUT ($FE),A
    RET
    DS $1E2-$,0
    INCBIN "terminal_prefix.bin"
    DS $900-$,0
    INCBIN "terminal_core.bin"
    DS $2400-$,0

select_surface:
    ; Native A=0/2 selects the left/right logical screen. RAM 7 remains
    ; the visible ULA screen; RAM 6 stores the other half of an 80-column page.
    PUSH BC
    LD C,A
    LD A,(callback_active)
    OR A
    LD A,C
    JR Z,.normal
    OR A
    LD A,$1D
    JR Z,.map                    ; main screen lives at 4000 during callback
    LD A,C
.normal:
    RRCA
    LD B,A
    LD A,(surface_front)
    XOR B
    AND 1
    LD A,$1F
    JR Z,.visible
    DEC A
    JR .map
.visible:
.map:
    LD BC,$7FFD
    OUT (C),A
    POP BC
    RET

finish_surface:
    LD A,(callback_active)
    OR A
    RET NZ
    LD A,(surface_shadow)
    RRCA
    RRCA
    RRCA
    AND 1
    LD B,A
    LD A,(surface_front)
    CP B
    RET Z
    LD A,B
    LD (surface_front),A
    LD HL,$C000
    LD (surface_address),HL
    LD A,32
    LD (surface_chunks),A
.swap:
    LD BC,$7FFD
    LD A,$1F
    OUT (C),A
    LD HL,(surface_address)
    LD DE,$2500
    LD BC,256
    LDIR
    LD BC,$7FFD
    LD A,$1E
    OUT (C),A
    LD HL,(surface_address)
    LD DE,$2600
    LD BC,256
    LDIR
    LD HL,$2500
    LD DE,(surface_address)
    LD BC,256
    LDIR
    LD BC,$7FFD
    LD A,$1F
    OUT (C),A
    LD HL,$2600
    LD DE,(surface_address)
    LD BC,256
    LDIR
    LD (surface_address),DE
    LD HL,surface_chunks
    DEC (HL)
    JR NZ,.swap
    RET
    ASSERT $ <= $2500
    DS $27EE-$,0
callback_sp: DW 0
    DS $27F0-$,0
surface_shadow: DB 0
surface_front: DB 0
terminal_stack: DW $2F00
entry_sp: DW 0
terminal_frame: DW 0
surface_address: DW 0
surface_chunks: DB 0
callback_active: DB 0
status_dirty: DB 1
last_mode: DB 0
last_status: DB 0
    ASSERT $ <= SCREEN_ENTRY
    DS SCREEN_ENTRY-$,0

screen_entry:
    LD (entry_sp),SP
    LD SP,(terminal_stack)
    PUSH IY
    PUSH IX
    PUSH HL
    PUSH DE
    PUSH BC
    PUSH AF
    LD HL,(entry_sp)
    PUSH HL
    LD HL,(GATE_CSP)
    PUSH HL
    LD HL,(entry_sp)
    LD (GATE_CSP),HL
    LD (terminal_frame),SP
    LD (terminal_stack),SP
    LD A,(FLOPPY_COUNT)
    DEC A
    NEG                           ; native 0075: 00 = one, FF = two floppies
    LD HL,$0075
    CP (HL)
    JR Z,.mapping
    LD (HL),A
    LD A,1
    LD (status_dirty),A
.mapping:
    LD A,(FLOPPY_MAPPED)
    LD HL,$1DE7
    CP (HL)
    JR Z,.dispatch
    LD (HL),A
    LD A,1
    LD (status_dirty),A
.dispatch:
    LD A,(SCREEN_FUNCTION)
    LD C,A
    CP $C2
    JR Z,.redraw
    CP $C8
    JR NZ,.function
.redraw:
    LD A,1
    LD (status_dirty),A
.function:
    LD A,C
    LD HL,$1025                  ; CONOUT, C = character
    CP $FF
    JR Z,.target
    LD HL,terminal_tick
    CP $FE
    JR Z,.target
    LD HL,$1010                  ; original cold terminal initialisation
    CP $FD
    JR Z,.target
    LD HL,$0C73                  ; CAPS+SYMBOL+ENTER: change visible half
    CP $FC
    JR Z,.target
    LD HL,status_message
    CP $FB
    JR Z,.target
    LD HL,$1665                  ; dismiss rotating messages and restore status
    CP $FA
    JR Z,.target
    LD HL,native_message
    CP $F9
    JR Z,.target
    LD HL,$163C                  ; native prompt/invalid-response bleep
    CP $F8
    JR Z,.target
    LD HL,screen_callback
    CP $E9
    JR Z,.target
    SUB $BF
    LD L,A
    LD H,0
    LD DE,terminal_targets
    ; The seven TE calls are spaced three bytes apart in the public block.
    LD B,0
.divide:
    SUB 3
    JR C,.index
    INC B
    JR .divide
.index:
    LD A,B
    ADD A,A
    LD L,A
    LD H,0
    ADD HL,DE
    LD E,(HL)
    INC HL
    LD D,(HL)
    EX DE,HL
.target:
    LD (terminal_call+1),HL
    LD HL,4
    ADD HL,SP
    LD SP,HL
    POP AF
    POP BC
    POP DE
    POP HL
    POP IX
    POP IY
    LD SP,(terminal_frame)
    EXX
    PUSH BC
    PUSH DE
    PUSH HL
    EXX
    EX AF,AF'
    PUSH AF
    EX AF,AF'
    CALL terminal_call
    EX AF,AF'
    POP AF
    EX AF,AF'
    EXX
    POP HL
    POP DE
    POP BC
    EXX
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    PUSH IX
    PUSH IY
    LD A,($1F25)
    LD HL,last_mode
    CP (HL)
    LD (HL),A
    JR NZ,.status
    LD A,($1DE6)
    LD HL,last_status
    CP (HL)
    LD (HL),A
    JR NZ,.status
    LD A,(status_dirty)
    OR A
    JR Z,.surface
.status:
    XOR A
    LD (status_dirty),A
    CALL $1674                   ; native status-line messages and layout
.surface:
    CALL finish_surface
    LD BC,$7FFD
    LD A,$18                     ; common RAM 0 before touching the SCB/gate
    OUT (C),A
    LD HL,($218B)
    LD (CURSOR_POSITION),HL
    LD HL,($2184)
    LD A,L
    LD ($F9B6),A
    LD A,H
    LD ($F9B8),A
    LD IX,12
    ADD IX,SP
    LD L,(IX+0)
    LD H,(IX+1)
    LD (GATE_CSP),HL
    LD L,(IX+2)
    LD H,(IX+3)
    LD (entry_sp),HL
    PUSH IX
    POP HL
    LD DE,16
    ADD HL,DE
    LD (terminal_stack),HL
    POP IY
    POP IX
    POP HL
    POP DE
    POP BC
    POP AF
    LD SP,(entry_sp)
    RET
terminal_call:
    JP 0
terminal_targets:
    DW $11EA,$101D,$15FF,$161A,$1455,$1460,$1466

status_message:
    LD HL,SCREEN_MESSAGE
.terminate:
    LD A,(HL)
    OR A
    JR Z,.ready
    INC HL
    JR .terminate
.ready:
    LD (HL),$FF
    LD DE,SCREEN_MESSAGE
    LD B,$14                     ; native FID text followed by recovery choices
native_message:
    XOR A
    LD (status_dirty),A
    JP $1625                     ; format, route, bleep and start status rotation

terminal_tick:
    CALL $119A                   ; original delayed, steady cursor
    JP $172B                     ; native rotating status-message timer

screen_callback:
    PUSH AF                     ; pass the caller's A and flags to either path
    LD A,(callback_active)
    OR A
    JR Z,.prepare
    POP AF
    JP call_bc                   ; a nested callback already has this window
.prepare:
    PUSH BC
    PUSH DE
    PUSH HL
    XOR A
    CALL copy_ram5               ; preserve all 16K of shared TPA in AllRAM D6
    XOR A
    CALL select_surface          ; the native main (left) screen
    LD HL,$C000
    LD DE,$4000
    LD BC,$2000
    LDIR
    LD BC,$7FFD
    LD A,$18
    OUT (C),A
    LD A,1
    LD (callback_active),A
    POP HL
    POP DE
    POP BC
    POP AF
    LD (callback_sp),SP
    LD (terminal_stack),SP
    ; The upper half of saved RAM 5 supplies a common callback stack. It is
    ; outside both native screen surfaces and is restored with the TPA.
    LD SP,$8000
    LD (GATE_CSP),SP
    CALL call_bc
    DI
    LD SP,(callback_sp)
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    XOR A
    CALL select_surface          ; still aliases the temporary RAM-5 window
    ; Select the actual main surface before clearing callback_active.
    XOR A
    LD (callback_active),A
    CALL select_surface
    LD HL,$4000
    LD DE,$C000
    LD BC,$2000
    LDIR
    LD BC,$7FFD
    LD A,$18
    OUT (C),A
    LD A,1
    CALL copy_ram5
    POP HL
    POP DE
    POP BC
    POP AF
    RET
call_bc:
    PUSH BC
    RET

copy_ram5:
    LD (COPY_DIRECTION),A
    LD HL,copy_code
    LD DE,$FF80
    LD BC,copy_code_end-copy_code
    LDIR
    JP $FF80
copy_code:
    ; A stackless common trampoline can restore even a caller's stack in
    ; 4000..7FFF. DI also makes MOVE's/IRQ's shared scratch buffer available.
    DISP $FF80
    DI
    LD BC,$7FFD
    LD A,8
    OUT (C),A
    LD BC,$00E3
    LD A,8
    OUT (C),A                    ; AllRAM D6, top common RAM 0
    LD HL,$4000
    LD DE,0
    LD A,(COPY_DIRECTION)
    OR A
    JR Z,.copy
    EX DE,HL
.copy:
    LD BC,$4000
    LDIR
    LD BC,$7FFD
    LD A,$18
    OUT (C),A
    LD BC,$00E3
    XOR A
    OUT (C),A                    ; AllRAM D3 (logical bank 5)
    JP copy_done
    ENT
copy_code_end:
copy_done:
    RET
    ASSERT $ < $2D00
    DS $3000-$,0
    INCBIN "font.bin"
    ASSERT $ == $4000
