; ZXTeensyIF1 CP/M Plus milestone 1: common BIOS, F400 BDOS remains original.
; No Locomotive or Digital Research machine code is included in this source.
    DEVICE ZXSPECTRUM128
    INCLUDE "layout.inc"
    ORG BIOS_BASE
    JP cold_boot
    JP warm_boot
    JP console_status
    JP console_input
    JP console_output
    JP console_output             ; LIST (console for this milestone)
    JP empty                      ; AUXOUT
    JP no_input                   ; AUXIN
    JP home
    JP select_disk
    JP set_track
    JP set_sector
    JP set_dma
    JP read_disk
    JP write_disk
    JP ready
    JP translate_sector
    JP ready
    JP no_input
    JP ready
    JP device_table
    JP empty
    JP drive_table
    JP empty                      ; MULTIO is an optional transfer hint
    JP flush_disk
    JP move_memory
    JP time_service
    JP bank_select
    JP set_dma_bank
    JP extended_move
    JP user_function              ; supported +3 XBIOS calls through USERF
    JP unsupported
    JP unsupported
    ASSERT $ == GATE
    JP driver_gate
    JP bank_select
    JP DRIVER_TRANSFER
    JP interrupt_return
    JP host_gate
    JP xbios_entry
    JP screen_gate

cold_boot:
    DI
    LD SP,STACK_TOP
    XOR A
    CALL bank_select
    LD HL,$8000
    LD ($F9BE),HL                 ; SCB console input vector
    LD ($F9C0),HL                 ; SCB console output vector
    LD HL,0
    LD ($F9C2),HL
    LD ($F9C4),HL
    LD ($F9C6),HL
    LD A,CONSOLE_COLUMNS-1
    LD ($F9B6),A                  ; console columns minus one
    LD A,23
    LD ($F9B8),A                  ; console page length minus one
    LD A,$40
    LD ($F9FA),A                  ; common base = 4000, not C000
    LD A,1
    LD (MULTI_COUNT),A
    LD ($F9E6),A
    LD HL,DRIVER_INIT
    CALL driver_gate
    ; All executable banks have a harmless IM 1 / NMI vector.
    LD E,7
.vectors:
    LD A,E
    CP 3
    JR Z,.skip_vector
    CALL bank_select
    LD A,$C3
    LD ($0038),A
    LD HL,$FA6C
    LD ($0039),HL
    LD HL,$45ED
    LD ($0066),HL
.skip_vector:
    DEC E
    JP P,.vectors
    IM 1
    LD HL,HOST_INIT
    CALL host_gate
    LD A,1
    LD (BOOT_FIDS),A
    JP warm_boot

warm_boot:
    DI
    LD SP,STACK_TOP
    CALL flush_disk
    XOR A
    CALL bank_select
    LD BC,$0103                   ; cached CCP bank 3 -> TPA bank 1
    CALL extended_move
    LD DE,0
    LD HL,$0100
    LD BC,$0C80
    CALL move_memory
    LD A,1
    CALL bank_select
    LD A,$C3
    LD (0),A
    LD (5),A
    LD HL,$FA03
    LD (1),HL
    LD HL,$F406
    LD (6),HL
    LD BC,$0080
    CALL set_dma
    XOR A
    LD (4),A
    LD C,A
    LD SP,BDOS_BASE
    EI
    LD A,(BOOT_FIDS)
    OR A
    JP NZ,$6000                   ; one-time BDOS-based FID loader, before CCP
    JP $0100

console_status:
    LD HL,DRIVER_CONST
    JP driver_gate
console_input:
    LD HL,DRIVER_CONIN
    JP driver_gate
console_output:
    LD HL,DRIVER_CONOUT
    JP driver_gate
read_disk:
    LD A,(SELECTED_DISK)
    CP 2
    LD HL,HOST_READ
    JP NC,host_gate
    LD HL,DRIVER_READ
    JP driver_gate
write_disk:
    LD A,(SELECTED_DISK)
    CP 2
    LD HL,HOST_WRITE
    JP NC,host_gate
    LD HL,DRIVER_WRITE

driver_gate:
    PUSH AF
    LD A,I
    DI
    PUSH AF
    LD (CALL_TARGET),HL
    LD A,2
    JR bank_gate
host_gate:
    PUSH AF
    LD A,I
    DI
    PUSH AF
    LD (CALL_TARGET),HL
    LD A,4
bank_gate:
    ; A common stack frame makes this gate safe for nested XBIOS/FID calls
    ; and timer interrupts. No active call relies on the temporary variables.
    LD (GATE_BANK),A
    LD (SAVED_SP),SP
    LD (GATE_HL),HL
    LD HL,0
    ADD HL,SP
    LD A,H
    CP $40
    JR NC,.common_stack
    LD HL,(GATE_CSP)
.common_stack:
    LD SP,HL
    LD HL,(GATE_HL)
    PUSH IX
    PUSH HL
    PUSH DE
    PUSH BC
    PUSH IY
    LD IX,(SAVED_SP)
    LD C,(IX+2)
    LD B,(IX+3)
    PUSH BC                       ; original AF
    LD HL,(GATE_CSP)
    PUSH HL                       ; previous common-stack frontier
    LD HL,(SAVED_SP)
    INC HL
    INC HL
    INC HL
    INC HL
    PUSH HL                       ; caller SP, excluding AF/IFF saves
    LD A,(CURRENT_BANK)
    LD C,A
    LD B,(IX+0)
    PUSH BC                       ; old bank and IFF2
    LD (GATE_CSP),SP
    LD A,(GATE_BANK)
    CALL bank_select
    LD HL,(CALL_TARGET)
    LD A,(GATE_BANK)
    CP 4
    JR NZ,.target_ready
    LD HL,HOST_ENTRY
.target_ready:
    LD (gate_indirect+5),HL
    LD HL,6
    ADD HL,SP
    LD SP,HL
    POP AF
    POP IY
    POP BC
    POP DE
    POP HL
    POP IX
    LD SP,(GATE_CSP)
    CALL gate_preserve
    DI
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    PUSH IX
    PUSH IY
    LD IX,12
    ADD IX,SP
    LD A,(IX+0)
    CALL bank_select
    LD C,(IX+2)
    LD B,(IX+3)
    LD (SAVED_SP),BC
    LD C,(IX+4)
    LD B,(IX+5)
    LD (GATE_CSP),BC
    LD A,(IX+1)
    LD (GATE_BANK),A
    POP IY
    POP IX
    POP HL
    POP DE
    POP BC
    POP AF
    LD SP,(SAVED_SP)
    PUSH AF
    LD A,(GATE_BANK)
    BIT 2,A
    JR Z,.disabled
    POP AF
    EI
    RET
.disabled:
    POP AF
    RET
gate_indirect:
    LD (GATE_CSP),SP
    JP 0
indirect:
    JP (HL)

bank_select:
    PUSH BC
    LD (CURRENT_BANK),A
    LD A,I
    DI
    PUSH AF                       ; keep the caller's interrupt-enable state
    LD A,(CURRENT_BANK)
    AND 1
    RLCA
    RLCA
    RLCA
    RLCA
    OR 8                          ; Spectrum RAM 0, shadow screen 7
    LD (BANK_05),A
    LD BC,$7FFD
    OUT (C),A
    LD A,(BANK_68)
    AND $FB
    PUSH HL
    LD HL,CURRENT_BANK
    BIT 2,(HL)
    POP HL
    JR Z,.port1
    OR 4
.port1:
    LD (BANK_68),A
    LD BC,$1FFD
    OUT (C),A
    ; Finish with E3 for compatibility with firmware versions where 7FFD
    ; alone left romPtr on the previous lower 8K page.
    LD A,(CURRENT_BANK)
    AND 2
    RLCA
    RLCA
    LD BC,$00E3
    OUT (C),A                     ; pages D0/D1 or D4/D5
    POP AF
    JP PO,.disabled
    POP BC
    EI
    RET
.disabled:
    POP BC
    RET

home:
    LD BC,0
set_track:
    LD (DISK_TRACK),BC
    RET
set_sector:
    LD (DISK_SECTOR),BC
    RET
set_dma:
    LD (DMA_ADDRESS),BC
    LD A,(CURRENT_BANK)
set_dma_bank:
    LD (DMA_BANK),A
    RET
flush_disk:
    PUSH BC
    LD BC,$1FFD
    LD A,(BANK_68)
    AND $F7
    LD (BANK_68),A
    OUT (C),A                     ; Teensy flushes DSK writes when the motor stops
    POP BC
    LD HL,HOST_FLUSH
    JP host_gate
select_disk:
    LD HL,HOST_SELECT
    JP host_gate
translate_sector:
    LD H,B
    LD L,C
    RET
device_table:
    LD HL,devices
    RET
drive_table:
    LD HL,drives
    RET
extended_move:
    LD (MOVE_BANKS),BC             ; C source, B destination
    LD A,1
    LD (MOVE_FLAG),A
    RET

move_memory:
    LD A,(MOVE_FLAG)
    OR A
    JR NZ,.extended
    EX DE,HL
    LDIR
    EX DE,HL
    RET
.extended:
    LD A,I
    DI                            ; protect shared context before saving SP
    LD (SAVED_SP),SP
    LD (MOVE_SOURCE),DE
    LD (MOVE_DEST),HL
    LD (MOVE_COUNT),BC
    PUSH AF
    POP BC
    LD A,C
    LD (MOVE_FLAG),A               ; saved IFF2 until the copy completes
    LD HL,0
    ADD HL,SP
    LD A,H
    CP $40
    JR NC,.stack_ready
    LD HL,(GATE_CSP)
.stack_ready:
    LD SP,HL
    LD A,(CURRENT_BANK)
    LD (MOVE_OLD_BANK),A
.next:
    LD BC,(MOVE_COUNT)
    LD A,B
    OR C
    JR Z,.done
    LD A,B
    OR A
    JR NZ,.full
    LD A,C
    CP 128
    JR C,.size
.full:
    LD BC,128
.size:
    LD (MOVE_CHUNK),BC
    LD A,(MOVE_BANKS)
    CALL bank_select
    LD HL,(MOVE_SOURCE)
    LD DE,BOUNCE
    LDIR
    LD (MOVE_SOURCE),HL
    LD A,(MOVE_BANKS+1)
    CALL bank_select
    LD HL,BOUNCE
    LD DE,(MOVE_DEST)
    LD BC,(MOVE_CHUNK)
    LDIR
    LD (MOVE_DEST),DE
    LD HL,(MOVE_COUNT)
    LD BC,(MOVE_CHUNK)
    OR A
    SBC HL,BC
    LD (MOVE_COUNT),HL
    JR .next
.done:
    LD A,(MOVE_OLD_BANK)
    CALL bank_select
    LD DE,(MOVE_SOURCE)
    LD HL,(MOVE_DEST)
    LD BC,0
    LD SP,(SAVED_SP)
    LD A,(MOVE_FLAG)
    BIT 2,A
    LD A,0
    LD (MOVE_FLAG),A
    RET Z
    EI
    RET

ready:
    LD A,$FF
    RET
unsupported:
    LD HL,0
no_input:
success:
    XOR A
empty:
    RET
time_service:
    LD HL,DRIVER_TIME
    JP driver_gate

user_function:
    PUSH AF
    LD A,I
    PUSH AF
    PUSH IX
    LD IX,0
    ADD IX,SP
    PUSH HL
    PUSH DE
    LD L,(IX+6)
    LD H,(IX+7)
    LD E,(HL)
    INC HL
    LD D,(HL)
    INC HL
    LD (IX+6),L
    LD (IX+7),H
    ; Empty KM KT GET polls must leave interrupts available. A full nested
    ; bank-0/driver round trip masks most of DISCKIT's polling loop, starving
    ; the keyboard scans, cursor and motor timeout that it is waiting for.
    LD A,D
    OR A
    JR NZ,.banked
    LD A,E
    CP $DA
    JR NZ,.banked
    LD A,(KEY_COUNT)
    OR A
    JR NZ,.banked
    POP DE
    POP HL
    POP IX
    POP AF                        ; discard the saved IFF without changing it
    POP AF
    XOR A
    LD (KEY_PENDING),A
    LD A,(KEY_LOCKS)
    LD B,A
    LD A,(KEY_SHIFTS)
    OR B
    LD B,A
    RET
.banked:
    DI
    LD (CALL_TARGET),DE
    POP DE
    POP HL
    POP IX
    XOR A                         ; USERF calls any bank-0 routine
    JP bank_gate

xbios_entry:
    ; Bank-0 stubs supply A=function and HL=driver, with AF/HL on the stack.
    LD (XBIOS_FUNCTION),A
    LD (CALL_TARGET),HL
    POP HL
    LD A,I
    DI
    PUSH AF
    LD A,2
    JP bank_gate

screen_gate:
    PUSH AF
    LD A,I
    DI
    PUSH AF
    PUSH HL
    LD HL,SCREEN_ENTRY
    LD (CALL_TARGET),HL
    POP HL
    LD A,5
    JP bank_gate

interrupt_return:
    ; The CCP loader has a small stack beside live workspace. Interrupts must
    ; not place a gate frame there. MOVE owns FF80..FFFF only with DI, so
    ; the same common buffer is available as a dedicated interrupt stack.
    LD (IRQ_SP),SP
    LD SP,0
    PUSH HL
    LD HL,DRIVER_TICK              ; timer preserves all other registers
    CALL driver_gate
    POP HL
    LD SP,(IRQ_SP)
    EI
    RETI

    ASSERT $ <= DPB_A
    DS DPB_A-$,0
    DUP 2
    DW 36
    DB 3,7,0
    DW 174,63
    DB $C0,0
    DW 16,1
    DB 2,3
    DB 0,40,9,1                   ; sidedness, tracks, sectors, first ID
    DW 512
    DB $2A,$52,$60,0              ; gaps, MFM/skip flags, freeze flag
    EDUP
    ASSERT $ == DIR_HEAD
dir_head:
    DW $0600
data_head:
    DW $060F
drives:
    ASSERT $ == DRIVE_TABLE
    DW DPH_A,DPH_B
    DS 28,0
devices:
    DB "CRT   ",$03,0
    DB 0
gate_preserve:
    ; Keep alternate registers outside the callee's register interface.
    ; This common-memory gap also serves USERF routines in arbitrary banks.
    EXX
    PUSH BC
    PUSH DE
    PUSH HL
    EXX
    EX AF,AF'
    PUSH AF
    EX AF,AF'
    CALL gate_indirect
    DI
    EX AF,AF'
    POP AF
    EX AF,AF'
    EXX
    POP HL
    POP DE
    POP BC
    EXX
    RET
    ASSERT $ <= $FE80
    DS $FE80-$,0
    DS 128,0                      ; common BIOS stack
    ASSERT $ == CURRENT_BANK
    DS GATE_CSP-$,0
    DW GATE_STACK
    DS FLOPPY_COUNT-$,0
    DB 1
    DS BUFFER_TABLE-$,0
    DB 0
    DW $0100,$0400                 ; contiguous bank-0 directory/data buffers
    DB $FF
    DS $10000-$,0                 ; context and MOVE/IRQ scratch buffer
    ASSERT $ == $10000
