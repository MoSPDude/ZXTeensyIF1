; Disk FID host. All code, FIDs and supervisor allocations live in logical bank 4.
; Shared DPBs and bank-0 DPH/ALVs are the only permanent OS-side allocations.
    DEVICE ZXSPECTRUM128
    INCLUDE "layout.inc"
    ORG 0
    DS $100,0
    JP initialise
    JP install
    JP select_disk
    JP read_disk
    JP write_disk
    JP flush
    JP find
    JP private_entry

private_entry:
    LD (private_sp),SP
    PUSH BC
    LD BC,(GATE_CSP)
    LD (private_csp),BC
    POP BC
    LD (GATE_CSP),SP
    LD SP,HOST_STACK
    CALL indirect
    PUSH AF
    PUSH BC
    LD BC,(private_csp)
    LD (GATE_CSP),BC
    POP BC
    POP AF
    LD SP,(private_sp)
    RET

initialise:
    XOR A
    LD HL,allocations
    LD DE,allocations+1
    LD BC,191
    LD (HL),A
    LDIR
    LD (hook_count),A
    LD HL,0
    LD (FID_POINTER),HL
    LD (last_alias),HL
    LD (callbacks),HL
    LD (callbacks+2),HL
    LD (callbacks+4),HL
    LD HL,HOST_HEAP
    LD (heap_low),HL
    LD HL,FID_BUFFER
    LD (heap_high),HL
    LD HL,$0900
    LD (next_alias),HL
    RET

; The transient boot loader supplies the complete PRL at 8000, length in BC,
; and its 11-byte filename in FID_NAME. Never execute unvalidated input.
install:
    LD HL,(next_alias)
    LD A,H
    CP $0B
    JP NC,rejected
    LD (file_length),BC
    LD HL,$8000
    LD A,(HL)
    OR A
    JP NZ,rejected
    LD HL,($8001)
    LD A,H
    OR L
    JP Z,rejected
    LD (code_length),HL
    LD DE,32
    OR A
    SBC HL,DE
    JP C,rejected
    LD HL,($8004)
    LD (bss_length),HL
    LD A,($8100)
    CP $C3
    JP NZ,rejected
    LD HL,$8103
    LD DE,signature
    LD B,11
.sig:
    LD A,(DE)
    CP (HL)
    JP NZ,rejected
    INC DE
    INC HL
    DJNZ .sig
    LD HL,(code_length)
    LD DE,7
    ADD HL,DE
    SRL H
    RR L
    SRL H
    RR L
    SRL H
    RR L
    LD (bitmap_length),HL
    LD DE,(code_length)
    ADD HL,DE
    LD DE,$0100
    ADD HL,DE
    LD (prl_length),HL
    LD DE,(file_length)
    OR A
    SBC HL,DE
    JP C,.length_ok
    JP NZ,rejected
.length_ok:
    LD BC,(prl_length)
    LD HL,$8000
    LD DE,0
.sum:
    LD A,(HL)
    ADD A,E
    LD E,A
    JR NC,.no_carry
    INC D
.no_carry:
    INC HL
    DEC BC
    LD A,B
    OR C
    JR NZ,.sum
    LD A,($8110)
    LD L,A
    LD H,0
    EX DE,HL
    OR A
    SBC HL,DE
    LD A,($8111)
    LD E,A
    LD D,0
    OR A
    SBC HL,DE
    LD DE,($8110)
    OR A
    SBC HL,DE
    JP NZ,rejected
    CALL find_image_space
    JP NC,rejected
    LD HL,$8100
    LD DE,(loading)
    LD BC,(code_length)
    LDIR
    LD BC,(bss_length)
.zero:
    LD A,B
    OR C
    JR Z,.relocate
    XOR A
    LD (DE),A
    INC DE
    DEC BC
    JR .zero
.relocate:
    LD HL,(code_length)
    LD DE,$8100
    ADD HL,DE
    LD (bitmap),HL
    LD IX,(loading)
    LD BC,(code_length)
    LD E,0
.next:
    LD A,B
    OR C
    JR Z,.relocated
    LD A,E
    AND 7
    JR NZ,.bit
    LD HL,(bitmap)
    LD D,(HL)
    INC HL
    LD (bitmap),HL
.bit:
    SLA D
    JR NC,.advance
    LD A,(IX+0)
    CP $FF
    JR NZ,.address
    LD A,(IX-1)
    CP 11
    JP NC,rejected
    PUSH DE
    PUSH BC
    ADD A,A
    LD L,A
    LD H,0
    LD DE,supervisors
    ADD HL,DE
    LD A,(HL)
    LD (IX-1),A
    INC HL
    LD A,(HL)
    LD (IX+0),A
    POP BC
    POP DE
    JR .advance
.address:
    PUSH HL
    LD HL,(loading)
    ADD A,H
    DEC A
    LD (IX+0),A
    POP HL
.advance:
    INC IX
    INC E
    DEC BC
    JR .next
.relocated:
    LD IX,(loading)
    LD L,(IX+1)
    LD H,(IX+2)
    LD DE,(loading)
    OR A
    SBC HL,DE
    JP C,rejected
    LD DE,(code_length)
    OR A
    SBC HL,DE
    JP NC,rejected
    LD HL,(loading)
    LD DE,3
    ADD HL,DE
    EX DE,HL
    LD HL,FID_NAME
    LD BC,11
    LDIR
    LD IX,(loading)
    LD HL,(FID_POINTER)
    LD (IX+30),L
    LD (IX+31),H
    LD HL,allocations
    LD DE,allocation_snapshot
    LD BC,192
    LDIR
    CALL empty_image_record
    JP NC,rejected
    LD HL,(loading)
    LD (IY+0),L
    LD (IY+1),H
    EX DE,HL
    LD HL,(new_low)
    OR A
    SBC HL,DE
    LD (IY+2),L
    LD (IY+3),H
    LD A,(hook_count)
    LD (old_hooks),A
    LD HL,(loading)
    LD DE,0                       ; native +3 FID environment 0.0, U.K.
    LD C,3
    CALL indirect
    DI
    CALL startup_message           ; consume HL before rollback or header copies
    JR C,.retained
    LD A,(old_hooks)
    LD HL,hook_count
    CP (HL)
    JR NZ,.retained                ; never leave a registered dangling callback
    LD HL,allocation_snapshot
    LD DE,allocations
    LD BC,192
    LDIR
    CALL allocation_limit
    JP rejected
.retained:
    ; FIND FID returns a readable bank-0 header, despite the implementation
    ; residing in private AllRAM. This disk host exports no application code.
    LD IX,(loading)
    LD HL,(next_alias)
    LD (IX+20),L
    LD (IX+21),H
    PUSH HL
    LD HL,(loading)
    LD DE,FID_BUFFER
    LD BC,32
    LDIR
    LD HL,(last_alias)
    LD (FID_BUFFER+30),HL
    POP HL
    LD (last_alias),HL
    PUSH HL
    LD DE,32
    ADD HL,DE
    LD (next_alias),HL
    LD BC,$0004
    CALL $FA57
    POP HL
    LD DE,FID_BUFFER
    LD BC,32
    CALL $FA4B
    DI
    LD HL,(loading)
    LD (FID_POINTER),HL
    XOR A
    RET
rejected:
    LD A,1
    RET
signature: DB "SPECTRUMFID"

; Loaded images and supervisor blocks share one arena. Code is page-aligned,
; but its unused prefix, suffix or middle may be returned by FID_EMS.
find_image_space:
    LD HL,HOST_HEAP
    LD A,($8112)
    CP H
    JR C,.align
    JR Z,.align
    LD H,A
    LD L,0
.align:
    LD A,L
    OR A
    JR Z,.candidate
    INC H
    LD L,0
    JP Z,allocation_failed
.candidate:
    LD (loading),HL
    LD DE,(code_length)
    ADD HL,DE
    JP C,allocation_failed
    LD DE,(bss_length)
    ADD HL,DE
    JP C,allocation_failed
    LD (new_low),HL
    LD A,($8113)
    OR A
    JR Z,.arena
    CP H
    JP C,allocation_failed
    JR NZ,.arena
    LD A,L
    OR A
    JP NZ,allocation_failed
.arena:
    LD DE,FID_BUFFER
    OR A
    SBC HL,DE
    JR Z,.fits
    JP NC,allocation_failed
.fits:
    LD IX,allocations
    LD B,48
.scan:
    LD E,(IX+0)
    LD D,(IX+1)
    LD A,D
    OR E
    JR Z,.next
    LD HL,(new_low)
    OR A
    SBC HL,DE
    JR C,.next
    JR Z,.next
    LD L,(IX+2)
    LD H,(IX+3)
    ADD HL,DE
    LD DE,(loading)
    OR A
    SBC HL,DE
    JR C,.next
    JR Z,.next
    ADD HL,DE
    JP .align                    ; retry after the occupied range
.next:
    LD DE,4
    ADD IX,DE
    DJNZ .scan
    SCF
    RET

; FID_EMS returns an FF-terminated sign-on or error string in this bank.
; Print while its allocation is still present and preserve the install result.
startup_message:
    PUSH AF
.next:
    LD A,(HL)
    CP $FF
    JR Z,.done
    INC HL
    LD C,A
    PUSH HL
    CALL $FA0C                    ; CONOUT through the common bank gate
    POP HL
    JR .next
.done:
    POP AF
    RET

select_disk:
    PUSH IX
    PUSH IY
    LD A,E
    LD (login_flag),A
    LD A,C
    CP 2
    JP C,.floppy
    SUB 2
    CP 3
    JP NC,.missing
    LD (slot),A
    CALL callback_pointer
    LD A,H
    OR L
    JP Z,.missing
    LD A,C
    LD (SELECTED_DISK),A
    LD A,(login_flag)
    BIT 0,A
    JP NZ,.ready
.logon:
    CALL set_dpb
    XOR A
    LD (error_allow_ignore),A
    LD A,(SELECTED_DISK)
    LD B,A
    CALL indirect
    DI
    JP C,.logged_on
    CALL fid_error_response
    CP 'R'
    JR Z,.retry_logon
    LD A,(error_code)
    JP .missing
.retry_logon:
    LD A,(slot)
    CALL callback_pointer
    JP .logon
.logged_on:
    CALL invalidate_disk_buffers
    CALL set_dpb
    LD A,(IX+15)
    CP 2
    JP NZ,.missing
    LD A,(IX+16)
    CP 3
    JP NZ,.missing
    LD A,(IX+11)
    OR A
    JP NZ,.missing
    LD A,(IX+12)
    CP $80
    JP NZ,.missing
    LD L,(IX+5)
    LD H,(IX+6)
    LD DE,2040
    OR A
    SBC HL,DE
    JP NC,.missing
    LD A,(IX+8)
    CP 2
    JP NC,.missing
.ready:
    LD A,(slot)
    ADD A,A
    ADD A,A
    ADD A,A
    ADD A,A
    ADD A,A
    LD L,A
    LD H,8
    JP .return
.floppy:
    LD (SELECTED_DISK),A
    LD HL,DRIVER_SELECT
    CALL GATE
    JP .return
.missing:
    LD HL,0
.return:
    POP IY
    POP IX
    RET

read_disk:
    XOR A
    JR disk_io
write_disk:
    LD A,1
disk_io:
    LD (writing),A
    LD (write_type),BC
    PUSH IX
    PUSH IY
    OR A
    CALL NZ,from_dma
    LD A,1
    LD (error_allow_ignore),A
io_retry:
    LD A,(SELECTED_DISK)
    SUB 2
    CP 3
    JR NC,io_missing
    LD (slot),A
    CALL callback_pointer
    LD A,H
    OR L
    JR Z,io_missing
    LD DE,3
    LD A,(writing)
    OR A
    JR Z,.offset
    LD E,6
.offset:
    ADD HL,DE
    LD (invoke+1),HL
    CALL set_dpb
    LD A,(SELECTED_DISK)
    LD B,A
    LD A,(write_type)
    LD C,A
    LD DE,(DISK_SECTOR)
    LD HL,(DISK_TRACK)
    LD IY,FID_BUFFER
    CALL invoke
    DI
    JR NC,io_error
    OR A
    JR NZ,io_error
    LD A,(writing)
    OR A
    CALL Z,to_dma
    XOR A
    JR io_return
io_error:
    CALL fid_error_response
    CP 'R'
    JP Z,io_retry
    CP 'I'
    JR NZ,.cancel
    LD A,(writing)
    OR A
    JR NZ,.ignored
    LD HL,FID_BUFFER
    LD DE,FID_BUFFER+1
    LD BC,511
    LD (HL),$E5
    LDIR
    CALL to_dma
.ignored:
    XOR A
    JR io_return
.cancel:
    LD A,(error_code)
io_return:
    POP IY
    POP IX
    RET
io_missing:
    LD A,1
    CP A                         ; no driver exists to supply an error message
    JR io_error
invoke:
    JP 0
indirect:
    JP (HL)
callback_pointer:
    ADD A,A
    LD L,A
    LD H,0
    LD DE,callbacks
    ADD HL,DE
    LD A,(HL)
    INC HL
    LD H,(HL)
    LD L,A
    RET
set_dpb:
    PUSH HL
    LD A,(slot)
    LD HL,FID_DPBS
    LD DE,17
    OR A
    JR Z,.done
.loop:
    ADD HL,DE
    DEC A
    JR NZ,.loop
.done:
    PUSH HL
    POP IX
    POP HL
    RET
from_dma:
    LD A,(DMA_BANK)
    LD C,A
    LD B,4
    CALL $FA57
    LD DE,(DMA_ADDRESS)
    LD HL,FID_BUFFER
    JR copy_sector
to_dma:
    LD A,(DMA_BANK)
    LD B,A
    LD C,4
    CALL $FA57
    LD DE,FID_BUFFER
    LD HL,(DMA_ADDRESS)
copy_sector:
    LD BC,512
    CALL $FA4B
    DI
    RET

; A/B/F are the FID's return code, message number and retry flags. The host
; retains the return code across message callbacks, banked copies and prompts.
fid_error_response:
    LD (error_code),A
    PUSH AF
    POP HL
    BIT 6,L                      ; NC/Z: return immediately, without a prompt
    JR NZ,.cancel
    INC A                        ; media changed is returned directly as FF
    JR Z,.cancel
    LD A,(BDOS_ERROR_MODE)
    INC A
    JR Z,.cancel
    LD A,B
    LD (error_message),A
    LD A,(slot)
    CALL callback_pointer
    LD A,H
    OR L
    JR Z,.generic
    LD DE,12                     ; FID_D_MESS
    ADD HL,DE
    CALL set_dpb
    LD A,(error_message)
    LD B,A
    CALL indirect
    DI
    JR NC,.generic
    LD DE,fid_message
    LD B,50                      ; documented maximum message length
.copy:
    LD A,(HL)
    CP $FF
    JR Z,.terminated
    LD (DE),A
    INC DE
    INC HL
    DJNZ .copy
.terminated:
    XOR A
    LD (DE),A
    LD BC,$0204
    CALL $FA57
    LD DE,fid_message
    LD HL,DISK_MESSAGE
    LD BC,51
    CALL $FA4B
    LD HL,DRIVER_MESSAGE
    JR .ask
.generic:
    LD HL,DRIVER_ERROR
.ask:
    LD A,(error_allow_ignore)
    LD C,A
    LD A,7                       ; fallback message when FID_D_MESS is absent
    CALL GATE
    RET
.cancel:
    LD A,'C'
    RET

flush:
    PUSH IX
    PUSH IY
    XOR A
    LD (flush_error),A
.loop:
    LD (slot),A
.retry:
    LD A,(slot)
    CALL callback_pointer
    LD A,H
    OR L
    JR Z,.next
    LD DE,9
    ADD HL,DE
    CALL set_dpb
    LD A,1
    LD (error_allow_ignore),A
    LD A,(slot)
    ADD A,2
    LD B,A
    CALL indirect
    DI
    JR NC,.error
    OR A
    JR Z,.next
.error:
    CALL fid_error_response
    CP 'R'
    JR Z,.retry
    CP 'I'
    JR Z,.next
    LD A,(flush_error)
    OR A
    JR NZ,.next
    LD A,(error_code)
    LD (flush_error),A
.next:
    LD A,(slot)
    INC A
    CP 3
    JR C,.loop
    POP IY
    POP IX
    LD A,(flush_error)
    RET
find:
    LD HL,(FID_POINTER)
.loop:
    LD A,H
    OR L
    RET Z
    PUSH HL
    LD DE,3
    ADD HL,DE
    LD DE,FID_NAME
    LD B,8
.name:
    LD A,(DE)
    XOR (HL)
    AND $7F                     ; filename attribute bits are not part of name
    JR NZ,.next
    INC DE
    INC HL
    DJNZ .name
    POP HL
    LD DE,20
    ADD HL,DE
    LD A,(HL)
    INC HL
    LD H,(HL)
    LD L,A
    SCF
    RET
.next:
    POP HL
    LD DE,30
    ADD HL,DE
    LD A,(HL)
    INC HL
    LD H,(HL)
    LD L,A
    JR .loop

supervisors:
    DW BANK_05,BANK_68,catchup,$F99C,unsupported,hook,changed
    DW allocate,max_allocate,deallocate,unsupported
catchup:
    LD HL,DRIVER_TICK
    JP GATE
unsupported:
    LD A,1
    OR A
    RET
; The optional directory hash cache is disabled. Three fixed 510-byte ALVs
; cover the 16MB disk FID DPB; no per-drive CSV is required for hard disks.
hook:
    PUSH DE
    LD A,H
    CP 2
    JP NC,.no_memory
    CP 1
    JR NZ,.alv_ok
    LD A,L
    CP $FF
    JP Z,.no_memory
.alv_ok:
    PUSH IX
    POP DE
    LD A,D
    OR E
    JP NZ,.no_memory
    LD A,B
    CP $FF
    JR NZ,.letter
    LD B,2
.search:
    LD A,B
    SUB 2
    CALL callback_pointer
    LD A,H
    OR L
    JR Z,.letter
    INC B
    LD A,B
    CP 5
    JR C,.search
    JR .in_use
.letter:
    LD A,B
    SUB 2
    CP 3
    JR NC,.in_use
    LD (slot),A
    CALL callback_pointer
    LD A,H
    OR L
    JR NZ,.in_use
    LD A,(slot)
    ADD A,A
    LD L,A
    LD H,0
    LD DE,callbacks
    ADD HL,DE
    POP DE
    LD (HL),E
    INC HL
    LD (HL),D
    LD A,(slot)
    LD HL,$3A00
    LD DE,510
    OR A
    JR Z,.alv
.alv_loop:
    ADD HL,DE
    DEC A
    JR NZ,.alv_loop
.alv:
    LD (dph_template+16),HL
    CALL set_dpb
    PUSH IX
    POP HL
    LD (dph_template+12),HL
    LD A,(slot)
    ADD A,A
    ADD A,A
    ADD A,A
    ADD A,A
    ADD A,A
    LD L,A
    LD H,8
    LD A,(slot)
    ADD A,2
    ADD A,A
    LD E,A
    LD D,0
    PUSH HL
    LD HL,DRIVE_TABLE
    ADD HL,DE
    POP DE
    LD (HL),E
    INC HL
    LD (HL),D
    PUSH DE
    LD BC,$0004
    CALL $FA57
    POP HL
    LD DE,dph_template
    LD BC,25
    CALL $FA4B
    DI
    LD HL,hook_count
    INC (HL)
    LD A,(slot)
    ADD A,2
    LD B,A
    CALL set_dpb
    XOR A
    SCF
    RET
.no_memory:
    POP DE
    JP unsupported
.in_use:
    POP DE
    LD A,2
    OR A
    RET
changed:
    ; Fixed IMG media: record a change for BDOS's next SELDSK/login.
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    LD A,B
    SUB 2
    CP 3
    JR NC,.done
    ADD A,A
    ADD A,A
    ADD A,A
    ADD A,A
    ADD A,A
    LD L,A
    LD H,8
    LD DE,11
    ADD HL,DE
    LD (change_dph),HL
    LD BC,$0004
    CALL $FA57
    LD DE,change_flag
    LD HL,(change_dph)
    LD BC,1
    CALL $FA4B
    LD A,$FF
    LD (BDOS_DISK_CHANGE),A
.done:
    POP HL
    POP DE
    POP BC
    POP AF
    RET
allocate:
    PUSH IX
    PUSH IY
    CALL allocate_block
    POP IY
    POP IX
    RET
allocate_block:
    LD A,D
    OR E
    JP Z,allocation_failed
    LD (allocation_size),DE
    CALL empty_record
    JP NC,allocation_failed
    LD HL,FID_BUFFER
    OR A
    SBC HL,DE
    JP C,allocation_failed
.rescan:
    LD (allocation_start),HL
    LD DE,(heap_low)
    OR A
    SBC HL,DE
    JP C,allocation_failed
    LD IX,allocations
    LD B,48                      ; private blocks and retained image fragments
.scan:
    LD E,(IX+0)
    LD D,(IX+1)
    LD A,D
    OR E
    JR Z,.next
    LD HL,(allocation_start)
    PUSH DE
    LD DE,(allocation_size)
    ADD HL,DE
    POP DE
    OR A
    SBC HL,DE
    JR C,.next
    JR Z,.next                     ; new end <= old start: no overlap
    PUSH BC
    LD L,(IX+2)
    LD H,(IX+3)
    ADD HL,DE
    LD DE,(allocation_start)
    OR A
    SBC HL,DE
    POP BC
    JR C,.next
    JR Z,.next                     ; old end <= new start: no overlap
    LD L,(IX+0)
    LD H,(IX+1)
    LD DE,(allocation_size)
    OR A
    SBC HL,DE
    JP C,allocation_failed
    JR .rescan
.next:
    LD DE,4
    ADD IX,DE
    DJNZ .scan
    LD HL,(allocation_start)
    LD DE,(allocation_size)
    LD (IY+0),L
    LD (IY+1),H
    LD (IY+2),E
    LD (IY+3),D
    CALL allocation_limit
    LD HL,(allocation_start)
    SCF
    RET
allocation_failed:
    OR A
    RET
empty_record:
    PUSH DE
    LD IY,allocations
    LD B,16
    JR empty_range_record
empty_image_record:
    PUSH DE
    LD IY,images
    LD B,32
empty_range_record:
.loop:
    LD A,(IY+0)
    OR (IY+1)
    JR Z,.found
    LD DE,4
    ADD IY,DE
    DJNZ .loop
    POP DE
    OR A
    RET
.found:
    POP DE
    SCF
    RET
allocation_limit:
    ; The next FID must fit below every outstanding private allocation.
    LD HL,FID_BUFFER
    LD (heap_high),HL
    LD IX,allocations
    LD B,16
.loop:
    LD L,(IX+0)
    LD H,(IX+1)
    LD A,H
    OR L
    JR Z,.next
    LD DE,(heap_high)
    OR A
    SBC HL,DE
    JR NC,.next
    ADD HL,DE
    LD (heap_high),HL
.next:
    LD DE,4
    ADD IX,DE
    DJNZ .loop
    RET
deallocate:
    PUSH IX
    PUSH IY
    LD (allocation_start),HL
    LD (allocation_size),DE
    LD A,D
    OR E
    JP Z,.done
    ADD HL,DE
    JP C,.done
    LD (release_end),HL
    LD IX,allocations
    LD B,48
.scan:
    LD E,(IX+0)
    LD D,(IX+1)
    LD A,D
    OR E
    JR Z,.next
    LD HL,(allocation_start)
    OR A
    SBC HL,DE
    JR C,.next
    LD (release_prefix),HL
    LD L,(IX+2)
    LD H,(IX+3)
    ADD HL,DE
    LD DE,(release_end)
    OR A
    SBC HL,DE
    JR C,.next
    LD (release_suffix),HL
    LD DE,(release_prefix)
    LD A,D
    OR E
    JR NZ,.prefix
    LD A,H
    OR L
    JR NZ,.suffix                ; returned prefix: retain the right part
    LD (IX+0),0
    LD (IX+1),0
    JR .released
.prefix:
    LD A,H
    OR L
    JR Z,.left                   ; returned suffix: retain the left part
    PUSH IX
    POP HL
    LD DE,images
    OR A
    SBC HL,DE
    JR NC,.image
    CALL empty_record
    JR .split
.image:
    CALL empty_image_record
.split:
    JR NC,.done                  ; bounded bookkeeping, leave the block intact
    LD HL,(release_end)
    LD (IY+0),L
    LD (IY+1),H
    LD HL,(release_suffix)
    LD (IY+2),L
    LD (IY+3),H
.left:
    LD HL,(release_prefix)
    JR .size
.suffix:
    LD DE,(release_end)
    LD (IX+0),E
    LD (IX+1),D
.size:
    LD (IX+2),L
    LD (IX+3),H
.released:
    CALL allocation_limit
    JR .done
.next:
    LD DE,4
    ADD IX,DE
    DEC B
    JP NZ,.scan
.done:
    POP IY
    POP IX
    RET
max_allocate:
    PUSH IX
    PUSH IY
    LD HL,0
    LD (largest_gap),HL
    LD HL,(heap_low)
    CALL evaluate_gap
    LD IX,allocations
    LD B,48
.loop:
    LD L,(IX+0)
    LD H,(IX+1)
    LD A,H
    OR L
    JR Z,.next
    LD E,(IX+2)
    LD D,(IX+3)
    ADD HL,DE
    PUSH BC
    CALL evaluate_gap
    POP BC
.next:
    LD DE,4
    ADD IX,DE
    DJNZ .loop
    LD DE,(largest_gap)
    POP IY
    POP IX
    CALL allocate
    LD DE,(largest_gap)
    RET

evaluate_gap:
    LD (gap_start),HL
    LD HL,FID_BUFFER
    LD (gap_end),HL
    LD IY,allocations
    LD B,48
.loop:
    LD L,(IY+0)
    LD H,(IY+1)
    LD A,H
    OR L
    JR Z,.next
    LD DE,(gap_start)
    OR A
    SBC HL,DE
    JR C,.next
    ADD HL,DE
    LD DE,(gap_end)
    OR A
    SBC HL,DE
    JR NC,.next
    ADD HL,DE
    LD (gap_end),HL
.next:
    LD DE,4
    ADD IY,DE
    DJNZ .loop
    LD HL,(gap_end)
    LD DE,(gap_start)
    OR A
    SBC HL,DE
    LD DE,(largest_gap)
    PUSH HL
    OR A
    SBC HL,DE
    POP HL
    RET C
    LD (largest_gap),HL
    RET

    INCLUDE "buffer_login.asm"

dph_template:
    DW 0,0,0,0,0
    DB 0,0
    DW 0,0,0,DIR_HEAD,DATA_HEAD,$FFFF
    DB 0
callbacks: DS 6,0
next_alias: DW 0
last_alias: DW 0
heap_low: DW 0
heap_high: DW 0
file_length: DW 0
code_length: DW 0
bss_length: DW 0
bitmap_length: DW 0
prl_length: DW 0
bitmap: DW 0
loading: DW 0
new_low: DW 0
old_hooks: DB 0
hook_count: DB 0
slot: DB 0
login_flag: DB 0
writing: DB 0
flush_error: DB 0
error_code: DB 0
error_message: DB 0
error_allow_ignore: DB 0
fid_message: DS 51,0
write_type: DW 0
change_dph: DW 0
change_flag: DB $FF
allocations: DS 64,0
images: DS 128,0
allocation_snapshot: DS 192,0
allocation_size: DW 0
allocation_start: DW 0
release_end: DW 0
release_prefix: DW 0
release_suffix: DW 0
largest_gap: DW 0
gap_start: DW 0
gap_end: DW 0
private_sp: DW 0
private_csp: DW 0
host_end:
    ASSERT $ < $2000

HOST_HEAP EQU (host_end+$FF)&$FF00
