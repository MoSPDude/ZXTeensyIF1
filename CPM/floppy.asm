; +3 floppy BIOS: two logical drives, native XDPBs and recoverable I/O.
; The original pure format routines are supplied locally by the patcher.
floppy_initialise:
    LD A,$FF
    LD (current_tracks),A
    LD (current_tracks+1),A
    XOR A
    LD (mapped_drive),A
    LD (FLOPPY_MAPPED),A
    LD (floppy_busy),A
    LD (motor_timeout),A
    LD (motor_timeout+1),A
    LD A,1
    LD (FLOPPY_COUNT),A
    CALL floppy_motor_start
    LD HL,specify
    LD B,3
    CALL send_command
    LD C,1
    CALL floppy_drive_status
    AND $60                       ; an unfitted B: has neither READY nor WP
    JR Z,.single
    LD A,2
    LD (FLOPPY_COUNT),A
.single:
    JP floppy_motor_stop

floppy_select:
    LD HL,floppy_bios_select
    JR floppy_private
disk_read:
    XOR A
    JR disk_io
disk_write:
    LD A,1
disk_io:
    LD (write_operation),A
    LD HL,floppy_bios_io
floppy_private:
    LD (disk_call+1),HL
    LD (floppy_sp),SP
    LD HL,(GATE_CSP)
    LD (floppy_csp),HL
    LD (GATE_CSP),SP
    LD SP,$4000
    PUSH IX
    PUSH IY
    LD A,1
    LD (floppy_busy),A
    CALL disk_call
    CALL floppy_dd_timeout
    PUSH AF
    PUSH HL
    XOR A
    LD (floppy_busy),A
    LD HL,(floppy_csp)
    LD (GATE_CSP),HL
    POP HL
    POP AF
    POP IY
    POP IX
    LD SP,(floppy_sp)
    RET
disk_call:
    JP 0

floppy_bios_select:
    LD A,E
    LD (floppy_login),A
    LD A,(SELECTED_DISK)
    LD (logical_drive),A
    CALL floppy_choose_unit
    LD A,(FLOPPY_COUNT)
    CP 2
    JR Z,.login
    LD A,(mapped_drive)
    LD B,A
    LD A,(logical_drive)
    CP B
    JR Z,.login
    LD A,(logical_drive)
    LD (FLOPPY_MAPPED),A
    CALL floppy_motor_stop
    LD B,$10                      ; native single-drive swap instructions
    LD A,(logical_drive)
    LD C,A
    CALL floppy_show_native
    CALL keyboard_discard
    CALL console_input
    CALL floppy_restore_status
    LD A,(logical_drive)
    LD (mapped_drive),A
    LD A,$FF
    LD (current_tracks),A
.login:
    CALL floppy_bios_xdpb
    LD A,(floppy_login)
    BIT 0,A
    JR NZ,.ready
    BIT 7,(IX+26)                 ; BIOS honours the user-patched freeze flag
    JR NZ,.ready
.retry:
    LD C,0
    LD A,(current_unit)
    LD C,A
    LD A,$FF
    CALL floppy_forget_track
    CALL floppy_login_disc
    JR NC,.login_error
    CALL floppy_check_drive
    JR NC,.login_error
    CALL invalidate_disk_buffers
    JR .ready
.login_error:
    LD (last_error),A
    XOR A
    LD (allow_ignore),A
    CALL floppy_error_prompt
    CP 'R'
    JR Z,.retry
    LD HL,0
    RET
.ready:
    LD HL,DPH_A
    LD A,(logical_drive)
    OR A
    RET Z
    LD HL,DPH_B
    RET

; The internal A: is single-sided/40-track; external B: accepts 80 tracks.
floppy_check_drive:
    LD A,(current_unit)
    OR A
    SCF
    RET NZ
    LD A,(IX+17)
    AND $7F
    JR NZ,.unsuitable
    LD A,(IX+18)
    CP 43
    RET C
.unsuitable:
    LD A,9
    OR A
    RET

floppy_choose_unit:
    LD A,(FLOPPY_COUNT)
    CP 2
    LD A,0
    JR NZ,.set
    LD A,(SELECTED_DISK)
.set:
    LD (current_unit),A
    RET
floppy_bios_xdpb:
    LD IX,DPB_A
    LD A,(SELECTED_DISK)
    OR A
    RET Z
    LD IX,DPB_B
    RET

floppy_bios_io:
    LD A,(SELECTED_DISK)
    LD (logical_drive),A
    CALL floppy_choose_unit
    CALL floppy_bios_xdpb
.retry:
    LD A,(DISK_TRACK+1)
    LD HL,DISK_SECTOR+1
    OR (HL)
    LD A,6
    JR NZ,.error
    LD A,(DISK_TRACK)
    LD D,A
    LD A,(DISK_SECTOR)
    LD E,A
    CALL floppy_translate
    JR NC,.error
    LD A,(write_operation)
    OR A
    CALL NZ,disk_prepare_write
    CALL floppy_sector_retry
    JR NC,.error
    LD A,(write_operation)
    OR A
    CALL Z,disk_finish_read
    XOR A
    RET
.error:
    LD (last_error),A
    LD A,1
    LD (allow_ignore),A
    CALL floppy_error_prompt
    CP 'R'
    JR Z,.retry
    CP 'I'
    JR NZ,.cancel
    LD A,(write_operation)
    OR A
    JR NZ,.ignored
    ; Ignore supplies a defined empty sector rather than stale private data.
    LD HL,FDC_BUFFER
    LD DE,FDC_BUFFER+1
    LD BC,511
    LD (HL),$E5
    LDIR
    CALL disk_finish_read
.ignored:
    XOR A
    RET
.cancel:
    LD A,1
    RET

; C = physical unit, IX = destination XDPB. Native LOGIN ignores freeze.
floppy_login_disc:
    LD A,C
    AND 1
    LD (current_unit),A
    XOR A
    CALL $1926                    ; original standard +3 XDPB
    RET NC
    LD D,0
    LD E,0
    CALL floppy_translate
    RET NC
    CALL floppy_read_id
    RET NC
    AND $C0
    LD A,1
    JR Z,.pcw
    LD A,(results+5)
    AND $C0
    CP $40
    JR NZ,.data_format
    LD A,1
    JP $1926
.data_format:
    CP $C0
    JP NZ,floppy_bad_format
    LD A,2
    JP $1926
.pcw:
    LD A,1
    LD (physical_sector),A
    XOR A
    LD (write_operation),A
    CALL floppy_sector_retry
    RET NC
    LD HL,FDC_BUFFER
    LD A,(HL)
    LD B,10
    LD DE,FDC_BUFFER
.uniform:
    CP (HL)
    JR NZ,.specification
    INC HL
    DJNZ .uniform
    XOR A                         ; blank sector: native standard +3 format
    JP $1926
.specification:
    LD HL,FDC_BUFFER
    CALL floppy_validate_spec
    RET NC
    CALL $1939                    ; original XDPB/DPB builder
    RET NC
    JP floppy_validate_xdpb

; Accept the native 512-byte-sector family within the BIOS allocation limits.
floppy_validate_spec:
    PUSH HL
    POP IY
    LD A,(IY+0)
    CP 4
    JP NC,floppy_bad_format
    LD A,(IY+1)
    AND $7F
    CP 3
    JP NC,floppy_bad_format
    LD A,(IY+2)
    DEC A
    CP 80
    JP NC,floppy_bad_format
    LD A,(IY+3)
    DEC A
    CP 36
    JP NC,floppy_bad_format
    LD A,(IY+4)
    CP 2
    JP NZ,floppy_bad_format
    LD A,(IY+6)
    CP 3
    JP C,floppy_bad_format
    CP 7
    JP NC,floppy_bad_format
    LD A,(IY+7)
    CP 17
    JP NC,floppy_bad_format
    LD A,(IY+5)
    LD B,(IY+2)
    CP B
    JP NC,floppy_bad_format
    SCF
    RET
floppy_validate_xdpb:
    LD A,(IX+15)
    CP 2
    JP NZ,floppy_bad_format
    LD L,(IX+5)
    LD H,(IX+6)
    LD DE,364
    OR A
    SBC HL,DE
    JP NC,floppy_bad_format
    LD A,(IX+8)
    OR A
    JP NZ,floppy_bad_format
    LD A,(IX+12)
    OR A
    JP NZ,floppy_bad_format
    LD A,(IX+11)
    CP 65
    JP NC,floppy_bad_format
    LD A,(IX+4)
    CP 16
    JP NC,floppy_bad_format
    LD A,(IX+9)
    LD B,(IX+10)
    OR B
    JP Z,floppy_bad_format
    LD A,(IX+7)
    INC A
    LD H,0
    LD L,A
    JR NZ,.entries
    INC H
.entries:
    ADD HL,HL
    ADD HL,HL
    PUSH HL
    LD L,(IX+5)
    LD H,(IX+6)
    LD B,2
    CALL $1B48
    INC HL
    INC HL
    EX DE,HL                      ; size of native two-bit ALV
    POP HL                        ; optional hash-table size
    LD A,(FDC_BUFFER)
    SCF
    RET
floppy_bad_format:
    LD A,6
    OR A
    RET

; D/E logical track/sector, IX XDPB -> private physical C/H/R.
floppy_translate:
    LD A,E
    CP (IX+19)
    JP NC,floppy_bad_format
    LD A,(IX+15)
    CP 2
    JP NZ,floppy_bad_format
    LD A,(IX+17)
    AND $7F
    OR A
    JR Z,.single
    CP 1
    JR Z,.alternate
    CP 2
    JP NZ,floppy_bad_format
    LD A,D
    CP (IX+18)
    JR C,.side_zero
    SUB (IX+18)
    LD B,A
    LD A,(IX+18)
    DEC A
    SUB B
    LD D,A
    LD A,1
    JR .head
.alternate:
    LD A,D
    AND 1
    SRL D
    JR .head
.single:
    LD A,D
    CP (IX+18)
    JP NC,floppy_bad_format
.side_zero:
    XOR A
.head:
    LD (physical_head),A
    LD A,D
    CP (IX+18)
    JP NC,floppy_bad_format
    LD (physical_track),A
    LD A,E
    ADD A,(IX+20)
    LD (physical_sector),A
    SCF
    RET

floppy_sector_retry:
    LD HL,floppy_sector
floppy_retry:
    LD (retry_call+1),HL
    LD A,15
    LD (retry_count),A
.retry:
    CALL retry_call
    RET C
    LD (last_error),A
    LD HL,retry_count
    DEC (HL)
    JR Z,.failed
    LD A,(HL)
    CP 12
    JR Z,.realign
    CP 4
    JR Z,.realign
    CP 8
    JR NZ,.retry
    LD A,(physical_track)
    PUSH AF
    LD A,(IX+18)
    DEC A
    LD (physical_track),A
    CALL floppy_seek
    POP AF
    LD (physical_track),A
    JR .retry
.realign:
    LD A,$FF
    CALL floppy_forget_track
    JR .retry
.failed:
    LD A,(last_error)
    OR A
    RET
retry_call:
    JP 0
floppy_sector:
    CALL floppy_motor_start
    LD A,(current_unit)
    LD C,A
    CALL floppy_drive_status
    BIT 5,A
    LD A,0
    RET Z
    LD A,(write_operation)
    OR A
    JR Z,.ready
    LD A,(drive_status)
    BIT 6,A
    LD A,1
    RET NZ
.ready:
    CALL floppy_seek
    RET NC
    LD A,(physical_head)
    RLCA
    RLCA
    LD B,A
    LD A,(current_unit)
    OR B
    LD (command+1),A
    LD A,(physical_track)
    LD (command+2),A
    LD A,(physical_head)
    LD (command+3),A
    LD A,(physical_sector)
    LD (command+4),A
    LD (command+6),A
    LD A,(IX+23)
    LD (command+7),A
    LD A,(write_operation)
    OR A
    LD A,$46
    JR Z,.command
    LD A,$45
.command:
    LD (command),A
    LD HL,512
    LD (fdc_length),HL
    LD HL,command
    LD B,9
    CALL send_command
    JP C,floppy_timeout_error
    LD A,(write_operation)
    CALL disk_transfer
    PUSH AF
    CALL floppy_results
    POP BC
    RET NC
    LD A,B
    OR A
    LD A,4
    RET NZ
    XOR A
    SCF
    RET

floppy_read_id:
    LD HL,floppy_id_once
    JP floppy_retry
floppy_id_once:
    CALL floppy_motor_start
    CALL floppy_seek
    RET NC
    LD A,(physical_head)
    RLCA
    RLCA
    LD B,A
    LD A,(current_unit)
    OR B
    LD (read_id+1),A
    LD HL,read_id
    LD B,2
    CALL send_command
    JP C,floppy_timeout_error
    CALL floppy_results
    RET NC
    LD A,(results+5)
    SCF
    RET

floppy_results:
    LD HL,results
    LD B,7
.read:
    CALL receive_byte
    JP C,floppy_timeout_error
    LD (HL),A
    INC HL
    DJNZ .read
    LD A,(results)
    BIT 3,A
    LD A,0
    RET NZ
    LD A,(results+1)
    BIT 1,A
    LD A,1
    RET NZ
    LD A,(results+2)
    BIT 4,A
    LD A,8
    RET NZ
    LD A,(results+1)
    LD B,A
    LD A,(results+2)
    OR B
    BIT 5,A
    LD A,3
    RET NZ
    LD A,B
    BIT 0,A
    LD A,5
    RET NZ
    LD A,B
    BIT 2,A
    LD A,4
    RET NZ
    LD A,(results)
    AND $C0
    LD A,4
    RET NZ
    XOR A
    SCF
    RET
floppy_timeout_error:
    LD A,10
    OR A
    RET

floppy_forget_track:
    PUSH HL
    PUSH DE
    LD E,A
    LD A,(current_unit)
    LD L,A
    LD H,0
    LD DE,current_tracks
    ADD HL,DE
    LD (HL),$FF
    POP DE
    POP HL
    RET
floppy_seek:
    LD A,(current_unit)
    LD L,A
    LD H,0
    LD DE,current_tracks
    ADD HL,DE
    LD A,(HL)
    LD (track_pointer),HL
    CP $FF
    JR NZ,.seek
    LD A,(current_unit)
    LD (recalibrate+1),A
    LD HL,recalibrate
    LD B,2
    CALL send_command
    JP C,floppy_timeout_error
    XOR A
    CALL floppy_wait_seek
    RET NC
.seek:
    LD A,(physical_track)
    LD HL,(track_pointer)
    CP (HL)
    JR Z,.done
    LD (seek+2),A
    LD A,(current_unit)
    LD (seek+1),A
    LD HL,seek
    LD B,3
    CALL send_command
    JP C,floppy_timeout_error
    LD A,(physical_track)
    CALL floppy_wait_seek
    RET NC
.done:
    SCF
    RET
floppy_wait_seek:
    LD (seek_wanted),A
    LD HL,256
.poll:
    LD A,8
    CALL send_byte
    JP C,floppy_timeout_error
    CALL receive_byte
    JP C,floppy_timeout_error
    CP $80
    JR Z,.pending
    LD (seek_status),A
    CALL receive_byte
    JP C,floppy_timeout_error
    LD B,A
    LD A,(seek_status)
    AND $C3
    LD C,A
    LD A,(current_unit)
    CP C
    JR NZ,.pending
    LD A,(seek_wanted)
    CP B
    JR NZ,.pending
    LD HL,(track_pointer)
    LD (HL),A
    SCF
    RET
.pending:
    DEC HL
    LD A,H
    OR L
    JR NZ,.poll
    LD A,2
    OR A
    RET

floppy_drive_status:
    PUSH BC
    PUSH DE
    PUSH HL
    CALL floppy_motor_start
    LD A,C
    LD (status_command+1),A
    LD HL,status_command
    LD B,2
    CALL send_command
    LD A,0
    JR C,.return
    CALL receive_byte
    JR NC,.return
    XOR A
.return:
    LD (drive_status),A
    CALL floppy_dd_timeout
    POP HL
    POP DE
    POP BC
    RET

floppy_motor_start:
    PUSH AF
    PUSH BC
    PUSH HL
    LD BC,$1FFD
    LD A,(BANK_68)
    BIT 3,A
    LD HL,0
    LD (motor_timeout),HL         ; ON cancels a pending off timeout
    JR NZ,.done
    OR 8
    LD (BANK_68),A
    OUT (C),A
    LD A,(dd_setup)
    PUSH AF
    LD A,(IRQ_READY)
    OR A
    JR Z,.disabled
    EI                            ; the native spin-up wait permits timer scans
.disabled:
    POP AF
.delay:
    PUSH AF
    LD BC,$3548                  ; native 100 ms motor-on delay unit
.wait:
    DEC BC
    LD A,B
    OR C
    JR NZ,.wait
    POP AF
    DEC A
    JR NZ,.delay
    DI                            ; resume the driver's protected I/O section
.done:
    POP HL
    POP BC
    POP AF
    RET
floppy_motor_stop:
    LD BC,$1FFD
    LD A,(BANK_68)
    AND $F7
    LD (BANK_68),A
    OUT (C),A
    LD HL,0
    LD (motor_timeout),HL
    RET
floppy_motor_tick:
    LD A,(floppy_busy)
    OR A
    RET NZ
    LD HL,(motor_timeout)
    LD A,H
    OR L
    RET Z
    DEC HL
    LD (motor_timeout),HL
    LD A,H
    OR L
    RET NZ
    JP floppy_motor_stop

disk_prepare_write:
    LD A,(DMA_BANK)
    LD C,A
    LD B,2
    CALL $FA57
    LD DE,(DMA_ADDRESS)
    LD HL,FDC_BUFFER
    LD BC,512
    JP $FA4B
disk_finish_read:
    LD A,(DMA_BANK)
    LD B,A
    LD C,2
    CALL $FA57
    LD DE,FDC_BUFFER
    LD HL,(DMA_ADDRESS)
    LD BC,512
    JP $FA4B

disk_transfer:
    LD (FDC_DIRECTION),A
    LD HL,FDC_BUFFER
    LD DE,(fdc_length)
.byte:
    LD BC,$2FFD
    PUSH DE
    LD DE,0
.wait:
    IN A,(C)
    JP M,.ready
    DEC DE
    LD A,D
    OR E
    JR NZ,.wait
    POP DE
    JR .short
.ready:
    POP DE
    BIT 5,A
    JR Z,.short
    LD A,(FDC_DIRECTION)
    OR A
    JR NZ,.write
    LD B,$3F
    INI
    JR .count
.write:
    LD B,$40
    OUTI
.count:
    DEC DE
    LD A,D
    OR E
    JR NZ,.byte
    XOR A
    RET
.short:
    LD A,1
    RET

send_command:
    LD A,(HL)
    CALL send_byte
    RET C
    INC HL
    DJNZ send_command
    RET
send_byte:
    PUSH BC
    PUSH DE
    LD E,A
    LD BC,$2FFD
    LD D,0
.wait:
    IN A,(C)
    AND $C0
    CP $80
    JR Z,.ready
    DEC D
    JR NZ,.wait
    SCF
    JR .return
.ready:
    LD B,$3F
    OUT (C),E
    OR A
.return:
    POP DE
    POP BC
    RET
receive_byte:
    PUSH BC
    PUSH DE
    LD BC,$2FFD
    LD DE,0
.wait:
    IN A,(C)
    AND $C0
    CP $C0
    JR Z,.ready
    DEC DE
    LD A,D
    OR E
    JR NZ,.wait
    SCF
    JR .return
.ready:
    LD B,$3F
    IN A,(C)
    OR A
.return:
    POP DE
    POP BC
    RET
