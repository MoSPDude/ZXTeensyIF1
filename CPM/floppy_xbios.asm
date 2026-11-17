; Disk XBIOS and BIOS message handling; private high RAM in driver bank 2.
floppy_error_prompt:
    LD A,(BDOS_ERROR_MODE)
    INC A
    LD A,'C'
    RET Z                       ; FF: return errors without asking the user
    CALL floppy_motor_stop       ; in-game mounting is allowed only when idle
    LD A,(last_error)
    CP 10
    JR C,.known
    LD A,7
.known:
    LD B,A
    LD A,(logical_drive)
    LD C,A
    LD A,(DISK_TRACK)
    LD D,A
    LD A,(DISK_SECTOR)
    LD E,A
    CALL floppy_show_native
    JR floppy_prompt_wait
floppy_prompt_choices:
    CALL floppy_show_message
floppy_prompt_wait:
    CALL keyboard_discard
.wait:
    CALL console_input
    AND $DF
    CP 'R'
    JR Z,.done
    CP 'C'
    JR Z,.done
    CP 'I'
    JR NZ,.invalid
    LD B,A
    LD A,(allow_ignore)
    OR A
    LD A,B
    JR NZ,.done
.invalid:
    LD A,$F8
    LD (SCREEN_FUNCTION),A
    CALL SCREEN_GATE
    CALL keyboard_discard
    JR .wait
.done:
    PUSH AF
    CALL floppy_restore_status
    POP AF
    RET
floppy_restore_status:
    LD A,$FA
    LD (SCREEN_FUNCTION),A
    JP SCREEN_GATE
floppy_external_error:
    LD (last_error),A
    LD A,(SELECTED_DISK)
    LD (logical_drive),A
    LD A,C
    AND 1
    LD (allow_ignore),A
    LD HL,floppy_error_prompt
    JP floppy_private
floppy_external_message:
    LD A,(SELECTED_DISK)
    LD (logical_drive),A
    LD A,C
    AND 1
    LD (allow_ignore),A
    LD HL,floppy_message_prompt
    JP floppy_private
floppy_message_prompt:
    CALL floppy_motor_stop
    LD HL,DISK_MESSAGE
    LD DE,error_text
    CALL copy_text
    JP floppy_prompt_choices
floppy_show_message:
    LD BC,$0502
    CALL $FA57
    LD DE,error_text
    LD HL,SCREEN_MESSAGE
    LD BC,128
    CALL $FA4B
    LD A,$FB
    LD (SCREEN_FUNCTION),A
    JP SCREEN_GATE
floppy_show_native:
    LD A,$F9
    LD (SCREEN_FUNCTION),A
    JP SCREEN_GATE
copy_text:
    LD A,(HL)
    LD (DE),A
    OR A
    RET Z
    INC HL
    INC DE
    JR copy_text
keyboard_discard:
    XOR A
    LD (key_head),A
    LD (key_tail),A
    LD (key_count),A
    LD (pending_valid),A
    LD (expansion_remaining),A
    LD (repeat_valid),A
    RET

floppy_dd_init:
    LD HL,dd_defaults
    CALL floppy_dd_setup
    CALL floppy_initialise
    RET
floppy_dd_setup:
    LD DE,dd_setup
    LD BC,7
    LDIR
    LD A,(dd_setup+1)
    LD L,A
    LD H,0
    LD D,H
    LD E,L
    ADD HL,HL
    ADD HL,HL
    ADD HL,DE
    LD (motor_ticks),HL
    LD A,(dd_setup+4)
    DEC A
    RLCA
    RLCA
    RLCA
    CPL
    AND $F0
    LD HL,dd_setup+5
    OR (HL)
    LD (specify+1),A
    LD A,(dd_setup+6)
    LD (specify+2),A
    LD HL,specify
    LD B,3
    JP send_command

floppy_dd_read:
    XOR A
    JR floppy_dd_io
floppy_dd_write:
    LD A,1
    JR floppy_dd_io
floppy_dd_check:
    LD A,2
floppy_dd_io:
    LD (dd_direction),A
    LD A,B
    LD (dd_bank),A
    LD (dd_buffer),HL
    LD (dd_track_sector),DE
    LD (dd_xdpb),IX
    LD A,C
    AND 1
    LD (current_unit),A
    LD HL,floppy_dd_sector
    JP floppy_private
floppy_dd_sector:
    LD IX,(dd_xdpb)
    LD DE,(dd_track_sector)
    CALL floppy_translate
    RET NC
    LD A,(dd_direction)
    OR A
    JR Z,.read
    LD A,(dd_bank)
    LD C,A
    LD B,2
    CALL $FA57
    LD DE,(dd_buffer)
    LD HL,FDC_BUFFER
    LD A,(dd_direction)
    CP 2
    JR NZ,.stage
    LD HL,$2600
.stage:
    LD BC,512
    CALL $FA4B
.read:
    LD A,(dd_direction)
    AND 1
    LD (write_operation),A
    CALL floppy_sector_retry
    RET NC
    LD A,(dd_direction)
    CP 1
    JR Z,.done
    CP 2
    JR Z,.compare
    LD A,(dd_bank)
    LD B,A
    LD C,2
    CALL $FA57
    LD DE,FDC_BUFFER
    LD HL,(dd_buffer)
    LD BC,512
    CALL $FA4B
.done:
    XOR A
    SCF
    RET
.compare:
    LD HL,FDC_BUFFER
    LD DE,$2600
    LD BC,512
.byte:
    LD A,(DE)
    CP (HL)
    JR NZ,.different
    INC HL
    INC DE
    DEC BC
    LD A,B
    OR C
    JR NZ,.byte
    JR .done
.different:
    LD A,1
    OR A
    SCF
    RET

floppy_dd_login:
    LD (dd_xdpb),IX
    LD A,C
    LD (current_unit),A
    LD HL,floppy_dd_login_body
    JP floppy_private
floppy_dd_login_body:
    LD IX,(dd_xdpb)
    LD A,(current_unit)
    LD C,A
    JP floppy_login_disc
floppy_dd_read_id:
    LD (dd_xdpb),IX
    LD A,C
    AND 1
    LD (current_unit),A
    LD (dd_track_sector),DE
    LD HL,floppy_dd_id_body
    JP floppy_private
floppy_dd_id_body:
    LD IX,(dd_xdpb)
    LD DE,(dd_track_sector)
    LD E,0
    CALL floppy_translate
    RET NC
    CALL floppy_read_id
    JP floppy_export_results
floppy_export_results:
    PUSH AF
    LD HL,FDC_RESULTS
    LD (HL),7
    INC HL
    EX DE,HL
    LD HL,results
    LD BC,7
    LDIR
    LD HL,FDC_RESULTS
    POP AF
    RET
floppy_dd_l_dpb:
    PUSH IY
    CALL floppy_validate_spec
    POP IY
    RET NC
    JP $197B
floppy_dd_l_xdpb:
    PUSH IY
    CALL floppy_validate_spec
    POP IY
    RET NC
    JP $1939

floppy_dd_format:
    LD A,E
    LD (format_command+5),A
    LD A,B
    LD (dd_bank),A
    LD (dd_buffer),HL
    LD (dd_track_sector),DE
    LD (dd_xdpb),IX
    LD A,C
    AND 1
    LD (current_unit),A
    LD HL,floppy_dd_format_body
    JP floppy_private
floppy_dd_format_body:
    LD IX,(dd_xdpb)
    LD DE,(dd_track_sector)
    LD E,0
    CALL floppy_translate
    RET NC
    LD A,(IX+19)
    LD (format_command+3),A
    LD L,A
    LD H,0
    ADD HL,HL
    ADD HL,HL
    LD (fdc_length),HL
    LD A,(IX+24)
    LD (format_command+4),A
    LD A,(physical_head)
    RLCA
    RLCA
    LD B,A
    LD A,(current_unit)
    OR B
    LD (format_command+1),A
    LD A,(dd_bank)
    LD C,A
    LD B,2
    CALL $FA57
    LD DE,(dd_buffer)
    LD HL,FDC_BUFFER
    LD BC,(fdc_length)
    CALL $FA4B
    LD HL,floppy_format_once
    JP floppy_retry
floppy_format_once:
    CALL floppy_motor_start
    CALL floppy_seek
    RET NC
    LD HL,format_command
    LD B,6
    CALL send_command
    JP C,floppy_timeout_error
    LD A,1
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

floppy_dd_seek:
    LD A,C
    AND 1
    LD (current_unit),A
    LD A,D
    LD (physical_track),A
    PUSH BC
    PUSH DE
    PUSH HL
    CALL floppy_motor_start
    CALL floppy_seek
    POP HL
    POP DE
    POP BC
    RET
floppy_dd_timeout:
    PUSH AF
    PUSH HL
    LD HL,0
    LD (motor_timeout),HL
    LD A,(BANK_68)
    AND 8
    JR Z,.done
    LD HL,(motor_ticks)
    LD (motor_timeout),HL
.done:
    POP HL
    POP AF
    RET
floppy_dd_low_read:
    XOR A
    JR floppy_dd_low
floppy_dd_low_write:
    LD A,1
floppy_dd_low:
    LD (dd_direction),A
    LD DE,raw_parameters
    LD BC,15
    LDIR
    LD HL,floppy_dd_low_body
    JP floppy_private
floppy_dd_low_body:
    LD HL,(raw_parameters+3)
    LD DE,513
    OR A
    SBC HL,DE
    JP NC,floppy_bad_format
    LD HL,(raw_parameters+3)
    LD (fdc_length),HL
    LD A,(raw_parameters+5)
    DEC A
    CP 9
    JP NC,floppy_bad_format
    LD A,(raw_parameters)
    LD (dd_bank),A
    LD HL,(raw_parameters+1)
    LD (dd_buffer),HL
    LD A,(dd_direction)
    OR A
    JR Z,.command
    LD A,(dd_bank)
    LD C,A
    LD B,2
    CALL $FA57
    LD DE,(dd_buffer)
    LD HL,FDC_BUFFER
    LD BC,(fdc_length)
    LD A,B
    OR C
    CALL NZ,$FA4B
.command:
    LD HL,raw_parameters+6
    LD A,(raw_parameters+5)
    LD B,A
    CALL send_command
    JP C,floppy_timeout_error
    LD HL,(fdc_length)
    LD A,H
    OR L
    JR Z,.results
    LD A,(dd_direction)
    CALL disk_transfer
.results:
    CALL floppy_results
    PUSH AF
    LD A,(dd_direction)
    OR A
    JR NZ,.export
    LD A,(dd_bank)
    LD B,A
    LD C,2
    CALL $FA57
    LD DE,FDC_BUFFER
    LD HL,(dd_buffer)
    LD BC,(fdc_length)
    LD A,B
    OR C
    CALL NZ,$FA4B
.export:
    POP AF
    JP floppy_export_results
raw_parameters: DS 15,0
dd_defaults: DB 10,50,175,30,12,15,3

    INCLUDE "buffer_login.asm"
