; Fresh media login discards only this drive's cached directory/data sectors.
; CP/M retains BCBs across warm boot even after its drive-login bitmap resets.
invalidate_disk_buffers:
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    LD HL,$0600
    CALL .one
    LD HL,$060F
    CALL .one
    POP HL
    POP DE
    POP BC
    POP AF
    RET
.one:
    PUSH HL
    EX DE,HL
    LD HL,login_buffer_drive
    LD A,(CURRENT_BANK)
    LD B,A
    LD C,0
    CALL $FA57
    LD BC,1
    CALL $FA4B
    POP HL
    LD A,(SELECTED_DISK)
    LD B,A
    LD A,(login_buffer_drive)
    CP B
    RET NZ
    PUSH HL
    LD A,(CURRENT_BANK)
    LD C,A
    LD B,0
    CALL $FA57
    POP HL
    LD DE,login_empty_drive
    LD BC,1
    JP $FA4B
login_buffer_drive: DB 0
login_empty_drive: DB $FF
