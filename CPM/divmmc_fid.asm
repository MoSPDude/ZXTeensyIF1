; DivMMC SDHC disk FID for the AllRAM BIOS. New source; no Locomotive bytes.
; IDEDOS layout and C:/D:/E: naming follow Paul Osmialowski's public-domain
; ZXMMC FID: https://sourceforge.net/projects/zxmmcfid/ (zxmmc.asm).
; Requires SDv2 block addressing, as provided by the Teensy's IMG backend.
    DEVICE ZXSPECTRUM128
    IFNDEF FID_ORIGIN
FID_ORIGIN EQU $0100
    ENDIF
    ORG FID_ORIGIN
SVC_D_HOOK EQU $FF05
SVC_ALLOCATE EQU $FF07
SVC_DEALLOCATE EQU $FF09
    JP start
    DB "SPECTRUMFID"
    DW $0100,0
    DB 0,0
    DS 12,0
jumpblock:
    JP logon
    JP read
    JP write
    JP flush
    JP message
names:
    DB "CP/M            SOFTWARE        DEVEL           "
starts: DS 12,0
found: DS 3,0
valid_dpb:
    DW $0200
    DB 6,$3F,3
    DW $07F7,$01FF
    DB $C0,0
    DW $8000,0
    DB 2,3
start:
    DI
    LD DE,512
.svc_allocate_call:
    CALL SVC_ALLOCATE
    JP NC,failed
    LD (scratch),HL
    CALL sd_init
    JP NZ,.failed
    CALL scan_partitions
    JP NC,.failed
.register:
    LD A,0
.next:
    LD (current_slot),A
    LD E,A
    LD D,0
    LD HL,found
    ADD HL,DE
    LD A,(HL)
    OR A
    JR Z,.skip
    LD A,(current_slot)
    ADD A,2
    LD B,A
    LD DE,jumpblock
    LD HL,510
    LD IX,0
    LD IY,0                       ; optional directory hashing disabled
.svc_hook_call:
    CALL SVC_D_HOOK
.skip:
    LD A,(current_slot)
    INC A
    CP 3
    JR C,.next
    LD HL,signon
    XOR A
    SCF
    RET
.failed:
    LD HL,(scratch)
    LD DE,512
.svc_deallocate_failure_call:
    CALL SVC_DEALLOCATE
failed:
    LD HL,unavailable
    OR A
    RET
scan_partitions:
    XOR A
    LD (found),A
    LD (found+1),A
    LD (found+2),A
    LD DE,0
    LD (table_lba),DE
    CALL read_table
    JR Z,.table
    LD DE,128
    LD (table_lba),DE
    CALL read_table
    JP NZ,scan_failed
.table:
    LD HL,(scratch)
    LD DE,$26
    ADD HL,DE
    LD E,(HL)
    INC HL
    LD D,(HL)
    LD A,D
    CP 8
    JP NC,scan_failed
    OR E
    JP Z,scan_failed
    LD (entries_left),DE
.sector:
    LD IX,(scratch)
    LD A,8
    LD (entries_here),A
.entry:
    LD A,(IX+16)
    OR A
    JR Z,.done
    CP 3
    CALL Z,examine
    LD BC,64
    ADD IX,BC
    LD HL,(entries_left)
    DEC HL
    LD (entries_left),HL
    LD A,H
    OR L
    JR Z,.done
    LD HL,entries_here
    DEC (HL)
    JR NZ,.entry
    LD DE,(table_lba)
    INC DE
    LD (table_lba),DE
    LD HL,0
    LD IY,(scratch)
    CALL sd_read
    JP NZ,scan_failed
    JR .sector
.done:
    SCF
    RET
scan_failed:
    OR A
    RET
read_table:
    LD HL,0
    LD IY,(scratch)
    CALL sd_read
    RET NZ
    LD HL,(scratch)
    LD DE,table_signature
    LD B,10
    CALL compare
    RET NZ
    LD IX,(scratch)
    LD A,(IX+16)
    CP 1
    RET NZ
    LD A,(IX+34)
    CP 2
    RET NZ
    LD A,(IX+35)
    CP 128
    RET NZ
    LD A,(IX+36)
    OR A
    RET NZ
    LD A,(IX+37)
    CP 1
    RET
examine:
    LD A,(IX+19)
    CP 2
    RET NC
    XOR (IX+22)
    CP 1
    RET NZ
    LD L,(IX+17)
    LD H,(IX+18)
    LD DE,127
    ADD HL,DE
    LD E,(IX+19)
    LD D,0
    ADD HL,DE
    LD A,L
    CP (IX+20)
    RET NZ
    LD A,H
    CP (IX+21)
    RET NZ
    PUSH IX
    POP HL
    LD DE,32
    ADD HL,DE
    LD DE,valid_dpb
    LD B,17
    CALL compare
    RET NZ
    XOR A
.name:
    LD (current_slot),A
    ADD A,A
    ADD A,A
    ADD A,A
    ADD A,A
    LD E,A
    LD D,0
    LD HL,names
    ADD HL,DE
    EX DE,HL
    PUSH IX
    POP HL
    LD B,16
    CALL compare
    JR Z,.match
    LD A,(current_slot)
    INC A
    CP 3
    JR C,.name
    RET
.match:
    LD A,(current_slot)
    LD E,A
    LD D,0
    LD HL,found
    ADD HL,DE
    LD (HL),1
    LD A,E
    ADD A,A
    ADD A,A
    LD E,A
    LD HL,starts
    ADD HL,DE
    LD A,(IX+19)
    RRCA
    AND $80
    LD (HL),A
    INC HL
    LD A,(IX+17)
    LD (HL),A
    INC HL
    LD A,(IX+18)
    LD (HL),A
    INC HL
    LD (HL),0
    RET
compare:
    LD A,(DE)
    CP (HL)
    RET NZ
    INC DE
    INC HL
    DJNZ compare
    RET
logon:
    ; Keep the 512-byte EMS allocation for the driver's lifetime. A new
    ; login rereads the mounted IMG's table without using BDOS or allocating.
    LD A,B
    SUB 2
    CP 3
    JR NC,.failed
    LD (login_slot),A
    PUSH IX
    CALL sd_init
    JR NZ,.restore_failed
    CALL scan_partitions
    JR NC,.restore_failed
    LD A,(login_slot)
    LD L,A
    LD H,0
    LD DE,found
    ADD HL,DE
    LD A,(HL)
    OR A
    JR Z,.restore_failed
    POP IX
    LD HL,valid_dpb
    PUSH IX
    POP DE
    LD BC,17
    LDIR
    XOR A
    LD (recovery_pending),A
    SCF
    RET
.restore_failed:
    POP IX
.failed:
    LD A,1
    OR A
    RET
read:
    CALL recover_io
    JR NZ,io_error
    CALL address
    JR NZ,io_error
    CALL sd_read
    JR finish
write:
    CALL recover_io
    JR NZ,io_error
    CALL address
    JR NZ,io_error
    CALL sd_write
finish:
    JR NZ,io_error
    XOR A
    SCF
    RET
io_error:
    LD A,1
    LD (recovery_pending),A
    OR A
    RET
recover_io:
    LD A,(recovery_pending)
    OR A
    RET Z
    ; A retry belongs to this FID. Preserve the original I/O parameters and
    ; staged write sector while refreshing SD and the mounted image's table.
    PUSH BC
    PUSH DE
    PUSH HL
    PUSH IX
    PUSH IY
    CALL logon
    POP IY
    POP IX
    POP HL
    POP DE
    POP BC
    RET
flush:
    CALL deselect
    XOR A
    SCF
    RET
message:
    OR A
    RET
; track HL (0..255), sector DE (0..127), B drive C/D/E -> HL:DE LBA.
address:
    LD A,H
    OR D
    RET NZ
    LD A,E
    CP 128
    JR NC,.bad
    LD A,B
    SUB 2
    CP 3
    JR NC,.bad
    ADD A,A
    ADD A,A
    LD C,A
    LD B,0
    PUSH HL
    LD HL,starts
    ADD HL,BC
    LD C,(HL)
    INC HL
    LD B,(HL)
    INC HL
    LD A,(HL)
    INC HL
    LD H,(HL)
    LD L,A
    EX (SP),HL
    LD D,0
    ADD HL,HL
    ADD HL,HL
    ADD HL,HL
    ADD HL,HL
    ADD HL,HL
    ADD HL,HL
    ADD HL,HL
    ADD HL,DE
    ADD HL,BC
    EX DE,HL
    POP HL
    JR NC,.ok
    INC HL
.ok:
    XOR A
    RET
.bad:
    LD A,1
    OR A
    RET

; SPI input clocks FF. Command and data waits are bounded even with no image.
deselect:
    PUSH AF
    LD A,$FF
    OUT ($E7),A
    IN A,($EB)
    POP AF
    RET
command:
    PUSH AF
    CALL deselect
    LD A,$FE
    OUT ($E7),A
    POP AF
    OUT ($EB),A
    PUSH AF
    LD A,H
    OUT ($EB),A
    LD A,L
    OUT ($EB),A
    LD A,D
    OUT ($EB),A
    LD A,E
    OUT ($EB),A
    POP AF
    CP $40
    LD A,$95
    JR Z,.crc
    LD A,$87                      ; CMD8 CRC, ignored for other commands
.crc:
    OUT ($EB),A
    LD BC,0
.wait:
    IN A,($EB)
    BIT 7,A
    RET Z
    DEC BC
    LD A,B
    OR C
    JR NZ,.wait
    LD A,$FF
    RET
sd_init:
    LD HL,0
    LD DE,0
    LD A,$40
    CALL command
    CP 1
    RET NZ
    LD DE,$01AA
    LD A,$48
    CALL command
    CP 1
    RET NZ
    IN A,($EB)
    IN A,($EB)
    IN A,($EB)
    CP 1
    RET NZ
    IN A,($EB)
    CP $AA
    RET NZ
    LD IX,256
.again:
    LD HL,0
    LD DE,0
    LD A,$77
    CALL command
    CP 2
    JR NC,.bad
    LD HL,$4000
    LD A,$69
    CALL command
    OR A
    JR Z,.ocr
    DEC IX
    PUSH IX
    POP BC
    LD A,B
    OR C
    JR NZ,.again
.bad:
    LD A,1
    OR A
    RET
.ocr:
    LD HL,0
    LD DE,0
    LD A,$7A
    CALL command
    OR A
    RET NZ
    IN A,($EB)
    AND $C0
    CP $C0
    RET NZ
    IN A,($EB)
    IN A,($EB)
    IN A,($EB)
    CALL deselect
    XOR A
    RET
sd_read:
    LD A,$51
    CALL command
    OR A
    JR NZ,sd_error
    LD BC,0
.token:
    IN A,($EB)
    CP $FE
    JR Z,.data
    CP $FF
    JR NZ,sd_error
    DEC BC
    LD A,B
    OR C
    JR NZ,.token
    JR sd_error
.data:
    PUSH IY
    POP HL
    LD BC,$00EB
    INIR
    INIR
    IN A,($EB)
    IN A,($EB)                     ; SD CRC disabled, Teensy supplies 0000
    CALL deselect
    XOR A
    RET
sd_write:
    LD A,$58
    CALL command
    OR A
    JR NZ,sd_error
    LD A,$FE
    OUT ($EB),A
    PUSH IY
    POP HL
    LD BC,$00EB
    OTIR
    OTIR
    LD A,$FF
    OUT ($EB),A
    OUT ($EB),A
    LD BC,0
.response:
    IN A,($EB)
    CP $FF
    JR NZ,.check
    DEC BC
    LD A,B
    OR C
    JR NZ,.response
    JR sd_error
.check:
    AND $1F
    CP 5
    JR NZ,sd_error
    LD BC,0
.busy:
    IN A,($EB)
    CP $FF
    JR Z,.done
    DEC BC
    LD A,B
    OR C
    JR NZ,.busy
    JR sd_error
.done:
    CALL deselect
    XOR A
    RET
sd_error:
    CALL deselect
    LD A,1
    OR A
    RET
scratch: DW 0
table_lba: DW 0
entries_left: DW 0
entries_here: DB 0
current_slot: DB 0
login_slot: DB 0
recovery_pending: DB 0
table_signature: DB "PLUSIDEDOS"
signon: DB "DivMMC SDHC disk FID",13,10,$FF
unavailable: DB "DivMMC image unavailable",13,10,$FF
fid_end:
