; Runs once in the shared TPA after the BIOS/BDOS are ready, before the CCP.
; Collect names first: OPEN and READ are allowed to disturb BDOS's search state.
    DEVICE ZXSPECTRUM128
    INCLUDE "layout.inc"
    ORG $6000
    LD SP,$5FF0
    XOR A
    LD (BOOT_FIDS),A
    LD C,13
    CALL 5
    LD C,32
    LD E,0
    CALL 5
    LD DE,$7F00
    LD C,26
    CALL 5
    LD HL,$7400
    LD (next_name),HL
    LD C,17
.search:
    LD DE,search_fcb
    CALL 5
    CP $FF
    JR Z,.load
    ADD A,A
    ADD A,A
    ADD A,A
    ADD A,A
    ADD A,A
    LD L,A
    LD H,$7F
    INC HL
    LD DE,(next_name)
    LD BC,11
    LDIR
    LD (next_name),DE
    LD C,18
    JR .search
.load:
    LD HL,$7400
.next:
    LD DE,(next_name)
    OR A
    SBC HL,DE
    JP Z,$FA03
    ADD HL,DE
    LD (name_position),HL
    LD DE,FID_NAME
    LD BC,11
    LDIR
    LD (name_position),HL
    LD HL,file_fcb
    LD DE,file_fcb+1
    LD BC,35
    LD (HL),0
    LDIR
    LD HL,FID_NAME
    LD DE,file_fcb+1
    LD BC,11
    LDIR
    LD DE,file_fcb
    LD C,15
    CALL 5
    CP $FF
    JR Z,.skip
    LD HL,$8000
    LD (destination),HL
.read:
    LD DE,(destination)
    LD C,26
    CALL 5
    LD DE,file_fcb
    LD C,20
    CALL 5
    OR A
    JR NZ,.eof
    LD HL,(destination)
    LD DE,128
    ADD HL,DE
    LD (destination),HL
    LD A,H
    CP $F0
    JR C,.read
    JR .skip                      ; oversized PRL: leave it uninstalled
.eof:
    CP 1
    JR NZ,.skip                    ; don't execute a partially read file
    LD HL,(destination)
    LD DE,$8000
    OR A
    SBC HL,DE
    LD B,H
    LD C,L
    LD HL,HOST_INSTALL
    CALL HOST_GATE
.skip:
    LD HL,(name_position)
    JP .next
next_name: DW 0
name_position: DW 0
destination: DW 0
search_fcb:
    DB 1,"????????FID"
    DS 24,0
file_fcb: DS 36,0
    ASSERT $ < $7000
