; Standalone GIOS exercise. The patched local PRL is embedded and relocated
; by the patcher, so this demonstration needs no third-party GDOS/runtime.
    DEVICE ZXSPECTRUM128
    ORG $0100
PB       EQU $9200
CONTROL  EQU $9000
INTIN    EQU $9040
PTSIN    EQU $9080
INTOUT   EQU $9100
PTSOUT   EQU $9180
DRIVER   EQU $C000
start:
    LD (saved_sp),SP
    LD SP,$9FF0
    LD HL,driver_bytes
    LD DE,DRIVER
    LD BC,driver_end-driver_bytes
    LDIR
    LD HL,parameters
    LD DE,PB
    LD BC,10
    LDIR
    LD HL,defaults
    LD DE,INTIN
    LD BC,20
    LDIR
    LD A,1
    LD BC,0
    LD HL,10
    CALL invoke
    LD A,3
    CALL simple
    LD HL,frame
    LD BC,20
    CALL points
    LD A,6
    LD BC,5
    LD HL,0
    CALL invoke
    LD HL,rectangle
    LD BC,8
    CALL points
    LD HL,1
    LD (CONTROL+10),HL
    LD A,11
    LD BC,2
    LD HL,0
    CALL invoke
    LD HL,title
    LD DE,INTIN
    LD BC,title_end-title
    LDIR
    LD HL,32
    LD (PTSIN),HL
    LD HL,151
    LD (PTSIN+2),HL
    LD A,8
    LD BC,1
    LD HL,(title_end-title)/2
    CALL invoke
    LD C,1
    CALL $0005
    LD A,2
    CALL simple
    LD SP,(saved_sp)
    RET
simple:
    LD BC,0
    LD HL,0
invoke:
    LD (CONTROL+2),BC
    LD (CONTROL+6),HL
    LD L,A
    LD H,0
    LD (CONTROL),HL
    XOR A
    LD H,A
    LD L,A
    LD (CONTROL+4),HL
    LD (CONTROL+8),HL
    LD DE,PB
    CALL DRIVER
    RET
points:
    LD DE,PTSIN
    LDIR
    RET
parameters: DW CONTROL,INTIN,PTSIN,INTOUT,PTSOUT
defaults: DW 1,1,1,3,1,1,1,1,1,1
frame: DW 8,8,247,8,247,183,8,183,8,8
rectangle: DW 48,48,207,112
title: DW 'Z','X','T','e','e','n','s','y','I','F','1',' ','G','S','X'
title_end:
saved_sp: DW 0
driver_bytes:
    INCBIN "demo-driver.bin"
driver_end:
    ASSERT $ < $4000
