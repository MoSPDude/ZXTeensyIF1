; A small original test program, installed only in a local test copy of the DSK.
; Fill every byte of shared TPA, including the staging loader's old location.
    DEVICE ZXSPECTRUM128
    ORG $0100
    LD SP,$3F00
    LD HL,$4000
    LD DE,$4001
    LD BC,$B3FF
    LD (HL),$A5
    LDIR
    LD C,2
    LD E,'.'
    CALL 5                        ; switches banks and renders to shadow RAM
    LD HL,$4000
    LD BC,$B400
.check:
    LD A,(HL)
    CP $A5
    JR NZ,.failed
    INC HL
    DEC BC
    LD A,B
    OR C
    JR NZ,.check
    LD DE,passed
    JR .message
.failed:
    LD DE,failed
.message:
    LD C,9
    CALL 5
    JP 0                          ; reload the original cached CCP
passed: DB "RAM TEST PASS",13,10,'$'
failed: DB "RAM TEST FAIL",13,10,'$'
