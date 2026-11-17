; Bank-0 +3 XBIOS jumpblock and bank-0 call stubs. No original BIOS bytes.
    DEVICE ZXSPECTRUM128
    INCLUDE "layout.inc"
    ORG 0
    DS $80,0
    DUP 37
    IF (($-$0080)/3) < 25
    JP $0B00+(($-$0080)/3)*10
    ELSE
    JP $0780+((($-$0080)/3)-25)*10
    ENDIF
    EDUP
    DS DPH_A-$,0
    DW 0,0,0,0,0
    DB 0,0
    DW DPB_A,$0700,$0640,DIR_HEAD,DATA_HEAD,$FFFF
    DB 0
    DS DPH_B-$,0
    DW 0,0,0,0,0
    DB 0,0
    DW DPB_B,$0740,$06A0,DIR_HEAD,DATA_HEAD,$FFFF
    DB 0
    DS $0600-$,0
bcb_initial:
    DB $FF,0,0,0,0,0
    DW 0,0,$0100
    DB 0
    DW 0
    DB $FF,0,0,0,0,0
    DW 0,0,$0300
    DB 0
    DW 0
bcb_initial_end:
    ; Use the gap after the floppy CSVs for the last twelve stubs. Keep the
    ; FID DPHs at 0800..085F and all sixteen header mirrors at 0900..0AFF free.
    DS $0780-$,0
    DUP 12
    PUSH AF
    PUSH HL
    LD HL,DRIVER_XBIOS
    LD A,$CB+(($-$0780-5)/10)*3
    JP XBIOS_GATE
    EDUP
    ASSERT $ <= $0800
    DS $0B00-$,0
    DUP 25
    PUSH AF
    PUSH HL
    LD HL,DRIVER_XBIOS
    LD A,$80+(($-$0B00-5)/10)*3
    JP XBIOS_GATE
    EDUP
    ASSERT $ <= $0C00
