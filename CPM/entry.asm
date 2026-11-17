; The stock disk bootstrap loads the locally rebuilt EMS at 0000..7FFF.
; Stage its first 16K in Spectrum RAM 4, then execute the loader in fixed RAM 2.
    DEVICE ZXSPECTRUM128
    ORG 0
    DI
    LD SP,$AF00
    LD BC,$7FFD
    LD A,$0C
    OUT (C),A
    LD HL,0
    LD DE,$C000
    LD BC,$4000
    LDIR
    LD HL,$C100
    LD DE,$B000
    LD BC,$0F00
    LDIR
    JP $B000
    ASSERT $ <= $100
    DS $100-$,0
