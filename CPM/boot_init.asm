; Installed in the boot sector's unused-on-success error-animation area.
    DEVICE ZXSPECTRUM128
    ORG $FF0A
    DI
    LD BC,$0F3B
    LD A,$A0                      ; ALLRAM + MAPDISABLE, writes enabled
    OUT (C),A
    LD BC,$7FFD
    LD A,3                        ; boot sector in Spectrum RAM 3
    OUT (C),A
    LD BC,$1FFD
    LD A,$18                      ; motor on; AllRAM page bit 1 is zero
    OUT (C),A
    LD BC,$00E3
    XOR A
    OUT (C),A                     ; establish both 8K pointers after ROM paging
    JP $FE18
    ASSERT $ <= $FF36
