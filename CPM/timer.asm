; The +3 updates the SCB's BCD clock and scans the keyboard at 50 Hz.
; All interrupts use the nested common gate; this stack remains private when
; the displayed Spectrum RAM page is temporarily mapped at C000.
timer_service:
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    LD (timer_sp),SP
    PUSH BC
    LD BC,(GATE_CSP)
    LD (timer_csp),BC
    POP BC
    LD (GATE_CSP),SP
    LD SP,TIMER_STACK
    CALL keyboard_tick
    CALL floppy_motor_tick
    CALL cursor_tick
    LD HL,clock_fraction
    INC (HL)
    LD A,(HL)
    CP 50
    JR C,.done
    LD (HL),0
    LD A,(SCB_SECOND)
    ADD A,1
    DAA
    CP $60
    JR C,.second
    XOR A
    LD (SCB_SECOND),A
    LD A,(SCB_MINUTE)
    ADD A,1
    DAA
    CP $60
    JR C,.minute
    XOR A
    LD (SCB_MINUTE),A
    LD A,(SCB_HOUR)
    ADD A,1
    DAA
    CP $24
    JR C,.hour
    XOR A
    LD (SCB_HOUR),A
    LD HL,(SCB_DATE)
    INC HL
    LD (SCB_DATE),HL
    JR .done
.second:
    LD (SCB_SECOND),A
    JR .done
.minute:
    LD (SCB_MINUTE),A
    JR .done
.hour:
    LD (SCB_HOUR),A
.done:
    LD BC,(timer_csp)
    LD (GATE_CSP),BC
    LD SP,(timer_sp)
    POP HL
    POP DE
    POP BC
    POP AF
    RET
clock_service:
    LD A,C
    OR A
    RET Z
    XOR A
    LD (clock_fraction),A
    RET
cursor_tick:
    LD A,$FE
    LD HL,FLIP_PENDING
    BIT 0,(HL)
    JR Z,.call
    LD (HL),0
    LD A,$FC
.call:
    LD (SCREEN_FUNCTION),A
    JP SCREEN_GATE
