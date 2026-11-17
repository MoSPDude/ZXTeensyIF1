; Token-oriented keyboard, following the original +3 driver and XBIOS API.
; Scans use the real forty-key numbering (CAPS=0, SYMBOL=36).
keyboard_initialise:
    XOR A
    LD (key_head),A
    LD (key_tail),A
    LD (key_count),A
    LD (key_locks),A
    LD (repeat_valid),A
    LD (pending_valid),A
    LD (expansion_remaining),A
    LD (clock_fraction),A
    LD HL,keys_down
    LD DE,keys_down+1
    LD BC,39
    LD (HL),A
    LDIR
    RET

keyboard_poll:
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    PUSH IX
    PUSH IY
    XOR A
    LD (new_key),A
    LD IX,matrix_scan+7
    LD BC,$7FFE
    LD D,8
.read:
    IN A,(C)
    CPL
    AND $1F
    LD (IX+0),A
    DEC IX
    RRC B
    DEC D
    JR NZ,.read
    LD B,0
    LD A,(matrix_scan)
    RRA
    JR NC,.symbol
    SET 5,B
.symbol:
    LD A,(matrix_scan+7)
    BIT 1,A
    JR Z,.shifts
    SET 7,B
.shifts:
    LD A,B
    LD (physical_shifts),A
    LD IX,matrix_scan
    LD IY,keys_down
    LD C,0
    LD D,8
.row:
    LD E,(IX+0)
    LD B,5
.key:
    RRC E
    LD A,0
    ADC A,0
    CP (IY+0)
    LD (IY+0),A
    JR Z,.next
    OR A
    JR Z,.next
    LD A,C
    ; Native CAPS+SYMBOL+ENTER changes the visible 80-column half and
    ; consumes ENTER. It is an edge-triggered screen action, not a token.
    CP 30
    JR NZ,.append
    LD A,(physical_shifts)
    AND $A0
    CP $A0
    JR NZ,.append
    LD A,1
    LD (FLIP_PENDING),A
    JR .next
.append:
    PUSH BC
    PUSH DE
    LD A,(physical_shifts)
    LD B,A
    CALL key_append
    LD A,C
    LD (repeat_key),A
    LD A,(repeat_delay)
    LD (repeat_count),A
    LD A,1
    LD (repeat_valid),A
    LD (new_key),A
    POP DE
    POP BC
.next:
    INC IY
    INC C
    DJNZ .key
    INC IX
    DEC D
    JR NZ,.row
    POP IY
    POP IX
    POP HL
    POP DE
    POP BC
    POP AF
    RET

keyboard_tick:
    CALL keyboard_poll
    LD A,(new_key)
    OR A
    RET NZ
    LD A,(repeat_valid)
    OR A
    RET Z
    LD A,(repeat_key)
    LD L,A
    LD H,0
    LD DE,keys_down
    ADD HL,DE
    LD A,(HL)
    OR A
    JR Z,.released
    LD HL,repeat_count
    DEC (HL)
    RET NZ
    LD A,(key_count)
    OR A
    JR NZ,.blocked
    LD A,(repeat_key)
    LD C,A
    LD A,(physical_shifts)
    OR 4
    LD B,A
.emit:
    ; Repeat flags are raw tokens; translation applies the repeat bitmap.
    CALL key_append
    LD A,(repeat_period)
    LD (repeat_count),A
    RET
.blocked:
    LD A,1
    LD (repeat_count),A
    RET
.released:
    XOR A
    LD (repeat_valid),A
    RET

repeat_allowed:
    PUSH BC
    PUSH HL
    LD A,C
    AND 7
    LD B,A
    LD A,C
    SRL A
    SRL A
    SRL A
    LD L,A
    LD H,0
    LD DE,key_repeat_mask
    ADD HL,DE
    LD A,(HL)
    INC B
.rotate:
    RRA
    DJNZ .rotate
    POP HL
    POP BC
    RET

key_append:
    LD A,(key_count)
    CP 20
    RET NC
    PUSH BC
    PUSH DE
    PUSH HL
    LD A,(key_tail)
    ADD A,A
    LD E,A
    LD D,0
    LD HL,key_queue
    ADD HL,DE
    LD (HL),C
    INC HL
    LD (HL),B
    LD A,(key_tail)
    INC A
    CP 20
    JR C,.tail
    XOR A
.tail:
    LD (key_tail),A
    LD HL,key_count
    INC (HL)
    POP HL
    POP DE
    POP BC
    RET

key_get_raw:
    LD A,(key_count)
    OR A
    RET Z
    LD A,(key_head)
    ADD A,A
    LD E,A
    LD D,0
    LD HL,key_queue
    ADD HL,DE
    LD C,(HL)
    INC HL
    LD B,(HL)
    LD A,(key_head)
    INC A
    CP 20
    JR C,.head
    XOR A
.head:
    LD (key_head),A
    LD HL,key_count
    DEC (HL)
    SCF
    RET

km_get:
    XOR A
    LD (pending_valid),A
    PUSH DE
    PUSH HL
    CALL key_get_raw
    JR NC,.empty
    LD HL,key_locks
    LD A,B
    AND $A0
    CP $20
    JR NZ,.locks
    LD A,C
    CP 16
    JR NZ,.locks
    LD A,(HL)
    XOR 8
    LD (HL),A
.locks:
    LD A,B
    OR (HL)
    LD B,A
    SCF
    JR .return
.empty:
    LD A,(physical_shifts)
    LD HL,key_locks
    OR (HL)
    LD B,A
    OR A
.return:
    POP HL
    POP DE
    RET

km_put:
    PUSH AF
    PUSH BC
    PUSH DE
    PUSH HL
    XOR A
    LD (pending_valid),A
    LD A,B
    AND $58
    LD (key_locks),A
    LD A,(key_count)
    CP 20
    JR C,.room
    LD A,(key_tail)
    DEC A
    JP P,.tail
    LD A,19
.tail:
    LD (key_tail),A
    JR .prepend
.room:
    LD HL,key_count
    INC (HL)
.prepend:
    LD A,(key_head)
    DEC A
    JP P,.head
    LD A,19
.head:
    LD (key_head),A
    ADD A,A
    LD E,A
    LD D,0
    LD HL,key_queue
    ADD HL,DE
    LD (HL),C
    INC HL
    LD (HL),B
    POP HL
    POP DE
    POP BC
    POP AF
    RET

km_set_key:
    PUSH BC
    PUSH DE
    LD A,C
    CP 40
    JR NC,.done
    LD L,C
    LD H,0
    LD DE,key_translation
    ADD HL,DE
    POP DE
    PUSH DE
    LD E,4
.loop:
    SRL D
    JR NC,.skip
    LD (HL),B
.skip:
    PUSH DE
    LD DE,40
    ADD HL,DE
    POP DE
    DEC E
    JR NZ,.loop
.done:
    POP DE
    POP BC
    RET

km_set_speed:
    LD A,H
    LD (repeat_delay),A
    LD A,L
    LD (repeat_period),A
    RET

keyboard_translate:
    LD A,C
    CP 40
    JR NC,.ignore
    BIT 2,B
    CALL NZ,repeat_allowed
    JR NC,.ignore
    LD E,C
    LD D,0
    BIT 4,B
    JR Z,.table
    LD HL,key_translation
    PUSH BC
    LD B,4
.numeric:
    PUSH HL
    ADD HL,DE
    LD A,(HL)
    POP HL
    CALL is_numeric
    JR C,.numeric_found
    PUSH DE
    LD DE,40
    ADD HL,DE
    POP DE
    DJNZ .numeric
    POP BC
.table:
    LD HL,key_translation
    LD A,B
    BIT 1,A
    JR NZ,.extend
    AND $E0
    CP $A0
    JR Z,.extend
    CP $C0
    JR Z,.extend
    CP $E0
    JR Z,.extend
    BIT 7,B
    JR NZ,.symbol
    AND $60
    JR Z,.lookup
    LD HL,key_translation+40
    JR .lookup
.symbol:
    LD HL,key_translation+80
    JR .lookup
.extend:
    LD HL,key_translation+120
.lookup:
    ADD HL,DE
    LD A,(HL)
    CP $9F
    JR Z,.ignore
    BIT 3,B
    JR Z,.ready
    CP $61
    JR C,.accent
    CP $7B
    JR C,.upper
.accent:
    CP $E0
    JR C,.ready
    CP $FC
    JR NC,.ready
.upper:
    SUB $20
.ready:
    SCF
    RET
.numeric_found:
    POP BC
    SCF
    RET
.ignore:
    OR A
    RET
is_numeric:
    CP 13
    SCF
    RET Z
    CP '0'
    JR C,.punctuation
    CP '9'+1
    RET
.punctuation:
    CP '+'
    SCF
    RET Z
    CP '-'
    SCF
    RET Z
    CP '.'
    SCF
    RET Z
    CP ','
    SCF
    RET Z
    OR A
    RET

keyboard_peek:
    LD A,(pending_valid)
    OR A
    RET NZ
.next:
    LD A,(expansion_remaining)
    OR A
    JR Z,.token
    LD A,(expansion_position)
    LD L,A
    LD H,0
    LD DE,expansion_output
    ADD HL,DE
    LD A,(HL)
    LD (pending_key),A
    LD A,(active_expansion)
    LD (pending_expansion),A
    LD A,2
    LD (pending_source),A
    JR .pending
.token:
    LD A,(key_count)
    OR A
    RET Z
    LD A,(key_head)
    ADD A,A
    LD E,A
    LD D,0
    LD HL,key_queue
    ADD HL,DE
    LD C,(HL)
    INC HL
    LD A,(key_locks)
    OR (HL)
    LD B,A
    ; GET toggles CAPS LOCK before translation; predict it without consuming.
    LD A,C
    CP 16
    JR NZ,.translate
    LD A,B
    AND $A0
    CP $20
    JR NZ,.translate
    LD A,B
    XOR 8
    LD B,A
.translate:
    CALL keyboard_translate
    JR NC,.discard
    CP $80
    JR C,.literal
    CP $9F
    JR NC,.literal
    LD (pending_expansion),A
    CALL expansion_address
    LD A,(HL)
    OR A
    JR Z,.discard
    LD (pending_length),A
    LD C,A
    LD B,0
    INC HL
    LD DE,expansion_output
    LDIR
    LD A,(expansion_output)
    LD (pending_key),A
    LD A,1
    LD (pending_source),A
    JR .pending
.discard:
    CALL km_get                    ; ignored keys still update lock state
    JR .next
.literal:
    LD (pending_key),A
    LD A,$FF
    LD (pending_expansion),A
    XOR A
    LD (pending_source),A
.pending:
    LD A,1
    LD (pending_valid),A
    RET

keyboard_read:
    LD A,(pending_source)
    CP 2
    JR Z,.advance
    CALL km_get
    LD A,(pending_source)
    OR A
    JR Z,.return
    LD A,(pending_length)
    LD (expansion_remaining),A
    LD A,(pending_expansion)
    LD (active_expansion),A
    XOR A
    LD (expansion_position),A
.advance:
    LD HL,expansion_position
    INC (HL)
    LD HL,expansion_remaining
    DEC (HL)
.return:
    XOR A
    LD (pending_valid),A
    LD A,(pending_key)
    RET

expansion_address:
    SUB $80
    LD HL,key_expansions
    OR A
    RET Z
    LD B,A
.loop:
    LD E,(HL)
    LD D,0
    INC HL
    ADD HL,DE
    DJNZ .loop
    RET

km_set_expand:
    LD A,B
    AND $7F                     ; native accepts 00..1E and 80..9E
    CP $1F
    JP NC,.failed
    OR $80
    LD (expansion_token),A
    LD A,C
    CP 121
    JP NC,.failed
    LD (expansion_length),A
    LD (expansion_source),HL
    OR A
    JR Z,.source_ok
    LD A,H
    CP $40
    JP C,.failed
    LD D,0
    LD E,C
    ADD HL,DE
    JP C,.failed
.source_ok:
    LD A,(expansion_token)
    CALL expansion_address
    LD (expansion_old),HL
    LD A,(HL)
    LD (expansion_old_length),A
    LD E,A
    LD D,0
    LD HL,(expansion_used)
    OR A
    SBC HL,DE
    LD A,(expansion_length)
    LD E,A
    ADD HL,DE
    LD (expansion_new_used),HL
    LD DE,152
    OR A
    SBC HL,DE
    JP NC,.failed
    LD HL,(expansion_old)
    LD DE,key_expansions
    OR A
    SBC HL,DE
    LD (expansion_prefix),HL
    LD B,H
    LD C,L
    LD HL,key_expansions
    LD DE,expansion_scratch
    LD A,B
    OR C
    JR Z,.replace
    LDIR
.replace:
    LD A,(expansion_length)
    LD (DE),A
    INC DE
    LD C,A
    LD B,0
    LD HL,(expansion_source)
    OR A
    JR Z,.tail
    LDIR
.tail:
    LD HL,(expansion_old)
    INC HL
    LD A,(expansion_old_length)
    LD C,A
    LD B,0
    ADD HL,BC
    PUSH HL
    PUSH DE
    LD HL,(expansion_used)
    LD BC,(expansion_prefix)
    OR A
    SBC HL,BC
    LD A,(expansion_old_length)
    LD C,A
    INC BC
    OR A
    SBC HL,BC
    LD B,H
    LD C,L
    POP DE
    POP HL
    LD A,B
    OR C
    JR Z,.commit
    LDIR
.commit:
    LD HL,expansion_scratch
    LD DE,key_expansions
    LD BC,(expansion_new_used)
    LD (expansion_used),BC
    LDIR
    LD A,(expansion_token)
    LD HL,active_expansion
    CP (HL)
    JR NZ,.pending
    XOR A
    LD (expansion_remaining),A
.pending:
    LD A,(expansion_token)
    LD HL,pending_expansion
    CP (HL)
    JR NZ,.ok
    XOR A
    LD (pending_valid),A
.ok:
    SCF
    RET
.failed:
    OR A
    RET
