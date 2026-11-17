; +3 keyboard state. Default maps are extracted locally from the user's EMS.
; The twenty-token queue matches the original driver; expansions allow 120
; character bytes plus one length byte for each of 31 tokens.
    DS $2000-$,0
key_translation:
    INCBIN "keyboard_defaults.bin",0,160
key_repeat_mask:
    INCBIN "keyboard_defaults.bin",160,5
key_expansions:
    INCBIN "expansion_defaults.bin"
    DS 109,0
expansion_scratch: DS 151,0
expansion_output: DS 120,0
key_queue: DS 40,0
matrix_scan: DS 8,0
keys_down: DS 40,0
key_head: DB 0
key_tail: DB 0
key_count EQU KEY_COUNT
key_locks EQU KEY_LOCKS
physical_shifts EQU KEY_SHIFTS
new_key: DB 0
repeat_key: DB 0
repeat_valid: DB 0
repeat_count: DB 0
repeat_delay: DB 30
repeat_period: DB 2
pending_key: DB 0
pending_valid EQU KEY_PENDING
pending_source: DB 0
pending_length: DB 0
pending_expansion: DB $FF
active_expansion: DB $FF
expansion_remaining: DB 0
expansion_position: DB 0
expansion_used: DW 42
expansion_token: DB 0
expansion_length: DB 0
expansion_source: DW 0
expansion_old: DW 0
expansion_old_length: DB 0
expansion_new_used: DW 0
expansion_prefix: DW 0
clock_fraction: DB 0
timer_sp: DW 0
timer_csp: DW 0
    ASSERT $ < FDC_BUFFER
