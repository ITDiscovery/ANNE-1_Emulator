; ANN-1 Monitor "GoldMaster 11 (CoCo Map)"
; Target: 6809 (asm6809 compatible)
; Memory Map:
;   $0000-$3FFF: User RAM
;   $C000-$EFFF: Extended BASIC (12K Window)
;   $F000-$FFFF: Monitor ROM (4K)
;   $FFxx      : Memory Mapped I/O (Punches through Monitor)
    
    ORG $F000           ; <--- MOVED from $E000

; --- HARDWARE MAPPINGS (CoCo Style) ---
KEY_IN   EQU $FF00      ; <--- MOVED from $C000 (PIA0 Data)
DISP_HI  EQU $FF20      ; <--- MOVED from $C001 (Custom)
DISP_LO  EQU $FF21      ; <--- MOVED from $C002 (Custom)
DISP_DAT EQU $FF22      ; <--- MOVED from $C003 (Custom)
ACIA_STS EQU $FF68      ; <--- MOVED from $C004 (RS232 Pak Status)
ACIA_DAT EQU $FF69      ; <--- MOVED from $C005 (RS232 Pak Data)

; --- ZERO PAGE VARIABLES ---
    ORG $0000
CUR_ADDR  RMB 2         ; $0000-$0001
LAST_KEY  RMB 1         ; $0002
MODE      RMB 1         ; $0003
INPUT_BUF RMB 2         ; $0004-$0005 (TTY Scratchpad)

; --- CONSTANTS ---
CR      EQU $0D
LF      EQU $0A
SPACE   EQU $20

; --- API VECTORS (Fixed Jump Table) ---
    ORG $F000           ; <--- MOVED from $E000
J_SCAND:  LBRA SCAND
J_SCANDS: LBRA SCANDS
J_GETKEY: LBRA GETKEY
J_INCPT:  LBRA INCPT
J_AK:     LBRA AK
J_KEYIN:  LBRA KEYIN

; --- SUBROUTINES (KEYPAD/LED) ---
SCAND:
    PSHS A,B,X
    LDA CUR_ADDR
    STA DISP_HI
    LDA CUR_ADDR+1
    STA DISP_LO
    LDX CUR_ADDR
    LDA ,X
    STA DISP_DAT
    JSR SCANDS
    PULS A,B,X,PC

SCANDS:
    PSHS X
    LDX #$0010
_SD_LOOP:
    LEAX -1,X
    BNE _SD_LOOP
    PULS X,PC

KEYIN: RTS

AK:
    PSHS A
    LDA KEY_IN
    CMPA #$FF
    BEQ _AK_NO
    CMPA #0
    BEQ _AK_NO
    PULS A
    ORCC #$04
    RTS
_AK_NO:
    PULS A
    ANDCC #$FB
    RTS

GETKEY:
    PSHS B,X
_GK_WAIT:
    JSR SCAND
    JSR AK
    BNE _GK_WAIT
    LDA KEY_IN
    STA LAST_KEY
    LDX #$0100          
_GK_DB:
    LEAX -1,X
    BNE _GK_DB
_GK_REL:
    JSR AK
    BEQ _GK_REL
    LDA LAST_KEY
    CMPA #'0'
    BLO _GK_CMD
    CMPA #'9'
    BHI _GK_ALPHA
    SUBA #'0'
    BRA _GK_DONE
_GK_ALPHA:
    CMPA #'A'
    BLO _GK_CMD
    CMPA #'F'
    BHI _GK_CMD
    SUBA #'A'
    ADDA #10
    BRA _GK_DONE
_GK_CMD:
    CMPA #'G'
    BEQ _RET_10
    CMPA #'H'
    BEQ _RET_11
    CMPA #'+'
    BEQ _RET_12
    CMPA #'X'
    BEQ _RET_13
    CMPA #'P'
    BEQ _RET_14
    CMPA #'U'
    BEQ _RET_16
    CMPA #'V'
    BEQ _RET_17
    BRA _GK_WAIT
_RET_10: LDA #$10
    BRA _GK_DONE
_RET_11: LDA #$11
    BRA _GK_DONE
_RET_12: LDA #$12
    BRA _GK_DONE
_RET_13: LDA #$13
    BRA _GK_DONE
_RET_14: LDA #$14
    BRA _GK_DONE
_RET_16: LDA #$16
    BRA _GK_DONE
_RET_17: LDA #$17
    BRA _GK_DONE
_GK_DONE:
    PULS B,X,PC

INCPT:
    PSHS X
    LDX CUR_ADDR
    LEAX 1,X
    STX CUR_ADDR
    PULS X,PC

; --- TTY PRIMITIVES ---
GETCH:
    PSHS A
_GC_WAIT:
    JSR SCAND       ; Refresh display one frame
    
    LDA ACIA_STS    ; Read Status
    BITA #$01       ; <--- FIX: Check Bit 0 (RDRF)
    BEQ _GC_WAIT    ; If 0 (Not Ready), loop back
    
    LDA ACIA_DAT    ; Read Data
    TFR A,B         ; Move to B to save it
    PULS A
    TFR B,A         ; Move back to A
    RTS

OUTCH:
    PSHS A
_OUT_WAIT:
    LDA ACIA_STS    ; Read Status
    BITA #$02       ; <--- FIX: Check Bit 1 (TDRE)
    BEQ _OUT_WAIT   ; Wait for empty (Standard 6850 behavior)
    PULS A
    STA ACIA_DAT    ; Write Data
    RTS

; --- MAIN RESET HANDLER ---
RESET:
    LDS #$3FFF
    LDX #$0000
    STX CUR_ADDR
    CLR MODE

    ; --- TTY DETECTION ---
    LDA ACIA_STS    ; Read Status ($FF68)
    BITA #$80       ; <--- FIX: Check Bit 7 (The Jumper)
    LBNE TTY_MODE   ; If Bit 7 is set, jump to TTY Monitor

    ; Fall through to Standard Keypad Monitor
MONITOR_LOOP:
    JSR GETKEY
    CMPA #$0F
    BLS HANDLE_DIGIT
    CMPA #$10
    LBEQ CMD_AD
    CMPA #$11
    LBEQ CMD_DA
    CMPA #$12
    LBEQ CMD_INC
    CMPA #$13
    LBEQ CMD_GO
    CMPA #$14
    LBEQ CMD_PC
    LBRA MONITOR_LOOP

HANDLE_DIGIT:
    PSHS A
    TST MODE
    BNE DATA_MODE
ADDR_MODE:
    LDD CUR_ADDR
    ASLB
    ROLA
    ASLB
    ROLA
    ASLB
    ROLA
    ASLB
    ROLA
    STD CUR_ADDR
    PULS A
    ORA CUR_ADDR+1
    STA CUR_ADDR+1
    LBRA MONITOR_LOOP
DATA_MODE:
    LDX CUR_ADDR
    LDB ,X
    ASLB
    ASLB
    ASLB
    ASLB
    STB ,X
    PULS A
    ORA ,X
    STA ,X
    LBRA MONITOR_LOOP

CMD_AD:
    CLR MODE
    LBRA MONITOR_LOOP
CMD_DA:
    LDA #1
    STA MODE
    LBRA MONITOR_LOOP
CMD_INC:
    JSR INCPT
    LBRA MONITOR_LOOP
CMD_PC:
    LDX #$F000      ; <--- UPDATED: Point to Monitor Start
    STX CUR_ADDR
    LBRA MONITOR_LOOP
CMD_GO:
    LDX CUR_ADDR
    JMP ,X
CMD_RS:
    JMP RESET

; =================================================================
; --- NEW TTY MONITOR IMPLEMENTATION (Full Feature) ---
; =================================================================

TTY_MODE:
    LDS #$3FFF         ; Safety reset of stack
    LDX #STR_BANNER    ; Print "ANNE-1"
    JSR OUTSTR

    ; Check if BASIC is present at $C000
    ; A simple check: Is the byte at $C000 usually $7E (JMP)?
    ; Or just print the prompt.

TTY_PROMPT:
    JSR OUT_CRLF
    LDA #'>'           ; Prompt character
    JSR OUTCH
    
    CLR INPUT_BUF      ; Clear input buffer
    CLR INPUT_BUF+1
    
TTY_INPUT_LOOP:
    JSR GETCH          ; Wait for character
    CMPA #$60          ; Check if lower case
    BLO _PROCESS_KEY
    SUBA #$20          ; Convert to Upper Case

_PROCESS_KEY:
    JSR OUTCH          ; Echo character
    
    ; --- COMMAND HANDLING ---
    CMPA #'G'          ; 'G' = GO
    BEQ TTY_GO
    CMPA #'R'          ; 'R' = RESET
    LBEQ RESET
    CMPA #'B'          ; 'B' = BASIC (NEW SHORTCUT)
    BEQ TTY_BASIC
    CMPA #'S'          ; 'S' = ST (Stop/Reset)
    LBEQ RESET
    CMPA #'X'          ; 'X' = AD (Exit Data Mode)
    BEQ TTY_TO_AD
    CMPA #'P'          ; 'P' = PC (Restore PC View)
    BEQ TTY_PC
    CMPA #'+'          ; '+' = Increment Address
    BEQ TTY_INC
    
    ; --- ENTER / SPACE ---
    CMPA #SPACE        ; SPACE = Commit Input
    BEQ TTY_SPACE
    CMPA #CR           ; ENTER = Commit Input
    BEQ TTY_SPACE
    
    ; --- HEX DIGITS ---
    CMPA #'0'
    BLO TTY_INPUT_LOOP ; Ignore non-digits
    CMPA #'9'
    BLS _IS_DIGIT
    CMPA #'A'
    BLO TTY_INPUT_LOOP ; Ignore garbage between 9 and A
    CMPA #'F'
    BHI TTY_INPUT_LOOP ; Ignore > F

    SUBA #7            ; Adjust ASCII 'A'-'F' down
_IS_DIGIT:
    SUBA #'0'          ; Convert '0'-'9' to 0-9
    
    ; Shift input buffer left by 4 bits
    PSHS A             ; Save new nibble
    
    LDD INPUT_BUF      ; Load 16-bit D (A=Hi, B=Lo)
    ASLB               ; Shift D left 4 times
    ROLA
    ASLB
    ROLA
    ASLB
    ROLA
    ASLB
    ROLA
    STD INPUT_BUF      ; Store back shifted D
    
    PULS A             ; Get new nibble back
    ORA INPUT_BUF+1    ; OR into bottom of Low Byte
    STA INPUT_BUF+1
    
    BRA TTY_INPUT_LOOP

; --- COMMAND HANDLERS ---

TTY_GO:
    JSR OUT_CRLF
    LDX CUR_ADDR
    JMP ,X             ; Jump to user program

TTY_BASIC:
    JSR OUT_CRLF
    JMP $C000          ; <--- NEW: Jump directly to BASIC Cold Start

TTY_TO_AD:
    CLR MODE           ; Set Address Mode
    LBRA TTY_PROMPT     ; New Prompt

TTY_PC:
    LDX #$F000         ; <--- UPDATED: Reset PC to Monitor Start
    STX CUR_ADDR
    CLR MODE           ; Switch to Address Mode
    LBRA TTY_PROMPT

TTY_INC:
    ; Increment Address (Data Mode style) without writing
    LDX CUR_ADDR
    LEAX 1,X
    STX CUR_ADDR
    ; Print New Line, Address, and Data
    JSR OUT_CRLF
    LDX CUR_ADDR
    JSR OUTWORD        ; Print Address
    LDA #SPACE
    JSR OUTCH
    LDA ,X             ; Read Data
    JSR OUTHEX         ; Print Data
    CLR INPUT_BUF      ; Clear buffer
    CLR INPUT_BUF+1
    LBRA TTY_INPUT_LOOP

TTY_SPACE:
    ; Space pressed. Action depends on MODE.
    TST MODE
    BNE _TTY_DATA_MODE

    ; ADDRESS MODE logic:
    ; Move Input Buffer value to Current Address
    LDX INPUT_BUF
    STX CUR_ADDR
    
    ; Print Data at this address
    LDA #SPACE
    JSR OUTCH
    LDA #'-'
    JSR OUTCH
    LDA #SPACE
    JSR OUTCH
    
    LDX CUR_ADDR
    LDA ,X             ; Read byte from RAM
    JSR OUTHEX         ; Print it
    
    INC MODE           ; Switch to Data Mode for next input
    CLR INPUT_BUF      ; Clear buffer for next entry
    CLR INPUT_BUF+1
    LBRA TTY_INPUT_LOOP

_TTY_DATA_MODE:
    ; DATA MODE logic:
    ; Write Low Byte of Input Buffer to RAM
    LDA INPUT_BUF+1
    LDX CUR_ADDR
    STA ,X             ; Write to Memory!
    
    ; Increment Address
    LEAX 1,X
    STX CUR_ADDR
    
    ; Print New Line and Next Address
    JSR OUT_CRLF
    LDX CUR_ADDR
    JSR OUTWORD        ; Print Address
    LDA #SPACE
    JSR OUTCH
    
    ; Show Data at new address
    LDA ,X
    JSR OUTHEX
    
    CLR INPUT_BUF      ; Clear input
    CLR INPUT_BUF+1
    LBRA TTY_INPUT_LOOP


; --- HELPER ROUTINES ---

OUT_CRLF:
    PSHS A
    LDA #CR
    JSR OUTCH
    LDA #LF
    JSR OUTCH
    PULS A,PC

; Print 16-bit value in X as 4 Hex Digits
OUTWORD:
    PSHS A
    TFR X,D     ; Transfer X to D (A:B)
    JSR OUTHEX  ; Print A (High byte)
    TFR B,A     ; Move B to A
    JSR OUTHEX  ; Print B (Low byte)
    PULS A,PC

; Print 8-bit value in A as 2 Hex Digits
OUTHEX:
    PSHS A
    PSHS A      ; Save original value
    LSRA        ; Shift top nibble down
    LSRA
    LSRA
    LSRA
    JSR OUTNIB  ; Print top nibble
    PULS A      ; Restore original
    ANDA #$0F   ; Mask bottom nibble
    JSR OUTNIB  ; Print bottom nibble
    PULS A,PC

OUTNIB:
    CMPA #9
    BHI _IS_AF
    ADDA #'0'
    BRA _PRN
_IS_AF:
    ADDA #'A'-10
_PRN:
    JSR OUTCH
    RTS

OUTSTR:
    LDA ,X+
    BEQ _STR_DONE
    JSR OUTCH
    BRA OUTSTR
_STR_DONE:
    RTS

STR_BANNER:
    FCB CR,LF
    FCC "ANNE-1 Monitor V11"
    FCB CR,LF,0

; --- VECTORS ---
    ORG $FFF0
    FDB RESET, RESET, RESET, RESET, RESET, RESET, RESET, RESET
