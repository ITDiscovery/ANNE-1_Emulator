; ANNE-1 Monitor "GoldMaster 12"
; Fixes: 'B' Shortcut removed, Variables moved to Safe RAM ($3F00)

    ORG $F000

; --- HARDWARE MAPPINGS ---
KEY_IN   EQU $FF00
DISP_HI  EQU $FF20
DISP_LO  EQU $FF21
DISP_DAT EQU $FF22
ACIA_STS EQU $FF68
ACIA_DAT EQU $FF69

; --- VARIABLES (MOVED TO TOP OF RAM) ---
; Moved from $0000 to $3F00 to protect BASIC Zero Page
VAR_BASE  EQU $3F00 
CUR_ADDR  EQU VAR_BASE+0  
LAST_KEY  EQU VAR_BASE+2  
MODE      EQU VAR_BASE+3  
INPUT_BUF EQU VAR_BASE+4  

; --- CONSTANTS ---
CR      EQU $0D
LF      EQU $0A
SPACE   EQU $20
BSC     EQU $C000      ; BASIC Cold Start

; --- API VECTORS ---
    LBRA SCAND
    LBRA SCANDS
    LBRA GETKEY
    LBRA INCPT
    LBRA AK
    LBRA KEYIN

; --- KEYPAD DRIVERS ---
SCAND:  PSHS A,B,X
        LDX CUR_ADDR   ; Uses new $3F00 address
        LDA CUR_ADDR
        STA DISP_HI
        LDA CUR_ADDR+1
        STA DISP_LO
        LDA ,X
        STA DISP_DAT
        JSR SCANDS
        PULS A,B,X,PC

SCANDS: PSHS X
        LDX #$0010
_SD_LP: LEAX -1,X
        BNE _SD_LP
        PULS X,PC

KEYIN:  RTS

AK:     PSHS A
        LDA KEY_IN
        CMPA #$FF
        BEQ _AK_N
        CMPA #0
        BEQ _AK_N
        PULS A
        ORCC #$04
        RTS
_AK_N:  PULS A
        ANDCC #$FB
        RTS

GETKEY: PSHS B,X
_GK_W:  JSR SCAND
        JSR AK
        BNE _GK_W
        LDA KEY_IN
        STA LAST_KEY
        LDX #$0100
_GK_DB: LEAX -1,X
        BNE _GK_DB
_GK_R:  JSR AK
        BEQ _GK_R
        LDA LAST_KEY
        CMPA #'0'
        BLO _GK_C
        CMPA #'9'
        BHI _GK_A
        SUBA #'0'
        BRA _GK_D
_GK_A:  CMPA #'A'
        BLO _GK_C
        CMPA #'F'
        BHI _GK_C
        SUBA #'A'
        ADDA #10
        BRA _GK_D
_GK_C:  CMPA #'a'
        BEQ _RET10
        CMPA #'d'
        BEQ _RET11
        CMPA #'+'
        BEQ _RET12
        CMPA #'X'
        BEQ _RET13
        CMPA #'P'
        BEQ _RET14
        CMPA #'U'
        BEQ _RET16
        CMPA #'V'
        BEQ _RET17
        BRA _GK_W
_RET10: LDA #$10
        BRA _GK_D
_RET11: LDA #$11
        BRA _GK_D
_RET12: LDA #$12
        BRA _GK_D
_RET13: LDA #$13
        BRA _GK_D
_RET14: LDA #$14
        BRA _GK_D
_RET16: LDA #$16
        BRA _GK_D
_RET17: LDA #$17
        BRA _GK_D
_GK_D:  PULS B,X,PC

INCPT:  PSHS X
        LDX CUR_ADDR
        LEAX 1,X
        STX CUR_ADDR
        PULS X,PC

; --- TTY DRIVERS ---
GETCH:  PSHS A
_GC_W:  JSR SCAND
        LDA ACIA_STS
        BITA #$01
        BEQ _GC_W
        LDA ACIA_DAT
        TFR A,B
        PULS A
        TFR B,A
        RTS

OUTCH:  PSHS A
_OUT_W: LDA ACIA_STS
        BITA #$02
        BEQ _OUT_W
        PULS A
        STA ACIA_DAT
        RTS

; --- RESET ---
RESET:
    LDS #$3FFF
    LDX #$0000
    STX CUR_ADDR
    CLR MODE
    STX INPUT_BUF

    LDA ACIA_STS
    BITA #$80
    LBNE TTY_MODE

    ; Keypad Monitor Loop
MON_LOOP:
    JSR GETKEY
    CMPA #$0F
    BLS DIGIT_H
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
    BRA MON_LOOP

DIGIT_H:
    PSHS A
    TST MODE
    BNE D_MODE
A_MODE: LDD CUR_ADDR
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
    BRA MON_LOOP
D_MODE: LDX CUR_ADDR
    LDB ,X
    ASLB
    ASLB
    ASLB
    ASLB
    STB ,X
    PULS A
    ORA ,X
    STA ,X
    BRA MON_LOOP

CMD_AD: CLR MODE
        BRA MON_LOOP
CMD_DA: LDA #1
        STA MODE
        BRA MON_LOOP
CMD_INC: JSR INCPT
        BRA MON_LOOP
CMD_PC: LDX #$F000
        STX CUR_ADDR
        BRA MON_LOOP
CMD_GO: LDX CUR_ADDR
        JMP ,X

; --- TTY MONITOR ---
TTY_MODE:
    LDS #$3FFF
    LDX #STR_BNR
    JSR OUTSTR

TTY_PRPT:
    JSR OUT_CRLF
    LDA #'>'
    JSR OUTCH
    CLR INPUT_BUF
    CLR INPUT_BUF+1

TTY_LOOP:
    JSR GETCH
    CMPA #$60
    BLO _KEY_U
    SUBA #$20
_KEY_U:
    JSR OUTCH

    ; --- COMMANDS ---
    CMPA #'G'       ; GO
    BEQ TTY_GO
    CMPA #'R'       ; RESET
    LBEQ RESET
    CMPA #'X'       ; EXIT DATA MODE
    BEQ TTY_AD
    CMPA #'P'       ; PC VIEW
    BEQ TTY_PC
    CMPA #'+'       ; INC
    BEQ TTY_INC
    CMPA #SPACE
    BEQ TTY_SP
    CMPA #CR
    BEQ TTY_SP
    
    ; --- HEX PARSING ---
    CMPA #'0'
    BLO TTY_LOOP
    CMPA #'9'
    BLS _IS_D
    CMPA #'A'
    BLO TTY_LOOP
    CMPA #'F'
    BHI TTY_LOOP
    SUBA #7
_IS_D:
    SUBA #'0'
    
    PSHS A
    LDD INPUT_BUF
    ASLB
    ROLA
    ASLB
    ROLA
    ASLB
    ROLA
    ASLB
    ROLA
    STD INPUT_BUF
    PULS A
    ORA INPUT_BUF+1
    STA INPUT_BUF+1
    BRA TTY_LOOP

TTY_GO:
    JSR OUT_CRLF
    LDX CUR_ADDR
    JMP ,X

TTY_BAS:
    JSR OUT_CRLF
    JMP BSC

TTY_AD:
    CLR MODE
    BRA TTY_PRPT

TTY_PC:
    LDX #$F000
    STX CUR_ADDR
    CLR MODE
    LBRA TTY_PRPT

TTY_INC:
    LDX CUR_ADDR
    LEAX 1,X
    STX CUR_ADDR
    JSR OUT_CRLF
    LDX CUR_ADDR
    JSR OUTWORD
    LDA #SPACE
    JSR OUTCH
    LDA ,X
    JSR OUTHEX
    CLR INPUT_BUF
    CLR INPUT_BUF+1
    LBRA TTY_LOOP

TTY_SP:
    TST MODE
    BNE _DT_MD
    LDX INPUT_BUF
    STX CUR_ADDR
    LDA #SPACE
    JSR OUTCH
    LDA #'-'
    JSR OUTCH
    LDA #SPACE
    JSR OUTCH
    LDX CUR_ADDR
    LDA ,X
    JSR OUTHEX
    INC MODE
    CLR INPUT_BUF
    CLR INPUT_BUF+1
    LBRA TTY_LOOP
    
_DT_MD:
    LDA INPUT_BUF+1
    LDX CUR_ADDR
    STA ,X
    LEAX 1,X
    STX CUR_ADDR
    JSR OUT_CRLF
    LDX CUR_ADDR
    JSR OUTWORD
    LDA #SPACE
    JSR OUTCH
    LDA ,X
    JSR OUTHEX
    CLR INPUT_BUF
    CLR INPUT_BUF+1
    LBRA TTY_LOOP

; --- HELPERS ---
OUT_CRLF: PSHS A
          LDA #CR
          JSR OUTCH
          LDA #LF
          JSR OUTCH
          PULS A,PC

OUTWORD:  PSHS A
          TFR X,D
          JSR OUTHEX
          TFR B,A
          JSR OUTHEX
          PULS A,PC

OUTHEX:   PSHS A
          PSHS A
          LSRA
          LSRA
          LSRA
          LSRA
          JSR OUTNIB
          PULS A
          ANDA #$0F
          JSR OUTNIB
          PULS A,PC

OUTNIB:   CMPA #9
          BHI _AF
          ADDA #'0'
          BRA _PR
_AF:      ADDA #'A'-10
_PR:      JSR OUTCH
          RTS

OUTSTR:   LDA ,X+
          BEQ _SD
          JSR OUTCH
          BRA OUTSTR
_SD:      RTS

STR_BNR:  FCB CR,LF
          FCC "ANNE-1 Mon V12"
          FCB CR,LF,0

; --- ROM VECTORS ---
    ORG $FFF0
    FDB RESET, RESET, RESET, RESET, RESET, RESET, RESET, RESET
