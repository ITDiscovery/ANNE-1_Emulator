#ifndef PIN_ASSIGNMENT_H
#define PIN_ASSIGNMENT_H

// --- ESP8266 PINS (NodeMCU/Wemos Mappings) ---
/* --- ESP8266 PIN MAP & BOOT TRAPS ---
   
   CRITICAL BOOT PINS (Strapping Pins):
   - D3 (GPIO 0):  Must be HIGH at boot. Low = Flash Mode. (Don't hold button at power on!)
   - D4 (GPIO 2):  Must be HIGH at boot. Low = Boot Failure. (Linked to onboard LED).
   - D8 (GPIO 15): Must be LOW at boot.  High = Boot Failure. (Has weak pull-down).
   
   SAFE PINS:
   - D1, D2, D5, D6, D7 (Standard GPIO)
   
   SERIAL PINS:
   - TX (GPIO 1) / RX (GPIO 3): Reserved for Monitor/Upload.
*/ 

// Display Pins (TM1638)
#define PIN_STB1    16  // D0 (GPIO16) -> Pin 37 on DSKY
#define PIN_DIO      5  // D1 (GPIO5)  -> Pin 35 on DSKY 
#define PIN_CLK     14  // D5 (GPIO14) -> Pin 33 on DSKY

// CPU Control Buttons (Interrupt Driven)
#define PIN_RS_BTN   4  // D2 (GPIO4)  -> Reset Button (Safe)
#define PIN_ST_BTN   0  // D3 (GPIO0)  -> NMI/Stop Button (Safe-ish)

// Mode Jumper
#define PIN_MODE_SELECT 13 // D7 (GPIO13) -> Existing TTY/Keypad Jumper

// Tape Interface (Future Use)
#define PIN_TAPE_OUT 12   // D6
#define PIN_TAPE_IN  A0   // A0

#endif
