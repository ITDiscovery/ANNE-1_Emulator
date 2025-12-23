#ifndef ANNEHAL_H
#define ANNEHAL_H

#include <Arduino.h>
#include <pgmspace.h> 
#include "TM1638.h"    
// Forward declaration to avoid circular dependency
class MOTO6809; 

// --- Memory Configuration ---
#define RAM_SIZE     16384 // $0000-$3FFF (16KB)
#define BASIC_SIZE   12288 // $C000-$EFFF (12KB)
#define ROM_SIZE     4096  // $F000-$FFFF (4KB)

// Display Registers (Write-Only) - Custom Hardware Area
#define ANNE_DISP_HI     0xFF20  // Digits 0,1
#define ANNE_DISP_LO     0xFF21  // Digits 2,3
#define ANNE_DISP_DAT    0xFF22  // Digits 4,5

// --- I/O MAP (CoCo Compatible) ---
// Keypad (Read-Only) - Mapped to CoCo PIA0 Data A
#define ANNE_KEY_IN      0xFF00

// ACIA (UART) - Mapped to CoCo RS-232 Pak
#define ANNE_ACIA_STS    0xFF68  // Status (Read)
#define ANNE_ACIA_DAT    0xFF69  // Data (Read/Write)

class ANNHal {
  public:
    TM1638 hardware;
    
    // Memory Pointers
    uint8_t *ram = nullptr;
    const uint8_t *basic_rom = nullptr; // Points to PROGMEM array
    uint8_t *rom = nullptr;             // Monitor RAM buffer 

    // Mode Flags
    bool keypadMode = true; 
    bool debug_dump = false;     

    // Keypad State
    uint8_t last_key_ascii = 0;   

    // Display Cache (Shadow Registers)
    uint8_t disp_hi = 0;
    uint8_t disp_lo = 0;
    uint8_t disp_dat = 0;

    ANNHal();

    void begin();
    bool initMemory();
    void flashMonitor();

    // Core Intercepts
    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t val);

    // Hardware Loop
    void updateHardware();
    void system_check(MOTO6809* cpu);
};
#endif