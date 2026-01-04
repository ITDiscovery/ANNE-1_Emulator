#ifndef ANNEHAL_H
#define ANNEHAL_H

#include <Arduino.h>
#include <pgmspace.h> 
#include "TM1638.h"

class MOTO6809; 

// --- Memory Configuration ---
#define RAM_SIZE     0x4000 // $0000-$3FFF (16KB)
#define BASIC_SIZE   0x3000  // $C000 - $EFFF (12K)
#define ROM_SIZE     0x1000  // $F000-$FFFF (4KB)
#define BASIC_BOTTOM 0xD000
#define BASIC_TOP    0xF000 

// Display Registers (Write-Only)
#define ANNE_DISP_HI     0xFF20
#define ANNE_DISP_LO     0xFF21
#define ANNE_DISP_DAT    0xFF22
#define ANNE_LED_REG     0xFF23

// --- I/O MAP ---
#define ANNE_KEY_IN      0xFF00
#define ANNE_ACIA_STS    0xFF68
#define ANNE_ACIA_DAT    0xFF69
#define ANNE_TAPE_OUT    0xFF24  // Write to GPIO 12
#define ANNE_TAPE_IN     0xFF25  // Read from A0

class ANNHal {
  public:
    TM1638 hardware; 
    
    uint8_t *ram = nullptr;
    const uint8_t *basic_rom = nullptr; 
    uint8_t *rom = nullptr;
    uint8_t *ext_ram = nullptr;        

    bool keypadMode = true; 
    bool debug_dump = false;     
    uint8_t last_key_ascii = 0;   

    // Display Cache
    uint8_t disp_hi = 0;
    uint8_t disp_lo = 0;
    uint8_t disp_dat = 0;

    ANNHal();
    void begin();
    bool initMemory();
    void flashMonitor();

    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t val);
    void updateHardware();
    void system_check(MOTO6809* cpu);
};
#endif