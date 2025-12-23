#include "ANNEHal.h"
#include "ANNE_ROM.h"
#include "BASIC_ROM.h"

ANNHal::ANNHal() {}

void ANNHal::begin() {
    hardware.begin();
    hardware.setupDisplay(true, 7);
    hardware.clear();

    // NEW: Setup Built-in LED for heartbeat
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

bool ANNHal::initMemory() {
    if (ram) free(ram);
    if (rom) free(rom);

    ram = (uint8_t*)malloc(RAM_SIZE);
    rom = (uint8_t*)malloc(ROM_SIZE);
    
    basic_rom = ANNE_BASIC_ROM; 

    if (!ram || !rom) return false;

    // Clear RAM
    memset(ram, 0x00, RAM_SIZE);
    memset(rom, 0xFF, ROM_SIZE);
    
    return true;
}

void ANNHal::flashMonitor() {
    if (rom) memcpy_P(rom, ANNE_MONITOR_ROM, sizeof(ANNE_MONITOR_ROM));
}

void ANNHal::system_check(MOTO6809* cpu) {
    // Intentionally empty for speed
}

// --- BUS READ ---
uint8_t ANNHal::read(uint16_t addr) {
    // 1. RAM ($0000 - $3FFF)
    if (addr < 0x4000) return ram[addr];

    // 2. EXTENDED BASIC ($C000 - $EFFF) [12KB]
    if (addr >= 0xC000 && addr < 0xF000) {
        return pgm_read_byte(&basic_rom[addr - 0xC000]);
    }

    // 3. I/O REGION ($FFxx) 
    if (addr >= 0xFF00 && addr <= 0xFF69) {
        
        // --- OPTIMIZED LED BLINKING ---
        // Helper Lambda to handle state caching
        // Only writes to hardware if the state actually changes
        auto updateLed = [&](uint8_t idx, uint8_t speedDiv) {
            static int lastState[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
            int newState = (millis() / speedDiv) % 2;
            if (lastState[idx] != newState) {
                hardware.setLed(idx, newState);
                lastState[idx] = newState;
            }
        };

        // LED 0: Activity Heartbeat (Slow)
        updateLed(0, 200);

        // A. Keypad Read (Parallel)
        if (addr == ANNE_KEY_IN) {
            updateLed(1, 100); // LED 1: Keypad Access
            return last_key_ascii;
        }

        // B. ACIA Status Register ($FF68)
        if (addr == ANNE_ACIA_STS) {
            updateLed(2, 50); // LED 2: Fast Blink (Polling)
            
            uint8_t status = 0x02; // TDRE always ready
            if (Serial.available() > 0) status |= 0x01; 
            if (!keypadMode) status |= 0x80; 
            return status;
        }

        // C. ACIA Data Register ($FF69)
        if (addr == ANNE_ACIA_DAT) {
             // For Data Read, we want a momentary flash, not a blink pattern.
             // We just force it ON, then rely on the Status polling loop to turn it off eventually
             // or just accept it might stay on briefly. 
             // Ideally, just use the blinker for consistency:
             updateLed(3, 50); 

            if (Serial.available() > 0) return Serial.read();
            return 0;
        }
        
        // D. Display Readback
        if (addr == ANNE_DISP_HI) return disp_hi;
        if (addr == ANNE_DISP_LO) return disp_lo;
        if (addr == ANNE_DISP_DAT) return disp_dat;
    }

    // 4. MONITOR ROM ($F000 - $FFFF) 
    if (addr >= 0xF000) {
        return rom[addr - 0xF000];
    }

    return 0xFF; // Unmapped
}

// --- BUS WRITE ---
void ANNHal::write(uint16_t addr, uint8_t val) {
    // 1. RAM
    if (addr < 0x4000) {
        ram[addr] = val;
        return;
    }

    // 2. I/O Region ($FFxx)
    if (addr >= 0xFF00 && addr <= 0xFF69) {
        if (addr == ANNE_DISP_HI) {
            disp_hi = val;
            hardware.writeDigit(0, (val >> 4) & 0x0F);
            hardware.writeDigit(1, val & 0x0F);
            return;
        }
        if (addr == ANNE_DISP_LO) {
            disp_lo = val;
            hardware.writeDigit(2, (val >> 4) & 0x0F);
            hardware.writeDigit(3, val & 0x0F);
            return;
        }
        if (addr == ANNE_DISP_DAT) {
            disp_dat = val;
            hardware.writeDigit(4, (val >> 4) & 0x0F);
            hardware.writeDigit(5, val & 0x0F);
            return;
        }
        if (addr == ANNE_ACIA_DAT) {
            Serial.write(val);
            return;
        }
    }
    // Writes to ROM areas ($C000+) are ignored
}

void ANNHal::updateHardware() {
    uint32_t buttons = hardware.readButtons(); 
    last_key_ascii = 0; // Default to no key

    if (buttons == 0) return;

    // Check for Shift Key (SW2 / 0x40)
    bool shift = (buttons & SHIFT_MASK);

    // Iterate through the KEY_MAP defined in TM1638.h
    for (uint8_t i = 0; i < MAP_SIZE; i++) {
        // Check if the specific bitmask for this key is active
        if ((buttons & KEY_MAP[i].rawSignature) == KEY_MAP[i].rawSignature) {
            
            // Avoid detecting the Shift key as a character itself if it's part of a combo
            if (KEY_MAP[i].rawSignature == SHIFT_MASK) continue;

            // Store the mapped ASCII value
            last_key_ascii = shift ? KEY_MAP[i].shifted : KEY_MAP[i].unshifted;
            return; // Return on first valid key press
        }
    }
}