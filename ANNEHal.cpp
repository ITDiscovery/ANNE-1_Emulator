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

    // Configure Tape Output ---
    pinMode(PIN_TAPE_OUT, OUTPUT);
    digitalWrite(PIN_TAPE_OUT, LOW); // Default to Low
    // (PIN_TAPE_IN / A0 does not require pinMode on ESP8266)
}

bool ANNHal::initMemory() {
    if (ram) free(ram);
    if (rom) free(rom);
    
    // 1. Allocate User RAM ($0000-$3FFF)
    ram = (uint8_t*)malloc(RAM_SIZE); 
    
    // 2. Allocate Monitor RAM Buffer ($F000-$FFFF)
    rom = (uint8_t*)malloc(ROM_SIZE); 

    if (!ram || !rom) return false;

    // 3. Point BASIC pointer to the PROGMEM array (Flash)
    // We don't need to malloc this since it's Read-Only
    basic_rom = ANNE_BASIC_ROM; 

    // 4. Clear/Init Memory
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

    // 2. BASIC ROM WINDOW ($C000 - $EFFF) [12KB]
    // We map this entire range to the basic_rom array.
    // Note: ensure your ANNE_BASIC_ROM array in "ANNE_ROM.h" is large enough 
    // or properly padded if you strictly use offsets.
    if (addr >= 0xC000 && addr < 0xF000) {
        // If your binary is 4k and you want it at D000, 
        // you might need offset math. For now, let's assume
        // the binary is compiled to fit where it is placed.
        // Safety check to prevent reading out of bounds of the array
        uint16_t offset = addr - 0xC000;
        if (offset < BASIC_SIZE) { 
             return pgm_read_byte(&basic_rom[offset]);
        }
        return 0xFF; // Empty ROM space
    }

    // 3. I/O REGION ($FFxx) 
    if (addr >= 0xFF00 && addr <= 0xFF69) {
        if (addr == ANNE_KEY_IN) {
            hardware.setLed(0, (millis() / 500) % 2);
            return last_key_ascii;
        }
        if (addr == ANNE_ACIA_STS) {
             uint8_t status = 0x02; 
             hardware.setLed(1, (millis() / 500) % 2); 
             if (Serial.available() > 0) status |= 0x01; 
             if (!keypadMode) status |= 0x80; 
             return status;
        }
        if (addr == ANNE_ACIA_DAT) {
            hardware.setLed(2, (millis() / 500) % 2); 
            if (Serial.available() > 0) {
                uint8_t c = Serial.read();
                // Map ASCII 0x7F (DEL) to 0x08 (BS) for backspace compatibility
                if (c == 0x7F) c = 0x08; 
                return c;
            }
            return 0;
        }
        // Tape Input (ADC) ---
        if (addr == ANNE_TAPE_IN) {
            // Read 10-bit value (0-1023), shift down to 8-bit (0-255)
            // Note: analogRead takes ~100us, which is slow but fine for BASIC
            return (uint8_t)(analogRead(PIN_TAPE_IN) >> 2);
        }

        // Optional: Read back the state of the Output pin
        if (addr == ANNE_TAPE_OUT) {
            return digitalRead(PIN_TAPE_OUT) ? 1 : 0;
        }
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
            #ifdef DISPLAY_FLIP
            hardware.writeDigit(2, (val >> 4) & 0x0F);
            hardware.writeDigit(3, val & 0x0F);
            #else
            hardware.writeDigit(0, (val >> 4) & 0x0F);
            hardware.writeDigit(1, val & 0x0F);
            #endif
            return;
        }
        if (addr == ANNE_DISP_LO) {
            disp_lo = val;
            #ifdef DISPLAY_FLIP
            hardware.writeDigit(4, (val >> 4) & 0x0F);
            hardware.writeDigit(5, val & 0x0F);
            #else
            hardware.writeDigit(2, (val >> 4) & 0x0F);
            hardware.writeDigit(3, val & 0x0F);
            #endif
            return;
        }
        if (addr == ANNE_DISP_DAT) {
            disp_dat = val;
            #ifdef DISPLAY_FLIP
            hardware.writeDigit(0, (val >> 4) & 0x0F);
            hardware.writeDigit(1, val & 0x0F);
            #else
            hardware.writeDigit(4, (val >> 4) & 0x0F);
            hardware.writeDigit(5, val & 0x0F);
            #endif
            return;
        }
        // 8-LED Control Register ($FF23) ---
        if (addr == ANNE_LED_REG) {
            // Loop through bits 0-7
            for (int i = 0; i < 8; i++) {
                // Check if bit 'i' is set
                uint8_t state = (val >> i) & 0x01;
                hardware.setLed(i, state); 
            }
            return;
        }
        // Tape Output (GPIO 12) ---
        if (addr == ANNE_TAPE_OUT) {
            // Write LSB to pin (0 or 1)
            digitalWrite(PIN_TAPE_OUT, val & 0x01);
            return;
        }
        if (addr == ANNE_ACIA_DAT) {
            Serial.write(val);
            return;
        }
    }
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