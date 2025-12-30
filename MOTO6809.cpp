/* MOTO6809.cpp - Full 6809 Core Implementation 
Copyright (c) 2026 Peter Nichols

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "MOTO6809.h"
#include "ANNEHal.h"
#include <stdio.h> 

// --- DEFINITIONS ---
union Reg16 {
    uint16_t w;
    struct { 
        // ESP8266 is Little Endian, so the first byte in memory is the Low byte.
        // We swap these so .h accesses the High byte of .w
        uint8_t l; 
        uint8_t h; 
    } b;
};

// 1. FIXED: Added semicolon at the end of struct
struct mc6809 {
    uint16_t pc;       
    Reg16    d;        
    uint16_t x, y, u, s; 
    uint8_t  dp;       
    uint8_t  cc;       // CC Register (EFHINZVC)

#ifdef ENABLE_HD6309
    Reg16    w;        // New Register W (E:F)
    uint16_t v;        // New Register V
    uint8_t  md;       // Mode Register
#endif

    uint16_t ir;       
    uint8_t  mode;     
    int32_t  cycles;
    bool is_waiting; // True if CWAI or SYNC has halted the CPU
    bool nmi_pending; // For ST Key (NMI)
    void* user; // This member is required for linking to HAL
};

// --- FORWARD DECLARATIONS ---
// This tells the compiler these functions exist, allowing helpers to call them.
static uint8_t fetch(mc6809__t* cpu);
static uint16_t fetch16(mc6809__t* cpu);
static uint16_t get_ea(mc6809__t* cpu);

// Addressing Modes
enum { INH, IMM, DIR, IND, EXT, REL };

// CC Flag Masks
#define CC_C 0x01
#define CC_V 0x02
#define CC_Z 0x04
#define CC_N 0x08
#define CC_I 0x10
#define CC_H 0x20
#define CC_F 0x40
#define CC_E 0x80

// --- HELPER FUNCTIONS ---
static void set_nz(mc6809__t* cpu, uint8_t r) {
    // Clear V flag too (Logic/Load ops always clear V)
    cpu->cc &= ~(CC_N | CC_Z | CC_V); 
    
    if (r == 0) cpu->cc |= CC_Z;
    if (r & 0x80) cpu->cc |= CC_N;
}

static void set_nz16(mc6809__t* cpu, uint16_t r) {
    // Clear V flag too
    cpu->cc &= ~(CC_N | CC_Z | CC_V);
    
    if (r == 0) cpu->cc |= CC_Z;
    if (r & 0x8000) cpu->cc |= CC_N;
}

// Helper for Math Flags (FIXED)
void update_flags_math(mc6809__t* c, uint16_t r, uint8_t v1, uint8_t v2, bool is_sub) {
    c->cc &= ~(CC_H | CC_N | CC_Z | CC_V | CC_C); 
    uint8_t r8 = (uint8_t)r;
    if (r8 == 0) c->cc |= CC_Z;
    if (r8 & 0x80) c->cc |= CC_N;
    
    if (is_sub) {
        // Check 16-bit result for underflow (borrow)
        if (r > 0xFF) c->cc |= CC_C; 
        
        if (((v1 ^ v2) & 0x80) && ((v1 ^ r8) & 0x80)) c->cc |= CC_V;
    } else {
        if (r & 0x100) c->cc |= CC_C; 
        if (!((v1 ^ v2) & 0x80) && ((v1 ^ r8) & 0x80)) c->cc |= CC_V;
        if ((v1 & 0x0F) + (v2 & 0x0F) > 0x0F) c->cc |= CC_H;
    }
}

// 16-BIT MATH FLAG HELPER (Must be defined before step())
static void update_flags_math16(mc6809__t* c, uint32_t r, uint16_t v1, uint16_t v2, bool is_sub) {
    c->cc &= ~(CC_N | CC_Z | CC_V | CC_C);
    uint16_t r16 = (uint16_t)r;
    
    if (r16 == 0) c->cc |= CC_Z;
    if (r16 & 0x8000) c->cc |= CC_N;
    
    if (is_sub) {
        if (v1 < v2) c->cc |= CC_C; 
        if (((v1 ^ v2) & 0x8000) && ((v1 ^ r16) & 0x8000)) c->cc |= CC_V;
    } else {
        if (r & 0x10000) c->cc |= CC_C; 
        if (!((v1 ^ v2) & 0x8000) && ((v1 ^ r16) & 0x8000)) c->cc |= CC_V;
    }
}

// Type: 0=LSL, 1=LSR, 2=ROL, 3=ROR, 4=ASR
static void do_shift_logic(mc6809__t* c, uint8_t& val, int type) {
    bool old_c = (c->cc & CC_C);
    bool new_c = false;
    
    // Perform Shift
    if(type == 0) { // LSL/ASL
        new_c = (val & 0x80); val <<= 1;
    } else if(type == 1) { // LSR
        new_c = (val & 0x01); val >>= 1;
    } else if(type == 2) { // ROL
        new_c = (val & 0x80); val <<= 1; if(old_c) val |= 1;
    } else if(type == 3) { // ROR
        new_c = (val & 0x01); val >>= 1; if(old_c) val |= 0x80;
    } else if(type == 4) { // ASR
        new_c = (val & 0x01); uint8_t sign = (val & 0x80); val >>= 1; val |= sign;
    }

    // 1. Update Carry
    if(new_c) c->cc |= CC_C; else c->cc &= ~CC_C;
    
    // 2. Update N and Z (and Clear V)
    set_nz(c, val);
    
    // Calculate V = N XOR C
    bool n = (c->cc & CC_N);
    bool curr_c = (c->cc & CC_C);
    
    if (n ^ curr_c) {
        c->cc |= CC_V; // Set V if N != C
    }
}

// Stack Helpers
void push_u(mc6809__t* cpu, uint8_t mask) {
    ANNHal* hal = (ANNHal*)cpu->user;
    if(mask & 0x80) { hal->write(--cpu->u, cpu->pc & 0xFF); hal->write(--cpu->u, cpu->pc >> 8); }
    if(mask & 0x40) { hal->write(--cpu->u, cpu->s & 0xFF); hal->write(--cpu->u, cpu->s >> 8); }
    if(mask & 0x20) { hal->write(--cpu->u, cpu->y & 0xFF); hal->write(--cpu->u, cpu->y >> 8); }
    if(mask & 0x10) { hal->write(--cpu->u, cpu->x & 0xFF); hal->write(--cpu->u, cpu->x >> 8); }
    if(mask & 0x08) { hal->write(--cpu->u, cpu->dp); }
    if(mask & 0x04) { hal->write(--cpu->u, cpu->d.b.l); }
    if(mask & 0x02) { hal->write(--cpu->u, cpu->d.b.h); }
    if(mask & 0x01) { hal->write(--cpu->u, cpu->cc); }
}

void pull_u(mc6809__t* cpu, uint8_t mask) {
    ANNHal* hal = (ANNHal*)cpu->user;
    if(mask & 0x01) { cpu->cc = hal->read(cpu->u++); }
    if(mask & 0x02) { cpu->d.b.h = hal->read(cpu->u++); }
    if(mask & 0x04) { cpu->d.b.l = hal->read(cpu->u++); }
    if(mask & 0x08) { cpu->dp = hal->read(cpu->u++); }
    if(mask & 0x10) { uint8_t h=hal->read(cpu->u++); uint8_t l=hal->read(cpu->u++); cpu->x = (h<<8)|l; }
    if(mask & 0x20) { uint8_t h=hal->read(cpu->u++); uint8_t l=hal->read(cpu->u++); cpu->y = (h<<8)|l; }
    if(mask & 0x40) { uint8_t h=hal->read(cpu->u++); uint8_t l=hal->read(cpu->u++); cpu->s = (h<<8)|l; }
    if(mask & 0x80) { uint8_t h=hal->read(cpu->u++); uint8_t l=hal->read(cpu->u++); cpu->pc = (h<<8)|l; }
}

void push_s(mc6809__t* cpu, uint8_t mask) {
    ANNHal* hal = (ANNHal*)cpu->user;
    if(mask & 0x80) { hal->write(--cpu->s, cpu->pc & 0xFF); hal->write(--cpu->s, cpu->pc >> 8); }
    if(mask & 0x40) { hal->write(--cpu->s, cpu->u & 0xFF); hal->write(--cpu->s, cpu->u >> 8); }
    if(mask & 0x20) { hal->write(--cpu->s, cpu->y & 0xFF); hal->write(--cpu->s, cpu->y >> 8); }
    if(mask & 0x10) { hal->write(--cpu->s, cpu->x & 0xFF); hal->write(--cpu->s, cpu->x >> 8); }
    if(mask & 0x08) { hal->write(--cpu->s, cpu->dp); }
    if(mask & 0x04) { hal->write(--cpu->s, cpu->d.b.l); }
    if(mask & 0x02) { hal->write(--cpu->s, cpu->d.b.h); }
    if(mask & 0x01) { hal->write(--cpu->s, cpu->cc); }
}

void pull_s(mc6809__t* cpu, uint8_t mask) {
    ANNHal* hal = (ANNHal*)cpu->user;
    if(mask & 0x01) { cpu->cc = hal->read(cpu->s++); }
    if(mask & 0x02) { cpu->d.b.h = hal->read(cpu->s++); }
    if(mask & 0x04) { cpu->d.b.l = hal->read(cpu->s++); }
    if(mask & 0x08) { cpu->dp = hal->read(cpu->s++); }
    if(mask & 0x10) { uint8_t h=hal->read(cpu->s++); uint8_t l=hal->read(cpu->s++); cpu->x = (h<<8)|l; }
    if(mask & 0x20) { uint8_t h=hal->read(cpu->s++); uint8_t l=hal->read(cpu->s++); cpu->y = (h<<8)|l; }
    if(mask & 0x40) { uint8_t h=hal->read(cpu->s++); uint8_t l=hal->read(cpu->s++); cpu->u = (h<<8)|l; }
    if(mask & 0x80) { uint8_t h=hal->read(cpu->s++); uint8_t l=hal->read(cpu->s++); cpu->pc = (h<<8)|l; }
}

// --- BRANCH HELPER ---
static void do_branch(mc6809__t* c, bool condition) {
    int8_t offset = (int8_t)fetch(c); // ALWAYS fetch to advance PC
    if (condition) {
        c->pc += offset;
    }
    c->cycles += 3; // 6809 always takes 3 cycles for short branches
}

// Handles 16-bit offset "Long" branches
static void do_lbranch(mc6809__t* c, bool condition) {
    int16_t offset = fetch16(c);
    if (condition) {
        c->pc += offset;
        c->cycles += 1; // 16-bit branches are slower (5-6 cycles usually)
    }
}

// --- CLASS LIFECYCLE ---
MOTO6809::MOTO6809(ANNHal* halInstance) : hal(halInstance) {
    cpu_core = new mc6809__t;
    cpu_core->user = hal;
    Reset();
}
MOTO6809::~MOTO6809() { delete cpu_core; }

void MOTO6809::Reset() {
    uint8_t h = hal->read(0xFFFE);
    uint8_t l = hal->read(0xFFFF);
    cpu_core->pc = (h << 8) | l;
    cpu_core->dp = 0x00;
    cpu_core->cc = CC_I | CC_F; 
    cpu_core->cycles = 0;
    cpu_core->is_waiting = false;
    cpu_core->nmi_pending = false;
#ifdef ENABLE_HD6309
    cpu_core->w.w = 0;
    cpu_core->v = 0;
    cpu_core->md = 0;
#endif
}

// --- PUBLIC ACCESSORS ---
uint16_t MOTO6809::get_pc_register() { return cpu_core->pc; }
uint8_t MOTO6809::get_A() { return cpu_core->d.b.h; }
uint8_t MOTO6809::get_B() { return cpu_core->d.b.l; }
uint8_t MOTO6809::read8(uint16_t addr) const { return hal->read(addr); }
void MOTO6809::write8(uint16_t addr, uint8_t val) const { hal->write(addr, val); }

void MOTO6809::Run(int32_t cycles) {
    int32_t target = cpu_core->cycles + cycles;
    static int traceCountdown = 0;

    while (cpu_core->cycles < target) {
        hal->system_check(this);

        // --- TRIGGER TRACE ---
        // Trigger slightly before the crash site
        //if (cpu_core->pc == 0x2000 && traceCountdown == 0) {
        //    Serial.println("\n--- TRACE START (PCR TESTS) ---");
        //    traceCountdown = 250; // Dump the next 20 instructions
        //}
        // --- STACK RUNAWAY TRAP ---
        // If PC points into the stack area (RAM_TOP), something has gone wrong.
        //if (cpu_core->pc >= 0x3F00 && cpu_core->pc <= 0x3FFF) {
        //     if (traceCountdown == 0) { // Only print if not already tracing
        //         Serial.print("\n[CRASH] PC in STACK at ");
        //         Serial.println(cpu_core->pc, HEX);
        //         traceCountdown = 10;
        //     }
        //}
        if (hal->debug_dump == true) {
            uint8_t op = hal->read(cpu_core->pc);
            
            Serial.print("[");
            Serial.print(traceCountdown);
            Serial.print("] PC:"); 
            Serial.print(cpu_core->pc, HEX);
            Serial.print(" OP:"); 
            Serial.print(op, HEX);
            
            // Register Dump
            Serial.print(" A:"); Serial.print(cpu_core->d.b.h, HEX);
            Serial.print(" B:"); Serial.print(cpu_core->d.b.l, HEX);
            Serial.print(" X:"); Serial.print(cpu_core->x, HEX);
            Serial.print(" Y:"); Serial.print(cpu_core->y, HEX);
            Serial.print(" U:"); Serial.print(cpu_core->u, HEX);
            Serial.print(" S:"); Serial.print(cpu_core->s, HEX);
            Serial.print(" CC:"); Serial.println(cpu_core->cc, HEX);
        }
        step();
    }
}

void MOTO6809::setPC(uint16_t val) {
    cpu_core->pc = val;
}

void MOTO6809::cancelWait() {
    cpu_core->is_waiting = false;
}

void MOTO6809::triggerNMI() {
    if (cpu_core) {
        cpu_core->nmi_pending = true;
        cpu_core->is_waiting = false; // Wake up if sleeping
    }
}

// --- FETCH & DECODE UTILITIES ---
uint8_t fetch(mc6809__t* cpu) {
    ANNHal* hal = (ANNHal*)cpu->user;
    return hal->read(cpu->pc++);
}

uint16_t fetch16(mc6809__t* cpu) {
    uint8_t h = fetch(cpu);
    uint8_t l = fetch(cpu);
    return (h << 8) | l;
}

uint16_t resolve_indexed(mc6809__t* cpu) {
    uint8_t post = fetch(cpu);
    uint16_t addr = 0;
    uint16_t reg_val = 0;
    
    // 1. Select Register
    switch((post >> 5) & 0x03) {
        case 0: reg_val = cpu->x; break;
        case 1: reg_val = cpu->y; break;
        case 2: reg_val = cpu->u; break;
        case 3: reg_val = cpu->s; break;
    }

    // 2. Determine Mode & Offset
    if ((post & 0x80) == 0) { 
        // 5-bit Offset (Direct only)
        int8_t offset = (post & 0x1F);
        if (offset & 0x10) offset |= 0xE0; // Sign extend
        addr = reg_val + offset;
        cpu->cycles += 1;
    } else {
        // Complex Modes
        // FIX: Added Indirect cases (0x1X) to match Direct cases (0x0X)
        switch(post & 0x1F) {
            // Auto Increment/Decrement
            case 0x00: addr = reg_val++; break;              // ,R+
            case 0x01: case 0x11: addr = reg_val; reg_val += 2; break; // ,R++ and [,R++]
            case 0x02: addr = --reg_val; break;              // ,-R
            case 0x03: case 0x13: reg_val -= 2; addr = reg_val; break; // ,--R and [,--R]
            
            // Zero Offset
            case 0x04: case 0x14: addr = reg_val; break;     // ,R and [,R]
            
            // Accumulator Offsets
            case 0x05: case 0x15: addr = reg_val + (int8_t)cpu->d.b.l; break; // B,R
            case 0x06: case 0x16: addr = reg_val + (int8_t)cpu->d.b.h; break; // A,R
            case 0x0B: case 0x1B: addr = reg_val + cpu->d.w; break;           // D,R
            
            // Constant Offsets (8-bit / 16-bit)
            case 0x08: case 0x18: addr = reg_val + (int8_t)fetch(cpu); break;  // 8-bit
            case 0x09: case 0x19: addr = reg_val + (int16_t)fetch16(cpu); break; // 16-bit
            
            // PC Relative (8-bit)
            case 0x0C: case 0x1C: {
                int8_t offset = (int8_t)fetch(cpu); // 1. Advance PC
                addr = cpu->pc + offset;            // 2. Add to updated PC
                break;  
            }

            // PC Relative (16-bit)
            case 0x0D: case 0x1D: {
                int16_t offset = (int16_t)fetch16(cpu); // 1. Force Fetch (PC advances by 2)
                addr = cpu->pc + offset;                // 2. Add to the updated PC
                break;
            }
            
            // Extended Indirect [nnnn]
            case 0x1F: addr = fetch16(cpu); break; 
            
            default: addr = reg_val; break;
        }
        
        // Write-back Register Update
        // (Only matters for Auto Inc/Dec which modified reg_val directly)
        switch((post >> 5) & 0x03) {
            case 0: cpu->x = reg_val; break;
            case 1: cpu->y = reg_val; break;
            case 2: cpu->u = reg_val; break;
            case 3: cpu->s = reg_val; break;
        }
    }
    
    // 3. Handle Indirection (Bit 4 set)
    // This performs the final memory lookup if the mode was Indirect
    // Note: 5-bit offsets (post & 0x80 == 0) are never indirect.
    if ((post & 0x10) && (post & 0x80)) {
        ANNHal* hal = (ANNHal*)cpu->user;
        uint8_t h = hal->read(addr);
        uint8_t l = hal->read(addr+1);
        addr = (h << 8) | l;
        cpu->cycles += 3;
    }
    return addr;
}

uint16_t get_ea(mc6809__t* cpu) {
    ANNHal* hal = (ANNHal*)cpu->user;
    switch(cpu->mode) {
        case IMM: return cpu->pc++; 
        case DIR: return ((uint16_t)cpu->dp << 8) | fetch(cpu);
        case EXT: return fetch16(cpu);
        case IND: return resolve_indexed(cpu);
        default: return 0;
    }
}

// --- EXECUTION ENGINE ---
int MOTO6809::step() {
    mc6809__t* c = cpu_core;
    ANNHal* h = (ANNHal*)c->user;

    // --- FLIGHT RECORDER START ---
    // Record state BEFORE execution
    history[historyIdx].pc = c->pc;
    history[historyIdx].opcode = h->read(c->pc);
    history[historyIdx].accA = c->d.b.h;
    history[historyIdx].accB = c->d.b.l;
    history[historyIdx].regX = c->x;
    history[historyIdx].regS = c->s;
    historyIdx = (historyIdx + 1) % 16;
    // -----------------------------

    // --- 1. HANDLE NMI (ST KEY) ---
    if (c->nmi_pending) {
        c->nmi_pending = false; // Clear the flag
        
        // 6809 NMI Sequence:
        // 1. Set 'E' flag (Entire state saved)
        c->cc |= CC_E;   
        
        // 2. Push all registers: PC, U, Y, X, DP, A, B, CC
        push_s(c, 0xFF); 
        
        // 3. Set 'F' and 'I' bits to mask other interrupts
        c->cc |= (CC_F | CC_I); 
        
        // 4. Load Vector from $FFFC/$FFFD
        uint8_t vh = h->read(0xFFFC);
        uint8_t vl = h->read(0xFFFD);
        c->pc = (vh << 8) | vl;
        
        c->cycles += 19; // NMI penalty
        return 0;        // Start executing NMI handler next cycle
    }

    // 2. HANDLE WAIT (CWAI/SYNC), spin without fetching
    if (c->is_waiting) {
        c->cycles += 1; // Consume cycles while waiting
        return 0;       // Return early, do not fetch
    }

    c->ir = fetch(c);
    
    if (c->ir == 0x10 || c->ir == 0x11) {
        c->ir = (c->ir << 8) | fetch(c);
    }
    
    // --- DECODE ADDRESSING MODE ---
    c->mode = INH;
    uint8_t op = c->ir & 0xFF;
    uint8_t row = op & 0xF0;

    // 1. General ALU/Load/Store Groups
    if (row == 0x80 || row == 0xC0) c->mode = IMM;
    if (row == 0x90 || row == 0xD0) c->mode = DIR;
    if (row == 0xA0 || row == 0xE0) c->mode = IND;
    if (row == 0xB0 || row == 0xF0) c->mode = EXT;
    
    // 2. Single Operand Groups (NEG, COM, LSR, ROR, ASR, ASL, ROL, DEC, INC, TST, JMP, CLR)
    if (row == 0x00) c->mode = DIR;
    if (row == 0x60) c->mode = IND;
    if (row == 0x70) c->mode = EXT;

    // 3. Explicit Overrides
    if (op == 0x8D) c->mode = REL;
    // LDD/LDS/LDU/LDX/LDY Immediate modes (16-bit data)
    if (c->ir == 0xCC || c->ir == 0xCE || c->ir == 0x10CE || c->ir == 0x10CC) c->mode = IMM;
    if (c->ir == 0x8E || c->ir == 0x108E) c->mode = IMM;

    uint16_t ea = 0;
    uint16_t v16 = 0;
    uint8_t val = 0;

    switch(c->ir) {
        case 0x12: c->cycles+=2; break; // NOP

        // --- LOADS ---
        case 0x86: case 0x96: case 0xA6: case 0xB6: // LDA
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            c->d.b.h = h->read(ea);
            set_nz(c, c->d.b.h); c->cycles+=2; break;
        case 0xC6: case 0xD6: case 0xE6: case 0xF6: // LDB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            c->d.b.l = h->read(ea);
            set_nz(c, c->d.b.l); c->cycles+=2; break;

        // LDD (Load D) - CC (Imm), DC (Dir), EC (Ind), FC (Ext)
        case 0xCC: case 0xDC: case 0xEC: case 0xFC: 
            if (c->mode == IMM) {
                c->d.w = fetch16(c); 
            } else { 
                ea = get_ea(c); 
                c->d.w = (h->read(ea) << 8) | h->read(ea+1); 
            }
            set_nz16(c, c->d.w); 
            c->cycles += 3; 
            break;

        // --- 16-BIT LOADS (LDX, LDY) ---
        // LDX (Load X) - 8E, 9E, AE, BE
        case 0x8E: case 0x9E: case 0xAE: case 0xBE: 
            // FIX: Explicitly check for Opcode 0x8E (Immediate) to prevent mode detection errors
            if (c->mode == IMM || c->ir == 0x8E) {
                c->x = fetch16(c); 
            } else { 
                ea = get_ea(c); 
                c->x = (h->read(ea) << 8) | h->read(ea+1); 
            }
            set_nz16(c, c->x); 
            c->cycles += 3; 
            break;

        // LDY (Load Y) - 108E, 109E, 10AE, 10BE
        case 0x108E: case 0x109E: case 0x10AE: case 0x10BE:
            if (c->mode == IMM || c->ir == 0x108E) { // <--- Added Explicit Check
                c->y = fetch16(c); 
            } else { 
                ea = get_ea(c); 
                c->y = (h->read(ea) << 8) | h->read(ea+1); 
            }
            set_nz16(c, c->y); 
            c->cycles += 4; 
            break;

        // LDS (Load S) - 10CE (Imm), 10DE (Dir), 10EE (Ind), 10FE (Ext)
        case 0x10CE: case 0x10DE: case 0x10EE: case 0x10FE:
            if (c->mode == IMM) c->s = fetch16(c); 
            else { ea = get_ea(c); c->s = (h->read(ea) << 8) | h->read(ea+1); }
            set_nz16(c, c->s); 
            c->cycles += 4; 
            break;

        // LDU (Load U) - CE (Imm), DE (Dir), EE (Ind), FE (Ext)
        case 0xCE: case 0xDE: case 0xEE: case 0xFE:
            if (c->mode == IMM) c->u = fetch16(c); 
            else { ea = get_ea(c); c->u = (h->read(ea) << 8) | h->read(ea+1); }
            set_nz16(c, c->u); 
            c->cycles += 3; 
            break;
            
        // --- STORES ---
        case 0x97: case 0xA7: case 0xB7: // STA
            ea = get_ea(c); h->write(ea, c->d.b.h); 
            set_nz(c, c->d.b.h); c->cycles+=4; break;
        case 0xD7: case 0xE7: case 0xF7: // STB
            ea = get_ea(c); h->write(ea, c->d.b.l); 
            set_nz(c, c->d.b.l); c->cycles+=4; break;

        // --- 16-BIT STORES (STX, STY, STS, STU) ---
        // STX (Store X) - 9F, AF, BF
        case 0x9F: case 0xAF: case 0xBF:
            ea = get_ea(c); 
            h->write(ea, c->x >> 8); h->write(ea+1, c->x & 0xFF);
            set_nz16(c, c->x); c->cycles+=5; 
            break;

        // STY (Store Y) - 109F, 10AF, 10BF
        case 0x109F: case 0x10AF: case 0x10BF:
            ea = get_ea(c); 
            h->write(ea, c->y >> 8); h->write(ea+1, c->y & 0xFF);
            set_nz16(c, c->y); c->cycles+=6; 
            break;

        // STS (Store S) - 10DF, 10EF, 10FF
        case 0x10DF: case 0x10EF: case 0x10FF:
            ea = get_ea(c); 
            h->write(ea, c->s >> 8); h->write(ea+1, c->s & 0xFF);
            set_nz16(c, c->s); c->cycles+=6; 
            break;

        // STU (Store U) - DF, EF, FF
        case 0xDF: case 0xEF: case 0xFF:
            ea = get_ea(c); 
            h->write(ea, c->u >> 8); h->write(ea+1, c->u & 0xFF);
            set_nz16(c, c->u); c->cycles+=5; 
            break;
            
        // STD (Store D) - DD, ED, FD
        case 0xDD: case 0xED: case 0xFD:
            ea = get_ea(c); 
            h->write(ea, c->d.b.h); h->write(ea+1, c->d.b.l);
            set_nz16(c, c->d.w); c->cycles+=5; 
            break;

        // --- SINGLE OPERAND MEMORY & ACCUMULATOR OPS ---
        // CLR (Clear) - 0F, 6F, 7F (Memory)
        case 0x0F: case 0x6F: case 0x7F:
            ea = get_ea(c); h->write(ea, 0); 
            // CLR: Z=1, N=0, V=0, C=0
            c->cc &= ~(CC_N | CC_V | CC_C); c->cc |= CC_Z;
            c->cycles += 6; 
            break;
        // CLRB - 5F (CLRA 4F was already present)
        case 0x5F: 
            c->d.b.l=0; 
            c->cc &= ~(CC_N | CC_V | CC_C); c->cc |= CC_Z; 
            c->cycles+=2; break; 

        // TST (Test) - 0D, 6D, 7D (Mem), 4D (A), 5D (B)
        // TST: Set N,Z. Clear V. C not affected.
        case 0x4D: // TSTA
            set_nz(c, c->d.b.h); c->cc &= ~CC_V; c->cycles+=2; break; 
        case 0x5D: // TSTB
            set_nz(c, c->d.b.l); c->cc &= ~CC_V; c->cycles+=2; break; 
        case 0x0D: case 0x6D: case 0x7D: // TST Mem
            ea = get_ea(c); val = h->read(ea); 
            set_nz(c, val); c->cc &= ~CC_V; c->cycles+=6; break;

        // INC (Increment) - 0C, 6C, 7C (Mem), 4C (A), 5C (B)
        // INC: Set N,Z. Set V if result is $80. C not affected.
        case 0x4C: // INCA (Override existing simple INCA to add V flag handling)
            c->d.b.h++; set_nz(c, c->d.b.h); 
            if(c->d.b.h == 0x80) c->cc |= CC_V; else c->cc &= ~CC_V;
            c->cycles+=2; break;
        case 0x5C: // INCB
            c->d.b.l++; set_nz(c, c->d.b.l); 
            if(c->d.b.l == 0x80) c->cc |= CC_V; else c->cc &= ~CC_V;
            c->cycles+=2; break;
        case 0x0C: case 0x6C: case 0x7C: // INC Mem
            ea = get_ea(c); val = h->read(ea) + 1; h->write(ea, val);
            set_nz(c, val); 
            if(val == 0x80) c->cc |= CC_V; else c->cc &= ~CC_V;
            c->cycles+=6; break;
        
        // ABX (Add B to X) - Unsigned add, no flags affected
        case 0x3A: 
            c->x += c->d.b.l; 
            c->cycles += 3; 
            break;

        // DAA (Decimal Adjust Accumulator)
        case 0x19: {
            uint8_t old_A = c->d.b.h;
            uint8_t cf = 0;
            
            // 1. Lower Nibble Correction
            // Triggered if low nibble > 9 OR Half-Carry (H) is set
            if ((old_A & 0x0F) > 0x09 || (c->cc & CC_H)) {
                cf |= 0x06;
            }
            
            // 2. Upper Nibble Correction
            // Triggered if:
            // a) A > 0x99 
            // b) Carry (C) is set
            // c) Upper nibble > 8 AND Low nibble > 9 
            //    (CRITICAL FIX: Do NOT use 'cf==6' here, as H=1 shouldn't trigger this)
            if (old_A > 0x99 || (c->cc & CC_C) || 
               ((old_A > 0x8F) && ((old_A & 0x0F) > 0x09))) {
                cf |= 0x60;
                c->cc |= CC_C; // DAA sets Carry if upper correction is needed
            }
            
            c->d.b.h = old_A + cf;
            
            // Updates N and Z. V is undefined (we leave it alone or use set_nz)
            set_nz(c, c->d.b.h); 
            c->cycles += 2;
        } break;

        // DEC (Decrement) - 0A, 6A, 7A (Mem), 4A (A), 5A (B)
        // DEC: Set N,Z. Set V if result is $7F. C not affected.
        case 0x4A: // DECA (Override existing)
            c->d.b.h--; set_nz(c, c->d.b.h); 
            if(c->d.b.h == 0x7F) c->cc |= CC_V; else c->cc &= ~CC_V;
            c->cycles+=2; break;
        case 0x5A: // DECB
            c->d.b.l--; set_nz(c, c->d.b.l); 
            if(c->d.b.l == 0x7F) c->cc |= CC_V; else c->cc &= ~CC_V;
            c->cycles+=2; break;
        case 0x0A: case 0x6A: case 0x7A: // DEC Mem
            ea = get_ea(c); val = h->read(ea) - 1; h->write(ea, val);
            set_nz(c, val); 
            if(val == 0x7F) c->cc |= CC_V; else c->cc &= ~CC_V;
            c->cycles+=6; break;

        // --- NEGATE (NEG) - 00, 40, 50, etc ---
        case 0x40: { // NEGA
            uint8_t val = c->d.b.h;
            c->d.b.h = -val;
            set_nz(c, c->d.b.h);
            // NEG Logic: C=1 if val!=0, V=1 if val=$80
            if (val != 0) c->cc |= CC_C; else c->cc &= ~CC_C;
            if (val == 0x80) c->cc |= CC_V;
            c->cycles+=2; 
        } break; 

        case 0x50: { // NEGB
            uint8_t val = c->d.b.l;
            c->d.b.l = -val;
            set_nz(c, c->d.b.l);
            if (val != 0) c->cc |= CC_C; else c->cc &= ~CC_C;
            if (val == 0x80) c->cc |= CC_V;
            c->cycles+=2; 
        } break;

        case 0x00: case 0x60: case 0x70: { // NEG (Memory)
            ea = get_ea(c); 
            uint8_t old_val = h->read(ea);
            val = -old_val; 
            h->write(ea, val); 
            set_nz(c, val); 
            if (old_val != 0) c->cc |= CC_C; else c->cc &= ~CC_C;
            if (old_val == 0x80) c->cc |= CC_V;
            c->cycles+=6; 
        } break;

        // --- COMPARES ---
        case 0x81: case 0x91: case 0xA1: case 0xB1: // CMPA
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            do_cmp8(c, c->d.b.h, val); // Calculates N, Z, V, C correctly
            c->cycles+=2; break;

         case 0xC1: case 0xD1: case 0xE1: case 0xF1: // CMPB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            do_cmp8(c, c->d.b.l, val); // Calculates N, Z, V, C correctly
            c->cycles+=2; break;

        // --- BRANCHES (8-bit Offset) ---

        // BNE (Branch if Not Equal / Not Zero) - 26
        // Logic: Branch if Z-Flag is 0
        case 0x26: 
            {
                int8_t rel = (int8_t)fetch(c); // Fetch Offset
                if ((c->cc & CC_Z) == 0) {     // Explicit Zero Check
                    c->pc += rel;
                }
                c->cycles += 3;
            }
            break;

        // BEQ (Branch if Equal / Zero) - 27
        // Logic: Branch if Z-Flag is 1
        case 0x27: 
            {
                int8_t rel = (int8_t)fetch(c); // Fetch Offset
                if (c->cc & CC_Z) {            // Explicit Set Check
                    c->pc += rel;
                }
                c->cycles += 3;
            }
            break;

        case 0x20: do_branch(c, true); break; // BRA
        case 0x21: do_branch(c, false); break; // BRN (Never)
        case 0x22: do_branch(c, !((c->cc & CC_C) || (c->cc & CC_Z))); break; // BHI (C+Z=0)
        case 0x23: do_branch(c, (c->cc & CC_C) || (c->cc & CC_Z)); break; // BLS (C+Z=1)
        case 0x24: do_branch(c, !(c->cc & CC_C)); break; // BCC (C=0)
        case 0x25: do_branch(c, (c->cc & CC_C)); break;  // BCS (C=1)
        case 0x28: do_branch(c, !(c->cc & CC_V)); break; // BVC (V=0)
        case 0x29: do_branch(c, (c->cc & CC_V)); break;  // BVS (V=1)
        case 0x2A: do_branch(c, !(c->cc & CC_N)); break; // BPL (N=0)
        case 0x2B: do_branch(c, (c->cc & CC_N)); break;  // BMI (N=1)
        
        // Signed Branches (The tricky ones)
        // BGE (Greater or Equal): (N XOR V) == 0
        case 0x2D: do_branch(c, ((c->cc & CC_N) ? 1 : 0) ^ ((c->cc & CC_V) ? 1 : 0)); break;
        // BGE (Opcode 0x2C)
        case 0x2C: do_branch(c, !(((c->cc & CC_N) ? 1 : 0) ^ ((c->cc & CC_V) ? 1 : 0))); break;
        // BGT (Greater Than): Z=0 AND (N XOR V)=0
        case 0x2E: do_branch(c, !(c->cc & CC_Z) && !((c->cc & CC_N) ? 1 : 0 ^ (c->cc & CC_V) ? 1 : 0)); break;
        // BLE (Less or Equal): Z=1 OR (N XOR V)=1
        case 0x2F: do_branch(c, (c->cc & CC_Z) || ((c->cc & CC_N) ? 1 : 0 ^ (c->cc & CC_V) ? 1 : 0)); break;

        // --- LONG BRANCHES (16-bit Offset) ---
        case 0x16: do_lbranch(c, true); c->cycles+=1; break; // LBRA
        case 0x1021: do_lbranch(c, false); c->cycles+=5; break; // LBRN
        case 0x1022: do_lbranch(c, !((c->cc & CC_C) || (c->cc & CC_Z))); c->cycles+=5; break; // LBHI
        case 0x1023: do_lbranch(c, (c->cc & CC_C) || (c->cc & CC_Z)); c->cycles+=5; break; // LBLS
        case 0x1024: do_lbranch(c, !(c->cc & CC_C)); c->cycles+=5; break; // LBCC
        case 0x1025: do_lbranch(c, (c->cc & CC_C)); c->cycles+=5; break; // LBCS
        case 0x1026: do_lbranch(c, !(c->cc & CC_Z)); c->cycles+=5; break; // LBNE
        case 0x1027: do_lbranch(c, (c->cc & CC_Z)); c->cycles+=5; break; // LBEQ
        case 0x1028: do_lbranch(c, !(c->cc & CC_V)); c->cycles+=5; break; // LBVC
        case 0x1029: do_lbranch(c, (c->cc & CC_V)); c->cycles+=5; break; // LBVS
        case 0x102A: do_lbranch(c, !(c->cc & CC_N)); c->cycles+=5; break; // LBPL
        case 0x102B: do_lbranch(c, (c->cc & CC_N)); c->cycles+=5; break; // LBMI
        // Long Branches (LBLT 0x102D / LBGE 0x102C)
        case 0x102D: do_lbranch(c, ((c->cc & CC_N) ? 1 : 0) ^ ((c->cc & CC_V) ? 1 : 0)); break;
        case 0x102C: do_lbranch(c, !(((c->cc & CC_N) ? 1 : 0) ^ ((c->cc & CC_V) ? 1 : 0))); break;
        case 0x102E: do_lbranch(c, !(c->cc & CC_Z) && !((c->cc & CC_N) ? 1 : 0 ^ (c->cc & CC_V) ? 1 : 0)); c->cycles+=5; break; // LBGT
        case 0x102F: do_lbranch(c, (c->cc & CC_Z) || ((c->cc & CC_N) ? 1 : 0 ^ (c->cc & CC_V) ? 1 : 0)); c->cycles+=5; break; // LBLE

        // LBSR (Long Branch to Subroutine)
        case 0x17: 
            ea = fetch16(c); // Offset
            h->write(--c->s, c->pc & 0xFF); 
            h->write(--c->s, c->pc >> 8);
            c->pc += (int16_t)ea; 
            c->cycles += 9; 
            break;

        // --- ALU ---
        case 0x4F: // CLRA
            c->d.b.h = 0; 
            // CLR must set Z=1 and clear N, V, C
            c->cc &= ~(CC_N | CC_V | CC_C); 
            c->cc |= CC_Z;
            c->cycles += 2; 
            break;
        
        // --- LOGIC OPERATIONS ---
        case 0x84: case 0x94: case 0xA4: case 0xB4: // ANDA
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            c->d.b.h &= h->read(ea); set_nz(c, c->d.b.h); c->cycles+=2; break;
        case 0xC4: case 0xD4: case 0xE4: case 0xF4: // ANDB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            c->d.b.l &= h->read(ea); set_nz(c, c->d.b.l); c->cycles+=2; break;
            
        case 0x8A: case 0x9A: case 0xAA: case 0xBA: // ORA
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            c->d.b.h |= h->read(ea); set_nz(c, c->d.b.h); c->cycles+=2; break;
        case 0xCA: case 0xDA: case 0xEA: case 0xFA: // ORB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            c->d.b.l |= h->read(ea); set_nz(c, c->d.b.l); c->cycles+=2; break;

        case 0x88: case 0x98: case 0xA8: case 0xB8: // EORA 
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            c->d.b.h ^= h->read(ea); set_nz(c, c->d.b.h); c->cycles+=2; break;
        case 0xC8: case 0xD8: case 0xE8: case 0xF8: // EORB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            c->d.b.l ^= h->read(ea); set_nz(c, c->d.b.l); c->cycles+=2; break;

        case 0x85: case 0x95: case 0xA5: case 0xB5: // BITA
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            set_nz(c, c->d.b.h & h->read(ea)); c->cycles+=2; break;
        case 0xC5: case 0xD5: case 0xE5: case 0xF5: // BITB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            set_nz(c, c->d.b.l & h->read(ea)); c->cycles+=2; break;

        // --- CONDITION CODE MANIPULATION ---
        case 0x1C: // ANDCC #Immediate
            val = fetch(c);
            c->cc &= val; // Clear flags (e.g. ANDCC #$FB clears Z)
            c->cycles += 3;
            break;

        case 0x1A: // ORCC #Immediate
            val = fetch(c);
            c->cc |= val; // Set flags
            c->cycles += 3;
            break;

        // --- SHIFTS & ROTATES ---
        case 0x48: do_shift_logic(c, c->d.b.h, 0); c->cycles+=2; break; // LSLA
        case 0x58: do_shift_logic(c, c->d.b.l, 0); c->cycles+=2; break; // LSLB
        case 0x08: case 0x68: case 0x78: { 
             ea = get_ea(c); val = h->read(ea); do_shift_logic(c, val, 0); h->write(ea, val); c->cycles+=6; break; 
        }
        
        case 0x44: do_shift_logic(c, c->d.b.h, 1); c->cycles+=2; break; // LSRA
        case 0x54: do_shift_logic(c, c->d.b.l, 1); c->cycles+=2; break; // LSRB
        case 0x04: case 0x64: case 0x74: { 
             ea = get_ea(c); val = h->read(ea); do_shift_logic(c, val, 1); h->write(ea, val); c->cycles+=6; break;
        }

        case 0x49: do_shift_logic(c, c->d.b.h, 2); c->cycles+=2; break; // ROLA
        case 0x59: do_shift_logic(c, c->d.b.l, 2); c->cycles+=2; break; // ROLB
        case 0x09: case 0x69: case 0x79: { 
             ea = get_ea(c); val = h->read(ea); do_shift_logic(c, val, 2); h->write(ea, val); c->cycles+=6; break;
        }

        case 0x46: do_shift_logic(c, c->d.b.h, 3); c->cycles+=2; break; // RORA
        case 0x56: do_shift_logic(c, c->d.b.l, 3); c->cycles+=2; break; // RORB
        case 0x06: case 0x66: case 0x76: { 
             ea = get_ea(c); val = h->read(ea); do_shift_logic(c, val, 3); h->write(ea, val); c->cycles+=6; break;
        }

        // --- JUMPS & SUBROUTINES ---
        case 0x0E: case 0x6E: case 0x7E: // JMP 
            ea = get_ea(c); c->pc = ea; break;
            
        case 0x9D: case 0xAD: case 0xBD: // JSR 
            ea = get_ea(c); 
            h->write(--c->s, c->pc & 0xFF); 
            h->write(--c->s, c->pc >> 8); 
            c->pc = ea; c->cycles += 8; break;

        case 0x39: // RTS
            {
                uint8_t pch = h->read(c->s++);
                uint8_t pcl = h->read(c->s++);
                c->pc = (pch << 8) | pcl;
            }
            c->cycles += 5; break;

        // --- ARITHMETIC SHIFT RIGHT (ASR) ---
        case 0x47: do_shift_logic(c, c->d.b.h, 4); c->cycles+=2; break; // ASRA
        case 0x57: do_shift_logic(c, c->d.b.l, 4); c->cycles+=2; break; // ASRB
        case 0x07: case 0x67: case 0x77: { // ASR Memory
             ea = get_ea(c); val = h->read(ea); 
             do_shift_logic(c, val, 4); 
             h->write(ea, val); c->cycles+=6; 
        } break;

        case 0x8D: // BSR 
            ea = fetch(c); 
            h->write(--c->s, c->pc & 0xFF); 
            h->write(--c->s, c->pc >> 8);
            c->pc += (int8_t)ea; c->cycles += 7; break;

        // --- STACK OPS ---
        case 0x34: // PSHS
            val = fetch(c); push_s(c, val); c->cycles += 5; break;
        case 0x35: // PULS
            val = fetch(c); pull_s(c, val); c->cycles += 5; break;
        case 0x36: // PSHU
            val = fetch(c); push_u(c, val); c->cycles += 5; break;
        case 0x37: // PULU
            val = fetch(c); pull_u(c, val); c->cycles += 5; break;

        // --- ARITHMETIC ---
        case 0x8B: case 0x9B: case 0xAB: case 0xBB: // ADDA
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            v16 = c->d.b.h + val; 
            update_flags_math(c, v16, c->d.b.h, val, false);
            c->d.b.h = (uint8_t)v16; c->cycles += 2; break;

        case 0xCB: case 0xDB: case 0xEB: case 0xFB: // ADDB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            v16 = c->d.b.l + val; 
            update_flags_math(c, v16, c->d.b.l, val, false);
            c->d.b.l = (uint8_t)v16; c->cycles += 2; break;

        // SUBA (Subtract Memory from A) - 80, 90, A0, B0
        case 0x80: case 0x90: case 0xA0: case 0xB0:
            // 1. Fetch Operand
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            
            // 2. Perform Math (Inline to guarantee flags)
            {
                uint16_t r = c->d.b.h; // Register A
                uint16_t m = val;      // Memory
                uint16_t res = r - m;  // 16-bit result to catch borrows
                
                c->cc &= ~(CC_N | CC_Z | CC_V | CC_C); // Clear flags
                
                // N: Result is negative (Bit 7 of 8-bit result)
                if (res & 0x80) c->cc |= CC_N;
                
                // Z: Result is zero (Low 8 bits only!)
                if ((res & 0xFF) == 0) c->cc |= CC_Z;
                
                // V: Overflow (Standard subtraction formula)
                // (Reg ^ Mem) & (Reg ^ Result) & 0x80
                if (((r ^ m) & 0x80) && ((r ^ res) & 0x80)) c->cc |= CC_V;

                // C: Borrow (Unsigned comparison)
                if (r < m) c->cc |= CC_C;
                
                // 3. Store Result
                c->d.b.h = (uint8_t)res;
            }
            c->cycles += 2; 
            break;

        case 0xC0: case 0xD0: case 0xE0: case 0xF0: // SUBB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            v16 = c->d.b.l - val; 
            update_flags_math(c, v16, c->d.b.l, val, true);
            c->d.b.l = (uint8_t)v16; c->cycles += 2; break;

        // --- ADD WITH CARRY (ADCA, ADCB) ---
        case 0x89: case 0x99: case 0xA9: case 0xB9: // ADCA
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            v16 = c->d.b.h + val + (c->cc & CC_C ? 1 : 0);
            update_flags_math(c, v16, c->d.b.h, val, false);
            c->d.b.h = (uint8_t)v16; c->cycles += 2; break;

        case 0xC9: case 0xD9: case 0xE9: case 0xF9: // ADCB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            v16 = c->d.b.l + val + (c->cc & CC_C ? 1 : 0);
            update_flags_math(c, v16, c->d.b.l, val, false);
            c->d.b.l = (uint8_t)v16; c->cycles += 2; break;

        // --- SUBTRACT WITH CARRY (SBCA, SBCB) ---
        case 0x82: case 0x92: case 0xA2: case 0xB2: // SBCA
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            v16 = c->d.b.h - val - (c->cc & CC_C ? 1 : 0);
            update_flags_math(c, v16, c->d.b.h, val, true);
            c->d.b.h = (uint8_t)v16; c->cycles += 2; break;

        case 0xC2: case 0xD2: case 0xE2: case 0xF2: // SBCB
            ea = (c->mode == IMM) ? c->pc++ : get_ea(c);
            val = h->read(ea);
            v16 = c->d.b.l - val - (c->cc & CC_C ? 1 : 0);
            update_flags_math(c, v16, c->d.b.l, val, true);
            c->d.b.l = (uint8_t)v16; c->cycles += 2; break;

        // --- 16-BIT MATH (ADDD, SUBD) ---
        case 0xC3: case 0xD3: case 0xE3: case 0xF3: // ADDD
            if (c->mode == IMM) v16 = fetch16(c); 
            else { ea = get_ea(c); v16 = (h->read(ea) << 8) | h->read(ea+1); }
            {
                uint32_t r32 = c->d.w + v16;
                update_flags_math16(c, r32, c->d.w, v16, false);
                c->d.w = (uint16_t)r32;
            }
            c->cycles += 4; break;

        case 0x83: case 0x93: case 0xA3: case 0xB3: // SUBD
            if (c->mode == IMM) v16 = fetch16(c); 
            else { ea = get_ea(c); v16 = (h->read(ea) << 8) | h->read(ea+1); }
            {
                uint32_t r32 = c->d.w - v16;
                update_flags_math16(c, r32, c->d.w, v16, true);
                c->d.w = (uint16_t)r32;
            }
            c->cycles += 4; break;

        // --- MISC ---
        case 0x3D: // MUL
            // 1. Perform Multiplication
            c->d.w = (uint16_t)c->d.b.h * (uint16_t)c->d.b.l;
            
            // 2. Set Zero Flag (Standard)
            c->cc &= ~CC_Z; 
            if(c->d.w == 0) c->cc |= CC_Z;
            
            // 3. THE FIX: Set Carry Flag to Bit 7 of the Result (B register)
            c->cc &= ~CC_C;
            if (c->d.b.l & 0x80) c->cc |= CC_C; // <--- OFTEN MISSED!
            
            c->cycles += 11; 
            break;

        case 0x1D: // SEX (Sign Extend B into A)
            // 1. Perform Extension
            c->d.b.h = (c->d.b.l & 0x80) ? 0xFF : 0x00;
            
            // 2. THE FIX: Update N and Z based on 16-bit D, not just A
            set_nz16(c, c->d.w); // <--- Ensure this checks D, not A!
            
            c->cycles += 2; 
            break;
            
        // SYNC (Synchronize to External Event)
        case 0x13:
            c->is_waiting = true; // <--- Enter Halt State (waits for interrupt)
            c->cycles += 2; 
            break;

        case 0x3B: pull_s(c, 0xFF); c->cycles += 15; break; // RTI

        // --- 16-BIT COMPARES ---        
        // CMPX (Compare X) - 8C, 9C, AC, BC
        case 0x8C: case 0x9C: case 0xAC: case 0xBC:
            if (c->mode == IMM) { 
                v16 = fetch16(c); 
            } else { 
                ea = get_ea(c); 
                v16 = (h->read(ea) << 8) | h->read(ea+1); 
            }
            do_cmp16(c, c->x, v16);
            c->cycles += 4; 
            break;

        // CMPD (Compare D) - 10 83, 10 93, 10 A3, 10 B3
        case 0x1083: case 0x1093: case 0x10A3: case 0x10B3:
            if (c->mode == IMM) v16 = fetch16(c); 
            else { ea = get_ea(c); v16 = (h->read(ea) << 8) | h->read(ea+1); }
            do_cmp16(c, c->d.w, v16);
            c->cycles += 5; 
            break;

        // CMPY (Compare Y) - 10 8C, 10 9C, 10 AC, 10 BC
        case 0x108C: case 0x109C: case 0x10AC: case 0x10BC:
            if (c->mode == IMM) v16 = fetch16(c); 
            else { ea = get_ea(c); v16 = (h->read(ea) << 8) | h->read(ea+1); }
            do_cmp16(c, c->y, v16);
            c->cycles += 5; 
            break;
            
        // CMPU (Compare U) - 11 83, 11 93, 11 A3, 11 B3
        case 0x1183: case 0x1193: case 0x11A3: case 0x11B3:
            if (c->mode == IMM) v16 = fetch16(c); 
            else { ea = get_ea(c); v16 = (h->read(ea) << 8) | h->read(ea+1); }
            do_cmp16(c, c->u, v16);
            c->cycles += 5; 
            break;

        // CMPS (Compare S) - 11 8C, 11 9C, 11 AC, 11 BC
        case 0x118C: case 0x119C: case 0x11AC: case 0x11BC:
            if (c->mode == IMM) v16 = fetch16(c); 
            else { ea = get_ea(c); v16 = (h->read(ea) << 8) | h->read(ea+1); }
            do_cmp16(c, c->s, v16);
            c->cycles += 5; 
            break;

        // --- TRANSFER & EXCHANGE (TFR, EXG) ---
        // Vital for moving data between registers (e.g. TFR A,B)
        case 0x1F: // TFR
        case 0x1E: // EXG
        {
            val = fetch(c); // Postbyte: Source (High) -> Dest (Low)
            uint8_t r1 = (val >> 4) & 0x0F;
            uint8_t r2 = val & 0x0F;
            
            // Register Map: 0=D, 1=X, 2=Y, 3=U, 4=S, 5=PC, 8=A, 9=B, A=CC, B=DP
            // We use a helper lambda to resolve pointers
            auto get_reg_ptr = [&](uint8_t r, bool& is16) -> void* {
                switch(r) {
                    case 0: is16=true; return &c->d.w;
                    case 1: is16=true; return &c->x;
                    case 2: is16=true; return &c->y;
                    case 3: is16=true; return &c->u;
                    case 4: is16=true; return &c->s;
                    case 5: is16=true; return &c->pc;
                    case 8: is16=false; return &c->d.b.h;
                    case 9: is16=false; return &c->d.b.l;
                    case 10: is16=false; return &c->cc;
                    case 11: is16=false; return &c->dp;
                    default: return nullptr;
                }
            };

            bool size1 = false, size2 = false;
            void* p1 = get_reg_ptr(r1, size1);
            void* p2 = get_reg_ptr(r2, size2);
            
            if (p1 && p2) {
                // We default to the size of the Destination (standard behavior)
                if (size2) { // 16-bit Op
                    uint16_t* w1 = (uint16_t*)p1;
                    uint16_t* w2 = (uint16_t*)p2;
                    // If source is 8-bit, 6809 usually concatenates or is undefined. 
                    // We assume valid code uses matching sizes (e.g. TFR D,X).
                    if (c->ir == 0x1E) { uint16_t t = *w1; *w1 = *w2; *w2 = t; } // EXG
                    else { *w2 = *w1; } // TFR
                } else { // 8-bit Op
                    uint8_t* b1 = (uint8_t*)p1;
                    uint8_t* b2 = (uint8_t*)p2;
                    if (c->ir == 0x1E) { uint8_t t = *b1; *b1 = *b2; *b2 = t; } // EXG
                    else { *b2 = *b1; } // TFR
                }
            }
            c->cycles += 6;
        }
        break;

        // --- LOAD EFFECTIVE ADDRESS (LEA) ---
        // Calculates address but loads it into Register instead of reading memory.
        case 0x30: // LEAX
            c->mode = IND; c->x = get_ea(c); 
            if(c->x == 0) c->cc |= CC_Z; else c->cc &= ~CC_Z; // Affects Z
            c->cycles += 4; break;
        case 0x31: // LEAY
            c->mode = IND; c->y = get_ea(c); 
            if(c->y == 0) c->cc |= CC_Z; else c->cc &= ~CC_Z; // Affects Z
            c->cycles += 4; break;
        // LEAS (Load Effective Address to S) - 32
        case 0x32: 
            ea = resolve_indexed(c); 
            c->s = ea; 
            c->cycles += 2; 
            break;
        case 0x33: // LEAU
            c->mode = IND; c->u = get_ea(c); 
            c->cycles += 4; break;

        // COM (Complement) - 03, 43, 53
        case 0x43: c->d.b.h = ~c->d.b.h; set_nz(c, c->d.b.h); c->cc |= CC_C; c->cycles+=2; break; // COMA
        case 0x53: c->d.b.l = ~c->d.b.l; set_nz(c, c->d.b.l); c->cc |= CC_C; c->cycles+=2; break; // COMB
        case 0x03: case 0x63: case 0x73: // COM (Memory)
            ea = get_ea(c); val = ~h->read(ea); h->write(ea, val);
            set_nz(c, val); c->cc |= CC_C; c->cycles+=6; break;

        // --- SOFTWARE INTERRUPTS ---
        case 0x3F: // SWI (Push All, E=1, Vector $FFFA)
            c->cc |= CC_E; // Set Entire Flag
            push_s(c, 0xFF); // Push Regs
            c->cc |= (CC_I | CC_F); // Mask Interrupts
            c->pc = (h->read(0xFFFA) << 8) | h->read(0xFFFB);
            c->cycles += 19; 
            break;
            
        case 0x103F: // SWI2 (Push All, E=1, Vector $FFF4)
            c->cc |= CC_E;
            push_s(c, 0xFF);
            c->pc = (h->read(0xFFF4) << 8) | h->read(0xFFF5);
            c->cycles += 20; 
            break;

        case 0x113F: // SWI3 (Push All, E=1, Vector $FFF2)
            c->cc |= CC_E;
            push_s(c, 0xFF);
            c->pc = (h->read(0xFFF2) << 8) | h->read(0xFFF3);
            c->cycles += 20; 
            break;

        // CWAI (Clear and Wait for Interrupt)
        case 0x3C:
            val = fetch(c);  // Read immediate byte (AND mask)
            c->cc &= val;    // AND CC with Operand
            c->cc |= CC_E;   // Set Entire flag
            push_s(c, 0xFF); // Stack all registers
            c->is_waiting = true; // <--- Enter Halt State
            c->cycles += 20; 
            break;

#ifdef ENABLE_HD6309
        case 0x1086: // LDW Imm
            c->w.w = fetch16(c); set_nz16(c, c->w.w); c->cycles+=4; break;
        case 0x104F: // CLRD
            c->d.w = 0; set_nz16(c, 0); c->cycles+=3; break;
        case 0x105F: // CLRW
            c->w.w = 0; set_nz16(c, 0); c->cycles+=3; break;
#endif
        default: 
            // --- ILLEGAL INSTRUCTION TRAP ---
            // 1. Visual Alarm: Light LED 6
            ANNHal* hal_debug = (ANNHal*)c->user;
            hal_debug->hardware.setLed(6, 1); 
            
            // 2. Log the crime scene
            Serial.print("\n!!! CPU CRASH !!! Illegal Opcode: ");
            Serial.print(c->ir, HEX); // The bad byte
            Serial.print(" at PC: ");
            Serial.println(c->pc - 1, HEX); // -1 because fetch() advanced it
            printCrashDump();
            
            // 3. Panic Delay (2 seconds)
            // Allows you to see the LED and prevents flooding the Serial console
            delay(2000); 
            
            // 4. Turn off LED and Hard Reset
            hal_debug->hardware.setLed(6, 0);
            Reset(); 
            
            // 5. Exit this step cleanly
            c->cycles = 0; // Reset cycle counter for the fresh start
            break;
    }
    return 0;
}

void MOTO6809::do_cmp8(mc6809__t* c, uint8_t r, uint8_t m) {
    uint16_t res = (uint16_t)r - (uint16_t)m;
    c->cc &= ~(CC_N | CC_Z | CC_V | CC_C);
    
    if (res & 0x80) c->cc |= CC_N;
    if ((res & 0xFF) == 0) c->cc |= CC_Z;  // <--- MUST HAVE & 0xFF
    
    // Overflow: (Reg ^ Mem) & (Reg ^ Result) & 0x80
    if (((r ^ m) & 0x80) && ((r ^ res) & 0x80)) c->cc |= CC_V;

    // Borrow: Reg < Mem
    if (r < m) c->cc |= CC_C;
}

// 16-bit Comparison (CMPX, CMPY, CMPU, CMPS, CMPD)
void MOTO6809::do_cmp16(mc6809__t* c, uint16_t r, uint16_t m) {
    uint32_t res = (uint32_t)r - (uint32_t)m;
    c->cc &= ~(CC_N | CC_Z | CC_V | CC_C); // Clear flags

    if (res & 0x8000) c->cc |= CC_N;        // Negative (Bit 15)
    if ((res & 0xFFFF) == 0) c->cc |= CC_Z; // Zero
    
    // Overflow: (Reg ^ Mem) & (Reg ^ Result) & 0x8000
    if (((r ^ m) & 0x8000) && ((r ^ res) & 0x8000)) c->cc |= CC_V;

    // Borrow/Carry: Reg < Mem
    if (r < m) c->cc |= CC_C;
}

void MOTO6809::printCrashDump() {
    Serial.println("\n--- CRASH FLIGHT RECORDER ---");
    // Print from oldest to newest
    uint8_t idx = historyIdx; 
    for (int i = 0; i < 16; i++) {
        StepRecord& r = history[idx];
        Serial.print("["); 
        if (i < 9) Serial.print("0"); Serial.print(i + 1);
        Serial.print("] PC:"); Serial.print(r.pc, HEX);
        Serial.print("  OP:"); Serial.print(r.opcode, HEX);
        Serial.print("  A:");  Serial.print(r.accA, HEX);
        Serial.print("  B:");  Serial.print(r.accB, HEX);
        Serial.print("  X:");  Serial.print(r.regX, HEX);
        Serial.print("  S:");  Serial.println(r.regS, HEX);
        
        idx = (idx + 1) % 16;
    }
    Serial.println("-----------------------------");
}