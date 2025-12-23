#ifndef MOTO6809_H
#define MOTO6809_H

#include <stdint.h>
#include <Arduino.h> 
#include <stdbool.h>
#include <setjmp.h> // Needed for jmp_buf in the core structure

// --- Forward Declarations for Clean Compilation ---

// Forward declare the C struct type defined fully in MOTO6809.cpp
typedef struct mc6809 mc6809__t; 

// Forward declare the C++ classes
class ANNHal; 

// --- MOTO6809 C++ Wrapper Class ---
class MOTO6809 {
private:
    mc6809__t* cpu_core = nullptr; 
    ANNHal* hal;

public:
    // Constructor/Destructor
    MOTO6809(ANNHal* halInstance);
    ~MOTO6809(); // Declare destructor to free memory

    // Public Interface Methods
    void Reset();
    void Run(int32_t cycles);
    int step(); // Calls the instruction execution logic

    // Allow manual jump for diagnostics
    void setPC(uint16_t val);

    // C++ Accessors for Debugging (Use explicit structure access in .cpp)
    uint16_t get_pc_register();
    uint8_t get_A();
    uint8_t get_B();

    // Core Memory Accessors for C++ side (Must match .cpp implementation)
    uint8_t read8(uint16_t addr) const; 
    void write8(uint16_t addr, uint8_t val) const; 

    void triggerNMI();
    void cancelWait();
    
private:
    // C-STYLE GLUE FUNCTIONS (Implemented in MOTO6809.cpp)
    static uint8_t core_read(struct mc6809 *cpu, uint16_t addr, bool is_code);
    static void core_write(struct mc6809 *cpu, uint16_t addr, uint8_t val);
    static void core_fault(struct mc6809 *cpu, int fault_code);
};

#endif