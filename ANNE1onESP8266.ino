// ANN1onESP8266.ino - ANNA-1 (6809) Emulator on ESP8266

#include <ESP8266WiFi.h> 
#include "PinAssignment.h"
#include "ANNEHal.h"      
#include "MOTO6809.h" 

// --- Global Pointers for Dynamic Allocation ---
// Initialize to nullptr to prevent static initialization crashes.
ANNHal *hal = nullptr;
MOTO6809 *cpu = nullptr;

// --- INTERRUPT GLOBALS ---
// Volatile is required so the main loop sees changes made by interrupts
volatile bool triggerReset = false;
volatile bool triggerNMI = false;
unsigned long lastHwUpdate = 0;

// --- INTERRUPT SERVICE ROUTINES (ISRs) ---
// IRAM_ATTR places these in RAM for maximum speed
void IRAM_ATTR handleReset() {
  triggerReset = true;
}

void IRAM_ATTR handleNMI() {
  triggerNMI = true;
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);
  delay(500); 

  // --- BUTTON SETUP ---
  pinMode(PIN_RS_BTN, INPUT_PULLUP);
  pinMode(PIN_ST_BTN, INPUT_PULLUP);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_RS_BTN), handleReset, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ST_BTN), handleNMI, FALLING);
    
  Serial.println("\n--- ANNE-1 on ESP8266 Ready ---"); 

  hal = new ANNHal();
  if (hal == nullptr) {
    Serial.println("ANNE-1 HAL Init Failed");
    while(1);
  }

  if (!hal->initMemory()) {
      Serial.println("Memory Allocation Failed!");
      while(1);
  }

  // Simplified call - Internal logic handles the rest
  hal->flashMonitor(); 
  Serial.println("Monitor ROM Loaded.");

  cpu = new MOTO6809(hal);
  if (cpu == nullptr) {
    Serial.println("6809 CPU Init Failed!");
    while(1);
  }
  
  hal->begin();
  cpu->Reset(); 

  pinMode(PIN_MODE_SELECT, INPUT_PULLUP);
  delay(10);
  
  if (digitalRead(PIN_MODE_SELECT) == LOW) {
    hal->keypadMode = false;
    Serial.println(">> Jumper Detected: TTY Mode Active");
    Serial.println(">> ANNE-1 BASIC at 0xC000");
  } else {
    hal->keypadMode = true;
    Serial.println(">> No Jumper: Keypad Mode Active");
  }
  hal->debug_dump = false;
}

void loop() {
  // 1. INTERRUPT HANDLING (Instant)
  if (triggerReset) {
    if (cpu != nullptr) { Serial.println("[ISR] RESET"); cpu->Reset(); }
    triggerReset = false; 
  }
  
  if (triggerNMI) {
    if (cpu != nullptr) { 
      // --- DIAGNOSTIC DUMP ---
      uint16_t currentPC = cpu->get_pc_register();
      Serial.print("\nNMI Button Pressed!");
      Serial.print(" CPU at PC: ");
      Serial.println(currentPC, HEX);
      
      // Optional: Peek at the instruction causing the hang
      uint8_t opcode = hal->read(currentPC);
      Serial.print("             Opcode: ");
      Serial.println(opcode, HEX);
      // -----------------------

      cpu->triggerNMI(); 
    }
    triggerNMI = false;   
  }

  // 2. RUN CPU (Massive Speed Chunk)
  // We can safely run 16k+ cycles because writes are now instant!
  if (cpu != nullptr) {
      cpu->Run(4096);
      yield();
  }
  //uint16_t previous_pc = cpu->get_pc_register(); 

  // 3. UPDATE HARDWARE (Every 30ms -> ~33fps)
  // This handles both screen refresh and keypad polling
  if (millis() - lastHwUpdate > 30) {
     if (hal != nullptr) {
       hal->updateHardware(); 
     }
     lastHwUpdate = millis();
  }
  
  yield(); 
}