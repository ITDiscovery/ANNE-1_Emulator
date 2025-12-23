#ifndef TM1638_H
#define TM1638_H

#include <Arduino.h>
#include "PinAssignment.h"

// ESP8266 runs fast enough that we might need a tiny delay 
// for the TM1638 to catch up, but usually standard digitalWrite is fine.
#define DELAY_BIT delayMicroseconds(1)

// --- KEY MAPPING ---
const uint32_t SHIFT_MASK = 0x00000040; // SW2 is Shift

struct KeyMapping {
    uint32_t rawSignature; 
    char unshifted;        
    char shifted;          
};

// Key Map (Same as before)
const KeyMapping KEY_MAP[] = {
    { 0x00000004, '0', 'P' }, // SW1
    { 0x00000400, '1', 'G' }, // SW3
    { 0x00004000, '2', '+' }, // SW4
    { 0x00040000, '3', 'S' }, // SW5
    { 0x00400000, '4', 'R' }, // SW6
    { 0x04000000, '5', '5' }, // SW7
    { 0x40000000, '6', '6' }, // SW8
    { 0x00000002, '7', '7' }, // SW9
    { 0x00000020, '8', '8' }, // SW10
    { 0x00000200, '9', '9' }, // SW11
    { 0x00002000, 'A', 'A' }, // SW12
    { 0x00020000, 'B', 'B' }, // SW13
    { 0x00200000, 'C', 'C' }, // SW14
    { 0x02000000, 'D', 'D' }, // SW15
    { 0x20000000, 'E', 'E' }, // SW16
    { 0x00000001, 'F', 'F' }, // SW17
    { 0x00000010, 'a', 'a' }, // SW18
    { 0x00000100, 'd', 'd' }, // SW19
};

const uint8_t MAP_SIZE = sizeof(KEY_MAP) / sizeof(KeyMapping);
const uint8_t HEX_FONT[] = { 0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71 };

class TM1638 {
  private:
    void sendByte(uint8_t data) {
      for (int i = 0; i < 8; i++) {
        digitalWrite(PIN_CLK, LOW);
        DELAY_BIT;
        digitalWrite(PIN_DIO, (data & 1)); 
        data >>= 1;
        digitalWrite(PIN_CLK, HIGH);
        DELAY_BIT;
      }
    }

  public:
    void begin() {
      pinMode(PIN_STB1, OUTPUT);
      pinMode(PIN_CLK, OUTPUT);
      pinMode(PIN_DIO, OUTPUT);
      digitalWrite(PIN_STB1, HIGH);
      digitalWrite(PIN_CLK, HIGH);
      sendCommand(0x8F); // Activate
      clear();
    }

    void sendCommand(uint8_t cmd) {
      digitalWrite(PIN_STB1, LOW);
      DELAY_BIT;
      sendByte(cmd);
      digitalWrite(PIN_STB1, HIGH);
      DELAY_BIT;
    }

    void setupDisplay(bool on, uint8_t intensity) {
      sendCommand(0x80 | (on ? 8 : 0) | (intensity & 7));
    }

    void writeDigit(uint8_t pos, uint8_t hexValue) {
      if (pos > 7) return;
      sendCommand(0x44); 
      digitalWrite(PIN_STB1, LOW);
      DELAY_BIT;
      sendByte(0xC0 | (pos << 1)); 
      sendByte(HEX_FONT[hexValue & 0x0F]);
      digitalWrite(PIN_STB1, HIGH);
    }
    
    void setLed(uint8_t ledIdx, uint8_t state) {
      if (ledIdx > 7) return;
      sendCommand(0x44); 
      digitalWrite(PIN_STB1, LOW);
      DELAY_BIT;
      sendByte(0xC1 + (ledIdx << 1)); 
      sendByte(state ? 1 : 0);
      digitalWrite(PIN_STB1, HIGH);
    }

    void clear() {
      sendCommand(0x40); 
      digitalWrite(PIN_STB1, LOW);
      DELAY_BIT;
      sendByte(0xC0); 
      for(int i=0; i<16; i++) sendByte(0x00); 
      digitalWrite(PIN_STB1, HIGH);
    }

    uint32_t readButtons() {
      uint32_t packedData = 0;
      digitalWrite(PIN_STB1, LOW);
      DELAY_BIT;
      sendByte(0x42); 
      pinMode(PIN_DIO, INPUT); // PULLUP internal often weak on ESP, external recommended but try INPUT first
      DELAY_BIT; 
      
      for (int i = 0; i < 4; i++) {
        uint8_t byteVal = 0;
        for (int b = 0; b < 8; b++) {
          digitalWrite(PIN_CLK, LOW);
          DELAY_BIT;
          if (digitalRead(PIN_DIO)) byteVal |= (1 << b);
          digitalWrite(PIN_CLK, HIGH);
          DELAY_BIT;
        }
        packedData |= ((uint32_t)byteVal << (i * 8));
      }
      pinMode(PIN_DIO, OUTPUT);
      digitalWrite(PIN_STB1, HIGH);
      return packedData;
    }
};
#endif