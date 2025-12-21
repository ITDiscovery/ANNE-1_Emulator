# ANNE-1_Emulator
# ANNE-1: The ESP8266 6809 Emulator

**ANNE-1** is a Motorola 6809 computer emulator running entirely on an ESP8266 microcontroller (NodeMCU/Wemos D1 Mini). It recreates the experience of a classic single-board computer (like the KIM-1), featuring a physical hex keypad/display interface and a full serial TTY monitor.

## 🚀 Features

* **Core:** Full emulation of the **Motorola 6809 CPU** (User & System stacks, Index registers, 16-bit math).
* **Dual Mode:**
    * **Keypad Mode:** Operates standalone using a TM1638 LED & Key module.
    * **TTY Mode:** Operates via USB Serial Terminal (Putty/Screen) with a KIM-1 style monitor.
* **Memory:** 16KB User RAM (`$0000` - `$3FFF`) + 8KB System ROM.
* **I/O:**
    * Emulated **6850 ACIA** for Serial communications.
    * Direct Hardware Abstraction Layer (HAL) for the TM1638 display.

## 🛠 Hardware Requirements

1.  **ESP8266 Board:** Wemos D1 Mini, NodeMCU, or generic ESP-12F.
2.  **TM1638 Module:** Common 8-digit, 8-key, 8-LED display board.
3.  **Jumper Wire:** Required for selecting TTY mode on boot.

### Wiring

| ESP8266 Pin | Function | TM1638 Pin | Note |
| :--- | :--- | :--- | :--- |
| **D0** (GPIO16) | Strobe (STB) | STB | Chip Select |
| **D1** (GPIO5) | Data (DIO) | DIO | Bi-directional Data |
| **D5** (GPIO14) | Clock (CLK) | CLK | Synchronous Clock |
| **D7** (GPIO13) | Mode Select | **GND** | **Low** = TTY Mode, **High/Open** = Keypad Mode |
| **3.3V/5V** | Power | VCC | 3.3V recommended for logic, 5V for brightness |
| **GND** | Ground | GND | Common Ground |

## 💾 Memory Map

| Address Range | Description | Notes |
| :--- | :--- | :--- |
| `$0000` - `$3FFF` | **System RAM** | 16KB User Memory. Zero page `$0000-$00FF`. |
| `$4000` - `$BFFF` | *Unmapped* | Expansion space. |
| `$C000` | **Keyboard Input** | Read-only. Returns ASCII of pressed key. |
| `$C001` - `$C003` | **Display Output** | Write-only. Hi/Lo Byte and Data Byte. |
| `$C004` | **ACIA Status** | Emulated 6850 Status (RDRF/TDRE). |
| `$C005` | **ACIA Data** | Emulated 6850 Data (UART I/O). |
| `$E000` - `$FFFF` | **Monitor ROM** | "GoldMaster" ANNE-1 Monitor Firmware. |

---

## 🖥 Usage

The system detects the operating mode at startup based on the state of **Pin D7**.

### 1. Keypad Mode (Default)
*Boot with D7 disconnected.*
The system behaves like a classic trainer kit. Use the keys on the TM1638 to enter hex codes.

**Key Mapping:**
* **0-F:** Enter Hex Data.
* **A (Button 1):** `AD` (Address Mode) - Select memory address.
* **B (Button 2):** `DA` (Data Mode) - Write data and increment.
* **C (Button 3):** `+` (Increment) - Next address.
* **D (Button 4):** `GO` - Execute program at current address.
* **Shift + ST:** NMI (Stop/Break).
* **Shift + RS:** Reset.

### 2. TTY Mode (Serial Monitor)
*Boot with D7 connected to GND.*
Connect via USB (115200 baud). The system acts as a KIM-1 style terminal.

**Commands:**
* **Input:** Type 4 digits for Address, 2 digits for Data.
* **SPACE:**
    * *If Address typed:* Open that address.
    * *If Data typed:* Store data and move to next address (`Store+`).
* **X:** Exit Data Mode (return to prompt).
* **G:** Go (Execute program at current address).
* **R:** Reset system.
* **P:** Restore PC (Return to Monitor entry).

**Example Session:**
```text
ANNE-1
> 2000 [SPACE]      (Select Address $2000)
2000 - 00           (Shows current data)
> A9 [SPACE]        (Write $A9 to $2000, move to $2001)
2001 - 00
> [X]               (Exit to prompt)
> 2000 [SPACE]      (Verify)
2000 - A9
> [G]               (Run program)
```

## Installation
- Clone this repository.
- Open ANNE1onESP8266.ino in the Arduino IDE.
- Install the ESP8266 Board Manager package.
- Optional: If using a different TM1638 library, ensure pin mappings in TM1638.h match your wiring.
- Upload to your device.

 ## Credits
- Emulator Core: Custom C++ implementation of the MC6809.
- Monitor: Custom 6809 Assembly ("GoldMaster 10") inspired by the MOS KIM-1.
- Platform: ESP8266 Community.
