import os
import sys
import subprocess

# --- CONFIGURATION ---
ASM_TOOL    = "asm6809"
ASM_FILE    = "monitor.asm"       # [CHANGED] Input Source
BIN_FILE    = "monitor.bin"
LST_FILE    = "monitor.lst"
HEADER_FILE = "ANNE_ROM.h"        # [CHANGED] Output Header
ARRAY_NAME  = "ANNE_MONITOR_ROM"  # [CHANGED] Array Name

# --- TARGET CONFIGURATION ---
TARGET_START = 0xF000   # [CHANGED] Monitor Start Address (Top 4K)
TARGET_SIZE  = 0x1000   # [CHANGED] 4KB Window ($F000-$FFFF)

def main():
    print(f"--- ANNE-1 MONITOR Toolchain: Packaging for ${TARGET_START:X} ---")

    # 1. RUN ASSEMBLER
    print(f"🔨 Assembling {ASM_FILE}...")
    cmd = [ASM_TOOL, ASM_FILE, "-o", BIN_FILE, "-l", LST_FILE]
    try:
        subprocess.run(cmd, check=True)
    except Exception as e:
        print(f"❌ Assembly Failed: {e}")
        return

    # 2. READ BINARY
    try:
        with open(BIN_FILE, "rb") as f:
            data = bytearray(f.read())
    except FileNotFoundError:
        print("❌ Binary not found.")
        return

    # STRIP PADDING
    # Assemblers often pad from $0000 to ORG. 
    # If file size suggests it contains the full memory map, strip the prefix.
    if len(data) >= TARGET_START: 
        print(f"ℹ️  Stripping leading {TARGET_START} bytes...")
        data = data[TARGET_START:] 

    # SAFETY PADDING / TRUNCATION
    if len(data) < TARGET_SIZE:
        pad = TARGET_SIZE - len(data)
        print(f"⚠️  Padding with {pad} bytes...")
        data.extend(b'\xFF' * pad)
    elif len(data) > TARGET_SIZE:
        print(f"❌ FATAL: Size {len(data)} exceeds limit ({TARGET_SIZE})!")
        return

    # 3. GENERATE HEADER
    with open(HEADER_FILE, "w") as out:
        out.write(f"// ANNE-1 Monitor ROM \n")
        out.write(f"// Target: 0x{TARGET_START:X}, Size: {len(data)} bytes\n")
        out.write(f"#ifndef ANNE_ROM_H\n#define ANNE_ROM_H\n#include <pgmspace.h>\n\n")
        out.write(f"const uint8_t {ARRAY_NAME}[] PROGMEM = {{\n")
        for i, b in enumerate(data):
            out.write(f"0x{b:02X}, ")
            if (i+1) % 16 == 0: 
                out.write("\n")
        out.write("};\n#endif\n")
    
    print(f"✅ DONE. Header written to {HEADER_FILE}")

if __name__ == "__main__":
    main()
