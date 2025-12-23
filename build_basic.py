import os
import sys
import subprocess

# --- CONFIGURATION ---
ASM_TOOL    = "asm6809"
ASM_FILE    = "ExBasROM.asm"
BIN_FILE    = "ExBasROM.bin"
LST_FILE    = "ExBasROM.lst"
HEADER_FILE = "BASIC_ROM.h"
ARRAY_NAME  = "ANNE_BASIC_ROM"

# --- TARGET CONFIGURATION ---
TARGET_START = 0xC000
TARGET_SIZE  = 12288   # 12KB Window ($C000-$EFFF)

def main():
    print(f"--- ANNE-1 BASIC Toolchain: Packaging for ${TARGET_START:X} ---")

    # 1. RUN ASSEMBLER
    # We run this automatically to ensure the .bin matches the .asm
    print(f"🔨 Assembling {ASM_FILE}...")
    cmd = [ASM_TOOL, ASM_FILE, "-o", BIN_FILE, "-l", LST_FILE]
    try:
        subprocess.run(cmd, check=True)
        print("✅ Assembly Successful.")
    except subprocess.CalledProcessError:
        print("❌ FATAL: Assembly Failed. Check errors above.")
        return
    except FileNotFoundError:
        print(f"❌ FATAL: '{ASM_TOOL}' not found in PATH. Please install asm6809 or remove step 1.")
        return

    # 2. READ BINARY
    try:
        with open(BIN_FILE, "rb") as f:
            data = bytearray(f.read())
        print(f"ℹ️  Read {len(data)} bytes from {BIN_FILE}")
    except FileNotFoundError:
        print("❌ Error: Binary file not found.")
        return

    # --- STRIP ASSEMBLER PADDING ---
    # asm6809 usually outputs a file starting at $0000. 
    # If the file is huge (> 49KB), we strip the first $C000 bytes.
    if len(data) > TARGET_START: 
        print(f"ℹ️  Detected assembler padding. Stripping leading {TARGET_START} bytes...")
        data = data[TARGET_START:] 

    # --- SAFETY PADDING (THE FIX) ---
    # Ensure the array is exactly 12KB. If it's short, fill with 0xFF.
    if len(data) < TARGET_SIZE:
        pad_len = TARGET_SIZE - len(data)
        print(f"⚠️  Binary is {len(data)} bytes (Short). Padding with {pad_len} bytes of 0xFF...")
        data.extend(b'\xFF' * pad_len)
    elif len(data) > TARGET_SIZE:
        print(f"❌ FATAL: BASIC size {len(data)} exceeds allocated {TARGET_SIZE} bytes!")
        return

    # 3. GENERATE HEADER
    print(f"💾 Writing {HEADER_FILE}...")
    with open(HEADER_FILE, "w") as out:
        out.write(f"// Generated from {ASM_FILE} via build_basic.py\n")
        out.write(f"// Target Address: 0x{TARGET_START:X}\n")
        out.write(f"// Total Size: {len(data)} bytes (padded)\n")
        out.write(f"#ifndef {HEADER_FILE.replace('.', '_').upper()}\n")
        out.write(f"#define {HEADER_FILE.replace('.', '_').upper()}\n\n")
        out.write("#include <pgmspace.h>\n\n")
        
        out.write(f"const uint8_t {ARRAY_NAME}[] PROGMEM = {{\n")
        
        for i, byte in enumerate(data):
            out.write(f"0x{byte:02X}, ")
            if (i + 1) % 16 == 0:
                out.write("\n")
        
        out.write("};\n\n#endif\n")

    print(f"✅ DONE. {HEADER_FILE} ready for ANNEHal.cpp.")

if __name__ == "__main__":
    main()
