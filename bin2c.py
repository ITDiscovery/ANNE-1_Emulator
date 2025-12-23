import os
import sys
import subprocess

# --- CONFIGURATION ---
ASM_TOOL  = "asm6809"
ASM_FILE  = "monitor.asm"
BIN_FILE  = "monitor.bin"
LST_FILE  = "monitor.lst"
HEADER_FILE = "ANNE_ROM.h"

# --- TARGET MEMORY SIZE ---
# Must match ROM_SIZE in ANNEHal.h
ROM_SIZE = 4096 

def main():
    print(f"--- ANNE-1 Toolchain: Building {ASM_FILE} for ${ROM_SIZE:X} Byte ROM ---")

    # 1. RUN ASSEMBLER
    # -B: Binary output
    # -l: Create listing file
    cmd = [ASM_TOOL, "-B", "-l", LST_FILE, "-o", BIN_FILE, ASM_FILE]
    
    try:
        subprocess.run(cmd, check=True)
        print("✅ Assembly Successful.")
    except Exception as e:
        print(f"❌ Assembly FAILED: {e}")
        return

    # 2. READ & PAD BINARY
    try:
        with open(BIN_FILE, "rb") as f:
            data = bytearray(f.read())
    except FileNotFoundError:
        print("❌ Error: Binary file not found.")
        return

    # asm6809 might output the full 64KB range if ORG is used without care.
    # If the file is huge (e.g. > $F000 bytes), we strip leading padding.
    if len(data) > 60000:
        print(f"ℹ️  Detected full binary map. Stripping leading bytes...")
        # Assume code starts at $F000 (Offset 61440)
        data = data[61440:]

    # Critical Check: Is it too big?
    if len(data) > ROM_SIZE:
        print(f"❌ FATAL: ROM size {len(data)} exceeds {ROM_SIZE} bytes!")
        return
        
    # Critical Fix: Pad to exactly ROM_SIZE (4096 bytes)
    if len(data) < ROM_SIZE:
        print(f"ℹ️  Padding binary from {len(data)} to {ROM_SIZE} bytes...")
        # 0xFF is the standard value for empty EPROM space
        padding = b'\xFF' * (ROM_SIZE - len(data))
        data.extend(padding)

    # 3. VERIFY VECTORS
    # In a 4KB file mapped to $F000-$FFFF, the vectors ($FFF0-$FFFF) 
    # are at the very end of the file (offsets 4080-4095).
    # Reset Vector ($FFFE/$FFFF) is at offset 4094/4095.
    
    vec_offset = ROM_SIZE - 2
    reset_hi = data[vec_offset]
    reset_lo = data[vec_offset+1]
    reset_addr = (reset_hi << 8) | reset_lo
    
    print(f"🔍 Vector Verification:")
    print(f"   Reset Vector stored in ROM: ${reset_addr:04X}")

    # 4. GENERATE HEADER
    print(f"💾 Writing {HEADER_FILE}...")
    with open(HEADER_FILE, "w") as out:
        out.write(f"// Generated from {ASM_FILE} via Master Build Script\n")
        out.write(f"// Target: $F000-$FFFF ({ROM_SIZE} bytes)\n")
        out.write(f"// Reset Vector: 0x{reset_addr:04X}\n")
        out.write(f"#ifndef {HEADER_FILE.replace('.', '_').upper()}\n")
        out.write(f"#define {HEADER_FILE.replace('.', '_').upper()}\n\n")
        out.write("#include <pgmspace.h>\n\n")
        
        # Use explicit size in definition
        out.write(f"const uint8_t ANNE_MONITOR_ROM[{ROM_SIZE}] PROGMEM = {{\n")
        
        for i, byte in enumerate(data):
            out.write(f"0x{byte:02X}, ")
            if (i + 1) % 16 == 0:
                out.write("\n")
        
        out.write("};\n\n#endif\n")

    print(f"✅ DONE. {HEADER_FILE} ready for ANNEHal.cpp.")

if __name__ == "__main__":
    main()
