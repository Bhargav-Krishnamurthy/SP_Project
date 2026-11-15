"""Receive the three FPGA images (white balanced, gamma, unsharp) over UART.
usage: python capture_all.py PORT [BAUD=115200] [SIZE=256]
Press BTN0 on the board AFTER this script says the port is open.
Writes (RRGGBB per line, same format as image.mem):
  white_balanced.mem  gamma_corrected.mem  unsharp_masking.mem   (+ .png of each)
"""
import sys, time, serial, numpy as np
from PIL import Image

PORT = sys.argv[1]
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200   # must match UART_BAUD in the Verilog
SIZE = int(sys.argv[3]) if len(sys.argv) > 3 else 256
N    = SIZE * SIZE * 3
NAMES = {0: "white_balanced", 1: "gamma_corrected", 2: "unsharp_masking"}

ser = serial.Serial(PORT, BAUD, timeout=1)
ser.reset_input_buffer()
print("Port open. Now press BTN0 (start) on the board...")

def read_exact(n, idle_timeout=30):
    buf = bytearray(); last = time.time()
    while len(buf) < n:
        chunk = ser.read(min(4096, n - len(buf)))
        if chunk:
            buf += chunk; last = time.time()
        elif time.time() - last > idle_timeout:
            break
    return bytes(buf)

def find_header(expect, wait=120):
    win = bytearray(); last = time.time()
    while True:
        b = ser.read(1)
        if not b:
            if time.time() - last > wait: return None
            continue
        last = time.time(); win += b; win = win[-4:]
        if len(win) == 4 and win[0] == 0xA5 and win[1] == 0x5A and win[3] == 0x00 and win[2] in NAMES:
            return win[2]

ok = True
for expect in (0, 1, 2):
    sid = find_header(expect, wait=120 if expect == 0 else 30)
    if sid is None:
        print("no header for", NAMES[expect], "- giving up"); ok = False; break
    if sid != expect: print("warning: expected stage", expect, "got", sid)
    t0 = time.time()
    data = read_exact(N)
    print("%-16s received %d of %d bytes (%.1fs)" % (NAMES[sid], len(data), N, time.time() - t0))
    if len(data) != N:
        ok = False; break
    img = np.frombuffer(data, dtype=np.uint8).reshape(SIZE, SIZE, 3)
    with open(NAMES[sid] + ".mem", "w") as f:
        for p in img.reshape(-1, 3): f.write("%02x%02x%02x\n" % tuple(p))
    Image.fromarray(img, "RGB").save(NAMES[sid] + ".png")
    print("   saved", NAMES[sid] + ".mem /", NAMES[sid] + ".png")
print("ALL DONE" if ok else "INCOMPLETE - press BTN0 again to re-send (no reprogram needed)")
