"""Receive all five FPGA images over UART:
  0: white_balanced
  1: gamma_corrected
  2: unsharp_masking
  3: fused_output
  4: weight_maps  (R = Wbar1*256, G = Wbar2*256, B = 0)
usage: python capture_fusion.py PORT [BAUD=115200] [SIZE=256]
Press BTN0 on the board AFTER this script says the port is open.
"""
import sys, time, serial, numpy as np
from PIL import Image

PORT = sys.argv[1]
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
SIZE = int(sys.argv[3]) if len(sys.argv) > 3 else 256
N    = SIZE * SIZE * 3
NAMES = {
    0: "white_balanced",
    1: "gamma_corrected",
    2: "unsharp_masking",
    3: "fused_output",
    4: "weight_maps",
}

ser = serial.Serial(PORT, BAUD, timeout=1)
ser.reset_input_buffer()
print("Port open. Now press BTN0 (start) on the board...")

def read_exact(n, idle_timeout=30):
    buf = bytearray()
    last = time.time()
    while len(buf) < n:
        chunk = ser.read(min(4096, n - len(buf)))
        if chunk:
            buf += chunk
            last = time.time()
        elif time.time() - last > idle_timeout:
            break
    return bytes(buf)

def find_header(wait=120):
    win = bytearray()
    last = time.time()
    while True:
        b = ser.read(1)
        if not b:
            if time.time() - last > wait:
                return None
            continue
        last = time.time()
        win += b
        win = win[-4:]
        if len(win) == 4 and win[0] == 0xA5 and win[1] == 0x5A and win[3] == 0x00 and win[2] in NAMES:
            return win[2]

def save_png_safe(arr, mode, path):
    try:
        Image.fromarray(arr, mode).save(path)
        return path
    except PermissionError:
        alt = path.replace(".png", "_new.png")
        Image.fromarray(arr, mode).save(alt)
        print("   warning:", path, "was locked; saved", alt, "instead")
        return alt

ok = True
for expect in (0, 1, 2, 3, 4):
    sid = find_header(wait=120 if expect == 0 else 30)
    if sid is None:
        print("no header for", NAMES[expect], "- giving up")
        ok = False
        break
    if sid != expect:
        print("warning: expected stage", expect, "got", sid)
    t0 = time.time()
    data = read_exact(N)
    print("%-16s received %d of %d bytes (%.1fs)" % (NAMES[sid], len(data), N, time.time() - t0))
    if len(data) != N:
        ok = False
        break
    img = np.frombuffer(data, dtype=np.uint8).reshape(SIZE, SIZE, 3)
    with open(NAMES[sid] + ".mem", "w") as f:
        for p in img.reshape(-1, 3):
            f.write("%02x%02x%02x\n" % tuple(p))
    png_path = save_png_safe(img, "RGB", NAMES[sid] + ".png")
    if sid == 4:
        save_png_safe(img[:, :, 0], "L", "normalized_weight1.png")
        save_png_safe(img[:, :, 1], "L", "normalized_weight2.png")
    print("   saved", NAMES[sid] + ".mem /", png_path)

print("ALL DONE" if ok else "INCOMPLETE - press BTN0 again to re-send (no reprogram needed)")
