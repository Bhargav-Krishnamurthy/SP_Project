import sys
g = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
out = sys.argv[2] if len(sys.argv) > 2 else "gamma_lut.mem"
with open(out, "w") as f:
    for i in range(256):
        f.write("%02x\n" % int(255.0 * (i / 255.0) ** g + 0.5))
print("wrote", out, "gamma =", g)
