"""Linearly stretch white_balanced.mem (per-channel 1%-99% percentile stretch
and min-max stretch) and save PNGs + summary statistics for verification.

usage:
  python stretch_wb.py [white_balanced.mem] [pct=1]
"""
import sys
import numpy as np
from PIL import Image


def load_mem(path, size=256):
    pixels = [
        (int(line[0:2], 16), int(line[2:4], 16), int(line[4:6], 16))
        for line in open(path).read().split()
    ]
    return np.array(pixels, dtype=np.uint8).reshape(size, size, 3)


def linear_stretch_percentile(img_u8, pct=1.0):
    out = np.empty_like(img_u8)
    bounds = []
    for c in range(3):
        ch = img_u8[:, :, c].astype(np.float32)
        lo = float(np.percentile(ch, pct))
        hi = float(np.percentile(ch, 100.0 - pct))
        if hi <= lo:
            out[:, :, c] = img_u8[:, :, c]
        else:
            scaled = np.clip((ch - lo) * 255.0 / (hi - lo), 0.0, 255.0)
            out[:, :, c] = np.round(scaled).astype(np.uint8)
        bounds.append((lo, hi))
    return out, bounds


def linear_stretch_minmax(img_u8):
    return linear_stretch_percentile(img_u8, pct=0.0)


if __name__ == "__main__":
    mem_path = sys.argv[1] if len(sys.argv) > 1 else "white_balanced.mem"
    pct = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    wb = load_mem(mem_path)
    wb_pct, bounds_pct = linear_stretch_percentile(wb, pct=pct)
    wb_mm, bounds_mm = linear_stretch_minmax(wb)

    Image.fromarray(wb_pct, "RGB").save("white_balanced_stretched.png")
    Image.fromarray(wb_mm, "RGB").save("white_balanced_minmax.png")

    with open("white_balanced_stretched.mem", "w") as f:
        for p in wb_pct.reshape(-1, 3):
            f.write("%02x%02x%02x\n" % tuple(p))

    print("Input (%s):" % mem_path)
    print("  min per ch:", wb.min(axis=(0, 1)), "max per ch:", wb.max(axis=(0, 1)), "mean:", wb.mean(axis=(0, 1)).round(2))
    print("Percentile stretch (%.1f%% - %.1f%%):" % (pct, 100.0 - pct))
    print("  bounds (R,G,B):", [(round(a, 1), round(b, 1)) for a, b in bounds_pct])
    print("  min per ch:", wb_pct.min(axis=(0, 1)), "max per ch:", wb_pct.max(axis=(0, 1)), "mean:", wb_pct.mean(axis=(0, 1)).round(2))
    print("  saved white_balanced_stretched.png / white_balanced_stretched.mem")
    print("Min-Max stretch (0% - 100%):")
    print("  bounds (R,G,B):", [(int(a), int(b)) for a, b in bounds_mm])
    print("  min per ch:", wb_mm.min(axis=(0, 1)), "max per ch:", wb_mm.max(axis=(0, 1)), "mean:", wb_mm.mean(axis=(0, 1)).round(2))
    print("  saved white_balanced_minmax.png")
