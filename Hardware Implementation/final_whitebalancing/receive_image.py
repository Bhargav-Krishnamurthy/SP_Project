#!/usr/bin/env python3
"""
Receive the FPGA's raw RGB UART stream and save both:
  fpga_white_balance_raw.png       -- white balance only, from FPGA
  fpga_white_balance_stretched.png -- same image after Claude-style P1/P99
                                     percentile stretching on the host PC.

Usage:
    python receive_image.py COM5
    python receive_image.py /dev/ttyUSB0

Install:
    python -m pip install pyserial pillow numpy

IMPORTANT:
1. Program the FPGA and connect the Boolean board's PROG/UART USB port.
2. Run this script FIRST so it opens the serial port and waits for bytes.
3. Press btn[0] on the board to start processing.
4. Wait for all 196608 bytes, then the PNG files are saved.
"""

import sys
import time
import numpy as np
import serial
from PIL import Image

WIDTH = 256
HEIGHT = 256
NBYTES = WIDTH * HEIGHT * 3
BAUD = 115200
STRETCH_PCT = 1.0

def percentile_stretch_rgb(img_u8, pct=1.0):
    """Same per-channel percentile stretch as the supplied Claude code."""
    x = img_u8.astype(np.float32) / 255.0
    out = np.empty_like(x)

    for c in range(3):
        channel = x[:, :, c]
        lo = np.percentile(channel, pct)
        hi = np.percentile(channel, 100.0 - pct)
        if hi - lo < 1e-6:
            out[:, :, c] = channel
        else:
            out[:, :, c] = np.clip((channel - lo) / (hi - lo), 0.0, 1.0)

    return np.clip(np.round(out * 255.0), 0, 255).astype(np.uint8)

def main():
    if len(sys.argv) != 2:
        print("Usage: python receive_image.py <serial-port>")
        raise SystemExit(2)

    port = sys.argv[1]
    print(f"Opening {port} at {BAUD} baud.")
    print("Start the FPGA by pressing btn[0] after this script says it is waiting.")

    with serial.Serial(port, BAUD, timeout=2) as ser:
        ser.reset_input_buffer()
        print(f"Waiting to receive {NBYTES} bytes...")
        data = bytearray()
        last_report = time.monotonic()

        while len(data) < NBYTES:
            chunk = ser.read(min(4096, NBYTES - len(data)))
            if chunk:
                data.extend(chunk)

            now = time.monotonic()
            if now - last_report >= 1.0:
                print(f"\rReceived {len(data)}/{NBYTES} bytes", end="", flush=True)
                last_report = now

        print(f"\nReceived {len(data)} bytes.")

    raw = np.frombuffer(bytes(data), dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
    Image.fromarray(raw, mode="RGB").save("fpga_white_balance_raw.png")

    stretched = percentile_stretch_rgb(raw, STRETCH_PCT)
    Image.fromarray(stretched, mode="RGB").save("fpga_white_balance_stretched.png")

    print("Saved fpga_white_balance_raw.png (FPGA output).")
    print("Saved fpga_white_balance_stretched.png (host percentile stretch).")

if __name__ == "__main__":
    main()
