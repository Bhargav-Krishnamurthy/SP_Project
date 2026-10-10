
import numpy as np
import cv2

# --------------------------------------------------
# Configuration
# --------------------------------------------------
WIDTH = 256
HEIGHT = 256

INPUT_MEM = "white_balance_output.mem"
OUTPUT_MEM = "white_balance_normalized.mem"
OUTPUT_PNG = "white_balance_normalized.png"

STRETCH_PCT = 1.0


# --------------------------------------------------
# Load 24-bit RGB pixels from the Verilog output.
# Each line must contain one six-digit RGB hex word.
# Format: R[7:0], G[7:0], B[7:0]
# --------------------------------------------------
def load_mem(filename, width, height):
    pixels = []

    with open(filename, "r") as f:
        for line_number, line in enumerate(f, start=1):
            value = line.strip()

            if not value:
                continue

            if len(value) != 6:
                raise ValueError(
                    f"Line {line_number}: expected 6 hex digits, "
                    f"got {value!r}"
                )

            try:
                pixel = int(value, 16)
            except ValueError:
                raise ValueError(
                    f"Line {line_number}: invalid hex value {value!r}"
                )

            pixels.append([
                (pixel >> 16) & 0xFF,  # Red
                (pixel >> 8) & 0xFF,   # Green
                pixel & 0xFF           # Blue
            ])

    expected = width * height

    if len(pixels) != expected:
        raise ValueError(
            f"Expected {expected} pixels, got {len(pixels)}"
        )

    return np.asarray(
        pixels, dtype=np.uint8
    ).reshape(height, width, 3)


# --------------------------------------------------
# Claude's percentile stretching:
# independently stretch R, G and B using P1 and P99.
# --------------------------------------------------
def percentile_stretch(img, pct=1.0):
    img = img.astype(np.float32) / 255.0
    out = np.empty_like(img)

    for c in range(3):
        channel = img[:, :, c]

        lo = np.percentile(channel, pct)
        hi = np.percentile(channel, 100.0 - pct)

        if hi - lo < 1e-6:
            out[:, :, c] = channel
        else:
            out[:, :, c] = np.clip(
                (channel - lo) / (hi - lo),
                0.0,
                1.0
            )

    return out


# --------------------------------------------------
# Main
# --------------------------------------------------
def main():
    # Reconstruct the Verilog output image.
    wb_u8 = load_mem(INPUT_MEM, WIDTH, HEIGHT)

    print("Loaded white-balanced image:", wb_u8.shape)

    # Apply the additional normalization.
    normalized = percentile_stretch(
        wb_u8, STRETCH_PCT
    )

    # Convert back to 8-bit RGB.
    normalized_u8 = np.clip(
        np.round(normalized * 255.0),
        0, 255
    ).astype(np.uint8)

    # OpenCV expects BGR when saving an RGB image.
    cv2.imwrite(
        OUTPUT_PNG,
        cv2.cvtColor(normalized_u8, cv2.COLOR_RGB2BGR)
    )

    # Export the normalized RGB image as 24-bit words.
    with open(OUTPUT_MEM, "w") as f:
        for y in range(HEIGHT):
            for x in range(WIDTH):
                r, g, b = map(
                    int, normalized_u8[y, x]
                )

                pixel = (r << 16) | (g << 8) | b
                f.write(f"{pixel:06X}\n")

    print("Saved normalized image:", OUTPUT_PNG)
    print("Saved normalized memory:", OUTPUT_MEM)

    print("\nChannel statistics (0-255 scale):")

    for c, name in enumerate(["Red", "Green", "Blue"]):
        original = wb_u8[:, :, c].astype(np.float32)
        result = normalized_u8[:, :, c].astype(np.float32)

        print(
            f"{name:5s}: "
            f"before mean={original.mean():7.2f}, "
            f"after mean={result.mean():7.2f}, "
            f"after min={result.min():3.0f}, "
            f"after max={result.max():3.0f}"
        )


if __name__ == "__main__":
    main()

