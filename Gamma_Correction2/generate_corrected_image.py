from pathlib import Path
from PIL import Image

# ---------- CONFIGURATION ----------
INPUT_MEM = "gamma_corrected_rgb.mem"
OUTPUT_IMAGE = "gamma_corrected_image.png"

WIDTH = 256
HEIGHT = 256
CHANNELS = 3
EXPECTED_VALUES = WIDTH * HEIGHT * CHANNELS


def main():
    mem_path = Path(INPUT_MEM)

    if not mem_path.is_file():
        raise FileNotFoundError(
            f"Cannot find '{INPUT_MEM}'. "
            "Place the .mem file in the same folder as this script."
        )

    # Read the hexadecimal values from the .mem file
    pixel_values = []

    with open(mem_path, "r", encoding="ascii") as mem_file:
        for line_number, line in enumerate(mem_file, start=1):
            value = line.strip()

            # Ignore blank lines
            if not value:
                continue

            try:
                pixel = int(value, 16)
            except ValueError:
                raise ValueError(
                    f"Invalid hexadecimal value on line {line_number}: "
                    f"{value!r}"
                )

            if not 0 <= pixel <= 255:
                raise ValueError(
                    f"Value on line {line_number} is outside "
                    f"the 8-bit range: {value}"
                )

            pixel_values.append(pixel)

    # Verify the number of received bytes
    if len(pixel_values) != EXPECTED_VALUES:
        raise ValueError(
            f"Incorrect number of values!\n"
            f"Expected: {EXPECTED_VALUES}\n"
            f"Found:    {len(pixel_values)}\n"
            "Check that the FPGA output contains all RGB bytes."
        )

    # Group consecutive values as R, G, B
    rgb_pixels = [
        (
            pixel_values[i],
            pixel_values[i + 1],
            pixel_values[i + 2]
        )
        for i in range(0, len(pixel_values), CHANNELS)
    ]

    # Create the RGB image
    image = Image.new("RGB", (WIDTH, HEIGHT))
    image.putdata(rgb_pixels)

    # Save the final corrected image
    image.save(OUTPUT_IMAGE)

    print("Image generated successfully!")
    print(f"Input file:  {INPUT_MEM}")
    print(f"Output file: {OUTPUT_IMAGE}")
    print(f"Image size:  {WIDTH} x {HEIGHT}")
    print(f"RGB pixels:  {len(rgb_pixels)}")


if __name__ == "__main__":
    main()
