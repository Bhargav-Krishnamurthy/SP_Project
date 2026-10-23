
from pathlib import Path
from PIL import Image

# -------- USER SETTINGS --------
INPUT_IMAGE = "white_balance_test.png"
OUTPUT_MEM = "image_rgb_channels.mem"
WIDTH = 256
HEIGHT = 256
# -------------------------------


def main():
    image_path = Path(INPUT_IMAGE)

    if not image_path.is_file():
        raise FileNotFoundError(
            f"Cannot find '{INPUT_IMAGE}'. "
            "Place it beside this script or update INPUT_IMAGE."
        )

    with Image.open(image_path) as source:
        image = source.convert("RGB")

        if image.size != (WIDTH, HEIGHT):
            image = image.resize(
                (WIDTH, HEIGHT),
                Image.Resampling.LANCZOS
            )

        with open(OUTPUT_MEM, "w", encoding="ascii") as mem_file:
            for y in range(HEIGHT):
                for x in range(WIDTH):
                    r, g, b = image.getpixel((x, y))

                    # One 8-bit intensity per line: R, G, B
                    mem_file.write(f"{r:02X}\n")
                    mem_file.write(f"{g:02X}\n")
                    mem_file.write(f"{b:02X}\n")

    print("Conversion successful!")
    print(f"Output file: {OUTPUT_MEM}")
    print(f"Image size: {WIDTH} x {HEIGHT}")
    print(f"Total entries: {WIDTH * HEIGHT * 3}")
    print("Order: R, G, B, repeated for every pixel")


if __name__ == "__main__":
    main()
