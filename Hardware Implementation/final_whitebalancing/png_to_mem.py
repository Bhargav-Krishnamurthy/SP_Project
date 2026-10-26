
from PIL import Image

# Change this to your input PNG filename
INPUT_IMAGE = "input.png"
OUTPUT_MEM = "image.mem"

WIDTH = 256
HEIGHT = 256

# Force the image to exactly 256x256 and RGB
image = Image.open(INPUT_IMAGE).convert("RGB")
image = image.resize((WIDTH, HEIGHT))

with open(OUTPUT_MEM, "w") as f:
    for y in range(HEIGHT):
        for x in range(WIDTH):
            r, g, b = image.getpixel((x, y))
            pixel = (r << 16) | (g << 8) | b
            f.write(f"{pixel:06X}\n")

print(f"Generated {OUTPUT_MEM}")
print(f"Pixels: {WIDTH * HEIGHT}")
print("Format: 24-bit RGB, one pixel per line")
