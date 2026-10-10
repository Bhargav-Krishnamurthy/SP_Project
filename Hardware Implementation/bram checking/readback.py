
from PIL import Image

WIDTH = 256
HEIGHT = 256

pixels = []

with open("readback.mem", "r") as f:
    for line in f:
        line = line.strip()
        if line:
            pixels.append(int(line, 16))

assert len(pixels) == WIDTH * HEIGHT, (
    f"Expected {WIDTH * HEIGHT} pixels, got {len(pixels)}"
)

image = Image.new("RGB", (WIDTH, HEIGHT))

for i, pixel in enumerate(pixels):
    r = (pixel >> 16) & 0xFF
    g = (pixel >> 8) & 0xFF
    b = pixel & 0xFF

    x = i % WIDTH
    y = i // WIDTH

    image.putpixel((x, y), (r, g, b))

image.save("output.png")

print("Saved output.png")
