
import numpy as np
from PIL import Image

# Input and output files
input_file = "fused_output.mem"
output_file = "fused_output.png"

# Image dimensions (CHANGE THESE)
width = 256
height = 256

# Read hexadecimal pixel values
pixels = []

with open(input_file, "r") as f:
    for line in f:
        line = line.strip()

        if line:
            pixels.append(int(line, 16))

# Convert to NumPy array
image_array = np.array(pixels, dtype=np.uint8)

# Reshape into image dimensions
image_array = image_array.reshape((height, width))

# Save as PNG
image = Image.fromarray(image_array, mode="L")
image.save(output_file)

print(f"Image successfully saved as {output_file}")

