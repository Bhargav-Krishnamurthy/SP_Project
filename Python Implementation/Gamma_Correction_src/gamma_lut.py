
gamma = 0.8

with open("gamma_lut.mem", "w") as f:
    for p in range(0,256,1):
        normalized = p / 255.0
        corrected = normalized ** gamma
        output_pixel = round(255 * corrected)

        f.write(f"{output_pixel:06X}\n")
