with open("reciprocal.mem", "w") as f:
    for den in range(256):
        if den == 0:
            val = 0  # avoid divide by zero
        else:
            # Matches Verilog: (255 << 8) / den
            val = min((255 << 8) // den, 255 << 8)

        f.write(f"{val:04X}\n")
