import serial, sys, numpy as np
from PIL import Image

PORT = sys.argv[1]            # e.g. COM5 or /dev/ttyUSB1
BAUD = 115200                 # must match .UART_BAUD in the Verilog
N    = 256*256*3

ser = serial.Serial(PORT, BAUD, timeout=60)
ser.reset_input_buffer()
print("Port open. Now press BTN0 (start) on the board...")

data = ser.read(N)            # waits until N bytes arrive (or 60 s timeout)
print("received", len(data), "of", N, "bytes")
if len(data) == N:
    img = np.frombuffer(data, dtype=np.uint8).reshape(256, 256, 3)
    np.save("fpga_out.npy", img)
    Image.fromarray(img, "RGB").save("fpga_out.png")
    print("saved fpga_out.png and fpga_out.npy")
