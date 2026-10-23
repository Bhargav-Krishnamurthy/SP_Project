
import serial

PORT = "COM5"
BAUD_RATE = 115200
TOTAL_BYTES = 256 * 256 * 3
OUTPUT_FILE = "gamma_corrected_rgb.mem"


def main():
    received = 0

    with serial.Serial(PORT, BAUD_RATE, timeout=10) as ser:
        input(
            "Start the FPGA processing, then press Enter here "
            "to receive the output..."
        )

        with open(OUTPUT_FILE, "w", encoding="ascii") as output:
            while received < TOTAL_BYTES:
                chunk = ser.read(min(4096, TOTAL_BYTES - received))

                if not chunk:
                    raise TimeoutError(
                        f"Timed out after receiving {received} "
                        f"of {TOTAL_BYTES} bytes."
                    )

                output.writelines(f"{value:02X}\n" for value in chunk)
                received += len(chunk)

                print(
                    f"\rReceived {received}/{TOTAL_BYTES} bytes",
                    end="",
                    flush=True,
                )

    print(f"\nSaved corrected image to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()

