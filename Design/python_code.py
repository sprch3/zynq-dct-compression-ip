import serial
import numpy as np
import cv2
import pickle
import matplotlib.pyplot as plt

# =========================================================
# UART SETTINGS
# =========================================================

PORT = "COM9"          # CHANGE THIS
BAUD = 115200

# =========================================================
# IMAGE PARAMETERS
# =========================================================

IMG_ROWS = 128
IMG_COLS = 128
BLOCKS_PER_ROW = 16

# =========================================================
# OPEN SERIAL PORT
# =========================================================

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Waiting for FPGA stream...")

# =========================================================
# RECEIVE DATA
# =========================================================

QTABLE = []
ZZ = []
blocks = []

while True:

    try:

        line = ser.readline().decode(errors='ignore').strip()

        if len(line) == 0:
            continue

        print(line)

        # -------------------------------------------------
        # START
        # -------------------------------------------------

        if line == "START":

            print("FPGA Transmission Started")
            continue

        # -------------------------------------------------
        # IMAGE DIMS
        # -------------------------------------------------

        elif line.startswith("%IMAGE_DIMS"):

            try:

                dims = line.split()

                IMG_ROWS = int(dims[1])
                IMG_COLS = int(dims[2])

                print(f"Image Size: {IMG_ROWS} x {IMG_COLS}")

            except:

                print("Corrupted IMAGE_DIMS line")

            continue

        # -------------------------------------------------
        # QTABLE
        # -------------------------------------------------

        elif line.startswith("%QTABLE"):

            try:

                QTABLE = list(map(int, line.split()[1:]))

                print("QTABLE Received")

            except:

                print("Corrupted QTABLE line")

            continue

        # -------------------------------------------------
        # ZIGZAG
        # -------------------------------------------------

        elif line.startswith("%ZIGZAG"):

            try:

                ZZ = list(map(int, line.split()[1:]))

                print("ZIGZAG Received")

            except:

                print("Corrupted ZIGZAG line")

            continue

        # -------------------------------------------------
        # BLOCK DATA
        # -------------------------------------------------

        elif line.startswith("%BLOCK"):

            try:

                tokens = line.split()

                blk_id = int(tokens[1])

                data = list(map(int, tokens[2:]))

                blocks.append((blk_id, data))

                print(f"Received Block {blk_id}")

            except:

                print("Corrupted BLOCK line")

            continue

        # -------------------------------------------------
        # END
        # -------------------------------------------------

        elif line == "END":

            print("FPGA Transmission Complete")

            break

    except Exception as e:

        print("UART Error:", e)

# =========================================================
# CLOSE UART
# =========================================================

ser.close()

print("\nUART Reception Finished")

# =========================================================
# SAVE COMPRESSED FILE
# =========================================================

compressed_data = {
    "QTABLE": QTABLE,
    "ZZ": ZZ,
    "BLOCKS": blocks
}

with open("compressed_image.bin", "wb") as f:

    pickle.dump(compressed_data, f)

print("Compressed file saved")

# =========================================================
# PREPARE TABLES
# =========================================================

Q = np.array(QTABLE).reshape((8, 8))

Q_FLAT = Q.flatten()

# =========================================================
# HARDWARE DCT MATRIX
# =========================================================

C_hw = np.array([
    [64, 64, 64, 64, 64, 64, 64, 64],
    [89, 75, 50, 18, -18, -50, -75, -89],
    [83, 35, -35, -83, -83, -35, 35, 83],
    [75, -18, -89, -50, 50, 89, 18, -75],
    [64, -64, -64, 64, 64, -64, -64, 64],
    [50, -89, 18, 75, -75, -18, 89, -50],
    [35, -83, 83, -35, -35, 83, -83, 35],
    [18, -50, 75, -89, 89, -75, 50, -18]
], dtype=np.float64)

# =========================================================
# IMAGE BUFFER
# =========================================================

img = np.zeros((IMG_ROWS, IMG_COLS), dtype=np.uint8)

print("\nStarting Reconstruction...\n")

# =========================================================
# RECONSTRUCT IMAGE
# =========================================================

for blk_id, data in blocks:

    # -----------------------------------------------------
    # RLE DECODE
    # -----------------------------------------------------

    qz = np.zeros(64)

    pos = 0

    for i in range(0, len(data), 2):

        run = data[i]
        val = data[i + 1]

        # DC coefficient
        if i == 0:

            qz[0] = val
            pos = 1

            continue

        # End Of Block
        if run == 0 and val == 0:

            break

        pos += run

        if pos < 64:

            qz[pos] = val

            pos += 1

    # -----------------------------------------------------
    # DEQUANTIZATION
    # -----------------------------------------------------

    dct = np.zeros(64)

    for i in range(64):

        zz_pos = ZZ[i]

        dct[zz_pos] = qz[i] * Q_FLAT[zz_pos]

    # -----------------------------------------------------
    # IMPORTANT FIX
    # MATLAB USED TRANSPOSE AFTER RESHAPE
    # -----------------------------------------------------

    D = dct.reshape((8, 8)).T

    # -----------------------------------------------------
    # EXACT HARDWARE IDCT
    # -----------------------------------------------------

    P = C_hw.T @ D @ C_hw

    P = P / 65536.0

    # -----------------------------------------------------
    # ADD BACK 128 LEVEL SHIFT
    # -----------------------------------------------------

    P = P + 128

    # -----------------------------------------------------
    # ROUND + CLIP
    # -----------------------------------------------------

    P = np.round(P)

    P = np.clip(P, 0, 255)

    P = P.astype(np.uint8)

    # -----------------------------------------------------
    # PLACE BLOCK INTO IMAGE
    # -----------------------------------------------------

    br = blk_id // BLOCKS_PER_ROW
    bc = blk_id % BLOCKS_PER_ROW

    row = br * 8
    col = bc * 8

    img[row:row + 8, col:col + 8] = P

    print(f"Reconstructed Block {blk_id}")

# =========================================================
# SAVE RECONSTRUCTED IMAGE
# =========================================================

cv2.imwrite("reconstructed.png", img)

print("\nReconstructed image saved as reconstructed.png")

# =========================================================
# DISPLAY IMAGE
# =========================================================

plt.figure(figsize=(6, 6))

plt.imshow(img, cmap='gray')

plt.title("Reconstructed Image")

plt.axis('off')

plt.show()