================================================================================
  README — FPGA-Based DCT Image Compression IP Core
  Project  : project_1
  Board    : ZedBoard (xc7z020clg484-1)
  Tool     : Vivado 2022.2
================================================================================

--------------------------------------------------------------------------------
PROJECT OVERVIEW
--------------------------------------------------------------------------------
This project implements an AXI4-Lite custom IP core (myip_DCT) on the ZYNQ7
Processing System that performs:
  1. 2-D 8x8 Forward DCT (two-pass: row-wise then column-wise)
  2. Zigzag scan
  3. JPEG-standard luminance quantisation
  4. Run-Length Encoding (RLE) with End-of-Block (EOB) marker

The PS (ARM Cortex-A9) feeds 8x8 pixel blocks to the IP via AXI registers,
triggers computation, and reads back RLE-encoded results. A Python script on
the host PC receives the compressed data over UART and reconstructs the image.

--------------------------------------------------------------------------------
ZIP FILE CONTENTS
--------------------------------------------------------------------------------

submission.zip
  |
  |-- project_1/
  |     |-- Project.tcl            (Vivado project recreation script)
  |     |-- Block_Design.tcl       (Block design recreation script)
  |     |-- project_1.srcs/
  |           |-- utils_1/imports/synth_1/
  |                 |-- design_1_wrapper.dcp   (synthesised design checkpoint)
  |
  |-- ip_repo/
  |     |-- myip_DCT_1_0/          (custom AXI IP source files)
  |
  |-- Report.pdf
  |-- README.txt                   (this file)
  |-- sample_input.png             (128x128 greyscale test image used in demo)

--------------------------------------------------------------------------------
SYSTEM REQUIREMENTS
--------------------------------------------------------------------------------
  - Xilinx Vivado 2022.2  (MUST use this exact version)
  - Vitis 2022.2 (for building and running the C application)
  - Python 3.x with packages: numpy, opencv-python, pyserial, matplotlib
      Install via:  pip install numpy opencv-python pyserial matplotlib
  - ZedBoard connected via USB-UART (check COM port in Device Manager)

--------------------------------------------------------------------------------
STEP 1 — RECREATE THE VIVADO PROJECT
--------------------------------------------------------------------------------

1. Extract the zip file. Maintain the folder structure exactly as provided.
   The ip_repo/ folder MUST be one level above project_1/ like so:

       <any_location>/
           project_1/
               Project.tcl
               Block_Design.tcl
               project_1.srcs/...
           ip_repo/
               myip_DCT_1_0/

2. Open Vivado 2022.2.

3. In the Tcl Console (bottom panel), navigate to the project_1 folder:
       cd <path_to_extracted_folder>/project_1

4. Source the project TCL script:
       source Project.tcl

5. Vivado will recreate the entire project including the block design,
   IP references, and synthesis/implementation run configurations.
   Wait for "INFO: Project created: project_1" in the Tcl Console.

6. Open the Block Design:
       In Flow Navigator -> IP INTEGRATOR -> Open Block Design

   You should see:
       - ZYNQ7 Processing System (processing_system7_0)
       - AXI Interconnect (ps7_0_axi_periph)
       - Processor System Reset (rst_ps7_0_50M)
       - myip_DCT_0 (custom DCT IP, myip_DCT v1.0)

--------------------------------------------------------------------------------
STEP 2 — VERIFY THE IP REPOSITORY
--------------------------------------------------------------------------------

1. In Vivado, go to:
       Tools -> Settings -> IP -> Repository

2. Confirm that the ip_repo/myip_DCT_1_0 path is listed.
   If not, click the "+" button and add:
       <path_to_extracted_folder>/ip_repo/myip_DCT_1_0

3. Click "Refresh All" then "OK".

4. Re-open the Block Design and confirm myip_DCT_0 shows no errors.

--------------------------------------------------------------------------------
STEP 3 — GENERATE BITSTREAM
--------------------------------------------------------------------------------

1. In Flow Navigator, click "Generate Bitstream".
   (This runs Synthesis -> Implementation -> Bitstream generation automatically.)

2. When complete, click "Open Implemented Design" if prompted.

3. Export Hardware (includes bitstream):
       File -> Export -> Export Hardware
       Check "Include Bitstream"
       Click OK

--------------------------------------------------------------------------------
STEP 4 — BUILD AND RUN THE C APPLICATION IN VITIS
--------------------------------------------------------------------------------

1. Launch Vitis 2022.2.

2. Create a new Application Project:
       File -> New -> Application Project
       Select the exported .xsa hardware file from Step 3.
       Choose "Empty Application (C)" template.

3. Add the following source files from the Design/ folder to the src/ directory:
       - main.c
       - image_data.h   (contains the 128x128 image pixel blocks)

4. Verify the DCT base address matches xparameters.h:
       In main.c: #define DCT_BASE  0x43C00000
       In xparameters.h: confirm XPAR_MYIP_DCT_0_S00_AXI_BASEADDR = 0x43C00000
       Update main.c if different.

5. Build the project (Ctrl+B or Project -> Build All).

6. Connect the ZedBoard via USB and program the FPGA:
       Xilinx -> Program Device -> Select the .bit file -> Program

7. Run the application:
       Right-click project -> Run As -> Launch on Hardware (GDB)

8. Open a serial terminal (e.g., Vitis Serial Terminal or PuTTY):
       Port   : your COM port (e.g., COM3 or /dev/ttyUSB1)
       Baud   : 115200
       Data   : 8N1

   You should see "START" printed, followed by block data lines.

--------------------------------------------------------------------------------
STEP 5 — RECEIVE DATA AND RECONSTRUCT IMAGE (Python)
--------------------------------------------------------------------------------

1. Before running the C application (or while it is running), open receive.py
   and set the correct COM port:
       PORT = "COM9"    <-- change to your actual port

2. Run the Python script:
       python receive.py

3. The script will:
       - Receive QTABLE, ZIGZAG order, and all RLE-encoded blocks over UART
       - Save compressed data to: compressed_image.bin
       - Perform IDCT and dequantisation
       - Save the reconstructed image to: reconstructed.png
       - Display the reconstructed image using matplotlib

4. Compare reconstructed.png with the original sample_input.png to verify.

--------------------------------------------------------------------------------
IP CORE REGISTER MAP (for reference)
--------------------------------------------------------------------------------

  Offset  Access  Description
  ------  ------  --------------------------------------------------
  0x00    R/W     Control: [0]=start pulse, [1]=pixel_write_en
  0x04    R/W     Index:   [6:0] pixel write index (0-63) / RLE read index
  0x08    R/W     Pixel:   [15:0] signed 16-bit input pixel value
  0x0C    R       State:   [2:0]  0=IDLE, 1=STAGE1, 2=STAGE2, 3=QZ_RLE, 4=DONE
  0x10    R       RLE data: [21:16]=run length, [15:0]=signed value
  0x14    R       RLE count: [6:0] total entries including EOB

--------------------------------------------------------------------------------
HARDWARE PIPELINE SUMMARY
--------------------------------------------------------------------------------

  IDLE -> STAGE1 (row-wise DCT, 64 cycles)
       -> STAGE2 (column-wise DCT + >>14 descale, 64 cycles)
       -> QZ_RLE (zigzag + quantise + RLE encode, ~66 cycles)
       -> DONE   (results stable; PS reads back RLE entries)
       -> IDLE   (after PS deasserts start)

--------------------------------------------------------------------------------
NOTES
--------------------------------------------------------------------------------

  - Use Vivado 2022.2 ONLY. Other versions may not recreate the project correctly.
  - The ip_repo folder must be at the exact relative path ../ip_repo/ from project_1/.
  - The design_1_wrapper.dcp must be present at:
        project_1/project_1.srcs/utils_1/imports/synth_1/design_1_wrapper.dcp
  - The sample image used is a 128x128 greyscale image (256 blocks of 8x8).
  - If UART shows garbled data, verify baud rate is exactly 115200.

================================================================================
