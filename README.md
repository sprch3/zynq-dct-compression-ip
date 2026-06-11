# zynq-dct-compression-ip
AXI4-Lite custom IP core on Zynq-7000 (ZedBoard) implementing 2D 8×8 Forward DCT, zigzag scan, JPEG luminance quantization, and RLE encoding in hardware (Verilog). ARM Cortex-A9 PS drives the pipeline via memory-mapped registers. Python host script reconstructs image over UART.
