/******************************************************************************
 * myip_DCT_v1_0_S00_AXI  ·  128×128 → 256 blocks
 ******************************************************************************/

#include "xil_printf.h"
#include <xparameters.h>
#include <stdint.h>
#include <string.h>
#include "image_data.h"   /* NUM_BLOCKS, IMG_ROWS, IMG_COLS, image_blocks[][] */

/* ─────────────────────────── USER SETTINGS ──────────────────────────────── */

/* Base address: update from xparameters.h */
#define DCT_BASE            0x43C00000

#define DEMO_VERBOSE_BLOCKS   4

/* ─────────────────────────── REGISTER MAP ───────────────────────────────── */
#define REG_CTRL        0x00   /* [0]=start  [1]=pixel_write_en             */
#define REG_INDEX       0x04   /* [6:0] = write index / RLE read index      */
#define REG_PIXEL       0x08   /* [15:0] signed pixel value                 */
#define REG_STATE       0x0C   /* [2:0]  0=IDLE … 4=DONE                   */
#define REG_RLE_DATA    0x10   /* [21:16]=run  [15:0]=val (signed)          */
#define REG_RLE_COUNT   0x14   /* [6:0] total RLE entries incl. EOB         */

#define RD(off)  (*(volatile uint32_t*)(DCT_BASE + (off)))
#define WR(off,v)(*(volatile uint32_t*)(DCT_BASE + (off)) = (uint32_t)(v))

/* ─────────────────────── DCT COEFFICIENT MATRIX ────────────────────────── */
/*
 * Integer-scaled version of the type-II 8-point DCT basis:
 *   C[0][k] = 64                        (DC row, all identical)
 *   C[u][k] ≈ round(90·cos(π·u·(2k+1)/16))   u = 1…7
 * The 2-D forward transform is  DCT = C · Pixel · Cᵀ  >>  14.
 */
static const int16_t C[8][8] = {
    { 64,  64,  64,  64,  64,  64,  64,  64 },
    { 89,  75,  50,  18, -18, -50, -75, -89 },
    { 83,  35, -35, -83, -83, -35,  35,  83 },
    { 75, -18, -89, -50,  50,  89,  18, -75 },
    { 64, -64, -64,  64,  64, -64, -64,  64 },
    { 50, -89,  18,  75, -75, -18,  89, -50 },
    { 35, -83,  83, -35, -35,  83, -83,  35 },
    { 18, -50,  75, -89,  89, -75,  50, -18 }
};

/* ───────────────────────── ZIGZAG SCAN ORDER ────────────────────────────── */
/* ZZ[i] = row-major linear index (0-based) of the i-th zigzag position      */
static const uint8_t ZZ[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
};

/* ─────────────────── STANDARD JPEG LUMA QUANTISATION TABLE ─────────────── */
/* Row-major 8×8; Q_LUMA[r*8+c] is the step for DCT coefficient [r][c].      */
static const uint16_t Q_LUMA[64] = {
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68,109,103, 77,
    24, 35, 55, 64, 81,104,113, 92,
    49, 64, 78, 87,103,121,120,101,
    72, 92, 95, 98,112,100,103, 99
};

/*
 * Pre-computed reciprocal table for fast quantisation:
 *   RQ[i] = round(2^20 / Q_LUMA[ZZ[i]])
 * so  qz[i] = round(dct[ZZ[i]] * RQ[i] / 2^20)
 *           = (dct[ZZ[i]] * RQ[i] + 2^19) >> 20
 */
static const uint32_t RQ[64] = {
    65536, 95325, 87381, 74898, 87381,104858, 65536, 74898,
    80660, 74898, 58254, 61681, 65536, 55188, 43691, 26214,
    40330, 43691, 47663, 47663, 43691, 21400, 29959, 28340,
    36158, 26214, 18079, 20560, 17190, 17476, 18396, 20560,
    18725, 19065, 16384, 14564, 11398, 13443, 16384, 15420,
    12053, 15197, 19065, 18725, 13107,  9620, 12945, 12053,
    11038, 10700, 10180, 10082, 10180, 16913, 13618,  9279,
     8666,  9362, 10486,  8738, 11398, 10382, 10180, 10592
};

/* ─────────────────────────── TYPE DEFINITIONS ───────────────────────────── */
typedef struct { uint8_t run; int16_t val; } rle_t;

/* ══════════════════════════════════════════════════════════════════════════
 * SOFTWARE REFERENCE PIPELINE
 *  pix    [in]  64-element signed-pixel block (row-major)
 *  dct    [out] 64 DCT coefficients, row-major, after >>14 shift
 *  qz_zz  [out] 64 quantised values in zigzag scan order
 *  rle    [out] up to 65 RLE entries (caller must supply ≥65 elements)
 *  cnt    [out] number of valid RLE entries including EOB
 * ════════════════════════════════════════════════════════════════════════ */
static void sw_reference(
        const int16_t  pix[64],
        int32_t        dct[64],
        int16_t        qz_zz[64],
        rle_t         *rle,
        uint32_t      *cnt)
{
    /* ── Stage 1: temp[r][c] = Σ_k  C[r][k] · pix[k][c]  (row pass) ─── */
    int64_t temp[64];
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++) {
            int64_t acc = 0;
            for (int k = 0; k < 8; k++)
                acc += (int64_t)C[r][k] * pix[k * 8 + c];
            temp[r * 8 + c] = acc;
        }

    /* ── Stage 2: dct[r][c] = (Σ_k temp[r][k] · C[c][k]) >> 14  (col pass) */
    for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++) {
            int64_t acc = 0;
            for (int k = 0; k < 8; k++)
                acc += temp[r * 8 + k] * (int64_t)C[c][k];
            dct[r * 8 + c] = (int32_t)(acc >> 14);
        }

    /* ── Stage 3: quantise along zigzag scan ──────────────────────────── */
    for (int i = 0; i < 64; i++) {
        int64_t v = ((int64_t)dct[ZZ[i]] * RQ[i] + (1LL << 19)) >> 20;
        qz_zz[i] = (v >  32767) ?  32767 :
                   (v < -32768) ? -32768 : (int16_t)v;
    }

    /* ── Stage 4: RLE encode ──────────────────────────────────────────── */
    /* Find last non-zero AC coefficient (determines where to stop)        */
    int last_nz = 0;
    for (int i = 1; i < 64; i++)
        if (qz_zz[i]) last_nz = i;

    uint32_t n  = 0;
    uint8_t  zr = 0;

    /* DC coefficient always comes first, run=0 by definition */
    rle[n].run = 0;
    rle[n].val = qz_zz[0];
    n++;

    /* AC coefficients, up to the last non-zero */
    for (int i = 1; i <= last_nz; i++) {
        if (qz_zz[i] == 0) {
            if (zr == 15) {           /* ZRL marker: run of 16 zeros */
                rle[n].run = 15;
                rle[n].val = 0;
                n++; zr = 0;
            } else {
                zr++;
            }
        } else {
            rle[n].run = zr;
            rle[n].val = qz_zz[i];
            n++; zr = 0;
        }
    }

    /* End-of-Block marker */
    rle[n].run = 0;
    rle[n].val = 0;
    n++;

    *cnt = n;
}

/* ══════════════════════════════════════════════════════════════════════════
 * HARDWARE DRIVER  — write pixels, pulse start, poll DONE, read RLE back
 *
 * Returns 0 on success, -1 on timeout.
 * ════════════════════════════════════════════════════════════════════════ */
static int hw_run_block(const int16_t pix[64],
                        rle_t *rle, uint32_t *cnt)
{
    /* Load all 64 pixels into the pixel RAM via the write-enable strobe */
    for (int i = 0; i < 64; i++) {
        WR(REG_INDEX, (uint32_t)i);
        WR(REG_PIXEL, (uint32_t)(uint16_t)pix[i]);  /* sign-extend preserved */
        WR(REG_CTRL,  2u);    /* assert pixel_write_en, start=0             */
        WR(REG_CTRL,  0u);    /* deassert — creates the write pulse          */
    }

    /* Rising edge on 'start' launches the pipeline */
    WR(REG_CTRL, 1u);

    /* Poll state register until DONE (state == 4), with timeout guard */
    uint32_t timeout = 2000000u;
    while ((RD(REG_STATE) & 0x7u) != 4u) {
        if (--timeout == 0u) return -1;
    }
    WR(REG_CTRL, 0u);   /* deassert start; ready for next block */

    /* Read RLE results back from the output FIFO registers */
    *cnt = RD(REG_RLE_COUNT) & 0x7Fu;
    for (uint32_t i = 0; i < *cnt; i++) {
        WR(REG_INDEX, i);
        uint32_t raw = RD(REG_RLE_DATA);
        rle[i].run = (uint8_t)((raw >> 16) & 0x3Fu);
        rle[i].val = (int16_t)(raw & 0xFFFFu);
    }
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════════
 * PRINT HELPERS
 * ════════════════════════════════════════════════════════════════════════ */

/* Print a 64-element int16 array as an 8×8 table with a caption */
static void print_i16_8x8(const int16_t m[64], const char *caption)
{
    xil_printf("    %s\r\n", caption);
    for (int r = 0; r < 8; r++) {
        xil_printf("      ");
        for (int c = 0; c < 8; c++)
            xil_printf("%6d", (int)m[r * 8 + c]);
        xil_printf("\r\n");
    }
    xil_printf("\r\n");
}

/* Print a 64-element int32 array as an 8×8 table with a caption */
static void print_i32_8x8(const int32_t m[64], const char *caption)
{
    xil_printf("    %s\r\n", caption);
    for (int r = 0; r < 8; r++) {
        xil_printf("      ");
        for (int c = 0; c < 8; c++)
            xil_printf("%8d", (int)m[r * 8 + c]);
        xil_printf("\r\n");
    }
    xil_printf("\r\n");
}

/*
 * Print the quantised zigzag vector de-scanned back to a natural 8×8
 * layout so it aligns visually with the DCT coefficient matrix above it.
 */
static void print_qz_natural_8x8(const int16_t qz_zz[64])
{
    int16_t nat[64] = {0};
    for (int i = 0; i < 64; i++)
        nat[ZZ[i]] = qz_zz[i];
    xil_printf("    Quantised DCT (natural 8x8, from zigzag)\r\n");
    for (int r = 0; r < 8; r++) {
        xil_printf("      ");
        for (int c = 0; c < 8; c++)
            xil_printf("%6d", (int)nat[r * 8 + c]);
        xil_printf("\r\n");
    }
    xil_printf("\r\n");
}

/* Print the zigzag-order quantised vector with position labels */
static void print_qz_zigzag(const int16_t qz_zz[64])
{
    xil_printf("    Quantised coefficients (zigzag scan order):\r\n");
    for (int i = 0; i < 64; i++) {
        if (i == 0)
            xil_printf("      [ZZ%02d/DC] %5d", i, (int)qz_zz[i]);
        else
            xil_printf("      [ZZ%02d/AC] %5d", i, (int)qz_zz[i]);
        if ((i & 3) == 3) xil_printf("\r\n");
    }
    if ((63 & 3) != 3) xil_printf("\r\n");
    xil_printf("\r\n");
}

/* Print annotated RLE listing */
static void print_rle_listing(const rle_t *rle, uint32_t cnt)
{
    xil_printf("    RLE Entries (%u total):\r\n", (unsigned)cnt);
    xil_printf("      %-5s  %-5s  %-7s\r\n", "Entry", "Run", "Value");
    xil_printf("      ---------------------------------\r\n");
    for (uint32_t i = 0; i < cnt; i++) {
        int is_eob = (rle[i].run == 0 && rle[i].val == 0 && i > 0);
        int is_zrl = (rle[i].run == 15 && rle[i].val == 0);
        if (i == 0)
            xil_printf("      [DC ]   %3u   %6d\r\n",
                       (unsigned)rle[i].run, (int)rle[i].val);
        else if (is_eob)
            xil_printf("      [%3u]   EOB\r\n", (unsigned)i);
        else if (is_zrl)
            xil_printf("      [%3u]   ZRL  (run-of-16-zeros)\r\n", (unsigned)i);
        else
            xil_printf("      [%3u]  %3u   %6d\r\n",
                       (unsigned)i, (unsigned)rle[i].run, (int)rle[i].val);
    }
    xil_printf("\r\n");
}

/* ══════════════════════════════════════════════════════════════════════════
 * MAIN
 * ════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    xil_printf("START\r\n");

    /* Send image metadata */
    xil_printf("%%IMAGE_DIMS %d %d\r\n", IMG_ROWS, IMG_COLS);

    xil_printf("%%QTABLE");
    for(int i=0;i<64;i++)
        xil_printf(" %d", Q_LUMA[i]);
    xil_printf("\r\n");

    xil_printf("%%ZIGZAG");
    for(int i=0;i<64;i++)
        xil_printf(" %d", ZZ[i]);
    xil_printf("\r\n");

    /* ===================================================== */
    /* PROCESS ALL BLOCKS                                    */
    /* ===================================================== */

    for(int blk=0; blk<NUM_BLOCKS; blk++)
    {
        const int16_t *pix = image_blocks[blk];

        rle_t hw_rle[65];

        uint32_t hw_cnt = 0;

        int ret = hw_run_block(pix, hw_rle, &hw_cnt);

        if(ret != 0)
        {
            xil_printf("ERROR BLOCK %d\r\n", blk);
            continue;
        }

        /* ================================================= */
        /* SEND RLE DATA THROUGH UART                        */
        /* ================================================= */

        xil_printf("%%BLOCK %d", blk);

        for(uint32_t i=0; i<hw_cnt; i++)
        {
            xil_printf(" %d %d",
                       (int)hw_rle[i].run,
                       (int)hw_rle[i].val);
        }

        xil_printf("\r\n");
    }

    xil_printf("END\r\n");

    while(1);

    return 0;
}
