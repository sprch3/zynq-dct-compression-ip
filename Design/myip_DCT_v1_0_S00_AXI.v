`timescale 1 ns / 1 ps

// =============================================================================
// Module  : myip_DCT_v1_0_S00_AXI
//
// Register map (32-bit, byte-addressed)
//   0x00  slv_reg0  [0]=start pulse  [1]=pixel_write_en
//   0x04  slv_reg1  [6:0]=index (pixel write: 0-63; rle read: 0-64)
//   0x08  slv_reg2  [15:0]=signed 16-bit pixel value
//   0x0C  (read)    {29'b0, state[2:0]}   0=IDLE 1=STAGE1 2=STAGE2 3=QZ_RLE 4=DONE
//   0x10  (read)    {10'b0, rle_mem[slv_reg1[6:0]]}  = {10'b0, run[5:0], val[15:0]}
//   0x14  (read)    {25'b0, rle_count[6:0]}
// =============================================================================

module myip_DCT_v1_0_S00_AXI #
(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 6
)
(
    input  wire                               S_AXI_ACLK,
    input  wire                               S_AXI_ARESETN,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     S_AXI_AWADDR,
    input  wire [2:0]                         S_AXI_AWPROT,
    input  wire                               S_AXI_AWVALID,
    output wire                               S_AXI_AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]     S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire                               S_AXI_WVALID,
    output wire                               S_AXI_WREADY,
    output wire [1:0]                         S_AXI_BRESP,
    output wire                               S_AXI_BVALID,
    input  wire                               S_AXI_BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     S_AXI_ARADDR,
    input  wire [2:0]                         S_AXI_ARPROT,
    input  wire                               S_AXI_ARVALID,
    output wire                               S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0]     S_AXI_RDATA,
    output wire [1:0]                         S_AXI_RRESP,
    output wire                               S_AXI_RVALID,
    input  wire                               S_AXI_RREADY
);

// ---------------------------------------------------------------------------
// AXI4-Lite internal handshake registers
// ---------------------------------------------------------------------------
reg [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr;
reg                            axi_awready;
reg                            axi_wready;
reg [1:0]                      axi_bresp;
reg                            axi_bvalid;
reg [C_S_AXI_ADDR_WIDTH-1:0]  axi_araddr;
reg                            axi_arready;
reg [C_S_AXI_DATA_WIDTH-1:0]  axi_rdata;
reg [1:0]                      axi_rresp;
reg                            axi_rvalid;

localparam integer ADDR_LSB          = 2;  
localparam integer OPT_MEM_ADDR_BITS = 3;  


reg [31:0] slv_reg0;  // [0]=start  [1]=pixel_write_en
reg [31:0] slv_reg1;  // [6:0]=index
reg [31:0] slv_reg2;  // [15:0]=pixel value (signed)
reg [31:0] slv_reg3;  

wire        slv_reg_rden;
wire        slv_reg_wren;
reg  [31:0] reg_data_out;
integer     byte_index;
reg         aw_en;

// ---------------------------------------------------------------------------
// DCT coefficient matrix 
// ---------------------------------------------------------------------------
localparam signed [15:0] C00= 16'sd64,  C01= 16'sd64,  C02= 16'sd64,  C03= 16'sd64;
localparam signed [15:0] C04= 16'sd64,  C05= 16'sd64,  C06= 16'sd64,  C07= 16'sd64;
localparam signed [15:0] C10= 16'sd89,  C11= 16'sd75,  C12= 16'sd50,  C13= 16'sd18;
localparam signed [15:0] C14=-16'sd18,  C15=-16'sd50,  C16=-16'sd75,  C17=-16'sd89;
localparam signed [15:0] C20= 16'sd83,  C21= 16'sd35,  C22=-16'sd35,  C23=-16'sd83;
localparam signed [15:0] C24=-16'sd83,  C25=-16'sd35,  C26= 16'sd35,  C27= 16'sd83;
localparam signed [15:0] C30= 16'sd75,  C31=-16'sd18,  C32=-16'sd89,  C33=-16'sd50;
localparam signed [15:0] C34= 16'sd50,  C35= 16'sd89,  C36= 16'sd18,  C37=-16'sd75;
localparam signed [15:0] C40= 16'sd64,  C41=-16'sd64,  C42=-16'sd64,  C43= 16'sd64;
localparam signed [15:0] C44= 16'sd64,  C45=-16'sd64,  C46=-16'sd64,  C47= 16'sd64;
localparam signed [15:0] C50= 16'sd50,  C51=-16'sd89,  C52= 16'sd18,  C53= 16'sd75;
localparam signed [15:0] C54=-16'sd75,  C55=-16'sd18,  C56= 16'sd89,  C57=-16'sd50;
localparam signed [15:0] C60= 16'sd35,  C61=-16'sd83,  C62= 16'sd83,  C63=-16'sd35;
localparam signed [15:0] C64=-16'sd35,  C65= 16'sd83,  C66=-16'sd83,  C67= 16'sd35;
localparam signed [15:0] C70= 16'sd18,  C71=-16'sd50,  C72= 16'sd75,  C73=-16'sd89;
localparam signed [15:0] C74= 16'sd89,  C75=-16'sd75,  C76= 16'sd50,  C77=-16'sd18;

function signed [15:0] coeff;
    input [2:0] r;
    input [2:0] c;
    case ({r, c})
        6'h00: coeff=C00; 6'h01: coeff=C01; 6'h02: coeff=C02; 6'h03: coeff=C03;
        6'h04: coeff=C04; 6'h05: coeff=C05; 6'h06: coeff=C06; 6'h07: coeff=C07;
        6'h08: coeff=C10; 6'h09: coeff=C11; 6'h0A: coeff=C12; 6'h0B: coeff=C13;
        6'h0C: coeff=C14; 6'h0D: coeff=C15; 6'h0E: coeff=C16; 6'h0F: coeff=C17;
        6'h10: coeff=C20; 6'h11: coeff=C21; 6'h12: coeff=C22; 6'h13: coeff=C23;
        6'h14: coeff=C24; 6'h15: coeff=C25; 6'h16: coeff=C26; 6'h17: coeff=C27;
        6'h18: coeff=C30; 6'h19: coeff=C31; 6'h1A: coeff=C32; 6'h1B: coeff=C33;
        6'h1C: coeff=C34; 6'h1D: coeff=C35; 6'h1E: coeff=C36; 6'h1F: coeff=C37;
        6'h20: coeff=C40; 6'h21: coeff=C41; 6'h22: coeff=C42; 6'h23: coeff=C43;
        6'h24: coeff=C44; 6'h25: coeff=C45; 6'h26: coeff=C46; 6'h27: coeff=C47;
        6'h28: coeff=C50; 6'h29: coeff=C51; 6'h2A: coeff=C52; 6'h2B: coeff=C53;
        6'h2C: coeff=C54; 6'h2D: coeff=C55; 6'h2E: coeff=C56; 6'h2F: coeff=C57;
        6'h30: coeff=C60; 6'h31: coeff=C61; 6'h32: coeff=C62; 6'h33: coeff=C63;
        6'h34: coeff=C64; 6'h35: coeff=C65; 6'h36: coeff=C66; 6'h37: coeff=C67;
        6'h38: coeff=C70; 6'h39: coeff=C71; 6'h3A: coeff=C72; 6'h3B: coeff=C73;
        6'h3C: coeff=C74; 6'h3D: coeff=C75; 6'h3E: coeff=C76; 6'h3F: coeff=C77;
        default: coeff=16'sd0;
    endcase
endfunction

// ---------------------------------------------------------------------------
// Memories
// ---------------------------------------------------------------------------
reg signed [15:0] pixel_mem [0:63];  // 8×8 signed 16-bit input pixels, row-major
reg signed [47:0] temp_mem  [0:63];  // 8×8 signed 48-bit Stage1 intermediates
reg signed [31:0] dct_mem   [0:63];  // 8×8 signed 32-bit DCT coefficients

reg [21:0] rle_mem  [0:64];
reg [6:0]  rle_count;

// ---------------------------------------------------------------------------
// State machine
//   0=IDLE  1=STAGE1  2=STAGE2  3=QZ_RLE  4=DONE
// ---------------------------------------------------------------------------
reg [2:0] state;

// Shared inner-loop counters reused across Stage1 and Stage2
reg [2:0] row;  // output row index (frequency row of the transform)
reg [2:0] k;    // inner-loop index

reg start_r;

reg signed [63:0] acc [0:7];

// ---------------------------------------------------------------------------
// QZ_RLE pipeline registers
// ---------------------------------------------------------------------------
reg [6:0]         qz_i;           // zigzag fetch index (0..63; 64 = done)
reg signed [15:0] qz_val_r;       // registered saturated quantised value
reg               qz_valid_r;     // high when qz_val_r is valid (1-cycle lag)
reg [6:0]         rle_i;          // RLE consumer pointer (lags qz_i by 1 cycle)
reg [5:0]         zero_count;     // accumulated AC zero-run length (0..15)
reg [6:0]         out_idx;        
reg [6:0]         last_nz_out_idx;

// ---------------------------------------------------------------------------
// Zigzag scan ROM - maps zigzag position 
// ---------------------------------------------------------------------------
function [5:0] zigzag_rom;
    input [5:0] i;
    case (i)
         6'd0: zigzag_rom= 6'd0;   6'd1: zigzag_rom= 6'd1;   6'd2: zigzag_rom= 6'd8;
         6'd3: zigzag_rom=6'd16;   6'd4: zigzag_rom= 6'd9;   6'd5: zigzag_rom= 6'd2;
         6'd6: zigzag_rom= 6'd3;   6'd7: zigzag_rom=6'd10;   6'd8: zigzag_rom=6'd17;
         6'd9: zigzag_rom=6'd24;  6'd10: zigzag_rom=6'd32;  6'd11: zigzag_rom=6'd25;
        6'd12: zigzag_rom=6'd18;  6'd13: zigzag_rom=6'd11;  6'd14: zigzag_rom= 6'd4;
        6'd15: zigzag_rom= 6'd5;  6'd16: zigzag_rom=6'd12;  6'd17: zigzag_rom=6'd19;
        6'd18: zigzag_rom=6'd26;  6'd19: zigzag_rom=6'd33;  6'd20: zigzag_rom=6'd40;
        6'd21: zigzag_rom=6'd48;  6'd22: zigzag_rom=6'd41;  6'd23: zigzag_rom=6'd34;
        6'd24: zigzag_rom=6'd27;  6'd25: zigzag_rom=6'd20;  6'd26: zigzag_rom=6'd13;
        6'd27: zigzag_rom= 6'd6;  6'd28: zigzag_rom= 6'd7;  6'd29: zigzag_rom=6'd14;
        6'd30: zigzag_rom=6'd21;  6'd31: zigzag_rom=6'd28;  6'd32: zigzag_rom=6'd35;
        6'd33: zigzag_rom=6'd42;  6'd34: zigzag_rom=6'd49;  6'd35: zigzag_rom=6'd56;
        6'd36: zigzag_rom=6'd57;  6'd37: zigzag_rom=6'd50;  6'd38: zigzag_rom=6'd43;
        6'd39: zigzag_rom=6'd36;  6'd40: zigzag_rom=6'd29;  6'd41: zigzag_rom=6'd22;
        6'd42: zigzag_rom=6'd15;  6'd43: zigzag_rom=6'd23;  6'd44: zigzag_rom=6'd30;
        6'd45: zigzag_rom=6'd37;  6'd46: zigzag_rom=6'd44;  6'd47: zigzag_rom=6'd51;
        6'd48: zigzag_rom=6'd58;  6'd49: zigzag_rom=6'd59;  6'd50: zigzag_rom=6'd52;
        6'd51: zigzag_rom=6'd45;  6'd52: zigzag_rom=6'd38;  6'd53: zigzag_rom=6'd31;
        6'd54: zigzag_rom=6'd39;  6'd55: zigzag_rom=6'd46;  6'd56: zigzag_rom=6'd53;
        6'd57: zigzag_rom=6'd60;  6'd58: zigzag_rom=6'd61;  6'd59: zigzag_rom=6'd54;
        6'd60: zigzag_rom=6'd47;  6'd61: zigzag_rom=6'd55;  6'd62: zigzag_rom=6'd62;
        6'd63: zigzag_rom=6'd63;
        default: zigzag_rom=6'd0;
    endcase
endfunction

// ---------------------------------------------------------------------------
// Reciprocal quantisation ROM
//   Index : zigzag scan position
//
//   Standard JPEG luminance Q-table at quality 50 (row-major, flat indices):
//     16  11  10  16  24  40  51  61    (row 0, flat  0.. 7)
//     12  12  14  19  26  58  60  55    (row 1, flat  8..15)
//     14  13  16  24  40  57  69  56    (row 2, flat 16..23)
//     14  17  22  29  51  87  80  62    (row 3, flat 24..31)
//     18  22  37  56  68 109 103  77    (row 4, flat 32..39)
//     24  35  55  64  81 104 113  92    (row 5, flat 40..47)
//     49  64  78  87 103 121 120 101    (row 6, flat 48..55)
//     72  92  95  98 112 100 103  99    (row 7, flat 56..63)
// ---------------------------------------------------------------------------
function [19:0] recip_rom;
    input [5:0] i;
    case (i)
        6'd0:  recip_rom=20'd65536;  
        6'd1:  recip_rom=20'd95325;  
        6'd2:  recip_rom=20'd87381;  
        6'd3:  recip_rom=20'd74898;  
        6'd4:  recip_rom=20'd87381;  
        6'd5:  recip_rom=20'd104858; 
        6'd6:  recip_rom=20'd65536; 
        6'd7:  recip_rom=20'd74898;  //  7? 10  Q= 14
        6'd8:  recip_rom=20'd80660;  //  8? 17  Q= 13
        6'd9:  recip_rom=20'd74898;  //  9? 24  Q= 14
        6'd10: recip_rom=20'd58254;  // 10? 32  Q= 18
        6'd11: recip_rom=20'd61681;  // 11? 25  Q= 17
        6'd12: recip_rom=20'd65536;  // 12? 18  Q= 16
        6'd13: recip_rom=20'd55188;  // 13? 11  Q= 19
        6'd14: recip_rom=20'd43691;  // 14?  4  Q= 24
        6'd15: recip_rom=20'd26214;  // 15?  5  Q= 40
        6'd16: recip_rom=20'd40330;  // 16? 12  Q= 26
        6'd17: recip_rom=20'd43691;  // 17? 19  Q= 24
        6'd18: recip_rom=20'd47663;  // 18? 26  Q= 22
        6'd19: recip_rom=20'd47663;  // 19? 33  Q= 22
        6'd20: recip_rom=20'd43691;  // 20? 40  Q= 24
        6'd21: recip_rom=20'd21400;  // 21? 48  Q= 49
        6'd22: recip_rom=20'd29959;  // 22? 41  Q= 35
        6'd23: recip_rom=20'd28340;  // 23? 34  Q= 37
        6'd24: recip_rom=20'd36158;  // 24? 27  Q= 29
        6'd25: recip_rom=20'd26214;  // 25? 20  Q= 40
        6'd26: recip_rom=20'd18079;  // 26? 13  Q= 58
        6'd27: recip_rom=20'd20560;  // 27?  6  Q= 51
        6'd28: recip_rom=20'd17190;  // 28?  7  Q= 61
        6'd29: recip_rom=20'd17476;  // 29? 14  Q= 60
        6'd30: recip_rom=20'd18396;  // 30? 21  Q= 57
        6'd31: recip_rom=20'd20560;  // 31? 28  Q= 51
        6'd32: recip_rom=20'd18725;  // 32? 35  Q= 56
        6'd33: recip_rom=20'd19065;  // 33? 42  Q= 55
        6'd34: recip_rom=20'd16384;  // 34? 49  Q= 64
        6'd35: recip_rom=20'd14564;  // 35? 56  Q= 72
        6'd36: recip_rom=20'd11398;  // 36? 57  Q= 92
        6'd37: recip_rom=20'd13443;  // 37? 50  Q= 78
        6'd38: recip_rom=20'd16384;  // 38? 43  Q= 64
        6'd39: recip_rom=20'd15420;  // 39? 36  Q= 68
        6'd40: recip_rom=20'd12053;  // 40? 29  Q= 87
        6'd41: recip_rom=20'd15197;  // 41? 22  Q= 69
        6'd42: recip_rom=20'd19065;  // 42? 15  Q= 55
        6'd43: recip_rom=20'd18725;  // 43? 23  Q= 56
        6'd44: recip_rom=20'd13107;  // 44? 30  Q= 80
        6'd45: recip_rom=20'd9620;   // 45? 37  Q=109
        6'd46: recip_rom=20'd12945;  // 46? 44  Q= 81
        6'd47: recip_rom=20'd12053;  // 47? 51  Q= 87
        6'd48: recip_rom=20'd11038;  // 48? 58  Q= 95
        6'd49: recip_rom=20'd10700;  // 49? 59  Q= 98
        6'd50: recip_rom=20'd10180;  // 50? 52  Q=103
        6'd51: recip_rom=20'd10082;  // 51? 45  Q=104
        6'd52: recip_rom=20'd10180;  // 52? 38  Q=103
        6'd53: recip_rom=20'd16913;  // 53? 31  Q= 62
        6'd54: recip_rom=20'd13618;  // 54? 39  Q= 77
        6'd55: recip_rom=20'd9279;   // 55? 46  Q=113
        6'd56: recip_rom=20'd8666;   // 56? 53  Q=121
        6'd57: recip_rom=20'd9362;   // 57? 60  Q=112
        6'd58: recip_rom=20'd10486;  // 58? 61  Q=100
        6'd59: recip_rom=20'd8738;   // 59? 54  Q=120
        6'd60: recip_rom=20'd11398;  // 60? 47  Q= 92
        6'd61: recip_rom=20'd10382;  // 61? 55  Q=101
        6'd62: recip_rom=20'd10180;  
        6'd63: recip_rom=20'd10592;  
        default: recip_rom=20'd0;
    endcase
endfunction

// ---------------------------------------------------------------------------
// AXI I/O wire assignments
// ---------------------------------------------------------------------------
assign S_AXI_AWREADY = axi_awready;
assign S_AXI_WREADY  = axi_wready;
assign S_AXI_BRESP   = axi_bresp;
assign S_AXI_BVALID  = axi_bvalid;
assign S_AXI_ARREADY = axi_arready;
assign S_AXI_RDATA   = axi_rdata;
assign S_AXI_RRESP   = axi_rresp;
assign S_AXI_RVALID  = axi_rvalid;


// ---------------------------------------------------------------------------
wire signed [52:0] qz_prod_w =
    $signed(dct_mem[zigzag_rom(qz_i[5:0])]) *        // 32-bit × 21-bit = 53-bit
    $signed({1'b0, recip_rom(qz_i[5:0])});            // MSB=0 : always positive

wire signed [52:0] qz_rounded_w = qz_prod_w + 53'sd524288; // + 2^19 = round-to-nearest
wire signed [52:0] qz_shifted_w = qz_rounded_w >>> 20;     // arithmetic descale

// ---------------------------------------------------------------------------
// AXI Write Address channel
// ---------------------------------------------------------------------------
always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
        axi_awready <= 1'b0;
        aw_en       <= 1'b1;
    end else begin
        if (!axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
            axi_awready <= 1'b1;
            aw_en       <= 1'b0;
        end else if (S_AXI_BREADY && axi_bvalid) begin
            aw_en       <= 1'b1;
            axi_awready <= 1'b0;
        end else begin
            axi_awready <= 1'b0;
        end
    end
end

always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN)
        axi_awaddr <= {C_S_AXI_ADDR_WIDTH{1'b0}};
    else if (!axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en)
        axi_awaddr <= S_AXI_AWADDR;
end

// ---------------------------------------------------------------------------
// AXI Write Data channel
// ---------------------------------------------------------------------------
always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN)
        axi_wready <= 1'b0;
    else if (!axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en)
        axi_wready <= 1'b1;
    else
        axi_wready <= 1'b0;
end

assign slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
        slv_reg0 <= 32'b0;
        slv_reg1 <= 32'b0;
        slv_reg2 <= 32'b0;
        slv_reg3 <= 32'b0;
    end else if (slv_reg_wren) begin
        case (axi_awaddr[ADDR_LSB + OPT_MEM_ADDR_BITS : ADDR_LSB])
            4'h0: for (byte_index=0; byte_index<4; byte_index=byte_index+1)
                      if (S_AXI_WSTRB[byte_index])
                          slv_reg0[byte_index*8 +: 8] <= S_AXI_WDATA[byte_index*8 +: 8];
            4'h1: for (byte_index=0; byte_index<4; byte_index=byte_index+1)
                      if (S_AXI_WSTRB[byte_index])
                          slv_reg1[byte_index*8 +: 8] <= S_AXI_WDATA[byte_index*8 +: 8];
            4'h2: for (byte_index=0; byte_index<4; byte_index=byte_index+1)
                      if (S_AXI_WSTRB[byte_index])
                          slv_reg2[byte_index*8 +: 8] <= S_AXI_WDATA[byte_index*8 +: 8];
            4'h3: for (byte_index=0; byte_index<4; byte_index=byte_index+1)
                      if (S_AXI_WSTRB[byte_index])
                          slv_reg3[byte_index*8 +: 8] <= S_AXI_WDATA[byte_index*8 +: 8];
            default: ;
        endcase
    end
end

// ---------------------------------------------------------------------------
// AXI Write Response channel
// ---------------------------------------------------------------------------
always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
        axi_bvalid <= 1'b0;
        axi_bresp  <= 2'b0;
    end else if (axi_awready && S_AXI_AWVALID && !axi_bvalid &&
                 axi_wready  && S_AXI_WVALID) begin
        axi_bvalid <= 1'b1;
        axi_bresp  <= 2'b00;  // OKAY
    end else if (S_AXI_BREADY && axi_bvalid) begin
        axi_bvalid <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// AXI Read Address channel
// ---------------------------------------------------------------------------
always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
        axi_arready <= 1'b0;
        axi_araddr  <= {C_S_AXI_ADDR_WIDTH{1'b0}};
    end else if (!axi_arready && S_AXI_ARVALID) begin
        axi_arready <= 1'b1;
        axi_araddr  <= S_AXI_ARADDR;
    end else begin
        axi_arready <= 1'b0;
    end
end

always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
        axi_rvalid <= 1'b0;
        axi_rresp  <= 2'b0;
    end else if (axi_arready && S_AXI_ARVALID && !axi_rvalid) begin
        axi_rvalid <= 1'b1;
        axi_rresp  <= 2'b00;
    end else if (axi_rvalid && S_AXI_RREADY) begin
        axi_rvalid <= 1'b0;
    end
end

assign slv_reg_rden = axi_arready & S_AXI_ARVALID & ~axi_rvalid;

// ---------------------------------------------------------------------------
// Read decode (combinational)
//   0x00  slv_reg0 mirror
//   0x04  slv_reg1 mirror
//   0x08  slv_reg2 mirror
//   0x0C  {29'b0, state[2:0]}
//   0x10  {10'b0, rle_mem[slv_reg1[6:0]]}
//   0x14  {25'b0, rle_count[6:0]}
// ---------------------------------------------------------------------------
always @(*) begin
    case (axi_araddr[ADDR_LSB + OPT_MEM_ADDR_BITS : ADDR_LSB])
        4'h0:    reg_data_out = slv_reg0;
        4'h1:    reg_data_out = slv_reg1;
        4'h2:    reg_data_out = slv_reg2;
        4'h3:    reg_data_out = {29'b0, state};
        4'h4:    reg_data_out = {10'b0, rle_mem[slv_reg1[6:0]]};
        4'h5:    reg_data_out = {25'b0, rle_count};
        default: reg_data_out = 32'b0;
    endcase
end

always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN)
        axi_rdata <= 32'b0;
    else if (slv_reg_rden)
        axi_rdata <= reg_data_out;
end

// ===========================================================================
// DCT + QZ_RLE engine
// ===========================================================================
always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN) begin
        state           <= 3'd0;
        row             <= 3'd0;
        k               <= 3'd0;
        start_r         <= 1'b0;
        acc[0]<=64'sd0; acc[1]<=64'sd0; acc[2]<=64'sd0; acc[3]<=64'sd0;
        acc[4]<=64'sd0; acc[5]<=64'sd0; acc[6]<=64'sd0; acc[7]<=64'sd0;
        qz_i            <= 7'd0;
        qz_val_r        <= 16'sd0;
        qz_valid_r      <= 1'b0;
        rle_i           <= 7'd0;
        zero_count      <= 6'd0;
        out_idx         <= 7'd0;
        last_nz_out_idx <= 7'd0;  
        rle_count       <= 7'd0;
    end else begin
        start_r <= slv_reg0[0];

        // ---------------------------------------------------------------
        // Pixel write - only accepted in IDLE.
        // slv_reg1[5:0] = flat row-major index 0..63
        // slv_reg2[15:0] = signed pixel value
        // ---------------------------------------------------------------
        if (slv_reg0[1] && (state == 3'd0))
            pixel_mem[slv_reg1[5:0]] <= $signed(slv_reg2[15:0]);

        case (state)

        // ===================================================================
        // STATE 0 - IDLE
        // Wait for a rising edge on start.  Edge detection prevents
        // re-triggering while PS holds start high during the DONE state.
        // ===================================================================
        3'd0: begin
            if (slv_reg0[0] && !start_r) begin
                state  <= 3'd1;
                row    <= 3'd0;
                k      <= 3'd0;
                acc[0]<=64'sd0; acc[1]<=64'sd0; acc[2]<=64'sd0; acc[3]<=64'sd0;
                acc[4]<=64'sd0; acc[5]<=64'sd0; acc[6]<=64'sd0; acc[7]<=64'sd0;
            end
        end

        // ===================================================================
        // STATE 1 - STAGE1 : row-wise 1-D DCT
        //   Eight output columns computed in parallel via acc[0..7].
        //   k = 0..6 : accumulate partial sums into acc[].
        //   k = 7    : add final term, write temp_mem row, reset acc[].
        //   pixel_mem layout: row-major, pixel[k][c] = pixel_mem[k*8+c]
        // ===================================================================
        3'd1: begin
            if (k == 3'd7) begin
                // Final partial product + write completed temp row
                temp_mem[row*8+0] <= acc[0] + ($signed(coeff(row,3'd7)) * $signed(pixel_mem[7*8+0]));
                temp_mem[row*8+1] <= acc[1] + ($signed(coeff(row,3'd7)) * $signed(pixel_mem[7*8+1]));
                temp_mem[row*8+2] <= acc[2] + ($signed(coeff(row,3'd7)) * $signed(pixel_mem[7*8+2]));
                temp_mem[row*8+3] <= acc[3] + ($signed(coeff(row,3'd7)) * $signed(pixel_mem[7*8+3]));
                temp_mem[row*8+4] <= acc[4] + ($signed(coeff(row,3'd7)) * $signed(pixel_mem[7*8+4]));
                temp_mem[row*8+5] <= acc[5] + ($signed(coeff(row,3'd7)) * $signed(pixel_mem[7*8+5]));
                temp_mem[row*8+6] <= acc[6] + ($signed(coeff(row,3'd7)) * $signed(pixel_mem[7*8+6]));
                temp_mem[row*8+7] <= acc[7] + ($signed(coeff(row,3'd7)) * $signed(pixel_mem[7*8+7]));

                // Reset accumulators for the next row
                acc[0]<=64'sd0; acc[1]<=64'sd0; acc[2]<=64'sd0; acc[3]<=64'sd0;
                acc[4]<=64'sd0; acc[5]<=64'sd0; acc[6]<=64'sd0; acc[7]<=64'sd0;
                k <= 3'd0;

                if (row == 3'd7) begin
                    row   <= 3'd0;  // reset for Stage2
                    state <= 3'd2;
                end else begin
                    row <= row + 3'd1;
                end

            end else begin
                // Accumulate: acc[c] += C[row][k] · pixel[k][c]
                acc[0] <= acc[0] + ($signed(coeff(row,k)) * $signed(pixel_mem[k*8+0]));
                acc[1] <= acc[1] + ($signed(coeff(row,k)) * $signed(pixel_mem[k*8+1]));
                acc[2] <= acc[2] + ($signed(coeff(row,k)) * $signed(pixel_mem[k*8+2]));
                acc[3] <= acc[3] + ($signed(coeff(row,k)) * $signed(pixel_mem[k*8+3]));
                acc[4] <= acc[4] + ($signed(coeff(row,k)) * $signed(pixel_mem[k*8+4]));
                acc[5] <= acc[5] + ($signed(coeff(row,k)) * $signed(pixel_mem[k*8+5]));
                acc[6] <= acc[6] + ($signed(coeff(row,k)) * $signed(pixel_mem[k*8+6]));
                acc[7] <= acc[7] + ($signed(coeff(row,k)) * $signed(pixel_mem[k*8+7]));
                k <= k + 3'd1;
            end
        end

        // ===================================================================
        // STATE 2 - STAGE2 : column-wise 1-D DCT with 2^14 descale
        //   Combined result: dct = C · pixel · C^T
        //   acc[c] accumulates the dot product for output frequency column c.
        //   $signed() on temp_mem is explicit to prevent unsigned inference.
        // ===================================================================
        3'd2: begin
            if (k == 3'd7) begin
                // Final partial product + descale + write completed dct row
                dct_mem[row*8+0] <= (acc[0] + ($signed(temp_mem[row*8+7]) * $signed(coeff(3'd0,3'd7)))) >>> 14;
                dct_mem[row*8+1] <= (acc[1] + ($signed(temp_mem[row*8+7]) * $signed(coeff(3'd1,3'd7)))) >>> 14;
                dct_mem[row*8+2] <= (acc[2] + ($signed(temp_mem[row*8+7]) * $signed(coeff(3'd2,3'd7)))) >>> 14;
                dct_mem[row*8+3] <= (acc[3] + ($signed(temp_mem[row*8+7]) * $signed(coeff(3'd3,3'd7)))) >>> 14;
                dct_mem[row*8+4] <= (acc[4] + ($signed(temp_mem[row*8+7]) * $signed(coeff(3'd4,3'd7)))) >>> 14;
                dct_mem[row*8+5] <= (acc[5] + ($signed(temp_mem[row*8+7]) * $signed(coeff(3'd5,3'd7)))) >>> 14;
                dct_mem[row*8+6] <= (acc[6] + ($signed(temp_mem[row*8+7]) * $signed(coeff(3'd6,3'd7)))) >>> 14;
                dct_mem[row*8+7] <= (acc[7] + ($signed(temp_mem[row*8+7]) * $signed(coeff(3'd7,3'd7)))) >>> 14;

                // Reset accumulators for the next row
                acc[0]<=64'sd0; acc[1]<=64'sd0; acc[2]<=64'sd0; acc[3]<=64'sd0;
                acc[4]<=64'sd0; acc[5]<=64'sd0; acc[6]<=64'sd0; acc[7]<=64'sd0;
                k <= 3'd0;

                if (row == 3'd7) begin
                    // All 64 DCT coefficients are ready - start QZ_RLE
                    state           <= 3'd3;
                    qz_i            <= 7'd0;
                    rle_i           <= 7'd0;
                    qz_valid_r      <= 1'b0;
                    zero_count      <= 6'd0;
                    out_idx         <= 7'd0;
                    last_nz_out_idx <= 7'd0;  // reset before RLE pass
                    rle_count       <= 7'd0;
                end else begin
                    row <= row + 3'd1;
                end

            end else begin
                // Accumulate: acc[c] += temp[row][k] · C[c][k]
                acc[0] <= acc[0] + ($signed(temp_mem[row*8+k]) * $signed(coeff(3'd0,k)));
                acc[1] <= acc[1] + ($signed(temp_mem[row*8+k]) * $signed(coeff(3'd1,k)));
                acc[2] <= acc[2] + ($signed(temp_mem[row*8+k]) * $signed(coeff(3'd2,k)));
                acc[3] <= acc[3] + ($signed(temp_mem[row*8+k]) * $signed(coeff(3'd3,k)));
                acc[4] <= acc[4] + ($signed(temp_mem[row*8+k]) * $signed(coeff(3'd4,k)));
                acc[5] <= acc[5] + ($signed(temp_mem[row*8+k]) * $signed(coeff(3'd5,k)));
                acc[6] <= acc[6] + ($signed(temp_mem[row*8+k]) * $signed(coeff(3'd6,k)));
                acc[7] <= acc[7] + ($signed(temp_mem[row*8+k]) * $signed(coeff(3'd7,k)));
                k <= k + 3'd1;
            end
        end

        // ===================================================================
        // STATE 3 - QZ_RLE : zigzag quantise + JPEG RLE encode
        // Two-stage pipeline with 1-cycle latency between fetch and consume:
        // ===================================================================
        3'd3: begin

            // ---- QZ FETCH -----------------------------------------------
            // Register only the final result.
            if (qz_i <= 7'd63) begin
                // Saturate to signed 16-bit and register
                if      (qz_shifted_w >  53'sd32767) qz_val_r <= 16'sd32767;
                else if (qz_shifted_w < -53'sd32768) qz_val_r <= -16'sd32768;
                else                                  qz_val_r <= qz_shifted_w[15:0];

                qz_valid_r <= 1'b1;
                qz_i       <= qz_i + 7'd1;
            end else begin
                qz_valid_r <= 1'b0;  // all 64 coefficients have been fetched
            end

            // ---- RLE CONSUME --------------------------------------------
            if (qz_valid_r) begin

                if (rle_i == 7'd0) begin
                    // --- DC coefficient: always emitted regardless of value ---
                    rle_mem[out_idx] <= {6'd0, qz_val_r};
                    out_idx          <= out_idx + 7'd1;
                    // DC is always kept
                    last_nz_out_idx  <= out_idx + 7'd1;
                    rle_i            <= rle_i  + 7'd1;

                end else if (rle_i <= 7'd63) begin
                    if (qz_val_r == 16'sd0) begin
                        // --- AC zero ---
                        if (zero_count == 6'd15) begin
                            rle_mem[out_idx] <= {6'd15, 16'd0};  // ZRL = (15,0)
                            out_idx          <= out_idx + 7'd1;
                            zero_count       <= 6'd0;
                            // last_nz_out_idx intentionally NOT updated here
                        end else begin
                            zero_count <= zero_count + 6'd1;
                        end
                    end else begin
                        // --- AC non-zero ---
                        // Flush accumulated zero run + value, then commit.
                        rle_mem[out_idx] <= {zero_count, qz_val_r};
                        out_idx          <= out_idx + 7'd1;
                        last_nz_out_idx  <= out_idx + 7'd1;
                        zero_count       <= 6'd0;
                    end
                    rle_i <= rle_i + 7'd1;
                end

            end // qz_valid_r

            // ---- FINALISATION -------------------------------------------
            if (rle_i == 7'd64) begin
                rle_mem[last_nz_out_idx] <= {6'd0, 16'd0};   // EOB = (0,0)
                rle_count                <= last_nz_out_idx + 7'd1; // includes EOB
                state                    <= 3'd4;
            end
        end // STATE 3

        // ===================================================================
        // STATE 4 - DONE
        //   rle_mem is stable; rle_count is valid.  Wait for PS to deassert
        //   start before returning to IDLE to prevent spurious re-trigger.
        // ===================================================================
        3'd4: begin
            if (!slv_reg0[0])
                state <= 3'd0;
        end

        default: state <= 3'd0;

        endcase
    end
end
endmodule