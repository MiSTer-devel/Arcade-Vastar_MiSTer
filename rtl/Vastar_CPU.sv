//============================================================================
//
//  Vastar CPU board — Phase 2: Full Video Rendering
//  Copyright (C) 2026 Rodimus
//
//  MAME reference: vastar.cpp, vastar_viddev.cpp
//  Hardware: Z80 CPU1 + Z80 CPU2 @ 3.072 MHz (XTAL 18.432 / 6)
//            AY-3-8910 @ 1.536 MHz (18.432 / 12)
//  Screen: 256x256 total, visible 256x224 (lines 16-239), 60.58 Hz
//
//============================================================================

module Vastar_CPU
(
	input         reset,
	input         clk_49m,
	output  [4:0] red, green, blue,
	output        video_hsync, video_vsync, video_csync,
	output        video_hblank, video_vblank,
	output        ce_pix,
	input   [7:0] p1_controls,
	input   [7:0] p2_controls,
	input   [7:0] sys_controls,
	input  [15:0] dip_sw,
    input         rot_flip,
	output signed [15:0] sound,
	input   [3:0] h_center, v_center,
	input         main_rom_cs_i, sub_rom_cs_i, fgtile_cs_i,
	input         sprite0_cs_i, sprite1_cs_i,
	input         bgtile0_cs_i, bgtile1_cs_i,
	input         prom_r_cs_i, prom_g_cs_i, prom_b_cs_i, prom_unk_cs_i,
	input  [24:0] ioctl_addr,
	input   [7:0] ioctl_data,
	input         ioctl_wr,
	input         pause,
	input  [15:0] hs_address,
	input   [7:0] hs_data_in,
	output  [7:0] hs_data_out,
	input         hs_write
);

//------------------------------------------------------- Clock enables -------------------------------------------------------//

// Pixel clock: 49.152 MHz * 3/32 = 4.608 MHz exactly (= 18.432 XTAL / 4)
wire [1:0] pix_cen_o;
jtframe_frac_cen #(2) pix_cen (.clk(clk_49m), .n(10'd3), .m(10'd32), .cen(pix_cen_o), .cenb());
wire cen_pix = pix_cen_o[0];
assign ce_pix = cen_pix;

reg [3:0] cpu_div = 4'd0;
always_ff @(posedge clk_49m) cpu_div <= cpu_div + 4'd1;
wire cen_cpu = (cpu_div == 4'd0);

reg ay_toggle = 1'b0;
always_ff @(posedge clk_49m) if (cen_cpu) ay_toggle <= ~ay_toggle;  // Verify cen_cpu for AY toggle
wire cen_ay = cen_cpu & ~ay_toggle & ~pause;

//-------------------------------------------------------- Video timing --------------------------------------------------------//

reg [8:0] base_h_cnt = 9'd0;
reg [8:0] v_cnt      = 9'd0;

wire [8:0] h_cnt_rot;
wire [8:0] v_cnt_rot;

// XOR-based flip
assign h_cnt_rot = { base_h_cnt[8], base_h_cnt[7:0] ^ {8{flip_screen}} };
assign v_cnt_rot = { v_cnt[8],      v_cnt[7:0]      ^ {8{flip_screen}} };

always_ff @(posedge clk_49m) begin
	if (cen_pix) begin
		if (base_h_cnt == 9'd287) begin
			base_h_cnt <= 9'd0;
			v_cnt <= (v_cnt == 9'd263) ? 9'd0 : v_cnt + 9'd1;
		end else begin
			base_h_cnt <= base_h_cnt + 9'd1;
		end
	end
end

wire hblk = (base_h_cnt >= 9'd256);
wire vblk = (v_cnt < 9'd17) | (v_cnt >= 9'd241);
assign video_hblank = hblk;
assign video_vblank = vblk;

wire [8:0] hs_start = 9'd264 + {5'd0, h_center};
wire [8:0] hs_end   = hs_start + 9'd16;
wire [8:0] vs_start = 9'd248 + {5'd0, v_center};
wire [8:0] vs_end   = vs_start + 9'd4;
assign video_hsync = (h_cnt_rot >= hs_start && h_cnt_rot < hs_end);
assign video_vsync = (v_cnt_rot >= vs_start && v_cnt_rot < vs_end);
assign video_csync = ~(video_hsync ^ video_vsync);

//------------------------------------------------------- CPU1 — Main ---------------------------------------------------------//

wire [15:0] cpu1_A;
wire [7:0]  cpu1_Dout;
wire        cpu1_WR_n, cpu1_RD_n, cpu1_MREQ_n, cpu1_IORQ_n, cpu1_M1_n, cpu1_RFSH_n;

T80s cpu1
(
	.RESET_n(reset), .CLK(clk_49m), .CEN(cen_cpu & ~pause), .WAIT_n(1'b1),
	.INT_n(1'b1), .NMI_n(~cpu1_nmi),
	.M1_n(cpu1_M1_n), .MREQ_n(cpu1_MREQ_n), .IORQ_n(cpu1_IORQ_n),
	.RD_n(cpu1_RD_n), .WR_n(cpu1_WR_n), .RFSH_n(cpu1_RFSH_n),
	.A(cpu1_A), .DI(cpu1_Din), .DO(cpu1_Dout)
);

reg cpu1_nmi = 1'b0;
reg vblk_prev = 1'b0;
always_ff @(posedge clk_49m) begin
	if (!reset) begin cpu1_nmi <= 0; vblk_prev <= 0; end
	else begin vblk_prev <= vblk; cpu1_nmi <= (vblk && !vblk_prev && nmi_mask); end
end

//-------------------------------------------------- CPU1 Address Decoding ----------------------------------------------------//

wire cpu1_mem_valid = ~cpu1_MREQ_n & cpu1_RFSH_n;
wire cs_rom      = cpu1_mem_valid & ~cpu1_A[15];
wire cs_bg1      = cpu1_mem_valid & ((cpu1_A[15:12] == 4'h8) | (cpu1_A[15:12] == 4'hA));
wire cs_bg0      = cpu1_mem_valid & ((cpu1_A[15:12] == 4'h9) | (cpu1_A[15:12] == 4'hB));
wire cs_priority = cpu1_mem_valid & (cpu1_A[15:0] == 16'hC000);
wire cs_fgvram   = cpu1_mem_valid & (cpu1_A[15:12] == 4'hC) & (cpu1_A[11:10] != 2'b00);
wire cs_watchdog = cpu1_mem_valid & (cpu1_A[15:12] == 4'hE);
wire cs_shared   = cpu1_mem_valid & (cpu1_A[15:11] == 5'b11110);
wire cs_mainlatch = ~cpu1_IORQ_n & ~cpu1_WR_n & (cpu1_A[3:0] <= 4'h7);

reg [7:0] mainlatch = 8'd0;
always_ff @(posedge clk_49m) begin
	if (!reset) mainlatch <= 8'd0;
	else if (cen_cpu && cs_mainlatch) mainlatch[cpu1_A[2:0]] <= cpu1_Dout[0];
end
wire nmi_mask    = mainlatch[0];
wire flip_screen = rot_flip ^ mainlatch[1];
wire cpu2_rst    = ~mainlatch[2];

//------------------------------------------------------- CPU2 — Sub ----------------------------------------------------------//

wire [15:0] cpu2_A;
wire [7:0]  cpu2_Dout;
wire        cpu2_WR_n, cpu2_RD_n, cpu2_MREQ_n, cpu2_IORQ_n, cpu2_M1_n, cpu2_RFSH_n;

T80pa cpu2
(
	.RESET_n(reset & ~cpu2_rst), .CLK(clk_49m),
	.CEN_p(cen_cpu & ~pause), .CEN_n(~cen_cpu & ~pause),
	.WAIT_n(1'b1),
	.INT_n(~cpu2_irq), .NMI_n(1'b1),
	.M1_n(cpu2_M1_n), .MREQ_n(cpu2_MREQ_n), .IORQ_n(cpu2_IORQ_n),
	.RD_n(cpu2_RD_n), .WR_n(cpu2_WR_n), .RFSH_n(cpu2_RFSH_n),
	.A(cpu2_A), .DI(cpu2_Din), .DO(cpu2_Dout)
);

reg [17:0] cpu2_irq_cnt = 18'd0;
reg        cpu2_irq = 1'b0;
always_ff @(posedge clk_49m) begin
	if (!reset || cpu2_rst) begin cpu2_irq_cnt <= 0; cpu2_irq <= 0; end
	else begin
		if (cpu2_irq_cnt == 18'd203107) begin cpu2_irq_cnt <= 0; cpu2_irq <= 1; end
		else cpu2_irq_cnt <= cpu2_irq_cnt + 1;
		if (~cpu2_IORQ_n & ~cpu2_M1_n) cpu2_irq <= 0;
	end
end

wire cpu2_mem_valid = ~cpu2_MREQ_n & cpu2_RFSH_n;
wire cs2_rom    = cpu2_mem_valid & ~cpu2_A[15] & ~cpu2_A[14] & ~cpu2_A[13];
wire cs2_shared = cpu2_mem_valid & (cpu2_A[15:11] == 5'b01000);
wire cs2_p2     = cpu2_mem_valid & (cpu2_A[15:0] == 16'h8000);
wire cs2_p1     = cpu2_mem_valid & (cpu2_A[15:0] == 16'h8040);
wire cs2_system = cpu2_mem_valid & (cpu2_A[15:0] == 16'h8080);
wire cs2_ay_addr = ~cpu2_IORQ_n & ~cpu2_WR_n & (cpu2_A[3:0] == 4'h0);
wire cs2_ay_wr   = ~cpu2_IORQ_n & ~cpu2_WR_n & (cpu2_A[3:0] == 4'h1);
wire cs2_ay_rd   = ~cpu2_IORQ_n & ~cpu2_RD_n & (cpu2_A[3:0] == 4'h2);

//---------------------------------------------------------- ROMs -------------------------------------------------------------//

wire [7:0] main_rom_D;
eprom_32k main_rom (.CLK(clk_49m), .ADDR(cpu1_A[14:0]), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(main_rom_cs_i), .WR(ioctl_wr), .DATA(main_rom_D));

wire [7:0] sub_rom_D;
eprom_8k sub_rom (.CLK(clk_49m), .ADDR(cpu2_A[12:0]), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(sub_rom_cs_i), .WR(ioctl_wr), .DATA(sub_rom_D));

//------------------------------------------------------- VRAM ---------------------------------------------------------------//

wire [10:0] shared_addr_a = pause ? hs_address[10:0] : cpu1_A[10:0];
wire shared_wr_a = pause ? hs_write : (cs_shared & ~cpu1_WR_n);
wire [7:0] shared_din_a = pause ? hs_data_in : cpu1_Dout;
wire [2:0] hs_bank = rot_flip ? 3'b011 : 3'b001;

wire [7:0] bg1_vram_D, bg1_vram_rD;
reg [11:0] bg1_raddr;
dpram_dc #(.widthad_a(12)) bg1_vram (
	.clock_a(clk_49m), .address_a(cpu1_A[11:0]), .data_a(cpu1_Dout),
	.wren_a(cs_bg1 & ~cpu1_WR_n), .q_a(bg1_vram_D),
	.clock_b(clk_49m), .address_b(bg1_raddr), .data_b(8'd0), .wren_b(1'b0), .q_b(bg1_vram_rD));

wire [7:0] bg0_vram_D, bg0_vram_rD;
reg [11:0] bg0_raddr;
dpram_dc #(.widthad_a(12)) bg0_vram (
	.clock_a(clk_49m), .address_a(cpu1_A[11:0]), .data_a(cpu1_Dout),
	.wren_a(cs_bg0 & ~cpu1_WR_n), .q_a(bg0_vram_D),
	.clock_b(clk_49m), .address_b(bg0_raddr), .data_b(8'd0), .wren_b(1'b0), .q_b(bg0_vram_rD));

wire [7:0] fg_vram_D, fg_vram_rD;
reg [11:0] fg_raddr;
dpram_dc #(.widthad_a(12)) fg_vram (
	.clock_a(clk_49m), .address_a(cpu1_A[11:0]), .data_a(cpu1_Dout),
	.wren_a(cs_fgvram & ~cpu1_WR_n), .q_a(fg_vram_D),
	.clock_b(clk_49m), .address_b(fg_raddr), .data_b(8'd0), .wren_b(1'b0), .q_b(fg_vram_rD));

wire [7:0] shared_ram_D_cpu1, shared_ram_D_cpu2;
dpram_dc #(.widthad_a(11)) shared_ram (
    .clock_a(clk_49m),
    .address_a(hs_write ? {hs_bank, hs_address[7:0]} : cpu1_A[10:0]),
    .data_a(hs_write ? hs_data_in : cpu1_Dout),
    .wren_a((cs_shared & ~cpu1_WR_n) | hs_write),
    .q_a(shared_ram_D_cpu1),
    .clock_b(clk_49m), .address_b(cpu2_A[10:0]), .data_b(cpu2_Dout),
    .wren_b(cs2_shared & ~cpu2_WR_n), .q_b(shared_ram_D_cpu2));

wire [7:0] hs_ram_q;
dpram_dc #(.widthad_a(8)) hs_ram (
    .clock_a(clk_49m), .address_a(cpu1_A[7:0]),
    .data_a(cpu1_Dout), .wren_a(cs_shared & ~cpu1_WR_n & (cpu1_A[10:8] == hs_bank)),
    .q_a(),
    .clock_b(clk_49m), .address_b(hs_address[7:0]),
    .data_b(hs_data_in), .wren_b(hs_write),
    .q_b(hs_ram_q));

wire [7:0] cpu1_Din = cs_rom ? main_rom_D : cs_bg1 ? bg1_vram_D : cs_bg0 ? bg0_vram_D :
                      cs_fgvram ? fg_vram_D : cs_shared ? shared_ram_D_cpu1 : 8'hFF;

wire [7:0] p1_inputs     = p1_controls;
wire [7:0] p2_inputs     = p2_controls;
wire [7:0] system_inputs = sys_controls;

assign hs_data_out = hs_ram_q;

//---------------------------------------------------- AY-3-8910 --------------------------------------------------------------//

wire [7:0] ay_dout;
wire [9:0] ay_sound;
jt49_bus ay (
	.rst_n(reset), .clk(clk_49m), .clk_en(cen_ay),
	.bdir(cs2_ay_addr | cs2_ay_wr), .bc1(cs2_ay_addr | cs2_ay_rd),
	.din(cpu2_Dout), .dout(ay_dout), .sel(1'b1),
	.sound(ay_sound), .sample(), .A(), .B(), .C(),
	.IOA_in(dip_sw[7:0]), .IOB_in(dip_sw[15:8]), .IOA_out(), .IOB_out()
);
// assign sound = {ay_sound[9], ay_sound, 5'd0};

// jt49 sound is UNSIGNED 10-bit (see jt49.v line 34). Previous code
// sign-extended the MSB which caused mid-scale wrap (ay_sound >= 512
// flipped to large negative) — audible as buzzing/static on loud notes.
// Pad a 0 MSB and upshift into the signed 16-bit frame instead.
assign sound = {1'b0, ay_sound, 5'd0};

wire [7:0] cpu2_Din = (~cpu2_IORQ_n) ? (cs2_ay_rd ? ay_dout : 8'hFF) :
                      cs2_rom ? sub_rom_D : cs2_shared ? shared_ram_D_cpu2 :
                      cs2_p2 ? p2_inputs : cs2_p1 ? p1_inputs :
                      cs2_system ? system_inputs : 8'hFF;

//---------------------------------------------- Priority Register -----------------------------------------------------------//

reg [2:0] priority_mode = 3'd0;
always_ff @(posedge clk_49m) begin
	if (!reset) priority_mode <= 0;
	else if (cs_priority & ~cpu1_WR_n) priority_mode <= cpu1_Dout[2:0];
end

//----------------------------------------------- Graphics ROMs --------------------------------------------------------------//

reg [12:0] fgtile_addr;
wire [7:0] fgtile_D;
eprom_8k fgtile_rom (.CLK(clk_49m), .ADDR(fgtile_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(fgtile_cs_i), .WR(ioctl_wr), .DATA(fgtile_D));

// BG tile ROMs — named by the layer they serve
// Note: bg0_tilerom loads from bgtiles1 ROM region (CS_DL=bgtile1_cs_i)
//       bg1_tilerom loads from bgtiles0 ROM region (CS_DL=bgtile0_cs_i)
// This cross-mapping matches MAME's "4 - Which" gfx assignment.

reg [12:0] bgtile0_addr;
wire [7:0] bgtile0_D;
eprom_8k bgtile_rom0 (.CLK(clk_49m), .ADDR(bgtile0_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(bgtile1_cs_i), .WR(ioctl_wr), .DATA(bgtile0_D));

reg [12:0] bgtile1_addr;
wire [7:0] bgtile1_D;
eprom_8k bgtile_rom1 (.CLK(clk_49m), .ADDR(bgtile1_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(bgtile0_cs_i), .WR(ioctl_wr), .DATA(bgtile1_D));

reg [12:0] sprite_addr;
wire [7:0] sprite0_D, sprite1_D;
eprom_8k sprite_rom0 (.CLK(clk_49m), .ADDR(sprite_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(sprite0_cs_i), .WR(ioctl_wr), .DATA(sprite0_D));
eprom_8k sprite_rom1 (.CLK(clk_49m), .ADDR(sprite_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(sprite1_cs_i), .WR(ioctl_wr), .DATA(sprite1_D));

wire [7:0] prom_r_D, prom_g_D, prom_b_D, prom_unk_D;
eprom_256b prom_r_rom (.CLK(clk_49m), .ADDR(prom_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(prom_r_cs_i), .WR(ioctl_wr), .DATA(prom_r_D));
eprom_256b prom_g_rom (.CLK(clk_49m), .ADDR(prom_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(prom_g_cs_i), .WR(ioctl_wr), .DATA(prom_g_D));
eprom_256b prom_b_rom (.CLK(clk_49m), .ADDR(prom_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(prom_b_cs_i), .WR(ioctl_wr), .DATA(prom_b_D));
eprom_256b prom_unk_rom (.CLK(clk_49m), .ADDR(8'd0), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(prom_unk_cs_i), .WR(ioctl_wr), .DATA(prom_unk_D));

//============================================= VIDEO RENDERING ==============================================//
//
// Line-buffer approach running at 49 MHz. During hblank (64 pixel clocks =
// ~640 master clocks), we render all 256 pixels of the NEXT line into line
// buffers for each layer. During active display, we read the buffers and
// composite based on priority_mode, then PROM-lookup the palette.
//
// 2bpp tile format (planes {0,4}, 16 bytes/tile):
//   byte_a = ROM[tile*16 + y], byte_b = ROM[tile*16 + y + 8]
//   pixel[x] for x=0..3: {byte_a[x+4], byte_a[x]}
//   pixel[x] for x=4..7: {byte_b[x-4+4], byte_b[x-4]}
//============================================================================================================//

// Line buffers (Double Buffered): 8-bit palette index per pixel (6-bit color + 2-bit pixel value)
reg [7:0] fg_lb_0  [0:255];
reg [7:0] fg_lb_1  [0:255];
reg [7:0] bg0_lb_0 [0:255];
reg [7:0] bg0_lb_1 [0:255];
reg [7:0] bg1_lb_0 [0:255];
reg [7:0] bg1_lb_1 [0:255];
reg [7:0] spr_lb_0 [0:255];
reg [7:0] spr_lb_1 [0:255];

reg lb_page = 1'b0; // 0 = render writes to _0, display reads from _1
                    // 1 = render writes to _1, display reads from _0

// Render engine state machine (runs at 49 MHz during hblank)
reg [4:0] rstate;
reg [8:0] rx;  // render x pixel counter (0-255)
// Latched tile info per fetch
reg [9:0] r_tile_num;
reg [5:0] r_color;
reg       r_flipx, r_flipy;
reg [7:0] r_byte_a, r_byte_b;
reg [7:0] r_scroll;
reg [2:0] r_layer; // 0=fg, 1=bg0, 2=bg1

// Which line we're rendering into buffers (next visible line)
wire [8:0] rnext     = v_cnt_rot + 9'd1;
wire [7:0] rline     = (8'd255 - rnext[7:0]);

// Decode 2bpp pixel from byte pair
function [1:0] pix2bpp;
    input [7:0] byt;
    input [1:0] x;
    begin
        pix2bpp = {byt[x+4], byt[x]};
    end
endfunction

localparam S_IDLE     = 5'd0;
localparam S_FG_CODE  = 5'd1,  S_FG_ATTR  = 5'd2,  S_FG_COL  = 5'd3;
localparam S_FG_ROMA  = 5'd4,  S_FG_ROMB  = 5'd5,  S_FG_DRAW = 5'd6;
localparam S_BG0_SCR  = 5'd7,  S_BG0_CODE = 5'd8,  S_BG0_ATTR = 5'd9,  S_BG0_COL = 5'd10;
localparam S_BG0_ROMA = 5'd11, S_BG0_ROMB = 5'd12, S_BG0_DRAW = 5'd13;
localparam S_BG1_SCR  = 5'd14, S_BG1_CODE = 5'd15, S_BG1_ATTR = 5'd16, S_BG1_COL = 5'd17;
localparam S_BG1_ROMA = 5'd18, S_BG1_ROMB = 5'd19, S_BG1_DRAW = 5'd20;
localparam S_SPR_INIT = 5'd21, S_SPR_RUN  = 5'd22;
localparam S_DONE     = 5'd23;

// Sprite engine sub-state
reg [4:0] spr_state;
reg [3:0] spr_idx;   // sprite index 0-15
reg [7:0] spr_y, spr_x, spr_code_raw, spr_attr_raw, spr_color_raw;
reg [7:0] spr_code;
reg [5:0] spr_color;
reg       spr_flipx, spr_flipy, spr_dbl;
reg [4:0] spr_next_state;
reg [4:0] spr_row;
reg [3:0] spr_col;
reg [7:0] spr_byte_a, spr_byte_b;
reg       spr_rom_half; // 0=sprite0_D, 1=sprite1_D
reg       spr_reading_b;

// Tile index helpers
wire [4:0] rx_col = rx[7:3];
wire [2:0] rx_fine = rx[2:0];
reg wait_cycle;

always_ff @(posedge clk_49m) begin
	if (!reset) begin
		rstate <= S_IDLE;
		rx <= 0;
		wait_cycle <= 0;
	end else if (rstate == S_IDLE) begin
		wait_cycle <= 0;
//		if (cen_pix && h_cnt_rot == 9'd256 && v_cnt_rot >= 9'd15 && v_cnt_rot < 9'd240) begin
		if (cen_pix && h_cnt_rot == 9'd240 && v_cnt_rot >= 9'd15 && v_cnt_rot < 9'd241) begin
			rx <= 0;
			rstate <= S_FG_CODE;
		    lb_page <= ~lb_page;			
		end
	end else if (wait_cycle) begin
		wait_cycle <= 0;
	end else begin
		if (rstate != S_FG_DRAW && rstate != S_BG0_DRAW && rstate != S_BG1_DRAW
		    && rstate != S_SPR_INIT && rstate != S_DONE && rstate != S_SPR_RUN)
			wait_cycle <= 1;
		case (rstate)

		//=== FG LAYER: read code, attr, color, then ROM bytes ===
		S_FG_CODE: begin
			fg_raddr <= 12'hC00 + {2'b00, rline[7:3], rx_col};
			rstate <= S_FG_ATTR;
		end
		S_FG_ATTR: begin
			r_tile_num[7:0] <= fg_vram_rD;
			fg_raddr <= 12'h800 + {2'b00, rline[7:3], rx_col};
			rstate <= S_FG_COL;
		end
		S_FG_COL: begin
			r_tile_num[9:8] <= fg_vram_rD[1:0];
			r_flipy <= fg_vram_rD[2];
			r_flipx <= fg_vram_rD[3];
			fg_raddr <= 12'h400 + {2'b00, rline[7:3], rx_col};
			rstate <= S_FG_ROMA;
		end
		S_FG_ROMA: begin
			r_color <= fg_vram_rD[5:0];
			begin
				reg [2:0] fy;
				fy = r_flipy ? (3'd7 - rline[2:0]) : rline[2:0];
				fgtile_addr <= {r_tile_num[8:0], 1'b0, fy};
			end
			rstate <= S_FG_ROMB;
		end
		S_FG_ROMB: begin
			r_byte_a <= fgtile_D;
			begin
				reg [2:0] fy;
				fy = r_flipy ? (3'd7 - rline[2:0]) : rline[2:0];
				fgtile_addr <= {r_tile_num[8:0], 1'b1, fy};
			end
			rstate <= S_FG_DRAW;
		end
		S_FG_DRAW: begin
			begin
				reg [7:0] ba, bb;
				reg [1:0] pval;
				ba = r_byte_a;
				bb = fgtile_D;
				// Left 4 pixels from ba, right 4 from bb
				// Normal: px 0-3 from ba, 4-7 from bb
				// FlipX:  px 0-3 from bb reversed, 4-7 from ba reversed
				if (r_flipx) begin
					if (lb_page) begin
						fg_lb_1[rx+7] <= {r_color, ba[7], ba[3]};
						fg_lb_1[rx+6] <= {r_color, ba[6], ba[2]};
						fg_lb_1[rx+5] <= {r_color, ba[5], ba[1]};
						fg_lb_1[rx+4] <= {r_color, ba[4], ba[0]};
						fg_lb_1[rx+3] <= {r_color, bb[7], bb[3]};
						fg_lb_1[rx+2] <= {r_color, bb[6], bb[2]};
						fg_lb_1[rx+1] <= {r_color, bb[5], bb[1]};
						fg_lb_1[rx+0] <= {r_color, bb[4], bb[0]};
					end else begin
						fg_lb_0[rx+7] <= {r_color, ba[7], ba[3]};
						fg_lb_0[rx+6] <= {r_color, ba[6], ba[2]};
						fg_lb_0[rx+5] <= {r_color, ba[5], ba[1]};
						fg_lb_0[rx+4] <= {r_color, ba[4], ba[0]};
						fg_lb_0[rx+3] <= {r_color, bb[7], bb[3]};
						fg_lb_0[rx+2] <= {r_color, bb[6], bb[2]};
						fg_lb_0[rx+1] <= {r_color, bb[5], bb[1]};
						fg_lb_0[rx+0] <= {r_color, bb[4], bb[0]};
					end
				end else begin
					if (lb_page) begin
						fg_lb_1[rx+7] <= {r_color, bb[4], bb[0]};
						fg_lb_1[rx+6] <= {r_color, bb[5], bb[1]};
						fg_lb_1[rx+5] <= {r_color, bb[6], bb[2]};
						fg_lb_1[rx+4] <= {r_color, bb[7], bb[3]};
						fg_lb_1[rx+3] <= {r_color, ba[4], ba[0]};
						fg_lb_1[rx+2] <= {r_color, ba[5], ba[1]};
						fg_lb_1[rx+1] <= {r_color, ba[6], ba[2]};
						fg_lb_1[rx+0] <= {r_color, ba[7], ba[3]};
					end else begin
						fg_lb_0[rx+7] <= {r_color, bb[4], bb[0]};
						fg_lb_0[rx+6] <= {r_color, bb[5], bb[1]};
						fg_lb_0[rx+5] <= {r_color, bb[6], bb[2]};
						fg_lb_0[rx+4] <= {r_color, bb[7], bb[3]};
						fg_lb_0[rx+3] <= {r_color, ba[4], ba[0]};
						fg_lb_0[rx+2] <= {r_color, ba[5], ba[1]};
						fg_lb_0[rx+1] <= {r_color, ba[6], ba[2]};
						fg_lb_0[rx+0] <= {r_color, ba[7], ba[3]};
					end
				end
			end
			rx <= rx + 9'd8;
			if (rx == 9'd248) begin
				rx <= 0;
				rstate <= S_BG0_SCR;
			end else
				rstate <= S_FG_CODE;
		end

		//=== BG0 LAYER: scroll, code, attr, color, ROM ===
		S_BG0_SCR: begin
			// Read scroll value from fgvram[0x3C0 + col]
			fg_raddr <= 12'h7C0 + {7'd0, rx_col};
			rstate <= S_BG0_CODE;
		end
		S_BG0_CODE: begin
			r_scroll <= fg_vram_rD;
			begin
				reg [7:0] sy;
				sy = rline + fg_vram_rD;
				bg0_raddr <= {2'b10, sy[7:3], rx_col}; // code @ +0x800
			end
			rstate <= S_BG0_ATTR;
		end
		S_BG0_ATTR: begin
			r_tile_num[7:0] <= bg0_vram_rD;
			begin
				reg [7:0] sy;
				sy = rline + r_scroll;
				bg0_raddr <= {2'b00, sy[7:3], rx_col}; // attr @ +0x000
			end
			rstate <= S_BG0_COL;
		end
		S_BG0_COL: begin
			r_tile_num[9:8] <= bg0_vram_rD[1:0];
			r_flipy <= bg0_vram_rD[2];
			r_flipx <= bg0_vram_rD[3];
			begin
				reg [7:0] sy;
				sy = rline + r_scroll;
				bg0_raddr <= {2'b11, sy[7:3], rx_col}; // color @ +0xC00
			end
			rstate <= S_BG0_ROMA;
		end
		S_BG0_ROMA: begin
			r_color <= bg0_vram_rD[5:0];
			begin
				reg [7:0] sy;
				reg [2:0] fy;
				sy = rline + r_scroll;
				fy = r_flipy ? (3'd7 - sy[2:0]) : sy[2:0];
				bgtile0_addr <= {r_tile_num[8:0], 1'b0, fy};
			end
			rstate <= S_BG0_ROMB;
		end
		S_BG0_ROMB: begin
			r_byte_a <= bgtile0_D;
			begin
				reg [7:0] sy;
				reg [2:0] fy;
				sy = rline + r_scroll;
				fy = r_flipy ? (3'd7 - sy[2:0]) : sy[2:0];
				bgtile0_addr <= {r_tile_num[8:0], 1'b1, fy};
			end
			rstate <= S_BG0_DRAW;
		end
		S_BG0_DRAW: begin
			begin
				reg [7:0] ba, bb;
				ba = r_byte_a;
				bb = bgtile0_D;
				if (r_flipx) begin
					if (lb_page) begin
						bg0_lb_1[rx+7] <= {r_color, ba[7], ba[3]};
						bg0_lb_1[rx+6] <= {r_color, ba[6], ba[2]};
						bg0_lb_1[rx+5] <= {r_color, ba[5], ba[1]};
						bg0_lb_1[rx+4] <= {r_color, ba[4], ba[0]};
						bg0_lb_1[rx+3] <= {r_color, bb[7], bb[3]};
						bg0_lb_1[rx+2] <= {r_color, bb[6], bb[2]};
						bg0_lb_1[rx+1] <= {r_color, bb[5], bb[1]};
						bg0_lb_1[rx+0] <= {r_color, bb[4], bb[0]};
					end else begin
						bg0_lb_0[rx+7] <= {r_color, ba[7], ba[3]};
						bg0_lb_0[rx+6] <= {r_color, ba[6], ba[2]};
						bg0_lb_0[rx+5] <= {r_color, ba[5], ba[1]};
						bg0_lb_0[rx+4] <= {r_color, ba[4], ba[0]};
						bg0_lb_0[rx+3] <= {r_color, bb[7], bb[3]};
						bg0_lb_0[rx+2] <= {r_color, bb[6], bb[2]};
						bg0_lb_0[rx+1] <= {r_color, bb[5], bb[1]};
						bg0_lb_0[rx+0] <= {r_color, bb[4], bb[0]};
					end
				end else begin
					if (lb_page) begin
						bg0_lb_1[rx+7] <= {r_color, bb[4], bb[0]};
						bg0_lb_1[rx+6] <= {r_color, bb[5], bb[1]};
						bg0_lb_1[rx+5] <= {r_color, bb[6], bb[2]};
						bg0_lb_1[rx+4] <= {r_color, bb[7], bb[3]};
						bg0_lb_1[rx+3] <= {r_color, ba[4], ba[0]};
						bg0_lb_1[rx+2] <= {r_color, ba[5], ba[1]};
						bg0_lb_1[rx+1] <= {r_color, ba[6], ba[2]};
						bg0_lb_1[rx+0] <= {r_color, ba[7], ba[3]};
					end else begin
						bg0_lb_0[rx+7] <= {r_color, bb[4], bb[0]};
						bg0_lb_0[rx+6] <= {r_color, bb[5], bb[1]};
						bg0_lb_0[rx+5] <= {r_color, bb[6], bb[2]};
						bg0_lb_0[rx+4] <= {r_color, bb[7], bb[3]};
						bg0_lb_0[rx+3] <= {r_color, ba[4], ba[0]};
						bg0_lb_0[rx+2] <= {r_color, ba[5], ba[1]};
						bg0_lb_0[rx+1] <= {r_color, ba[6], ba[2]};
						bg0_lb_0[rx+0] <= {r_color, ba[7], ba[3]};
					end
				end
			end
			rx <= rx + 9'd8;
			if (rx == 9'd248) begin
				rx <= 0;
				rstate <= S_BG1_SCR;
			end else
				rstate <= S_BG0_SCR;
		end

		//=== BG1 LAYER: same pattern as BG0 but uses bg1_vram and bgtile1_rom ===
		S_BG1_SCR: begin
			fg_raddr <= 12'h7E0 + {7'd0, rx_col};
			rstate <= S_BG1_CODE;
		end
		S_BG1_CODE: begin
			r_scroll <= fg_vram_rD;
			begin
				reg [7:0] sy;
				sy = rline + fg_vram_rD;
				bg1_raddr <= {2'b10, sy[7:3], rx_col};
			end
			rstate <= S_BG1_ATTR;
		end
		S_BG1_ATTR: begin
			r_tile_num[7:0] <= bg1_vram_rD;
			begin
				reg [7:0] sy;
				sy = rline + r_scroll;
				bg1_raddr <= {2'b00, sy[7:3], rx_col};
			end
			rstate <= S_BG1_COL;
		end
		S_BG1_COL: begin
			r_tile_num[9:8] <= bg1_vram_rD[1:0];
			r_flipy <= bg1_vram_rD[2];
			r_flipx <= bg1_vram_rD[3];
			begin
				reg [7:0] sy;
				sy = rline + r_scroll;
				bg1_raddr <= {2'b11, sy[7:3], rx_col};
			end
			rstate <= S_BG1_ROMA;
		end
		S_BG1_ROMA: begin
			r_color <= bg1_vram_rD[5:0];
			begin
				reg [7:0] sy;
				reg [2:0] fy;
				sy = rline + r_scroll;
				fy = r_flipy ? (3'd7 - sy[2:0]) : sy[2:0];
				bgtile1_addr <= {r_tile_num[8:0], 1'b0, fy};
			end
			rstate <= S_BG1_ROMB;
		end
		S_BG1_ROMB: begin
			r_byte_a <= bgtile1_D;
			begin
				reg [7:0] sy;
				reg [2:0] fy;
				sy = rline + r_scroll;
				fy = r_flipy ? (3'd7 - sy[2:0]) : sy[2:0];
				bgtile1_addr <= {r_tile_num[8:0], 1'b1, fy};
			end
			rstate <= S_BG1_DRAW;
		end
		S_BG1_DRAW: begin
			begin
				reg [7:0] ba, bb;
				ba = r_byte_a;
				bb = bgtile1_D;
				if (r_flipx) begin
					if (lb_page) begin
						bg1_lb_1[rx+7] <= {r_color, ba[7], ba[3]};
						bg1_lb_1[rx+6] <= {r_color, ba[6], ba[2]};
						bg1_lb_1[rx+5] <= {r_color, ba[5], ba[1]};
						bg1_lb_1[rx+4] <= {r_color, ba[4], ba[0]};
						bg1_lb_1[rx+3] <= {r_color, bb[7], bb[3]};
						bg1_lb_1[rx+2] <= {r_color, bb[6], bb[2]};
						bg1_lb_1[rx+1] <= {r_color, bb[5], bb[1]};
						bg1_lb_1[rx+0] <= {r_color, bb[4], bb[0]};
					end else begin
						bg1_lb_0[rx+7] <= {r_color, ba[7], ba[3]};
						bg1_lb_0[rx+6] <= {r_color, ba[6], ba[2]};
						bg1_lb_0[rx+5] <= {r_color, ba[5], ba[1]};
						bg1_lb_0[rx+4] <= {r_color, ba[4], ba[0]};
						bg1_lb_0[rx+3] <= {r_color, bb[7], bb[3]};
						bg1_lb_0[rx+2] <= {r_color, bb[6], bb[2]};
						bg1_lb_0[rx+1] <= {r_color, bb[5], bb[1]};
						bg1_lb_0[rx+0] <= {r_color, bb[4], bb[0]};
					end
				end else begin
					if (lb_page) begin
						bg1_lb_1[rx+7] <= {r_color, bb[4], bb[0]};
						bg1_lb_1[rx+6] <= {r_color, bb[5], bb[1]};
						bg1_lb_1[rx+5] <= {r_color, bb[6], bb[2]};
						bg1_lb_1[rx+4] <= {r_color, bb[7], bb[3]};
						bg1_lb_1[rx+3] <= {r_color, ba[4], ba[0]};
						bg1_lb_1[rx+2] <= {r_color, ba[5], ba[1]};
						bg1_lb_1[rx+1] <= {r_color, ba[6], ba[2]};
						bg1_lb_1[rx+0] <= {r_color, ba[7], ba[3]};
					end else begin
						bg1_lb_0[rx+7] <= {r_color, bb[4], bb[0]};
						bg1_lb_0[rx+6] <= {r_color, bb[5], bb[1]};
						bg1_lb_0[rx+5] <= {r_color, bb[6], bb[2]};
						bg1_lb_0[rx+4] <= {r_color, bb[7], bb[3]};
						bg1_lb_0[rx+3] <= {r_color, ba[4], ba[0]};
						bg1_lb_0[rx+2] <= {r_color, ba[5], ba[1]};
						bg1_lb_0[rx+1] <= {r_color, ba[6], ba[2]};
						bg1_lb_0[rx+0] <= {r_color, ba[7], ba[3]};
					end
				end
			end
			rx <= rx + 9'd8;
			if (rx == 9'd248) begin
				rx <= 0;
				rstate <= S_SPR_INIT;
			end else
				rstate <= S_BG1_SCR;
		end

		//=== SPRITES ===
		// 16 sprites total (2 banks × 8), data in FG VRAM at fixed offsets.
		// Bank 0: rambase=0x030, tilebase=0x80. Bank 1: rambase=0x010, tilebase=0x00.
		// Y/Col @ fgvram[rambase+0x000], Attr @ fgvram[rambase+0x400], Code/X @ fgvram[rambase+0x800]
		S_SPR_INIT: begin
			// Clear sprite buffer
			if (lb_page) begin
				for (integer i = 0; i < 256; i = i + 1) spr_lb_1[i] <= 8'd0;
			end else begin
				for (integer i = 0; i < 256; i = i + 1) spr_lb_0[i] <= 8'd0;
			end
			spr_idx <= 0;
			spr_state <= 0;
			rstate <= S_SPR_RUN;
		end

		S_SPR_RUN: begin
			// Sprite sub-state machine
			case (spr_state)
			5'd0: begin
				// Compute rambase: idx 0-7 = bank0 (0x030), idx 8-15 = bank1 (0x010)
				// Read Y: fgvram[rambase + (idx&7)*2]
				begin
					reg [11:0] rambase;
					rambase = (spr_idx < 8) ? 12'h430 : 12'h410;
					fg_raddr <= rambase + {8'd0, spr_idx[2:0], 1'b0};
				end
			    spr_next_state <= 1;
			    spr_state <= 11;
			end
			5'd1: begin
				spr_y <= fg_vram_rD;
				// Read Color: fgvram[rambase + (idx&7)*2 + 1]
				begin
					reg [11:0] rambase;
					rambase = (spr_idx < 8) ? 12'h430 : 12'h410;
					fg_raddr <= rambase + {8'd0, spr_idx[2:0], 1'b1};
				end
				spr_next_state <= 2;
				spr_state <= 11;
			end
			5'd2: begin
				spr_color <= fg_vram_rD[5:0];
				// Read Attr: fgvram[rambase + 0x400 + (idx&7)*2]
				begin
					reg [11:0] rambase;
					rambase = (spr_idx < 8) ? 12'h430 : 12'h410;
					fg_raddr <= rambase + 12'h400 + {8'd0, spr_idx[2:0], 1'b0};
				end
				spr_next_state <= 3;
				spr_state <= 11;
			end
			5'd3: begin
				spr_attr_raw <= fg_vram_rD;
				// Read Code/X byte 0: fgvram[rambase + 0x800 + (idx&7)*2]
				begin
					reg [11:0] rambase;
					rambase = (spr_idx < 8) ? 12'h430 : 12'h410;
					fg_raddr <= rambase + 12'h800 + {8'd0, spr_idx[2:0], 1'b0};
				end
				spr_next_state <= 4;
				spr_state <= 11;
			end
			5'd4: begin
				spr_code_raw <= fg_vram_rD;
				// Read X position: fgvram[rambase + 0x800 + (idx&7)*2 + 1]
				begin
					reg [11:0] rambase;
					rambase = (spr_idx < 8) ? 12'h430 : 12'h410;
					fg_raddr <= rambase + 12'h800 + {8'd0, spr_idx[2:0], 1'b1};
				end
				spr_next_state <= 5;
				spr_state <= 11;
			end
			5'd5: begin
				spr_x <= fg_vram_rD;
				// Decode sprite
				begin
					reg [7:0] code;
					reg [7:0] tilebase;
					tilebase = (spr_idx < 8) ? 8'd128 : 8'd0; // 0x80/2=64 for bank0, 0 for bank1
					code = {spr_attr_raw[0], spr_code_raw[7:2]} + tilebase;
					spr_code <= code;
					spr_flipy <= spr_code_raw[0] ^ rot_flip;
					spr_flipx <= spr_code_raw[1] ^ rot_flip;
					spr_dbl <= spr_attr_raw[3];
				end
				spr_state <= 6;
			end
			5'd6: begin
				// Check if sprite is on this line
				begin
					reg [7:0] sy, sprite_height;
					reg [8:0] local_y;
					sy = flip_screen ? spr_y : (spr_dbl ? (8'd224 - spr_y) : (8'd240 - spr_y));
					sprite_height = spr_dbl ? 8'd32 : 8'd16;
					local_y = {1'b0, rline} - {1'b0, sy};
					if (local_y[8:0] < {1'b0, sprite_height}) begin
						// Sprite is on this line — start pixel rendering
						spr_row <= local_y[4:0];
						spr_col <= 0;
						spr_reading_b <= 0;
						spr_state <= 7; // fetch ROM byte_a
					end else begin
						// Not on this line, next sprite
						if (spr_idx == 4'd15) rstate <= S_DONE;
						else begin spr_idx <= spr_idx + 1; spr_state <= 0; end
					end
				end
			end
			5'd7: begin
				// Set sprite ROM address for current row, left half (byte_a)
				// 16x16 sprite layout (64 bytes/sprite):
				// row 0-7:  base + row                 (byte_a for cols 0-3)
				//           base + row + 8             (byte_b for cols 4-7)
				//           base + row + 16            (cols 8-11)
				//           base + row + 24            (cols 12-15)
				// row 8-15: base + 32 + (row-8)        etc.
				// For double height: code/2, 128 bytes
				begin
					reg [13:0] base;
					reg [4:0] erow;
					reg [3:0] quarter;
					erow = spr_flipy ? ((spr_dbl ? 5'd31 : 5'd15) - spr_row) : spr_row;
					if (spr_dbl)
						base = {spr_code[7:1], 7'd0}; // code/2 * 128
					else
						base = {spr_code, 6'd0}; // code * 64
					quarter = spr_col[3:2]; // which quarter (0-3)
					// Byte offset within sprite: row_base + quarter*8
					// row_base = (erow < 8) ? erow : 32 + (erow-8)  for 16x16
					// For 32 tall: (erow < 8) ? erow : (erow < 16) ? 32+(erow-8) : (erow < 24) ? 64+(erow-16) : 96+(erow-24)
					begin
						reg [6:0] row_base;
						reg [13:0] full_addr;
						if (spr_dbl) begin
							row_base = {erow[4:3], 2'b00, erow[2:0]};
						end else begin
							row_base = erow[3] ? (7'd32 + {4'd0, erow[2:0]}) : {4'd0, erow[2:0]};
						end
						full_addr = base + {7'd0, row_base} + {10'd0, quarter, 3'b000};
						sprite_addr <= full_addr[12:0];
						spr_rom_half <= full_addr[13];
					end
				end
				spr_state <= 10;
			end
			5'd8: begin
				// ROM data ready — latch
				spr_byte_a <= spr_rom_half ? sprite1_D : sprite0_D;
				spr_state <= 9;
			end
			5'd9: begin
				// Draw 4 pixels from this quarter
				begin
					reg [1:0] pval;
					reg [2:0] bx;
					reg [7:0] xpos;
					bx = spr_flipx ? spr_col[1:0] : (3'd3 - spr_col[1:0]);
					pval = {spr_byte_a[bx + 4], spr_byte_a[bx]};
					xpos = rot_flip	? (8'd255 - (spr_x + {4'd0, spr_col})): (spr_x + {4'd0, spr_col});
					if (lb_page) begin
						if (pval != 2'd0 && spr_lb_1[xpos] == 8'd0)
							spr_lb_1[xpos] <= {spr_color, pval};
					end else begin
						if (pval != 2'd0 && spr_lb_0[xpos] == 8'd0)
							spr_lb_0[xpos] <= {spr_color, pval};
					end
				end
				if (spr_col == 4'd15) begin
					// Done with this sprite
					if (spr_idx == 4'd15) rstate <= S_DONE;
					else begin spr_idx <= spr_idx + 1; spr_state <= 0; end
				end else begin
					spr_col <= spr_col + 1;
					// Every 4 pixels, need new ROM fetch
					if (spr_col[1:0] == 2'd3) spr_state <= 7;
				end
			end
			5'd10: begin
				spr_state <= 8;   // ROM data now valid
			end
			5'd11: begin
				spr_state <= spr_next_state;
			end
			default: spr_state <= 0;
			endcase
		end

		S_DONE: begin
			rstate <= S_IDLE;
		end

		default: rstate <= S_IDLE;
		endcase
	end
end

//--- Display compositing ---
wire [7:0] disp_x = 8'd255 - h_cnt_rot[7:0];
wire [7:0] fg_pix  = lb_page ? fg_lb_0[disp_x]  : fg_lb_1[disp_x];
wire [7:0] bg0_pix = lb_page ? bg0_lb_0[disp_x] : bg0_lb_1[disp_x];
wire [7:0] bg1_pix = lb_page ? bg1_lb_0[disp_x] : bg1_lb_1[disp_x];
wire [7:0] spr_pix = lb_page ? spr_lb_0[disp_x] : spr_lb_1[disp_x];

wire fg_opaque  = (fg_pix[1:0]  != 2'd0);
wire bg0_opaque = (bg0_pix[1:0] != 2'd0);
wire bg1_opaque = (bg1_pix[1:0] != 2'd0);
wire spr_opaque = (spr_pix[1:0] != 2'd0);

reg [7:0] final_pixel;
always_comb begin
	case (priority_mode)
	3'd0: begin // BG0(opaque) → sprites → BG1 → FG
		if (fg_opaque)       final_pixel = fg_pix;
		else if (bg1_opaque) final_pixel = bg1_pix;
		else if (spr_opaque) final_pixel = spr_pix;
		else                 final_pixel = bg0_pix;
	end
	3'd1: begin // BG0(opaque) → BG1 → sprites → FG
		if (fg_opaque)       final_pixel = fg_pix;
		else if (spr_opaque) final_pixel = spr_pix;
		else if (bg1_opaque) final_pixel = bg1_pix;
		else                 final_pixel = bg0_pix;
	end
	3'd2: begin // BG0(opaque) → sprites → BG0(transp) → BG1 → FG
		if (fg_opaque)       final_pixel = fg_pix;
		else if (bg1_opaque) final_pixel = bg1_pix;
		else if (bg0_opaque) final_pixel = bg0_pix;
		else if (spr_opaque) final_pixel = spr_pix;
		else                 final_pixel = bg0_pix;
	end
	3'd3: begin // BG0 → BG1 → FG → sprites
		if (spr_opaque)      final_pixel = spr_pix;
		else if (fg_opaque)  final_pixel = fg_pix;
		else if (bg1_opaque) final_pixel = bg1_pix;
		else                 final_pixel = bg0_pix;
	end
	3'd4: begin // FG(opaque) → sprites → BG0 → BG1
		if (bg1_opaque)      final_pixel = bg1_pix;
		else if (bg0_opaque) final_pixel = bg0_pix;
		else if (spr_opaque) final_pixel = spr_pix;
		else                 final_pixel = fg_pix;
	end
	default: final_pixel = bg0_pix;
	endcase
end

// PROM palette lookup (combinational — no pipeline delay)
wire [7:0] prom_addr = (hblk | vblk) ? 8'd0 : final_pixel;

// Scale 4-bit PROM output to 5-bit for MiSTer
assign red   = {prom_r_D[3:0], prom_r_D[3]};
assign green = {prom_g_D[3:0], prom_g_D[3]};
assign blue  = {prom_b_D[3:0], prom_b_D[3]};

endmodule
