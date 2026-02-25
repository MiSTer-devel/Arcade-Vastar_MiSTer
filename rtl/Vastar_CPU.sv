//============================================================================
//
//  Vastar CPU board — Phase 1: Dual CPU + Memory Map + ROM Layout
//  Copyright (C) 2026 Rodimus
//
//  MAME reference: vastar.cpp, vastar_viddev.cpp
//  Hardware: Z80 CPU1 + Z80 CPU2 @ 3.072 MHz (XTAL 18.432 / 6)
//            AY-3-8910 @ 1.536 MHz (18.432 / 12)
//  Screen: 256x256 total, visible 256x224 (lines 16-239), 60.58 Hz
//
//  Phase 1 goal: Both CPUs boot from ROM, shared RAM communication,
//  correct I/O decoding, interrupt generation.
//  Video: forced red screen to verify timing path.
//
//============================================================================

module Vastar_CPU
(
	input         reset,      // Active HIGH: 0 = in reset, 1 = running
	input         clk_49m,

	// Video outputs
	output  [4:0] red, green, blue,
	output        video_hsync, video_vsync, video_csync,
	output        video_hblank, video_vblank,
	output        ce_pix,

	// Player controls (active HIGH from MiSTer)
	// p1/p2: {2'b00, btn2, btn1, right, left, down, up}
	// sys:   {2'b00, service, start2, start1, 1'b0, coin2, coin1}
	input   [7:0] p1_controls,
	input   [7:0] p2_controls,
	input   [7:0] sys_controls,

	// DIP switches
	input  [15:0] dip_sw,

	// Audio (Phase 1: silent)
	output signed [15:0] sound,

	// Screen centering
	input   [3:0] h_center, v_center,

	// ROM loading (from selector, index 0 only)
	input         main_rom_cs_i, sub_rom_cs_i, fgtile_cs_i,
	input         sprite0_cs_i, sprite1_cs_i,
	input         bgtile0_cs_i, bgtile1_cs_i,
	input         prom_r_cs_i, prom_g_cs_i, prom_b_cs_i, prom_unk_cs_i,
	input  [24:0] ioctl_addr,
	input   [7:0] ioctl_data,
	input         ioctl_wr,

	input         pause,

	// Hiscore interface (stubbed for Phase 1)
	input  [15:0] hs_address,
	input   [7:0] hs_data_in,
	output  [7:0] hs_data_out,
	input         hs_write
);

// Hiscore stubbed — no dedicated work RAM in Vastar CPU1 map
assign hs_data_out = 8'hFF;

//------------------------------------------------------- Clock enables -------------------------------------------------------//

// Pixel clock: ~5 MHz (49.152 * 89/875 ≈ 4.997 MHz) via fractional divider
wire [1:0] pix_cen_o;
jtframe_frac_cen #(2) pix_cen
(
	.clk(clk_49m),
	.n(10'd89),
	.m(10'd875),
	.cen(pix_cen_o),
	.cenb()
);
wire cen_5m = pix_cen_o[0];
assign ce_pix = cen_5m;

// CPU clock: 49.152 / 16 = 3.072 MHz (matches hardware: 18.432 / 6 = 3.072 MHz)
reg [3:0] cpu_div = 4'd0;
always_ff @(posedge clk_49m) begin
	cpu_div <= cpu_div + 4'd1;
end
wire cen_cpu = (cpu_div == 4'd0);

// AY clock: every other CPU enable = 1.536 MHz (matches hardware: 18.432 / 12)
reg ay_toggle = 1'b0;
always_ff @(posedge clk_49m) begin
	if (cen_cpu) ay_toggle <= ~ay_toggle;
end
wire cen_ay = cen_cpu & ~ay_toggle;

//-------------------------------------------------------- Video timing --------------------------------------------------------//

// H counter 0-319, V counter 0-263
// From MAME: set_raw(5MHz, 320, 0, 256, 264, 16, 240)
reg [8:0] base_h_cnt = 9'd0;
reg [8:0] v_cnt = 9'd0;
always_ff @(posedge clk_49m) begin
	if (cen_5m) begin
		if (base_h_cnt == 9'd319) begin
			base_h_cnt <= 9'd0;
			v_cnt <= (v_cnt == 9'd263) ? 9'd0 : v_cnt + 9'd1;
		end else
			base_h_cnt <= base_h_cnt + 9'd1;
	end
end

// Blanking
wire hblk = (base_h_cnt >= 9'd256);
wire vblk = (v_cnt < 9'd16) | (v_cnt >= 9'd240);
assign video_hblank = hblk;
assign video_vblank = vblk;

// Sync generation with screen centering offsets
wire [8:0] hs_start = 9'd280 + {5'd0, h_center};
wire [8:0] hs_end   = hs_start + 9'd32;
wire [8:0] vs_start = 9'd248 + {5'd0, v_center};
wire [8:0] vs_end   = vs_start + 9'd4;
assign video_hsync = (base_h_cnt >= hs_start && base_h_cnt < hs_end);
assign video_vsync = (v_cnt >= vs_start && v_cnt < vs_end);
assign video_csync = ~(video_hsync ^ video_vsync);

//------------------------------------------------------- CPU1 — Main ---------------------------------------------------------//

wire [15:0] cpu1_A;
wire [7:0]  cpu1_Dout;
wire        cpu1_WR_n, cpu1_RD_n, cpu1_MREQ_n, cpu1_IORQ_n, cpu1_M1_n, cpu1_RFSH_n;

T80s cpu1
(
	.RESET_n(reset),
	.CLK(clk_49m),
	.CEN(cen_cpu & ~pause),
	.WAIT_n(1'b1),
	.INT_n(1'b1),
	.NMI_n(~cpu1_nmi),
	.M1_n(cpu1_M1_n),
	.MREQ_n(cpu1_MREQ_n),
	.IORQ_n(cpu1_IORQ_n),
	.RD_n(cpu1_RD_n),
	.WR_n(cpu1_WR_n),
	.RFSH_n(cpu1_RFSH_n),
	.A(cpu1_A),
	.DI(cpu1_Din),
	.DO(cpu1_Dout)
);

//------------------------------------------------------ CPU1 Interrupts ------------------------------------------------------//

// CPU1: NMI on vblank rising edge, gated by mainlatch[0] (nmi_mask)
// Z80 NMI is edge-triggered — a single-cycle pulse is sufficient
reg cpu1_nmi  = 1'b0;
reg vblk_prev = 1'b0;
always_ff @(posedge clk_49m) begin
	if (!reset) begin
		cpu1_nmi  <= 1'b0;
		vblk_prev <= 1'b0;
	end else begin
		vblk_prev <= vblk;
		cpu1_nmi  <= (vblk && !vblk_prev && nmi_mask);
	end
end

//-------------------------------------------------- CPU1 Address Decoding ----------------------------------------------------//

wire cpu1_mem_valid = ~cpu1_MREQ_n & cpu1_RFSH_n;

wire cs_rom      = cpu1_mem_valid & ~cpu1_A[15];                                           // 0x0000-0x7FFF
wire cs_bg1      = cpu1_mem_valid & ((cpu1_A[15:12] == 4'h8) |                             // 0x8000-0x8FFF
                                     (cpu1_A[15:12] == 4'hA));                             // 0xA000-0xAFFF (mirror)
wire cs_bg0      = cpu1_mem_valid & ((cpu1_A[15:12] == 4'h9) |                             // 0x9000-0x9FFF
                                     (cpu1_A[15:12] == 4'hB));                             // 0xB000-0xBFFF (mirror)
wire cs_priority = cpu1_mem_valid & (cpu1_A[15:0] == 16'hC000);                            // 0xC000 (write only)
wire cs_fgvram   = cpu1_mem_valid & (cpu1_A[15:12] == 4'hC) & (cpu1_A[11:10] != 2'b00);  // 0xC400-0xCFFF
wire cs_watchdog = cpu1_mem_valid & (cpu1_A[15:12] == 4'hE);                               // 0xE000
wire cs_shared   = cpu1_mem_valid & (cpu1_A[15:11] == 5'b11110);                           // 0xF000-0xF7FF

// CPU1 I/O: mainlatch at ports 0x00-0x07
wire cs_mainlatch = ~cpu1_IORQ_n & ~cpu1_WR_n & (cpu1_A[3:0] <= 4'h7);

//------------------------------------------------------ CPU1 Mainlatch -------------------------------------------------------//

reg [7:0] mainlatch = 8'd0;
always_ff @(posedge clk_49m) begin
	if (!reset)
		mainlatch <= 8'd0;
	else if (cen_cpu && cs_mainlatch)
		mainlatch[cpu1_A[2:0]] <= cpu1_Dout[0];
end

wire nmi_mask    = mainlatch[0];
wire flip_screen = mainlatch[1];
wire cpu2_rst    = ~mainlatch[2];  // CPU2 held in reset until CPU1 writes 1 to bit 2

//------------------------------------------------------- CPU2 — Sub ----------------------------------------------------------//

wire [15:0] cpu2_A;
wire [7:0]  cpu2_Dout;
wire        cpu2_WR_n, cpu2_RD_n, cpu2_MREQ_n, cpu2_IORQ_n, cpu2_M1_n, cpu2_RFSH_n;

T80s cpu2
(
	.RESET_n(reset & ~cpu2_rst),
	.CLK(clk_49m),
	.CEN(cen_cpu & ~pause),
	.WAIT_n(1'b1),
	.INT_n(~cpu2_irq),
	.NMI_n(1'b1),
	.M1_n(cpu2_M1_n),
	.MREQ_n(cpu2_MREQ_n),
	.IORQ_n(cpu2_IORQ_n),
	.RD_n(cpu2_RD_n),
	.WR_n(cpu2_WR_n),
	.RFSH_n(cpu2_RFSH_n),
	.A(cpu2_A),
	.DI(cpu2_Din),
	.DO(cpu2_Dout)
);

//------------------------------------------------------ CPU2 Interrupts ------------------------------------------------------//

// CPU2 periodic IRQ at ~242 Hz (4 × 60.58 Hz)
// 49,152,000 / 242 ≈ 203,107 clocks per interrupt
reg [17:0] cpu2_irq_cnt = 18'd0;
reg        cpu2_irq = 1'b0;
always_ff @(posedge clk_49m) begin
	if (!reset || cpu2_rst) begin
		cpu2_irq_cnt <= 18'd0;
		cpu2_irq     <= 1'b0;
	end else begin
		if (cpu2_irq_cnt == 18'd203107) begin
			cpu2_irq_cnt <= 18'd0;
			cpu2_irq     <= 1'b1;
		end else
			cpu2_irq_cnt <= cpu2_irq_cnt + 18'd1;
		// IRQ cleared on interrupt acknowledge (IORQ + M1)
		if (~cpu2_IORQ_n & ~cpu2_M1_n)
			cpu2_irq <= 1'b0;
	end
end

//-------------------------------------------------- CPU2 Address Decoding ----------------------------------------------------//

wire cpu2_mem_valid = ~cpu2_MREQ_n & cpu2_RFSH_n;

wire cs2_rom    = cpu2_mem_valid & ~cpu2_A[15] & ~cpu2_A[14] & ~cpu2_A[13]; // 0x0000-0x1FFF
wire cs2_shared = cpu2_mem_valid & (cpu2_A[15:11] == 5'b01000);              // 0x4000-0x47FF
wire cs2_p2     = cpu2_mem_valid & (cpu2_A[15:0] == 16'h8000);               // P2 inputs
wire cs2_p1     = cpu2_mem_valid & (cpu2_A[15:0] == 16'h8040);               // P1 inputs
wire cs2_system = cpu2_mem_valid & (cpu2_A[15:0] == 16'h8080);               // System inputs

// CPU2 I/O: AY-3-8910 at ports 0x00-0x02
wire cs2_ay_addr = ~cpu2_IORQ_n & ~cpu2_WR_n & (cpu2_A[3:0] == 4'h0); // Port 0x00: latch address
wire cs2_ay_wr   = ~cpu2_IORQ_n & ~cpu2_WR_n & (cpu2_A[3:0] == 4'h1); // Port 0x01: write data
wire cs2_ay_rd   = ~cpu2_IORQ_n & ~cpu2_RD_n & (cpu2_A[3:0] == 4'h2); // Port 0x02: read data

//---------------------------------------------------------- ROMs -------------------------------------------------------------//

// Main CPU ROM — 32KB (8 × 4KB ROMs e_f4..e_n5, CPU1 address space 0x0000-0x7FFF)
wire [7:0] main_rom_D;
eprom_32k main_rom
(
	.CLK(clk_49m),
	.ADDR(cpu1_A[14:0]),
	.CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr),
	.DATA_IN(ioctl_data),
	.CS_DL(main_rom_cs_i),
	.WR(ioctl_wr),
	.DATA(main_rom_D)
);

// Sub CPU ROM — 8KB (e_f2.rom @ 0x0000, e_j2.rom @ 0x1000; CPU2 address 0x0000-0x1FFF)
wire [7:0] sub_rom_D;
eprom_8k sub_rom
(
	.CLK(clk_49m),
	.ADDR(cpu2_A[12:0]),
	.CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr),
	.DATA_IN(ioctl_data),
	.CS_DL(sub_rom_cs_i),
	.WR(ioctl_wr),
	.DATA(sub_rom_D)
);

//------------------------------------------------------- CPU1 VRAM -----------------------------------------------------------//

// BG1 VRAM — 4KB (CPU1: 0x8000-0x8FFF, mirror 0xA000-0xAFFF)
wire [7:0] bg1_vram_D;
dpram_dc #(.widthad_a(12)) bg1_vram
(
	.clock_a(clk_49m),
	.address_a(cpu1_A[11:0]),
	.data_a(cpu1_Dout),
	.wren_a(cs_bg1 & ~cpu1_WR_n),
	.q_a(bg1_vram_D),
	.clock_b(clk_49m),
	.address_b(12'd0), .data_b(8'd0), .wren_b(1'b0), .q_b()
);

// BG0 VRAM — 4KB (CPU1: 0x9000-0x9FFF, mirror 0xB000-0xBFFF)
wire [7:0] bg0_vram_D;
dpram_dc #(.widthad_a(12)) bg0_vram
(
	.clock_a(clk_49m),
	.address_a(cpu1_A[11:0]),
	.data_a(cpu1_Dout),
	.wren_a(cs_bg0 & ~cpu1_WR_n),
	.q_a(bg0_vram_D),
	.clock_b(clk_49m),
	.address_b(12'd0), .data_b(8'd0), .wren_b(1'b0), .q_b()
);

// FG VRAM — 4KB (CPU1: 0xC400-0xCFFF)
wire [7:0] fg_vram_D;
dpram_dc #(.widthad_a(12)) fg_vram
(
	.clock_a(clk_49m),
	.address_a(cpu1_A[11:0]),
	.data_a(cpu1_Dout),
	.wren_a(cs_fgvram & ~cpu1_WR_n),
	.q_a(fg_vram_D),
	.clock_b(clk_49m),
	.address_b(12'd0), .data_b(8'd0), .wren_b(1'b0), .q_b()
);

//------------------------------------------------------ Shared RAM -----------------------------------------------------------//

// 2KB dual-port RAM — CPU1 @ 0xF000-0xF7FF, CPU2 @ 0x4000-0x47FF
wire [7:0] shared_ram_D_cpu1, shared_ram_D_cpu2;
dpram_dc #(.widthad_a(11)) shared_ram
(
	.clock_a(clk_49m),
	.address_a(cpu1_A[10:0]),
	.data_a(cpu1_Dout),
	.wren_a(cs_shared & ~cpu1_WR_n),
	.q_a(shared_ram_D_cpu1),
	.clock_b(clk_49m),
	.address_b(cpu2_A[10:0]),
	.data_b(cpu2_Dout),
	.wren_b(cs2_shared & ~cpu2_WR_n),
	.q_b(shared_ram_D_cpu2)
);

//-------------------------------------------------- CPU1 Data Input Mux -------------------------------------------------------//

wire [7:0] cpu1_Din = cs_rom    ? main_rom_D       :
                      cs_bg1    ? bg1_vram_D        :
                      cs_bg0    ? bg0_vram_D        :
                      cs_fgvram ? fg_vram_D         :
                      cs_shared ? shared_ram_D_cpu1 :
                      8'hFF;

//------------------------------------------------------ Input Wires ----------------------------------------------------------//

// Invert active-HIGH MiSTer inputs to active-LOW as MAME expects
wire [7:0] p1_inputs     = ~p1_controls;
wire [7:0] p2_inputs     = ~p2_controls;
wire [7:0] system_inputs = ~sys_controls;

//---------------------------------------------------- AY-3-8910 --------------------------------------------------------------//

// AY-3-8910 on CPU2's I/O bus. DSW reads via IOA (DSW1) and IOB (DSW2).
wire [7:0] ay_dout;
jt49_bus ay
(
	.rst_n(reset),
	.clk(clk_49m),
	.clk_en(cen_ay),
	.bdir(cs2_ay_addr | cs2_ay_wr),
	.bc1(cs2_ay_addr | cs2_ay_rd),
	.din(cpu2_Dout),
	.dout(ay_dout),
	.sel(1'b1),
	.sound(),           // Phase 1: audio not routed
	.sample(),
	.A(),
	.B(),
	.C(),
	.IOA_in(dip_sw[7:0]),   // DSW1
	.IOB_in(dip_sw[15:8]),  // DSW2
	.IOA_out(),
	.IOB_out()
);

// Phase 1: silent
assign sound = 16'h0000;

//-------------------------------------------------- CPU2 Data Input Mux -------------------------------------------------------//

wire [7:0] cpu2_Din = (~cpu2_IORQ_n)   ? (cs2_ay_rd ? ay_dout : 8'hFF) :
                      cs2_rom           ? sub_rom_D                      :
                      cs2_shared        ? shared_ram_D_cpu2              :
                      cs2_p2            ? p2_inputs                      :
                      cs2_p1            ? p1_inputs                      :
                      cs2_system        ? system_inputs                  :
                      8'hFF;

//------------------------------------------- Graphics ROM Stubs (Phase 1) ---------------------------------------------------//

// Instantiated for ROM loading; rendering not implemented until Phase 2.
wire [7:0] fgtile_D;
eprom_8k fgtile_rom
(
	.CLK(clk_49m), .ADDR(13'd0),
	.CLK_DL(clk_49m), .ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data),
	.CS_DL(fgtile_cs_i), .WR(ioctl_wr), .DATA(fgtile_D)
);

wire [7:0] sprite0_D;
eprom_8k sprite_rom0
(
	.CLK(clk_49m), .ADDR(13'd0),
	.CLK_DL(clk_49m), .ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data),
	.CS_DL(sprite0_cs_i), .WR(ioctl_wr), .DATA(sprite0_D)
);

wire [7:0] sprite1_D;
eprom_8k sprite_rom1
(
	.CLK(clk_49m), .ADDR(13'd0),
	.CLK_DL(clk_49m), .ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data),
	.CS_DL(sprite1_cs_i), .WR(ioctl_wr), .DATA(sprite1_D)
);

wire [7:0] bgtile0_D;
eprom_8k bgtile_rom0
(
	.CLK(clk_49m), .ADDR(13'd0),
	.CLK_DL(clk_49m), .ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data),
	.CS_DL(bgtile0_cs_i), .WR(ioctl_wr), .DATA(bgtile0_D)
);

wire [7:0] bgtile1_D;
eprom_8k bgtile_rom1
(
	.CLK(clk_49m), .ADDR(13'd0),
	.CLK_DL(clk_49m), .ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data),
	.CS_DL(bgtile1_cs_i), .WR(ioctl_wr), .DATA(bgtile1_D)
);

wire [7:0] prom_r_D;
eprom_256b prom_r_rom
(
	.CLK(clk_49m), .ADDR(8'd0),
	.CLK_DL(clk_49m), .ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data),
	.CS_DL(prom_r_cs_i), .WR(ioctl_wr), .DATA(prom_r_D)
);

wire [7:0] prom_g_D;
eprom_256b prom_g_rom
(
	.CLK(clk_49m), .ADDR(8'd0),
	.CLK_DL(clk_49m), .ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data),
	.CS_DL(prom_g_cs_i), .WR(ioctl_wr), .DATA(prom_g_D)
);

wire [7:0] prom_b_D;
eprom_256b prom_b_rom
(
	.CLK(clk_49m), .ADDR(8'd0),
	.CLK_DL(clk_49m), .ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data),
	.CS_DL(prom_b_cs_i), .WR(ioctl_wr), .DATA(prom_b_D)
);

wire [7:0] prom_unk_D;
eprom_256b prom_unk_rom
(
	.CLK(clk_49m), .ADDR(8'd0),
	.CLK_DL(clk_49m), .ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data),
	.CS_DL(prom_unk_cs_i), .WR(ioctl_wr), .DATA(prom_unk_D)
);

//------------------------------------------------ DIAGNOSTIC: Force Red Screen -----------------------------------------------//

// Phase 1: force solid red to verify video timing path.
// Replace with real renderer in Phase 2.
assign red   = (hblk | vblk) ? 5'd0 : 5'd31;
assign green = 5'd0;
assign blue  = 5'd0;

endmodule
