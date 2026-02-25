//============================================================================
//
//  Vastar top-level game module
//  Copyright (C) 2026 Rodimus
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//
//============================================================================

module Vastar
(
	input                reset,
	input                clk_49m,

	// Player controls (active HIGH, assembled from MiSTer inputs)
	// p1/p2: {2'b00, btn2, btn1, right, left, down, up}
	// sys:   {2'b00, service, start2, start1, 1'b0, coin2, coin1}
	input          [7:0] p1_controls,
	input          [7:0] p2_controls,
	input          [7:0] sys_controls,

	// DIP switches (directly from MiSTer OSD)
	input         [15:0] dip_sw,

	// Video outputs
	output               video_hsync, video_vsync, video_csync,
	output               video_hblank, video_vblank,
	output               ce_pix,
	output         [4:0] video_r, video_g, video_b,

	// Audio output
	output signed [15:0] sound,

	// Screen centering
	input          [3:0] h_center, v_center,

	// ROM loading
	input         [24:0] ioctl_addr,
	input          [7:0] ioctl_data,
	input                ioctl_wr,
	input          [7:0] ioctl_index,

	input                pause,

	// Hiscore interface
	input         [15:0] hs_address,
	input          [7:0] hs_data_in,
	output         [7:0] hs_data_out,
	input                hs_write
);

// ROM chip selects from selector (index 0 only)
wire main_rom_cs_i, sub_rom_cs_i, fgtile_cs_i;
wire sprite0_cs_i, sprite1_cs_i;
wire bgtile0_cs_i, bgtile1_cs_i;
wire prom_r_cs_i, prom_g_cs_i, prom_b_cs_i, prom_unk_cs_i;

// Gate ROM loading to index 0
wire ioctl_wr_cpu = ioctl_wr && (ioctl_index == 8'd0);

// ROM address selector
selector DLSEL
(
	.ioctl_addr(ioctl_addr),
	.main_rom_cs(main_rom_cs_i),
	.sub_rom_cs(sub_rom_cs_i),
	.fgtile_cs(fgtile_cs_i),
	.sprite0_cs(sprite0_cs_i),
	.sprite1_cs(sprite1_cs_i),
	.bgtile0_cs(bgtile0_cs_i),
	.bgtile1_cs(bgtile1_cs_i),
	.prom_r_cs(prom_r_cs_i),
	.prom_g_cs(prom_g_cs_i),
	.prom_b_cs(prom_b_cs_i),
	.prom_unk_cs(prom_unk_cs_i)
);

// Instantiate main CPU board (dual Z80 + AY)
Vastar_CPU main_pcb
(
	.reset(reset),
	.clk_49m(clk_49m),

	.red(video_r),
	.green(video_g),
	.blue(video_b),
	.video_hsync(video_hsync),
	.video_vsync(video_vsync),
	.video_csync(video_csync),
	.video_hblank(video_hblank),
	.video_vblank(video_vblank),
	.ce_pix(ce_pix),

	.p1_controls(p1_controls),
	.p2_controls(p2_controls),
	.sys_controls(sys_controls),

	.dip_sw(dip_sw),
	.sound(sound),

	.h_center(h_center),
	.v_center(v_center),

	.main_rom_cs_i(main_rom_cs_i),
	.sub_rom_cs_i(sub_rom_cs_i),
	.fgtile_cs_i(fgtile_cs_i),
	.sprite0_cs_i(sprite0_cs_i),
	.sprite1_cs_i(sprite1_cs_i),
	.bgtile0_cs_i(bgtile0_cs_i),
	.bgtile1_cs_i(bgtile1_cs_i),
	.prom_r_cs_i(prom_r_cs_i),
	.prom_g_cs_i(prom_g_cs_i),
	.prom_b_cs_i(prom_b_cs_i),
	.prom_unk_cs_i(prom_unk_cs_i),
	.ioctl_addr(ioctl_addr),
	.ioctl_data(ioctl_data),
	.ioctl_wr(ioctl_wr_cpu),

	.pause(pause),

	.hs_address(hs_address),
	.hs_data_out(hs_data_out),
	.hs_data_in(hs_data_in),
	.hs_write(hs_write)
);

endmodule
