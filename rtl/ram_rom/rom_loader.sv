//============================================================================
//
//  SD card ROM loader and ROM selector for Vastar MiSTer.
//  Original framework Copyright (C) 2019, 2020 Kitrinx (aka Rysha)
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

// ROM layout for Vastar (index 0 - all CPU + GFX ROMs):
// 0x00000 - 0x07FFF = main_rom  (maincpu: 8 x 4KB ROMs, e_f4..e_n5, fully populated)
// 0x08000 - 0x09FFF = sub_rom   (sub CPU: e_f2.rom @ 0x0000, e_j2.rom @ 0x1000)
// 0x0A000 - 0x0BFFF = fgtile    (c_c9.rom, 8KB)
// 0x0C000 - 0x0DFFF = sprite0   (c_f7.rom, 8KB)
// 0x0E000 - 0x0FFFF = sprite1   (c_f9.rom, 8KB)
// 0x10000 - 0x11FFF = bgtile0   (c_n4.rom, 8KB)
// 0x12000 - 0x13FFF = bgtile1   (c_s4.rom, 8KB)
// 0x14000 - 0x140FF = prom_r    (tbp24s10.6p - red PROM,   256 bytes)
// 0x14100 - 0x141FF = prom_g    (tbp24s10.6s - green PROM, 256 bytes)
// 0x14200 - 0x142FF = prom_b    (tbp24s10.6m - blue PROM,  256 bytes)
// 0x14300 - 0x143FF = prom_unk  (tbp24s10.8n - unknown,    256 bytes)
// Total: 0x14400 = 82,944 bytes

module selector
(
    input  logic [24:0] ioctl_addr,
    output logic        main_rom_cs,  // 0x00000-0x07FFF (32KB address space, 8KB each ROM × 8)
    output logic        sub_rom_cs,   // 0x08000-0x09FFF (8KB: 2 × 4KB)
    output logic        fgtile_cs,    // 0x0A000-0x0BFFF (8KB)
    output logic        sprite0_cs,   // 0x0C000-0x0DFFF (8KB)
    output logic        sprite1_cs,   // 0x0E000-0x0FFFF (8KB)
    output logic        bgtile0_cs,   // 0x10000-0x11FFF (8KB)
    output logic        bgtile1_cs,   // 0x12000-0x13FFF (8KB)
    output logic        prom_r_cs,    // 0x14000-0x140FF (256B)
    output logic        prom_g_cs,    // 0x14100-0x141FF (256B)
    output logic        prom_b_cs,    // 0x14200-0x142FF (256B)
    output logic        prom_unk_cs   // 0x14300-0x143FF (256B)
);
    always_comb begin
        {main_rom_cs, sub_rom_cs, fgtile_cs, sprite0_cs, sprite1_cs,
         bgtile0_cs, bgtile1_cs, prom_r_cs, prom_g_cs, prom_b_cs, prom_unk_cs} = 11'd0;

        if      (ioctl_addr < 25'h08000) main_rom_cs = 1;
        else if (ioctl_addr < 25'h0A000) sub_rom_cs  = 1;
        else if (ioctl_addr < 25'h0C000) fgtile_cs   = 1;
        else if (ioctl_addr < 25'h0E000) sprite0_cs  = 1;
        else if (ioctl_addr < 25'h10000) sprite1_cs  = 1;
        else if (ioctl_addr < 25'h12000) bgtile0_cs  = 1;
        else if (ioctl_addr < 25'h14000) bgtile1_cs  = 1;
        else if (ioctl_addr < 25'h14100) prom_r_cs   = 1;
        else if (ioctl_addr < 25'h14200) prom_g_cs   = 1;
        else if (ioctl_addr < 25'h14300) prom_b_cs   = 1;
        else if (ioctl_addr < 25'h14400) prom_unk_cs = 1;
    end
endmodule

////////////
// EPROMS //
////////////

// Main CPU ROM — 32KB (15-bit address), all 8KB populated at 0x0000-0x7FFF
module eprom_32k
(
    input  logic        CLK,
    input  logic        CLK_DL,
    input  logic [14:0] ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(15)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[14:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[14:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule

// Generic 8KB ROM module (13-bit address) — sub CPU ROM and graphics ROMs
module eprom_8k
(
    input  logic        CLK,
    input  logic        CLK_DL,
    input  logic [12:0] ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(13)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[12:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[12:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule

// 256-byte ROM module (8-bit address) — color PROMs
module eprom_256b
(
    input  logic       CLK,
    input  logic       CLK_DL,
    input  logic [7:0] ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(8)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[7:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[7:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule

// Generic 4KB ROM module (12-bit address) — kept for future use
module eprom_4k
(
    input  logic        CLK,
    input  logic        CLK_DL,
    input  logic [11:0] ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(12)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[11:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[11:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule
