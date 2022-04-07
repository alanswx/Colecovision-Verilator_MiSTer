//-----------------------------------------------------------------------------
//
// FPGA Colecovision
//
// $Id: cv_console.vhd,v 1.13 2006/02/28 22:29:55 arnim Exp $
//
// Toplevel of the Colecovision console
//
// References:
//
//   * Dan Boris' schematics of the Colecovision board
//     http://www.atarihq.com/danb/files/colecovision.pdf
//
//   * Schematics of the Colecovision controller, same source
//     http://www.atarihq.com/danb/files/ColecoController.pdf
//
//   * Technical information, same source
//     http://www.atarihq.com/danb/files/CV-Tech.txt
//
//-----------------------------------------------------------------------------
//
// Copyright (c) 2006, Arnim Laeuger (arnim.laeuger@gmx.net)
//
// All rights reserved
//
// Redistribution and use in source and synthezised forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// Redistributions in synthesized form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// Neither the name of the author nor the names of other contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please report bugs to the author, but before you do so, please
// make sure that this is not a derivative work and that
// you have the latest version of this file.
//
// SystemVerilog conversion (c) 2022 Frank Bruno (fbruno@asicsolutions.com)
//
//-----------------------------------------------------------------------------

module cv_console
  #
  (
   parameter     is_pal_g     = 0,
   parameter     compat_rgb_g = 0
   )
  (
   // Global Interface -------------------------------------------------------
   input         clk_i,
   input         clk_en_10m7_i,
   input         reset_n_i,
   input         sg1000,
   input         dahjeeA_i, // SG-1000 RAM extension at 0x2000-0x3fff
   output        por_n_o,
   // Controller Interface ---------------------------------------------------
   input [1:0]   ctrl_p1_i,
   input [1:0]   ctrl_p2_i,
   input [1:0]   ctrl_p3_i,
   input [1:0]   ctrl_p4_i,
   output [1:0]  ctrl_p5_o,
   input [1:0]   ctrl_p6_i,
   input [1:0]   ctrl_p7_i,
   output [1:0]  ctrl_p8_o,
   input [1:0]   ctrl_p9_i,
   input [7:0]   joy0_i,
   input [7:0]   joy1_i,
   // BIOS ROM Interface -----------------------------------------------------
   output [12:0] bios_rom_a_o,
   output        bios_rom_ce_n_o,
   input [7:0]   bios_rom_d_i,
   // CPU RAM Interface ------------------------------------------------------
   output [14:0] cpu_ram_a_o,
   output        cpu_ram_ce_n_o,
   output        cpu_ram_rd_n_o,
   output        cpu_ram_we_n_o,
   input [7:0]   cpu_ram_d_i,
   output [7:0]  cpu_ram_d_o,
   // Video RAM Interface ----------------------------------------------------
   output [13:0] vram_a_o,
   output        vram_we_o,
   output [7:0]  vram_d_o,
   input [7:0]   vram_d_i,
   // Cartridge ROM Interface ------------------------------------------------
   output [19:0] cart_a_o,
   input [5:0]   cart_pages_i,
   output        cart_en_80_n_o,
   output        cart_en_a0_n_o,
   output        cart_en_c0_n_o,
   output        cart_en_e0_n_o,
   output        cart_rd,
   input [7:0]   cart_d_i,
   output        cart_en_sg1000_n_o,
   // RGB Video Interface ----------------------------------------------------
   input         border_i,
   output [3:0]  col_o,
   output [7:0]  rgb_r_o,
   output [7:0]  rgb_g_o,
   output [7:0]  rgb_b_o,
   output        hsync_n_o,
   output        vsync_n_o,
   output        blank_n_o,
   output        hblank_o,
   output        vblank_o,
   output        comp_sync_n_o,
   // Audio Interface --------------------------------------------------------
   output [10:0] audio_o
   );
  // pragma translate_off
  // pragma translate_on


  // Bus Direction (0 - read , 1 - write)
  // Bus control
  wire           por_n_s;
  wire           reset_n_s;

  wire           clk_en_3m58_p_s;
  wire           clk_en_3m58_n_s;

  // CPU signals
  wire           wait_n_s;
  wire           nmi_n_s;
  wire           int_n_s;
  wire           iorq_n_s;
  wire           m1_n_s;
  reg            m1_wait_q;
  wire           rd_n_s;
  wire           wr_n_s;
  wire           mreq_n_s;
  wire           rfsh_n_s;
  wire [15:0]    a_s;
  wire [7:0]     d_to_cpu_s;
  wire [7:0]     d_from_cpu_s;

  // VDP18 signal
  wire [7:0]     d_from_vdp_s;
  wire           vdp_int_n_s;

  // SN76489 signal
  wire           psg_ready_s;
  wire [7:0]     psg_audio_s;

  // AY-8910 signal
  wire [7:0]     ay_d_s;
  wire [7:0]     ay_ch_a_s;
  wire [7:0]     ay_ch_b_s;
  wire [7:0]     ay_ch_c_s;

  wire [9:0]     audio_mix;

  // Controller signals
  wire [7:0]     d_from_ctrl_s;
  wire [7:0]     d_to_ctrl_s;

  // Address decoder signals
  wire           bios_rom_ce_n_s;
  wire           ram_ce_n_s;
  wire           vdp_r_n_s;
  wire           vdp_w_n_s;
  wire           psg_we_n_s;
  wire           ay_addr_we_n_s;
  wire           ay_data_we_n_s;
  wire           ay_data_rd_n_s;
  wire           ctrl_r_n_s;
  wire           ctrl_en_key_n_s;
  wire           ctrl_en_joy_n_s;
  wire           cart_en_80_n_s;
  wire           cart_en_a0_n_s;
  wire           cart_en_c0_n_s;
  wire           cart_en_e0_n_s;
  wire [5:0]     cart_page_s;

  wire           cart_en_sg1000_n_s;
  // misc signals
  wire           vdd_s;

  // pragma translate_off
  // pragma translate_on

  assign vdd_s = 1'b1;
  assign audio_o = ({1'b0, psg_audio_s, 2'b00}) + ay_ch_a_s + ay_ch_b_s + ay_ch_c_s;

  assign int_n_s = (sg1000 == 1'b0) ? 1'b1 :
                   vdp_int_n_s;
  assign nmi_n_s = (sg1000 == 1'b0) ? vdp_int_n_s :
                   joy0_i[7] & joy1_i[7];



  //---------------------------------------------------------------------------
  // Reset generation
  //---------------------------------------------------------------------------

  cv_por por_b(
               .clk_i(clk_i),
               .por_n_o(por_n_s)
               );
  assign por_n_o = por_n_s;
  assign reset_n_s = por_n_s & reset_n_i;

  //---------------------------------------------------------------------------
  // Clock generation
  //---------------------------------------------------------------------------

  cv_clock clock_b(
                   .clk_i(clk_i),
                   .clk_en_10m7_i(clk_en_10m7_i),
                   .reset_n_i(reset_n_s),
                   .clk_en_3m58_p_o(clk_en_3m58_p_s),
                   .clk_en_3m58_n_o(clk_en_3m58_n_s)
                   );

  //---------------------------------------------------------------------------
  // T80 CPU
  //---------------------------------------------------------------------------

  //`ifdef VERILATOR

  tv80e Cpu
    (
     .reset_n(reset_n_s),
     .clk(clk_i),
     .cen(clk_en_3m58_p_s),
     //.cen_n(clk_en_3m58_n_s),
     .wait_n(wait_n_s),
     .int_n(int_n_s),
     .nmi_n(nmi_n_s),
     .busrq_n(vdd_s),
     .m1_n(m1_n_s),
     .mreq_n(mreq_n_s),
     .iorq_n(iorq_n_s),
     .rd_n(rd_n_s),
     .wr_n(wr_n_s),
     .rfsh_n(rfsh_n_s),
     .halt_n(),
     .busak_n(),
     .A(a_s),
     .di(d_to_cpu_s),
     .dout(d_from_cpu_s),
     .dirset (0)
     );
  //`else
`ifdef NO
  T80pa #(.mode(0)) t80a_b(
                           .reset_n(reset_n_s),
                           .clk(clk_i),
                           .cen_p(clk_en_3m58_p_s),
                           .cen_n(clk_en_3m58_n_s),
                           .wait_n(wait_n_s),
                           .int_n(int_n_s),
                           .nmi_n(nmi_n_s),
                           .busrq_n(vdd_s),
                           .m1_n(m1_n_s),
                           .mreq_n(mreq_n_s),
                           .iorq_n(iorq_n_s),
                           .rd_n(rd_n_s),
                           .wr_n(wr_n_s),
                           .rfsh_n(rfsh_n_s),
                           .halt_n(),
                           .busak_n(),
                           .a(a_s),
                           .di(d_to_cpu_s),
                           .do(d_from_cpu_s)
                           );

`endif


  YM2149 ym2149_inst(
                     .CLK(clk_i),
                     .CE(clk_en_3m58_p_s),
                     .RESET((~reset_n_s)),
                     .BDIR((~ay_addr_we_n_s) | (~ay_data_we_n_s)),
                     .BC((~ay_addr_we_n_s) | (~ay_data_rd_n_s)),
                     .DI(d_from_cpu_s),
                     .DO(ay_d_s),
                     .CHANNEL_A(ay_ch_a_s),
                     .CHANNEL_B(ay_ch_b_s),
                     .CHANNEL_C(ay_ch_c_s),

                     .SEL(1'b0),
                     .MODE(1'b0),

                     .ACTIVE(),

                     .IOA_in(1'b0),
                     .IOA_out(),

                     .IOB_in(1'b0),
                     .IOB_out()
                     );

  //---------------------------------------------------------------------------
  // Process m1_wait
  //
  // Purpose:
  //   Implements flip-flop U8A which asserts a wait states controlled by M1.
  //

  always @(posedge clk_i or negedge reset_n_s or posedge m1_n_s)
    begin: m1_wait
      if (reset_n_s == 1'b0 | m1_n_s == 1'b1)
        m1_wait_q <= 1'b0;
      else
        begin
          if (clk_en_3m58_p_s == 1'b1)
            m1_wait_q <= (~m1_wait_q);
        end
    end

  assign wait_n_s = psg_ready_s & (~m1_wait_q);

  //
  //---------------------------------------------------------------------------

  //---------------------------------------------------------------------------
  // TMS9928A Video Display Processor
  //---------------------------------------------------------------------------

  vdp18_core #(.is_pal_g(is_pal_g), .compat_rgb_g(compat_rgb_g)) vdp18_b(
                                                                         .clk_i(clk_i),
                                                                         .clk_en_10m7_i(clk_en_10m7_i),
                                                                         .reset_n_i(reset_n_s),
                                                                         .csr_n_i(vdp_r_n_s),
                                                                         .csw_n_i(vdp_w_n_s),
                                                                         .mode_i(a_s[0]),
                                                                         .int_n_o(vdp_int_n_s),
                                                                         .cd_i(d_from_cpu_s),
                                                                         .cd_o(d_from_vdp_s),
                                                                         .vram_we_o(vram_we_o),
                                                                         .vram_a_o(vram_a_o),
                                                                         .vram_d_o(vram_d_o),
                                                                         .vram_d_i(vram_d_i),
                                                                         .col_o(col_o),
                                                                         .rgb_r_o(rgb_r_o),
                                                                         .rgb_g_o(rgb_g_o),
                                                                         .rgb_b_o(rgb_b_o),
                                                                         .hsync_n_o(hsync_n_o),
                                                                         .vsync_n_o(vsync_n_o),
                                                                         .blank_n_o(blank_n_o),
                                                                         .border_i(border_i),
                                                                         .hblank_o(hblank_o),
                                                                         .vblank_o(vblank_o),
                                                                         .comp_sync_n_o(comp_sync_n_o)
                                                                         );

  //---------------------------------------------------------------------------
  // SN76489 Programmable Sound Generator
  //---------------------------------------------------------------------------
//`ifdef VERILATOR
//  assign psg_ready_s = 1'b1;
//`else
  sn76489_top #(.clock_div_16_g(1)) psg_b(
                                          .clock_i(clk_i),
                                          .clock_en_i(clk_en_3m58_p_s),
                                          .res_n_i(reset_n_s),
                                          .ce_n_i(psg_we_n_s),
                                          .we_n_i(psg_we_n_s),
                                          .ready_o(psg_ready_s),
                                          .d_i(d_from_cpu_s),
                                          .aout_o(psg_audio_s)
                                          );
//`endif
  //---------------------------------------------------------------------------
  // Controller ports
  //---------------------------------------------------------------------------

  cv_ctrl ctrl_b(
                 .clk_i(clk_i),
                 .clk_en_3m58_i(clk_en_3m58_p_s),
                 .reset_n_i(reset_n_s),
                 .ctrl_en_key_n_i(ctrl_en_key_n_s),
                 .ctrl_en_joy_n_i(ctrl_en_joy_n_s),
                 .a1_i(a_s[1]),
                 .ctrl_p1_i(ctrl_p1_i),
                 .ctrl_p2_i(ctrl_p2_i),
                 .ctrl_p3_i(ctrl_p3_i),
                 .ctrl_p4_i(ctrl_p4_i),
                 .ctrl_p5_o(ctrl_p5_o),
                 .ctrl_p6_i(ctrl_p6_i),
                 .ctrl_p7_i(ctrl_p7_i),
                 .ctrl_p8_o(ctrl_p8_o),
                 .ctrl_p9_i(ctrl_p9_i),
                 .d_o(d_from_ctrl_s)
                 );

  //---------------------------------------------------------------------------
  // Address decoder
  //---------------------------------------------------------------------------

  cv_addr_dec addr_dec_b(
                         .clk_i(clk_i),
                         .reset_n_i(reset_n_i),
                         .sg1000(sg1000),
                         .dahjeeA_i(dahjeeA_i),
                         .a_i(a_s),
                         .d_i(d_from_cpu_s),
                         .cart_pages_i(cart_pages_i),
                         .cart_page_o(cart_page_s),
                         .iorq_n_i(iorq_n_s),
                         .rd_n_i(rd_n_s),
                         .wr_n_i(wr_n_s),
                         .mreq_n_i(mreq_n_s),
                         .rfsh_n_i(rfsh_n_s),
                         .bios_rom_ce_n_o(bios_rom_ce_n_s),
                         .ram_ce_n_o(ram_ce_n_s),
                         .vdp_r_n_o(vdp_r_n_s),
                         .vdp_w_n_o(vdp_w_n_s),
                         .psg_we_n_o(psg_we_n_s),
                         .ay_addr_we_n_o(ay_addr_we_n_s),
                         .ay_data_we_n_o(ay_data_we_n_s),
                         .ay_data_rd_n_o(ay_data_rd_n_s),
                         .ctrl_r_n_o(ctrl_r_n_s),
                         .ctrl_en_key_n_o(ctrl_en_key_n_s),
                         .ctrl_en_joy_n_o(ctrl_en_joy_n_s),
                         .cart_en_80_n_o(cart_en_80_n_s),
                         .cart_en_a0_n_o(cart_en_a0_n_s),
                         .cart_en_c0_n_o(cart_en_c0_n_s),
                         .cart_en_e0_n_o(cart_en_e0_n_s),
                         .cart_en_sg1000_n_o(cart_en_sg1000_n_s)
                         );

  assign bios_rom_ce_n_o = bios_rom_ce_n_s;
  assign cpu_ram_ce_n_o = ram_ce_n_s;
  assign cpu_ram_we_n_o = wr_n_s;
  assign cpu_ram_rd_n_o = rd_n_s;
  assign cart_en_80_n_o = cart_en_80_n_s;
  assign cart_en_a0_n_o = cart_en_a0_n_s;
  assign cart_en_c0_n_o = cart_en_c0_n_s;
  assign cart_en_e0_n_o = cart_en_e0_n_s;
  assign cart_en_sg1000_n_o = cart_en_sg1000_n_s;
  assign cart_rd = (~(cart_en_80_n_s & cart_en_a0_n_s & cart_en_c0_n_s & cart_en_e0_n_s & cart_en_sg1000_n_s));

  //---------------------------------------------------------------------------
  // Bus multiplexer
  //---------------------------------------------------------------------------

  assign d_to_ctrl_s = (sg1000 == 1'b0) ? d_from_ctrl_s :
                       (a_s[0] == 1'b0) ? {joy1_i[2], joy1_i[3], joy0_i[5], joy0_i[4], joy0_i[0], joy0_i[1], joy0_i[2], joy0_i[3]} :
                       {3'b111, reset_n_i, joy1_i[5], joy1_i[4], joy1_i[0], joy1_i[1]};


  cv_bus_mux bus_mux_b(
                       .bios_rom_ce_n_i(bios_rom_ce_n_s),
                       .ram_ce_n_i(ram_ce_n_s),
                       .vdp_r_n_i(vdp_r_n_s),
                       .ctrl_r_n_i(ctrl_r_n_s),
                       .cart_en_80_n_i(cart_en_80_n_s),
                       .cart_en_a0_n_i(cart_en_a0_n_s),
                       .cart_en_c0_n_i(cart_en_c0_n_s),
                       .cart_en_e0_n_i(cart_en_e0_n_s),
                       .cart_en_sg1000_n_i(cart_en_sg1000_n_s),
                       .ay_data_rd_n_i(ay_data_rd_n_s),
                       .bios_rom_d_i(bios_rom_d_i),
                       .cpu_ram_d_i(cpu_ram_d_i),
                       .vdp_d_i(d_from_vdp_s),
                       .ctrl_d_i(d_to_ctrl_s),
                       .cart_d_i(cart_d_i),
                       .ay_d_i(ay_d_s),
                       .d_o(d_to_cpu_s)
                       );

  //---------------------------------------------------------------------------
  // Misc outputs
  //---------------------------------------------------------------------------
  assign bios_rom_a_o = a_s[12:0];
  assign cpu_ram_a_o = a_s[14:0];
  assign cpu_ram_d_o = d_from_cpu_s;
  assign cart_a_o = (sg1000 == 1'b0) ? {cart_page_s, a_s[13:0]} :
                    {4'b0000, a_s[15:0]};

endmodule
