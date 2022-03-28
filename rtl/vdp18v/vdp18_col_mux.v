//-----------------------------------------------------------------------------
//
// Synthesizable model of TI's TMS9918A, TMS9928A, TMS9929A.
//
// $Id: vdp18_col_mux.vhd,v 1.10 2006/06/18 10:47:01 arnim Exp $
//
// Color Information Multiplexer
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
//-----------------------------------------------------------------------------

module vdp18_col_mux(
    clk_i,
    clk_en_5m37_i,
    reset_i,
    vert_active_i,
    hor_active_i,
    border_i,
    blank_i,
    hblank_i,
    vblank_i,
    blank_n_o,
    hblank_n_o,
    vblank_n_o,
    reg_col0_i,
    pat_col_i,
    spr0_col_i,
    spr1_col_i,
    spr2_col_i,
    spr3_col_i,
    col_o,
    rgb_r_o,
    rgb_g_o,
    rgb_b_o
);
    
    parameter        compat_rgb_g = 0;
    input            clk_i;
    input            clk_en_5m37_i;
    input            reset_i;
    input            vert_active_i;
    input            hor_active_i;
    input            border_i;
    input            blank_i;
    input            hblank_i;
    input            vblank_i;
    output reg       blank_n_o;
    output reg       hblank_n_o;
    output reg       vblank_n_o;
    input [0:3]      reg_col0_i;
    input [0:3]      pat_col_i;
    input [0:3]      spr0_col_i;
    input [0:3]      spr1_col_i;
    input [0:3]      spr2_col_i;
    input [0:3]      spr3_col_i;
    output [0:3]     col_o;
    output reg [0:7] rgb_r_o;
    output reg [0:7] rgb_g_o;
    output reg [0:7] rgb_b_o;
    
    
    reg [0:3]        col_s;
    
    //---------------------------------------------------------------------------
    // Process col_mux
    //
    // Purpose:
    //   Multiplexes the color information from different sources.
    //
    
    always @(*)
    begin: col_mux
        if (~blank_i)
        begin
            if (hor_active_i & vert_active_i)
            begin
                // priority decoder
                if (spr0_col_i != 4'b0000)
                    col_s <= spr0_col_i;
                else if (spr1_col_i != 4'b0000)
                    col_s <= spr1_col_i;
                else if (spr2_col_i != 4'b0000)
                    col_s <= spr2_col_i;
                else if (spr3_col_i != 4'b0000)
                    col_s <= spr3_col_i;
                else if (pat_col_i != 4'b0000)
                    col_s <= pat_col_i;
                else
                    col_s <= reg_col0_i;
            end
            else
                
                // display border
                col_s <= reg_col0_i;
        end
        else
            
            // blank color channels during horizontal and vertical
            // trace back
            // required to initialize colors for each new scan line
            col_s <= {4{1'b0}};
    end
    //
    //---------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------
    // Process rgb_reg
    //
    // Purpose:
    //   Converts the color information to simple RGB and saves these in
    //   output registers.
    //
   
reg [7:0] compat_rgb_table_c_r [16];
reg [7:0] compat_rgb_table_c_g [16];
reg [7:0] compat_rgb_table_c_b [16];
reg [7:0] full_rgb_table_c_r [16];
reg [7:0] full_rgb_table_c_g [16];
reg [7:0] full_rgb_table_c_b [16];

initial
begin
   // (  0,   0,   0),                    -- Transparent
compat_rgb_table_c_r[0] = 0;
compat_rgb_table_c_g[0] = 0;
compat_rgb_table_c_b[0] = 0;
   // (  0,   0,   0),                    -- Black
compat_rgb_table_c_r[1] = 0;
compat_rgb_table_c_g[1] = 0;
compat_rgb_table_c_b[1] = 0;
   // ( 32, 192,  32),                    -- Medium Green
compat_rgb_table_c_r[2] = 32;
compat_rgb_table_c_g[2] = 192;
compat_rgb_table_c_b[2] = 32;
   // ( 96, 224,  96),                    -- Light Green
compat_rgb_table_c_r[3] = 96;
compat_rgb_table_c_g[3] = 224;
compat_rgb_table_c_b[3] = 96;
  //  ( 32,  32, 224),                    -- Dark Blue
compat_rgb_table_c_r[4] = 32;
compat_rgb_table_c_g[4] = 32;
compat_rgb_table_c_b[4] = 224;
   // ( 64,  96, 224),                    -- Light Blue
compat_rgb_table_c_r[5] = 64;
compat_rgb_table_c_g[5] = 96;
compat_rgb_table_c_b[5] = 224;
    //(160,  32,  32),                    -- Dark Red
compat_rgb_table_c_r[6] = 160;
compat_rgb_table_c_g[6] = 32;
compat_rgb_table_c_b[6] = 32;
   // ( 64, 192, 224),                    -- Cyan
compat_rgb_table_c_r[7] = 64;
compat_rgb_table_c_g[7] = 192;
compat_rgb_table_c_b[7] = 224;
   // (224,  32,  32),                    -- Medium Red
compat_rgb_table_c_r[8] = 224;
compat_rgb_table_c_g[8] = 32;
compat_rgb_table_c_b[8] = 32;
    //(224,  96,  96),                    -- Light Red
compat_rgb_table_c_r[9] = 224;
compat_rgb_table_c_g[9] = 96;
compat_rgb_table_c_b[9] = 96;
   // (192, 192,  32),                    -- Dark Yellow
compat_rgb_table_c_r[10] = 192;
compat_rgb_table_c_g[10] = 192;
compat_rgb_table_c_b[10] = 32;
    //(192, 192, 128),                    -- Light Yellow
compat_rgb_table_c_r[11] = 192;
compat_rgb_table_c_g[11] = 192;
compat_rgb_table_c_b[11] = 128;
    //( 32, 128,  32),                    -- Dark Green
compat_rgb_table_c_r[12] = 32;
compat_rgb_table_c_g[12] = 128;
compat_rgb_table_c_b[12] = 32;
    //(192,  64, 160),                    -- Magenta
compat_rgb_table_c_r[13] = 192;
compat_rgb_table_c_g[13] = 64;
compat_rgb_table_c_b[13] = 160;
    //(160, 160, 160),                    -- Gray
compat_rgb_table_c_r[14] = 160;
compat_rgb_table_c_g[14] = 160;
compat_rgb_table_c_b[14] = 160;
    //(224, 224, 224)                     -- White
compat_rgb_table_c_r[15] = 224;
compat_rgb_table_c_g[15] = 224;
compat_rgb_table_c_b[15] = 224;


   // (  0,   0,   0),                    -- Transparent
full_rgb_table_c_r[0] = 0;
full_rgb_table_c_g[0] = 0;
full_rgb_table_c_b[0] = 0;
   // (  0,   0,   0),                    -- Black
full_rgb_table_c_r[1] = 0;
full_rgb_table_c_g[1] = 0;
full_rgb_table_c_b[1] = 0;
    //( 33, 200,  66),                    -- Medium Green
full_rgb_table_c_r[2] = 32;
full_rgb_table_c_g[2] = 200;
full_rgb_table_c_b[2] = 66;
//    ( 94, 220, 120),                    -- Light Green
full_rgb_table_c_r[3] = 94;
full_rgb_table_c_g[3] = 220;
full_rgb_table_c_b[3] = 120;
//    ( 84,  85, 237),                    -- Dark Blue
full_rgb_table_c_r[4] = 84;
full_rgb_table_c_g[4] = 85;
full_rgb_table_c_b[4] = 237;
//    (125, 118, 252),                    -- Light Blue
full_rgb_table_c_r[5] = 125;
full_rgb_table_c_g[5] = 118;
full_rgb_table_c_b[5] = 252;
 //   (212,  82,  77),                    -- Dark Red
full_rgb_table_c_r[6] = 212;
full_rgb_table_c_g[6] = 82;
full_rgb_table_c_b[6] = 77;
    //( 66, 235, 245),                    -- Cyan
full_rgb_table_c_r[7] = 66;
full_rgb_table_c_g[7] = 235;
full_rgb_table_c_b[7] = 245;
    //(252,  85,  84),                    -- Medium Red
full_rgb_table_c_r[8] = 225;
full_rgb_table_c_g[8] = 85;
full_rgb_table_c_b[8] = 84;
//    (255, 121, 120),                    -- Light Red
full_rgb_table_c_r[9] = 255;
full_rgb_table_c_g[9] = 121;
full_rgb_table_c_b[9] = 120;
//    (212, 193,  84),                    -- Dark Yellow
full_rgb_table_c_r[10] = 212;
full_rgb_table_c_g[10] = 193;
full_rgb_table_c_b[10] = 84;
//    (230, 206, 128),                    -- Light Yellow
full_rgb_table_c_r[11] = 230;
full_rgb_table_c_g[11] = 206;
full_rgb_table_c_b[11] = 128;
//    ( 33, 176,  59),                    -- Dark Green
full_rgb_table_c_r[12] = 33;
full_rgb_table_c_g[12] = 176;
full_rgb_table_c_b[12] = 59;
//    (201,  91, 186),                    -- Magenta
full_rgb_table_c_r[13] = 201;
full_rgb_table_c_g[13] = 91;
full_rgb_table_c_b[13] = 186;
//    (204, 204, 204),                    -- Gray
full_rgb_table_c_r[14] = 204;
full_rgb_table_c_g[14] = 204;
full_rgb_table_c_b[14] = 204;
//    (255, 255, 255)                     -- White
full_rgb_table_c_r[15] = 255;
full_rgb_table_c_g[15] = 255;
full_rgb_table_c_b[15] = 255;
end
	  /*
  constant compat_rgb_table_c : rgb_table_t := (
  -- R  G  B

    (  0,   0,   0),                    -- Transparent
    (  0,   0,   0),                    -- Black
    ( 32, 192,  32),                    -- Medium Green
    ( 96, 224,  96),                    -- Light Green
    ( 32,  32, 224),                    -- Dark Blue
    ( 64,  96, 224),                    -- Light Blue
    (160,  32,  32),                    -- Dark Red
    ( 64, 192, 224),                    -- Cyan
    (224,  32,  32),                    -- Medium Red
    (224,  96,  96),                    -- Light Red
    (192, 192,  32),                    -- Dark Yellow
    (192, 192, 128),                    -- Light Yellow
    ( 32, 128,  32),                    -- Dark Green
    (192,  64, 160),                    -- Magenta
    (160, 160, 160),                    -- Gray
    (224, 224, 224)                     -- White
 */
/*
  -----------------------------------------------------------------------------
  -- Full RGB Value Array
  --
  -- Refer to tms9928a.c of the MAME source distribution.
  --
  constant full_rgb_table_c : rgb_table_t := (
  --   R    G    B
    (  0,   0,   0),                    -- Transparent
    (  0,   0,   0),                    -- Black
    ( 33, 200,  66),                    -- Medium Green
    ( 94, 220, 120),                    -- Light Green
    ( 84,  85, 237),                    -- Dark Blue
    (125, 118, 252),                    -- Light Blue
    (212,  82,  77),                    -- Dark Red
    ( 66, 235, 245),                    -- Cyan
    (252,  85,  84),                    -- Medium Red
    (255, 121, 120),                    -- Light Red
    (212, 193,  84),                    -- Dark Yellow
    (230, 206, 128),                    -- Light Yellow
    ( 33, 176,  59),                    -- Dark Green
    (201,  91, 186),                    -- Magenta
    (204, 204, 204),                    -- Gray
    (255, 255, 255)                     -- White
    );

*/


    always @(posedge clk_i )
    begin: rgb_reg
        reg [3:0]        col_v;
        reg              rgb_r_v;
        reg              rgb_g_v;
        reg              rgb_b_v;
        reg              rgb_table_v;
        if (reset_i)
        begin
            rgb_r_o <= {8{1'b0}};
            rgb_g_o <= {8{1'b0}};
            rgb_b_o <= {8{1'b0}};
        end
        
        else 
        begin
            if (clk_en_5m37_i)
            begin
                // select requested RGB table
                if (compat_rgb_g == 1)
		begin
                rgb_r_v = compat_rgb_table_c_r[col_v];
                rgb_g_v = compat_rgb_table_c_g[col_v];
                rgb_b_v = compat_rgb_table_c_b[col_v];
	        end
                else
		begin
                rgb_r_v = full_rgb_table_c_r[col_v];
                rgb_g_v = full_rgb_table_c_b[col_v];
                rgb_b_v = full_rgb_table_c_g[col_v];
	        end
                
                // assign color to RGB channels
                col_v = col_s;
                //
                rgb_r_o <= rgb_r_v;
                rgb_g_o <= rgb_g_v;
                rgb_b_o <= rgb_b_v;
                blank_n_o <= (~blank_i);
                if (border_i == 1'b0)
                begin
                    hblank_n_o <= hor_active_i;
                    vblank_n_o <= vert_active_i;
                end
                else
                begin
                    hblank_n_o <= (~hblank_i);
                    vblank_n_o <= (~vblank_i);
                end
            end
        end
    end
    
    //
    //---------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------
    // Output mapping
    //---------------------------------------------------------------------------
    assign col_o = col_s;
    
endmodule
