//-----------------------------------------------------------------------------
//
// Synthesizable model of TI's TMS9918A, TMS9928A, TMS9929A.
//
// $Id: vdp18_ctrl.vhd,v 1.26 2006/06/18 10:47:01 arnim Exp $
//
// Timing Controller
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

module vdp18_ctrl(
    clk_i,
    clk_en_5m37_i,
    reset_i,
    opmode_i,
    num_pix_i,
    num_line_i,
    vert_inc_i,
    reg_blank_i,
    reg_size1_i,
    stop_sprite_i,
    clk_en_acc_o,
    access_type_o,
    vert_active_o,
    hor_active_o,
    irq_o
);
    
    input      clk_i;
    input      clk_en_5m37_i;
    input      reset_i;
    input      [0:1] opmode_i;
    input      signed [0:8] num_pix_i;
    input      signed [0:8] num_line_i;
    input      vert_inc_i;
    input      reg_blank_i;
    input      reg_size1_i;
    input      stop_sprite_i;
    output     clk_en_acc_o;
    output     [0:7]   access_type_o;
    output     vert_active_o;
    output     hor_active_o;
    output     irq_o;
 
   
   `include "constants.vh"

    //---------------------------------------------------------------------------
    // This enables a workaround for a bug in XST.
    // ISE 8.1.02i implements wrong functionality otherwise :-(
    //
    parameter  xst_bug_wa_c = 1'b1;
    //
    //---------------------------------------------------------------------------
    
    reg        [0:7]  access_type_s;
    
    // pragma translate_off
    // Testbench signals --------------------------------------------------------
    //
    wire [3:0] ac_s;
    //
    //---------------------------------------------------------------------------
    // pragma translate_on
    
    reg        vert_active_q;
    reg        hor_active_q;
    reg        sprite_active_q;
    reg        sprite_line_act_q;
    
    // pragma translate_off
    // Testbench signals --------------------------------------------------------
    //
    assign ac_s = enum_to_vec_f(access_type_s);
    //
    //---------------------------------------------------------------------------
    // pragma translate_on
    
    //---------------------------------------------------------------------------
    // Process decode_access
    //
    // Purpose:
    //   Decode horizontal counter value to access type.
    //
    
    always @(*)
    begin: decode_access
        reg        signed [0:8] num_pix_plus_6_v;
        reg        signed [0:8] mod_6_v;
        reg        signed [0:8] num_pix_plus_8_v;
        reg        signed [0:8] num_pix_plus_32_v;
        integer    num_pix_spr_v;
        // default assignment
        access_type_s <= AC_CPU;
        
        // prepare number of pixels for pattern operations
        num_pix_plus_6_v = num_pix_i + 6;
        num_pix_plus_8_v = num_pix_i + 8;
        num_pix_plus_32_v = num_pix_i + 32;
        num_pix_spr_v = ({9{num_pix_i}} & 9'b111111110);
        
        case (opmode_i)
            // Graphics I, II and Multicolor Mode -----------------------------------
            OPMODE_GRAPH1, OPMODE_GRAPH2, OPMODE_MULTIC :
                begin
                    //
                    // Patterns
                    //
                    if (vert_active_q)
                    begin
                        if (num_pix_plus_8_v[0] == 1'b0)
                        begin
                            if (~xst_bug_wa_c)
                                
                                // original code, we want this
                                case (num_pix_plus_8_v[6:7])
                                    2'b01 :
                                        access_type_s <= AC_PNT;
                                    2'b10 :
                                        if (opmode_i != OPMODE_MULTIC)
                                            // no access to pattern color table in multicolor mode
                                            access_type_s <= AC_PCT;
                                    2'b11 :
                                        access_type_s <= AC_PGT;
                                    default :
                                        ;
                                endcase
                            else
                                
                                // workaround for XST bug, we need this
                                if (num_pix_plus_8_v[6:7] == 2'b01)
                                    access_type_s <= AC_PNT;
                                else if (num_pix_plus_8_v[6:7] == 2'b10)
                                begin
                                    if (opmode_i != OPMODE_MULTIC)
                                        access_type_s <= AC_PCT;
                                end
                                else if (num_pix_plus_8_v[6:7] == 2'b11)
                                    access_type_s <= AC_PGT;
                        end
                    end
                    
                    //
                    // Sprite test
                    //
                    if (sprite_line_act_q)
                    begin
                        if (num_pix_i[0] == 1'b0 & num_pix_i[0:5] != 6'b011111 & num_pix_i[6:7] == 2'b00 & num_pix_i[4:5] != 2'b00)
                            // sprite test interleaved with pattern accesses - 23 slots
                            access_type_s <= AC_STST;
                        if (num_pix_plus_32_v[0:4] == 5'b00000 | num_pix_plus_32_v[0:7] == 8'b00001000)
                            // sprite tests before starting pattern phase - 9 slots
                            access_type_s <= AC_STST;
                        
                        //
                        // Sprite Attribute Table and Sprite Pattern Table
                        //
                        case (num_pix_spr_v)
                            250, -78, -62, -46 :
                                access_type_s <= AC_SATY;
                            254, -76, -60, -44 :
                                access_type_s <= AC_SATX;
                            252, -74, -58, -42 :
                                access_type_s <= AC_SATN;
                            -86, -70, -54, -38 :
                                access_type_s <= AC_SATC;
                            -84, -68, -52, -36 :
                                access_type_s <= AC_SPTH;
                            -82, -66, -50, -34 :
                                if (reg_size1_i)
                                    access_type_s <= AC_SPTL;
                            default :
                                ;
                        endcase
                    end
                end
            
            // Text Mode ------------------------------------------------------------
            OPMODE_TEXTM :
                if (vert_active_q & num_pix_plus_6_v[0] == 1'b0 & num_pix_plus_6_v[0:4] != 5'b01111)
                begin
                    mod_6_v = mod_6_f(num_pix_plus_6_v);
                    case (mod_6_v[6:7])
                        2'b00 :
                            access_type_s <= AC_PNT;
                        2'b10 :
                            access_type_s <= AC_PGT;
                        default :
                            ;
                    endcase
                end
            
            // Unknown --------------------------------------------------------------
            default :
                ;
        endcase
    end
    
    //
    //---------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------
    // Process vert_flags
    //
    // Purpose:
    //   Track the vertical position with flags.
    //
    
    always @(posedge clk_i )
    begin: vert_flags
        if (reset_i)
        begin
            vert_active_q <= 1'b0;
            sprite_active_q <= 1'b0;
            sprite_line_act_q <= 1'b0;
        end
        
        else 
        begin
            if (clk_en_5m37_i)
            begin
                // line-local sprite processing
                if (sprite_active_q)
                begin
                    // sprites are globally enabled
                    if (vert_inc_i)
                        // reload at beginning of every new line
                        // => scan with STST
                        sprite_line_act_q <= 1'b1;
                    
                    if (num_pix_i == hv_sprite_start_c)
                        // reload when access to sprite memory starts
                        sprite_line_act_q <= 1'b1;
                end
                
                if (vert_inc_i)
                begin
                    // global sprite processing
                    if (reg_blank_i)
                    begin
                        sprite_active_q <= 1'b0;
                        sprite_line_act_q <= 1'b0;
                    end
                    else if (num_line_i == -2)
                    begin
                        // start at line -1
                        sprite_active_q <= 1'b1;
                        // initialize immediately
                        sprite_line_act_q <= 1'b1;
                    end
                    else if (num_line_i == 191)
                    begin
                        // stop at line 192
                        sprite_active_q <= 1'b0;
                        // force stop
                        sprite_line_act_q <= 1'b0;
                    end
                    
                    // global vertical display
                    if (reg_blank_i)
                        vert_active_q <= 1'b0;
                    else if (num_line_i == -1)
                        // start vertical display at line 0
                        vert_active_q <= 1'b1;
                    else if (num_line_i == 191)
                        // stop at line 192
                        vert_active_q <= 1'b0;
                end
                
                if (stop_sprite_i)
                    // stop processing of sprites in this line
                    sprite_line_act_q <= 1'b0;
            end
        end
    end
    
    //
    //---------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------
    // Process hor_flags
    //
    // Purpose:
    //   Track the horizontal position.
    //
    
    always @(posedge clk_i )
    begin: hor_flags
        if (reset_i)
            hor_active_q <= 1'b0;
        
        else 
        begin
            if (clk_en_5m37_i)
            begin
                if ((~reg_blank_i) & num_pix_i == -1)
                    hor_active_q <= 1'b1;
                
                if (opmode_i == OPMODE_TEXTM)
                begin
                    if (num_pix_i == 239)
                        hor_active_q <= 1'b0;
                end
                else
                    if (num_pix_i == 255)
                        hor_active_q <= 1'b0;
            end
        end
    end
    //
    //---------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------
    // Ouput mapping
    //---------------------------------------------------------------------------
    // generate clock enable for flip-flops working on access_type
    assign clk_en_acc_o = clk_en_5m37_i & num_pix_i[8] == 1'b1;
    assign access_type_o = access_type_s;
    assign vert_active_o = vert_active_q;
    assign hor_active_o = hor_active_q;
    assign irq_o = vert_inc_i & num_line_i == 191;
    
endmodule
