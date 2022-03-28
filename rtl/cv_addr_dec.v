//-----------------------------------------------------------------------------
//
// FPGA Colecovision
//
// $Id: cv_addr_dec.vhd,v 1.3 2006/01/05 22:22:29 arnim Exp $
//
// Address Decoder
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

module cv_addr_dec(
    clk_i,
    reset_n_i,
    sg1000,
    dahjeeA_i,
    a_i,
    d_i,
    cart_pages_i,
    cart_page_o,
    iorq_n_i,
    rd_n_i,
    wr_n_i,
    mreq_n_i,
    rfsh_n_i,
    bios_rom_ce_n_o,
    ram_ce_n_o,
    vdp_r_n_o,
    vdp_w_n_o,
    psg_we_n_o,
    ay_addr_we_n_o,
    ay_data_we_n_o,
    ay_data_rd_n_o,
    ctrl_r_n_o,
    ctrl_en_key_n_o,
    ctrl_en_joy_n_o,
    cart_en_80_n_o,
    cart_en_a0_n_o,
    cart_en_c0_n_o,
    cart_en_e0_n_o,
    cart_en_sg1000_n_o
);
    
    input            clk_i;
    input            reset_n_i;
    input            sg1000;
    input            dahjeeA_i;
    input [15:0]     a_i;
    input [7:0]      d_i;
    input [5:0]      cart_pages_i;
    output reg [5:0] cart_page_o;
    input            iorq_n_i;
    input            rd_n_i;
    input            wr_n_i;
    input            mreq_n_i;
    input            rfsh_n_i;
    output reg       bios_rom_ce_n_o;
    output reg       ram_ce_n_o;
    output reg       vdp_r_n_o;
    output reg       vdp_w_n_o;
    output reg       psg_we_n_o;
    output reg       ay_addr_we_n_o;
    output reg       ay_data_we_n_o;
    output reg       ay_data_rd_n_o;
    output reg       ctrl_r_n_o;
    output reg       ctrl_en_key_n_o;
    output reg       ctrl_en_joy_n_o;
    output reg       cart_en_80_n_o;
    output reg       cart_en_a0_n_o;
    output reg       cart_en_c0_n_o;
    output reg       cart_en_e0_n_o;
    output reg       cart_en_sg1000_n_o;
    
    
    reg              megacart_en;
    reg [5:0]        megacart_page;
    reg              bios_en;
    
    //---------------------------------------------------------------------------
    // Process dec
    //
    // Purpose:
    //   Implements the address decoding logic.
    //
    
    always @(a_i or iorq_n_i or rd_n_i or wr_n_i or mreq_n_i or rfsh_n_i or cart_pages_i or bios_en or sg1000 or dahjeeA_i or megacart_en or megacart_page)
    begin: dec
        reg [2:0]        mux_v;
        // default assignments
        bios_rom_ce_n_o <= 1'b1;
        ram_ce_n_o <= 1'b1;
        vdp_r_n_o <= 1'b1;
        vdp_w_n_o <= 1'b1;
        psg_we_n_o <= 1'b1;
        ay_addr_we_n_o <= 1'b1;
        ay_data_we_n_o <= 1'b1;
        ay_data_rd_n_o <= 1'b1;
        ctrl_r_n_o <= 1'b1;
        ctrl_en_key_n_o <= 1'b1;
        ctrl_en_joy_n_o <= 1'b1;
        cart_en_80_n_o <= 1'b1;
        cart_en_a0_n_o <= 1'b1;
        cart_en_c0_n_o <= 1'b1;
        cart_en_e0_n_o <= 1'b1;
        cart_en_sg1000_n_o <= 1'b1;
        
        //  64k
        // 128k
        // 256k
        // 512k
        if (sg1000 == 1'b0 & (cart_pages_i == 6'b000011 | cart_pages_i == 6'b000111 | cart_pages_i == 6'b001111 | cart_pages_i == 6'b011111 | cart_pages_i == 6'b111111))		// 1M
            megacart_en <= 1'b1;
        else
            megacart_en <= 1'b0;
        
        // Paging
        case (a_i[15:14])
            2'b10 :
                if (megacart_en == 1'b1)
                    cart_page_o <= cart_pages_i;
                else
                    cart_page_o <= 6'b000000;
            2'b11 :
                if (megacart_en == 1'b1)
                    cart_page_o <= megacart_page;
                else
                    cart_page_o <= 6'b000001;
            default :
                cart_page_o <= 6'b000000;
        endcase
        
        // Memory access ----------------------------------------------------------
        if (mreq_n_i == 1'b0 & rfsh_n_i == 1'b1)
        begin
            if (sg1000 == 1'b1)
            begin
                if (a_i[15:14] == 2'b11)		// c000 - ffff
                    ram_ce_n_o <= 1'b0;
                else if (a_i[15:13] == 3'b001 & dahjeeA_i == 1'b1)		// 2000 - 3fff
                    ram_ce_n_o <= 1'b0;
                else
                    cart_en_sg1000_n_o <= 1'b0;
            end
            else
                case (a_i[15:13])
                    3'b000 :
                        if (bios_en == 1'b1)
                            bios_rom_ce_n_o <= 1'b0;
                        else
                            ram_ce_n_o <= 1'b0;
                    3'b001, 3'b010, 3'b011 :
                        ram_ce_n_o <= 1'b0;		// 2000 - 7fff = 24k
                    3'b100 :
                        cart_en_80_n_o <= 1'b0;
                    3'b101 :
                        cart_en_a0_n_o <= 1'b0;
                    3'b110 :
                        cart_en_c0_n_o <= 1'b0;
                    3'b111 :
                        cart_en_e0_n_o <= 1'b0;
                    default :
                        ;
                endcase
        end
        
        // I/O access -------------------------------------------------------------
        if (iorq_n_i == 1'b0)
        begin
            if (sg1000 == 1'b0 & a_i[7] == 1'b1)
            begin
                mux_v = {a_i[6], a_i[5], wr_n_i};
                case (mux_v)
                    3'b000 :
                        ctrl_en_key_n_o <= 1'b0;
                    3'b010 :
                        vdp_w_n_o <= 1'b0;
                    3'b011 :
                        if (rd_n_i == 1'b0)
                            vdp_r_n_o <= 1'b0;
                    3'b100 :
                        ctrl_en_joy_n_o <= 1'b0;
                    3'b110 :
                        psg_we_n_o <= 1'b0;
                    3'b111 :
                        if (rd_n_i == 1'b0)
                            ctrl_r_n_o <= 1'b0;
                    default :
                        ;
                endcase
            end
            
            if (sg1000 == 1'b1)
            begin
                mux_v = {a_i[7], a_i[6], wr_n_i};
                case (mux_v)
                    3'b010 :
                        psg_we_n_o <= 1'b0;
                    3'b100 :
                        vdp_w_n_o <= 1'b0;
                    3'b101 :
                        if (rd_n_i == 1'b0)
                            vdp_r_n_o <= 1'b0;
                    3'b111 :
                        if (rd_n_i == 1'b0)
                            ctrl_r_n_o <= 1'b0;
                    default :
                        ;
                endcase
            end
            
            if (a_i[7:0] == 8'h50 & wr_n_i == 1'b0)
                ay_addr_we_n_o <= 1'b0;
            else if (a_i[7:0] == 8'h51 & wr_n_i == 1'b0)
                ay_data_we_n_o <= 1'b0;
            else if (a_i[7:0] == 8'h52 & rd_n_i == 1'b0)
                ay_data_rd_n_o <= 1'b0;
        end
    end
    
    //
    //---------------------------------------------------------------------------
    
    always @(negedge reset_n_i or posedge clk_i)
    begin: megacart
        if (reset_n_i == 1'b0)
        begin
            megacart_page <= 6'b000000;
            bios_en <= 1'b1;
        end
        else 
        begin
            // MegaCart paging
            if (megacart_en == 1'b1 & rfsh_n_i == 1'b1 & mreq_n_i == 1'b0 & rd_n_i == 1'b0 & a_i[15:6] == {8'hFF, 2'b11})
                megacart_page <= a_i[5:0] & cart_pages_i;
            
            // SGM BIOS enable/disable
            if (sg1000 == 1'b1)
                bios_en <= 1'b0;
            else if (iorq_n_i == 1'b0 & mreq_n_i == 1'b1 & rfsh_n_i == 1'b1 & wr_n_i == 1'b0 & a_i[7:0] == 8'h7f)
                bios_en <= d_i[1];
        end
    end
    
endmodule
