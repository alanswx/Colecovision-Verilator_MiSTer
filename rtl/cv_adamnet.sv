//-----------------------------------------------------------------------------
//
// FPGA Colecovision AdamNet
//
// Adamnet implementation for Disk, Tape and Printer
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
// Copyright (c) 2022, Frank Bruno (fbruno@asicsolutions.com)
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

module cv_adamnet
  #
  (
   parameter NUM_DISKS = 1,
   parameter NUM_TAPES = 1,
   parameter USE_REQ   = 1
   )
  (
   input                       clk_i,
   input                       adam_reset_pcb_n_i /*verilator public_flat*/,
   input                       z80_wr/*verilator public_flat*/,
   input                       z80_rd/*verilator public_flat*/,
   input [15:0]                z80_addr/*verilator public_flat*/,
   input [7:0]                 z80_data_wr/*verilator public_flat*/,
   input [7:0]                 z80_data_rd/*verilator public_flat*/,

   // Dual port or mux into ADAM system memory
   output logic [15:0]         ramb_addr,
   output logic                ramb_wr,
   output logic                ramb_rd,
   output logic [7:0]          ramb_dout,
   input [7:0]                 ramb_din,
   input                       ramb_wr_ack,
   input                       ramb_rd_ack,

   // Keyboard interface. Not sure how we should do this
   output logic [7:0]          kbd_status,
   output logic                kbd_status_upd,
   output logic                lastkey_in_valid,
   input logic [7:0]           lastkey_in,
   output logic                lastkey_in_ack,
   output logic [7:0]          lastkey_out,

   // Disk interface
   input logic [NUM_DISKS-1:0] disk_present,
   input logic [NUM_TAPES-1:0] tape_present,
   output logic [31:0]         disk_sector, // sector
   output logic                disk_load, // load the 512 byte sector
   input logic                 disk_sector_loaded, // set high when sector ready
   output logic [8:0]          disk_addr, // Byte to read or write from sector
   output logic                disk_wr, // Write data into sector (read when low)
   output logic                disk_flush, // sector access done, so flush (hint)
   input logic                 disk_error, // out of bounds (?)
   input logic [7:0]           disk_data,
   output logic [7:0]          disk_din,

   output logic                adamnet_req_n,
   input logic                 adamnet_ack_n,
   output logic                adamnet_wait_n,
   output logic                adamnet_sel,
   output logic [7:0]          adamnet_dout
   );

  //assign adamnet_dout = ramb_dout;

  localparam PCB_BASE_INIT = 16'hFEC0; // PCB base address on reset

  /** PCB Field Offsets ****************************************/
  localparam PCB_CMD_STAT   = 0;
  localparam PCB_BA_LO      = 1;
  localparam PCB_BA_HI      = 2;
  localparam PCB_MAX_DCB    = 3;
  localparam PCB_SIZE       = 4;

  typedef struct {
    logic [7:0]  pcb_cmd_stat;
    logic [7:0]  pcb_ba_lo;
    logic [7:0]  pcb_ba_hi;
    logic [7:0]  pcb_max_dcb;
  } pcb_table_t;

  pcb_table_t pcb_table;

  /** DCB Field Offsets ****************************************/
  localparam DCB_CMD_STAT   = 0;
  localparam DCB_BA_LO      = 1;
  localparam DCB_BA_HI      = 2;
  localparam DCB_BUF_LEN_LO = 3;
  localparam DCB_BUF_LEN_HI = 4;
  localparam DCB_SEC_NUM_0  = 5;
  localparam DCB_SEC_NUM_1  = 6;
  localparam DCB_SEC_NUM_2  = 7;
  localparam DCB_SEC_NUM_3  = 8;
  localparam DCB_DEV_NUM    = 9;
  localparam DCB_RETRY_LO   = 14;
  localparam DCB_RETRY_HI   = 15;
  localparam DCB_ADD_CODE   = 16;
  localparam DCB_MAXL_LO    = 17;
  localparam DCB_MAXL_HI    = 18;
  localparam DCB_DEV_TYPE   = 19;
  localparam DCB_NODE_TYPE  = 20;
  localparam DCB_SIZE       = 21;

  typedef struct {
    logic [7:0]  dcb_cmd_stat;
    logic [7:0]  dcb_ba_lo;
    logic [7:0]  dcb_ba_hi;
    logic [7:0]  dcb_buf_len_lo;
    logic [7:0]  dcb_buf_len_hi;
    logic [7:0]  dcb_sec_num_0;
    logic [7:0]  dcb_sec_num_1;
    logic [7:0]  dcb_sec_num_2;
    logic [7:0]  dcb_sec_num_3;
    logic [7:0]  dcb_dev_num;
    logic [7:0]  dcb_retry_lo;
    logic [7:0]  dcb_retry_hi;
    logic [7:0]  dcb_add_code;
    logic [7:0]  dcb_maxl_lo;
    logic [7:0]  dcb_maxl_hi;
    logic [7:0]  dcb_dev_type;
    logic [7:0]  dcb_node_type;
  } dcb_table_t;

  dcb_table_t dcb_table[15];

  /** PCB Commands *********************************************/
  localparam CMD_PCB_IDLE   = 8'h00;
  localparam CMD_PCB_SYNC1  = 8'h01;
  localparam CMD_PCB_SYNC2  = 8'h02;
  localparam CMD_PCB_SNA    = 8'h03;
  localparam CMD_PCB_RESET  = 8'h04;
  localparam CMD_PCB_WAIT   = 8'h05;

  /** DCB Commands *********************************************/
  localparam CMD_RESET      = 8'h00;
  localparam CMD_STATUS     = 8'h01;
  localparam CMD_ACK        = 8'h02;
  localparam CMD_CLEAR      = 8'h03;
  localparam CMD_RECEIVE    = 8'h04;
  localparam CMD_CANCEL     = 8'h05;
  localparam CMD_SEND       = 8'h06; /* + SIZE_HI + SIZE_LO + DATA + CRC */
  localparam CMD_NACK       = 8'h07;

  localparam CMD_SOFT_RESET = 8'h02;
  localparam CMD_WRITE      = 8'h03;
  localparam CMD_READ       = 8'h04;

  /** Response Codes *******************************************/
  localparam RSP_STATUS     = 8'h80; /* + SIZE_HI + SIZE_LO + TXCODE + STATUS + CRC */
  localparam RSP_ACK        = 8'h90;
  localparam RSP_CANCEL     = 8'hA0;
  localparam RSP_SEND       = 8'hB0; /* + SIZE_HI + SIZE_LO + DATA + CRC */
  localparam RSP_NACK       = 8'hC0;

  localparam CB_RANGE = PCB_SIZE + (15*DCB_SIZE);

  logic [15:0]  pcb_base; /* Base of the PCB/ DCB in memory
                           * The size of the memory segment is
                           * pcb_base + PCB_SIZE + (15*DCB_SIZE) - 1
                           */
  logic [15:0]  pcb_addr;
  logic [15:0]  dcb_base; // precompute the dcb_base
  logic [15:0]  pcb_offset;
  logic [15:0]  dcb_offset;
  logic [15:0]  dcb_counter, data_counter;
  logic [7:0]   data_reg;
  logic [7:0]   devid;
  logic [7:0]   command;
  logic [3:0]   max_dcb;
  logic [3:0]   max_dcb_next;
  logic [4:0]   step; // use to step through devices
  logic [15:0]  value0; // operation variable
  logic [15:0]  value1; // operation variable
  logic [7:0]   addr;   // address for operation
  logic [15:0]  iaddr;
  logic [15:0]  ivalue; // Intermediate step for calculating the addressing
  logic [15:0]  pcb_raw;
  logic         pcb_wr;
  logic         pcb_rd;
  logic [7:0]   pcb_wr_data;
  logic [7:0]   return_value; // Read value from RAM subroutine
  logic         is_block;     // Flag for block device
  logic [15:0]  msg_size;
  logic [2:0]   diskid;
  logic [1:0]   tapeid;
  logic [16:0]  upper_pcb;
  logic [15:0]  dcb_base_cmd[15];
  logic [14:0]  dcb_cmd_hit;
  logic [3:0]   dcb_cmd_dev;
  logic         dcb_dev_hit_any;
  logic [14:0]  dcb_dev_hit;
  logic [3:0]   dcb_dev;
  logic [15:0]  z80_addr_last;
  // Disk access
  logic [15:0]  buffer;
  logic [15:0]  len;
  logic [31:0]  sec;
  logic         disk_req;   // Request a disk acccess
  logic         disk_rd;    // Read or ~write
  logic         disk_done;  // Disk transfer complete
  logic [3:0]   disk_dev;   // Device that is done

  assign upper_pcb = pcb_base + CB_RANGE;
  assign max_dcb = pcb_table.pcb_max_dcb;
  typedef enum bit [5:0]
               {
                MOVE_PCB,
                IDLE
                } adam_state_t;

  adam_state_t adam_state, return_state, next_state;

  function logic [2:0] InterleaveTable(input logic [2:0] sector);
    case (sector)
      0: InterleaveTable = 0;
      1: InterleaveTable = 5;
      2: InterleaveTable = 2;
      3: InterleaveTable = 7;
      4: InterleaveTable = 4;
      5: InterleaveTable = 1;
      6: InterleaveTable = 6;
      7: InterleaveTable = 3;
    endcase // case (sector)
  endfunction

  bit [14:0] is_dsk[2];
  bit [14:0] is_kbd;
  bit [14:0] is_prn;
  bit [14:0] is_tap;

  always_ff @(posedge clk_i) begin
    dcb_cmd_hit     = '0;
    dcb_cmd_dev     = '0;
    dcb_dev         = '0;
    dcb_dev_hit     = '0;
    dcb_dev_hit_any = '0;
    is_dsk          = '{default: '0};
    is_kbd          = '0;
    is_prn          = '0;
    is_tap          = '0;
    diskid          = '0;
    tapeid          = '0;

    for (int i = 0; i < 15; i++) begin
      if ((z80_addr >= dcb_base_cmd[i]) &&
          (z80_addr < (dcb_base_cmd[i] + DCB_SIZE))  &&
          (i < max_dcb)) begin
        dcb_dev_hit_any = '1;
        dcb_dev_hit[i]  = '1;
        dcb_dev         = i;
      end
      if ((z80_addr == dcb_base_cmd[i]) && (i < max_dcb)) begin
        dcb_cmd_hit[i] = '1;
        dcb_cmd_dev    = i;
        diskid         = {dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} - 4;
        tapeid         = {dcb_table[i].dcb_dev_num[0], dcb_table[i].dcb_add_code[0]};
      end
      //if (i < max_dcb) begin
        is_dsk[0][i] = ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h4) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h5) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h6) |
                       ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h7) & (i < max_dcb);
        is_dsk[1][i] = ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h52) & (i < max_dcb);
        is_kbd[i] = ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h1) & (i < max_dcb);
        is_prn[i] = ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h2) & (i < max_dcb);
        is_tap[i] = ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h8) |
                    ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h9) |
                    ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h18) |
                    ({dcb_table[i].dcb_dev_num[3:0], dcb_table[i].dcb_add_code[3:0]} == 8'h19) & (i < max_dcb);
      //end // if (i < pcb_table.pcb_max_dcb)
    end
  end

  logic [15:0] dcb_reg;

  assign pcb_base = {pcb_table.pcb_ba_hi, pcb_table.pcb_ba_lo};
  assign dcb_reg = z80_addr - dcb_base_cmd[dcb_dev];
  logic [7:0]  temp_reg;
  assign temp_reg = {dcb_table[0].dcb_dev_num[3:0], dcb_table[0].dcb_add_code[3:0]};
  always_ff @(posedge clk_i) begin
    // defaults
    adamnet_wait_n <= '0;
    kbd_status_upd <= '0;
    disk_req       <= '0;

    for (int i = 0; i < 15; i++) begin
      dcb_base_cmd[i]    <= (pcb_base + PCB_SIZE) + i * DCB_SIZE;
    end

    case (adam_state)
      /** MovePCB() ************************************************/
      /** Move PCB and related DCBs to a new address.             **/
      /*************************************************************/
      // static void MovePCB(word NewAddr,byte MaxDCB)
      MOVE_PCB: begin
        $display("Adamnet (HDL): MovePCB Address: %x", pcb_addr);
        pcb_table.pcb_cmd_stat <= CMD_PCB_IDLE;
        pcb_table.pcb_ba_lo    <= pcb_addr[7:0];
        pcb_table.pcb_ba_hi    <= pcb_addr[15:8];
        pcb_table.pcb_max_dcb  <= 15;

        for (int i = 0; i < 15; i++) begin
          dcb_table[i].dcb_dev_num  <= '0;
          dcb_table[i].dcb_add_code <= i;
        end
        adam_state   <= IDLE;
      end // case: MOVE_PCB

      IDLE: begin
        //$display("IDLE");
        adamnet_req_n        <= '1;
        adamnet_wait_n       <= '1;
        pcb_wr               <= z80_wr;
        pcb_rd               <= z80_rd;
        pcb_wr_data          <= (z80_wr) ? z80_data_wr : '1; // set to -1 if read
        if ((USE_REQ == 1) ? adamnet_ack_n : adamnet_wait_n) begin
          // Snoop PCB/ DCB Writes
          if ((z80_addr - pcb_base) < PCB_SIZE) begin
            case (z80_addr - pcb_base)
              PCB_CMD_STAT: begin
                if (z80_wr) begin
                  case (z80_data_wr)
                    CMD_PCB_SYNC1: begin
                      $display("Adamnet (HDL): SyncZ80: %x, %x",PCB_CMD_STAT,RSP_STATUS|CMD_PCB_SYNC1);
                      pcb_table.pcb_cmd_stat <= z80_data_wr | RSP_STATUS;
                    end
                    CMD_PCB_SYNC2: begin
                      $display("Adamnet (HDL): Sync6801: %x, %x",PCB_CMD_STAT,RSP_STATUS|CMD_PCB_SYNC2);
                      pcb_table.pcb_cmd_stat <= z80_data_wr | RSP_STATUS;
                    end
                    CMD_PCB_SNA: begin
                      // Todo: add in movepcb
                      $display("Adamnet (HDL): Rellocate PCB: %x, %x",PCB_CMD_STAT,RSP_STATUS|CMD_PCB_SNA);
                      pcb_table.pcb_cmd_stat <= z80_data_wr | RSP_STATUS;
                      $finish;
                    end
                    default: begin
                      $display("Adamnet (HDL): Unimplemented PCB Operation");
                      pcb_table.pcb_cmd_stat <= z80_data_wr;
                    end
                  endcase
                end
              end
              PCB_BA_LO: begin
                //if (z80_wr) pcb_table.pcb_ba_lo <= z80_data_wr;
                if (z80_wr) pcb_addr[7:0] <= z80_data_wr;
              end
              PCB_BA_HI: begin
                //if (z80_wr) pcb_table.pcb_ba_hi <= z80_data_wr;
                if (z80_wr) pcb_addr[15:8] <= z80_data_wr;
              end
              PCB_MAX_DCB: begin
                if (z80_wr) pcb_table.pcb_max_dcb <= z80_data_wr;
                if (z80_wr) max_dcb_next <= z80_data_wr;
              end
            endcase // case (z80_addr - pcb_base)
          end
          if (dcb_dev_hit_any) begin
            case (z80_addr - dcb_base_cmd[dcb_dev])// (pcb_base + PCB_SIZE + DCB_SIZE * dcb_dev))
              DCB_CMD_STAT: begin
                if (|z80_data_wr[6:0] && ~z80_data_wr[7]) begin
                  if (is_kbd[dcb_dev]) begin
                    if (z80_rd) begin
                      kbd_status <= RSP_STATUS;
                      kbd_status_upd <= '1;
                      lastkey_out    <= '0;
                    end else if (z80_wr) begin
                      case (z80_data_wr)
                        CMD_STATUS, CMD_SOFT_RESET: begin
                          kbd_status     <= RSP_STATUS;
                          kbd_status_upd <= '1;
                          lastkey_out    <= '0;
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                          dcb_table[dcb_dev].dcb_maxl_lo  <= 8'h01;
                          dcb_table[dcb_dev].dcb_maxl_hi  <= 8'h00;
                          dcb_table[dcb_dev].dcb_dev_type <= '0;
                        end
                        CMD_WRITE: begin
                          kbd_status     <= RSP_STATUS;
                          kbd_status_upd <= '1;
                          dcb_table[dcb_dev].dcb_cmd_stat <= 8'h9B; // RSP_ACK + 8'h0B;
                        end
                        CMD_READ: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                          // FIXME!!!!!
                          //A = GetDCBBase(Dev);
                          //N = GetDCBLen(Dev);
                          //for(J=0 ; (J<N) && (V=GetKBD()) ; ++J, A=(A+1)&0xFFFF)
                          //  RAM(A) = V;
                          //KBDStatus = RSP_STATUS+(J<N? 0x0C:0x00);
                        end
                      endcase
                    end // if (z80_wr)
                  end else if (is_prn[dcb_dev]) begin
                    if (z80_wr) begin
                      case (z80_data_wr)
                        CMD_STATUS, CMD_SOFT_RESET: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                          dcb_table[dcb_dev].dcb_maxl_lo  <= 8'h01;
                          dcb_table[dcb_dev].dcb_maxl_hi  <= 8'h00;
                          dcb_table[dcb_dev].dcb_dev_type <= '0;
                        end
                        CMD_READ: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= 8'h9B; // RSP_ACK + 8'h0B;
                        end
                        CMD_WRITE: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                          // FIXME!!!!!
                          //A = GetDCBBase(Dev);
                          //N = GetDCBLen(Dev);
                          //for(J=0 ; (J<N) && (V=GetKBD()) ; ++J, A=(A+1)&0xFFFF)
                          //  RAM(A) = V;
                          //KBDStatus = RSP_STATUS+(J<N? 0x0C:0x00);
                        end
                        default: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                      endcase
                    end // if (z80_wr)
                  end else if (is_dsk[0][dcb_dev]) begin
                    $display("Adamnet (HDL): UpdateDSK N %x Dev %x V %x",diskid,dcb_dev,z80_data_wr);
                    if (z80_rd && dcb_cmd_hit[dcb_dev]) begin
                      dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                    end else if (z80_wr) begin
                      dcb_table[dcb_dev].dcb_node_type[3:0] <= disk_present[diskid] ? '0 : 4'h3;
                      case (z80_data_wr)
                        CMD_STATUS: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                          dcb_table[dcb_dev].dcb_maxl_lo  <= 8'h00;
                          dcb_table[dcb_dev].dcb_maxl_hi  <= 8'h04;
                          dcb_table[dcb_dev].dcb_dev_type <= 8'h01;
                        end
                        CMD_SOFT_RESET: begin
                          $display("Adamnet (HDL): Disk %s: Soft reset",diskid+65);
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                        CMD_WRITE, CMD_READ: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                          if (disk_present[diskid]) begin
                            buffer <= {dcb_table[dcb_dev].dcb_ba_hi, dcb_table[dcb_dev].dcb_ba_lo};
                            len    <= {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo} < 16'h400 ?
                                      16'h400 : {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo};
                            sec <= {dcb_table[dcb_dev].dcb_sec_num_3, dcb_table[dcb_dev].dcb_sec_num_2,
                                    dcb_table[dcb_dev].dcb_sec_num_1, dcb_table[dcb_dev].dcb_sec_num_0};
                            $display("Adamnet (HDL): Disk %s: %s %d bytes, sector 0x%X, memory 0x%04X",
                                     devid+65,z80_data_wr==CMD_READ? "Reading":"Writing",len,sec<<1,buffer);
                            disk_req <= '1;
                            disk_rd  <= z80_data_wr == CMD_READ;
                          end
                        end
                      endcase
                    end
                  end else if (is_dsk[1][dcb_dev]) begin
                    $display("Adamnet (HDL): UpdateDSK N %x Dev %x V %x",4,dcb_dev,z80_data_wr);
                    if (z80_rd && dcb_cmd_hit[dcb_dev]) begin
                      dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                    end
                  end else if (is_tap[dcb_dev]) begin
                    //$display("Adamnet (HDL): UpdateDSK N %x Dev %x V %x",tapeid,dcb_dev,z80_data_wr);
                    if (z80_rd && dcb_cmd_hit[dcb_dev]) begin
                      dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                    end else if (z80_wr) begin
                      dcb_table[dcb_dev].dcb_node_type[3:0] <= tape_present[tapeid] ? '0 : 4'h3;
                      case (z80_data_wr)
                        CMD_STATUS: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                          dcb_table[dcb_dev].dcb_maxl_lo  <= 8'h00;
                          dcb_table[dcb_dev].dcb_maxl_hi  <= 8'h04;
                          dcb_table[dcb_dev].dcb_dev_type <= 8'h01;
                        end
                        CMD_SOFT_RESET: begin
                          $display("Adamnet (HDL): Tape %s: Soft reset",tapeid+65);
                          dcb_table[dcb_dev].dcb_cmd_stat <= RSP_STATUS;
                        end
                        CMD_WRITE, CMD_READ: begin
                          dcb_table[dcb_dev].dcb_cmd_stat <= '0;
                          if (tape_present[tapeid]) begin
                            buffer <= {dcb_table[dcb_dev].dcb_ba_hi, dcb_table[dcb_dev].dcb_ba_lo};
                            len    <= {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo} < 16'h400 ?
                                      16'h400 : {dcb_table[dcb_dev].dcb_buf_len_hi, dcb_table[dcb_dev].dcb_buf_len_lo};
                            sec <= {dcb_table[dcb_dev].dcb_sec_num_3, dcb_table[dcb_dev].dcb_sec_num_2,
                                    dcb_table[dcb_dev].dcb_sec_num_1, dcb_table[dcb_dev].dcb_sec_num_0};
                            $display("Adamnet (HDL): Tape %s: %s %d bytes, sector 0x%X, memory 0x%04X",
                                     devid+65,z80_data_wr==CMD_READ? "Reading":"Writing",len,sec<<1,buffer);
                            // FIXME!!!!!
                            $finish;
                          end
                        end // case: CMD_WRITE, CMD_READ
                      endcase
                    end // if (z80_wr)
                  end else if (dcb_cmd_hit[dcb_dev]) begin // if (is_tap[dcb_dev])
                    dcb_table[dcb_dev].dcb_cmd_stat <= 8'h9B; // RSP_ACK + 8'h0B;
                    $display("Adamnet (HDL): %s Unknown device #%d",
                             z80_data_wr==CMD_READ? "Reading":"Writing", devid);
                  end
                end // if (|z80_data_wr[6:0] && ~z80_data_wr[7])
              end // case: DCB_CMD_STAT
              DCB_BA_LO: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_ba_lo <= z80_data_wr;
              end
              DCB_BA_HI: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_ba_hi <= z80_data_wr;
              end
              DCB_BUF_LEN_LO: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_buf_len_lo <= z80_data_wr;
              end
              DCB_BUF_LEN_HI: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_buf_len_hi <= z80_data_wr;
              end
              DCB_SEC_NUM_0: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_sec_num_0 <= z80_data_wr;
              end
              DCB_SEC_NUM_1: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_sec_num_1 <= z80_data_wr;
              end
              DCB_SEC_NUM_2: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_sec_num_2 <= z80_data_wr;
              end
              DCB_SEC_NUM_3: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_sec_num_3 <= z80_data_wr;
              end
              DCB_DEV_NUM: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_dev_num <= z80_data_wr;
              end
              DCB_RETRY_LO: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_retry_lo <= z80_data_wr;
              end
              DCB_RETRY_HI: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_retry_hi <= z80_data_wr;
              end
              DCB_ADD_CODE: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_add_code <= z80_data_wr;
              end
              DCB_MAXL_LO: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_maxl_lo <= z80_data_wr;
              end
              DCB_MAXL_HI: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_maxl_hi <= z80_data_wr;
              end
              DCB_DEV_TYPE: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_dev_type <= z80_data_wr;
              end
              DCB_NODE_TYPE: begin
                if (z80_wr) dcb_table[dcb_dev].dcb_node_type <= z80_data_wr;
              end
            endcase // case (z80_addr - DCB_SIZE * dcb_dev)
          end // if (dcb_dev_hit_any)
        end // if ((USE_REQ == 1) ? adamnet_ack_n : adamnet_wait_n)
      end // case: IDLE
    endcase // case (adam_state)

    if (disk_done) dcb_table[disk_dev].dcb_cmd_stat <= RSP_STATUS;

    if (~adam_reset_pcb_n_i) begin
      // On reset setup PCB table
      pcb_addr       <= PCB_BASE_INIT;
      adam_state     <= MOVE_PCB;
      adamnet_req_n  <= '1;
      adamnet_wait_n <= '1;
      kbd_status     <= RSP_STATUS;
    end
  end

  initial begin
    // On reset setup PCB table
    pcb_addr       = PCB_BASE_INIT;
    adam_state     = MOVE_PCB;
    adamnet_req_n  = '1;
    adamnet_wait_n = '1;
    kbd_status     = RSP_STATUS;
  end

  // Readback
  always_comb begin
    adamnet_dout = '0;
    adamnet_sel  = '0;
    if ((z80_addr - pcb_base) < PCB_SIZE) begin
      adamnet_sel  = '1;
      case (z80_addr - pcb_base)
        PCB_CMD_STAT: adamnet_dout = pcb_table.pcb_cmd_stat;
        PCB_BA_LO:    adamnet_dout = pcb_addr[7:0];
        PCB_BA_HI:    adamnet_dout = pcb_addr[15:8];
        PCB_MAX_DCB:  adamnet_dout = pcb_table.pcb_max_dcb;
      endcase // case (z80_addr - pcb_base)
    end else if (dcb_dev_hit_any) begin
      adamnet_sel  = '1;
      case (z80_addr - dcb_base_cmd[dcb_dev])
        DCB_CMD_STAT:   adamnet_dout = dcb_table[dcb_dev].dcb_cmd_stat;
        DCB_BA_LO:      adamnet_dout = dcb_table[dcb_dev].dcb_ba_lo;
        DCB_BA_HI:      adamnet_dout = dcb_table[dcb_dev].dcb_ba_hi;
        DCB_BUF_LEN_LO: adamnet_dout = dcb_table[dcb_dev].dcb_buf_len_lo;
        DCB_BUF_LEN_HI: adamnet_dout = dcb_table[dcb_dev].dcb_buf_len_hi;
        DCB_SEC_NUM_0:  adamnet_dout = dcb_table[dcb_dev].dcb_sec_num_0;
        DCB_SEC_NUM_1:  adamnet_dout = dcb_table[dcb_dev].dcb_sec_num_1;
        DCB_SEC_NUM_2:  adamnet_dout = dcb_table[dcb_dev].dcb_sec_num_2;
        DCB_SEC_NUM_3:  adamnet_dout = dcb_table[dcb_dev].dcb_sec_num_3;
        DCB_DEV_NUM:    adamnet_dout = dcb_table[dcb_dev].dcb_dev_num;
        DCB_RETRY_LO:   adamnet_dout = dcb_table[dcb_dev].dcb_retry_lo;
        DCB_RETRY_HI:   adamnet_dout = dcb_table[dcb_dev].dcb_retry_hi;
        DCB_ADD_CODE:   adamnet_dout = dcb_table[dcb_dev].dcb_add_code;
        DCB_MAXL_LO:    adamnet_dout = dcb_table[dcb_dev].dcb_maxl_lo;
        DCB_MAXL_HI:    adamnet_dout = dcb_table[dcb_dev].dcb_maxl_hi;
        DCB_DEV_TYPE:   adamnet_dout = dcb_table[dcb_dev].dcb_dev_type;
        DCB_NODE_TYPE:  adamnet_dout = dcb_table[dcb_dev].dcb_node_type;
      endcase // case (z80_addr - DCB_SIZE * dcb_dev)
    end
  end // always_comb

  logic [15:0] ram_buffer;
  logic [15:0] disk_len;
  logic [31:0] disk_sec;
  logic [15:0] int_ramb_addr;
  logic        int_ramb_wr;
  logic        int_ramb_rd;

  assign ramb_dout    = disk_data;

  typedef enum bit [2:0]
               {
                DISK_IDLE,
                DISK_READ[4],
                DISK_WRITE[1]
                } disk_state_t;

  disk_state_t disk_state;

  always_ff @(posedge clk_i) begin
    ramb_addr  <= int_ramb_addr;
    ramb_wr    <= int_ramb_wr;
    ramb_rd    <= int_ramb_rd;
    int_ramb_wr<= '0;
    int_ramb_rd<= '0;
    disk_done  <= '0;
    disk_wr    <= '0;
    disk_flush <= '0;

    case (disk_state)
      DISK_IDLE: begin
        if (disk_req) begin
          ram_buffer  <= buffer;
          disk_len    <= len;
          dcb_counter <= len;
          disk_sec    <= sec<<1;
          disk_dev    <= dcb_dev;
          disk_state  <= DISK_READ0;
        end
      end // case: DISK_IDLE
      DISK_READ0: begin
        //disk_sector <= {disk_sec[31:3], InterleaveTable(disk_sec[2:0])};
        disk_sector <= {disk_sec[31:3], InterleaveTable(disk_sec[2:0])};
        disk_load   <= '1;
        if (disk_sector_loaded) begin
          $display("Adamnet (HDL): Disk %s: %s %d bytes, sector 0x%X, memory 0x%04X\n",
                   devid+65,disk_rd? "Reading":"Writing",dcb_counter,disk_sec<<1,ram_buffer);
          int_ramb_addr    <= ram_buffer - 1'b1; // We will advance automatically in next state
          disk_load    <= '0;
          data_counter <= '0;
          disk_state   <= disk_rd ? DISK_READ1 : DISK_WRITE0;
        end
      end
      DISK_READ1: begin
        // Read up to 512 bytes (might be less)
        disk_addr <= data_counter;
        if (data_counter < dcb_counter && data_counter < 16'h200) begin
          // We are within the sector
          int_ramb_addr    <= int_ramb_addr + 1'b1;
          int_ramb_wr      <= '1;
          data_counter <= data_counter + 1'b1;
          ram_buffer   <= ram_buffer + 1'b1;
        end else if (data_counter < dcb_counter) begin
          // We are leaving the sector, but we have more data to read.
          // Flush the current sector
          disk_wr      <= '0;
          disk_flush   <= '1;
          data_counter <= '0;
          disk_sec     <= disk_sec + 1'b1; // Advance for next sector
          dcb_counter  <= dcb_counter - 16'h200;
          disk_state   <= DISK_READ2;
        end else begin
          // Done reading
          disk_wr    <= '0;
          disk_flush <= '1;
          disk_state <= DISK_IDLE;
          disk_done  <= '1;
        end // else: !if(data_counter < dcb_counter)
      end // case: DISK_READ1
      DISK_READ2: begin
        if (~disk_sector_loaded) disk_state <= DISK_READ0;
      end
      DISK_WRITE0: begin
        $display("Write not supported");
        $finish;
      end
    endcase
  end
endmodule
