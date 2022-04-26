module cv_adamnet
  #
  (
   parameter NUM_DISKS = 1
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
   output logic [31:0]         disk_sector, // sector
   output logic                disk_load, // load the 512 byte sector
   input logic                 disk_sector_loaded, // set high when sector ready
   output logic [8:0]          disk_addr, // Byte to read or write from sector
   output logic                disk_wr, // Write data into sector (read when low)
   output logic                disk_flush, // sector access done, so flush (hint)
   input logic                 disk_error, // out of bounds (?)
   input logic [7:0]           disk_data
   );

  localparam PCB_BASE_INIT = 16'hFEC0; // PCB base address on reset

  /** PCB Field Offsets ****************************************/
  localparam PCB_CMD_STAT   = 0;
  localparam PCB_BA_LO      = 1;
  localparam PCB_BA_HI      = 2;
  localparam PCB_MAX_DCB    = 3;
  localparam PCB_SIZE       = 4;

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
  logic [31:0]  sector;
  logic [4:0]   step; // use to step through devices
  logic [15:0]  value0; // operation variable
  logic [15:0]  value1; // operation variable
  logic [7:0]   addr;   // address for operation
  logic [15:0]  iaddr;
  logic [15:0]  ivalue; // Intermediate step for calculating the addressing
  logic [15:0]  pcb_raw;
  logic         pcb_wr;
  logic [7:0]   pcb_wr_data;
  logic [7:0]   return_value; // Read value from RAM subroutine
  logic         is_block;     // Flag for block device
  logic [15:0]  msg_size;
  logic [1:0]   diskid;

  typedef enum bit [5:0]
               {
                MOVE_PCB,
                MOVE_DCB,
                SET_PCB,
                SET_DCB[3],
                TEST_DCB_DONE,
                IDLE,
                IDLE_WR,
                IDLE_WR_MOVEPCB,
                WALK_DCB[3],
                GET_DCB[2],
                REPORT_DEVICE,
                READ_KEY[3],
                UPDATE_DSK[10],
                GET_BUFLEN[5],
                GET_DCB_SECTOR[5]
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

  always_ff @(posedge clk_i) begin
    // defaults
    disk_wr      <= '0;
    case (adam_state)
      /** MovePCB() ************************************************/
      /** Move PCB and related DCBs to a new address.             **/
      /*************************************************************/
      // static void MovePCB(word NewAddr,byte MaxDCB)
      MOVE_PCB: begin
        step <= step + 1'b1;
        case (step)
          0: begin
            $display("Adamnet (HDL): MovePCB Address: %x\n",pcb_addr);
            return_state <= MOVE_PCB;
            adam_state   <= SET_PCB;
            value0       <= {8'h0, pcb_addr[7:0]};
            addr         <= PCB_BA_LO;
          end
          1: begin
            return_state <= MOVE_PCB;
            adam_state   <= SET_PCB;
            value0       <= {8'h0, pcb_addr[15:8]};
            addr         <= PCB_BA_HI;
          end
          2: begin
            return_state <= MOVE_DCB;
            adam_state   <= SET_PCB;
            value0       <= {8'h0, pcb_addr[15:8]};
            addr         <= PCB_MAX_DCB;
            step         <= '0; // reset the step since the next part of move_pcb writes the dcb
          end
        endcase // case (step)
      end
      MOVE_DCB: begin
        // second half of movepcb, we wrrite the DCB
        step <= step + 1'b1;
        case (step[0])
          0: begin
            // SetDCB(J,DCB_DEV_NUM,0);
            return_state <= MOVE_DCB;
            adam_state   <= SET_DCB0;
            value0       <= '0;
            devid        <= step[4:1];
            addr         <= DCB_DEV_NUM;
          end
          1: begin
            // SetDCB(J,DCB_ADD_CODE,J);
            return_state <= TEST_DCB_DONE;
            adam_state   <= SET_DCB0;
            value0       <= step[4:1];
            devid        <= step[4:1];
            addr         <= DCB_ADD_CODE;
          end
        endcase // case (step[0])
      end
      /** SetPCB() *************************************************/
      /** Set PCB byte at given offset.                           **/
      /*************************************************************/
      // static void SetPCB(word Offset,byte Value)
      SET_PCB: begin
        ramb_addr <= pcb_base + addr;
        ramb_wr   <= '1;
        ramb_rd   <= '0;
        ramb_dout <= value0[7:0];
        if (ramb_wr_ack) begin
          $display("Adamnet (HDL): SetPCB A %x Value %x\n",pcb_base + addr,value0[7:0]);
         adam_state <= return_state;
        end
      end
      /** SetDCB() *************************************************/
      /** Set DCB byte at given offset.                           **/
      /*************************************************************/
      // static void SetDCB(byte Dev,byte Offset,byte Value)
      SET_DCB0: begin
        ramb_addr <= pcb_base + PCB_SIZE;
        ivalue    <= devid * DCB_SIZE + addr; // Do we want to split this?
        ramb_wr   <= '0;
        ramb_rd   <= '0;
        ramb_dout <= value0[7:0];
        adam_state <= SET_DCB1;
      end
      SET_DCB1: begin
        ramb_addr <= ramb_addr + ivalue;
        ramb_wr   <= '1;
        ramb_rd   <= '0;
        ramb_dout <= value0[7:0];
        if (ramb_wr_ack) begin
          adam_state <= return_state;
          $display("Adamnet (HDL): SetDCB A %x Value %x\n",ramb_addr + ivalue,value0[7:0]);
        end else begin
          adam_state <= SET_DCB2;
        end
      end // case: SET_DCB1
      SET_DCB2: begin
        ramb_wr   <= '1;
        ramb_rd   <= '0;
        ramb_dout <= value0[7:0];
        if (ramb_wr_ack) begin
          adam_state <= return_state;
          $display("Adamnet (HDL): SetDCB A %x Value %x\n",ramb_addr + ivalue,value0[7:0]);
        end
      end // case: SET_DCB1
      TEST_DCB_DONE: begin
        if (devid <= value1)
          adam_state <= MOVE_DCB;
        else
          adam_state <= IDLE;
      end
      /** WritePCB() ***********************************************/
      /** Write value to a given PCB or DCB address.              **/
      /*************************************************************/
      // void WritePCB(word A,byte V)
      /** ReadPCB() ************************************************/
      /** Read value from a given PCB or DCB address.             **/
      /*************************************************************/
      //void ReadPCB(word A)
      IDLE: begin
        // Snoop PCB/ DCB Writes
        pcb_raw     <= z80_addr - pcb_base;
        pcb_wr      <= z80_wr;
        pcb_wr_data <= (z80_wr) ? z80_data_wr : '1; // set to -1 if read
        if ((z80_addr > pcb_base) &&
            (z80_addr < (pcb_base + PCB_SIZE))) begin
          // Falls within the PCB table
          /*if (z80_wr) */ adam_state <= IDLE_WR;
          // if (z80_rd) adam_state <= IDLE_RD; // FIXME!!!!
        end else if ((z80_addr > pcb_base) &&
                     (z80_addr < (pcb_base + CB_RANGE))) begin
          // Falls within the DCB table
          pcb_raw     <= z80_addr - pcb_base - PCB_SIZE;
          dcb_counter <= '0;
          if (z80_wr && (|pcb_wr_data || ~pcb_wr_data[7]) ||
              z80_rd && (z80_addr == pcb_base)) adam_state  <= WALK_DCB0;
        end
      end
      IDLE_WR: begin
        case (pcb_raw)
          PCB_CMD_STAT: begin
            case (pcb_wr_data)
              CMD_PCB_SYNC1: begin
                // Sync Z80
                $display("Adamnet (HDL): SyncZ80: %x, %x \n",PCB_CMD_STAT,RSP_STATUS|pcb_wr_data);
                return_state <= IDLE;
                adam_state   <= SET_PCB;
                value0       <= RSP_STATUS | pcb_wr_data;
                addr         <= PCB_CMD_STAT;
              end
              CMD_PCB_SYNC2: begin
                // Sync master 6801
                $display("Adamnet (HDL): Sync6801: %x, %x \n",PCB_CMD_STAT,RSP_STATUS|pcb_wr_data);
                return_state <= IDLE;
                adam_state   <= SET_PCB;
                value0       <= RSP_STATUS | pcb_wr_data;
                addr         <= PCB_CMD_STAT;
              end
              CMD_PCB_SNA: begin
                // Rellocate PCB -- TODO
                // Need to read the PCB base from memory then move then set
                $display("Adamnet (HDL): Rellocate PCB: %x, %x \n",PCB_CMD_STAT,RSP_STATUS|pcb_wr_data);
                return_state <= IDLE;
                adam_state   <= IDLE_WR_MOVEPCB;
                value0       <= RSP_STATUS | pcb_wr_data;
                addr         <= PCB_CMD_STAT;
              end
            endcase // case (pcb_wr_data)
          end // case: PCB_CMD_STAT
          PCB_BA_LO: begin
            pcb_addr[7:0] <= pcb_wr_data;
            adam_state    <= IDLE;
          end
          PCB_BA_HI: begin
            pcb_addr[15:8] <= pcb_wr_data;
            adam_state    <= IDLE;
          end
          PCB_MAX_DCB: begin
            max_dcb       <= pcb_wr_data;
            adam_state    <= IDLE;
          end
        endcase // case (pcb_raw)
      end // case: IDLE_WR
      IDLE_WR_MOVEPCB: begin
        adam_state   <= MOVE_PCB;
        value0       <= pcb_addr;
        value1       <= max_dcb;
        step         <= '0;
        return_state <= IDLE;
      end
      WALK_DCB0: begin
        // Test if we are within a given device
        // This is a kludge of IsPCB. What about port 60 and ranges?
        // We also test the whole 15 devices range, even if we may
        // have less, so this might be optimized
        if (pcb_raw >= 0 && pcb_raw <= DCB_SIZE) begin
          // We are within a given DCB
          adam_state     <= GET_DCB0;
          addr           <= DCB_DEV_NUM;
          value0         <= dcb_counter;
          return_state   <= WALK_DCB1;
        end else begin
          // Continue walking the list
          dcb_counter <= dcb_counter + 1'b1;
          pcb_raw     <= pcb_raw - DCB_SIZE;
          adam_state  <= (dcb_counter <= max_dcb) ? WALK_DCB0 : IDLE;
        end
      end // case: WALK_DCB
      WALK_DCB1: begin
        devid[7:4]     <= return_value[3:0];
        adam_state     <= GET_DCB0;
        addr           <= DCB_ADD_CODE;
        value0         <= dcb_counter;
        return_state   <= WALK_DCB2;
      end
      WALK_DCB2: begin
        // if ~pcb_wr, then force V to be -1
        case ({devid[7:4], return_value[3:0]})
          8'h01: begin
            // UpdateKBD(Dev,V)
            $display("Adamnet (HDL): UpdateKBD: Device %x, CommandL %x\n",dcb_counter, pcb_wr_data);
            case (pcb_wr_data)
              8'hFF: begin
                // SetDCB(J,DCB_DEV_NUM,0);
                // SetDCB(Dev,DCB_CMD_STAT,KBDStatus);
                return_state <= IDLE;
                adam_state   <= SET_DCB0;
                value0       <= kbd_status;
                devid        <= dcb_counter;
                addr         <= DCB_CMD_STAT;
              end
              CMD_STATUS, CMD_SOFT_RESET: begin
                /* Character-based device, single character buffer */
                kbd_status   <= RSP_STATUS;
                lastkey_out  <= '0;
                //ReportDevice(Dev,0x0001,0);
                return_state <= IDLE;
                adam_state   <= REPORT_DEVICE;
                is_block     <= 0;
                devid        <= dcb_counter;
                msg_size     <= 16'h0001;
                dcb_counter  <= '0;
              end
              CMD_WRITE: begin
                kbd_status   <= RSP_STATUS;
                // SetDCB(Dev,DCB_CMD_STAT,RSP_ACK+0x0B);
                return_state <= IDLE;
                adam_state   <= SET_DCB0;
                value0       <= RSP_ACK + 8'h0B;
                devid        <= dcb_counter;
                addr         <= DCB_CMD_STAT;
              end
              CMD_READ: begin
                //SetDCB(Dev,DCB_CMD_STAT,0x00);
                return_state <= IDLE;
                next_state   <= READ_KEY0;
                adam_state   <= GET_BUFLEN0;
                value0       <= '0;
                devid        <= dcb_counter;
                addr         <= DCB_CMD_STAT;
              end
            endcase // case (pcb_wr_data)
          end
          8'h02: begin
            // UpdatePRN(Dev,V)
          end
          8'h04, 8'h05, 8'h06, 8'h07: begin
            // UpdateDSK(DiskID=DevID-4,Dev,V)
            devid        <= dcb_counter;
            diskid       <= return_value[2:1];
            adam_state   <= UPDATE_DSK0;
          end
          8'h08, 8'h09, 8'h18, 8'h19: begin
            //UpdateTAP((DevID>>4)+((DevID&1)<<1),Dev,V);
          end
          8'h52: begin
            // UpdateDSK(DiskID,Dev,-2)
            devid        <= dcb_counter;
            diskid       <= return_value[2:1];
            pcb_wr_data  <= 8'hFE;
            adam_state   <= UPDATE_DSK0;
         end
          default: begin
            // SetDCB(Dev,DCB_CMD_STAT,RSP_ACK+0x0B);
            $display("Adamnet (HDL): AdamNet: write to unknown device #%d\n",
                     {devid[7:4], return_value[3:0]});
            return_state <= IDLE;
            adam_state   <= SET_DCB0;
            value0       <= '0;
            devid        <= step[4:1];
            addr         <= DCB_CMD_STAT;
          end
        endcase
      end
      /** GetDCB() *************************************************/
      /** Get DCB byte at given offset.                           **/
      /*************************************************************/
      //static byte GetDCB(byte Dev,byte Offset)
      GET_DCB0: begin
        iaddr     <= pcb_base + PCB_SIZE;
        ivalue    <= dcb_counter * DCB_SIZE + addr; // Do we want to split this?
        adam_state <= GET_DCB1;
      end
      GET_DCB1: begin
        ramb_addr <= iaddr + ivalue;
        ramb_wr   <= '0;
        ramb_rd   <= '1;
        if (ramb_rd_ack) begin
          return_value <= ramb_din;
          adam_state   <= return_state;
        end
      end
      /** ReportDevice() *******************************************/
      /** Reply to STATUS command with device parameters.         **/
      /*************************************************************/
      // static void ReportDevice(byte Dev,word MsgSize,byte IsBlock)
      REPORT_DEVICE: begin
        dcb_counter <= dcb_counter + 1'b1;
        case (dcb_counter)
          0: begin
            // SetDCB(Dev,DCB_CMD_STAT, RSP_STATUS);
            return_state <= REPORT_DEVICE;
            adam_state   <= SET_DCB0;
            value0       <= RSP_STATUS;
            addr         <= DCB_CMD_STAT;
          end
          1: begin
            //SetDCB(Dev,DCB_MAXL_LO,  MsgSize&0xFF);
            return_state <= REPORT_DEVICE;
            adam_state   <= SET_DCB0;
            value0       <= msg_size[7:0];
            addr         <= DCB_MAXL_LO;
          end
          2: begin
            //SetDCB(Dev,DCB_MAXL_HI,  MsgSize>>8);
            return_state <= REPORT_DEVICE;
            adam_state   <= SET_DCB0;
            value0       <= msg_size[15:8];
            addr         <= DCB_MAXL_HI;
          end
          3: begin
            // SetDCB(Dev,DCB_DEV_TYPE, IsBlock? 0x01:0x00);
            return_state <= IDLE;
            adam_state   <= SET_DCB0;
            value0       <= is_block ? 8'h1 : '0;
            addr         <= DCB_DEV_TYPE;
          end
        endcase // case (dcb_counter)
      end // case: REPORT_DEVICE

      /*
      UPDATE_KBD0: begin
        // A = GetDCBBase(Dev);
        adam_state     <= GET_DCB0;
        addr           <= DCB_BA_LO;
        value0         <= dcb_counter;
        return_state   <= UPDATE_KBD1;
      end
      UPDATE_KBD1: begin
        // A = GetDCBBase(Dev);
        iaddr[7:0]     <= return_value;
        adam_state     <= GET_DCB0;
        addr           <= DCB_BA_HI;
        value0         <= dcb_counter;
        return_state   <= UPDATE_KBD2;
      end
      UPDATE_KBD2: begin
        //N = GetDCBLen(Dev);
        iaddr[15:8]    <= return_value;
        adam_state     <= GET_DCB0;
        addr           <= DCB_BUF_LEN_LO;
        value0         <= dcb_counter;
        return_state   <= UPDATE_KBD3;
      end
      UPDATE_KBD3: begin
        //N = GetDCBLen(Dev);
        dcb_counter[7:0] <= return_value;
        adam_state       <= GET_DCB0;
        addr             <= DCB_BUF_LEN_HI;
        value0           <= dcb_counter;
        return_state     <= UPDATE_KBD4;
      end
      UPDATE_KBD4: begin
        dcb_counter[15:8] <= return_value;
        adam_state        <= READ_KEY0;
      end
       */
      READ_KEY0: begin
        if (lastkey_in_valid) begin
          dcb_counter    <= dcb_counter - 1'b1;
          ramb_addr      <= iaddr;
          ramb_wr        <= '1;
          ramb_dout      <= lastkey_in;
          lastkey_in_ack <= '1;
          lastkey_out    <= '0;
          adam_state     <= (dcb_counter > 0) ? READ_KEY1 : READ_KEY2;
        end
      end
      READ_KEY1: begin
        ramb_wr <= '1;
        if (ramb_wr_ack) begin
          ramb_wr    <= '0;
          adam_state <= READ_KEY0;
          iaddr      <= iaddr + 1'b1;
        end
      end
      READ_KEY2: begin
        // FIXME!!!!!!!
        // KBDStatus = RSP_STATUS+(J<N? 0x0C:0x00);
        kbd_status     <= RSP_STATUS;
        kbd_status_upd <= '1;
        adam_state     <= IDLE;
      end

      UPDATE_DSK0: begin
        $display("Adamnet: UpdateDSK N %x Dev %x V %x \n",diskid,devid,pcb_wr_data);
        // Fixme: Do we need to test for max disks?
        if (pcb_wr_data[7]) begin
          // If reading DCB status, stop here
          // SetDCB(Dev,DCB_CMD_STAT,RSP_STATUS);
          return_state <= IDLE;
          adam_state   <= SET_DCB0;
          value0       <= RSP_STATUS;
          addr         <= DCB_CMD_STAT;
        end else begin
          // Reset errors, report missing disks
          // SetDCB(Dev,DCB_NODE_TYPE,(GetDCB(Dev,DCB_NODE_TYPE)&0xF0) | (Disks[N].Data? 0x00:0x03));
          // GetDCB(Dev,DCB_NODE_TYPE)
          adam_state     <= GET_DCB0;
          addr           <= DCB_NODE_TYPE;
          value0         <= devid;
          return_state   <= UPDATE_DSK1;
        end
      end
      UPDATE_DSK1: begin
        // SetDCB(Dev,DCB_NODE_TYPE,(GetDCB(Dev,DCB_NODE_TYPE)&0xF0) | (Disks[N].Data? 0x00:0x03));
        // SetDCB(J,DCB_DEV_NUM,0);
        return_state <= UPDATE_DSK2;
        adam_state   <= SET_DCB0;
        value0       <= {return_value[7:4], 2'b0, disk_present[diskid] ? 2'b0 : 2'b11};
        addr         <= DCB_NODE_TYPE;
      end
      UPDATE_DSK2: begin
        case (pcb_wr_data)
          CMD_STATUS: begin
            // Block-based device, 1kB buffer
            //ReportDevice(Dev,0x0400,1);
            return_state <= IDLE;
            adam_state   <= REPORT_DEVICE;
            is_block     <= 1;
            msg_size     <= 16'h0400;
            dcb_counter  <= '0;
          end
          CMD_SOFT_RESET: begin
            $display("Adamnet (HDL): Disk %s: Soft reset\n",diskid+65);
            //SetDCB(Dev,DCB_CMD_STAT,RSP_STATUS);
            return_state <= IDLE;
            adam_state   <= SET_DCB0;
            value0       <= RSP_STATUS;
            addr         <= DCB_CMD_STAT;
          end
          CMD_WRITE, CMD_READ: begin
            // Busy status by default
            //SetDCB(Dev,DCB_CMD_STAT,0x00);
            return_state  <= UPDATE_DSK3;
            adam_state    <= SET_DCB0;
            value0        <= '0;
            addr          <= DCB_CMD_STAT;
          end
        endcase // case (pcb_wr_data)
      end // case: UPDATE_DSK2
      UPDATE_DSK3: begin
        next_state    <= UPDATE_DSK4;
        adam_state    <= GET_BUFLEN0;
      end
      UPDATE_DSK4: begin
        if (dcb_counter < 16'h0400) dcb_counter <= 16'h0400;
        next_state    <= UPDATE_DSK5;
        adam_state    <= GET_DCB_SECTOR0;
      end
      UPDATE_DSK5: begin
        // We now have our sector that we are reading or writing. Request access
        disk_sector <= {sector[31:3], InterleaveTable(sector[2:0])};
        disk_load   <= '1;
        if (disk_sector_loaded) begin
          $display("Adamnet: Disk %s: %s %d bytes, sector 0x%X, memory 0x%04X\n",
                   devid+65,pcb_wr_data==CMD_READ? "Reading":"Writing",dcb_counter,sector<<1,iaddr);
          disk_load  <= '0;
          next_state <= (pcb_wr) ? UPDATE_DSK8 : UPDATE_DSK6;
          data_counter <= '0;
        end
      end // case: UPDATE_DSK5
      // Disk Read
      UPDATE_DSK6: begin
        // Read up to 512 bytes (might be less)
        disk_addr <= data_counter;
        if (data_counter < dcb_counter && data_counter < 16'h200) begin
          // We are within the sector
          adam_state   <= UPDATE_DSK7;
          data_counter <= data_counter + 1'b1;
        end else if (data_counter < dcb_counter) begin
          // We are leaving the sector, but we have more data to read.
          // Flush the current sector
          disk_wr      <= '0;
          disk_flush   <= '1;
          data_counter <= '0;
          sector       <= sector + 1'b1;
          dcb_counter  <= dcb_counter - 16'h200;
          adam_state   <= UPDATE_DSK5;
        end else begin
          // Done reading
          disk_wr      <= '0;
          disk_flush   <= '1;
          adam_state   <= IDLE;
        end // else: !if(data_counter < dcb_counter)
      end // case: UPDATE_DSK6
      UPDATE_DSK7: begin
        ramb_addr      <= iaddr;
        ramb_wr        <= '1;
        ramb_dout      <= disk_data;
        if (ramb_wr_ack) begin
          iaddr      <= iaddr + 1'b1;
          adam_state <= UPDATE_DSK6;
          ramb_wr    <= '0;
        end
      end
      // Disk Write
      UPDATE_DSK8: begin
        ramb_addr  <= iaddr;
        ramb_rd    <= '1;
        adam_state <= UPDATE_DSK9;
      end
      UPDATE_DSK9: begin
        if (ramb_rd_ack) begin
          ramb_rd <= '1;
          iaddr   <= iaddr + 1'b1;
          // Write up to 512 bytes (might be less)
          disk_addr <= data_counter;
          disk_wr   <= '1;
          if (data_counter < dcb_counter && data_counter < 16'h200) begin
            // We are within the sector
            adam_state   <= UPDATE_DSK8;
            data_counter <= data_counter + 1'b1;
          end else if (data_counter < dcb_counter) begin
            // We are leaving the sector, but we have more data to read.
            // Flush the current sector
            disk_wr      <= '0;
            disk_flush   <= '1;
            data_counter <= '0;
            sector       <= sector + 1'b1;
            dcb_counter  <= dcb_counter - 16'h200;
            adam_state   <= UPDATE_DSK5;
          end else begin
            // Done reading
            disk_wr      <= '0;
            disk_flush   <= '1;
            adam_state   <= IDLE;
          end // else: !if(data_counter < dcb_counter)
        end // if (ramb_rd_ack)
      end // case: UPDATE_DSK9

      // Subroutine to load the base address into iaddr and the len into dcb_counter
      GET_BUFLEN0: begin
        // A = GetDCBBase(Dev);
        adam_state     <= GET_DCB0;
        addr           <= DCB_BA_LO;
        value0         <= dcb_counter;
        return_state   <= GET_BUFLEN1;
      end
      GET_BUFLEN1: begin
        // A = GetDCBBase(Dev);
        iaddr[7:0]     <= return_value;
        adam_state     <= GET_DCB0;
        addr           <= DCB_BA_HI;
        value0         <= dcb_counter;
        return_state   <= GET_BUFLEN2;
      end
      GET_BUFLEN2: begin
        //N = GetDCBLen(Dev);
        iaddr[15:8]    <= return_value;
        adam_state     <= GET_DCB0;
        addr           <= DCB_BUF_LEN_LO;
        value0         <= dcb_counter;
        return_state   <= GET_BUFLEN3;
      end
      GET_BUFLEN3: begin
        //N = GetDCBLen(Dev);
        dcb_counter[7:0] <= return_value;
        adam_state       <= GET_DCB0;
        addr             <= DCB_BUF_LEN_HI;
        value0           <= dcb_counter;
        return_state     <= GET_BUFLEN4;
      end
      GET_BUFLEN4: begin
        dcb_counter[15:8] <= return_value;
        adam_state        <= next_state;
      end
      // Return the sectro number
      GET_DCB_SECTOR0: begin
        //GetDCB(Dev,DCB_SEC_NUM_0)
        adam_state       <= GET_DCB_SECTOR1;
        addr             <= DCB_SEC_NUM_0;
        value0           <= devid;
        return_state     <= GET_BUFLEN4;
      end
      GET_DCB_SECTOR1: begin
        //GetDCB(Dev,DCB_SEC_NUM_0)
        sector[7:0]      <= return_value;
        adam_state       <= GET_DCB_SECTOR2;
        addr             <= DCB_SEC_NUM_1;
        value0           <= devid;
        return_state     <= GET_BUFLEN4;
      end
      GET_DCB_SECTOR2: begin
        //GetDCB(Dev,DCB_SEC_NUM_0)
        sector[15:8]     <= return_value;
        adam_state       <= GET_DCB_SECTOR3;
        addr             <= DCB_SEC_NUM_2;
        value0           <= devid;
        return_state     <= GET_BUFLEN4;
      end
      GET_DCB_SECTOR3: begin
        //GetDCB(Dev,DCB_SEC_NUM_0)
        sector[23:16]    <= return_value;
        adam_state       <= GET_DCB_SECTOR4;
        addr             <= DCB_SEC_NUM_3;
        value0           <= devid;
        return_state     <= GET_BUFLEN4;
      end
      GET_DCB_SECTOR4: begin
        //GetDCB(Dev,DCB_SEC_NUM_0)
        sector[31:24]    <= return_value;
        adam_state       <= next_state;
      end
    endcase // case (adam_state)

    if (~adam_reset_pcb_n_i) begin
      // On reset setup PCB table
      pcb_base <= PCB_BASE_INIT;
      dcb_base <= PCB_BASE_INIT + PCB_SIZE;
      pcb_offset <= '0;
      adam_state   <= MOVE_PCB;
      value0       <= 16'hFEC0;
      value1       <= 16'd15;
      pcb_addr     <= 16'hFEC0;
      step         <= '0;
      return_state <= IDLE;
    end
  end


endmodule
