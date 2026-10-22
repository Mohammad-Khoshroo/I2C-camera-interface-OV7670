`timescale 1ns / 1ps
//=============================================================================
// i2c_slave_golden.v        *** SIMULATION ONLY -- NOT SYNTHESIZABLE ***
//
// Golden I2C slave for testing a student I2C master in simulation (Exp 1).
// Event-driven on SCL/SDA, no clock, no reset. Open-drain: it only ever pulls
// SDA low or releases it. It never stretches SCL.
//
// THE TESTBENCH MUST PROVIDE THE PULL-UPS:
//     tri1 scl, sda;                      // or: wire + pullup(scl); pullup(sda);
//     assign scl = m_scl_t ? 1'bz : m_scl_o;   // master side, no IOBUF needed
//     assign sda = m_sda_t ? 1'bz : m_sda_o;
//     assign m_scl_i = scl;
//     assign m_sda_i = sda;
//
// PROTOCOL (7-bit address, two 8-bit registers at pointer 0 and 1)
//   write : S  ADDR+W A  PTR A  DATA A  [DATA A ...]  P
//   read  : S  ADDR+W A  PTR A  Sr ADDR+R A  DATA Am [DATA Am ...] DATA Nm  P
//   read at current pointer : S  ADDR+R A  DATA Nm  P
//   A = slave ACK, Am/Nm = master ACK/NACK. The pointer auto-increments and
//   wraps 0 -> 1 -> 0. A pointer byte > 1 is NACKed (err_bad_reg).
//   Any other address is ignored (no ACK, info_addr_nack).
//
// DEBUG OUTPUTS  (all sticky until clear_flags is called from the testbench,
//                 e.g.  dut_slave.clear_flags; )
//   err_bus_undef       SCL/SDA read as Z or X: missing pull-up, or a master
//                       driving a hard 1 against a device pulling low
//   err_start_mid_byte  START seen in the middle of a byte / ACK bit
//   err_stop_mid_byte   STOP  seen in the middle of a byte / ACK bit
//   err_timing          a bus timing parameter below its minimum (see T_*)
//   err_bad_reg         register pointer byte was not 0 or 1
//   err_clk_after_nack  master kept clocking after it NACKed a read byte
//   info_addr_nack      an address other than SLAVE_ADDR was seen (not an error)
//   err_any / err_count OR of the err_* flags / number of error events
//   state_ascii         current state as text - show it as ASCII in the wave
//
// Timing defaults are the I2C fast-mode (400 kHz) minimums, so a correct
// 100 kHz master passes with margin. Set TIMING_CHECK = 0 to switch them off.
//=============================================================================
module i2c_slave_golden #(
    parameter [6:0]   SLAVE_ADDR   = 7'h50,
    parameter [7:0]   REG0_INIT    = 8'hA5,
    parameter [7:0]   REG1_INIT    = 8'h3C,
    parameter         NAME         = "I2C_SLAVE",
    parameter integer VERBOSE      = 1,      // 1 = log every transaction
    parameter integer MAX_MSGS     = 50,     // stop printing errors after this many
    parameter integer T_OUT_NS     = 100,    // SCL fall -> slave SDA change
    parameter integer TIMING_CHECK = 1,
    parameter integer T_SU_DAT_NS  = 100,    // SDA stable before SCL rise
    parameter integer T_HD_STA_NS  = 600,    // START: SDA fall -> SCL fall
    parameter integer T_SU_STA_NS  = 600,    // repeated START: SCL rise -> SDA fall
    parameter integer T_SU_STO_NS  = 600,    // STOP: SCL rise -> SDA rise
    parameter integer T_BUF_NS     = 1300,   // STOP -> next START
    parameter integer T_LOW_NS     = 1300,   // SCL low time
    parameter integer T_HIGH_NS    = 600     // SCL high time
)(
    input  wire        scl,
    inout  wire        sda,

    // ---- debug / observation (leave unconnected if not needed) -------------
    output reg         busy,              // between START and STOP
    output reg  [63:0] state_ascii,
    output reg  [7:0]  reg0,
    output reg  [7:0]  reg1,
    output reg  [7:0]  reg_ptr,
    output reg  [7:0]  last_addr_byte,    // last address byte seen, R/W bit included
    output reg  [7:0]  last_wr_data,
    output reg  [7:0]  last_rd_data,
    output reg  [31:0] start_cnt,
    output reg  [31:0] stop_cnt,
    output reg  [31:0] wr_cnt,            // data bytes written to a register
    output reg  [31:0] rd_cnt,            // data bytes handed to the master
    output reg         err_bus_undef,
    output reg         err_start_mid_byte,
    output reg         err_stop_mid_byte,
    output reg         err_timing,
    output reg         err_bad_reg,
    output reg         err_clk_after_nack,
    output reg         info_addr_nack,
    output wire        err_any,
    output reg  [31:0] err_count
);

    //-------------------------------------------------------------------------
    // Open-drain SDA driver
    //-------------------------------------------------------------------------
    reg sda_low;                               // 1 = pull SDA low
    assign sda = sda_low ? 1'b0 : 1'bz;

    assign err_any = err_bus_undef | err_start_mid_byte | err_stop_mid_byte |
                     err_timing | err_bad_reg | err_clk_after_nack;

    //-------------------------------------------------------------------------
    // State
    //-------------------------------------------------------------------------
    localparam [2:0] PH_IDLE   = 3'd0,   // bus free
                     PH_RX     = 3'd1,   // receiving a byte from the master
                     PH_RX_ACK = 3'd2,   // slave is driving ACK
                     PH_TX     = 3'd3,   // sending a byte to the master
                     PH_TX_ACK = 3'd4,   // master's ACK/NACK bit
                     PH_WAIT   = 3'd5;   // not addressed / finished: wait for S or P

    reg [2:0] phase;
    reg [3:0] bitcnt;          // SCL rising edges seen in the current byte
    reg [7:0] shreg;
    reg [7:0] txreg;
    reg       got_addr;        // address byte of this transaction already seen
    reg       got_ptr;         // pointer byte of this write already seen
    reg       rw;              // 1 = master reads
    reg       ack_this;
    reg       master_ack;
    reg       after_nack;      // in PH_WAIT because the master NACKed a read
    integer   wait_clks;
    reg       start_pending;   // START seen, its closing SCL fall not yet
    reg       stop_seen;

    real t_scl_rise, t_scl_fall, t_sda_chg, t_sda_fall, t_sda_rise, t_start, t_stop;

    initial begin
        sda_low = 1'b0;  busy = 1'b0;  phase = PH_IDLE;  bitcnt = 4'd0;
        shreg = 8'h00;   txreg = 8'h00;
        got_addr = 1'b0; got_ptr = 1'b0; rw = 1'b0; ack_this = 1'b0;
        master_ack = 1'b0; after_nack = 1'b0; wait_clks = 0;
        start_pending = 1'b0; stop_seen = 1'b0;
        reg0 = REG0_INIT; reg1 = REG1_INIT; reg_ptr = 8'h00;
        last_addr_byte = 8'h00; last_wr_data = 8'h00; last_rd_data = 8'h00;
        start_cnt = 0; stop_cnt = 0; wr_cnt = 0; rd_cnt = 0; err_count = 0;
        err_bus_undef = 1'b0; err_start_mid_byte = 1'b0; err_stop_mid_byte = 1'b0;
        err_timing = 1'b0; err_bad_reg = 1'b0; err_clk_after_nack = 1'b0;
        info_addr_nack = 1'b0;
        t_scl_rise = -1.0; t_scl_fall = -1.0; t_sda_chg = -1.0;
        t_sda_fall = -1.0; t_sda_rise = -1.0; t_start = -1.0; t_stop = -1.0;
    end

    // State name for the waveform viewer (radix: ASCII)
    always @(phase or got_addr or got_ptr or rw) begin
        case (phase)
            PH_IDLE:   state_ascii = "IDLE";
            PH_RX:     state_ascii = !got_addr ? "ADDR" : (!got_ptr ? "REG_PTR" : "WR_DATA");
            PH_RX_ACK: state_ascii = "SLV_ACK";
            PH_TX:     state_ascii = "RD_DATA";
            PH_TX_ACK: state_ascii = "MST_ACK";
            default:   state_ascii = "WAIT";
        endcase
    end

    //-------------------------------------------------------------------------
    // Reporting helpers
    //-------------------------------------------------------------------------
    task report_err;
        input [8*80-1:0] msg;
        begin
            err_count = err_count + 1;
            if (err_count <= MAX_MSGS)
                $display("[%0d ns] %0s ERROR: %0s", $time, NAME, msg);
        end
    endtask

    task check_min;
        input real       meas;
        input integer    limit;
        input [8*10-1:0] pname;
        begin
            if (TIMING_CHECK != 0 && meas < limit) begin
                err_timing = 1'b1;
                err_count  = err_count + 1;
                if (err_count <= MAX_MSGS)
                    $display("[%0d ns] %0s ERROR: timing %0s = %0.1f ns, minimum %0d ns",
                             $time, NAME, pname, meas, limit);
            end
        end
    endtask

    // Call from the testbench:  <instance>.clear_flags;
    task clear_flags;
        begin
            err_bus_undef = 1'b0; err_start_mid_byte = 1'b0; err_stop_mid_byte = 1'b0;
            err_timing = 1'b0; err_bad_reg = 1'b0; err_clk_after_nack = 1'b0;
            info_addr_nack = 1'b0; err_count = 0;
        end
    endtask

    //-------------------------------------------------------------------------
    // Byte-level behaviour
    //-------------------------------------------------------------------------
    // Called when the 8th bit of a master byte has been received.
    task byte_received;
        begin
            ack_this = 1'b0;
            if (!got_addr) begin
                got_addr       = 1'b1;
                last_addr_byte = shreg;
                if (shreg[7:1] === SLAVE_ADDR) begin
                    rw       = shreg[0];
                    got_ptr  = 1'b0;
                    ack_this = 1'b1;
                end else begin
                    info_addr_nack = 1'b1;
                    if (VERBOSE != 0)
                        $display("[%0d ns] %0s: address byte 0x%02h is not mine, ignored",
                                 $time, NAME, shreg);
                end
            end else if (!got_ptr) begin
                if (shreg !== 8'h00 && shreg !== 8'h01) begin
                    err_bad_reg = 1'b1;
                    report_err("register pointer out of range (only 0 and 1 exist), NACK");
                end else begin
                    reg_ptr  = shreg;
                    got_ptr  = 1'b1;
                    ack_this = 1'b1;
                end
            end else begin
                if (reg_ptr[0]) reg1 = shreg; else reg0 = shreg;
                last_wr_data = shreg;
                wr_cnt       = wr_cnt + 1;
                if (VERBOSE != 0)
                    $display("[%0d ns] %0s: WRITE reg%0d = 0x%02h", $time, NAME, reg_ptr[0], shreg);
                reg_ptr  = {7'b0, ~reg_ptr[0]};
                ack_this = 1'b1;
            end
        end
    endtask

    // Fetch the next byte the master will read.
    task load_tx;
        begin
            txreg        = reg_ptr[0] ? reg1 : reg0;
            last_rd_data = txreg;
            rd_cnt       = rd_cnt + 1;
            if (VERBOSE != 0)
                $display("[%0d ns] %0s: READ  reg%0d -> 0x%02h", $time, NAME, reg_ptr[0], txreg);
            reg_ptr = {7'b0, ~reg_ptr[0]};
        end
    endtask

    //-------------------------------------------------------------------------
    // START / STOP
    //-------------------------------------------------------------------------
    task do_start;
        input real t;
        begin
            start_cnt = start_cnt + 1;
            if (busy) begin                                   // repeated START
                check_min(t - t_scl_rise, T_SU_STA_NS, "tSU;STA");
                if (!((phase == PH_RX && bitcnt <= 4'd1) || phase == PH_WAIT)) begin
                    err_start_mid_byte = 1'b1;
                    report_err("START in the middle of a byte or ACK bit");
                end
            end else if (stop_seen) begin
                check_min(t - t_stop, T_BUF_NS, "tBUF");
            end
            busy          = 1'b1;
            phase         = PH_RX;
            bitcnt        = 4'd0;
            got_addr      = 1'b0;
            after_nack    = 1'b0;
            wait_clks     = 0;
            start_pending = 1'b1;
            t_start       = t;
            sda_low      <= 1'b0;
        end
    endtask

    task do_stop;
        input real t;
        begin
            if (busy) begin
                stop_cnt = stop_cnt + 1;
                check_min(t - t_scl_rise, T_SU_STO_NS, "tSU;STO");
                if (!((phase == PH_RX && bitcnt <= 4'd1) || phase == PH_WAIT)) begin
                    err_stop_mid_byte = 1'b1;
                    report_err("STOP in the middle of a byte or ACK bit");
                end
            end
            busy       = 1'b0;
            phase      = PH_IDLE;
            after_nack = 1'b0;
            stop_seen  = 1'b1;
            t_stop     = t;
            sda_low   <= 1'b0;
        end
    endtask

    // The 1 ps settle lets SCL and SDA edges that land in the same time step
    // be ordered correctly: SDA moving together with an SCL edge is not a
    // START/STOP (it is a set-up violation, reported by the timing check).
    always @(negedge sda) begin
        t_sda_fall = $realtime;
        #0.001;
        if (sda === 1'b0 && scl === 1'b1 && t_scl_rise < t_sda_fall)
            do_start(t_sda_fall);
    end

    always @(posedge sda) begin
        t_sda_rise = $realtime;
        #0.001;
        if (sda === 1'b1 && scl === 1'b1 && t_scl_rise < t_sda_rise)
            do_stop(t_sda_rise);
    end

    always @(sda) begin
        t_sda_chg = $realtime;
        if ($time > 0 && sda === 1'bx) begin
            err_bus_undef = 1'b1;
            report_err("SDA is X: bus contention - is a device driving a hard 1? (open-drain only)");
        end
    end

    always @(scl) begin
        if ($time > 0 && scl !== 1'b0 && scl !== 1'b1) begin
            err_bus_undef = 1'b1;
            report_err("SCL is Z/X: missing pull-up on SCL, or contention");
        end
    end

    //-------------------------------------------------------------------------
    // SCL rising edge: sample
    //-------------------------------------------------------------------------
    always @(posedge scl) begin
        if (scl === 1'b1) begin
            t_scl_rise = $realtime;
            #0.001;
            if (busy) begin
                check_min(t_scl_rise - t_scl_fall, T_LOW_NS,    "tLOW");
                check_min(t_scl_rise - t_sda_chg,  T_SU_DAT_NS, "tSU;DAT");
                if (sda !== 1'b0 && sda !== 1'b1) begin
                    err_bus_undef = 1'b1;
                    report_err("SDA is Z/X at the SCL rising edge: missing pull-up on SDA?");
                end
                case (phase)
                    PH_RX: if (bitcnt < 4'd8) begin
                        shreg  = {shreg[6:0], sda};
                        bitcnt = bitcnt + 4'd1;
                    end
                    PH_TX:     bitcnt = bitcnt + 4'd1;
                    PH_TX_ACK: master_ack = (sda === 1'b0);
                    PH_WAIT: if (after_nack) begin
                        // one more rising edge is normal (it belongs to the
                        // STOP or repeated START); two means more clocking
                        wait_clks = wait_clks + 1;
                        if (wait_clks == 2) begin
                            err_clk_after_nack = 1'b1;
                            report_err("master keeps clocking after it NACKed the read");
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

    //-------------------------------------------------------------------------
    // SCL falling edge: advance, drive SDA after T_OUT_NS
    //-------------------------------------------------------------------------
    always @(negedge scl) begin
        if (scl === 1'b0) begin
            t_scl_fall = $realtime;
            if (busy) begin
                check_min(t_scl_fall - t_scl_rise, T_HIGH_NS, "tHIGH");
                if (start_pending) begin
                    check_min(t_scl_fall - t_start, T_HD_STA_NS, "tHD;STA");
                    start_pending = 1'b0;
                end
                case (phase)
                    PH_RX: if (bitcnt == 4'd8) begin
                        byte_received;
                        if (ack_this) begin
                            sda_low <= #(T_OUT_NS) 1'b1;          // ACK
                            phase    = PH_RX_ACK;
                        end else begin
                            phase     = PH_WAIT;                   // NACK: stay released
                            wait_clks = 0;
                        end
                    end
                    PH_RX_ACK: begin                               // end of the ACK bit
                        bitcnt = 4'd0;
                        if (rw) begin
                            load_tx;
                            sda_low <= #(T_OUT_NS) ~txreg[7];
                            phase    = PH_TX;
                        end else begin
                            sda_low <= #(T_OUT_NS) 1'b0;
                            phase    = PH_RX;
                        end
                    end
                    PH_TX: begin
                        if (bitcnt == 4'd8) begin
                            sda_low   <= #(T_OUT_NS) 1'b0;         // release for master ACK
                            master_ack = 1'b0;
                            phase      = PH_TX_ACK;
                        end else if (bitcnt != 4'd0) begin
                            sda_low <= #(T_OUT_NS) ~txreg[7 - bitcnt];
                        end
                    end
                    PH_TX_ACK: begin
                        bitcnt = 4'd0;
                        if (master_ack) begin
                            load_tx;
                            sda_low <= #(T_OUT_NS) ~txreg[7];
                            phase    = PH_TX;
                        end else begin
                            sda_low   <= #(T_OUT_NS) 1'b0;
                            phase      = PH_WAIT;
                            after_nack = 1'b1;
                            wait_clks  = 0;
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule
