`timescale 1ns / 1ps
//=============================================================================
// ov7670_sccb_golden.v      *** SIMULATION ONLY -- NOT SYNTHESIZABLE ***
//
// Golden model of the OV7670 SCCB (register) interface, for testing the
// student SCCB FSM in simulation (Exp 1). It models ONLY the register bus:
// no XCLK, no PWDN/RESET# pins, no video output.
// Event-driven on SIO_C/SIO_D, no clock. Open-drain on SIO_D.
//
// THE TESTBENCH MUST PROVIDE THE PULL-UPS  (tri1 sio_c, sio_d;)
//
// SCCB TRANSACTIONS  (ID = 0x42 write / 0x43 read)
//   3-phase write : S  0x42 A  SUB A  DATA A  P
//   2-phase write : S  0x42 A  SUB A  P              (sets the sub-address)
//   2-phase read  : S  0x43 A  DATA Nm  P            (reads at the sub-address)
//   So one register read = 2-phase write + STOP + 2-phase read. SCCB has no
//   repeated START. The 9th bit of each phase is a "don't care" in SCCB; like
//   the real sensor this model pulls it low, so an I2C master sees an ACK.
//
// REGISTERS
//   256 x 8 bit. PID (0x0A), VER (0x0B), MIDH (0x1C), MIDL (0x1D) are
//   read-only. Reset values of the common registers are taken from the
//   OV7670 datasheet register table (see load_defaults); all others reset to
//   0x00. Check the table against your datasheet before relying on a value
//   other than PID / VER / MIDH / MIDL.
//   Writing COM7 (0x12) with bit 7 = 1 is a soft reset: all registers return
//   to their defaults and the model does not answer for T_RESET_SETTLE_NS.
//
// STRICT_SCCB = 1 (default): the non-SCCB cases below are flagged AND refused.
// STRICT_SCCB = 0          : they are flagged but handled like a plain I2C
//                            device (repeated START works, pointer increments)
//
// DEBUG OUTPUTS  (sticky until  <instance>.clear_flags;  is called)
//   err_bus_undef          SIO_C/SIO_D read as Z or X (missing pull-up, or a
//                          hard 1 driven against a device pulling low)
//   err_start_mid_byte     START in the middle of a byte / 9th bit
//   err_stop_mid_byte      STOP  in the middle of a byte / 9th bit
//   err_timing             a bus timing parameter below its minimum
//   err_clk_after_nack     master kept clocking after the end of a read
//   err_repeated_start     camera addressed after a repeated START
//   err_multi_byte_write   more than one data byte in a write
//   err_master_ack_on_read master ACKed the read byte (SCCB wants NACK)
//   err_write_read_only    write to PID / VER / MIDH / MIDL
//   err_access_in_reset    camera addressed too soon after a COM7 soft reset
//   info_addr_nack         a different device address was seen (not an error)
//   err_any / err_count    OR of the err_* flags / number of error events
//   state_ascii            current state as text - show as ASCII in the wave
//=============================================================================
module ov7670_sccb_golden #(
    parameter [6:0]   SCCB_ID           = 7'h21,       // 0x42 write, 0x43 read
    parameter [7:0]   PID_VALUE         = 8'h76,       // change to test the fail path
    parameter [7:0]   VER_VALUE         = 8'h73,
    parameter integer STRICT_SCCB       = 1,
    parameter integer T_RESET_SETTLE_NS = 1000000,     // 1 ms after COM7[7]
    parameter         NAME              = "OV7670",
    parameter integer VERBOSE           = 1,
    parameter integer MAX_MSGS          = 50,
    parameter integer T_OUT_NS          = 100,         // SIO_C fall -> SIO_D change
    parameter integer TIMING_CHECK      = 1,           // limits: 400 kHz bus
    parameter integer T_SU_DAT_NS       = 100,
    parameter integer T_HD_STA_NS       = 600,
    parameter integer T_SU_STA_NS       = 600,
    parameter integer T_SU_STO_NS       = 600,
    parameter integer T_BUF_NS          = 1300,
    parameter integer T_LOW_NS          = 1300,
    parameter integer T_HIGH_NS         = 600
)(
    input  wire        sio_c,
    inout  wire        sio_d,

    // ---- debug / observation (leave unconnected if not needed) -------------
    output reg         busy,
    output reg  [63:0] state_ascii,
    output reg  [7:0]  sub_addr,
    output reg  [7:0]  last_addr_byte,
    output reg  [7:0]  last_wr_addr,
    output reg  [7:0]  last_wr_data,
    output reg  [7:0]  last_rd_addr,
    output reg  [7:0]  last_rd_data,
    output reg  [31:0] start_cnt,
    output reg  [31:0] stop_cnt,
    output reg  [31:0] wr_cnt,
    output reg  [31:0] rd_cnt,
    output reg  [31:0] soft_reset_cnt,
    output reg         err_bus_undef,
    output reg         err_start_mid_byte,
    output reg         err_stop_mid_byte,
    output reg         err_timing,
    output reg         err_clk_after_nack,
    output reg         err_repeated_start,
    output reg         err_multi_byte_write,
    output reg         err_master_ack_on_read,
    output reg         err_write_read_only,
    output reg         err_access_in_reset,
    output reg         info_addr_nack,
    output wire        err_any,
    output reg  [31:0] err_count
);

    wire scl = sio_c;
    wire sda = sio_d;

    reg sda_low;                               // 1 = pull SIO_D low
    assign sio_d = sda_low ? 1'b0 : 1'bz;

    assign err_any = err_bus_undef | err_start_mid_byte | err_stop_mid_byte |
                     err_timing | err_clk_after_nack | err_repeated_start |
                     err_multi_byte_write | err_master_ack_on_read |
                     err_write_read_only | err_access_in_reset;

    //-------------------------------------------------------------------------
    // Register file
    //-------------------------------------------------------------------------
    reg [7:0] mem [0:255];      // testbench may read  <instance>.mem[addr]
    integer   k;

    task load_defaults;
        begin
            for (k = 0; k < 256; k = k + 1) mem[k] = 8'h00;
            mem[8'h01] = 8'h80;       // BLUE
            mem[8'h02] = 8'h80;       // RED
            mem[8'h09] = 8'h01;       // COM2
            mem[8'h0A] = PID_VALUE;   // PID   (read-only)
            mem[8'h0B] = VER_VALUE;   // VER   (read-only)
            mem[8'h0E] = 8'h01;       // COM5
            mem[8'h0F] = 8'h43;       // COM6
            mem[8'h10] = 8'h40;       // AECH
            mem[8'h11] = 8'h80;       // CLKRC
            mem[8'h12] = 8'h00;       // COM7
            mem[8'h13] = 8'h8F;       // COM8
            mem[8'h14] = 8'h4A;       // COM9
            mem[8'h15] = 8'h00;       // COM10
            mem[8'h17] = 8'h11;       // HSTART
            mem[8'h18] = 8'h61;       // HSTOP
            mem[8'h19] = 8'h03;       // VSTRT
            mem[8'h1A] = 8'h7B;       // VSTOP
            mem[8'h1C] = 8'h7F;       // MIDH  (read-only)
            mem[8'h1D] = 8'hA2;       // MIDL  (read-only)
            mem[8'h1E] = 8'h01;       // MVFP
            mem[8'h3A] = 8'h0D;       // TSLB
            mem[8'h3C] = 8'h68;       // COM12
            mem[8'h3D] = 8'h88;       // COM13
            mem[8'h40] = 8'hC0;       // COM15
            mem[8'h41] = 8'h08;       // COM16
        end
    endtask

    //-------------------------------------------------------------------------
    // State
    //-------------------------------------------------------------------------
    localparam [2:0] PH_IDLE   = 3'd0,
                     PH_RX     = 3'd1,   // receiving a phase from the master
                     PH_RX_ACK = 3'd2,   // model is driving the 9th bit low
                     PH_TX     = 3'd3,   // sending read data
                     PH_TX_ACK = 3'd4,   // master's 9th bit
                     PH_WAIT   = 3'd5;   // not addressed / finished

    reg [2:0] phase;
    reg [3:0] bitcnt;
    reg [7:0] shreg;
    reg [7:0] txreg;
    reg       got_addr;
    reg       rw;
    reg       ack_this;
    reg       master_ack;
    reg       after_nack;
    reg       rstart_seen;     // this transaction began with a repeated START
    reg       had_reset;
    reg       start_pending;
    reg       stop_seen;
    integer   byte_idx;        // bytes received after the ID byte
    integer   wait_clks;

    real t_scl_rise, t_scl_fall, t_sda_chg, t_sda_fall, t_sda_rise, t_start, t_stop;
    real t_soft_reset;

    initial begin
        sda_low = 1'b0;  busy = 1'b0;  phase = PH_IDLE;  bitcnt = 4'd0;
        shreg = 8'h00;   txreg = 8'h00;
        got_addr = 1'b0; rw = 1'b0; ack_this = 1'b0; master_ack = 1'b0;
        after_nack = 1'b0; rstart_seen = 1'b0; had_reset = 1'b0;
        start_pending = 1'b0; stop_seen = 1'b0; byte_idx = 0; wait_clks = 0;
        sub_addr = 8'h00; last_addr_byte = 8'h00;
        last_wr_addr = 8'h00; last_wr_data = 8'h00;
        last_rd_addr = 8'h00; last_rd_data = 8'h00;
        start_cnt = 0; stop_cnt = 0; wr_cnt = 0; rd_cnt = 0; soft_reset_cnt = 0;
        err_count = 0;
        err_bus_undef = 1'b0; err_start_mid_byte = 1'b0; err_stop_mid_byte = 1'b0;
        err_timing = 1'b0; err_clk_after_nack = 1'b0; err_repeated_start = 1'b0;
        err_multi_byte_write = 1'b0; err_master_ack_on_read = 1'b0;
        err_write_read_only = 1'b0; err_access_in_reset = 1'b0;
        info_addr_nack = 1'b0;
        t_scl_rise = -1.0; t_scl_fall = -1.0; t_sda_chg = -1.0;
        t_sda_fall = -1.0; t_sda_rise = -1.0; t_start = -1.0; t_stop = -1.0;
        t_soft_reset = -1.0;
        load_defaults;
    end

    always @(phase or got_addr or byte_idx) begin
        case (phase)
            PH_IDLE:   state_ascii = "IDLE";
            PH_RX:     state_ascii = !got_addr ? "ID_ADDR" : ((byte_idx == 0) ? "SUB_ADDR" : "WR_DATA");
            PH_RX_ACK: state_ascii = "DONTCARE";
            PH_TX:     state_ascii = "RD_DATA";
            PH_TX_ACK: state_ascii = "MST_NACK";
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
            err_timing = 1'b0; err_clk_after_nack = 1'b0; err_repeated_start = 1'b0;
            err_multi_byte_write = 1'b0; err_master_ack_on_read = 1'b0;
            err_write_read_only = 1'b0; err_access_in_reset = 1'b0;
            info_addr_nack = 1'b0; err_count = 0;
        end
    endtask

    //-------------------------------------------------------------------------
    // Register access
    //-------------------------------------------------------------------------
    task write_reg;
        input [7:0] a;
        input [7:0] d;
        begin
            last_wr_addr = a;
            last_wr_data = d;
            if (a == 8'h0A || a == 8'h0B || a == 8'h1C || a == 8'h1D) begin
                err_write_read_only = 1'b1;
                report_err("write to a read-only register (PID/VER/MIDH/MIDL), ignored");
            end else if (a == 8'h12 && d[7]) begin
                load_defaults;
                had_reset      = 1'b1;
                t_soft_reset   = $realtime;
                soft_reset_cnt = soft_reset_cnt + 1;
                wr_cnt         = wr_cnt + 1;
                if (VERBOSE != 0)
                    $display("[%0d ns] %0s: WRITE 0x12 = 0x%02h -> SOFT RESET, no answer for %0d ns",
                             $time, NAME, d, T_RESET_SETTLE_NS);
            end else begin
                mem[a] = d;
                wr_cnt = wr_cnt + 1;
                if (VERBOSE != 0)
                    $display("[%0d ns] %0s: WRITE 0x%02h = 0x%02h", $time, NAME, a, d);
            end
        end
    endtask

    task load_tx;
        begin
            txreg        = mem[sub_addr];
            last_rd_addr = sub_addr;
            last_rd_data = txreg;
            rd_cnt       = rd_cnt + 1;
            if (VERBOSE != 0)
                $display("[%0d ns] %0s: READ  0x%02h -> 0x%02h", $time, NAME, sub_addr, txreg);
        end
    endtask

    // Called when the 8th bit of a master byte has been received.
    task byte_received;
        begin
            ack_this = 1'b0;
            if (!got_addr) begin
                got_addr       = 1'b1;
                last_addr_byte = shreg;
                if (shreg[7:1] !== SCCB_ID) begin
                    info_addr_nack = 1'b1;
                    if (VERBOSE != 0)
                        $display("[%0d ns] %0s: ID byte 0x%02h is not mine, ignored",
                                 $time, NAME, shreg);
                end else if (had_reset && (t_start - t_soft_reset) < T_RESET_SETTLE_NS) begin
                    err_access_in_reset = 1'b1;
                    report_err("addressed too soon after COM7 soft reset, no answer");
                end else begin
                    ack_this = 1'b1;
                    if (rstart_seen) begin
                        err_repeated_start = 1'b1;
                        report_err("addressed after a repeated START - SCCB needs STOP then START");
                        if (STRICT_SCCB != 0) ack_this = 1'b0;
                    end
                    if (ack_this) begin
                        rw       = shreg[0];
                        byte_idx = 0;
                    end
                end
            end else begin
                byte_idx = byte_idx + 1;
                if (byte_idx == 1) begin
                    sub_addr = shreg;
                    ack_this = 1'b1;
                    if (VERBOSE > 1)
                        $display("[%0d ns] %0s: sub-address = 0x%02h", $time, NAME, shreg);
                end else if (byte_idx == 2) begin
                    write_reg(sub_addr, shreg);
                    ack_this = 1'b1;
                end else begin
                    err_multi_byte_write = 1'b1;
                    report_err("more than one data byte in a write - SCCB is 3-phase only");
                    if (STRICT_SCCB == 0) begin
                        sub_addr = sub_addr + 8'h01;
                        write_reg(sub_addr, shreg);
                        ack_this = 1'b1;
                    end
                end
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // START / STOP
    //-------------------------------------------------------------------------
    task do_start;
        input real t;
        begin
            start_cnt   = start_cnt + 1;
            rstart_seen = busy;
            if (busy) begin
                check_min(t - t_scl_rise, T_SU_STA_NS, "tSU;STA");
                if (!((phase == PH_RX && bitcnt <= 4'd1) || phase == PH_WAIT)) begin
                    err_start_mid_byte = 1'b1;
                    report_err("START in the middle of a byte or 9th bit");
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
                    report_err("STOP in the middle of a byte or 9th bit");
                end
            end
            busy        = 1'b0;
            phase       = PH_IDLE;
            after_nack  = 1'b0;
            rstart_seen = 1'b0;
            stop_seen   = 1'b1;
            t_stop      = t;
            sda_low    <= 1'b0;
        end
    endtask

    // The 1 ps settle orders SIO_C and SIO_D edges that land in the same time
    // step: SIO_D moving together with a SIO_C edge is not a START/STOP.
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
            report_err("SIO_D is X: bus contention - is a device driving a hard 1?");
        end
    end

    always @(scl) begin
        if ($time > 0 && scl !== 1'b0 && scl !== 1'b1) begin
            err_bus_undef = 1'b1;
            report_err("SIO_C is Z/X: missing pull-up on SIO_C, or contention");
        end
    end

    //-------------------------------------------------------------------------
    // SIO_C rising edge: sample
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
                    report_err("SIO_D is Z/X at the SIO_C rising edge: missing pull-up?");
                end
                case (phase)
                    PH_RX: if (bitcnt < 4'd8) begin
                        shreg  = {shreg[6:0], sda};
                        bitcnt = bitcnt + 4'd1;
                    end
                    PH_TX:     bitcnt = bitcnt + 4'd1;
                    PH_TX_ACK: master_ack = (sda === 1'b0);
                    PH_WAIT: if (after_nack) begin
                        // one more rising edge is normal (STOP / START)
                        wait_clks = wait_clks + 1;
                        if (wait_clks == 2) begin
                            err_clk_after_nack = 1'b1;
                            report_err("master keeps clocking after the end of the read");
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

    //-------------------------------------------------------------------------
    // SIO_C falling edge: advance, drive SIO_D after T_OUT_NS
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
                            sda_low <= #(T_OUT_NS) 1'b1;          // 9th bit low
                            phase    = PH_RX_ACK;
                        end else begin
                            phase     = PH_WAIT;
                            wait_clks = 0;
                        end
                    end
                    PH_RX_ACK: begin
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
                            sda_low   <= #(T_OUT_NS) 1'b0;         // release for the 9th bit
                            master_ack = 1'b0;
                            phase      = PH_TX_ACK;
                        end else if (bitcnt != 4'd0) begin
                            sda_low <= #(T_OUT_NS) ~txreg[7 - bitcnt];
                        end
                    end
                    PH_TX_ACK: begin
                        bitcnt = 4'd0;
                        if (master_ack) begin
                            err_master_ack_on_read = 1'b1;
                            report_err("master ACKed the read byte - SCCB expects NACK");
                        end
                        if (master_ack && STRICT_SCCB == 0) begin
                            sub_addr = sub_addr + 8'h01;
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
