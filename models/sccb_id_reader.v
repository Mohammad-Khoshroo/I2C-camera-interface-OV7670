`timescale 1ns / 1ps

module sccb_id_reader #(
    parameter [7:0]  SLAVE_WR = 8'h42,
    parameter [7:0]  SLAVE_RD = 8'h43,
    parameter [7:0]  REG_PID  = 8'h0A,
    parameter [7:0]  REG_VER  = 8'h0B,
    parameter [7:0]  PID_EXP  = 8'h76,
    parameter [7:0]  VER_EXP  = 8'h73,
    parameter [15:0] CLK_DIV  = 16'd250     // 100 MHz -> SCL = 100 kHz
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          // button (level), one run per rising edge

    // interface to i2c_master_core
    output reg         start_txn,
    output reg         gen_start,
    output reg         stop_bit,
    output reg         rw_bit,
    output reg         nack_on_read,
    output reg  [7:0]  tx_data,
    output wire [15:0] clk_div,
    input  wire [7:0]  rx_data,
    input  wire        done,
    input  wire        ack_error,

    // results
    output reg         id_match,       // 1: PID and VER both matched (LED)
    output reg         busy,           // run in progress
    output reg         finished,       // 1-cycle pulse at end of a run
    output reg         ack_fail,       // some byte was not ACKed
    output reg         id_mismatch,    // read completed but IDs differ
    output reg  [7:0]  pid_val,
    output reg  [7:0]  ver_val
);

    assign clk_div = CLK_DIV;

    // ---------- start synchronizer + rising-edge detector ----------
    reg s1, s2, s3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1 <= 1'b0; s2 <= 1'b0; s3 <= 1'b0;
        end else begin
            s1 <= start; s2 <= s1; s3 <= s2;
        end
    end
    wire start_pulse = s2 & ~s3;

    // ---------- FSM ----------
    localparam [2:0] S_IDLE   = 3'd0,
                     S_LOAD   = 3'd1,
                     S_FIRE   = 3'd2,
                     S_WAIT   = 3'd3,
                     S_CHECK  = 3'd4,
                     S_FINISH = 3'd5;

    reg [2:0] state;
    reg [1:0] step;       // byte index inside one register read (0..3)
    reg       reg_sel;    // 0: PID, 1: VER
    reg       err;        // sticky ACK error of the current run

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            step         <= 2'd0;
            reg_sel      <= 1'b0;
            err          <= 1'b0;
            start_txn    <= 1'b0;
            gen_start    <= 1'b0;
            stop_bit     <= 1'b0;
            rw_bit       <= 1'b0;
            nack_on_read <= 1'b1;
            tx_data      <= 8'd0;
            id_match     <= 1'b0;
            busy         <= 1'b0;
            finished     <= 1'b0;
            ack_fail     <= 1'b0;
            id_mismatch  <= 1'b0;
            pid_val      <= 8'd0;
            ver_val      <= 8'd0;
        end else begin
            start_txn <= 1'b0;
            finished  <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start_pulse) begin
                        busy        <= 1'b1;
                        id_match    <= 1'b0;
                        ack_fail    <= 1'b0;
                        id_mismatch <= 1'b0;
                        err         <= 1'b0;
                        pid_val     <= 8'd0;
                        ver_val     <= 8'd0;
                        step        <= 2'd0;
                        reg_sel     <= 1'b0;
                        state       <= S_LOAD;
                    end
                end

                // set core inputs for the current byte
                S_LOAD: begin
                    nack_on_read <= 1'b1;
                    case (step)
                        2'd0: begin
                            tx_data <= SLAVE_WR; gen_start <= 1'b1;
                            stop_bit <= 1'b0;    rw_bit <= 1'b0;
                        end
                        2'd1: begin
                            tx_data <= reg_sel ? REG_VER : REG_PID; gen_start <= 1'b0;
                            stop_bit <= 1'b1;    rw_bit <= 1'b0;
                        end
                        2'd2: begin
                            tx_data <= SLAVE_RD; gen_start <= 1'b1;
                            stop_bit <= 1'b0;    rw_bit <= 1'b0;
                        end
                        2'd3: begin
                            tx_data <= 8'h00;    gen_start <= 1'b0;
                            stop_bit <= 1'b1;    rw_bit <= 1'b1;
                        end
                    endcase
                    state <= S_FIRE;
                end

                // one-cycle start pulse to the core
                S_FIRE: begin
                    start_txn <= 1'b1;
                    state     <= S_WAIT;
                end

                // wait for the core to finish this byte
                S_WAIT: begin
                    if (done) begin
                        if (ack_error) err <= 1'b1;
                        if (step == 2'd3) begin
                            if (reg_sel) ver_val <= rx_data;
                            else         pid_val <= rx_data;
                        end
                        state <= S_CHECK;
                    end
                end

                S_CHECK: begin
                    if (step == 2'd1 || step == 2'd3) begin
                        // a transaction (ending with STOP) just finished
                        if (err) begin
                            state <= S_FINISH;               // abort run
                        end else if (step == 2'd3) begin
                            if (reg_sel) begin
                                state <= S_FINISH;           // both regs read
                            end else begin
                                reg_sel <= 1'b1;
                                step    <= 2'd0;
                                state   <= S_LOAD;
                            end
                        end else begin
                            step  <= step + 2'd1;
                            state <= S_LOAD;
                        end
                    end else begin
                        step  <= step + 2'd1;
                        state <= S_LOAD;
                    end
                end

                S_FINISH: begin
                    busy        <= 1'b0;
                    finished    <= 1'b1;
                    ack_fail    <= err;
                    id_mismatch <= ~err & ~((pid_val == PID_EXP) & (ver_val == VER_EXP));
                    id_match    <= ~err &  ((pid_val == PID_EXP) & (ver_val == VER_EXP));
                    state       <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule