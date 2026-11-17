`timescale 1ns / 1ps

module ov7670_id_reader #(
    parameter [7:0]  DEV_ADDR_W = 8'h42,
    parameter [7:0]  DEV_ADDR_R = 8'h43,
    parameter [7:0]  PID_REG    = 8'h0A,
    parameter [7:0]  VER_REG    = 8'h0B,
    parameter [7:0]  PID_EXP    = 8'h76,
    parameter [7:0]  VER_EXP    = 8'h73,
    parameter [15:0] CLK_DIV    = 16'd250
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          // Testbench input
    input  wire        start_btn,      // Board push-button input

    // Interface to i2c_master_core
    output reg         start_txn,
    output reg         gen_start,
    output reg         stop_bit,
    output reg         rw_bit,
    output reg         nack_on_read,
    output reg  [7:0]  tx_data,
    output wire [15:0] clk_div,
    input  wire [7:0]  rx_data,
    input  wire        busy,           // from core
    input  wire        done,
    input  wire        ack_error,

    // Status outputs for Testbench & Hardware
    output reg         id_match,       // 1: PID & VER match
    output reg         id_mismatch,    // 1: IDs differ
    output reg         ack_fail,       // 1: ACK error
    output reg         finished,       // 1-cycle pulse at end
    output wire        id_ok_led,      // Board LED (match)
    output wire        id_fail_led,    // Board LED (mismatch or error)
    output wire        probe_busy,     // Busy status
    output reg  [7:0]  pid_val,
    output reg  [7:0]  ver_val
);

    assign clk_div     = CLK_DIV;
    assign id_ok_led   = id_match;
    assign id_fail_led = id_mismatch | ack_fail;

    // Edge-detect for start signal (synchronizer + rising edge)
    wire start_raw = start | start_btn;
    reg  s1, s2, s3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1 <= 1'b0; s2 <= 1'b0; s3 <= 1'b0;
        end else begin
            s1 <= start_raw; s2 <= s1; s3 <= s2;
        end
    end
    wire start_pulse = s2 & ~s3;

    localparam [3:0]
        IDLE        = 4'd0,
        W_PID_ADDR  = 4'd1,
        W_PID_REG   = 4'd2,
        R_PID_ADDR  = 4'd3,
        R_PID_DATA  = 4'd4,
        W_VER_ADDR  = 4'd5,
        W_VER_REG   = 4'd6,
        R_VER_ADDR  = 4'd7,
        R_VER_DATA  = 4'd8,
        CHECK       = 4'd9,
        FINISH      = 4'd10;

    reg [3:0] state;
    reg       running;

    assign probe_busy = running | busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            running      <= 1'b0;
            start_txn    <= 1'b0;
            gen_start    <= 1'b0;
            stop_bit     <= 1'b0;
            rw_bit       <= 1'b0;
            nack_on_read <= 1'b0;
            tx_data      <= 8'h00;
            pid_val      <= 8'h00;
            ver_val      <= 8'h00;
            id_match     <= 1'b0;
            id_mismatch  <= 1'b0;
            ack_fail     <= 1'b0;
            finished     <= 1'b0;
        end else begin
            start_txn <= 1'b0; // Default 1-cycle pulse
            finished  <= 1'b0;

            case (state)
                IDLE: begin
                    running <= 1'b0;
                    if (start_pulse) begin
                        running     <= 1'b1;
                        id_match    <= 1'b0;
                        id_mismatch <= 1'b0;
                        ack_fail    <= 1'b0;
                        pid_val     <= 8'h00;
                        ver_val     <= 8'h00;

                        tx_data     <= DEV_ADDR_W;
                        rw_bit      <= 1'b0;
                        gen_start   <= 1'b1;
                        stop_bit    <= 1'b0;
                        start_txn   <= 1'b1;
                        state       <= W_PID_ADDR;
                    end
                end

                W_PID_ADDR: begin
                    if (done) begin
                        if (ack_error) begin
                            ack_fail <= 1'b1;
                            state    <= FINISH;
                        end else begin
                            tx_data   <= PID_REG;
                            rw_bit    <= 1'b0;
                            gen_start <= 1'b0;
                            stop_bit  <= 1'b1; // SCCB STOP after setting register
                            start_txn <= 1'b1;
                            state     <= W_PID_REG;
                        end
                    end
                end

                W_PID_REG: begin
                    if (done) begin
                        if (ack_error) begin
                            ack_fail <= 1'b1;
                            state    <= FINISH;
                        end else begin
                            tx_data   <= DEV_ADDR_R;
                            rw_bit    <= 1'b0;
                            gen_start <= 1'b1; // New START
                            stop_bit  <= 1'b0;
                            start_txn <= 1'b1;
                            state     <= R_PID_ADDR;
                        end
                    end
                end

                R_PID_ADDR: begin
                    if (done) begin
                        if (ack_error) begin
                            ack_fail <= 1'b1;
                            state    <= FINISH;
                        end else begin
                            rw_bit       <= 1'b1; // Read mode
                            gen_start    <= 1'b0;
                            stop_bit     <= 1'b1;
                            nack_on_read <= 1'b1; // NACK on read
                            start_txn    <= 1'b1;
                            state        <= R_PID_DATA;
                        end
                    end
                end

                R_PID_DATA: begin
                    if (done) begin
                        pid_val   <= rx_data;
                        tx_data   <= DEV_ADDR_W;
                        rw_bit    <= 1'b0;
                        gen_start <= 1'b1;
                        stop_bit  <= 1'b0;
                        start_txn <= 1'b1;
                        state     <= W_VER_ADDR;
                    end
                end

                W_VER_ADDR: begin
                    if (done) begin
                        if (ack_error) begin
                            ack_fail <= 1'b1;
                            state    <= FINISH;
                        end else begin
                            tx_data   <= VER_REG;
                            rw_bit    <= 1'b0;
                            gen_start <= 1'b0;
                            stop_bit  <= 1'b1; // STOP after setting register
                            start_txn <= 1'b1;
                            state     <= W_VER_REG;
                        end
                    end
                end

                W_VER_REG: begin
                    if (done) begin
                        if (ack_error) begin
                            ack_fail <= 1'b1;
                            state    <= FINISH;
                        end else begin
                            tx_data   <= DEV_ADDR_R;
                            rw_bit    <= 1'b0;
                            gen_start <= 1'b1; // New START
                            stop_bit  <= 1'b0;
                            start_txn <= 1'b1;
                            state     <= R_VER_ADDR;
                        end
                    end
                end

                R_VER_ADDR: begin
                    if (done) begin
                        if (ack_error) begin
                            ack_fail <= 1'b1;
                            state    <= FINISH;
                        end else begin
                            rw_bit       <= 1'b1; // Read mode
                            gen_start    <= 1'b0;
                            stop_bit     <= 1'b1;
                            nack_on_read <= 1'b1;
                            start_txn    <= 1'b1;
                            state        <= R_VER_DATA;
                        end
                    end
                end

                R_VER_DATA: begin
                    if (done) begin
                        ver_val <= rx_data;
                        state   <= CHECK;
                    end
                end

                CHECK: begin
                    if (pid_val == PID_EXP && ver_val == VER_EXP) begin
                        id_match    <= 1'b1;
                        id_mismatch <= 1'b0;
                    end else begin
                        id_match    <= 1'b0;
                        id_mismatch <= 1'b1;
                    end
                    state <= FINISH;
                end

                FINISH: begin
                    running  <= 1'b0;
                    finished <= 1'b1; // 1-cycle pulse
                    state    <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
