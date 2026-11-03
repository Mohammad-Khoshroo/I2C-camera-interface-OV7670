`timescale 1ns / 1ps

module ov7670_id_reader (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start_btn,   // Debounced button input

    // To the I2C core
    output reg         start_txn,
    output reg         gen_start,
    output reg         stop_bit,
    output reg         rw_bit,
    output reg         nack_on_read,
    output reg  [7:0]  tx_data,
    input  wire [7:0]  rx_data,
    input  wire        busy,
    input  wire        done,
    input  wire        ack_error,

    // Status & ILA Probes
    output reg         id_ok_led,
    output reg         id_fail_led,
    output wire        probe_busy,
    output reg  [7:0]  pid_val,
    output reg  [7:0]  ver_val
);

    localparam DEV_ADDR_W = 8'h42,
               DEV_ADDR_R = 8'h43,
               PID_REG    = 8'h0A,
               VER_REG    = 8'h0B,
               PID_VAL    = 8'h76,
               VER_VAL    = 8'h73;

    localparam [3:0]
        IDLE        = 4'd0,
        // PID Read (0x0A): 2-phase write + STOP, then 2-phase read + STOP
        W_PID_ADDR  = 4'd1,
        W_PID_REG   = 4'd2,
        R_PID_ADDR  = 4'd3,
        R_PID_DATA  = 4'd4,
        // VER Read (0x0B): 2-phase write + STOP, then 2-phase read + STOP
        W_VER_ADDR  = 4'd5,
        W_VER_REG   = 4'd6,
        R_VER_ADDR  = 4'd7,
        R_VER_DATA  = 4'd8,
        CHECK       = 4'd9,
        FINISH      = 4'd10;

    reg [3:0] state;
    reg       start_btn_d;
    reg       err_flag;

    // Edge detection for push button
    wire btn_pressed = start_btn && !start_btn_d;

    // Bus is considered busy whenever the FSM is running or the I2C core reports busy
    assign probe_busy = (state != IDLE && state != FINISH) || busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            start_txn    <= 1'b0;
            gen_start    <= 1'b0;
            stop_bit     <= 1'b0;
            rw_bit       <= 1'b0;
            nack_on_read <= 1'b0;
            tx_data      <= 8'h00;
            pid_val      <= 8'h00;
            ver_val      <= 8'h00;
            id_ok_led    <= 1'b0;
            id_fail_led  <= 1'b0;
            err_flag     <= 1'b0;
            start_btn_d  <= 1'b0;
        end else begin
            start_btn_d <= start_btn;
            start_txn   <= 1'b0; // Default 1-cycle pulse

            // Track any ACK failure received during any transaction
            if (done && ack_error)
                err_flag <= 1'b1;

            case (state)
                IDLE: begin
                    if (btn_pressed) begin
                        id_ok_led   <= 1'b0;
                        id_fail_led <= 1'b0;
                        err_flag    <= 1'b0;
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
                        tx_data   <= PID_REG;
                        rw_bit    <= 1'b0;
                        gen_start <= 1'b0;
                        stop_bit  <= 1'b1; // SCCB requires STOP after setting sub-address
                        start_txn <= 1'b1;
                        state     <= W_PID_REG;
                    end
                end

                W_PID_REG: begin
                    if (done) begin
                        tx_data   <= DEV_ADDR_R;
                        rw_bit    <= 1'b0; // Address byte is transmitted by master
                        gen_start <= 1'b1; // New START
                        stop_bit  <= 1'b0;
                        start_txn <= 1'b1;
                        state     <= R_PID_ADDR;
                    end
                end

                R_PID_ADDR: begin
                    if (done) begin
                        rw_bit       <= 1'b1; // Master reads incoming byte
                        gen_start    <= 1'b0;
                        stop_bit     <= 1'b1;
                        nack_on_read <= 1'b1; // Master sends NACK on final byte
                        start_txn    <= 1'b1;
                        state        <= R_PID_DATA;
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
                        tx_data   <= VER_REG;
                        rw_bit    <= 1'b0;
                        gen_start <= 1'b0;
                        stop_bit  <= 1'b1; // STOP after setting sub-address
                        start_txn <= 1'b1;
                        state     <= W_VER_REG;
                    end
                end

                W_VER_REG: begin
                    if (done) begin
                        tx_data   <= DEV_ADDR_R;
                        rw_bit    <= 1'b0;
                        gen_start <= 1'b1; // New START
                        stop_bit  <= 1'b0;
                        start_txn <= 1'b1;
                        state     <= R_VER_ADDR;
                    end
                end

                R_VER_ADDR: begin
                    if (done) begin
                        rw_bit       <= 1'b1;
                        gen_start    <= 1'b0;
                        stop_bit     <= 1'b1;
                        nack_on_read <= 1'b1;
                        start_txn    <= 1'b1;
                        state        <= R_VER_DATA;
                    end
                end

                R_VER_DATA: begin
                    if (done) begin
                        ver_val <= rx_data;
                        state   <= CHECK;
                    end
                end

                CHECK: begin
                    if (!err_flag && pid_val == PID_VAL && ver_val == VER_VAL) begin
                        id_ok_led   <= 1'b1;
                        id_fail_led <= 1'b0;
                    end else begin
                        id_ok_led   <= 1'b0;
                        id_fail_led <= 1'b1;
                    end
                    state <= FINISH;
                end

                FINISH: begin
                    // Wait until the user presses the button again to start a new check
                    if (btn_pressed)
                        state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule