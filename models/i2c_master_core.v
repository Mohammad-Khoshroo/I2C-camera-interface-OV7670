`timescale 1ns / 1ps

module i2c_master_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start_txn,
    input  wire        gen_start,
    input  wire        stop_bit,
    input  wire        rw_bit,         // 0: Write, 1: Read
    input  wire        nack_on_read,   // 1: Master NACKs read byte
    input  wire [7:0]  tx_data,
    input  wire [15:0] clk_div,        // e.g. 250 for 100 kHz at 100 MHz clk
    
    output reg  [7:0]  rx_data,
    output reg         busy,
    output reg         ack_error,
    output reg         done,
    
    output wire        scl_o,          // Constant 0
    output reg         scl_t,          // 0: Drive Low, 1: Release (High-Z)
    input  wire        scl_i,
    
    output wire        sda_o,          // Constant 0
    output reg         sda_t,          // 0: Drive Low, 1: Release (High-Z)
    input  wire        sda_i
);

    assign scl_o = 1'b0;
    assign sda_o = 1'b0;

    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_START     = 3'd1,
        ST_DATA      = 3'd2,
        ST_ACK_WRITE = 3'd3,
        ST_ACK_READ  = 3'd4,
        ST_STOP      = 3'd5,
        ST_DONE      = 3'd6;

    reg [2:0]  state;
    reg [15:0] div_cnt;
    reg [1:0]  phase;
    reg [2:0]  bit_idx;

    reg [7:0]  shreg_tx;
    reg [7:0]  shreg_rx;
    reg        r_stop_bit;
    reg        r_rw_bit;
    reg        r_nack;

    wire tick = (div_cnt >= clk_div - 1'b1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            busy       <= 1'b0;
            ack_error  <= 1'b0;
            done       <= 1'b0;
            scl_t      <= 1'b1;
            sda_t      <= 1'b1;
            div_cnt    <= 16'd0;
            phase      <= 2'd0;
            bit_idx    <= 3'd7;
            rx_data    <= 8'd0;
            shreg_tx   <= 8'd0;
            shreg_rx   <= 8'd0;
            r_stop_bit <= 1'b0;
            r_rw_bit   <= 1'b0;
            r_nack     <= 1'b0;
        end else begin
            done <= 1'b0; // Default 1-cycle pulse

            case (state)
                ST_IDLE: begin
                    div_cnt <= 16'd0;
                    phase   <= 2'd0;
                    if (start_txn) begin
                        busy       <= 1'b1;
                        shreg_tx   <= tx_data;
                        r_stop_bit <= stop_bit;
                        r_rw_bit   <= rw_bit;
                        r_nack     <= nack_on_read;
                        bit_idx    <= 3'd7;
                        ack_error  <= 1'b0;
                        if (gen_start) begin
                            state <= ST_START;
                        end else begin
                            // Continuous transfer: drive bit 7 immediately during phase 0
                            scl_t <= 1'b0;
                            sda_t <= (!rw_bit) ? tx_data[7] : 1'b1;
                            state <= ST_DATA;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                ST_START: begin
                    if (tick) begin
                        div_cnt <= 16'd0;
                        phase   <= phase + 1'b1;
                        case (phase)
                            2'd0: begin sda_t <= 1'b1; end
                            2'd1: begin scl_t <= 1'b1; sda_t <= 1'b1; end
                            2'd2: begin scl_t <= 1'b1; sda_t <= 1'b0; end // Falling edge on SDA
                            2'd3: begin
                                scl_t   <= 1'b0;
                                sda_t   <= (!r_rw_bit) ? shreg_tx[7] : 1'b1; // Setup bit 7
                                bit_idx <= 3'd7;
                                state   <= ST_DATA;
                            end
                        endcase
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                ST_DATA: begin
                    if (tick) begin
                        div_cnt <= 16'd0;
                        phase   <= phase + 1'b1;
                        case (phase)
                            2'd0: begin
                                scl_t <= 1'b0;
                                sda_t <= (!r_rw_bit) ? shreg_tx[bit_idx] : 1'b1;
                            end
                            2'd1: begin
                                scl_t <= 1'b1;
                            end
                            2'd2: begin
                                scl_t <= 1'b1;
                                if (r_rw_bit)
                                    shreg_rx[bit_idx] <= sda_i;
                            end
                            2'd3: begin
                                scl_t <= 1'b0;
                                if (bit_idx == 3'd0) begin
                                    if (!r_rw_bit) begin
                                        sda_t <= 1'b1; // Release for slave ACK
                                        state <= ST_ACK_WRITE;
                                    end else begin
                                        sda_t <= r_nack ? 1'b1 : 1'b0;
                                        state <= ST_ACK_READ;
                                    end
                                end else begin
                                    bit_idx <= bit_idx - 1'b1;
                                    // Pre-drive next bit during SCL low
                                    if (!r_rw_bit)
                                        sda_t <= shreg_tx[bit_idx - 1'b1];
                                end
                            end
                        endcase
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                ST_ACK_WRITE: begin
                    if (tick) begin
                        div_cnt <= 16'd0;
                        phase   <= phase + 1'b1;
                        case (phase)
                            2'd0: begin scl_t <= 1'b0; sda_t <= 1'b1; end
                            2'd1: begin scl_t <= 1'b1; end
                            2'd2: begin
                                scl_t <= 1'b1;
                                if (sda_i !== 1'b0)
                                    ack_error <= 1'b1;
                            end
                            2'd3: begin
                                scl_t <= 1'b0;
                                if (r_stop_bit) begin
                                    sda_t <= 1'b0; // Prepare SDA low for STOP rise
                                    state <= ST_STOP;
                                end else begin
                                    state <= ST_DONE;
                                end
                            end
                        endcase
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                ST_ACK_READ: begin
                    if (tick) begin
                        div_cnt <= 16'd0;
                        phase   <= phase + 1'b1;
                        case (phase)
                            2'd0: begin scl_t <= 1'b0; sda_t <= r_nack ? 1'b1 : 1'b0; end
                            2'd1: begin scl_t <= 1'b1; end
                            2'd2: begin scl_t <= 1'b1; end
                            2'd3: begin
                                scl_t   <= 1'b0;
                                rx_data <= shreg_rx;
                                if (r_stop_bit) begin
                                    sda_t <= 1'b0;
                                    state <= ST_STOP;
                                end else begin
                                    state <= ST_DONE;
                                end
                            end
                        endcase
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                ST_STOP: begin
                    if (tick) begin
                        div_cnt <= 16'd0;
                        phase   <= phase + 1'b1;
                        case (phase)
                            2'd0: begin scl_t <= 1'b0; sda_t <= 1'b0; end
                            2'd1: begin scl_t <= 1'b1; sda_t <= 1'b0; end
                            2'd2: begin scl_t <= 1'b1; sda_t <= 1'b1; end // Rising edge on SDA
                            2'd3: begin
                                scl_t <= 1'b1;
                                sda_t <= 1'b1;
                                state <= ST_DONE;
                            end
                        endcase
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                // Pulsed immediately without waiting for a division tick
                ST_DONE: begin
                    done    <= 1'b1;
                    busy    <= 1'b0;
                    div_cnt <= 16'd0;
                    phase   <= 2'd0;
                    state   <= ST_IDLE;
                    if (!r_stop_bit)
                        scl_t <= 1'b0; // Keep SCL low between bytes
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule