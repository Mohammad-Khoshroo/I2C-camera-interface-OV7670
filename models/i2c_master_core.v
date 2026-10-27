`timescale 1ns / 1ps

module i2c_master_core (
    input  wire        clk,           // 100 MHz
    input  wire        rst_n,         // active-low

    // control
    input  wire        start_txn,     // 1-cycle pulse: start one byte transfer
    input  wire        gen_start,     // generate START before the byte
    input  wire        stop_bit,      // generate STOP after the byte
    input  wire        rw_bit,        // 0: send tx_data, 1: receive one byte
    input  wire        nack_on_read,  // read mode: 1 -> send NACK, 0 -> send ACK
    input  wire [7:0]  tx_data,
    input  wire [15:0] clk_div,       // f_scl = f_clk / (4*clk_div)  (250 -> 100 kHz)

    // status
    output reg  [7:0]  rx_data,
    output reg         busy,
    output reg         ack_error,     // write mode: Slave did not ACK
    output reg         done,          // 1-cycle pulse at end of transaction

    // open-drain pins (IOBUF: I = *_o, T = *_t, O = *_i)
    output wire        scl_o,
    output wire        scl_t,
    input  wire        scl_i,         // not used (no clock stretching)
    output wire        sda_o,
    output wire        sda_t,
    input  wire        sda_i
);

    // ---------------- states ----------------
    localparam [2:0] ST_IDLE  = 3'd0,
                     ST_START = 3'd1,
                     ST_DATA  = 3'd2,
                     ST_ACK   = 3'd3,
                     ST_STOP  = 3'd4,
                     ST_DONE  = 3'd5;

    reg [2:0]  state;
    reg [1:0]  ph;            // quarter-bit phase
    reg [2:0]  bit_cnt;
    reg [7:0]  shreg;
    reg        rw_r, stop_r, nack_r;
    reg [15:0] div_r;
    reg [15:0] cnt;

    // line control: 1 = released (high via pull-up), 0 = driven low
    reg scl_rel, sda_rel;
    assign scl_t = scl_rel;
    assign sda_t = sda_rel;
    assign scl_o = 1'b0;
    assign sda_o = 1'b0;

    // synchronizer for SDA input
    reg sda_m, sda_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_m <= 1'b1;
            sda_s <= 1'b1;
        end else begin
            sda_m <= sda_i;
            sda_s <= sda_m;
        end
    end

    // phase tick generator
    wire        tick   = (cnt == 16'd0);
    wire [15:0] reload = (div_r > 16'd1) ? (div_r - 16'd1) : 16'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            ph        <= 2'd0;
            bit_cnt   <= 3'd7;
            shreg     <= 8'd0;
            rw_r      <= 1'b0;
            stop_r    <= 1'b0;
            nack_r    <= 1'b0;
            div_r     <= 16'd1;
            cnt       <= 16'd0;
            scl_rel   <= 1'b1;
            sda_rel   <= 1'b1;
            rx_data   <= 8'd0;
            busy      <= 1'b0;
            done      <= 1'b0;
            ack_error <= 1'b0;
        end else begin
            done <= 1'b0;

            // counter runs only while a transaction is active
            if (state != ST_IDLE)
                cnt <= tick ? reload : (cnt - 16'd1);

            case (state)
                // -------------------------------------------------
                ST_IDLE: begin
                    cnt <= 16'd0;
                    ph  <= 2'd0;
                    if (start_txn) begin
                        busy      <= 1'b1;
                        ack_error <= 1'b0;
                        shreg     <= tx_data;
                        rw_r      <= rw_bit;
                        stop_r    <= stop_bit;
                        nack_r    <= nack_on_read;
                        div_r     <= clk_div;
                        bit_cnt   <= 3'd7;
                        state     <= gen_start ? ST_START : ST_DATA;
                    end
                end

                // -------------------------------------------------
                ST_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                // -------------------------------------------------
                default: if (tick) begin
                    ph <= ph + 2'd1;

                    case (state)
                        // START: SDA=1 -> SCL=1 -> SDA=0 -> SCL=0
                        ST_START: begin
                            case (ph)
                                2'd0: sda_rel <= 1'b1;
                                2'd1: scl_rel <= 1'b1;
                                2'd2: sda_rel <= 1'b0;
                                2'd3: begin
                                    scl_rel <= 1'b0;
                                    state   <= ST_DATA;
                                end
                            endcase
                        end

                        // 8 data bits, MSB first
                        ST_DATA: begin
                            case (ph)
                                2'd0: sda_rel <= rw_r ? 1'b1 : shreg[7];
                                2'd1: scl_rel <= 1'b1;
                                2'd2: if (rw_r) rx_data <= {rx_data[6:0], sda_s};
                                2'd3: begin
                                    scl_rel <= 1'b0;
                                    shreg   <= {shreg[6:0], 1'b0};
                                    if (bit_cnt == 3'd0)
                                        state <= ST_ACK;
                                    else
                                        bit_cnt <= bit_cnt - 3'd1;
                                end
                            endcase
                        end

                        // 9th clock: ACK/NACK
                        ST_ACK: begin
                            case (ph)
                                // write: release SDA for Slave ACK
                                // read : Master drives ACK(0)/NACK(1)
                                2'd0: sda_rel <= rw_r ? nack_r : 1'b1;
                                2'd1: scl_rel <= 1'b1;
                                2'd2: if (!rw_r && sda_s) ack_error <= 1'b1;
                                2'd3: begin
                                    scl_rel <= 1'b0;
                                    state   <= stop_r ? ST_STOP : ST_DONE;
                                end
                            endcase
                        end

                        // STOP: SDA=0 -> SCL=1 -> SDA=1
                        ST_STOP: begin
                            case (ph)
                                2'd0: sda_rel <= 1'b0;
                                2'd1: scl_rel <= 1'b1;
                                2'd2: begin
                                    sda_rel <= 1'b1;
                                    ph      <= 2'd0;
                                    state   <= ST_DONE;
                                end
                                default: ;
                            endcase
                        end

                        default: state <= ST_IDLE;
                    endcase
                end
            endcase
        end
    end

endmodule