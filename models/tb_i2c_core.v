`timescale 1ns / 1ps

module tb_i2c_core;

    // ---------------- clock / reset ----------------
    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz

    // ---------------- DUT signals ----------------
    reg         start_txn    = 1'b0;
    reg         gen_start    = 1'b0;
    reg         stop_bit     = 1'b0;
    reg         rw_bit       = 1'b0;
    reg         nack_on_read = 1'b1;
    reg  [7:0]  tx_data      = 8'h00;
    reg  [15:0] clk_div      = 16'd250;         // SCL = 100 kHz

    wire [7:0]  rx_data;
    wire        busy, ack_error, done;
    wire        scl_o, scl_t, sda_o, sda_t;

    // ---------------- bus (pull-ups via tri1) ----------------
    tri1 scl, sda;
    assign scl = scl_t ? 1'bz : scl_o;
    assign sda = sda_t ? 1'bz : sda_o;
    wire scl_i = scl;
    wire sda_i = sda;

    i2c_master_core dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .start_txn    (start_txn),
        .gen_start    (gen_start),
        .stop_bit     (stop_bit),
        .rw_bit       (rw_bit),
        .nack_on_read (nack_on_read),
        .tx_data      (tx_data),
        .clk_div      (clk_div),
        .rx_data      (rx_data),
        .busy         (busy),
        .ack_error    (ack_error),
        .done         (done),
        .scl_o        (scl_o),
        .scl_t        (scl_t),
        .scl_i        (scl_i),
        .sda_o        (sda_o),
        .sda_t        (sda_t),
        .sda_i        (sda_i)
    );

    // ---------------- provided Slave (addr 0x50) ----------------
    wire         slv_err_any;
    wire [7:0]   slv_reg0, slv_reg1;
    wire [63:0]  slv_state;
    wire [31:0]  slv_err_count;

    i2c_slave_golden #(.SLAVE_ADDR(7'h50)) slave (
        .scl         (scl),
        .sda         (sda),
        .reg0        (slv_reg0),
        .reg1        (slv_reg1),
        .state_ascii (slv_state),
        .err_any     (slv_err_any),
        .err_count   (slv_err_count)
    );

    // ---------------- helper tasks ----------------
    integer errors      = 0;
    integer unexp_nack  = 0;
    reg     expect_nack = 1'b0;
    reg     last_ack_err;

    // send ONE byte through the core and wait for done
    task send_byte;
        input [7:0] data;
        input       gs;
        input       st;
        input       rw;
        input       nack;
        begin
            @(negedge clk);
            tx_data      = data;
            gen_start    = gs;
            stop_bit     = st;
            rw_bit       = rw;
            nack_on_read = nack;
            start_txn    = 1'b1;
            @(negedge clk);
            start_txn    = 1'b0;
            @(posedge done);
            @(negedge clk);
            last_ack_err = ack_error;
            if (ack_error && !expect_nack) begin
                unexp_nack = unexp_nack + 1;
                $display("[%0t] TB FAIL: unexpected NACK on byte 0x%02h", $time, data);
            end
        end
    endtask

    // S | ADDR+W | A | PTR | A | DATA | A | P
    task i2c_write_reg;
        input [6:0] addr7;
        input [7:0] ptr;
        input [7:0] data;
        begin
            send_byte({addr7, 1'b0}, 1'b1, 1'b0, 1'b0, 1'b1);
            send_byte(ptr,           1'b0, 1'b0, 1'b0, 1'b1);
            send_byte(data,          1'b0, 1'b1, 1'b0, 1'b1);
        end
    endtask

    // S | ADDR+W | A | PTR | A | Sr | ADDR+R | A | DATA | NA | P
    task i2c_read_reg;
        input  [6:0] addr7;
        input  [7:0] ptr;
        output [7:0] data;
        begin
            send_byte({addr7, 1'b0}, 1'b1, 1'b0, 1'b0, 1'b1);
            send_byte(ptr,           1'b0, 1'b0, 1'b0, 1'b1);
            send_byte({addr7, 1'b1}, 1'b1, 1'b0, 1'b0, 1'b1);   // repeated START
            send_byte(8'h00,         1'b0, 1'b1, 1'b1, 1'b1);   // read, NACK, STOP
            data = rx_data;
        end
    endtask

    task check_eq;
        input [8*28-1:0] msg;
        input [7:0]      got;
        input [7:0]      expv;
        begin
            if (got !== expv) begin
                errors = errors + 1;
                $display("[%0t] TB FAIL: %0s  got=0x%02h  expected=0x%02h", $time, msg, got, expv);
            end else
                $display("[%0t] TB PASS: %0s  = 0x%02h", $time, msg, got);
        end
        end
    endtask

    // ---------------- test sequence ----------------
    reg [7:0] rd;

    initial begin
        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);

        // 1) initial values of both registers
        i2c_read_reg(7'h50, 8'h00, rd);  check_eq("read reg0 (initial)", rd, 8'hA5);
        i2c_read_reg(7'h50, 8'h01, rd);  check_eq("read reg1 (initial)", rd, 8'h3C);

        // 2) write an arbitrary value, then read it back
        i2c_write_reg(7'h50, 8'h00, 8'hC3);
        i2c_read_reg (7'h50, 8'h00, rd);  check_eq("readback reg0",    rd,       8'hC3);
        check_eq("slave reg0 content", slv_reg0, 8'hC3);

        i2c_write_reg(7'h50, 8'h01, 8'h5A);
        i2c_read_reg (7'h50, 8'h01, rd);  check_eq("readback reg1",    rd,       8'h5A);
        check_eq("slave reg1 content", slv_reg1, 8'h5A);

        // reg0 must not have been disturbed by the write to reg1
        i2c_read_reg (7'h50, 8'h00, rd);  check_eq("reg0 still intact", rd, 8'hC3);

        // 3) wrong address (0x51 write): Slave must not answer -> ack_error = 1
        expect_nack = 1'b1;
        send_byte({7'h51, 1'b0}, 1'b1, 1'b1, 1'b0, 1'b1);   // S, addr, (no ACK), P
        expect_nack = 1'b0;
        if (last_ack_err !== 1'b1) begin
            errors = errors + 1;
            $display("[%0t] TB FAIL: ack_error was NOT set for a wrong address", $time);
        end else
            $display("[%0t] TB PASS: ack_error set for wrong address", $time);

        // 4) bus must still work after the failed access
        i2c_read_reg(7'h50, 8'h01, rd);  check_eq("read reg1 after NACK test", rd, 8'h5A);

        // ---------------- summary ----------------
        repeat (50) @(posedge clk);
        if (slv_err_any || slv_err_count != 0) begin
            errors = errors + 1;
            $display("[%0t] TB FAIL: the Slave model reported %0d protocol problem(s)", $time, slv_err_count);
        end
        if (errors == 0 && unexp_nack == 0)
            $display("[%0t] TB RESULT: ALL TESTS PASSED", $time);
        else
            $display("[%0t] TB RESULT: FAILED  (%0d check(s), %0d unexpected NACK)", $time, errors, unexp_nack);
        $finish;
    end
    end

    // watchdog
    initial begin
        #50_000_000;
        $display("TB FAIL: timeout");
        $finish;
    end

endmodule