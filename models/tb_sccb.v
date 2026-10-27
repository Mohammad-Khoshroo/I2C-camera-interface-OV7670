`timescale 1ns / 1ps

//---------------------------------------------------------------------
// One complete system: core + SCCB FSM + camera model on its own bus
//---------------------------------------------------------------------
module sccb_env #(
    parameter [7:0] CAM_PID = 8'h76,
    parameter [7:0] CAM_VER = 8'h73,
    parameter [6:0] CAM_ID  = 7'h21          // 0x42 write / 0x43 read
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    output wire       id_match,
    output wire       id_mismatch,
    output wire       ack_fail,
    output wire       busy,
    output wire       finished,
    output wire [7:0] pid_val,
    output wire [7:0] ver_val,
    output wire       cam_err_any,
    output wire [31:0] cam_start_cnt,
    output wire [31:0] cam_stop_cnt,
    output wire [31:0] cam_rd_cnt
);
    // bus with pull-ups
    tri1 scl, sda;
    wire scl_o, scl_t, sda_o, sda_t;
    assign scl = scl_t ? 1'bz : scl_o;
    assign sda = sda_t ? 1'bz : sda_o;
    wire scl_i = scl;
    wire sda_i = sda;

    // FSM <-> core
    wire        start_txn, gen_start, stop_bit, rw_bit, nack_on_read;
    wire [7:0]  tx_data, rx_data;
    wire [15:0] clk_div;
    wire        core_busy, core_ack_error, core_done;

    i2c_master_core core (
        .clk(clk), .rst_n(rst_n),
        .start_txn(start_txn), .gen_start(gen_start), .stop_bit(stop_bit),
        .rw_bit(rw_bit), .nack_on_read(nack_on_read),
        .tx_data(tx_data), .clk_div(clk_div),
        .rx_data(rx_data), .busy(core_busy),
        .ack_error(core_ack_error), .done(core_done),
        .scl_o(scl_o), .scl_t(scl_t), .scl_i(scl_i),
        .sda_o(sda_o), .sda_t(sda_t), .sda_i(sda_i)
    );

    sccb_id_reader reader (
        .clk(clk), .rst_n(rst_n), .start(start),
        .start_txn(start_txn), .gen_start(gen_start), .stop_bit(stop_bit),
        .rw_bit(rw_bit), .nack_on_read(nack_on_read),
        .tx_data(tx_data), .clk_div(clk_div),
        .rx_data(rx_data), .done(core_done), .ack_error(core_ack_error),
        .id_match(id_match), .busy(busy), .finished(finished),
        .ack_fail(ack_fail), .id_mismatch(id_mismatch),
        .pid_val(pid_val), .ver_val(ver_val)
    );

    wire [63:0] cam_state;
    ov7670_sccb_golden #(
        .SCCB_ID   (CAM_ID),
        .PID_VALUE (CAM_PID),
        .VER_VALUE (CAM_VER)
    ) cam (
        .sio_c       (scl),
        .sio_d       (sda),
        .state_ascii (cam_state),
        .start_cnt   (cam_start_cnt),
        .stop_cnt    (cam_stop_cnt),
        .rd_cnt      (cam_rd_cnt),
        .err_any     (cam_err_any)
    );
endmodule


//---------------------------------------------------------------------
// Testbench
//---------------------------------------------------------------------
module tb_sccb;

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    reg start = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz

    // ---- 1) correct camera ----
    wire ok_match, ok_mism, ok_nack, ok_busy, ok_fin, ok_err;
    wire [7:0] ok_pid, ok_ver;
    wire [31:0] ok_starts, ok_stops, ok_reads;
    sccb_env #(.CAM_PID(8'h76), .CAM_VER(8'h73)) env_ok (
        .clk(clk), .rst_n(rst_n), .start(start),
        .id_match(ok_match), .id_mismatch(ok_mism), .ack_fail(ok_nack),
        .busy(ok_busy), .finished(ok_fin), .pid_val(ok_pid), .ver_val(ok_ver),
        .cam_err_any(ok_err), .cam_start_cnt(ok_starts),
        .cam_stop_cnt(ok_stops), .cam_rd_cnt(ok_reads));

    // ---- 2) wrong PID ----
    wire bp_match, bp_mism, bp_nack, bp_busy, bp_fin, bp_err;
    wire [7:0] bp_pid, bp_ver;
    sccb_env #(.CAM_PID(8'h77), .CAM_VER(8'h73)) env_bad_pid (
        .clk(clk), .rst_n(rst_n), .start(start),
        .id_match(bp_match), .id_mismatch(bp_mism), .ack_fail(bp_nack),
        .busy(bp_busy), .finished(bp_fin), .pid_val(bp_pid), .ver_val(bp_ver),
        .cam_err_any(bp_err), .cam_start_cnt(), .cam_stop_cnt(), .cam_rd_cnt());

    // ---- 3) wrong VER ----
    wire bv_match, bv_mism, bv_nack, bv_busy, bv_fin, bv_err;
    wire [7:0] bv_pid, bv_ver;
    sccb_env #(.CAM_PID(8'h76), .CAM_VER(8'h72)) env_bad_ver (
        .clk(clk), .rst_n(rst_n), .start(start),
        .id_match(bv_match), .id_mismatch(bv_mism), .ack_fail(bv_nack),
        .busy(bv_busy), .finished(bv_fin), .pid_val(bv_pid), .ver_val(bv_ver),
        .cam_err_any(bv_err), .cam_start_cnt(), .cam_stop_cnt(), .cam_rd_cnt());

    // ---- 4) camera that does not answer (different SCCB ID) ----
    wire nc_match, nc_mism, nc_nack, nc_busy, nc_fin, nc_err;
    wire [7:0] nc_pid, nc_ver;
    sccb_env #(.CAM_ID(7'h30)) env_no_cam (
        .clk(clk), .rst_n(rst_n), .start(start),
        .id_match(nc_match), .id_mismatch(nc_mism), .ack_fail(nc_nack),
        .busy(nc_busy), .finished(nc_fin), .pid_val(nc_pid), .ver_val(nc_ver),
        .cam_err_any(nc_err), .cam_start_cnt(), .cam_stop_cnt(), .cam_rd_cnt());

    // ---------------- checking helpers ----------------
    integer errors = 0;

    task check1;
        input [8*40-1:0] msg;
        input            got;
        input            expv;
        begin
            if (got !== expv) begin
                errors = errors + 1;
                $display("[%0t] TB FAIL: %0s  got=%0b  expected=%0b", $time, msg, got, expv);
            end else
                $display("[%0t] TB PASS: %0s = %0b", $time, msg, got);
        end
        end
    endtask

    task check32;
        input [8*40-1:0] msg;
        input [31:0]     got;
        input [31:0]     expv;
        begin
            if (got !== expv) begin
                errors = errors + 1;
                $display("[%0t] TB FAIL: %0s  got=%0d  expected=%0d", $time, msg, got, expv);
            end else
                $display("[%0t] TB PASS: %0s = %0d", $time, msg, got);
        end
        end
    endtask

    task check_results;
        begin
            check1("correct cam : id_match",    ok_match, 1'b1);
            check1("correct cam : id_mismatch", ok_mism,  1'b0);
            check1("correct cam : ack_fail",    ok_nack,  1'b0);
            check1("wrong PID   : id_match",    bp_match, 1'b0);
            check1("wrong PID   : id_mismatch", bp_mism,  1'b1);
            check1("wrong VER   : id_match",    bv_match, 1'b0);
            check1("wrong VER   : id_mismatch", bv_mism,  1'b1);
            check1("no camera   : id_match",    nc_match, 1'b0);
            check1("no camera   : ack_fail",    nc_nack,  1'b1);
            check1("camera model flagged no bus violation (ok)",   ok_err, 1'b0);
            check1("camera model flagged no bus violation (PID)",  bp_err, 1'b0);
            check1("camera model flagged no bus violation (VER)",  bv_err, 1'b0);
            check1("camera model flagged no bus violation (none)", nc_err, 1'b0);
        end
    endtask

    // ---------------- stimulus ----------------
    initial begin
        rst_n = 1'b0;
        start = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);

        // ---- run 1: start is held for a very long time ----
        @(negedge clk) start = 1'b1;
        @(posedge ok_fin);
        repeat (10) @(posedge clk);
        check_results;
        $display("[%0t] TB INFO: PID read = 0x%02h, VER read = 0x%02h", $time, ok_pid, ok_ver);

        // keep the button pressed: must NOT start a second run
        #500_000;
        check32("one run per press: camera START count", ok_starts, 32'd4);
        check32("one run per press: camera STOP count",  ok_stops,  32'd4);
        check32("one run per press: registers read",     ok_reads,  32'd2);
        check1 ("not busy while button is still held",   ok_busy,   1'b0);

        // ---- release, then a short second press ----
        @(negedge clk) start = 1'b0;
        #100_000;
        @(negedge clk) start = 1'b1;
        #2_000;
        @(negedge clk) start = 1'b0;
        @(posedge ok_fin);
        repeat (10) @(posedge clk);
        check_results;
        check32("second press: camera START count", ok_starts, 32'd8);
        check32("second press: registers read",     ok_reads,  32'd4);

        #20_000;
        if (errors == 0) $display("[%0t] TB RESULT: ALL TESTS PASSED", $time);
        else             $display("[%0t] TB RESULT: FAILED (%0d check(s))", $time, errors);
        $finish;
    end

    // watchdog
    initial begin
        #50_000_000;
        $display("TB FAIL: timeout");
        $finish;
    end

endmodule