`timescale 1ns / 1ps

module my_IOBUF (
    input  wire io_in,
    input  wire io_t,
    output wire io_out,
    inout  wire io_io
);

    IOBUF iobuf_inst (
        .I  (io_in),
        .T  (io_t),
        .O  (io_out),
        .IO (io_io)
    );

endmodule