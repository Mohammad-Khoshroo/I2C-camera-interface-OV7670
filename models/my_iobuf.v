`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/23/2026 11:49:43 PM
// Design Name: 
// Module Name: my_IOBUF
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module my_IOBUF(
    input  io_in, io_t,
    inout  io_io,
    output io_out
);
    
    
IOBUF #(
    .DRIVE      (12),
    .IBUF_LOW_PWR ("TRUE"),
    .IOSTANDARD ("LVCMOS33"),
    .SLEW       ("SLOW")
) iobuf_inst (
    .O  (io_out),      
    .IO (io_io),  
    .I  (io_in),     
    .T  (io_t)       
);
endmodule
