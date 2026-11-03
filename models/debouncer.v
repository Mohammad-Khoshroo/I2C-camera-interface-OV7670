module button_debouncer #(
    parameter DEBOUNCE_CYCLES = 200_000 
) (
    input  wire clk,
    input  wire rst_n,        // active-low async reset
    input  wire btn_in,       // raw, noisy button input
    output reg  btn_out       // debounced, clean output (level)
);

    // --- 2-FF synchronizer to avoid metastability ---
    reg btn_sync_0, btn_sync_1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_sync_0 <= 1'b0;
            btn_sync_1 <= 1'b0;
        end else begin
            btn_sync_0 <= btn_in;
            btn_sync_1 <= btn_sync_0;
        end
    end

    // --- Counter-based debounce ---
    localparam CNT_WIDTH = $clog2(DEBOUNCE_CYCLES);
    reg [CNT_WIDTH-1:0] count;
    reg btn_stable;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count      <= 0;
            btn_stable <= 1'b0;
            btn_out    <= 1'b0;
        end else begin
            if (btn_sync_1 != btn_stable) begin
                // input differs from accepted state -> count
                count <= count + 1'b1;
                if (count == DEBOUNCE_CYCLES - 1) begin
                    btn_stable <= btn_sync_1;
                    btn_out    <= btn_sync_1;
                    count      <= 0;
                end
            end else begin
                // matches current stable value, reset counter
                count <= 0;
            end
        end
    end

endmodule