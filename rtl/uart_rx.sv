
// =====================================================================
// uart_rx — 8N1 UART receiver, sys_clk domain
//
//   Default: 115200 baud @ 100 MHz sys_clk ⇒ 868 clocks per bit.
//   Override via CLK_HZ / BAUD parameters.
//
//   Input `rx` is an asynchronous board pin (from the Arty USB-UART
//   bridge at pin A9); it is brought into sys_clk through a 2-FF
//   synchronizer before the state machine looks at it.
//
//   Protocol: idle-high, one LOW start bit, 8 data bits LSB-first,
//   one HIGH stop bit.  No parity.  Framing errors (missing stop bit)
//   silently drop the byte — byte_valid is only pulsed on a clean
//   stop-bit sample.
//
//   State machine:
//     IDLE   — wait for rx_sync to fall (start bit seen).
//     START  — count to mid-bit (CLKS_PER_BIT/2); re-sample rx_sync;
//              if still LOW → advance to DATA, else → back to IDLE.
//     DATA   — every CLKS_PER_BIT, shift rx_sync into shift[7:0];
//              after 8 bits advance to STOP.
//     STOP   — after CLKS_PER_BIT, sample rx_sync; if HIGH, pulse
//              byte_valid for one cycle with byte_data = shift.
// =====================================================================
module uart_rx #(
    parameter int CLK_HZ = 100_000_000,
    parameter int BAUD   =    115_200
)(
    input  logic       clk,
    input  logic       rst_n,
    input  logic       rx,              // async board pin
    output logic       byte_valid,
    output logic [7:0] byte_data
);

    localparam int CLKS_PER_BIT  = CLK_HZ / BAUD;
    localparam int CLKS_PER_HALF = CLKS_PER_BIT / 2;

    // ----------------------------------------------------------------
    // 2-FF input synchronizer (async → sys_clk)
    // ----------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic rx_meta;
    (* ASYNC_REG = "TRUE" *) logic rx_sync;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    // ----------------------------------------------------------------
    // State machine
    // ----------------------------------------------------------------
    typedef enum logic [1:0] {
        S_IDLE  = 2'd0,
        S_START = 2'd1,
        S_DATA  = 2'd2,
        S_STOP  = 2'd3
    } state_t;

    state_t             state;
    logic [15:0]        clk_cnt;    // up to ~65k clocks — ample for 115200@100 MHz
    logic [2:0]         bit_idx;
    logic [7:0]         shift;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            clk_cnt    <= '0;
            bit_idx    <= '0;
            shift      <= '0;
            byte_valid <= 1'b0;
            byte_data  <= '0;
        end else begin
            byte_valid <= 1'b0;

            unique case (state)
                S_IDLE: begin
                    clk_cnt <= '0;
                    bit_idx <= '0;
                    if (!rx_sync) state <= S_START;
                end

                S_START: begin
                    if (clk_cnt == CLKS_PER_HALF - 1) begin
                        clk_cnt <= '0;
                        if (!rx_sync) state <= S_DATA;  // confirmed start
                        else          state <= S_IDLE;  // glitch, bail
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= '0;
                        shift   <= {rx_sync, shift[7:1]};   // LSB-first
                        if (bit_idx == 3'd7) state <= S_STOP;
                        else                 bit_idx <= bit_idx + 3'd1;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                S_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= '0;
                        if (rx_sync) begin
                            byte_valid <= 1'b1;
                            byte_data  <= shift;
                        end
                        state <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end
            endcase
        end
    end

endmodule
