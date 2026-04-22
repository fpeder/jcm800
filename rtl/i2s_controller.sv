
// =====================================================================
// i2s_controller — Pmod I2S2 interface (based on Digilent axis_i2s2)
//
//   12.288 MHz MCLK, 48 kHz LRCK, 3.072 MHz SCLK
//   MCLK/SCLK = 4, SCLK/LRCK = 64, 32 bits per channel
//   24-bit audio, MSB-first, 1-SCLK delay after LRCK edge
//
//   Clocks are combinational from a free-running counter
//   (matches Digilent axis_i2s2 approach).
// =====================================================================
module i2s_controller
    import jcm800_pkg::*;
(
    input  logic             clk,        // 100 MHz system clock
    input  logic             rst_n,
    input  logic             mclk,       // 12.288 MHz master clock

    // I2S pins (directly to Pmod)
    output logic             lrck,       // Left/right clock (48 kHz)
    output logic             sclk,       // Bit clock (3.072 MHz)
    output logic             sdout,      // DAC serial data
    input  logic             sdin,       // ADC serial data
    output logic             mclk_out,   // MCLK pass-through

    // Audio sample interface (sys_clk domain)
    output logic             sample_valid,
    output sample_t          sample_out,  // RX: from ADC
    input  logic             out_ready,   // TX: data available
    input  sample_t          sample_in    // TX: to DAC
);

    assign mclk_out = mclk;

    // ---------------------------------------------------------------
    // Free-running 8-bit counter (256 MCLKs per frame)
    //   count[7]   = LRCK  (0 = left, 1 = right)
    //   count[6:2] = bit position (0-31 per channel)
    //   count[1:0] = sub-SCLK phase
    // ---------------------------------------------------------------
    logic [7:0] count;

    always_ff @(posedge mclk or negedge rst_n)
        if (!rst_n)
            count <= 8'd0;
        else
            count <= count + 8'd1;

    assign lrck = count[7];
    assign sclk = count[1];

    logic [4:0] bit_pos;
    assign bit_pos = count[6:2];

    // ---------------------------------------------------------------
    // SDIN synchronizer (3-stage, matches Digilent)
    // ---------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic [2:0] din_sync;
    logic       din;
    assign din = din_sync[2];

    always_ff @(posedge mclk or negedge rst_n)
        if (!rst_n)
            din_sync <= 3'd0;
        else
            din_sync <= {din_sync[1:0], sdin};

    // ---------------------------------------------------------------
    // RX: capture SDIN into shift register (left channel only)
    //
    //   SCLK = count[1]:
    //     LOW  when count[1:0] = 00, 01
    //     HIGH when count[1:0] = 10, 11
    //   SCLK falls at transition 11→00.
    //   ADC shifts SDOUT on SCLK fall.
    //   3-stage sync settles 3 cycles later at count[1:0] = 11.
    //   Sample din at count[1:0] == 2'b11.
    // ---------------------------------------------------------------
    logic [23:0] rx_data_l_shift;
    logic [23:0] rx_sample;
    logic        rx_done_toggle;

    always_ff @(posedge mclk or negedge rst_n) begin
        if (!rst_n) begin
            rx_data_l_shift <= 24'd0;
            rx_sample       <= 24'd0;
            rx_done_toggle  <= 1'b0;
        end else if (count[1:0] == 2'b11 && bit_pos >= 5'd1 && bit_pos <= 5'd24) begin
            if (~count[7])
                rx_data_l_shift <= {rx_data_l_shift[22:0], din};
        end else if (count == 8'd0) begin
            rx_sample      <= rx_data_l_shift;
            rx_done_toggle <= ~rx_done_toggle;
        end
    end

    // ---------------------------------------------------------------
    // TX: shift out MSB-first
    //
    //   Load shift register at start of each half-frame.
    //   Shift on count[1:0] == 2'b11 (just before SCLK falls).
    //   SDOUT is combinational from shift register MSB.
    // ---------------------------------------------------------------
    logic [23:0] tx_data_latched;
    logic [23:0] tx_shift;

    always_ff @(posedge mclk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift <= 24'd0;
        // Load at start of both L (count=3) and R (count=131) half-frames,
        // mirroring mono tx_data_latched to both stereo channels.
        end else if (count[6:0] == 7'd3) begin
            tx_shift <= tx_data_latched;
        end else if (count[1:0] == 2'b11 && bit_pos >= 5'd1 && bit_pos <= 5'd24) begin
            tx_shift <= {tx_shift[22:0], 1'b0};
        end
    end

    always_comb begin
        if (bit_pos >= 5'd1 && bit_pos <= 5'd24)
            sdout = tx_shift[23];
        else
            sdout = 1'b0;
    end

    // ---------------------------------------------------------------
    // CDC: MCLK → sys_clk (RX toggle synchronizer)
    // ---------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic rx_toggle_sync0, rx_toggle_sync1, rx_toggle_sync2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_toggle_sync0 <= 1'b0;
            rx_toggle_sync1 <= 1'b0;
            rx_toggle_sync2 <= 1'b0;
        end else begin
            rx_toggle_sync0 <= rx_done_toggle;
            rx_toggle_sync1 <= rx_toggle_sync0;
            rx_toggle_sync2 <= rx_toggle_sync1;
        end
    end

    logic rx_new;
    assign rx_new = rx_toggle_sync1 ^ rx_toggle_sync2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_valid <= 1'b0;
            sample_out   <= '0;
        end else begin
            sample_valid <= rx_new;
            if (rx_new)
                sample_out <= sample_t'(rx_sample);
        end
    end

    // ---------------------------------------------------------------
    // CDC: sys_clk → MCLK (TX toggle synchronizer)
    // ---------------------------------------------------------------
    logic        tx_ready_toggle;
    logic [23:0] tx_data_sys;
    (* ASYNC_REG = "TRUE" *) logic tx_toggle_sync0, tx_toggle_sync1, tx_toggle_sync2;

    always_ff @(posedge mclk or negedge rst_n) begin
        if (!rst_n) begin
            tx_toggle_sync0 <= 1'b0;
            tx_toggle_sync1 <= 1'b0;
            tx_toggle_sync2 <= 1'b0;
        end else begin
            tx_toggle_sync0 <= tx_ready_toggle;
            tx_toggle_sync1 <= tx_toggle_sync0;
            tx_toggle_sync2 <= tx_toggle_sync1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_ready_toggle <= 1'b0;
            tx_data_sys     <= 24'd0;
        end else if (out_ready) begin
            tx_data_sys     <= sample_in;
            tx_ready_toggle <= ~tx_ready_toggle;
        end
    end

    always_ff @(posedge mclk or negedge rst_n) begin
        if (!rst_n)
            tx_data_latched <= 24'd0;
        else if (tx_toggle_sync1 ^ tx_toggle_sync2)
            tx_data_latched <= tx_data_sys;
    end

endmodule

