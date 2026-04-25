
// =====================================================================
// biquad_2nd_coef_ports — 2nd-order biquad with coefficient ports
//
//   Direct-form-I biquad:
//       y[n] = b0·x[n] + b1·x[n−1] + b2·x[n−2] − a1·y[n−1] − a2·y[n−2]
//
//   Coefficients (Q3.29 signed) arrive as input ports, NOT a .mem ROM.
//   Used by the tonestack to dispatch per-sample-pot-triple coefficients
//   from the lookup ROM into the biquad without rebuilding the .mem on
//   every UART pot change.
//
//   Datapath, pipeline, accumulator widths — identical to speaker_load.sv
//   (same b0/b1/b2/a1/a2 multiplies + 5-way 82-bit accumulate + 96-bit
//   round chain).  This module is a straight port-ified copy: only the
//   $readmemh ROM is replaced with five top-level ports.
// =====================================================================
module biquad_2nd_coef_ports
    import jcm800_pkg::*;
(
    input  logic                clk,
    input  logic                rst_n,
    input  logic signed [31:0]  b0_q29,
    input  logic signed [31:0]  b1_q29,
    input  logic signed [31:0]  b2_q29,
    input  logic signed [31:0]  a1_q29,
    input  logic signed [31:0]  a2_q29,
    input  sample_t             x_in,
    input  logic                x_valid,
    output sample_t             y_out,
    output logic                y_valid
);

    // ----------------------------------------------------------------
    // State
    // ----------------------------------------------------------------
    sample_t             x_prev1, x_prev2;
    logic signed [47:0]  y_prev1_q39, y_prev2_q39;

    // ----------------------------------------------------------------
    // S0 — five multiplies; latch x; register products
    // ----------------------------------------------------------------
    logic signed [55:0]  prod_b0_c, prod_b1_c, prod_b2_c;
    logic signed [79:0]  prod_a1_c, prod_a2_c;

    logic signed [55:0]  s0_prod_b0, s0_prod_b1, s0_prod_b2;
    logic signed [79:0]  s0_prod_a1, s0_prod_a2;
    logic                s0_valid;

    always_comb begin
        prod_b0_c = b0_q29 * x_in;
        prod_b1_c = b1_q29 * x_prev1;
        prod_b2_c = b2_q29 * x_prev2;
        prod_a1_c = a1_q29 * y_prev1_q39;
        prod_a2_c = a2_q29 * y_prev2_q39;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_prod_b0 <= '0;
            s0_prod_b1 <= '0;
            s0_prod_b2 <= '0;
            s0_prod_a1 <= '0;
            s0_prod_a2 <= '0;
            s0_valid   <= 1'b0;
        end else begin
            s0_valid <= x_valid;
            if (x_valid) begin
                s0_prod_b0 <= prod_b0_c;
                s0_prod_b1 <= prod_b1_c;
                s0_prod_b2 <= prod_b2_c;
                s0_prod_a1 <= prod_a1_c;
                s0_prod_a2 <= prod_a2_c;
            end
        end
    end

    // ----------------------------------------------------------------
    // x-latch chain (same as speaker_load.sv).
    // ----------------------------------------------------------------
    sample_t s0_x_lat, s1_x_lat, s2_x_lat;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_x_lat <= '0;
            s1_x_lat <= '0;
        end else begin
            if (x_valid)  s0_x_lat <= x_in;
            if (s0_valid) s1_x_lat <= s0_x_lat;
        end
    end

    // ----------------------------------------------------------------
    // S1 — align b-products to Q12.68 and 5-way accumulate.
    // ----------------------------------------------------------------
    logic signed [79:0]  prod_b0_aligned, prod_b1_aligned, prod_b2_aligned;
    logic signed [81:0]  acc_q68_c;

    logic signed [81:0]  s1_acc_q68;
    logic                s1_valid;

    always_comb begin
        prod_b0_aligned = {{8{s0_prod_b0[55]}}, s0_prod_b0, 16'b0};
        prod_b1_aligned = {{8{s0_prod_b1[55]}}, s0_prod_b1, 16'b0};
        prod_b2_aligned = {{8{s0_prod_b2[55]}}, s0_prod_b2, 16'b0};
        acc_q68_c       = { {2{prod_b0_aligned[79]}}, prod_b0_aligned }
                        + { {2{prod_b1_aligned[79]}}, prod_b1_aligned }
                        + { {2{prod_b2_aligned[79]}}, prod_b2_aligned }
                        - { {2{s0_prod_a1[79]}},      s0_prod_a1 }
                        - { {2{s0_prod_a2[79]}},      s0_prod_a2 };
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_acc_q68 <= '0;
            s1_valid   <= 1'b0;
        end else begin
            s1_valid <= s0_valid;
            if (s0_valid) s1_acc_q68 <= acc_q68_c;
        end
    end

    // ----------------------------------------------------------------
    // S2 — round acc_q68 >>> 29 → Q9.39
    // ----------------------------------------------------------------
    logic signed [95:0]  acc_q68_ext;
    logic signed [95:0]  y_q39_96_c;
    logic signed [47:0]  y_q39_next_c;

    logic signed [47:0]  s2_y_q39;
    logic                s2_valid;

    always_comb begin
        acc_q68_ext  = {{14{s1_acc_q68[81]}}, s1_acc_q68};
        y_q39_96_c   = round_conv96(acc_q68_ext, 29);
        y_q39_next_c = y_q39_96_c[47:0];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_y_q39 <= '0;
            s2_x_lat <= '0;
            s2_valid <= 1'b0;
        end else begin
            s2_valid <= s1_valid;
            if (s1_valid) begin
                s2_y_q39 <= y_q39_next_c;
                s2_x_lat <= s1_x_lat;
            end
        end
    end

    // ----------------------------------------------------------------
    // S3 — narrow → Q1.23 + state commit
    // ----------------------------------------------------------------
    logic signed [95:0]  y_q39_ext;
    logic signed [95:0]  y_q23_96_c;
    logic signed [95:0]  y_q23_96_q;
    logic                s3_valid;

    always_comb begin
        y_q39_ext  = {{48{s2_y_q39[47]}}, s2_y_q39};
        y_q23_96_c = round_conv96(y_q39_ext, 16);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_prev1     <= '0;
            x_prev2     <= '0;
            y_prev1_q39 <= '0;
            y_prev2_q39 <= '0;
            y_q23_96_q  <= '0;
            s3_valid    <= 1'b0;
        end else begin
            s3_valid <= s2_valid;
            if (s2_valid) begin
                y_q23_96_q  <= y_q23_96_c;
                x_prev1     <= s2_x_lat;
                x_prev2     <= x_prev1;
                y_prev1_q39 <= s2_y_q39;
                y_prev2_q39 <= y_prev1_q39;
            end
        end
    end

    // ----------------------------------------------------------------
    // S4 — saturate
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out   <= '0;
            y_valid <= 1'b0;
        end else begin
            y_valid <= s3_valid;
            if (s3_valid) y_out <= saturate_s24(y_q23_96_q);
        end
    end

endmodule
