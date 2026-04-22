
// =====================================================================
// biquad_coef_ports — first-order biquad, coefficients via input ports
//
//   Datapath is identical to cathode_shelf.sv; the only difference is
//   that b0/b1/a1 arrive as 32-bit Q3.29 signed input ports instead of
//   being loaded from a .mem at elaboration time.  This lets the
//   tonestack wire up three of these in series behind a single pot-
//   indexed coefficient ROM.
//
//   y[n] = b0·x[n] + b1·x[n−1] − a1·y[n−1]
//
//   See cathode_shelf.sv for the full derivation of the arithmetic
//   widths, rounding rule, and four-stage pipeline (S0..S3).  This
//   module is a straight port-ified copy.
// =====================================================================
module biquad_coef_ports
    import jcm800_pkg::*;
(
    input  logic                clk,
    input  logic                rst_n,
    input  logic signed [31:0]  b0_q29,
    input  logic signed [31:0]  b1_q29,
    input  logic signed [31:0]  a1_q29,
    input  sample_t             x_in,
    input  logic                x_valid,
    output sample_t             y_out,
    output logic                y_valid
);

    // ----------------------------------------------------------------
    // State
    // ----------------------------------------------------------------
    sample_t             x_prev;
    logic signed [47:0]  y_prev_q39;

    // ----------------------------------------------------------------
    // S0 — three multiplies; latch x; register products
    // ----------------------------------------------------------------
    logic signed [55:0]  prod_b0_c, prod_b1_c;
    logic signed [79:0]  prod_a1_c;

    logic signed [55:0]  s0_prod_b0, s0_prod_b1;
    logic signed [79:0]  s0_prod_a1;
    sample_t             s0_x_lat;
    logic                s0_valid;

    always_comb begin
        prod_b0_c = b0_q29 * x_in;
        prod_b1_c = b1_q29 * x_prev;
        prod_a1_c = a1_q29 * y_prev_q39;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_prod_b0 <= '0;
            s0_prod_b1 <= '0;
            s0_prod_a1 <= '0;
            s0_x_lat   <= '0;
            s0_valid   <= 1'b0;
        end else begin
            s0_valid <= x_valid;
            if (x_valid) begin
                s0_prod_b0 <= prod_b0_c;
                s0_prod_b1 <= prod_b1_c;
                s0_prod_a1 <= prod_a1_c;
                s0_x_lat   <= x_in;
            end
        end
    end

    // ----------------------------------------------------------------
    // S1 — align b-products to Q12.68, 3-way accumulate → acc_q68 (80b)
    // ----------------------------------------------------------------
    logic signed [79:0]  prod_b0_aligned, prod_b1_aligned;
    logic signed [79:0]  acc_q68_c;

    logic signed [79:0]  s1_acc_q68;
    sample_t             s1_x_lat;
    logic                s1_valid;

    always_comb begin
        prod_b0_aligned = {{8{s0_prod_b0[55]}}, s0_prod_b0, 16'b0};
        prod_b1_aligned = {{8{s0_prod_b1[55]}}, s0_prod_b1, 16'b0};
        acc_q68_c       = prod_b0_aligned + prod_b1_aligned - s0_prod_a1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_acc_q68 <= '0;
            s1_x_lat   <= '0;
            s1_valid   <= 1'b0;
        end else begin
            s1_valid <= s0_valid;
            if (s0_valid) begin
                s1_acc_q68 <= acc_q68_c;
                s1_x_lat   <= s0_x_lat;
            end
        end
    end

    // ----------------------------------------------------------------
    // S2 — round acc_q68 >>> 29 → Q9.39 y_q39_next
    // ----------------------------------------------------------------
    logic signed [95:0]  acc_q68_ext;
    logic signed [95:0]  y_q39_96_c;
    logic signed [47:0]  y_q39_next_c;

    logic signed [47:0]  s2_y_q39;
    sample_t             s2_x_lat;
    logic                s2_valid;

    always_comb begin
        acc_q68_ext  = {{16{s1_acc_q68[79]}}, s1_acc_q68};
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
    // S3a — narrow Q9.39 → Q1.23 via round >>> 16; commit state.
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
            x_prev     <= '0;
            y_prev_q39 <= '0;
            y_q23_96_q <= '0;
            s3_valid   <= 1'b0;
        end else begin
            s3_valid <= s2_valid;
            if (s2_valid) begin
                y_q23_96_q <= y_q23_96_c;
                x_prev     <= s2_x_lat;
                y_prev_q39 <= s2_y_q39;
            end
        end
    end

    // ----------------------------------------------------------------
    // S3b — saturate rounded value to sample_t
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
