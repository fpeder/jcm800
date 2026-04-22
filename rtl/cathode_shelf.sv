
// =====================================================================
// cathode_shelf — first-order shelving filter in biquad form
//
//   y[n] = b0·x[n] + b1·x[n−1] − a1·y[n−1]
//
//   (b2 = a2 = 0 — first-order, but the biquad structure is kept so a
//   future upgrade to a true 2nd-order network needs no pipeline surgery.)
//
//   Coefficient file layout
//   ───────────────────────
//     3 entries × 32-bit Q3.29 hex at addresses 0, 1, 2:
//         addr 0: b0   (Q3.29)
//         addr 1: b1   (Q3.29)
//         addr 2: a1   (Q3.29)
//     Q3.29 range is ±4 — needed because bilinear-transform coefficients
//     of the V1A high-shelf exceed ±2 at 48 kHz.  a1 itself is ∈ (−1,+1)
//     but shares the wider format.
//
//   Datapath arithmetic
//   ───────────────────
//     x  : Q1.23 s24               x_prev : Q1.23 s24
//     y_prev_q39 : Q9.39 s48 (wide state — matches HPF/LPF)
//
//     prod_b0 = b0·x       : Q4.52 s56
//     prod_b1 = b1·x_prev  : Q4.52 s56
//     prod_a1 = a1·y_prev  : Q12.68 s80
//
//     Align b0/b1 products to Q12.68 by sign-extending + left-shift 16:
//         prod_b_aligned = {{8{prod_b[55]}}, prod_b, 16'b0}  // 80-bit Q12.68
//
//     acc_q68  = prod_b0_aligned + prod_b1_aligned − prod_a1   // Q12.68 s80
//     Convergent-round >>> 29 → Q9.39 (48-bit signed state width)
//     y_q39_next updates the state register.
//     Narrow to Q1.23 via convergent-round >>> 16 and saturate for y_out.
//
//   Pipeline (4 register stages — the 80-bit 3-way accumulate + 96-bit
//   round + narrow + saturate chain does not fit in two cycles; split
//   the accumulate and the first round across S1/S2):
//     S0 — on x_valid: compute three products, latch x, register products.
//     S1 — align b-products to Q12.68 and accumulate → acc_q68 (80b).
//          Register s1_acc_q68.
//     S2 — convergent-round s1_acc_q68 >>> 29 → Q9.39.  Register s2_y_q39.
//     S3 — narrow Q9.39 → Q1.23 via round >>> 16, saturate s24; update
//          state (x_prev, y_prev_q39).
// =====================================================================
module cathode_shelf
    import jcm800_pkg::*;
#(
    parameter string COEFF_FILE = ""
)(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t x_in,
    input  logic    x_valid,
    output sample_t y_out,
    output logic    y_valid
);

    // ----------------------------------------------------------------
    // Coefficient ROM — 3 × 32-bit (b0, b1, a1) Q3.29
    // ----------------------------------------------------------------
    (* rom_style = "distributed" *)
    logic [31:0] coef_mem [0:2];
    initial $readmemh(COEFF_FILE, coef_mem);

    wire signed [31:0] b0_q29 = $signed(coef_mem[0]);
    wire signed [31:0] b1_q29 = $signed(coef_mem[1]);
    wire signed [31:0] a1_q29 = $signed(coef_mem[2]);

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
    // S1 — align b-products, 3-way accumulate → acc_q68 (80-bit).
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
    // S2 — round acc_q68 >>> 29 → Q9.39 y_q39_next.  Register.
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
    // S3a — narrow Q9.39 → Q1.23 via round >>> 16; register rounded
    //       96-bit value.  Also commits x_prev / y_prev_q39 state here
    //       (they don't depend on the rounded output).  S3b does the
    //       saturate on the next cycle so the large add-with-bias and
    //       the saturating compare do not share one sys_clk period.
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
    // S3b — saturate rounded value to sample_t.
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
