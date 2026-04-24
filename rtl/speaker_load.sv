
// =====================================================================
// speaker_load — 2nd-order biquad that shapes v_sec before it enters
//                the NFB loop, replacing the flat-8 Ω load assumption
//                with a reactive guitar-speaker Zs(f).
//
//   Transfer function realised:
//       y[n] = b0·x[n] + b1·x[n−1] + b2·x[n−2] − a1·y[n−1] − a2·y[n−2]
//
//   Coefficients come from scripts/gen_speaker_zs.py as a Bristow-Johnson
//   peaking-EQ biquad tuned for a typical 12" guitar speaker's
//   cone-resonance peak (≈ 80 Hz, Q ≈ 5, ≈ +15 dB above the nominal
//   plateau), with the baseline scaled by Re/Z_ideal so the DC gain
//   matches the real voice-coil DCR rather than the ideal 8 Ω assumption.
//
//   Coefficient file layout
//   ───────────────────────
//     5 entries × 32-bit Q3.29 hex at addresses 0..4:
//         addr 0: b0   (Q3.29)
//         addr 1: b1   (Q3.29)
//         addr 2: b2   (Q3.29)
//         addr 3: a1   (Q3.29)
//         addr 4: a2   (Q3.29)
//     a0 is normalised to 1.0 at generation time and not stored.
//     Q3.29 range is ±4 — comfortably holds all five coefficients for
//     the default peaking-EQ parameters (|b1|,|a1| ≈ 1.5 ; |a2| ≈ 1).
//
//   Datapath
//   ────────
//     x  : Q1.23 s24               x_prev1, x_prev2 : Q1.23 s24
//     y_prev{1,2}_q39 : Q9.39 s48  (wide state — same width as
//                                   cathode_shelf / the IIR HPF/LPF)
//
//     prod_b{0,1,2} = b·x     : Q4.52 s56
//     prod_a{1,2}   = a·y_prev: Q12.68 s80
//
//     Align b-products to Q12.68 by sign-extend + left-shift 16.
//     acc_q68 = (b0_al + b1_al + b2_al) − (a1_prod + a2_prod)
//               → 82-bit signed (one extra bit of headroom over the
//                 3-term cathode_shelf accumulator covers the five-way
//                 sum without relying on coefficient-range luck).
//     Convergent-round >>> 29 → Q9.39 (48-bit state).
//     Narrow to Q1.23 via round >>> 16 and saturate for y_out.
//
//   Pipeline (4 stages — matches cathode_shelf.sv's shape so timing
//   pressure is familiar and the 82-bit 5-way accumulate is split from
//   the 96-bit round):
//     S0 — on x_valid: compute five products, latch x, register products.
//     S1 — align b-products to Q12.68 and 5-way accumulate → acc_q68 (82b).
//     S2 — convergent-round acc_q68 >>> 29 → Q9.39.  Register s2_y_q39.
//     S3 — narrow Q9.39 → Q1.23 via round >>> 16 and register the wide
//          rounded value; commit state (x_prev1/2, y_prev1/2_q39).
//     S4 — saturate to s24, register y_out.
// =====================================================================
module speaker_load
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
    // Coefficient ROM — 5 × 32-bit Q3.29 (b0, b1, b2, a1, a2).
    // ----------------------------------------------------------------
    (* rom_style = "distributed" *)
    logic [31:0] coef_mem [0:4];
    initial $readmemh(COEFF_FILE, coef_mem);

    wire signed [31:0] b0_q29 = $signed(coef_mem[0]);
    wire signed [31:0] b1_q29 = $signed(coef_mem[1]);
    wire signed [31:0] b2_q29 = $signed(coef_mem[2]);
    wire signed [31:0] a1_q29 = $signed(coef_mem[3]);
    wire signed [31:0] a2_q29 = $signed(coef_mem[4]);

    // ----------------------------------------------------------------
    // State
    // ----------------------------------------------------------------
    sample_t             x_prev1, x_prev2;
    logic signed [47:0]  y_prev1_q39, y_prev2_q39;

    // ----------------------------------------------------------------
    // S0 — five multiplies; latch x; register products.
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
    // Pipeline x through the stages so x_prev1/x_prev2 commits at S3
    // against the same sample that entered S0.  (cathode_shelf does
    // the same s0_x_lat / s1_x_lat chain — we just carry s2_x_lat one
    // stage further because the 2nd-order state update needs x_prev2
    // alongside x_prev1.)  s2_x_lat is committed in the S2 block below.
    // ----------------------------------------------------------------
    sample_t s0_x_lat, s1_x_lat, s2_x_lat;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_x_lat <= '0;
            s1_x_lat <= '0;
        end else begin
            if (x_valid) s0_x_lat <= x_in;
            if (s0_valid) s1_x_lat <= s0_x_lat;
        end
    end

    // ----------------------------------------------------------------
    // S1 — align b-products to Q12.68 and 5-way accumulate.  Use 82-bit
    //      accumulator for headroom over the 3-term cathode_shelf sum.
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
            if (s0_valid) begin
                s1_acc_q68 <= acc_q68_c;
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
    // S3 — narrow Q9.39 → Q1.23 (round >>> 16), register rounded 96-bit
    //      value; commit state (x_prev1/2, y_prev1/2_q39).  S4 saturates.
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
    // S4 — saturate rounded value to sample_t.
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
