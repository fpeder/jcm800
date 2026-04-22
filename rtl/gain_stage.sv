
// =====================================================================
// gain_stage — one redesigned JCM800 triode stage
//
//   Intra-stage order (spec §7 — non-negotiable for matching SPICE and
//   keeping the noise floor well-behaved across the cascade):
//
//     x_in ──► [Vgk clamp] ──► LUT + PCHIP (f_norm: Vak/Vk bias-folded)
//                                      │
//                                      ▼
//                             × G_STAGE_Q4_20[STAGE]
//                                      │
//                                      ▼
//                                  iir_lpf1        (plate LPF, Miller)
//                                      ▼
//                                cathode_shelf    (first-order biquad)
//                                      ▼
//                                  iir_hpf1       (coupling HPF, τ≈10 ms)
//                                      ▼
//                                    y_out
//
//   Pre-LUT signal is Q1.23 in the LUT's "Vgk-normalised" domain — V1A's
//   input scale lives at the top level, not inside this module.  The LUT
//   output f_norm is bias-folded (zero at quiescence), so silence does
//   not accumulate a DC residue through the cascade.
//
//   The Vgk-clamp keeps the BRAM address inside [0, 4095] with a 12-LSB
//   margin, so the PCHIP cubic cannot extrapolate into the zeroed-
//   tangent saturation shoulder from inside.
//
//   LUT+PCHIP datapath (S0–S6) is UNCHANGED from the old design — same
//   6-stage pipeline, same triode_bram dual-port reads, same Hermite
//   basis polynomials.  What changes is what the LUTs contain (normalised
//   plate voltage, not current) and what happens after S6 (×G + filters
//   instead of feeding the next stage directly).
//
//   Pipeline budget:
//     LUT+PCHIP  6 cycles
//     ×G_stage   2 cycles (mul + round-narrow-saturate)
//     iir_lpf1   2 cycles
//     shelf      2 cycles
//     iir_hpf1   2 cycles
//     total     14 cycles  (~0.7% of a 48 kHz sample period at 100 MHz)
// =====================================================================
module gain_stage
    import jcm800_pkg::*;
    import jcm800_lut_pkg::*;
#(
    parameter stage_id_t STAGE       = STAGE_V1A,
    parameter string     LUT_FILE    = "v1a_lut.mem",
    parameter string     TAN_FILE    = "v1a_tan.mem",
    parameter string     LPF_FILE    = "lpf_v1a.mem",
    parameter string     SHELF_FILE  = "shelf_v1a.mem",
    parameter string     HPF_FILE    = "hpf_v1a_out.mem",
    // Non-zero override bypasses G_STAGE_Q4_20[STAGE] — for stages the
    // auto-generated jcm800_lut_pkg does not yet know about (e.g. PI_A/PI_B).
    parameter logic signed [31:0] G_OVERRIDE_Q4_20 = 32'sh0,
    // Gap 2 realism fix: for a bypassed-cathode stage (V1A in the 2203),
    // move cathode_shelf PRE-LUT as a pre-emphasis filter so LF content
    // enters the LUT with its correctly-degenerated gain.  The .mem file
    // content must match: SHELF_FILE expects the pre-emphasis form
    // (b0, b1 scaled by 1/(1+gm·Rk)) when this flag is asserted, or the
    // boost-HF form otherwise.  scripts/gen_triode_lut.py emits the
    // correct form based on stage name (V1A is pre-emphasis today).
    parameter bit SHELF_PRE_LUT = 1'b0
)(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t x_in,
    input  logic    x_valid,
    output sample_t y_out,
    output logic    y_valid
);

    // ================================================================
    // Vgk clamp — reserve 12 LSB of margin so BRAM addr < 4095 and
    // addr+1 ≤ 4096 without a boundary-case mux in triode_bram.
    // ================================================================
    sample_t x_clamped;
    always_comb begin
        if      ($signed(x_in) >  $signed(24'sh7FF000)) x_clamped =  24'sh7FF000;
        else if ($signed(x_in) < -$signed(24'sh7FF000)) x_clamped = -24'sh7FF000;
        else                                            x_clamped =  x_in;
    end

    // ================================================================
    // Gap 2 — conditional shelf placement.  The generate block below
    // routes cathode_shelf either pre-LUT (x_clamped → lut_in) or
    // post-LPF (lpf_y → hpf_in).  The unused side is bypassed with a
    // direct wire, keeping gain_stage structurally identical aside from
    // the shelf's 4-cycle pipeline showing up on whichever side is active.
    // ================================================================
    sample_t lut_in;
    logic    lut_in_valid;
    sample_t hpf_in;
    logic    hpf_in_valid;

    // ================================================================
    // S0 — latch shelf-or-clamp input on its valid pulse
    // ================================================================
    sample_t s0_x;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)             s0_x <= '0;
        else if (lut_in_valid)  s0_x <= lut_in;
    end

    // ================================================================
    // S2 — bias (offset binary) + addr decode (combinational dispatch)
    //   biased  = s0_x XOR 0x800000   (Q1.23 → 0..0xFFFFFF unsigned)
    //   addr    = biased[23:12]       (12-bit, 0..4095)
    //   addr_p1 = addr + 1            (13-bit, 1..4096)
    //   t       = biased[11:1]        (Q0.11 unsigned, 0..2047)
    // ================================================================
    logic [23:0] biased;
    logic [11:0] addr;
    logic [12:0] addr_p1;
    logic [10:0] t_c;

    assign biased  = s0_x ^ 24'h800000;
    assign addr    = biased[23:12];
    assign addr_p1 = {1'b0, addr} + 13'd1;
    assign t_c     = biased[11:1];

    // ================================================================
    // S3 — dual BRAM reads + t² register (BRAM has 1-cycle latency,
    // so its outputs land at S3 while t²_c is computed combinationally
    // from the live t_c bits and registered in parallel).
    // ================================================================
    sample_t      s3_y0, s3_y1;
    sample_t      s3_t0, s3_t1;
    logic [10:0]  s3_t;
    logic [21:0]  s3_t2;

    triode_bram #(.INIT_FILE(LUT_FILE)) u_lut (
        .clk    (clk),
        .addr_a ({1'b0, addr}),
        .addr_b (addr_p1),
        .q_a    (s3_y0),
        .q_b    (s3_y1)
    );
    triode_bram #(.INIT_FILE(TAN_FILE)) u_tan (
        .clk    (clk),
        .addr_a ({1'b0, addr}),
        .addr_b (addr_p1),
        .q_a    (s3_t0),
        .q_b    (s3_t1)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_t  <= '0;
            s3_t2 <= '0;
        end else begin
            s3_t  <= t_c;
            s3_t2 <= t_c * t_c;
        end
    end

    // ================================================================
    // S4 — four PCHIP basis polynomials in Q1.22 s26
    //   t³ = (t · t²) >>> 11
    //   h00 = (1<<22) − 3·t² + 2·t³
    //   h01 =           3·t² − 2·t³
    //   h10 = (t<<11) − 2·t² +   t³
    //   h11 =               −   t² + t³
    // ================================================================
    logic [32:0]        t_t2_full;
    logic [21:0]        t3_c;

    logic signed [25:0] t_q22_s, t2_q22_s, t3_q22_s;
    logic signed [25:0] one_q22;
    logic signed [25:0] h00_c, h10_c, h01_c, h11_c;

    assign t_t2_full = s3_t * s3_t2;
    assign t3_c      = t_t2_full[32:11];

    assign t_q22_s  = $signed({4'b0000, s3_t,  11'b000_0000_0000});
    assign t2_q22_s = $signed({4'b0000, s3_t2});
    assign t3_q22_s = $signed({4'b0000, t3_c});
    assign one_q22  = 26'sd4194304;

    always_comb begin
        h00_c = one_q22 - (t2_q22_s <<< 1) - t2_q22_s + (t3_q22_s <<< 1);
        h01_c =          (t2_q22_s <<< 1) + t2_q22_s - (t3_q22_s <<< 1);
        h10_c = t_q22_s - (t2_q22_s <<< 1) + t3_q22_s;
        h11_c =                              t3_q22_s - t2_q22_s;
    end

    logic signed [25:0] s4_h00, s4_h10, s4_h01, s4_h11;
    sample_t            s4_y0,  s4_y1,  s4_t0,  s4_t1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s4_h00 <= '0; s4_h10 <= '0; s4_h01 <= '0; s4_h11 <= '0;
            s4_y0  <= '0; s4_y1  <= '0; s4_t0  <= '0; s4_t1  <= '0;
        end else begin
            s4_h00 <= h00_c; s4_h10 <= h10_c;
            s4_h01 <= h01_c; s4_h11 <= h11_c;
            s4_y0  <= s3_y0; s4_y1  <= s3_y1;
            s4_t0  <= s3_t0; s4_t1  <= s3_t1;
        end
    end

    // ================================================================
    // S5 — four DSP multiplies (hXX [Q1.22 s26] × {y0,t0,y1,t1} [Q1.23 s24])
    // ================================================================
    logic signed [49:0] s5_p00, s5_p10, s5_p01, s5_p11;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s5_p00 <= '0; s5_p10 <= '0; s5_p01 <= '0; s5_p11 <= '0;
        end else begin
            s5_p00 <= s4_h00 * s4_y0;
            s5_p10 <= s4_h10 * s4_t0;
            s5_p01 <= s4_h01 * s4_y1;
            s5_p11 <= s4_h11 * s4_t1;
        end
    end

    // ================================================================
    // S6 — sum 4 products into sum52 and register; the round+saturate
    // chain (96-bit round + narrow + sat-compare) moves to S7.  This
    // keeps the LUT datapath at 100 MHz when the extra iir_lpf1 stages
    // push combinational slack tight elsewhere.
    // ================================================================
    logic signed [51:0] sum52_c;
    logic signed [51:0] s6_sum52;

    always_comb begin
        sum52_c = $signed(s5_p00) + $signed(s5_p10)
                + $signed(s5_p01) + $signed(s5_p11);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) s6_sum52 <= '0;
        else        s6_sum52 <= sum52_c;
    end

    // ================================================================
    // S7a — round sum52 >>> 22 → Q1.23, register the rounded 96-bit
    //       value.  Splitting round and saturate across two pipeline
    //       stages breaks the long 96-bit carry chain (add+compare) that
    //       would otherwise run in a single sys_clk cycle.
    // ================================================================
    logic signed [95:0] sum52_ext;
    logic signed [95:0] lut_y_96_c;
    logic signed [95:0] lut_y_96_q;

    always_comb begin
        sum52_ext  = {{44{s6_sum52[51]}}, s6_sum52};
        lut_y_96_c = round_conv96(sum52_ext, 22);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) lut_y_96_q <= '0;
        else        lut_y_96_q <= lut_y_96_c;
    end

    // ================================================================
    // S7b — saturate rounded value to Q1.23 sample_t.
    // ================================================================
    sample_t s6_y;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) s6_y <= '0;
        else        s6_y <= saturate_s24(lut_y_96_q);
    end

    // Valid pipeline for S0..S7b — 8 shift stages (one added for the new
    // round/saturate split).  Tracks lut_in_valid so the pipeline shifts
    // in lock-step with whatever drove S0, whether that's x_valid directly
    // (SHELF_PRE_LUT=0) or the pre-shelf's output valid (SHELF_PRE_LUT=1,
    // lagged by 4 cycles).
    logic [7:0] valid_sr;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) valid_sr <= '0;
        else        valid_sr <= {valid_sr[6:0], lut_in_valid};
    end
    logic s6_valid;
    assign s6_valid = valid_sr[7];

    // ================================================================
    // G_stage multiplier — Q1.23 × Q4.20 → Q5.43 (56-bit signed)
    // Convergent-round >>> 20 → Q1.23 ; saturate.
    // 2-cycle pipeline (mul register → round/narrow register).
    // ================================================================
    localparam logic signed [31:0] G_USED =
        (G_OVERRIDE_Q4_20 != 32'sh0) ? G_OVERRIDE_Q4_20 : G_STAGE_Q4_20[STAGE];

    logic signed [31:0] g_stage;
    assign g_stage = G_USED;

    logic signed [55:0] g_prod_c;
    logic signed [55:0] g0_prod;
    logic               g0_valid;

    always_comb g_prod_c = g_stage * s6_y;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g0_prod  <= '0;
            g0_valid <= 1'b0;
        end else begin
            g0_valid <= s6_valid;
            if (s6_valid) g0_prod <= g_prod_c;
        end
    end

    logic signed [95:0] g_prod_ext;
    logic signed [95:0] g_q23_96_c;
    logic signed [95:0] g_q23_96_q;
    logic               g_round_valid;

    always_comb begin
        g_prod_ext = {{40{g0_prod[55]}}, g0_prod};
        g_q23_96_c = round_conv96(g_prod_ext, SHIFT_G);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g_q23_96_q    <= '0;
            g_round_valid <= 1'b0;
        end else begin
            g_round_valid <= g0_valid;
            if (g0_valid) g_q23_96_q <= g_q23_96_c;
        end
    end

    sample_t g_y;
    logic    g_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g_y     <= '0;
            g_valid <= 1'b0;
        end else begin
            g_valid <= g_round_valid;
            if (g_round_valid) g_y <= saturate_s24(g_q23_96_q);
        end
    end

    // ================================================================
    // iir_lpf1 — plate LPF (Miller + Rp_ac)
    // ================================================================
    sample_t lpf_y;
    logic    lpf_valid;

    iir_lpf1 #(.COEFF_FILE(LPF_FILE)) u_lpf (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (g_y),
        .x_valid (g_valid),
        .y_out   (lpf_y),
        .y_valid (lpf_valid)
    );

    // ================================================================
    // cathode_shelf — position selected by SHELF_PRE_LUT.
    //   =0 (default, unbypassed stages and the PI): shelf sits between
    //       the plate LPF and the output coupling HPF, as before.  For
    //       stages with no bypass cap the .mem encodes identity and this
    //       instance is a 4-cycle delay with no tonal effect.
    //   =1 (V1A in the JCM800 2203): shelf sits BEFORE the LUT, acting
    //       as a pre-emphasis that attenuates LF so the LUT sees the
    //       correctly-degenerated LF amplitude.  The post-LPF path then
    //       wires straight to the HPF.
    // ================================================================
    generate
        if (SHELF_PRE_LUT) begin : g_pre_shelf
            cathode_shelf #(.COEFF_FILE(SHELF_FILE)) u_shelf (
                .clk     (clk),
                .rst_n   (rst_n),
                .x_in    (x_clamped),
                .x_valid (x_valid),
                .y_out   (lut_in),
                .y_valid (lut_in_valid)
            );
            assign hpf_in       = lpf_y;
            assign hpf_in_valid = lpf_valid;
        end else begin : g_post_shelf
            assign lut_in       = x_clamped;
            assign lut_in_valid = x_valid;
            cathode_shelf #(.COEFF_FILE(SHELF_FILE)) u_shelf (
                .clk     (clk),
                .rst_n   (rst_n),
                .x_in    (lpf_y),
                .x_valid (lpf_valid),
                .y_out   (hpf_in),
                .y_valid (hpf_in_valid)
            );
        end
    endgenerate

    // ================================================================
    // iir_hpf1 — output coupling HPF (+ Gap 1 asymmetric bias tracker)
    // ================================================================
    iir_hpf1 #(.COEFF_FILE(HPF_FILE)) u_hpf (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (hpf_in),
        .x_valid (hpf_in_valid),
        .y_out   (y_out),
        .y_valid (y_valid)
    );

endmodule
