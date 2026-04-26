
// =====================================================================
// power_amp — JCM800 2203 push-pull EL34 output stage + OT
//
//   Thin structural layer between the phase_inverter arms and the
//   output transformer.  Two pentode_stage instances share the same
//   768 kHz valid strobe (both PI arms fire on the same cycle because
//   their LUT pipelines have identical depth), so the tubes stay
//   phase-aligned through the power section.
//
//       y_pos (V3A plate) ──► pentode_stage A ──► ip_a ─┐
//       y_neg (V3B plate) ──► pentode_stage B ──► ip_b ─┤
//                                   │    │                │
//                                   │    └──► screen_supply ──► vg2_ratio
//                                   └─────► ht_supply     ──► ht_ratio
//                                           (|ip_a|+|ip_b|)
//                                                                 │
//                                   ┌──── vg2_ratio × ht_ratio ────┘
//                                   │      = pent_scale_q2_14
//                                   ▼      (combined plate-output scale
//                                           passed to both pentodes)
//                              output_transformer (Ip_A − Ip_B,
//                                                  HPF → OT sat LUT → LPF)
//                                   │
//                                   ▼
//                                 v_sec  (to downsample + NFB tap)
//
//   Both tubes use the same el34_lut.mem — PI already delivers anti-
//   phase grid drives, so the push-pull sum in output_transformer.sv
//   recovers class-AB operation naturally.
//
//   Supply sag: vg2_ratio (screen, driven by |Ig2| sum) compresses plate
//   current as the screen bypass cap discharges.  ht_ratio (HT rail,
//   driven by |Ip_a|+|Ip_b|) compresses peak swing as the B+ reservoir
//   sags under sustained drive — the "bloom" character.  The two slow
//   envelopes multiply before entering each pentode; we combine them
//   here rather than adding a second scheduled op inside pentode_stage
//   because the result is still a single Q2.14 multiplicand and the
//   envelopes are orders of magnitude slower than the audio rate.
// =====================================================================
module power_amp
    import jcm800_pkg::*;
    import jcm800_power_pkg::*;
(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t y_pos,
    input  sample_t y_neg,
    input  logic    y_valid,

    output sample_t v_sec,
    output logic    v_sec_valid
);

    // -----------------------------------------------------------------
    // Shared Vg2 state.  Driven by the sum of the two tubes' Ig2 outputs
    // (computed once per sample inside each pentode_stage as
    // |ip_out| · IG2_RATIO_Q1_23).
    // Also: shared HT rail state, driven by |Ip_a|+|Ip_b| sampled on the
    // same valid strobe.  Both are Q2.14 signed (nom = 0x4000 = 1.0).
    // -----------------------------------------------------------------
    logic signed [15:0] vg2_ratio_q2_14;
    logic signed [15:0] ht_ratio_q2_14;

    // The combined plate-output scale fed to each pentode's
    // vg2_ratio_q2_14 input.  vg2_ratio × ht_ratio in Q2.14·Q2.14 → Q4.28,
    // sliced back to Q2.14.  Registered to keep the 16×16 multiplier off
    // any long combinational chain.  At reset both sources sit at 1.0,
    // so pent_scale resets to 1.0 as well (no pop on release).
    logic signed [31:0] pent_scale_prod_q;
    logic signed [15:0] pent_scale_q2_14;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 0x4000 · 0x4000 = 0x1000_0000 = 1.0 in Q4.28.
            pent_scale_prod_q <= 32'sh10000000;
        end else begin
            pent_scale_prod_q <= $signed(vg2_ratio_q2_14)
                               * $signed(ht_ratio_q2_14);
        end
    end

    // Q4.28 → Q2.14: right-shift 14.  Inputs are both non-negative and
    // ≤ 1.0 in practice, so the product is ≤ 1.0 and bits [31:30] are
    // zero — slicing the 16-bit window [29:14] is both sign-correct and
    // overflow-free without an explicit saturator.
    assign pent_scale_q2_14 = pent_scale_prod_q[29:14];

    // -----------------------------------------------------------------
    // Tube A
    // -----------------------------------------------------------------
    sample_t ip_a;
    logic    ip_a_valid;
    sample_t ig2_a;
    logic    ig2_a_valid;

    pentode_stage #(
        .LUT_FILE         (EL34_LUT_A_FILE),
        .TAN_FILE         (EL34_TAN_A_FILE),
        .G_OVERRIDE_Q4_20 (G_EL34_A_Q4_20)
    ) u_tube_a (
        .clk             (clk),
        .rst_n           (rst_n),
        .x_in            (y_pos),
        .x_valid         (y_valid),
        .vg2_ratio_q2_14 (pent_scale_q2_14),
        .ip_out          (ip_a),
        .ip_valid        (ip_a_valid),
        .ig2_out         (ig2_a),
        .ig2_valid       (ig2_a_valid)
    );

    // -----------------------------------------------------------------
    // Tube B — slightly cooler bias (push-pull mismatch baked into
    // EL34_LUT_B_FILE by gen_pentode_lut.py).  PI still delivers
    // anti-phase y_neg; the small per-tube Vg1 offset adds 2nd-harmonic
    // content that a perfectly matched pair would cancel out.
    // -----------------------------------------------------------------
    sample_t ip_b;
    logic    ip_b_valid;
    sample_t ig2_b;
    logic    ig2_b_valid;

    pentode_stage #(
        .LUT_FILE         (EL34_LUT_B_FILE),
        .TAN_FILE         (EL34_TAN_B_FILE),
        .G_OVERRIDE_Q4_20 (G_EL34_B_Q4_20)
    ) u_tube_b (
        .clk             (clk),
        .rst_n           (rst_n),
        .x_in            (y_neg),
        .x_valid         (y_valid),
        .vg2_ratio_q2_14 (pent_scale_q2_14),
        .ip_out          (ip_b),
        .ip_valid        (ip_b_valid),
        .ig2_out         (ig2_b),
        .ig2_valid       (ig2_b_valid)
    );

    // -----------------------------------------------------------------
    // Shared screen-supply state.  Sum Ig2_A + Ig2_B at 25 bits so the
    // two Q1.23 values combine without losing the top bit when both
    // tubes briefly clip (|ig2_max| ≈ 0.3 of Q1.23 in normal use).
    // -----------------------------------------------------------------
    logic signed [24:0] ig2_sum_c;
    assign ig2_sum_c = $signed({ig2_a[23], ig2_a}) + $signed({ig2_b[23], ig2_b});

    screen_supply u_screen (
        .clk             (clk),
        .rst_n           (rst_n),
        .ig2_sum_q1_23   (ig2_sum_c),
        .ig2_sum_valid   (ig2_a_valid),    // both arms share the same strobe
        .vg2_ratio_q2_14 (vg2_ratio_q2_14)
    );

    // -----------------------------------------------------------------
    // Shared HT-supply (B+) state.  Driven by |Ip_A| + |Ip_B| instead of
    // the screen current proxy.  abs of the saturated Q1.23 ip samples
    // follows the same pattern as pentode_stage.sv's internal
    // ip_abs_c (treat the −2^23 corner as +2^23−1 to keep the result
    // non-negative without overflow), then sum at 25 bits.
    // -----------------------------------------------------------------
    sample_t ip_a_abs_c;
    sample_t ip_b_abs_c;

    always_comb begin
        if      (ip_a == 24'sh800000) ip_a_abs_c = 24'sh7FFFFF;
        else if (ip_a[23])            ip_a_abs_c = -ip_a;
        else                          ip_a_abs_c =  ip_a;

        if      (ip_b == 24'sh800000) ip_b_abs_c = 24'sh7FFFFF;
        else if (ip_b[23])            ip_b_abs_c = -ip_b;
        else                          ip_b_abs_c =  ip_b;
    end

    logic signed [24:0] ip_sum_c;
    // Both abs values are guaranteed non-negative → zero-extend to 25
    // bits and sum.  Worst-case sum is 2·(2^23−1) ≈ 2^24, still within
    // a 25-bit signed non-negative range.
    assign ip_sum_c = $signed({1'b0, ip_a_abs_c}) + $signed({1'b0, ip_b_abs_c});

    ht_supply u_ht (
        .clk            (clk),
        .rst_n          (rst_n),
        .ip_sum_q1_23   (ip_sum_c),
        .ip_sum_valid   (ip_a_valid),     // shared strobe with the pentodes
        .ht_ratio_q2_14 (ht_ratio_q2_14)
    );

    // -----------------------------------------------------------------
    // OT
    // -----------------------------------------------------------------
    output_transformer u_ot (
        .clk         (clk),
        .rst_n       (rst_n),
        .ip_a        (ip_a),
        .ip_a_valid  (ip_a_valid),
        .ip_b        (ip_b),
        .v_sec       (v_sec),
        .v_sec_valid (v_sec_valid)
    );

endmodule
