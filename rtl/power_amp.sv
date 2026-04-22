
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
//                                   └────┴─► screen_supply ──► vg2_ratio
//                                                              (shared)
//                                                                 │
//                                   ┌──────────────────────────────┘
//                                   │ (fed back as input scale/sag)
//                                   ▼
//                              output_transformer (Ip_A − Ip_B,
//                                                  HPF → OT sat LUT → LPF)
//                                   │
//                                   ▼
//                                 v_sec  (to downsample + NFB tap)
//
//   Both tubes use the same el34_lut.mem — PI already delivers anti-
//   phase grid drives, so the push-pull sum in output_transformer.sv
//   recovers class-AB operation naturally.
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
    // -----------------------------------------------------------------
    logic signed [15:0] vg2_ratio_q2_14;

    // -----------------------------------------------------------------
    // Tube A
    // -----------------------------------------------------------------
    sample_t ip_a;
    logic    ip_a_valid;
    sample_t ig2_a;
    logic    ig2_a_valid;

    pentode_stage u_tube_a (
        .clk             (clk),
        .rst_n           (rst_n),
        .x_in            (y_pos),
        .x_valid         (y_valid),
        .vg2_ratio_q2_14 (vg2_ratio_q2_14),
        .ip_out          (ip_a),
        .ip_valid        (ip_a_valid),
        .ig2_out         (ig2_a),
        .ig2_valid       (ig2_a_valid)
    );

    // -----------------------------------------------------------------
    // Tube B (identical LUT; PI delivers anti-phase y_neg)
    // -----------------------------------------------------------------
    sample_t ip_b;
    logic    ip_b_valid;
    sample_t ig2_b;
    logic    ig2_b_valid;

    pentode_stage u_tube_b (
        .clk             (clk),
        .rst_n           (rst_n),
        .x_in            (y_neg),
        .x_valid         (y_valid),
        .vg2_ratio_q2_14 (vg2_ratio_q2_14),
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
