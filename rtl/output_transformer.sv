
// =====================================================================
// output_transformer — JCM800 2203 push-pull OT model
//
//   Pragmatic pipeline at 768 kHz:
//
//     ip_a, ip_b (Q1.23)
//           │
//           ▼  half-difference (keeps Q1.23 range when both tubes swing
//              to opposite rails)
//         i_prim = (ip_a − ip_b) · 0.5
//           │
//           ▼  iir_hpf1 (primary-inductance roll-off, fc ≈ Raa/2π·Lp
//              ≈ 11 Hz — models the OT bleeding drive into Lp at LF)
//         i_hp
//           │
//           ▼  pentode_stage (re-used as a bare LUT+PCHIP+×G; vg2_ratio
//              tied to 1.0) with the OT core-sat tanh-shaped curve
//         i_sat
//           │
//           ▼  iir_lpf1 (leakage-inductance roll-off, fc ≈ Raa/2π·Lleak
//              ≈ 18 kHz — models HF blocking by series L_leak)
//         v_sec (Q1.23; nominally scaled to match NFB + downsample budget)
//
//   Core saturation is applied on the instantaneous current here rather
//   than on an integrated flux state.  A flux-based model would be
//   strictly more physical (saturation is a volt-second phenomenon), but
//   at the drive levels we care about the instantaneous compression gives
//   a similar audible character without a second integrator state.  The
//   soft-clip knee is baked into ot_sat_lut.mem by gen_pentode_lut.py.
//
//   The secondary voltage is expressed in the same Q1.23 domain as every
//   other audio sample in the chain; the TURNS_RATIO scaling is a fixed
//   property of the transformer the generator emits (no runtime pot).
// =====================================================================
module output_transformer
    import jcm800_pkg::*;
    import jcm800_power_pkg::*;
(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t ip_a,
    input  logic    ip_a_valid,
    input  sample_t ip_b,
    // ip_b is assumed to share ip_a_valid (both tubes get the same strobe
    // from the push-pull driver above).  Keep the second valid out of the
    // interface — it saved a shift register in jcm800.sv.

    output sample_t v_sec,
    output logic    v_sec_valid
);

    // -----------------------------------------------------------------
    // Half-difference with saturation.  25-bit intermediate covers the
    // full range of (ip_a − ip_b) before the halver scales it back to s24.
    // -----------------------------------------------------------------
    logic signed [24:0] diff25_c;
    assign diff25_c = $signed({ip_a[23], ip_a}) - $signed({ip_b[23], ip_b});

    // Apply PP_HALF_Q4_20 (= 2.0 — despite the name; the raw EL34 LUT sits
    // around ±0.37 peak so the push-pull difference is scaled up, not down,
    // to recover Q1.23 headroom) via the same Q4.20 × Q1.23 → round >>>20
    // pattern as gain_stage's ×G_stage.  We operate on the 25-bit diff
    // directly: prod56 = PP_HALF · diff25  (Q4.20 · Q1.24 → Q5.44 s57 ≤ 56b).
    // Round >>> 20 into Q1.24, saturate back to s24.
    logic signed [55:0] pp_prod_c;
    logic signed [55:0] pp_prod_r;
    logic               pp_valid;

    always_comb pp_prod_c = $signed(PP_HALF_Q4_20) * diff25_c;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pp_prod_r <= '0;
            pp_valid  <= 1'b0;
        end else begin
            pp_valid <= ip_a_valid;
            if (ip_a_valid) pp_prod_r <= pp_prod_c;
        end
    end

    logic signed [95:0] pp_prod_ext;
    logic signed [95:0] i_prim_96_c;
    logic signed [95:0] i_prim_96_q;
    logic               pp_round_valid;

    always_comb begin
        pp_prod_ext = {{40{pp_prod_r[55]}}, pp_prod_r};
        i_prim_96_c = round_conv96(pp_prod_ext, SHIFT_PP);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_prim_96_q    <= '0;
            pp_round_valid <= 1'b0;
        end else begin
            pp_round_valid <= pp_valid;
            if (pp_valid) i_prim_96_q <= i_prim_96_c;
        end
    end

    sample_t i_prim_r;
    logic    i_prim_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_prim_r     <= '0;
            i_prim_valid <= 1'b0;
        end else begin
            i_prim_valid <= pp_round_valid;
            if (pp_round_valid) i_prim_r <= saturate_s24(i_prim_96_q);
        end
    end

    // -----------------------------------------------------------------
    // Primary-inductance HPF.  Reuse iir_hpf1 with γ=0 (no bias tracker,
    // emitted by gen_pentode_lut.py write_hpf_mem).  5-cycle latency.
    // -----------------------------------------------------------------
    sample_t i_hp;
    logic    i_hp_valid;

    iir_hpf1 #(
        .COEFF_FILE (OT_HPF_FILE)
    ) u_hpf_prim (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (i_prim_r),
        .x_valid (i_prim_valid),
        .y_out   (i_hp),
        .y_valid (i_hp_valid)
    );

    // -----------------------------------------------------------------
    // Core-saturation LUT reusing pentode_stage's LUT+PCHIP+×G pipeline.
    // vg2_ratio pinned to 1.0 (0x4000) so the post-×G multiplier is a
    // pass-through.  We discard the Ig2 side output — the OT path doesn't
    // feed screen current; it's stale from the LUT abs/scale hardware.
    // -----------------------------------------------------------------
    sample_t i_sat;
    logic    i_sat_valid;
    sample_t ot_ig2_unused;
    logic    ot_ig2_valid_unused;

    pentode_stage #(
        .LUT_FILE (OT_SAT_LUT_FILE),
        .TAN_FILE (OT_SAT_TAN_FILE)
    ) u_sat (
        .clk             (clk),
        .rst_n           (rst_n),
        .x_in            (i_hp),
        .x_valid         (i_hp_valid),
        .vg2_ratio_q2_14 (16'sh4000),       // pass-through scale
        .ip_out          (i_sat),
        .ip_valid        (i_sat_valid),
        .ig2_out         (ot_ig2_unused),
        .ig2_valid       (ot_ig2_valid_unused)
    );

    // -----------------------------------------------------------------
    // Leakage-inductance LPF.  5-cycle latency.  The secondary voltage
    // shares the primary's Q1.23 domain; no turns-ratio scaling is
    // applied in the stream (the NFB network and downsampler normalise
    // absolute levels together with the rest of the preamp gain budget).
    // -----------------------------------------------------------------
    iir_lpf1 #(
        .COEFF_FILE (OT_LPF_FILE)
    ) u_lpf_leak (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (i_sat),
        .x_valid (i_sat_valid),
        .y_out   (v_sec),
        .y_valid (v_sec_valid)
    );

endmodule
