
// =====================================================================
// phase_inverter — JCM800 2203 long-tailed-pair (LTP) phase splitter
//
//   Drives the push-pull power stage.  Gap 3 realism fix (see
//   /home/fab/.claude/plans/way-of-improving-sound-glimmering-marshmallow.md):
//   instead of the "two independent triodes, one gets sign-flipped input"
//   ideal-LTP shortcut, BOTH arms are driven with the SAME signal and
//   their LUTs encode the PLATE RESPONSE AS A FUNCTION OF V3A GRID DRIVE
//   under a joint shared-tail solve (see scripts/gen_triode_lut.py :
//   _solve_ltp_vin).  LUT_B has its contents pre-inverted so both arms
//   follow the same digital sign convention (G_stage = −1, natural slope
//   negative), and their outputs are already anti-phase.
//
//       x_in ──► [reg] ─┬─► gain_stage V3A (pi_a_*.mem) ──► y_pos
//                       │
//                       └─► gain_stage V3B (pi_b_*.mem) ──► y_neg
//
//   Each side owns its own LUT+PCHIP pipeline, ×G multiplier, plate
//   LPF, cathode shelf (identity — shared tail, no per-arm Ck), and
//   coupling HPF to its EL34 grid.  Runs in the 768 kHz oversampled
//   domain so its nonlinear harmonics fold cleanly inside Nyquist.
//
//   Parameter defaults come from jcm800_lut_pkg; override any of them
//   at instantiation to swap in alternative PI shapes (e.g. a bypassed-
//   cathode "bright" PI, or SPICE-matched curves).
// =====================================================================
module phase_inverter
    import jcm800_pkg::*;
    import jcm800_lut_pkg::*;
#(
    // STAGE_A/STAGE_B select which G_STAGE_Q4_20 entry to fall back on
    // when the G_OVERRIDE values below are zero.  With non-zero overrides
    // (the default below), these are effectively unused.
    parameter stage_id_t STAGE_A = STAGE_V2B,
    parameter stage_id_t STAGE_B = STAGE_V2B,

    // LTP-aware gain overrides (Q4.20 signed).  Both come out of the
    // generator as −1.0 because LUT_B is pre-inverted for anti-phase.
    parameter logic signed [31:0] G_PI_A_Q4_20_OV = jcm800_lut_pkg::G_PI_A_Q4_20,
    parameter logic signed [31:0] G_PI_B_Q4_20_OV = jcm800_lut_pkg::G_PI_B_Q4_20,

    parameter string LUT_FILE_A   = jcm800_lut_pkg::PI_A_LUT_FILE,
    parameter string TAN_FILE_A   = jcm800_lut_pkg::PI_A_TAN_FILE,
    parameter string LPF_FILE_A   = jcm800_lut_pkg::PI_A_LPF_FILE,
    parameter string SHELF_FILE_A = jcm800_lut_pkg::PI_A_SHELF_FILE,
    parameter string HPF_FILE_A   = jcm800_lut_pkg::PI_A_HPF_FILE,

    parameter string LUT_FILE_B   = jcm800_lut_pkg::PI_B_LUT_FILE,
    parameter string TAN_FILE_B   = jcm800_lut_pkg::PI_B_TAN_FILE,
    parameter string LPF_FILE_B   = jcm800_lut_pkg::PI_B_LPF_FILE,
    parameter string SHELF_FILE_B = jcm800_lut_pkg::PI_B_SHELF_FILE,
    parameter string HPF_FILE_B   = jcm800_lut_pkg::PI_B_HPF_FILE
)(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t x_in,
    input  logic    x_valid,

    // Negative feedback injection into the shared V3 cathode (see
    // nfb_network.sv).  In the real 2203 the OT secondary is attenuated
    // and fed back to V3's shared cathode resistor, effectively
    // subtracting a fraction of the output from the grid drive.  We
    // model that by summing −nfb_inject into x_q before the LUT address
    // decode; both arms see the same correction.  nfb_inject is a held
    // value from nfb_network (updated once per 768 kHz sample with a
    // 1-sample delay that breaks the algebraic loop cleanly).
    input  sample_t nfb_inject,

    output sample_t y_pos,      // V3A plate  (anti-phase of y_neg)
    output sample_t y_neg,      // V3B plate
    output logic    y_valid
);

    // ------------------------------------------------------------------
    // Single input register — both arms start their pipelines on the
    // same cycle so their 14-stage gain_stage latencies stay aligned.
    // No input negation: the opposite-phase response of V3B is already
    // encoded in LUT_B (pre-inverted by the generator under the shared-
    // tail LTP solve).
    //
    // NFB is summed in here (x_in − nfb_inject) with a saturating
    // 25-bit intermediate before the narrow back to sample_t.  The
    // feedback factor is small (≈ 0.045 baked into nfb_network), so the
    // combined value almost never saturates except under heavy clip.
    // ------------------------------------------------------------------
    sample_t x_q;
    logic    x_valid_q;

    logic signed [24:0] x_sum25_c;
    sample_t            x_sum_sat_c;

    always_comb begin
        x_sum25_c = $signed({x_in[23], x_in}) - $signed({nfb_inject[23], nfb_inject});
        if      (x_sum25_c >  25'sd8388607)  x_sum_sat_c =  24'sh7FFFFF;
        else if (x_sum25_c < -25'sd8388608)  x_sum_sat_c =  24'sh800000;
        else                                 x_sum_sat_c =  x_sum25_c[23:0];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_q       <= '0;
            x_valid_q <= 1'b0;
        end else begin
            x_q       <= x_sum_sat_c;
            x_valid_q <= x_valid;
        end
    end

    // ------------------------------------------------------------------
    // V3A — signal arm (Rp_a = 82 kΩ in the real 2203)
    // ------------------------------------------------------------------
    gain_stage #(
        .STAGE            (STAGE_A),
        .LUT_FILE         (LUT_FILE_A),
        .TAN_FILE         (TAN_FILE_A),
        .LPF_FILE         (LPF_FILE_A),
        .SHELF_FILE       (SHELF_FILE_A),
        .HPF_FILE         (HPF_FILE_A),
        .G_OVERRIDE_Q4_20 (G_PI_A_Q4_20_OV)
    ) u_v3a (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (x_q),
        .x_valid (x_valid_q),
        .y_out   (y_pos),
        .y_valid (y_valid)
    );

    // ------------------------------------------------------------------
    // V3B — reference arm (Rp_b = 100 kΩ; grid AC-grounded in the real
    // circuit).  Fed with the SAME x_q as V3A.  The anti-phase output
    // comes from LUT_B's pre-inverted contents, not from negated input.
    // Same 14-cycle pipeline so its y_valid tracks u_v3a's — we take
    // the shared y_valid from u_v3a and drop this one.
    // ------------------------------------------------------------------
    logic v3b_valid_unused;

    gain_stage #(
        .STAGE            (STAGE_B),
        .LUT_FILE         (LUT_FILE_B),
        .TAN_FILE         (TAN_FILE_B),
        .LPF_FILE         (LPF_FILE_B),
        .SHELF_FILE       (SHELF_FILE_B),
        .HPF_FILE         (HPF_FILE_B),
        .G_OVERRIDE_Q4_20 (G_PI_B_Q4_20_OV)
    ) u_v3b (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (x_q),
        .x_valid (x_valid_q),
        .y_out   (y_neg),
        .y_valid (v3b_valid_unused)
    );

endmodule
