
// =====================================================================
// preamp — JCM800 2203 four-stage triode preamp
//
//   Four gain_stage instances with the schematic's two "Attenuator +
//   Treble Peak" bright pads and the Preamp Volume pot interleaved:
//
//       x_in ─► V1A ─►[BRIGHT_PAD_PRE_POT]─►[GAIN]─►
//                  ─► V1B ─►[BRIGHT_PAD_V1B_V2A]─► V2A ─► V2B ─► y_out
//
//   RTL stage labels are swapped vs the real-circuit schematic:
//       RTL V1A = schematic V1B (Preamp 1, input stage, bypassed cathode)
//       RTL V1B = schematic V1A (Preamp 2, cold clipper, 10 kΩ unbypassed)
//   Functionally identical — same circuit topology, just renamed.
//
//   Bright pads (470 kΩ series + 470 pF cap, feeding the next stage's
//   470 kΩ grid leak): −6 dB at DC rising to 0 dB above ~1.4 kHz.  See
//   scripts/gen_triode_lut.py BRIGHT_PADS dict and lut_out/bright_pad_*.
//   They are first-order shelves implemented with cathode_shelf.sv (the
//   same biquad form used elsewhere in the chain).
//
//   The Preamp Volume (Gain) pot is a log-taper attenuator driven by
//   `gain_pot_pos` (0..255, see scripts/gen_log_taper.py), positioned
//   between bright_pad_pre_pot and V1B (= schematic-V1A cold clipper)
//   exactly as on the real schematic.
//
//   Diagnostic taps expose the post-HPF output of V1A/V1B/V2A, held
//   between samples by the filter state inside each gain_stage.
//   `v1a_tap` is the PRE-pad V1A output (what a probe on V1A's plate
//   would see); the post-pad / post-pot signal is internal.
// =====================================================================
module preamp
    import jcm800_pkg::*;
    import jcm800_lut_pkg::*;
(
    input  logic       clk,
    input  logic       rst_n,
    input  sample_t    x_in,
    input  logic       x_valid,
    input  logic [7:0] gain_pot_pos,
    output sample_t    y_out,
    output logic       y_valid,

    output sample_t    v1a_tap,
    output sample_t    v1b_tap,
    output sample_t    v2a_tap
);

    sample_t v1a_y, v1b_y, v2a_y;
    logic    v1a_v, v1b_v, v2a_v;

    sample_t v1a_post_pad, v1a_post_gain;
    logic    v1a_post_pad_v, v1a_post_gain_v;

    sample_t v1b_post_pad;
    logic    v1b_post_pad_v;

    gain_stage #(
        .STAGE         (STAGE_V1A),
        .LUT_FILE      (LUT_FILE  [STAGE_V1A]),
        .TAN_FILE      (TAN_FILE  [STAGE_V1A]),
        .LPF_FILE      (LPF_FILE  [STAGE_V1A]),
        .SHELF_FILE    (SHELF_FILE[STAGE_V1A]),
        .HPF_FILE      (HPF_FILE  [STAGE_V1A]),
        // Gap 2 — V1A is the only bypassed-cathode stage in the 2203, so
        // its shelf moves pre-LUT and shelf_v1a.mem is the pre-emphasis form.
        .SHELF_PRE_LUT  (1'b1),
        // Gap 1 — V1A's output coupling HPF is the only preamp HPF currently
        // shipping non-zero γ_atk / γ_rel (see scripts/gen_triode_lut.py
        // BIAS_TRACKER_STAGES).  Drop SHIFT_BIAS to 3 so the per-stage env
        // subtract is ~6 % at full drive — matching the probe_attack baseline
        // (env_peak ≈ 5.87 % cascade-total) instead of the ~1.5 % the default
        // SHIFT_BIAS=5 would give for a single tracking stage.
        .HPF_SHIFT_BIAS (3)
    ) u_v1a (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (x_in),
        .x_valid (x_valid),
        .y_out   (v1a_y),
        .y_valid (v1a_v)
    );

    // Bright pad #1 — 470 kΩ series + 470 pF "Treble Peak" cap feeding the
    // top of the Preamp Volume pot.  −6 dB at DC, flat above ~1.4 kHz.
    cathode_shelf #(
        .COEFF_FILE ("bright_pad_pre_pot.mem")
    ) u_bright_pad_pre_pot (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (v1a_y),
        .x_valid (v1a_v),
        .y_out   (v1a_post_pad),
        .y_valid (v1a_post_pad_v)
    );

    // Preamp Volume / Gain pot — log-taper attenuator after the bright pad.
    level_ctrl #(
        .COEF_FILE ("log_taper_256.mem")
    ) u_gain (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (v1a_post_pad),
        .x_valid (v1a_post_pad_v),
        .pot_pos (gain_pot_pos),
        .y_out   (v1a_post_gain),
        .y_valid (v1a_post_gain_v)
    );

    gain_stage #(
        .STAGE      (STAGE_V1B),
        .LUT_FILE   (LUT_FILE  [STAGE_V1B]),
        .TAN_FILE   (TAN_FILE  [STAGE_V1B]),
        .LPF_FILE   (LPF_FILE  [STAGE_V1B]),
        .SHELF_FILE (SHELF_FILE[STAGE_V1B]),
        .HPF_FILE   (HPF_FILE  [STAGE_V1B])
    ) u_v1b (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (v1a_post_gain),
        .x_valid (v1a_post_gain_v),
        .y_out   (v1b_y),
        .y_valid (v1b_v)
    );

    // Bright pad #2 — same 470 kΩ + 470 pF + 470 kΩ network feeding V2A's
    // grid leak.  Sits between the cold-clipper plate and V2A's grid.
    cathode_shelf #(
        .COEFF_FILE ("bright_pad_v1b_v2a.mem")
    ) u_bright_pad_v1b_v2a (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (v1b_y),
        .x_valid (v1b_v),
        .y_out   (v1b_post_pad),
        .y_valid (v1b_post_pad_v)
    );

    gain_stage #(
        .STAGE      (STAGE_V2A),
        .LUT_FILE   (LUT_FILE  [STAGE_V2A]),
        .TAN_FILE   (TAN_FILE  [STAGE_V2A]),
        .LPF_FILE   (LPF_FILE  [STAGE_V2A]),
        .SHELF_FILE (SHELF_FILE[STAGE_V2A]),
        .HPF_FILE   (HPF_FILE  [STAGE_V2A]),
        // Gap 1 — V2A→V2B coupling tracker.  V2B is a cathode follower
        // whose grid does conduct on positive peaks but with smaller
        // drive than V1B's grid sees the cold-clipper output.  Use
        // SHIFT_BIAS=4 (~3 % subtract) — half-depth of V1A's =3.
        .HPF_SHIFT_BIAS (4)
    ) u_v2a (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (v1b_post_pad),
        .x_valid (v1b_post_pad_v),
        .y_out   (v2a_y),
        .y_valid (v2a_v)
    );

    gain_stage #(
        .STAGE      (STAGE_V2B),
        .LUT_FILE   (LUT_FILE  [STAGE_V2B]),
        .TAN_FILE   (TAN_FILE  [STAGE_V2B]),
        .LPF_FILE   (LPF_FILE  [STAGE_V2B]),
        .SHELF_FILE (SHELF_FILE[STAGE_V2B]),
        .HPF_FILE   (HPF_FILE  [STAGE_V2B])
    ) u_v2b (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (v2a_y),
        .x_valid (v2a_v),
        .y_out   (y_out),
        .y_valid (y_valid)
    );

    assign v1a_tap = v1a_y;
    assign v1b_tap = v1b_y;
    assign v2a_tap = v2a_y;

endmodule
