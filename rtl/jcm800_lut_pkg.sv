//===========================================================================
// jcm800_lut_pkg — AUTO-GENERATED; DO NOT EDIT BY HAND.
//   Produced by: scripts/gen_triode_lut.py
//   Timestamp:   2026-04-24 17:08:39
//
//   Per-stage gain constants + .mem filenames for the redesigned gain_stage
//   (LUT + PCHIP + ×G + plate LPF + cathode shelf + coupling HPF).
//===========================================================================
package jcm800_lut_pkg;

    localparam int STAGE_COUNT = 4;

    typedef enum logic [1:0] {
        STAGE_V1A = 2'd0,
        STAGE_V1B = 2'd1,
        STAGE_V2A = 2'd2,
        STAGE_V2B = 2'd3
    } stage_id_t;

    // Signed Q4.20 per-stage gain.  RTL op (Q1.23 in, Q1.23 out, convergent round):
    //   prod52 = $signed(lut_q23) * $signed(G_STAGE_Q4_20[stage])    // Q5.43
    //   y_q23  = convergent_round(prod52, 20)                        // → Q1.23
    // G includes the tube phase inversion sign (CC negative, CF ~+0.95).
    localparam int                 SHIFT_G = 20;
    localparam logic signed [31:0] G_STAGE_Q4_20 [STAGE_COUNT] = '{
        32'hFFF00000,   // STAGE_V1A  G=-1.0000  (CC)
        32'hFFF00000,   // STAGE_V1B  G=-1.0000  (CC)
        32'hFFF00000,   // STAGE_V2A  G=-1.0000  (CC)
        32'h00100000    // STAGE_V2B  G=+1.0000  (CF)
    };

    // Single top-level input sensitivity default (Q16.16, signed 32-bit).
    // Applied ONCE at the ADC boundary (before V1A), not per-stage.  Intended
    // to become a runtime register ("input drive" knob).  RTL op:
    //   x_q23 = saturate_s24( ($signed(adc_q23) * INPUT_SCALE_Q16_16) >>> 16 )
    localparam int                 SHIFT_IN = 16;
    localparam logic signed [31:0] INPUT_SCALE_DEFAULT_Q16_16 = 32'h00002148;

    // ─── DEPRECATED — preserved so the legacy preamp.sv still elaborates. ───
    // The redesigned gain_stage puts all per-stage gain in G_STAGE_Q4_20 and
    // all input sensitivity in INPUT_SCALE_DEFAULT_Q16_16.  This array is
    // an identity (1.0×) placeholder so the old cascade's muxed pre-LUT
    // multiplier compiles; it will disappear when preamp.sv is rewired.
    localparam logic signed [31:0] INPUT_SCALE_Q16_16 [STAGE_COUNT] = '{
        32'h00010000,   // STAGE_V1A  (deprecated; use G_STAGE_Q4_20 + INPUT_SCALE_DEFAULT)
        32'h00010000,   // STAGE_V1B
        32'h00010000,   // STAGE_V2A
        32'h00010000    // STAGE_V2B
    };

    // LUT + companion tangent-table filenames (4097 × 24-bit Q1.23).
    localparam string LUT_FILE   [STAGE_COUNT] = '{
        "v1a_lut.mem",
        "v1b_lut.mem",
        "v2a_lut.mem",
        "v2b_lut.mem"
    };
    localparam string TAN_FILE   [STAGE_COUNT] = '{
        "v1a_tan.mem",
        "v1b_tan.mem",
        "v2a_tan.mem",
        "v2b_tan.mem"
    };

    // Plate LPF coefficient files (1 × 32-bit Q1.31 β).
    localparam string LPF_FILE   [STAGE_COUNT] = '{
        "lpf_v1a.mem",
        "lpf_v1b.mem",
        "lpf_v2a.mem",
        "lpf_v2b.mem"
    };

    // Cathode shelf biquad coefficient files (3 × 32-bit Q3.29: b0, b1, a1).
    localparam string SHELF_FILE [STAGE_COUNT] = '{
        "shelf_v1a.mem",
        "shelf_v1b.mem",
        "shelf_v2a.mem",
        "shelf_v2b.mem"
    };

    // Coupling HPF coefficient files (1 × 32-bit Q1.31 α, τ ≈ 10 ms).
    localparam string HPF_FILE   [STAGE_COUNT] = '{
        "hpf_v1a_out.mem",
        "hpf_v1b_out.mem",
        "hpf_v2a_out.mem",
        "hpf_v2b_out.mem"
    };

    // ------------------------------------------------------------------
    // Phase-inverter (V3 long-tailed pair) constants — Gap 3 of the
    // realism plan.  A-arm is the signal side (V3A, Rp_a < Rp_b for
    // balance); B-arm encodes V3B's plate response as a function of V3A
    // drive (shared-tail coupling baked in, not input-negation).  Both
    // arms' LUTs are indexed by V3A's grid voltage on the SAME x axis.
    // ------------------------------------------------------------------
    // DC quiescence: Vk_q = 3.075 V
    //   Ip_a_q = 0.158 mA  Vak_a_q = 307.07 V
    //   Ip_b_q = 0.150 mA  Vak_b_q = 305.01 V
    //   A: |span|=62.31 V  |Gss|=11.13  dV=±6.07 V
    //   B: |span|=26.39 V  |Gss|=9.32  dV=±6.07 V
    // LUT_B has its contents pre-inverted AND G_PI_B carries the opposite
    // sign of G_PI_A — the double sign-flip delivers the anti-phase drive
    // the PI contract requires.  The historical "both G_PI values land
    // at −1.0" convention was a bug: `G_stage = sign(slope_at_zero)` of the
    // flipped LUT coincidentally matched the A-arm's G sign, cancelling
    // the flip and producing in-phase outputs.
    localparam logic signed [31:0] G_PI_A_Q4_20 = 32'hFFF00000;   // -1.0000
    localparam logic signed [31:0] G_PI_B_Q4_20 = 32'h00100000;   // +1.0000

    localparam string PI_A_LUT_FILE   = "pi_a_lut.mem";
    localparam string PI_A_TAN_FILE   = "pi_a_tan.mem";
    localparam string PI_A_LPF_FILE   = "lpf_pi_a.mem";
    localparam string PI_A_SHELF_FILE = "shelf_pi_a.mem";
    localparam string PI_A_HPF_FILE   = "hpf_pi_a_out.mem";

    localparam string PI_B_LUT_FILE   = "pi_b_lut.mem";
    localparam string PI_B_TAN_FILE   = "pi_b_tan.mem";
    localparam string PI_B_LPF_FILE   = "lpf_pi_b.mem";
    localparam string PI_B_SHELF_FILE = "shelf_pi_b.mem";
    localparam string PI_B_HPF_FILE   = "hpf_pi_b_out.mem";

endpackage
