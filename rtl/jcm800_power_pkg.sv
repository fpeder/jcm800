//===========================================================================
// jcm800_power_pkg — AUTO-GENERATED; DO NOT EDIT BY HAND.
//   Produced by: scripts/gen_pentode_lut.py
//   Timestamp:   2026-04-20 17:41:25
//
//   Power-stage constants + .mem filenames for the new EL34 push-pull +
//   output transformer + NFB/Presence subtree.  Generated alongside the
//   preamp-side jcm800_lut_pkg; the two packages are independent (no
//   cross-references) so either can be regenerated without touching the
//   other.
//
//   DC operating point (nominal rails B+=460.0 V, Vg2=440.0 V,
//   fixed bias Vg1=-38.0 V):
//     Ip_q  = 35.744 mA        Vak_q = 460.00 V
//     Ig2_q = 5.163 mA        Ik_q  = 40.907 mA
//   EL34 span_abs = 456.386 mA (bias-fold peak-to-peak)
//   Tube |Gss| = 0.400 (baked into LUT slope)
//===========================================================================
package jcm800_power_pkg;

    // ----- EL34 push-pull grid→plate-current LUT (shared for both tubes) ----
    localparam string EL34_LUT_FILE = "el34_lut.mem";
    localparam string EL34_TAN_FILE = "el34_tan.mem";

    // Post-LUT gain (Q4.20).  LUT already bakes span and natural slope
    // into Q1.23; G collapses to the phase sign.  Both tubes use +1.0 —
    // the PI delivers anti-phase grid drives, so Ip_A − Ip_B in the OT
    // gives the correct push-pull difference without LUT inversion.
    localparam int                 SHIFT_G_EL34 = 20;
    localparam logic signed [31:0] G_EL34_Q4_20 = 32'h00100000;

    // ----- Screen-scaling side table (Vg2/Vg2_nom)^Ex − 1 ------------------
    // Emitted for future use; the current pentode_stage.sv implementation
    // applies a linear vg2_ratio multiplier instead of k^Ex for simplicity.
    //   localparam string EG_SCALE_LUT_FILE = "pentode_eg_scale_lut.mem";
    //   localparam string EG_SCALE_TAN_FILE = "pentode_eg_scale_tan.mem";
    //   localparam real   EG_K_MIN          = 0.25;
    //   localparam real   EG_K_MAX          = 1.25;

    // ----- OT core-saturation LUT -----------------------------------------
    localparam string OT_SAT_LUT_FILE = "ot_sat_lut.mem";
    localparam string OT_SAT_TAN_FILE = "ot_sat_tan.mem";

    // ----- Screen supply RC (shared Vg2 node) -----------------------------
    // vg2_ratio[n+1] = vg2_ratio[n] + α·(1 − vg2_ratio[n]) − β·ig2_sum
    // α, β are Q1.31 signed (tiny fractional values).
    localparam logic signed [31:0] SCREEN_ALPHA_Q1_31 = 32'h0000E866;  // α = 2.770390e-05
    localparam logic signed [31:0] SCREEN_BETA_Q1_31  = 32'h00003A19;  // β = 6.925975e-06

    // Ig2 estimate: Ig2_tube ≈ IG2_RATIO_Q1_23 · |Ip_tube| (Q1.23 signed).
    // Derived from Koren Kg1/Kg2 for EL34 ≈ 0.1444.
    localparam logic signed [23:0] IG2_RATIO_Q1_23 = 24'h127D28;

    // ----- Output transformer filter coefficient files --------------------
    localparam string OT_HPF_FILE = "hpf_prim.mem";    // fc ≈ 10.8 Hz (Lp/Raa)
    localparam string OT_LPF_FILE = "lpf_leak.mem";    // fc ≈ 18038 Hz (Lleak/Raa)

    // Push-pull difference halver: i_prim = (Ip_A − Ip_B) · PP_HALF so the
    // Q1.23 range is preserved when the two tubes are driven to opposite rails.
    localparam int                 SHIFT_PP = 20;
    localparam logic signed [31:0] PP_HALF_Q4_20 = 32'h00200000;

    // ----- NFB / Presence --------------------------------------------------
    // v_sec → [1-sample z⁻¹] → ×NFB_Q4_20 → presence shelf → level_ctrl
    //     → injected at V3 shared cathode inside phase_inverter.sv.
    localparam logic signed [31:0] NFB_Q4_20         = 32'h0000B852;
    localparam int                 SHIFT_NFB         = 20;
    localparam string              PRESENCE_SHELF_FILE = "shelf_presence.mem";
    localparam string              PRESENCE_POT_FILE   = "log_taper_256.mem";
    localparam real                PRESENCE_FC_HZ      = 800.00;

endpackage
