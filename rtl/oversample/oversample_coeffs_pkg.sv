//===========================================================================
// oversample_coeffs_pkg — AUTO-GENERATED; DO NOT EDIT BY HAND.
//   Produced by: scripts/gen_oversample_coeffs.py
//   Timestamp:   2026-04-17 14:56:04
//
//   Polyphase halfband coefficient lengths and .mem filenames for the 16×
//   minimum-phase oversampling cascade.  Regenerate via:
//     python scripts/gen_oversample_coeffs.py
//===========================================================================
package oversample_coeffs_pkg;

    // Fixed-point widths shared across all stages
    localparam int COEF_W = 18;
    localparam int COEF_Q = 17;

    // HB1: 48→96 kHz  (passband 20 kHz, achieved atten 121.3 dB)
    localparam int    HB1_N        = 209;
    localparam int    HB1_K0       = 105;   // even-index branch length
    localparam int    HB1_K1       = 104;   // odd-index branch length
    localparam string HB1_E0_FILE  = "hb1_e0.mem";
    localparam string HB1_E1_FILE  = "hb1_e1.mem";

    // HB2: 96→192 kHz  (passband 20 kHz, achieved atten 100.1 dB)
    localparam int    HB2_N        = 51;
    localparam int    HB2_K0       = 26;   // even-index branch length
    localparam int    HB2_K1       = 25;   // odd-index branch length
    localparam string HB2_E0_FILE  = "hb2_e0.mem";
    localparam string HB2_E1_FILE  = "hb2_e1.mem";

    // HB3: 192→384 kHz  (passband 20 kHz, achieved atten 99.6 dB)
    localparam int    HB3_N        = 39;
    localparam int    HB3_K0       = 20;   // even-index branch length
    localparam int    HB3_K1       = 19;   // odd-index branch length
    localparam string HB3_E0_FILE  = "hb3_e0.mem";
    localparam string HB3_E1_FILE  = "hb3_e1.mem";

    // HB4: 384→768 kHz  (passband 20 kHz, achieved atten 99.9 dB)
    localparam int    HB4_N        = 35;
    localparam int    HB4_K0       = 18;   // even-index branch length
    localparam int    HB4_K1       = 17;   // odd-index branch length
    localparam string HB4_E0_FILE  = "hb4_e0.mem";
    localparam string HB4_E1_FILE  = "hb4_e1.mem";

endpackage
