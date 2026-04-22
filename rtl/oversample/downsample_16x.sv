
// =====================================================================
// downsample_16x — cascade of four halfband_down2 stages (768→48 kHz)
//
//   Stage order is the mirror of upsample_16x:
//     hb4: 768 → 384 kHz
//     hb3: 384 → 192 kHz
//     hb2: 192 →  96 kHz
//     hb1:  96 →  48 kHz
//
//   No output pacing needed: each halfband_down2 emits one output per
//   two inputs, so the inter-stage stream is already evenly spaced.
// =====================================================================
module downsample_16x
    import jcm800_pkg::*;
    import oversample_coeffs_pkg::*;
(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t in_sample,
    input  logic    in_valid,
    output sample_t out_sample,
    output logic    out_valid
);

    sample_t s4_out,   s3_out,   s2_out;
    logic    s4_valid, s3_valid, s2_valid;

    halfband_down2 #(
        .K0      (HB4_K0),
        .K1      (HB4_K1),
        .E0_FILE (HB4_E0_FILE),
        .E1_FILE (HB4_E1_FILE)
    ) u_hb4 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (in_sample),
        .in_valid   (in_valid),
        .out_sample (s4_out),
        .out_valid  (s4_valid)
    );

    halfband_down2 #(
        .K0      (HB3_K0),
        .K1      (HB3_K1),
        .E0_FILE (HB3_E0_FILE),
        .E1_FILE (HB3_E1_FILE)
    ) u_hb3 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (s4_out),
        .in_valid   (s4_valid),
        .out_sample (s3_out),
        .out_valid  (s3_valid)
    );

    halfband_down2 #(
        .K0      (HB2_K0),
        .K1      (HB2_K1),
        .E0_FILE (HB2_E0_FILE),
        .E1_FILE (HB2_E1_FILE)
    ) u_hb2 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (s3_out),
        .in_valid   (s3_valid),
        .out_sample (s2_out),
        .out_valid  (s2_valid)
    );

    halfband_down2 #(
        .K0      (HB1_K0),
        .K1      (HB1_K1),
        .E0_FILE (HB1_E0_FILE),
        .E1_FILE (HB1_E1_FILE)
    ) u_hb1 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (s2_out),
        .in_valid   (s2_valid),
        .out_sample (out_sample),
        .out_valid  (out_valid)
    );

endmodule
