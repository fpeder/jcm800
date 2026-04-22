
// =====================================================================
// upsample_16x — cascade of four paced halfband_up2 stages (48→768 kHz)
//
//   Stage order and rate progression:
//     hb1:  48 →  96 kHz   (2083 sys_clk cyc per input @ 100 MHz)
//     hb2:  96 → 192 kHz   (1041 cyc)
//     hb3: 192 → 384 kHz   ( 520 cyc)
//     hb4: 384 → 768 kHz   ( 260 cyc)
//
//   Each stage paces its own outputs to halve the input interval, so
//   the inter-stage stream is evenly spaced and no downstream module
//   sees the bursty raw polyphase output of a halfband.  The final
//   768 kHz stream feeds jcm800 at one x_valid per ~130 sys_clk cycles,
//   still above the ~30-cycle preamp FSM window.
// =====================================================================
module upsample_16x
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

    // Cycle budget per input at each stage (at 100 MHz sys_clk).
    // These numbers assume 48 kHz ADC rate; other rates need regeneration.
    localparam int HB1_IN_INTERVAL = 2083;   // 100e6 / 48e3
    localparam int HB2_IN_INTERVAL = 1041;   // 100e6 / 96e3
    localparam int HB3_IN_INTERVAL =  520;   // 100e6 / 192e3
    localparam int HB4_IN_INTERVAL =  260;   // 100e6 / 384e3

    sample_t s1_out,   s2_out,   s3_out;
    logic    s1_valid, s2_valid, s3_valid;

    halfband_up2 #(
        .K0              (HB1_K0),
        .K1              (HB1_K1),
        .IN_INTERVAL_CYC (HB1_IN_INTERVAL),
        .E0_FILE         (HB1_E0_FILE),
        .E1_FILE         (HB1_E1_FILE)
    ) u_hb1 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (in_sample),
        .in_valid   (in_valid),
        .out_sample (s1_out),
        .out_valid  (s1_valid)
    );

    halfband_up2 #(
        .K0              (HB2_K0),
        .K1              (HB2_K1),
        .IN_INTERVAL_CYC (HB2_IN_INTERVAL),
        .E0_FILE         (HB2_E0_FILE),
        .E1_FILE         (HB2_E1_FILE)
    ) u_hb2 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (s1_out),
        .in_valid   (s1_valid),
        .out_sample (s2_out),
        .out_valid  (s2_valid)
    );

    halfband_up2 #(
        .K0              (HB3_K0),
        .K1              (HB3_K1),
        .IN_INTERVAL_CYC (HB3_IN_INTERVAL),
        .E0_FILE         (HB3_E0_FILE),
        .E1_FILE         (HB3_E1_FILE)
    ) u_hb3 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (s2_out),
        .in_valid   (s2_valid),
        .out_sample (s3_out),
        .out_valid  (s3_valid)
    );

    halfband_up2 #(
        .K0              (HB4_K0),
        .K1              (HB4_K1),
        .IN_INTERVAL_CYC (HB4_IN_INTERVAL),
        .E0_FILE         (HB4_E0_FILE),
        .E1_FILE         (HB4_E1_FILE)
    ) u_hb4 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (s3_out),
        .in_valid   (s3_valid),
        .out_sample (out_sample),
        .out_valid  (out_valid)
    );

endmodule
