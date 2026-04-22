
// =====================================================================
// pmod_i2s2 — Digilent Pmod I2S2 wrapper (CS5343 ADC + CS4344 DAC)
//
//   Bundles the MMCM (12.288 MHz MCLK), the I2S controller, and the
//   Pmod JA pin fan-out (DAC top row + ADC bottom row share clocks).
//   Exposes a sys_clk-domain sample_t interface for the rest of the
//   design.
// =====================================================================
module pmod_i2s2
    import jcm800_pkg::*;
(
    input  logic    clk,            // 100 MHz system clock
    input  logic    rst_n,          // active-LOW reset (pre-locked)
    output logic    locked,         // MMCM locked status

    // Pmod JA top row — DAC
    output logic    ja_mclk,
    output logic    ja_lrck,
    output logic    ja_dac_sclk,
    output logic    ja_dac_sdin,

    // Pmod JA bottom row — ADC
    output logic    ja_adc_mclk,
    output logic    ja_adc_lrck,
    output logic    ja_adc_sclk,
    input  logic    ja_adc_sdout,

    // Sample interface (sys_clk domain)
    output logic    sample_valid,
    output sample_t sample_out,
    input  logic    out_ready,
    input  sample_t sample_in
);

    logic mclk;
    logic rst_n_int;

    i2s_clkgen u_clkgen (
        .clk    (clk),
        .rst_n  (rst_n),
        .mclk   (mclk),
        .locked (locked)
    );

    assign rst_n_int = rst_n & locked;

    logic lrck;
    logic sclk;
    logic sdout;
    logic mclk_pass;

    i2s_controller u_i2s (
        .clk          (clk),
        .rst_n        (rst_n_int),
        .mclk         (mclk),

        .lrck         (lrck),
        .sclk         (sclk),
        .sdout        (sdout),
        .sdin         (ja_adc_sdout),
        .mclk_out     (mclk_pass),

        .sample_valid (sample_valid),
        .sample_out   (sample_out),
        .out_ready    (out_ready),
        .sample_in    (sample_in)
    );

    // DAC and ADC share mclk/lrck/sclk; sdout drives DAC input.
    assign ja_mclk     = mclk_pass;
    assign ja_adc_mclk = mclk_pass;
    assign ja_lrck     = lrck;
    assign ja_adc_lrck = lrck;
    assign ja_dac_sclk = sclk;
    assign ja_adc_sclk = sclk;
    assign ja_dac_sdin = sdout;

endmodule
