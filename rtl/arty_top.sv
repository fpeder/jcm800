
// =====================================================================
// arty_top — board wrapper for the Arty A7-100T
//
//   Thin structural layer: instantiates pmod_i2s2 (audio path) and the
//   jcm800 DSP chain, wires the Pmod JA pins straight through, and
//   drives the status LEDs.
// =====================================================================
module arty_top
    import jcm800_pkg::*;
(
    input  logic clk,            // 100 MHz oscillator
    input  logic rst_btn,        // active-LOW reset button

    // USB-UART bridge RX (FT2232H interface 1, Arty pin A9)
    input  logic uart_rx,

    // Pmod JA top row — DAC
    output logic ja_mclk,
    output logic ja_lrck,
    output logic ja_dac_sclk,
    output logic ja_dac_sdin,

    // Pmod JA bottom row — ADC
    output logic ja_adc_mclk,
    output logic ja_adc_lrck,
    output logic ja_adc_sclk,
    input  logic ja_adc_sdout,

    output logic led_locked,
    output logic led_lrck,
    output logic led_out_valid,
    output logic led_nonzero
);

    logic    locked;
    logic    rst_n;

    sample_t sample_adc;
    logic    sample_valid;
    sample_t y_out;
    logic    y_valid;

    assign rst_n = rst_btn & locked;

    // ---------------------------------------------------------------
    // Audio path
    // ---------------------------------------------------------------
    pmod_i2s2 u_i2s2 (
        .clk          (clk),
        .rst_n        (rst_btn),
        .locked       (locked),

        .ja_mclk      (ja_mclk),
        .ja_lrck      (ja_lrck),
        .ja_dac_sclk  (ja_dac_sclk),
        .ja_dac_sdin  (ja_dac_sdin),
        .ja_adc_mclk  (ja_adc_mclk),
        .ja_adc_lrck  (ja_adc_lrck),
        .ja_adc_sclk  (ja_adc_sclk),
        .ja_adc_sdout (ja_adc_sdout),

        .sample_valid (sample_valid),
        .sample_out   (sample_adc),
        .out_ready    (y_valid),
        .sample_in    (y_out)
    );

    // ---------------------------------------------------------------
    // UART-driven pot register file
    //   Host sends 2-byte commands at 115200 8N1:
    //     [0x01, pot]  → gain_pot_pos
    //     [0x02, pot]  → master_pot_pos
    //     [0x03, pot]  → presence_pot_pos
    //     [0x04, pot]  → bass_pot_pos
    //     [0x05, pot]  → mid_pot_pos
    //     [0x06, pot]  → treble_pot_pos
    //   All six reset defaults are 0x80 so the board boots audible with
    //   presence at stock noon and the tonestack flat.
    // ---------------------------------------------------------------
    logic       byte_valid;
    logic [7:0] byte_data;
    logic [7:0] gain_pot_pos;
    logic [7:0] master_pot_pos;
    logic [7:0] presence_pot_pos;
    logic [7:0] bass_pot_pos;
    logic [7:0] mid_pot_pos;
    logic [7:0] treble_pot_pos;

    uart_rx u_uart_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx         (uart_rx),
        .byte_valid (byte_valid),
        .byte_data  (byte_data)
    );

    uart_pot_regs u_uart_pot_regs (
        .clk              (clk),
        .rst_n            (rst_n),
        .byte_valid       (byte_valid),
        .byte_data        (byte_data),
        .gain_pot_pos     (gain_pot_pos),
        .master_pot_pos   (master_pot_pos),
        .presence_pot_pos (presence_pot_pos),
        .bass_pot_pos     (bass_pot_pos),
        .mid_pot_pos      (mid_pot_pos),
        .treble_pot_pos   (treble_pot_pos)
    );

    jcm800 u_jcm800 (
        .clk              (clk),
        .rst_n            (rst_n),
        .x_in             (sample_adc),
        .x_valid          (sample_valid),
        .gain_pot_pos     (gain_pot_pos),
        .master_pot_pos   (master_pot_pos),
        .presence_pot_pos (presence_pot_pos),
        .bass_pot_pos     (bass_pot_pos),
        .mid_pot_pos      (mid_pot_pos),
        .treble_pot_pos   (treble_pot_pos),
        .y_out            (y_out),
        .y_valid          (y_valid),
        // pi_neg_tap is driven internally by power_amp now; left as a
        // diagnostic port on the top-level interface but unused here.
        .pi_neg_tap       (),
        .pi_neg_valid     ()
    );

    // ---------------------------------------------------------------
    // Status LEDs (LRCK divided down to ~6 Hz so it's visible)
    // ---------------------------------------------------------------
    logic [23:0] blink_cnt;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) blink_cnt <= '0;
        else        blink_cnt <= blink_cnt + 24'd1;

    assign led_locked    = locked;
    assign led_lrck      = blink_cnt[23];
    assign led_out_valid = sample_valid;
    assign led_nonzero   = |sample_adc;

endmodule
