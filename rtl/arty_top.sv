
// =====================================================================
// arty_top — board wrapper for the Arty A7-100T
//
//   Thin structural layer: instantiates pmod_i2s2 (audio path), the
//   jcm800 DSP chain, the UART pot interface, and the minimal UDP
//   audio-sample streamer over MII Ethernet. Wires Pmod JA pins
//   straight through and drives the status LEDs.
//
//   Ethernet notes (Arty A7 Rev. E, DP83848J):
//     - NO crystal on the PHY; FPGA pin G18 *is* the PHY CLKIN.
//       eth_ref_clk must be a 25 MHz clock for the PHY to operate.
//     - MII mode is the board strap default; no MDIO init required.
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

    // Ethernet MII TX-only (DP83848J on-board PHY)
    input  logic       eth_tx_clk,
    output logic       eth_ref_clk,
    output logic [3:0] eth_txd,
    output logic       eth_tx_en,
    output logic       eth_rst_n,

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
        .pi_neg_tap       (),
        .pi_neg_valid     ()
    );

    // ---------------------------------------------------------------
    // UDP audio-sample streamer (MII TX-only)
    //   Dest: 192.168.1.255 : 0xBEEF  (broadcast, no ARP)
    //   Src:  192.168.1.10  : 0xBEEF  (MAC 02:00:00:00:00:01)
    //   Payload: 64 × 24-bit big-endian samples per packet (~750 pps)
    // ---------------------------------------------------------------
    udp_audio_tx u_udp_tx (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_in    (y_out),
        .sample_valid (y_valid),
        .eth_tx_clk   (eth_tx_clk),
        .eth_txd      (eth_txd),
        .eth_tx_en    (eth_tx_en),
        .eth_rst_n    (eth_rst_n)
    );

    // ---------------------------------------------------------------
    // 25 MHz reference clock for the DP83848J PHY.
    //   Divide-by-4 of the 100 MHz system clock. The Arty A7 has NO
    //   crystal on the PHY; this pin *is* the PHY's CLKIN — without it
    //   the PHY can't operate at all.
    // ---------------------------------------------------------------
    logic [1:0] eth_ref_div;
    always_ff @(posedge clk) eth_ref_div <= eth_ref_div + 2'd1;
    assign eth_ref_clk = eth_ref_div[1];

    // ---------------------------------------------------------------
    // Status LEDs
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
