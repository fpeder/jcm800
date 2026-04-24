
// =====================================================================
// jcm800 — Marshall JCM800 2203 full DSP chain (16× oversampled)
//
//   Now includes the push-pull EL34 power stage, output transformer
//   (with core-sat soft-clip), and closed NFB loop with Presence:
//
//     x_in 48k ──► upsample_16x ──► preamp(+Gain)
//                                   │
//                                   ▼
//                               tonestack (B/M/T)
//                                   │
//                                   ▼
//                                Master vol
//                                   │
//                                   ▼
//                          phase_inverter ◄── nfb_inject
//                           │       │         (from nfb_network,
//                           │       │          1-sample delayed)
//                       y_pos     y_neg
//                           │       │
//                           ▼       ▼
//                         power_amp (EL34 push-pull + OT)
//                                   │
//                                   ▼
//                                 v_sec ───► downsample_16x ──► ir_cab ──► y_out 48k
//                                   │
//                                   └─► speaker_load ─► nfb_network ─► nfb_inject
//                                       (reactive Zs(f) shaping on the
//                                        NFB return only — the DAC path
//                                        keeps tapping raw v_sec so the
//                                        ir_cab isn't double-counted)
//
//   All DSP inside the oversample boundary runs at 768 kHz.  The
//   secondary voltage v_sec feeds both the DAC path (through the 768k
//   → 48k decimator) and the NFB return path in parallel.
//
//   Pots:
//     gain_pot_pos      (Preamp Volume) — inside preamp, V1A → V1B.
//     master_pot_pos    (Master Volume) — between tonestack and PI.
//     presence_pot_pos  (Presence)      — attenuates NFB shelf depth.
//     bass_pot_pos      (Bass)          — tonestack section 1 gain.
//     mid_pot_pos       (Mid)           — tonestack section 2 gain.
//     treble_pot_pos    (Treble)        — tonestack section 3 gain.
// =====================================================================
module jcm800
    import jcm800_pkg::*;
(
    input  logic       clk,
    input  logic       rst_n,
    input  sample_t    x_in,
    input  logic       x_valid,
    output sample_t    y_out,
    output logic       y_valid,

    input  logic [7:0] gain_pot_pos,
    input  logic [7:0] master_pot_pos,
    input  logic [7:0] presence_pot_pos,
    input  logic [7:0] bass_pot_pos,
    input  logic [7:0] mid_pot_pos,
    input  logic [7:0] treble_pot_pos,

    // Anti-phase LTP plate (V3B) at 768 kHz — kept on the top interface
    // for bench/diagnostic taps; already consumed internally by power_amp.
    output sample_t    pi_neg_tap,
    output logic       pi_neg_valid
);

    // -----------------------------------------------------------------
    // Upsampler → Preamp → Tonestack → Master Volume
    // -----------------------------------------------------------------
    sample_t x_os;
    logic    x_os_valid;

    upsample_16x u_up (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (x_in),
        .in_valid   (x_valid),
        .out_sample (x_os),
        .out_valid  (x_os_valid)
    );

    sample_t y_pre;
    logic    y_pre_valid;

    preamp u_preamp (
        .clk          (clk),
        .rst_n        (rst_n),
        .x_in         (x_os),
        .x_valid      (x_os_valid),
        .gain_pot_pos (gain_pot_pos),
        .y_out        (y_pre),
        .y_valid      (y_pre_valid),
        .v1a_tap      (),
        .v1b_tap      (),
        .v2a_tap      ()
    );

    // Tone stack — 3-band shelving EQ between V2B and Master Volume.
    // Coefficients are looked up from an 8^3 pot-triple grid and held
    // for the duration of the 12-cycle biquad cascade; see tonestack.sv
    // and scripts/gen_tonestack_lut.py.
    sample_t y_tone;
    logic    y_tone_valid;

    tonestack #(
        .COEF_FILE ("tonestack_coefs.mem")
    ) u_tonestack (
        .clk            (clk),
        .rst_n          (rst_n),
        .x_in           (y_pre),
        .x_valid        (y_pre_valid),
        .bass_pot_pos   (bass_pot_pos),
        .mid_pot_pos    (mid_pot_pos),
        .treble_pot_pos (treble_pot_pos),
        .y_out          (y_tone),
        .y_valid        (y_tone_valid)
    );

    sample_t y_master;
    logic    y_master_v;

    level_ctrl #(
        .COEF_FILE ("log_taper_256.mem")
    ) u_master (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (y_tone),
        .x_valid (y_tone_valid),
        .pot_pos (master_pot_pos),
        .y_out   (y_master),
        .y_valid (y_master_v)
    );

    // -----------------------------------------------------------------
    // Phase inverter — NFB injected at its shared cathode.  nfb_inject
    // is a held value from nfb_network, sampled combinationally when
    // phase_inverter's own x_valid fires.
    // -----------------------------------------------------------------
    sample_t nfb_inject;
    logic    nfb_inject_valid;

    sample_t y_pos_os;
    sample_t y_neg_os;
    logic    y_pi_valid;

    phase_inverter u_pi (
        .clk        (clk),
        .rst_n      (rst_n),
        .x_in       (y_master),
        .x_valid    (y_master_v),
        .nfb_inject (nfb_inject),
        .y_pos      (y_pos_os),
        .y_neg      (y_neg_os),
        .y_valid    (y_pi_valid)
    );

    assign pi_neg_tap   = y_neg_os;
    assign pi_neg_valid = y_pi_valid;

    // -----------------------------------------------------------------
    // Power amp + OT
    // -----------------------------------------------------------------
    sample_t v_sec_os;
    logic    v_sec_os_valid;

    power_amp u_power (
        .clk         (clk),
        .rst_n       (rst_n),
        .y_pos       (y_pos_os),
        .y_neg       (y_neg_os),
        .y_valid     (y_pi_valid),
        .v_sec       (v_sec_os),
        .v_sec_valid (v_sec_os_valid)
    );

    // -----------------------------------------------------------------
    // Speaker-impedance shaping on the NFB path.  A real 12" guitar
    // speaker's Zs(f) has a resonance peak near 80 Hz and an inductive
    // HF trend; nfb_network otherwise treats v_sec as if the secondary
    // loaded into a flat 8 Ω.  speaker_load is a 2nd-order biquad tuned
    // to Zs(ω)/Z_ideal (see scripts/gen_speaker_zs.py).  We insert it on
    // the NFB return only — the DAC path keeps tapping v_sec_os, because
    // ir_cab already bakes a full miked-cab response in.
    //
    // BYPASS (palm-mute-choke A/B): the biquad is instantiated but its
    // output is not consumed — u_nfb taps v_sec_os directly.  Synthesis
    // will prune u_speaker_load.  Revert by rewiring .v_sec/.v_sec_valid
    // back to v_sec_nfb / v_sec_nfb_valid once the A/B is done.
    // -----------------------------------------------------------------
    sample_t v_sec_nfb;
    logic    v_sec_nfb_valid;

    speaker_load #(
        .COEFF_FILE ("speaker_zs.mem")
    ) u_speaker_load (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (v_sec_os),
        .x_valid (v_sec_os_valid),
        .y_out   (v_sec_nfb),
        .y_valid (v_sec_nfb_valid)
    );

    // Suppress unused-driver warnings while the biquad is bypassed.
    sample_t _unused_v_sec_nfb;
    logic    _unused_v_sec_nfb_valid;
    assign _unused_v_sec_nfb       = v_sec_nfb;
    assign _unused_v_sec_nfb_valid = v_sec_nfb_valid;

    // -----------------------------------------------------------------
    // NFB return (1-sample delay is inside nfb_network)
    // -----------------------------------------------------------------
    nfb_network u_nfb (
        .clk              (clk),
        .rst_n            (rst_n),
        .v_sec            (v_sec_os),
        .v_sec_valid      (v_sec_os_valid),
        .presence_pot_pos (presence_pot_pos),
        .nfb_inject       (nfb_inject),
        .nfb_inject_valid (nfb_inject_valid)
    );

    // nfb_inject_valid is consumed only for observability/assertions —
    // phase_inverter samples nfb_inject combinationally on its own
    // x_valid pulse, and the pipeline delays guarantee the value is
    // stable well before the next sample arrives.
    logic _unused_nfb_inject_valid;
    assign _unused_nfb_inject_valid = nfb_inject_valid;

    // -----------------------------------------------------------------
    // Decimate OT secondary back to 48 kHz, then convolve with the
    // cabinet IR — the speaker-cab coloration that turns the bare
    // transformer-secondary signal into a miked-cab sound.
    // -----------------------------------------------------------------
    sample_t y_dac_raw;
    logic    y_dac_raw_valid;

    downsample_16x u_down (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (v_sec_os),
        .in_valid   (v_sec_os_valid),
        .out_sample (y_dac_raw),
        .out_valid  (y_dac_raw_valid)
    );

    ir_cab u_ir_cab (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_sample  (y_dac_raw),
        .in_valid   (y_dac_raw_valid),
        .out_sample (y_out),
        .out_valid  (y_valid)
    );

endmodule
