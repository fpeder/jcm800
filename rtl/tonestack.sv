
// =====================================================================
// tonestack — Marshall TMB tone stack, circuit-accurate factorisation
//
//   Sits between u_preamp (post-V2B cathode follower) and u_master in
//   the 2203 signal chain.  Realises the FMV transfer function H(s)
//   derived by nodal analysis on the 6-node Marshall stack (470pF
//   treble, 0.022 µF bass, 0.022 µF mid, 33 kΩ slope, 250 kΩ linear
//   treble pot, 1 MΩ audio bass pot, 25 kΩ linear mid pot — see
//   scripts/gen_tonestack_lut.py for the symbolic derivation).
//
//   Each pot triple's analog H(s) is bilinear-transformed at fs = 768 kHz
//   to a 3rd-order digital H(z), then factored as
//
//       H(z) = (b0 + b1·z⁻¹ + b2·z⁻²)/(1 + a1·z⁻¹ + a2·z⁻²)
//            × (b0' + b1'·z⁻¹)/(1 + a1'·z⁻¹)
//
//   so the runtime pipeline is one biquad_2nd_coef_ports cascaded with
//   one biquad_coef_ports.
//
//   Signal flow (sample_t Q1.23 s24 at 768 kHz):
//
//     x_in ──► u_biquad_2nd  ──► u_biquad_1st  ──► y_out
//                ▲                  ▲
//                │  5 coefs         │  3 coefs   (registered once
//                │                  │            per x_valid)
//                │                  │
//         ┌──────┴──────────────────┴──────┐
//         │ coef_mem[grid_addr] — 8 × Q3.29│
//         │ addressed by {bass[7:5],       │
//         │               mid [7:5],       │
//         │               treble[7:5]}     │
//         └────────────────────────────────┘
//
//   Pot grid
//   ────────
//     8 × 8 × 8 = 512 pot triples.  RTL takes the top 3 bits of each
//     8-bit pot position; bass uses an audio (A10) taper at gen time so
//     idx 0..7 is non-linearly spaced in physical resistance.  Treble
//     and mid are linear taper.
//
//   Coefficient ROM layout
//   ──────────────────────
//     512 entries × 8 × 32-bit Q3.29 signed.  Per-entry layout (offsets
//     0..7):
//         0..2:  biquad b0, b1, b2
//         3..4:  biquad a1, a2
//         5..6:  1st-order b0, b1
//         7   :  1st-order a1
//
//   Pipeline
//   ────────
//     Coefficients are latched once on x_valid and held for the duration
//     of the 9-cycle (5+4) pipeline.  Total latency ≈ 9 sys_clk from
//     x_valid to y_valid.  Trivial against the ~130 cycle / sample
//     budget at fs=768kHz / sys_clk=100MHz.
// =====================================================================
module tonestack
    import jcm800_pkg::*;
#(
    parameter string COEF_FILE = "tonestack_coefs.mem"
)(
    input  logic        clk,
    input  logic        rst_n,
    input  sample_t     x_in,
    input  logic        x_valid,
    input  logic [7:0]  bass_pot_pos,
    input  logic [7:0]  mid_pot_pos,
    input  logic [7:0]  treble_pot_pos,
    output sample_t     y_out,
    output logic        y_valid
);

    localparam int GRID      = 8;
    localparam int N_ENTRIES = GRID * GRID * GRID;   // 512
    localparam int N_COEFS   = 8;

    logic [31:0] coef_mem [0:N_ENTRIES-1][0:N_COEFS-1];
    initial $readmemh(COEF_FILE, coef_mem);

    // ----------------------------------------------------------------
    // Grid-address lookup
    // ----------------------------------------------------------------
    wire [2:0] bass_idx   = bass_pot_pos  [7:5];
    wire [2:0] mid_idx    = mid_pot_pos   [7:5];
    wire [2:0] treble_idx = treble_pot_pos[7:5];
    wire [8:0] grid_addr  = {bass_idx, mid_idx, treble_idx};

    // ----------------------------------------------------------------
    // Coefficient registers — latched on x_valid, held for the pipeline
    // ----------------------------------------------------------------
    logic signed [31:0]  biq_b0, biq_b1, biq_b2, biq_a1, biq_a2;
    logic signed [31:0]  fst_b0, fst_b1, fst_a1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            biq_b0 <= '0; biq_b1 <= '0; biq_b2 <= '0;
            biq_a1 <= '0; biq_a2 <= '0;
            fst_b0 <= '0; fst_b1 <= '0; fst_a1 <= '0;
        end else if (x_valid) begin
            biq_b0 <= $signed(coef_mem[grid_addr][0]);
            biq_b1 <= $signed(coef_mem[grid_addr][1]);
            biq_b2 <= $signed(coef_mem[grid_addr][2]);
            biq_a1 <= $signed(coef_mem[grid_addr][3]);
            biq_a2 <= $signed(coef_mem[grid_addr][4]);
            fst_b0 <= $signed(coef_mem[grid_addr][5]);
            fst_b1 <= $signed(coef_mem[grid_addr][6]);
            fst_a1 <= $signed(coef_mem[grid_addr][7]);
        end
    end

    // ----------------------------------------------------------------
    // Cascade: 2nd-order biquad → 1st-order biquad
    // ----------------------------------------------------------------
    sample_t y_biq;
    logic    y_biq_valid;

    biquad_2nd_coef_ports u_biquad_2nd (
        .clk     (clk),
        .rst_n   (rst_n),
        .b0_q29  (biq_b0),
        .b1_q29  (biq_b1),
        .b2_q29  (biq_b2),
        .a1_q29  (biq_a1),
        .a2_q29  (biq_a2),
        .x_in    (x_in),
        .x_valid (x_valid),
        .y_out   (y_biq),
        .y_valid (y_biq_valid)
    );

    biquad_coef_ports u_biquad_1st (
        .clk     (clk),
        .rst_n   (rst_n),
        .b0_q29  (fst_b0),
        .b1_q29  (fst_b1),
        .a1_q29  (fst_a1),
        .x_in    (y_biq),
        .x_valid (y_biq_valid),
        .y_out   (y_out),
        .y_valid (y_valid)
    );

endmodule
