
// =====================================================================
// ht_supply — main HT (B+) rail sag
//
//   Models the JCM800 2203's plate supply: rectifier + choke + reservoir
//   cap, lumped into a single R_HT·C_HT leaky integrator driven by the
//   push-pull plate-current sum |Ip_a|+|Ip_b|.  Under sustained drive the
//   reservoir cap can't refill fast enough through the choke/rectifier
//   DCR, so the rail droops and the tubes lose peak swing — the "bloom"
//   / compression character that a static-B+ model misses.
//
//   Discrete update at the oversampled rate fs = 768 kHz:
//
//       Δ_up = α_HT · (NOM − ht_state)         // recovery toward B+
//       Δ_dn = β_HT · ip_sum                   // drain from both plates
//       ht_state[n+1] = ht_state[n] + Δ_up − Δ_dn
//
//   α_HT and β_HT are Q1.31 coefficients baked by
//   scripts/gen_pentode_lut.py:ht_supply_coeffs (HT_ALPHA_Q1_31 /
//   HT_BETA_Q1_31 in jcm800_power_pkg.sv).
//
//   State is kept as a 32-bit signed Q2.30 word (nom = 0x4000_0000 =
//   1.0) so the tiny α and β updates don't get lost in a narrower
//   accumulator.  The output is the top 16 bits in Q2.14 format
//   (nom = 0x4000), good enough for the slow-envelope signal that
//   power_amp.sv combines with vg2_ratio before feeding the pentode
//   output scale.
//
//   Pipeline structure mirrors screen_supply.sv exactly — same 3-cycle
//   latency, same DSP-friendly multiply registering, same reset value.
//   The integrator only advances once per 768 kHz oversampled sample
//   (≈ 130 sys_clk cycles), so the 3-cycle pipeline is invisible to
//   the slow-envelope response.
// =====================================================================
module ht_supply
    import jcm800_pkg::*;
    import jcm800_power_pkg::*;
(
    input  logic                clk,
    input  logic                rst_n,

    // Sum of per-tube |ip| estimates (Q1.23 signed, always ≥ 0 in
    // practice — power_amp.sv computes it from the two pentodes' ip_out
    // ports via an absolute-value + add).  Width is 25 bits so two Q1.23
    // values sum without overflow before entering this module.
    input  logic signed [24:0]  ip_sum_q1_23,
    input  logic                ip_sum_valid,

    // Slowly-varying HT ratio, Q2.14 signed (≥ 0 in practice).
    //   nom (B+ = B+_nom) → 16'h4000 (= 1.0)
    //   full sag          → ~16'h3999 (≈ 0.9) at worst with default tuning
    output logic signed [15:0]  ht_ratio_q2_14
);

    // --------------------------------------------------------------
    // State: Q2.30 signed (1.0 = 0x4000_0000).  Reset to nominal.
    // --------------------------------------------------------------
    logic signed [31:0] state_q2_30;

    localparam logic signed [31:0] STATE_NOM_Q2_30 = 32'sh40000000;

    // --------------------------------------------------------------
    // P1 — latch inputs and pre-compute err = NOM − state.
    // --------------------------------------------------------------
    logic signed [31:0] err_q;
    logic signed [31:0] ip_q2_30_q;
    logic               valid_d1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            err_q      <= '0;
            ip_q2_30_q <= '0;
            valid_d1   <= 1'b0;
        end else begin
            err_q      <= STATE_NOM_Q2_30 - state_q2_30;
            // 25-bit Q1.23 signed << 7 → 32-bit Q2.30 signed.
            ip_q2_30_q <= {ip_sum_q1_23, 7'b0};
            valid_d1   <= ip_sum_valid;
        end
    end

    // --------------------------------------------------------------
    // P2 — two DSP multiplies (Q1.31 · Q2.30 → Q3.61).
    // --------------------------------------------------------------
    logic signed [63:0] prod_up_q;
    logic signed [63:0] prod_dn_q;
    logic               valid_d2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod_up_q <= '0;
            prod_dn_q <= '0;
            valid_d2  <= 1'b0;
        end else begin
            prod_up_q <= $signed(HT_ALPHA_Q1_31) * err_q;
            prod_dn_q <= $signed(HT_BETA_Q1_31)  * ip_q2_30_q;
            valid_d2  <= valid_d1;
        end
    end

    // --------------------------------------------------------------
    // P3 — shift back to Q2.30 and apply the state update.
    // --------------------------------------------------------------
    logic signed [31:0] delta_up_c;
    logic signed [31:0] delta_dn_c;

    always_comb begin
        delta_up_c = prod_up_q >>> 31;
        delta_dn_c = prod_dn_q >>> 31;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q2_30 <= STATE_NOM_Q2_30;
        end else if (valid_d2) begin
            state_q2_30 <= state_q2_30 + delta_up_c - delta_dn_c;
        end
    end

    // --------------------------------------------------------------
    // Output: Q2.30 → Q2.14 via top-16-bit slice.
    // --------------------------------------------------------------
    assign ht_ratio_q2_14 = state_q2_30[31:16];

endmodule
