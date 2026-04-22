
// =====================================================================
// level_ctrl — pot-controlled log-taper attenuator
//
//   y[n] = saturate_s24( round( x[n] · coef[pot_pos] , 24 ) )
//
//   coef is a 256-entry unsigned Q0.24 LUT loaded from COEF_FILE (audio
//   taper: −60 dB at pot_pos=0 to 0 dB at pot_pos=255, see
//   scripts/gen_log_taper.py).  Treated as a 25-bit signed value with a
//   leading zero for the signed×unsigned multiply.
//
//   Pipeline (4 cycles, matches the ×G path in gain_stage.sv):
//     S0 — on x_valid: address coef_mem with pot_pos; latch {coef, x}.
//     S1 — prod48 = $signed({1'b0, coef_u24}) · x_s24  (Q0.24 · Q1.23 → Q1.47).
//     S2a — round_conv96(prod, 24) → register rounded 96-bit value.
//     S2b — saturate_s24 → y_out.  Round/saturate are split into two
//           registered stages so the 96-bit add + saturation compare do
//           not have to fit in a single sys_clk period.
//
//   Per-instance latency is fixed and independent of pot_pos, so using
//   two of these in series (Gain and Master) adds a deterministic
//   6-cycle pipeline — trivial against the ~130 sys_clk/sample budget
//   at 768 kHz oversampled.
// =====================================================================
module level_ctrl
    import jcm800_pkg::*;
#(
    parameter string COEF_FILE = "log_taper_256.mem"
)(
    input  logic         clk,
    input  logic         rst_n,
    input  sample_t      x_in,
    input  logic         x_valid,
    input  logic [7:0]   pot_pos,
    output sample_t      y_out,
    output logic         y_valid
);

    // ----------------------------------------------------------------
    // Coefficient LUT (256 × 24-bit unsigned Q0.24) — distributed ROM
    // ----------------------------------------------------------------
    (* rom_style = "distributed" *)
    logic [23:0] coef_mem [0:255];
    initial $readmemh(COEF_FILE, coef_mem);

    // ----------------------------------------------------------------
    // S0 — register {coef, x} on x_valid
    // ----------------------------------------------------------------
    logic [23:0] s0_coef;
    sample_t     s0_x;
    logic        s0_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_coef  <= '0;
            s0_x     <= '0;
            s0_valid <= 1'b0;
        end else begin
            s0_valid <= x_valid;
            if (x_valid) begin
                s0_coef <= coef_mem[pot_pos];
                s0_x    <= x_in;
            end
        end
    end

    // ----------------------------------------------------------------
    // S1 — signed × unsigned multiply (treat coef as 25-bit signed with
    //      leading zero).  prod ∈ Q1.47 s49, fits in s48 for coef < 2^24.
    // ----------------------------------------------------------------
    wire signed [24:0] coef_s25 = $signed({1'b0, s0_coef});

    logic signed [49:0] prod_c;
    logic signed [49:0] s1_prod;
    logic               s1_valid;

    always_comb prod_c = coef_s25 * s0_x;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_prod  <= '0;
            s1_valid <= 1'b0;
        end else begin
            s1_valid <= s0_valid;
            if (s0_valid) s1_prod <= prod_c;
        end
    end

    // ----------------------------------------------------------------
    // S2a — convergent-round >>> 24.  Register the rounded 96-bit value.
    // ----------------------------------------------------------------
    logic signed [95:0] prod_ext;
    logic signed [95:0] y_q23_96_c;
    logic signed [95:0] y_q23_96_q;
    logic               s2a_valid;

    always_comb begin
        prod_ext   = {{46{s1_prod[49]}}, s1_prod};
        y_q23_96_c = round_conv96(prod_ext, 24);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_q23_96_q <= '0;
            s2a_valid  <= 1'b0;
        end else begin
            s2a_valid <= s1_valid;
            if (s1_valid) y_q23_96_q <= y_q23_96_c;
        end
    end

    // ----------------------------------------------------------------
    // S2b — saturate rounded value to sample_t Q1.23.
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out   <= '0;
            y_valid <= 1'b0;
        end else begin
            y_valid <= s2a_valid;
            if (s2a_valid) y_out <= saturate_s24(y_q23_96_q);
        end
    end

endmodule
