
// =====================================================================
// pentode_stage — one EL34 of the push-pull power stage
//
//   Mirrors the LUT + PCHIP datapath of gain_stage.sv, but drops the
//   plate LPF / cathode shelf / coupling HPF blocks (handled in the OT
//   instead) and adds:
//     · a vg2_ratio output scale so that screen sag reduces plate-
//       current swing in real time (simpler proxy for the Koren
//       factorisation Ik(Vgk, k·Vg2) ≈ k·Ik(Vgk, Vg2_nom); the true
//       k^Ex form is available as a side LUT in lut_out/ but not
//       consumed here for simplicity)
//     · an |ip|-proxy screen-current output fed to screen_supply.sv.
//
//   Intra-stage order:
//
//     x_in ──► [Vgk clamp] ──► LUT + PCHIP ──► ×G_EL34 ──► ×vg2_ratio
//                                                      ├──► ip_out
//                                                      └──► |·| · IG2_RATIO ──► ig2_out
//
//   Both tubes A and B share the SAME el34_lut.mem content — the PI
//   upstream delivers anti-phase grid drives (y_pos / y_neg), so the
//   output transformer's Ip_A − Ip_B sum recovers push-pull operation
//   without any LUT content inversion.
//
//   Pipeline:
//     LUT+PCHIP             7 cycles  (S0…S6 + sum reg, identical to gain_stage)
//     round + saturate      2 cycles  (S7a: round reg; S7b: saturate reg)
//     ×G_EL34               3 cycles  (mul → round reg → saturate reg)
//     ×vg2_ratio            3 cycles  (mul → round reg → saturate reg)
//     |·| × IG2_RATIO       3 cycles  (abs+mul → round reg → saturate reg)
//     total ip path        15 cycles
//     total ig2 path       18 cycles  (3 extra for abs+scale)
//
//   Round/saturate in every ×-block is split across two registered stages
//   so the 96-bit round-half-to-even add and the subsequent saturation
//   compare do not have to cross a single sys_clk period.
// =====================================================================
module pentode_stage
    import jcm800_pkg::*;
    import jcm800_power_pkg::*;
#(
    // Default LUT pair is the EL34 Ik curve; overridable so other nonlinear
    // tables (e.g. the OT saturation curve) can reuse the same LUT+PCHIP+×G
    // pipeline without duplicating hardware.  G_OVERRIDE_Q4_20 = 0 keeps the
    // package default (G_EL34 = +1.0); non-zero value lets callers retune.
    parameter string LUT_FILE = EL34_LUT_FILE,
    parameter string TAN_FILE = EL34_TAN_FILE,
    parameter logic signed [31:0] G_OVERRIDE_Q4_20 = 32'sh0
)(
    input  logic                clk,
    input  logic                rst_n,
    input  sample_t             x_in,
    input  logic                x_valid,
    // vg2_ratio is a slowly-varying Q2.14 unsigned scale (nom=1.0=0x4000)
    // delivered by screen_supply.sv.  Treated as signed [15:0] here so the
    // mul can use a signed DSP cell uniformly; values are always ≥0 in
    // practice so no sign-bit issues.
    input  logic signed [15:0]  vg2_ratio_q2_14,

    output sample_t             ip_out,
    output logic                ip_valid,
    output sample_t             ig2_out,
    output logic                ig2_valid
);

    // ================================================================
    // Vgk clamp — reserve 12 LSB of margin so BRAM addr < 4095
    // ================================================================
    sample_t x_clamped;
    always_comb begin
        if      ($signed(x_in) >  $signed(24'sh7FF000)) x_clamped =  24'sh7FF000;
        else if ($signed(x_in) < -$signed(24'sh7FF000)) x_clamped = -24'sh7FF000;
        else                                            x_clamped =  x_in;
    end

    // ================================================================
    // S0 — latch on x_valid
    // ================================================================
    sample_t s0_x;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)        s0_x <= '0;
        else if (x_valid)  s0_x <= x_clamped;
    end

    // ================================================================
    // S2 — bias (offset binary) + addr decode
    // ================================================================
    logic [23:0] biased;
    logic [11:0] addr;
    logic [12:0] addr_p1;
    logic [10:0] t_c;

    assign biased  = s0_x ^ 24'h800000;
    assign addr    = biased[23:12];
    assign addr_p1 = {1'b0, addr} + 13'd1;
    assign t_c     = biased[11:1];

    // ================================================================
    // S3 — dual BRAM reads + t² register
    // ================================================================
    sample_t      s3_y0, s3_y1;
    sample_t      s3_t0, s3_t1;
    logic [10:0]  s3_t;
    logic [21:0]  s3_t2;

    triode_bram #(.INIT_FILE(LUT_FILE)) u_lut (
        .clk    (clk),
        .addr_a ({1'b0, addr}),
        .addr_b (addr_p1),
        .q_a    (s3_y0),
        .q_b    (s3_y1)
    );
    triode_bram #(.INIT_FILE(TAN_FILE)) u_tan (
        .clk    (clk),
        .addr_a ({1'b0, addr}),
        .addr_b (addr_p1),
        .q_a    (s3_t0),
        .q_b    (s3_t1)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_t  <= '0;
            s3_t2 <= '0;
        end else begin
            s3_t  <= t_c;
            s3_t2 <= t_c * t_c;
        end
    end

    // ================================================================
    // S4 — four PCHIP basis polynomials in Q1.22 s26
    // ================================================================
    logic [32:0]        t_t2_full;
    logic [21:0]        t3_c;

    logic signed [25:0] t_q22_s, t2_q22_s, t3_q22_s;
    logic signed [25:0] one_q22;
    logic signed [25:0] h00_c, h10_c, h01_c, h11_c;

    assign t_t2_full = s3_t * s3_t2;
    assign t3_c      = t_t2_full[32:11];

    assign t_q22_s  = $signed({4'b0000, s3_t,  11'b000_0000_0000});
    assign t2_q22_s = $signed({4'b0000, s3_t2});
    assign t3_q22_s = $signed({4'b0000, t3_c});
    assign one_q22  = 26'sd4194304;

    always_comb begin
        h00_c = one_q22 - (t2_q22_s <<< 1) - t2_q22_s + (t3_q22_s <<< 1);
        h01_c =          (t2_q22_s <<< 1) + t2_q22_s - (t3_q22_s <<< 1);
        h10_c = t_q22_s - (t2_q22_s <<< 1) + t3_q22_s;
        h11_c =                              t3_q22_s - t2_q22_s;
    end

    logic signed [25:0] s4_h00, s4_h10, s4_h01, s4_h11;
    sample_t            s4_y0,  s4_y1,  s4_t0,  s4_t1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s4_h00 <= '0; s4_h10 <= '0; s4_h01 <= '0; s4_h11 <= '0;
            s4_y0  <= '0; s4_y1  <= '0; s4_t0  <= '0; s4_t1  <= '0;
        end else begin
            s4_h00 <= h00_c; s4_h10 <= h10_c;
            s4_h01 <= h01_c; s4_h11 <= h11_c;
            s4_y0  <= s3_y0; s4_y1  <= s3_y1;
            s4_t0  <= s3_t0; s4_t1  <= s3_t1;
        end
    end

    // ================================================================
    // S5 — four DSP multiplies
    // ================================================================
    logic signed [49:0] s5_p00, s5_p10, s5_p01, s5_p11;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s5_p00 <= '0; s5_p10 <= '0; s5_p01 <= '0; s5_p11 <= '0;
        end else begin
            s5_p00 <= s4_h00 * s4_y0;
            s5_p10 <= s4_h10 * s4_t0;
            s5_p01 <= s4_h01 * s4_y1;
            s5_p11 <= s4_h11 * s4_t1;
        end
    end

    // ================================================================
    // S6 — sum, then S7 round+sat (matches gain_stage pipeline)
    // ================================================================
    logic signed [51:0] sum52_c;
    logic signed [51:0] s6_sum52;

    always_comb begin
        sum52_c = $signed(s5_p00) + $signed(s5_p10)
                + $signed(s5_p01) + $signed(s5_p11);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) s6_sum52 <= '0;
        else        s6_sum52 <= sum52_c;
    end

    // S7a — round, then register the 96-bit rounded value so the large
    //       add-with-bias and the subsequent saturation compare split
    //       across two sys_clk cycles.  Same split is applied to the ×G,
    //       ×vg2_ratio, and ×IG2 blocks below.
    logic signed [95:0] sum52_ext;
    logic signed [95:0] lut_y_96_c;
    logic signed [95:0] lut_y_96_q;

    always_comb begin
        sum52_ext  = {{44{s6_sum52[51]}}, s6_sum52};
        lut_y_96_c = round_conv96(sum52_ext, 22);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) lut_y_96_q <= '0;
        else        lut_y_96_q <= lut_y_96_c;
    end

    // S7b — saturate to Q1.23.
    sample_t s6_y;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) s6_y <= '0;
        else        s6_y <= saturate_s24(lut_y_96_q);
    end

    // Valid pipeline: 8 stages (S0..S7b).
    logic [7:0] valid_sr;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) valid_sr <= '0;
        else        valid_sr <= {valid_sr[6:0], x_valid};
    end
    logic s6_valid;
    assign s6_valid = valid_sr[7];

    // ================================================================
    // ×G_EL34 (Q4.20 × Q1.23 → Q5.43 → round to Q1.23 → saturate)
    // 2-cycle pipeline identical to gain_stage.
    // ================================================================
    logic signed [31:0] g_el34;
    assign g_el34 = (G_OVERRIDE_Q4_20 != 32'sh0) ? G_OVERRIDE_Q4_20 : G_EL34_Q4_20;

    logic signed [55:0] g_prod_c;
    logic signed [55:0] g0_prod;
    logic               g0_valid;

    always_comb g_prod_c = g_el34 * s6_y;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g0_prod  <= '0;
            g0_valid <= 1'b0;
        end else begin
            g0_valid <= s6_valid;
            if (s6_valid) g0_prod <= g_prod_c;
        end
    end

    logic signed [95:0] g_prod_ext;
    logic signed [95:0] g_q23_96_c;
    logic signed [95:0] g_q23_96_q;
    logic               g_round_valid;

    always_comb begin
        g_prod_ext = {{40{g0_prod[55]}}, g0_prod};
        g_q23_96_c = round_conv96(g_prod_ext, SHIFT_G_EL34);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g_q23_96_q    <= '0;
            g_round_valid <= 1'b0;
        end else begin
            g_round_valid <= g0_valid;
            if (g0_valid) g_q23_96_q <= g_q23_96_c;
        end
    end

    sample_t g_y;
    logic    g_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g_y     <= '0;
            g_valid <= 1'b0;
        end else begin
            g_valid <= g_round_valid;
            if (g_round_valid) g_y <= saturate_s24(g_q23_96_q);
        end
    end

    // ================================================================
    // ×vg2_ratio (Q2.14 × Q1.23 → Q3.37 → round to Q1.23 → saturate)
    // Linear proxy for the Koren screen-factor k^Ex — see module header.
    // vg2_ratio_q2_14 is unsigned-in-practice but carried as signed [15:0]
    // to keep the DSP multiply signed.
    // ================================================================
    logic signed [39:0] s_prod_c;
    logic signed [39:0] s0_prod;
    logic               s0_valid;

    always_comb s_prod_c = vg2_ratio_q2_14 * g_y;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_prod  <= '0;
            s0_valid <= 1'b0;
        end else begin
            s0_valid <= g_valid;
            if (g_valid) s0_prod <= s_prod_c;
        end
    end

    logic signed [95:0] s_prod_ext;
    logic signed [95:0] s_q23_96_c;
    logic signed [95:0] s_q23_96_q;
    logic               s_round_valid;

    always_comb begin
        s_prod_ext = {{56{s0_prod[39]}}, s0_prod};
        s_q23_96_c = round_conv96(s_prod_ext, 14);   // Q2.14 → shift 14
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_q23_96_q    <= '0;
            s_round_valid <= 1'b0;
        end else begin
            s_round_valid <= s0_valid;
            if (s0_valid) s_q23_96_q <= s_q23_96_c;
        end
    end

    sample_t s_y;
    logic    s_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_y     <= '0;
            s_valid <= 1'b0;
        end else begin
            s_valid <= s_round_valid;
            if (s_round_valid) s_y <= saturate_s24(s_q23_96_q);
        end
    end

    assign ip_out   = s_y;
    assign ip_valid = s_valid;

    // ================================================================
    // Ig2 proxy: |ip_out| · IG2_RATIO_Q1_23 → Q1.23 unsigned-positive.
    // 2-cycle pipeline: abs+mul, then round+narrow.
    // ================================================================
    sample_t           ip_abs_c;
    sample_t           ip_abs;
    logic              ip_abs_valid;

    always_comb begin
        ip_abs_c = s_y[23] ? -s_y : s_y;
        // corner case: -2^23 negates to itself; saturate upward
        if (s_y == 24'sh800000) ip_abs_c = 24'sh7FFFFF;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ip_abs       <= '0;
            ip_abs_valid <= 1'b0;
        end else begin
            ip_abs_valid <= s_valid;
            if (s_valid) ip_abs <= ip_abs_c;
        end
    end

    logic signed [23:0] ig2_ratio;
    assign ig2_ratio = IG2_RATIO_Q1_23;

    logic signed [47:0] ig2_prod_c;
    logic signed [47:0] ig2_prod;
    logic               ig2_prod_valid;

    always_comb ig2_prod_c = $signed({1'b0, ip_abs}) * ig2_ratio;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ig2_prod       <= '0;
            ig2_prod_valid <= 1'b0;
        end else begin
            ig2_prod_valid <= ip_abs_valid;
            if (ip_abs_valid) ig2_prod <= ig2_prod_c;
        end
    end

    logic signed [95:0] ig2_prod_ext;
    logic signed [95:0] ig2_q23_96_c;
    logic signed [95:0] ig2_q23_96_q;
    logic               ig2_round_valid;

    always_comb begin
        ig2_prod_ext = {{48{ig2_prod[47]}}, ig2_prod};
        ig2_q23_96_c = round_conv96(ig2_prod_ext, 23);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ig2_q23_96_q    <= '0;
            ig2_round_valid <= 1'b0;
        end else begin
            ig2_round_valid <= ig2_prod_valid;
            if (ig2_prod_valid) ig2_q23_96_q <= ig2_q23_96_c;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ig2_out   <= '0;
            ig2_valid <= 1'b0;
        end else begin
            ig2_valid <= ig2_round_valid;
            if (ig2_round_valid) ig2_out <= saturate_s24(ig2_q23_96_q);
        end
    end

endmodule
