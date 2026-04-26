
// =====================================================================
// gain_stage — one redesigned JCM800 triode stage
//
//   Intra-stage order (spec §7 — non-negotiable for matching SPICE and
//   keeping the noise floor well-behaved across the cascade):
//
//     x_in ──► [Vgk clamp] ──► LUT + PCHIP (f_norm: Vak/Vk bias-folded)
//                                      │
//                                      ▼
//                             × G_STAGE_Q4_20[STAGE]
//                                      │
//                                      ▼
//                                  iir_lpf1        (plate LPF, Miller)
//                                      ▼
//                                cathode_shelf    (first-order biquad)
//                                      ▼
//                                  iir_hpf1       (coupling HPF, τ≈10 ms)
//                                      ▼
//                                    y_out
//
//   Pre-LUT signal is Q1.23 in the LUT's "Vgk-normalised" domain — V1A's
//   input scale lives at the top level, not inside this module.  The LUT
//   output f_norm is bias-folded (zero at quiescence), so silence does
//   not accumulate a DC residue through the cascade.
//
//   The Vgk-clamp keeps the BRAM address inside [0, 4095] with a 12-LSB
//   margin, so the PCHIP cubic cannot extrapolate into the zeroed-
//   tangent saturation shoulder from inside.
//
//   LUT+PCHIP+×G datapath uses a SHARED sequential multiplier.  One
//   signed 32×32 → 64 multiplier is exercised in strict dependency order
//   across the 7 original products (t², t·t², 4× PCHIP, g·y_lut).
//   Products, widths, rounding (`round_conv96`) and saturation are
//   bit-identical to the earlier parallel implementation; only the
//   intra-stage schedule changes.  At 768 kHz the sample interval is
//   ≈130 sys_clk cycles — the 19-cycle sequence fits with 6× headroom.
// =====================================================================
module gain_stage
    import jcm800_pkg::*;
    import jcm800_lut_pkg::*;
#(
    parameter stage_id_t STAGE       = STAGE_V1A,
    parameter string     LUT_FILE    = "v1a_lut.mem",
    parameter string     TAN_FILE    = "v1a_tan.mem",
    parameter string     LPF_FILE    = "lpf_v1a.mem",
    parameter string     SHELF_FILE  = "shelf_v1a.mem",
    parameter string     HPF_FILE    = "hpf_v1a_out.mem",
    // Non-zero override bypasses G_STAGE_Q4_20[STAGE] — for stages the
    // auto-generated jcm800_lut_pkg does not yet know about (e.g. PI_A/PI_B).
    parameter logic signed [31:0] G_OVERRIDE_Q4_20 = 32'sh0,
    // Gap 2 realism fix: for a bypassed-cathode stage (V1A in the 2203),
    // move cathode_shelf PRE-LUT as a pre-emphasis filter so LF content
    // enters the LUT with its correctly-degenerated gain.  The .mem file
    // content must match: SHELF_FILE expects the pre-emphasis form
    // (b0, b1 scaled by 1/(1+gm·Rk)) when this flag is asserted, or the
    // boost-HF form otherwise.  scripts/gen_triode_lut.py emits the
    // correct form based on stage name (V1A is pre-emphasis today).
    parameter bit SHELF_PRE_LUT = 1'b0,
    // Gap 1 — coupling-HPF asymmetric bias-tracker depth.  Right-shift on
    // the env state before subtracting from the HPF output (see iir_hpf1.sv
    // header for the per-value mapping).  Only takes effect when the
    // HPF .mem file ships non-zero γ_atk / γ_rel; otherwise the env state
    // stays at zero and the shift has no audible effect.
    parameter int unsigned HPF_SHIFT_BIAS = 5
)(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t x_in,
    input  logic    x_valid,
    output sample_t y_out,
    output logic    y_valid
);

    // ================================================================
    // Vgk clamp — reserve 12 LSB of margin so BRAM addr < 4095 and
    // addr+1 ≤ 4096 without a boundary-case mux in triode_bram.
    // ================================================================
    sample_t x_clamped;
    always_comb begin
        if      ($signed(x_in) >  $signed(24'sh7FF000)) x_clamped =  24'sh7FF000;
        else if ($signed(x_in) < -$signed(24'sh7FF000)) x_clamped = -24'sh7FF000;
        else                                            x_clamped =  x_in;
    end

    // ================================================================
    // Gap 2 — conditional shelf placement.  The generate block below
    // routes cathode_shelf either pre-LUT (x_clamped → lut_in) or
    // post-LPF (lpf_y → hpf_in).  The unused side is bypassed with a
    // direct wire, keeping gain_stage structurally identical aside from
    // the shelf's 4-cycle pipeline showing up on whichever side is active.
    // ================================================================
    sample_t lut_in;
    logic    lut_in_valid;
    sample_t hpf_in;
    logic    hpf_in_valid;

    // ================================================================
    // Latch the LUT-domain input on its valid pulse.  Combinational
    // addr/t decode off s0_x remains identical to the original design.
    // ================================================================
    sample_t s0_x;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)             s0_x <= '0;
        else if (lut_in_valid)  s0_x <= lut_in;
    end

    logic [23:0] biased;
    logic [11:0] addr;
    logic [12:0] addr_p1;
    logic [10:0] t_c;

    assign biased  = s0_x ^ 24'h800000;
    assign addr    = biased[23:12];
    assign addr_p1 = {1'b0, addr} + 13'd1;
    assign t_c     = biased[11:1];

    // ================================================================
    // Dual BRAM reads (1-cycle latency).  s3_y0/s3_y1 and s3_t0/s3_t1
    // are the BRAM output registers and remain stable for as long as
    // s0_x is stable (≥130 cycles between samples at 768 kHz), so the
    // scheduler reads them directly as operands without re-latching.
    // ================================================================
    sample_t s3_y0, s3_y1;
    sample_t s3_t0, s3_t1;

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

    // ================================================================
    // Shared sequential multiplier
    //
    //   Exercises one 32×32 → 64 signed multiplier over the seven
    //   dependency-ordered products of the original pipeline:
    //
    //     0) t  · t            → t²     (11×11 unsigned, fits 22b)
    //     1) t  · t²           → t³_raw (11×22 unsigned, fits 33b)
    //     2) h00 · y0          → p00    (26×24 signed,    50b)
    //     3) h10 · t0          → p10
    //     4) h01 · y1          → p01
    //     5) h11 · t1          → p11
    //     6) G_STAGE · s6_y    → g_prod (32×24 signed,    56b)
    //
    //   Dependent products (t², t³, g_prod) require a one-cycle bubble
    //   after the producing op before their result can be consumed as
    //   an operand.  The four independent PCHIP products stream through
    //   back-to-back.  Total sequence = 19 sys_clk cycles (op_cnt 1..18).
    // ================================================================
    logic signed [31:0] mul_a_q, mul_b_q;
    logic signed [31:0] mul_a_d, mul_b_d;
    (* use_dsp = "yes" *) logic signed [63:0] mul_p_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_a_q <= '0;
            mul_b_q <= '0;
            mul_p_q <= '0;
        end else begin
            mul_a_q <= mul_a_d;
            mul_b_q <= mul_b_d;
            mul_p_q <= mul_a_q * mul_b_q;
        end
    end

    // ---- FSM counter ------------------------------------------------
    //   op_cnt == 0       : idle, waiting for lut_in_valid
    //   op_cnt == 1..18   : running the sequence
    //   op_cnt transitions back to 0 after step 18 (end-of-sequence)
    localparam int unsigned OP_LAST = 18;

    logic [4:0] op_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            op_cnt <= 5'd0;
        end else if (op_cnt == 5'd0) begin
            if (lut_in_valid) op_cnt <= 5'd1;
        end else if (op_cnt == OP_LAST[4:0]) begin
            op_cnt <= 5'd0;
        end else begin
            op_cnt <= op_cnt + 5'd1;
        end
    end

    // ---- Intermediate holding registers -----------------------------
    logic        [21:0] t2_r;
    logic        [21:0] t3_r;
    logic signed [49:0] p00_r, p10_r, p01_r, p11_r;
    logic signed [51:0] sum52_r;
    logic signed [95:0] lut_y_96_r;
    logic signed [55:0] g_prod_r;
    logic signed [95:0] g_q23_96_r;

    // ---- Combinational reconstructions of h00..h11 -------------------
    //   Identical Q1.22 polynomial basis as the original parallel design.
    //   t_c is comb (stable while s0_x held); t2_r and t3_r are the
    //   latched squares/cubes captured from mul_p_q during earlier ops.
    logic signed [25:0] t_q22_s, t2_q22_s, t3_q22_s;
    logic signed [25:0] one_q22;
    logic signed [25:0] h00_c, h10_c, h01_c, h11_c;

    assign t_q22_s  = $signed({4'b0000, t_c,  11'b000_0000_0000});
    assign t2_q22_s = $signed({4'b0000, t2_r});
    assign t3_q22_s = $signed({4'b0000, t3_r});
    assign one_q22  = 26'sd4194304;

    always_comb begin
        h00_c = one_q22 - (t2_q22_s <<< 1) - t2_q22_s + (t3_q22_s <<< 1);
        h01_c =          (t2_q22_s <<< 1) + t2_q22_s - (t3_q22_s <<< 1);
        h10_c = t_q22_s - (t2_q22_s <<< 1) + t3_q22_s;
        h11_c =                              t3_q22_s - t2_q22_s;
    end

    // ---- Sum / round / saturate combinational paths -----------------
    logic signed [51:0] sum52_c;
    logic signed [95:0] sum52_ext;
    logic signed [95:0] lut_y_96_c;
    sample_t            s6_y_c;
    logic signed [95:0] g_prod_ext;
    logic signed [95:0] g_q23_96_c;
    sample_t            g_y_c;

    always_comb begin
        sum52_c    = $signed(p00_r) + $signed(p10_r)
                   + $signed(p01_r) + $signed(p11_r);
        sum52_ext  = {{44{sum52_r[51]}}, sum52_r};
        lut_y_96_c = round_conv96(sum52_ext, 22);
        s6_y_c     = saturate_s24(lut_y_96_r);
        g_prod_ext = {{40{g_prod_r[55]}}, g_prod_r};
        g_q23_96_c = round_conv96(g_prod_ext, SHIFT_G);
        g_y_c      = saturate_s24(g_q23_96_r);
    end

    // ---- Gain coefficient (static per configuration) ---------------
    localparam logic signed [31:0] G_USED =
        (G_OVERRIDE_Q4_20 != 32'sh0) ? G_OVERRIDE_Q4_20 : G_STAGE_Q4_20[STAGE];

    // ---- Operand mux ------------------------------------------------
    //   Each op_cnt value selects the operands for the posedge that
    //   samples mul_a_d / mul_b_d.  `DC` slots are kept at zero so the
    //   shared multiplier does no useful work in the bubble cycles
    //   (dependent-product wait slots and the tail of the sequence).
    localparam logic signed [31:0] DC = 32'sh0;

    // Sign-extend helpers (sample_t is 24-bit signed; h is 26-bit signed).
    function automatic logic signed [31:0] ext_u11 (input logic [10:0] x);
        ext_u11 = {21'b0, x};
    endfunction
    function automatic logic signed [31:0] ext_u22 (input logic [21:0] x);
        ext_u22 = {10'b0, x};
    endfunction
    function automatic logic signed [31:0] ext_s24 (input sample_t x);
        ext_s24 = {{8{x[23]}}, x};
    endfunction
    function automatic logic signed [31:0] ext_s26 (input logic signed [25:0] x);
        ext_s26 = {{6{x[25]}}, x};
    endfunction

    always_comb begin
        mul_a_d = DC;
        mul_b_d = DC;
        unique case (op_cnt)
            // op 0: t*t    — t_c combinational, always valid while s0_x stable
            5'd1:  begin mul_a_d = ext_u11(t_c);  mul_b_d = ext_u11(t_c);  end
            // op 1: bubble (t² in flight)
            // op 2: t*t²   — uses mul_p_q which has just committed to t²
            5'd3:  begin mul_a_d = ext_u11(t_c);  mul_b_d = ext_u22(mul_p_q[21:0]); end
            // op 3: bubble (t³ in flight)
            // op 4: bubble — capture t3_r; compute h00..h11 comb next cycle
            // op 5: PCHIP p00 = h00 * y0
            5'd6:  begin mul_a_d = ext_s26(h00_c); mul_b_d = ext_s24(s3_y0); end
            // op 6: PCHIP p10 = h10 * t0
            5'd7:  begin mul_a_d = ext_s26(h10_c); mul_b_d = ext_s24(s3_t0); end
            // op 7: PCHIP p01 = h01 * y1
            5'd8:  begin mul_a_d = ext_s26(h01_c); mul_b_d = ext_s24(s3_y1); end
            // op 8: PCHIP p11 = h11 * t1
            5'd9:  begin mul_a_d = ext_s26(h11_c); mul_b_d = ext_s24(s3_t1); end
            // op 9..11: bubbles while last PCHIP product lands and sum/round/sat proceed
            // op 14: G_STAGE * s6_y — s6_y_c is combinational off lut_y_96_r
            5'd14: begin mul_a_d = G_USED;        mul_b_d = ext_s24(s6_y_c); end
            default: begin mul_a_d = DC; mul_b_d = DC; end
        endcase
    end

    // ---- Capture pipeline -------------------------------------------
    //   Each op_cnt value updates exactly the registers whose inputs
    //   are valid on that cycle.  Because `mul_p_q` is overwritten on
    //   every posedge, captures of dependent products are done in the
    //   cycle immediately after the product lands.
    logic g_valid_q;
    sample_t g_y;
    logic    g_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t2_r        <= '0;
            t3_r        <= '0;
            p00_r       <= '0;
            p10_r       <= '0;
            p01_r       <= '0;
            p11_r       <= '0;
            sum52_r     <= '0;
            lut_y_96_r  <= '0;
            g_prod_r    <= '0;
            g_q23_96_r  <= '0;
            g_y         <= '0;
            g_valid_q   <= 1'b0;
        end else begin
            g_valid_q <= 1'b0;
            unique case (op_cnt)
                // op 3: mul_p_q now holds t² (from op 1's t*t, latched at prior posedge)
                5'd3:  t2_r <= mul_p_q[21:0];
                // op 5: mul_p_q holds t³_raw = t * t² (from op 3's multiply)
                5'd5:  t3_r <= mul_p_q[32:11];
                // op 8..11: capture the four PCHIP products as they stream out
                5'd8:  p00_r <= mul_p_q[49:0];
                5'd9:  p10_r <= mul_p_q[49:0];
                5'd10: p01_r <= mul_p_q[49:0];
                5'd11: p11_r <= mul_p_q[49:0];
                // op 12: all four products captured; sum the tree
                5'd12: sum52_r <= sum52_c;
                // op 13: round sum52 by 22 into a 96-bit convergent-rounded value
                5'd13: lut_y_96_r <= lut_y_96_c;
                // op 16: mul_p_q holds g_stage * s6_y (from op 14's multiply)
                5'd16: g_prod_r <= mul_p_q[55:0];
                // op 17: round g_prod by SHIFT_G
                5'd17: g_q23_96_r <= g_q23_96_c;
                // op 18: saturate → final g_y; emit g_valid pulse
                5'd18: begin
                    g_y       <= g_y_c;
                    g_valid_q <= 1'b1;
                end
                default: ;
            endcase
        end
    end

    assign g_valid = g_valid_q;

    // ================================================================
    // iir_lpf1 — plate LPF (Miller + Rp_ac)
    // ================================================================
    sample_t lpf_y;
    logic    lpf_valid;

    iir_lpf1 #(.COEFF_FILE(LPF_FILE)) u_lpf (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (g_y),
        .x_valid (g_valid),
        .y_out   (lpf_y),
        .y_valid (lpf_valid)
    );

    // ================================================================
    // cathode_shelf — position selected by SHELF_PRE_LUT.
    //   =0 (default, unbypassed stages and the PI): shelf sits between
    //       the plate LPF and the output coupling HPF, as before.  For
    //       stages with no bypass cap the .mem encodes identity and this
    //       instance is a 4-cycle delay with no tonal effect.
    //   =1 (V1A in the JCM800 2203): shelf sits BEFORE the LUT, acting
    //       as a pre-emphasis that attenuates LF so the LUT sees the
    //       correctly-degenerated LF amplitude.  The post-LPF path then
    //       wires straight to the HPF.
    // ================================================================
    generate
        if (SHELF_PRE_LUT) begin : g_pre_shelf
            cathode_shelf #(.COEFF_FILE(SHELF_FILE)) u_shelf (
                .clk     (clk),
                .rst_n   (rst_n),
                .x_in    (x_clamped),
                .x_valid (x_valid),
                .y_out   (lut_in),
                .y_valid (lut_in_valid)
            );
            assign hpf_in       = lpf_y;
            assign hpf_in_valid = lpf_valid;
        end else begin : g_post_shelf
            assign lut_in       = x_clamped;
            assign lut_in_valid = x_valid;
            cathode_shelf #(.COEFF_FILE(SHELF_FILE)) u_shelf (
                .clk     (clk),
                .rst_n   (rst_n),
                .x_in    (lpf_y),
                .x_valid (lpf_valid),
                .y_out   (hpf_in),
                .y_valid (hpf_in_valid)
            );
        end
    endgenerate

    // ================================================================
    // iir_hpf1 — output coupling HPF (+ Gap 1 asymmetric bias tracker)
    // ================================================================
    iir_hpf1 #(
        .COEFF_FILE (HPF_FILE),
        .SHIFT_BIAS (HPF_SHIFT_BIAS)
    ) u_hpf (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (hpf_in),
        .x_valid (hpf_in_valid),
        .y_out   (y_out),
        .y_valid (y_valid)
    );

endmodule
