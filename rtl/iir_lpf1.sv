
// =====================================================================
// iir_lpf1 — first-order IIR low-pass filter (Q9.39 wide state)
//
//   y[n] = y[n−1] + β · (x[n] − y[n−1])
//
//   β is Q1.31 signed, loaded from COEFF_FILE (one 32-bit hex word).
//   x, y are sample_t (Q1.23 s24).  Used for:
//     · per-stage plate LPF (Rp ∥ next-grid, Miller-loaded C_plate)
//     · the single top-level input anti-alias (input_aa.sv instantiates
//       this module with input_aa.mem)
//
//   Pipeline (5 register stages — β narrow at 16× oversampling puts a
//   very long carry chain between err-reg and y_out; splitting at every
//   adder/rounder hop keeps each stage under 10 ns):
//     S0 — on x_valid: latch x, compute err_q39 = (x << 16) − y_prev_q39;
//          register s0_err_q39.
//     S1 — prod = β · err_q39 (Q1.31 × Q9.39 → Q9.70 s80).  Register the
//          DSP cascade output (s1_prod80).
//     S2 — delta_q39 = round_conv96(s1_prod80, 31)[47:0].  Register.
//     S3 — y_q39_next = y_prev_q39 + delta_q39.  Register and update
//          y_prev_q39 ← y_q39_next.
//     S4 — narrow Q9.39 → Q1.23 via round >>> 16 and saturate.
// =====================================================================
module iir_lpf1
    import jcm800_pkg::*;
#(
    parameter string COEFF_FILE = ""
)(
    input  logic    clk,
    input  logic    rst_n,
    input  sample_t x_in,
    input  logic    x_valid,
    output sample_t y_out,
    output logic    y_valid
);

    // ----------------------------------------------------------------
    // Coefficient ROM — one Q1.31 β (32-bit)
    // ----------------------------------------------------------------
    (* rom_style = "distributed" *)
    logic [31:0] coef_mem [0:0];
    initial $readmemh(COEFF_FILE, coef_mem);

    wire signed [31:0] beta_q31 = $signed(coef_mem[0]);

    // ----------------------------------------------------------------
    // State
    // ----------------------------------------------------------------
    logic signed [47:0] y_prev_q39;

    // ----------------------------------------------------------------
    // S0 — on x_valid: err_q39 = (x_in << 16) − y_prev_q39  (both Q9.39)
    // ----------------------------------------------------------------
    logic signed [47:0] x_q39_c;
    logic signed [47:0] err_c;
    logic signed [47:0] s0_err_q39;
    logic               s0_valid;

    always_comb begin
        x_q39_c = {{9{x_in[23]}}, x_in, 16'b0};     // Q1.23 → Q9.39
        err_c   = x_q39_c - y_prev_q39;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_err_q39 <= '0;
            s0_valid   <= 1'b0;
        end else begin
            s0_valid <= x_valid;
            if (x_valid) s0_err_q39 <= err_c;
        end
    end

    // ----------------------------------------------------------------
    // S1 — prod = β · err_q39 (Q9.70 s80).  Register the DSP cascade
    //      output so the downstream rounder/adder/saturator gets a
    //      fresh clock period to ride its 48-bit carry chain.
    // ----------------------------------------------------------------
    logic signed [79:0] prod80_c;
    logic signed [79:0] s1_prod80;
    logic               s1_valid;

    always_comb prod80_c = beta_q31 * s0_err_q39;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_prod80 <= '0;
            s1_valid  <= 1'b0;
        end else begin
            s1_valid <= s0_valid;
            if (s0_valid) s1_prod80 <= prod80_c;
        end
    end

    // ----------------------------------------------------------------
    // S2 — convergent-round s1_prod80 >>> 31 → Q9.39 delta_q39.  Register.
    // ----------------------------------------------------------------
    logic signed [95:0] prod80_ext;
    logic signed [95:0] delta_q39_96;
    logic signed [47:0] delta_q39_c;
    logic signed [47:0] s2_delta_q39;
    logic               s2_valid;

    always_comb begin
        prod80_ext   = {{16{s1_prod80[79]}}, s1_prod80};
        delta_q39_96 = round_conv96(prod80_ext, 31);
        delta_q39_c  = delta_q39_96[47:0];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_delta_q39 <= '0;
            s2_valid     <= 1'b0;
        end else begin
            s2_valid <= s1_valid;
            if (s1_valid) s2_delta_q39 <= delta_q39_c;
        end
    end

    // ----------------------------------------------------------------
    // S3 — y_q39_next = y_prev_q39 + s2_delta_q39.  Register and update
    //      y_prev_q39 ← y_q39_next.
    // ----------------------------------------------------------------
    logic signed [47:0] y_q39_next_c;
    logic signed [47:0] s3_y_q39;
    logic               s3_valid;

    always_comb y_q39_next_c = y_prev_q39 + s2_delta_q39;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_prev_q39 <= '0;
            s3_y_q39   <= '0;
            s3_valid   <= 1'b0;
        end else begin
            s3_valid <= s2_valid;
            if (s2_valid) begin
                s3_y_q39   <= y_q39_next_c;
                y_prev_q39 <= y_q39_next_c;
            end
        end
    end

    // ----------------------------------------------------------------
    // S4a — narrow Q9.39 → Q1.23 via round >>> 16; register rounded
    //       value.  S4b does the saturate on the next cycle so the
    //       96-bit bias-add and the 96-bit saturation compare split
    //       across two sys_clk periods.
    // ----------------------------------------------------------------
    logic signed [95:0] y_q39_ext;
    logic signed [95:0] y_q23_96_c;
    logic signed [95:0] y_q23_96_q;
    logic               s4_valid;

    always_comb begin
        y_q39_ext  = {{48{s3_y_q39[47]}}, s3_y_q39};
        y_q23_96_c = round_conv96(y_q39_ext, 16);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_q23_96_q <= '0;
            s4_valid   <= 1'b0;
        end else begin
            s4_valid <= s3_valid;
            if (s3_valid) y_q23_96_q <= y_q23_96_c;
        end
    end

    // ----------------------------------------------------------------
    // S4b — saturate rounded value to sample_t.
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out   <= '0;
            y_valid <= 1'b0;
        end else begin
            y_valid <= s4_valid;
            if (s4_valid) y_out <= saturate_s24(y_q23_96_q);
        end
    end

endmodule
