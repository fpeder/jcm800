
// =====================================================================
// nfb_network — closed-loop negative feedback from OT secondary back
//               to the PI shared cathode (with Presence control)
//
//   Signal chain at 768 kHz:
//
//     v_sec ──► [1-sample z⁻¹] ──► × NFB_Q4_20 (≈ 0.045)
//                                        │
//                                        ▼
//                                  cathode_shelf (presence shelf biquad
//                                   from shelf_presence.mem — tilts HF
//                                   up in the feedback path, which is
//                                   why moving the Presence pot sounds
//                                   like it brightens the amp)
//                                        │
//                                        ▼
//                                  level_ctrl (presence pot log-taper —
//                                   at pot=0 the shelf output is muted,
//                                   collapsing NFB to flat; at pot=255
//                                   the shelf runs at full depth)
//                                        │
//                                        ▼
//                                  nfb_inject (to phase_inverter.sv)
//
//   The 1-sample delay breaks the algebraic loop cleanly.  At 768 kHz it
//   introduces ≈ 1.3 µs of lag in the feedback path — a fraction of a
//   degree at low frequencies, ≈ 50° at 100 kHz.  Well outside the audio
//   band the loop is still stable with a 27 dB static gain.
//
//   The shelf runs in the LOW-amplitude NFB path, so we don't need the
//   asymmetric HPF's bias-tracker machinery; the cathode_shelf reuse is
//   sufficient (it's already a first-order biquad with its own coefficient
//   file, and we emit shelf_presence.mem specifically for this slot).
// =====================================================================
module nfb_network
    import jcm800_pkg::*;
    import jcm800_power_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  sample_t     v_sec,
    input  logic        v_sec_valid,
    input  logic [7:0]  presence_pot_pos,

    // Held output — phase_inverter.sv samples this combinationally when
    // its own x_valid fires, so the loop closes cleanly once the NFB
    // pipeline has completed (well before the next 768 kHz sample arrives).
    output sample_t     nfb_inject,
    output logic        nfb_inject_valid
);

    // -----------------------------------------------------------------
    // 1-sample delay.  Capture v_sec on its valid pulse; the registered
    // value becomes the input to the NFB pipeline, which fires the same
    // cycle.  By the time phase_inverter starts its NEXT sample, the NFB
    // pipeline has delivered its updated inject value.
    // -----------------------------------------------------------------
    sample_t v_sec_z1;
    logic    v_sec_z1_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_sec_z1       <= '0;
            v_sec_z1_valid <= 1'b0;
        end else begin
            v_sec_z1_valid <= v_sec_valid;
            if (v_sec_valid) v_sec_z1 <= v_sec;
        end
    end

    // -----------------------------------------------------------------
    // × NFB_Q4_20 — reuse the ×G pattern from gain_stage (2 cycles).
    // -----------------------------------------------------------------
    logic signed [31:0] nfb_coef;
    assign nfb_coef = NFB_Q4_20;

    logic signed [55:0] nfb_prod_c;
    logic signed [55:0] nfb_prod_r;
    logic               nfb_prod_valid;

    always_comb nfb_prod_c = nfb_coef * v_sec_z1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nfb_prod_r     <= '0;
            nfb_prod_valid <= 1'b0;
        end else begin
            nfb_prod_valid <= v_sec_z1_valid;
            if (v_sec_z1_valid) nfb_prod_r <= nfb_prod_c;
        end
    end

    logic signed [95:0] nfb_prod_ext;
    logic signed [95:0] nfb_q23_96_c;
    logic signed [95:0] nfb_q23_96_q;
    logic               nfb_round_valid;

    always_comb begin
        nfb_prod_ext = {{40{nfb_prod_r[55]}}, nfb_prod_r};
        nfb_q23_96_c = round_conv96(nfb_prod_ext, SHIFT_NFB);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nfb_q23_96_q    <= '0;
            nfb_round_valid <= 1'b0;
        end else begin
            nfb_round_valid <= nfb_prod_valid;
            if (nfb_prod_valid) nfb_q23_96_q <= nfb_q23_96_c;
        end
    end

    sample_t nfb_scaled;
    logic    nfb_scaled_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nfb_scaled       <= '0;
            nfb_scaled_valid <= 1'b0;
        end else begin
            nfb_scaled_valid <= nfb_round_valid;
            if (nfb_round_valid) nfb_scaled <= saturate_s24(nfb_q23_96_q);
        end
    end

    // -----------------------------------------------------------------
    // Presence shelf — first-order high-shelf biquad (4-cycle pipeline).
    // -----------------------------------------------------------------
    sample_t shelf_y;
    logic    shelf_valid;

    cathode_shelf #(
        .COEFF_FILE (PRESENCE_SHELF_FILE)
    ) u_shelf (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (nfb_scaled),
        .x_valid (nfb_scaled_valid),
        .y_out   (shelf_y),
        .y_valid (shelf_valid)
    );

    // -----------------------------------------------------------------
    // Presence pot — log-taper attenuator (3-cycle pipeline).  At pos=0
    // the shelf output is muted (≈ −60 dB), so the NFB signal reverts
    // to flat (pre-shelf coupling): in a real 2203 this is the "presence
    // at 0" behaviour that leaves the raw NFB shaping intact.  At pos=255
    // the shelf passes at unity, so the full HF lift is applied.
    // -----------------------------------------------------------------
    level_ctrl #(
        .COEF_FILE (PRESENCE_POT_FILE)
    ) u_pres_pot (
        .clk     (clk),
        .rst_n   (rst_n),
        .x_in    (shelf_y),
        .x_valid (shelf_valid),
        .pot_pos (presence_pot_pos),
        .y_out   (nfb_inject),
        .y_valid (nfb_inject_valid)
    );

endmodule
