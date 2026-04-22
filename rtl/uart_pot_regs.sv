
// =====================================================================
// uart_pot_regs — 2-byte command parser, pot register file
//
//   Protocol (no framing byte):
//     Byte 0  = channel selector   (0x01 = gain, 0x02 = master,
//                                   0x03 = presence, 0x04 = bass,
//                                   0x05 = mid, 0x06 = treble)
//     Byte 1  = pot position 0..255, latched into the addressed register
//
//   Any other channel byte aborts the parser back to IDLE without
//   updating a register.  Reset defaults are chosen so the board boots
//   audible without UART traffic, but without pinning the preamp in
//   rail-to-rail clipping:
//     gain     = 0x80  (noon — clean-to-breakup on the preamp cascade)
//     master   = 0x80  (≈ −30 dB, log-taper midpoint — quiet, audible,
//                       keeps the PI / power amp out of hard saturation
//                       at the default gain setting)
//     presence = 0x80  (stock "noon" setting on the 2203 front panel)
//     bass     = 0x80  (noon — flat tonestack at reset)
//     mid      = 0x80
//     treble   = 0x80
//
//   Host-side, the simplest client is:
//       import serial
//       s = serial.Serial("/dev/ttyUSB1", 115200)
//       s.write(bytes([0x01, 0xFF]))   # gain = unity
//       s.write(bytes([0x02, 0x80]))   # master ≈ −30 dB
//       s.write(bytes([0x03, 0xC0]))   # presence ≈ −14 dB shelf depth
//       s.write(bytes([0x04, 0xFF]))   # bass = max boost
//       s.write(bytes([0x05, 0x40]))   # mid  = cut
//       s.write(bytes([0x06, 0xC0]))   # treble = moderate boost
// =====================================================================
module uart_pot_regs #(
    parameter logic [7:0] GAIN_DEFAULT     = 8'h80,
    parameter logic [7:0] MASTER_DEFAULT   = 8'h80,
    parameter logic [7:0] PRESENCE_DEFAULT = 8'h80,
    parameter logic [7:0] BASS_DEFAULT     = 8'h80,
    parameter logic [7:0] MID_DEFAULT      = 8'h80,
    parameter logic [7:0] TREBLE_DEFAULT   = 8'h80
)(
    input  logic       clk,
    input  logic       rst_n,

    input  logic       byte_valid,
    input  logic [7:0] byte_data,

    output logic [7:0] gain_pot_pos,
    output logic [7:0] master_pot_pos,
    output logic [7:0] presence_pot_pos,
    output logic [7:0] bass_pot_pos,
    output logic [7:0] mid_pot_pos,
    output logic [7:0] treble_pot_pos
);

    localparam logic [7:0] CH_GAIN     = 8'h01;
    localparam logic [7:0] CH_MASTER   = 8'h02;
    localparam logic [7:0] CH_PRESENCE = 8'h03;
    localparam logic [7:0] CH_BASS     = 8'h04;
    localparam logic [7:0] CH_MID      = 8'h05;
    localparam logic [7:0] CH_TREBLE   = 8'h06;

    typedef enum logic [0:0] {
        S_CH  = 1'b0,
        S_VAL = 1'b1
    } state_t;

    state_t     state;
    logic [7:0] pending_ch;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_CH;
            pending_ch       <= '0;
            gain_pot_pos     <= GAIN_DEFAULT;
            master_pot_pos   <= MASTER_DEFAULT;
            presence_pot_pos <= PRESENCE_DEFAULT;
            bass_pot_pos     <= BASS_DEFAULT;
            mid_pot_pos      <= MID_DEFAULT;
            treble_pot_pos   <= TREBLE_DEFAULT;
        end else if (byte_valid) begin
            unique case (state)
                S_CH: begin
                    if (byte_data == CH_GAIN     ||
                        byte_data == CH_MASTER   ||
                        byte_data == CH_PRESENCE ||
                        byte_data == CH_BASS     ||
                        byte_data == CH_MID      ||
                        byte_data == CH_TREBLE) begin
                        pending_ch <= byte_data;
                        state      <= S_VAL;
                    end
                    // else: unknown channel → stay in S_CH (effectively
                    // resyncs on the next recognised selector byte).
                end

                S_VAL: begin
                    if      (pending_ch == CH_GAIN)     gain_pot_pos     <= byte_data;
                    else if (pending_ch == CH_MASTER)   master_pot_pos   <= byte_data;
                    else if (pending_ch == CH_PRESENCE) presence_pot_pos <= byte_data;
                    else if (pending_ch == CH_BASS)     bass_pot_pos     <= byte_data;
                    else if (pending_ch == CH_MID)      mid_pot_pos      <= byte_data;
                    else if (pending_ch == CH_TREBLE)   treble_pot_pos   <= byte_data;
                    state <= S_CH;
                end
            endcase
        end
    end

endmodule
