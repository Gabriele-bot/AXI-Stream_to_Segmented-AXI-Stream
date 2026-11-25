// SPDX-License-Identifier: CERN-OHL-P-2.0
// Copyright 2026 Gabriele Bortolato
// Source Location: https://github.com/Gabriele-bot/AXI-Stream_to_Segmented-AXI-Stream
// This source describes Open Hardware and is licensed under the CERN-OHL-P v2.
// You may redistribute and modify this documentation and make products using it
// under the terms of the CERN-OHL-P v2 (https://ohwr.org/cern_ohl_p_v2.txt).
// This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
// INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE.
// Please see the CERN-OHL-P v2 for applicable conditions.
`resetall `timescale 1ns / 1ps `default_nettype none

/*
 * Segment splitter: separates the frames of a Segmented AXI-Stream into N lanes.
 *
 * The input beat can carry the end of one frame, one or more complete frames and the start of
 * another one (the sop of a frame does not need to be in segment 0). Every frame is assigned
 * to a lane in round-robin order (frame k -> lane k mod N) and stays on that lane until its eop.
 * Each lane sees only the segments of its own frame, so a lane carries at most one frame per beat
 * (an "unaligned" segmented stream, as expected by unaligned_axi_seg_param_2_axis).
 *
 * N must be >= the maximum number of frames that can touch one beat. With a minimum frame of
 * 60 Bytes (FCS stripped) this is 3 for a 128 Byte beat (400G) and 2 for 64 / 32 Byte beats (200G / 100G).
 *
 * tdata and mty are common to all lanes (only the ena/sop/eop/err masks and the valid differ).
 * One register stage on the output.
 */
module seg_axis_split #(
    parameter NSEG = 8,
    parameter SEGB = 16, // bytes per segment
    parameter N    = (NSEG*SEGB >= 128) ? 3 : 2
)(
    input  wire clk,
    input  wire rst,

    input  wire [NSEG*SEGB*8-1:0]       s_axis_seg_tdata,
    input  wire                         s_axis_seg_tvalid,
    input  wire [NSEG-1:0]              s_axis_seg_tuser_ena,
    input  wire [NSEG-1:0]              s_axis_seg_tuser_sop,
    input  wire [NSEG-1:0]              s_axis_seg_tuser_eop,
    input  wire [NSEG-1:0]              s_axis_seg_tuser_err,
    input  wire [NSEG*$clog2(SEGB)-1:0] s_axis_seg_tuser_mty,

    output wire [NSEG*SEGB*8-1:0]       m_axis_seg_tdata,
    output wire [N-1:0]                 m_axis_seg_tvalid,
    output wire [N*NSEG-1:0]            m_axis_seg_tuser_ena,
    output wire [N*NSEG-1:0]            m_axis_seg_tuser_sop,
    output wire [N*NSEG-1:0]            m_axis_seg_tuser_eop,
    output wire [N*NSEG-1:0]            m_axis_seg_tuser_err,
    output wire [NSEG*$clog2(SEGB)-1:0] m_axis_seg_tuser_mty
);

    localparam MTYW = $clog2(SEGB);
    localparam LW   = (N > 1) ? $clog2(N) : 1;

    reg conflict; // simulation aid: a lane would receive two frames in the same beat (N too small)

    // ------------------------------------------------------------------
    // registers and their next values
    // ------------------------------------------------------------------
    reg [LW-1:0]            cur_lane_reg = {LW{1'b0}}, cur_lane_next; // lane of the frame in progress
    reg [LW-1:0]            rr_reg       = {LW{1'b0}}, rr_next; // lane of the next new frame

    reg [NSEG*SEGB*8-1:0]   axis_seg_tdata_reg  = {NSEG*SEGB*8{1'b0}}, axis_seg_tdata_next;
    reg [N-1:0]             axis_seg_tvalid_reg = {N{1'b0}},           axis_seg_tvalid_next;
    reg [N*NSEG-1:0]        ena_reg = {N*NSEG{1'b0}}, ena_next;
    reg [N*NSEG-1:0]        sop_reg = {N*NSEG{1'b0}}, sop_next;
    reg [N*NSEG-1:0]        eop_reg = {N*NSEG{1'b0}}, eop_next;
    reg [N*NSEG-1:0]        err_reg = {N*NSEG{1'b0}}, err_next;
    reg [NSEG*MTYW-1:0]     mty_reg = {NSEG*MTYW{1'b0}}, mty_next;

    reg [NSEG*SEGB*8-1:0]   m_axis_seg_tdata_reg  = {NSEG*SEGB*8{1'b0}}, m_axis_seg_tdata_next;
    reg [N-1:0]             m_axis_seg_tvalid_reg = {N{1'b0}},           m_axis_seg_tvalid_next;
    reg [N*NSEG-1:0]        m_ena_reg = {N*NSEG{1'b0}}, m_ena_next;
    reg [N*NSEG-1:0]        m_sop_reg = {N*NSEG{1'b0}}, m_sop_next;
    reg [N*NSEG-1:0]        m_eop_reg = {N*NSEG{1'b0}}, m_eop_next;
    reg [N*NSEG-1:0]        m_err_reg = {N*NSEG{1'b0}}, m_err_next;
    reg [NSEG*MTYW-1:0]     m_mty_reg = {NSEG*MTYW{1'b0}}, m_mty_next;

    // ------------------------------------------------------------------
    // combinational process
    // ------------------------------------------------------------------
    reg [LW-1:0]            lane; // lane the segment being scanned belongs to
    reg [LW-1:0]            rr; // running round-robin pointer inside the beat
    reg [N-1:0][NSEG-1:0]   ena_lane_next, ena_lane_reg; // segment masks per lane
    reg [N-1:0][NSEG-1:0]   sop_lane_next, sop_lane_reg; // segment masks per lane
    reg [N-1:0][NSEG-1:0]   eop_lane_next, eop_lane_reg; // segment masks per lane
    reg [N-1:0][NSEG-1:0]   err_lane_next, err_lane_reg; // segment masks per lane



    integer s, i;

    always @(*) begin
        // default: hold the state, clear the lane masks
        cur_lane_next = cur_lane_reg;
        rr_next       = rr_reg;

        ena_lane_next = {N*NSEG{1'b0}};
        sop_lane_next = {N*NSEG{1'b0}};
        eop_lane_next = {N*NSEG{1'b0}};
        err_lane_next = {N*NSEG{1'b0}};

        lane = cur_lane_reg;
        rr   = rr_reg;

        conflict = 1'b0;

        // scan the segments in order and give each one to the lane of its frame
        for (s = 0; s < NSEG; s = s + 1) begin
            if (s_axis_seg_tuser_ena[s]) begin
                if (s_axis_seg_tuser_sop[s]) begin
                    // a new frame starts here: take the next lane in round-robin order
                    lane = rr;
                    rr   = (rr == N-1) ? {LW{1'b0}} : rr + 1'b1;
                    `ifndef SYNTHESIS
                    if (|ena_lane_next[lane]) begin
                        conflict = 1'b1;
                    end
                    `endif
                end
                ena_lane_next[lane][s] = 1'b1;
                sop_lane_next[lane][s] = s_axis_seg_tuser_sop[s];
                eop_lane_next[lane][s] = s_axis_seg_tuser_eop[s];
                err_lane_next[lane][s] = s_axis_seg_tuser_err[s];
            end
        end

        // lane outputs, only meaningful for a valid input beat
        m_axis_seg_tdata_next = axis_seg_tdata_reg;
        m_mty_next            = mty_reg;

        for (i = 0; i < N; i = i + 1) begin
            m_axis_seg_tvalid_next[i]      = axis_seg_tvalid_reg & (|ena_lane_reg[i]);
            m_ena_next[i*NSEG +: NSEG]     = axis_seg_tvalid_reg ? ena_lane_reg[i] : {NSEG{1'b0}};
            m_sop_next[i*NSEG +: NSEG]     = axis_seg_tvalid_reg ? sop_lane_reg[i] : {NSEG{1'b0}};
            m_eop_next[i*NSEG +: NSEG]     = axis_seg_tvalid_reg ? eop_lane_reg[i] : {NSEG{1'b0}};
            m_err_next[i*NSEG +: NSEG]     = axis_seg_tvalid_reg ? err_lane_reg[i] : {NSEG{1'b0}};
        end

        // the lane / round-robin state only advances on a valid beat
        if (s_axis_seg_tvalid) begin
            cur_lane_next = lane;
            rr_next       = rr;
        end
    end

    // ------------------------------------------------------------------
    // clocked process
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            axis_seg_tvalid_reg   <= 1'b0;
            cur_lane_reg          <= {LW{1'b0}};
            rr_reg                <= {LW{1'b0}};
            m_axis_seg_tvalid_reg <= {N{1'b0}};
            m_ena_reg             <= {N*NSEG{1'b0}};
            m_sop_reg             <= {N*NSEG{1'b0}};
            m_eop_reg             <= {N*NSEG{1'b0}};
            m_err_reg             <= {N*NSEG{1'b0}};
        end else begin
            axis_seg_tdata_reg    <= s_axis_seg_tdata;
            axis_seg_tvalid_reg   <= s_axis_seg_tvalid;
            mty_reg               <= s_axis_seg_tuser_mty;
            cur_lane_reg          <= cur_lane_next;
            rr_reg                <= rr_next;
            ena_lane_reg          <= ena_lane_next;
            sop_lane_reg          <= sop_lane_next;
            eop_lane_reg          <= eop_lane_next;
            err_lane_reg          <= err_lane_next;
            m_axis_seg_tvalid_reg <= m_axis_seg_tvalid_next;
            m_ena_reg             <= m_ena_next;
            m_sop_reg             <= m_sop_next;
            m_eop_reg             <= m_eop_next;
            m_err_reg             <= m_err_next;
        end

        // data and mty are not reset (they only qualify a valid lane beat)
        m_axis_seg_tdata_reg <= m_axis_seg_tdata_next;
        m_mty_reg            <= m_mty_next;
    end

    assign m_axis_seg_tdata     = m_axis_seg_tdata_reg;
    assign m_axis_seg_tvalid    = m_axis_seg_tvalid_reg;
    assign m_axis_seg_tuser_ena = m_ena_reg;
    assign m_axis_seg_tuser_sop = m_sop_reg;
    assign m_axis_seg_tuser_eop = m_eop_reg;
    assign m_axis_seg_tuser_err = m_err_reg;
    assign m_axis_seg_tuser_mty = m_mty_reg;


    `ifndef SYNTHESIS
    always @(posedge clk) begin
        if (!rst && s_axis_seg_tvalid && conflict)
            $display("SPLIT ERR @%0t: more frames in one beat than lanes (N=%0d too small)", $time, N);
    end
    `endif

endmodule

`resetall
