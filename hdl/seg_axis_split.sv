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
    input  wire [NSEG-1:0]              s_ena,
    input  wire [NSEG-1:0]              s_sop,
    input  wire [NSEG-1:0]              s_eop,
    input  wire [NSEG-1:0]              s_err,
    input  wire [NSEG*$clog2(SEGB)-1:0] s_mty,

    output wire [NSEG*SEGB*8-1:0]       m_axis_seg_tdata,
    output wire [N-1:0]                 m_axis_seg_tvalid,
    output wire [N*NSEG-1:0]            m_ena,
    output wire [N*NSEG-1:0]            m_sop,
    output wire [N*NSEG-1:0]            m_eop,
    output wire [N*NSEG-1:0]            m_err,
    output wire [NSEG*$clog2(SEGB)-1:0] m_mty
);

    localparam MTYW = $clog2(SEGB);
    localparam LW   = (N > 1) ? $clog2(N) : 1;

    reg conflict; // simulation aid: a lane would receive two frames in the same beat (N too small)

    // ------------------------------------------------------------------
    // registers and their next values
    // ------------------------------------------------------------------
    reg [LW-1:0]            cur_lane_reg = {LW{1'b0}}, cur_lane_next; // lane of the frame in progress
    reg [LW-1:0]            rr_reg       = {LW{1'b0}}, rr_next; // lane of the next new frame

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
    reg [N-1:0][NSEG-1:0]   ena_lane, sop_lane, eop_lane, err_lane; // segment masks per lane

    integer s, i;

    always @(*) begin
        // default: hold the state, clear the lane masks
        cur_lane_next = cur_lane_reg;
        rr_next       = rr_reg;

        ena_lane = {N*NSEG{1'b0}};
        sop_lane = {N*NSEG{1'b0}};
        eop_lane = {N*NSEG{1'b0}};
        err_lane = {N*NSEG{1'b0}};

        lane = cur_lane_reg;
        rr   = rr_reg;

        conflict = 1'b0;

        // scan the segments in order and give each one to the lane of its frame
        for (s = 0; s < NSEG; s = s + 1) begin
            if (s_ena[s]) begin
                if (s_sop[s]) begin
                    // a new frame starts here: take the next lane in round-robin order
                    lane = rr;
                    rr   = (rr == N-1) ? {LW{1'b0}} : rr + 1'b1;
                    `ifndef SYNTHESIS
                    if (|ena_lane[lane]) begin
                        conflict = 1'b1;
                    end
                    `endif
                end
                ena_lane[lane][s] = 1'b1;
                sop_lane[lane][s] = s_sop[s];
                eop_lane[lane][s] = s_eop[s];
                err_lane[lane][s] = s_err[s];
            end
        end

        // lane outputs, only meaningful for a valid input beat
        m_axis_seg_tdata_next = s_axis_seg_tdata;
        m_mty_next            = s_mty;

        for (i = 0; i < N; i = i + 1) begin
            m_axis_seg_tvalid_next[i]      = s_axis_seg_tvalid & (|ena_lane[i]);
            m_ena_next[i*NSEG +: NSEG]     = s_axis_seg_tvalid ? ena_lane[i] : {NSEG{1'b0}};
            m_sop_next[i*NSEG +: NSEG]     = s_axis_seg_tvalid ? sop_lane[i] : {NSEG{1'b0}};
            m_eop_next[i*NSEG +: NSEG]     = s_axis_seg_tvalid ? eop_lane[i] : {NSEG{1'b0}};
            m_err_next[i*NSEG +: NSEG]     = s_axis_seg_tvalid ? err_lane[i] : {NSEG{1'b0}};
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
    cur_lane_reg          <= {LW{1'b0}};
    rr_reg                <= {LW{1'b0}};
    m_axis_seg_tvalid_reg <= {N{1'b0}};
    m_ena_reg             <= {N*NSEG{1'b0}};
    m_sop_reg             <= {N*NSEG{1'b0}};
    m_eop_reg             <= {N*NSEG{1'b0}};
    m_err_reg             <= {N*NSEG{1'b0}};
    end else begin
    cur_lane_reg          <= cur_lane_next;
    rr_reg                <= rr_next;
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

    assign m_axis_seg_tdata  = m_axis_seg_tdata_reg;
    assign m_axis_seg_tvalid = m_axis_seg_tvalid_reg;
    assign m_ena             = m_ena_reg;
    assign m_sop             = m_sop_reg;
    assign m_eop             = m_eop_reg;
    assign m_err             = m_err_reg;
    assign m_mty             = m_mty_reg;


    `ifndef SYNTHESIS
    always @(posedge clk) begin
    if (!rst && s_axis_seg_tvalid && conflict)
    $display("SPLIT ERR @%0t: more frames in one beat than lanes (N=%0d too small)", $time, N);
    end
    `endif

endmodule

`resetall
