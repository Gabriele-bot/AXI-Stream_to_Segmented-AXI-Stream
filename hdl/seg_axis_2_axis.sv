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
 Segmented AXI-Stream (NSEG segments of SEGB bytes, multiple frames per beat) -> N AXI-Stream outputs to be arbitrated.

 seg_axis_split sends every frame to a lane (round-robin, frame k -> lane k mod N) and each lane
 is an unaligned_seg_axis_2_axis instance, which sees one frame per beat with the sop not
 necessarily in segment 0 and converts it to a plain AXI-Stream.

 The N outputs are independent; lane i carries frames i, i+N, i+2N, ... The merge (if any) is
 done downstream in a roudn robin fashion.

 N: 3 for 400G (128 Byte beat), 2 for 200G / 100G, assuming a minimum frame of 60 Bytes (FCS stripped).

 Configurations examples:
   Xilinx DCMAC           128b segments (SEGB=16): 100G NSEG=2, 200G NSEG=4, 400G NSEG=8
   Altera F-Tile Ethernet 64b  segments (SEGB=8) : 100G NSEG=4, 200G NSEG=8, 400G NSEG=16
========================================================================================================================
Example with N=3
+----------------------+
|        INPUT         | 
+----------------------+
Packets can arrive back-to-back (sop and eop same beat), no idle cycles (no gap and sop seg 0), with idle cycles 
beat  0     1     2     3     4     5     
      +-----+-----+-----+-----+-----+-----+
      |  A0 |  B0 |  C2 |  C6 |  D2 |  E0  |
      +-----+-----+-----+-----+-----+-----+
      |  A1 |  B1 |  C3 |  C7 |     |  E1 |
      +-----+-----+-----+-----+-----+-----+
      |  A2 |  C0 |  C4 |  D0 |     |  E2 |
      +-----+-----+-----+-----+-----+-----+
      |     |  C1 |  C5 |  D1 |     |  E3 |
      +-----+-----+-----+-----+-----+-----+

+----------------------+
|        SPLIT         | 
+----------------------+
Each packet is separated from the others, but they are unaligned (sop not in seg 0)
                   Stream 0                               Stream 1                               Stream 2
beat  0     1     2     3     4     5        0     1     2     3     4     5        0     1     2     3     4     5      
      +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+
      |  A0 |     |     |     | D2  |     |  |     |  B0 |     |     |     |  E0 |  |     |     |  C2 |  C6 |     |     |
      +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+
      |  A1 |     |     |     |     |     |  |     |  B1 |     |     |     |  E1 |  |     |     |  C3 |  C7 |     |     |
      +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+
      |  A2 |     |     |  D0 |     |     |  |     |     |     |     |     |  E2 |  |     |  C0 |  C4 |     |     |     |
      +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+
      |     |     |     |  D1 |     |     |  |     |     |     |     |     |  E3 |  |     |  C1 |  C5 |     |     |     |
      +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+  +-----+-----+-----+-----+-----+-----+

+----------------------+
|    MERGE & ARB       | 
+----------------------+
Packets are aligned and an AXI-Stream is produced, then they are merged in one single AXI-Stream output
beat  0     1     2     3     4     5     
      +-----+-----+-----+-----+-----+-----+
      |  A0 |  B0 |  C0 |  C4 |  D0 |  E0 |
      +-----+-----+-----+-----+-----+-----+
      |  A1 |  B1 |  C1 |  C5 |  D1 |  E1 |
      +-----+-----+-----+-----+-----+-----+
      |  A2 |     |  C2 |  C6 |  D2 |  E2 |
      +-----+-----+-----+-----+-----+-----+
      |     |     |  C3 |  C7 |     |  E3 |
      +-----+-----+-----+-----+-----+-----+
 */
module seg_axis_2_axis #(
    parameter NSEG       = 8,
    parameter SEGB       = 16, // bytes per segment
    parameter N          = (NSEG*SEGB >= 128) ? 3 : 2,
    parameter FIFO_DEPTH = 4096,
    parameter ASYNC_FIFO = 1
)(
    input  wire s_clk,
    input  wire s_rst,

    input  wire [NSEG*SEGB*8-1:0]       s_axis_seg_tdata,
    input  wire                         s_axis_seg_tvalid,
    output wire                         s_axis_seg_tready,
    input  wire [NSEG-1:0]              s_ena,
    input  wire [NSEG-1:0]              s_sop,
    input  wire [NSEG-1:0]              s_eop,
    input  wire [NSEG-1:0]              s_err,
    input  wire [NSEG*$clog2(SEGB)-1:0] s_mty,

    input  wire m_clk,
    input  wire m_rst,

    output wire [NSEG*SEGB*8-1:0] m_axis_tdata,
    output wire [NSEG*SEGB-1:0]   m_axis_tkeep,
    output wire                   m_axis_tvalid,
    input  wire                   m_axis_tready,
    output wire                   m_axis_tlast,
    output wire                   m_axis_tuser
);

    localparam MTYW = $clog2(SEGB);
    localparam DW   = NSEG*SEGB*8;
    localparam KW   = NSEG*SEGB;

    wire [DW-1:0]        lane_tdata;
    wire [N-1:0]         lane_tvalid;
    wire [N*NSEG-1:0]    lane_ena, lane_sop, lane_eop, lane_err;
    wire [NSEG*MTYW-1:0] lane_mty;

    wire [N*NSEG*SEGB*8-1:0] axis_arb_tdata;
    wire [N*NSEG*SEGB-1:0]   axis_arb_tkeep;
    wire [N-1:0]             axis_arb_tvalid;
    wire [N-1:0]             axis_arb_tready;
    wire [N-1:0]             axis_arb_tlast;
    wire [N-1:0]             axis_arb_tuser;

    wire [N-1:0]             s_fifo_almost_full;

    assign s_axis_seg_tready = ~(|s_fifo_almost_full); // all fifos must have space

    seg_axis_split #(
        .NSEG(NSEG),
        .SEGB(SEGB),
        .N(N)
    ) seg_axis_split_inst (
        .clk(s_clk),
        .rst(s_rst),
        .s_axis_seg_tdata (s_axis_seg_tdata),
        .s_axis_seg_tvalid(s_axis_seg_tvalid && s_axis_seg_tready),
        .s_ena(s_ena),
        .s_sop(s_sop),
        .s_eop(s_eop),
        .s_err(s_err),
        .s_mty(s_mty),
        .m_axis_seg_tdata (lane_tdata),
        .m_axis_seg_tvalid(lane_tvalid),
        .m_ena(lane_ena),
        .m_sop(lane_sop),
        .m_eop(lane_eop),
        .m_err(lane_err),
        .m_mty(lane_mty)
    );

    generate
        for (genvar i = 0; i < N; i = i + 1) begin : g_lane
            unaligned_seg_axis_2_axis #(
                .NSEG(NSEG),
                .SEGB(SEGB),
                .FIFO_DEPTH(FIFO_DEPTH),
                .ASYNC_FIFO(ASYNC_FIFO)
            ) unaligned_seg_axis_2_axis_inst (
                .s_clk(s_clk),
                .s_rst(s_rst),
                .s_axis_seg_tdata (lane_tdata),
                .s_axis_seg_tvalid(lane_tvalid[i]),
                .s_ena(lane_ena[i*NSEG +: NSEG]),
                .s_sop(lane_sop[i*NSEG +: NSEG]),
                .s_eop(lane_eop[i*NSEG +: NSEG]),
                .s_err(lane_err[i*NSEG +: NSEG]),
                .s_mty(lane_mty),
                .m_clk(m_clk),
                .m_rst(m_rst),
                .m_axis_tdata (axis_arb_tdata[i*DW +: DW]),
                .m_axis_tkeep (axis_arb_tkeep[i*KW +: KW]),
                .m_axis_tvalid(axis_arb_tvalid[i]),
                .m_axis_tready(axis_arb_tready[i]),
                .m_axis_tlast (axis_arb_tlast[i]),
                .m_axis_tuser (axis_arb_tuser[i]),

                .s_fifo_almost_full(s_fifo_almost_full[i])
            );
        end
    endgenerate

    axis_arb_mux #(
        .S_COUNT             (N),
        .DATA_WIDTH          (DW),
        .KEEP_ENABLE         (1),
        .KEEP_WIDTH          (KW),
        .USER_ENABLE         (1),
        .USER_WIDTH          (1),
        .ID_ENABLE           (1),
        .UPDATE_TID          (1),
        .ARB_TYPE_ROUND_ROBIN(1)
    ) axis_mux_instance (
        .clk(m_clk),
        .rst(m_rst),
        .s_axis_tdata ({axis_arb_tdata }),
        .s_axis_tkeep ({axis_arb_tkeep }),
        .s_axis_tvalid({axis_arb_tvalid}),
        .s_axis_tready({axis_arb_tready}),
        .s_axis_tlast ({axis_arb_tlast }),
        .s_axis_tuser ({axis_arb_tuser }),
        .s_axis_tid(0),
        .s_axis_tdest(0),

        .m_axis_tdata (m_axis_tdata ),
        .m_axis_tkeep (m_axis_tkeep ),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast (m_axis_tlast ),
        .m_axis_tuser (m_axis_tuser )
    );

endmodule

`resetall
