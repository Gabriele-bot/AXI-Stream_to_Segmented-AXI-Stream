// SPDX-License-Identifier: CERN-OHL-P-2.0
// Copyright 2026 Gabriele Bortolato
// Source Location: https://github.com/Gabriele-bot/AXI-Stream_to_Segmented-AXI-Stream
// This source describes Open Hardware and is licensed under the CERN-OHL-P v2.
// You may redistribute and modify this documentation and make products using it
// under the terms of the CERN-OHL-P v2 (https://ohwr.org/cern_ohl_p_v2.txt).
// This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
// INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE.
// Please see the CERN-OHL-P v2 for applicable conditions.
/*
NSEG --> Number of Segments
SEGB --> Bytes per Segments 
e.g. Xilinx 400G DCMAC                NSEG=8, SEGB=16
e.g. Altera 400G F-Tile Hard Ethernet NSEG=16, SEGB=8

Input AXI-Stream in packet mode required (valid stays asserted for the whole frame)

Output async fifo for the clock domain crossing and the back pressure generation 
*/

module axis_2_seg_axis_wrapper #(
    parameter NSEG       = 8,
    parameter SEGB       = 16, // bytes per segment
    //simple implementation
    parameter SIMPLE = 0,
    // input axis register
    parameter INPUT_REGS = 1,
    // Async options
    parameter IS_ASYNC  = 1,
    // Async fifo depth in number of entries
    parameter ASYNC_FIFO_DEPTH = 8
)(
    input  wire s_clk,
    input  wire s_rst,

    input  wire [NSEG*SEGB*8-1:0] s_axis_tdata,
    input  wire [NSEG*SEGB-1:0]   s_axis_tkeep,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,
    input  wire                   s_axis_tlast,
    input  wire                   s_axis_tuser,

    input  wire m_clk,
    input  wire m_rst,

    output wire [NSEG*SEGB*8-1:0]       m_axis_seg_tdata,
    output wire                         m_axis_seg_tvalid,
    input  wire                         m_axis_seg_tready,
    output wire [NSEG-1:0]              m_axis_seg_tuser_ena,
    output wire [NSEG-1:0]              m_axis_seg_tuser_sop,
    output wire [NSEG-1:0]              m_axis_seg_tuser_eop,
    output wire [NSEG-1:0]              m_axis_seg_tuser_err,
    output wire [NSEG*$clog2(SEGB)-1:0] m_axis_seg_tuser_mty
);

    wire [NSEG*SEGB*8-1:0]       axis_seg_tdata;
    wire                         axis_seg_tvalid;
    wire                         axis_seg_tready;
    wire [NSEG-1:0]              axis_seg_tuser_ena;
    wire [NSEG-1:0]              axis_seg_tuser_sop;
    wire [NSEG-1:0]              axis_seg_tuser_eop;
    wire [NSEG-1:0]              axis_seg_tuser_err;
    wire [NSEG*$clog2(SEGB)-1:0] axis_seg_tuser_mty;

    generate
        if (SIMPLE) begin
            axis_2_seg_axis_simple #(
                .NSEG(NSEG),
                .SEGB(SEGB),
                .INPUT_REGS(INPUT_REGS)
            ) axis_2_seg_axis_instance (
                .clk(s_clk),
                .rst(s_rst),

                .s_axis_tdata (s_axis_tdata),
                .s_axis_tkeep (s_axis_tkeep),
                .s_axis_tvalid(s_axis_tvalid),
                .s_axis_tready(s_axis_tready),
                .s_axis_tlast (s_axis_tlast),
                .s_axis_tuser (s_axis_tuser),

                .m_axis_seg_tdata    (axis_seg_tdata),
                .m_axis_seg_tvalid   (axis_seg_tvalid),
                .m_axis_seg_tready   (axis_seg_tready),
                .m_axis_seg_tuser_ena(axis_seg_tuser_ena),
                .m_axis_seg_tuser_sop(axis_seg_tuser_sop),
                .m_axis_seg_tuser_eop(axis_seg_tuser_eop),
                .m_axis_seg_tuser_err(axis_seg_tuser_err),
                .m_axis_seg_tuser_mty(axis_seg_tuser_mty)
            );
        end else begin
            axis_2_seg_axis #(
                .NSEG(NSEG),
                .SEGB(SEGB),
                .INPUT_REGS(INPUT_REGS)
            ) axis_2_seg_axis_instance (
                .clk(s_clk),
                .rst(s_rst),

                .s_axis_tdata (s_axis_tdata),
                .s_axis_tkeep (s_axis_tkeep),
                .s_axis_tvalid(s_axis_tvalid),
                .s_axis_tready(s_axis_tready),
                .s_axis_tlast (s_axis_tlast),
                .s_axis_tuser (s_axis_tuser),

                .m_axis_seg_tdata    (axis_seg_tdata),
                .m_axis_seg_tvalid   (axis_seg_tvalid),
                .m_axis_seg_tready   (axis_seg_tready),
                .m_axis_seg_tuser_ena(axis_seg_tuser_ena),
                .m_axis_seg_tuser_sop(axis_seg_tuser_sop),
                .m_axis_seg_tuser_eop(axis_seg_tuser_eop),
                .m_axis_seg_tuser_err(axis_seg_tuser_err),
                .m_axis_seg_tuser_mty(axis_seg_tuser_mty)
            );
        end
        
        if (IS_ASYNC) begin
            axis_async_fifo #(
                .DEPTH(ASYNC_FIFO_DEPTH),
                .DATA_WIDTH(NSEG*SEGB*8),
                .KEEP_ENABLE(0),
                .ID_ENABLE(0),
                .DEST_ENABLE(0),
                .USER_ENABLE(1),
                .USER_WIDTH(NSEG*4+NSEG*$clog2(SEGB)), // ena, sop, eop, err and mty
                .LAST_ENABLE(0),
                .RAM_PIPELINE(1),
                .FRAME_FIFO(0)
            ) axis_async_out_fifo_inst (
                .s_clk(s_clk),
                .s_rst(s_rst),

                .s_axis_tdata (axis_seg_tdata),
                .s_axis_tvalid(axis_seg_tvalid),
                .s_axis_tready(axis_seg_tready),
                .s_axis_tuser ({
                axis_seg_tuser_ena,
                axis_seg_tuser_sop,
                axis_seg_tuser_eop,
                axis_seg_tuser_err,
                axis_seg_tuser_mty
                }),
                .s_axis_tlast (0),
                .s_axis_tkeep (0),
                .s_axis_tid   (0),
                .s_axis_tdest (0),

                .m_clk(m_clk),
                .m_rst(m_rst),

                .m_axis_tdata (m_axis_seg_tdata),
                .m_axis_tvalid(m_axis_seg_tvalid),
                .m_axis_tready(m_axis_seg_tready),
                .m_axis_tuser ({
                m_axis_seg_tuser_ena,
                m_axis_seg_tuser_sop,
                m_axis_seg_tuser_eop,
                m_axis_seg_tuser_err,
                m_axis_seg_tuser_mty
                })
            );
        end else begin
            assign m_axis_seg_tdata     = axis_seg_tdata;
            assign m_axis_seg_tvalid    = axis_seg_tvalid;
            assign axis_seg_tready      = m_axis_seg_tready;
            assign m_axis_seg_tuser_ena = axis_seg_tuser_ena;
            assign m_axis_seg_tuser_sop = axis_seg_tuser_sop;
            assign m_axis_seg_tuser_eop = axis_seg_tuser_eop;
            assign m_axis_seg_tuser_err = axis_seg_tuser_err;
            assign m_axis_seg_tuser_mty = axis_seg_tuser_mty;
        end
    endgenerate




endmodule