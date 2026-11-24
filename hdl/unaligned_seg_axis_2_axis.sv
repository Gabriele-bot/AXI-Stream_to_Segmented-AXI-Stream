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
NSEG AXI segments of SEGB bytes -> 1 AXI Stream (NSEG*SEGB bytes wide).
Parametrized version of unaligned_axi_seg_2_axis / unaligned_axi_seg_16x64_2_axis.

The frame alignment (segment where the SOP lands) is tracked and the data is
shifted so that every frame starts at byte 0 of the AXI Stream.

Configurations:
  Xilinx 128b segments (SEGB=16): 100G NSEG=2 (256b), 200G NSEG=4 (512b), 400G NSEG=8 (1024b)
  Altera  64b segments (SEGB=8) : 100G NSEG=4 (256b), 200G NSEG=8 (512b), 400G NSEG=16 (1024b)

  ena[i] : segment i carries data
  sop[i] : segment i carries the first bytes of a frame
  eop[i] : segment i carries the last bytes of a frame
  err[i] : frame ending in segment i is bad
  mty[i] : number of unused bytes in segment i (0..SEGB-1), meaningful on eop segments
 */
module unaligned_seg_axis_2_axis #(
    parameter NSEG       = 8,
    parameter SEGB       = 16,   // bytes per segment
    parameter FIFO_DEPTH = 4096,
    parameter ASYNC_FIFO = 1
)(
    input  wire s_clk,
    input  wire s_rst,

    input  wire [NSEG*SEGB*8-1:0]       s_axis_seg_tdata,
    input  wire                         s_axis_seg_tvalid,
    input  wire [NSEG-1:0]              s_ena,
    input  wire [NSEG-1:0]              s_sop,
    input  wire [NSEG-1:0]              s_eop,
    input  wire [NSEG-1:0]              s_err,
    input  wire [NSEG*$clog2(SEGB)-1:0] s_mty,

    input  wire m_clk,
    input  wire m_rst,

    output wire [NSEG*SEGB*8-1:0]  m_axis_tdata,
    output wire [NSEG*SEGB-1:0]    m_axis_tkeep,
    output wire                    m_axis_tvalid,
    input  wire                    m_axis_tready,
    output wire                    m_axis_tlast,
    output wire                    m_axis_tuser,

    output wire                    s_fifo_almost_full // DEPTH - 3 words
);

    localparam SEGW = SEGB*8;
    localparam DW   = NSEG*SEGW;
    localparam KW   = NSEG*SEGB;
    localparam IDXW = $clog2(NSEG);
    localparam MTYW = $clog2(SEGB);

    reg [DW-1:0]  m_axis_tdata_int;
    reg [KW-1:0]  m_axis_tkeep_int;
    reg           m_axis_tvalid_int;
    reg           m_axis_tlast_int;
    reg           m_axis_tuser_int;

    reg [DW-1:0]  m_axis_tdata_reg;
    reg [KW-1:0]  m_axis_tkeep_reg;
    reg           m_axis_tvalid_reg;
    reg           m_axis_tlast_reg;
    reg           m_axis_tuser_reg;

    wire          m_axis_fifo_tready; // not used

    reg [IDXW-1:0] current_align_reg, current_align_next;
    reg [IDXW-1:0] current_align_reg_del;

    reg [DW-1:0]        s_axis_seg_tdata_del;
    reg                 s_axis_seg_tvalid_del;
    reg [NSEG-1:0]      s_ena_del;
    reg [NSEG-1:0]      s_sop_del;
    reg [NSEG-1:0]      s_eop_del;
    reg [NSEG-1:0]      s_err_del;
    reg [NSEG*MTYW-1:0] s_mty_del;

    reg [DW-1:0]        shift_axi_seg_tdata;
    reg                 shift_axi_seg_tvalid;
    reg [NSEG-1:0]      shift_axi_seg_ena;
    reg [NSEG-1:0]      shift_axi_seg_sop;
    reg [NSEG-1:0]      shift_axi_seg_eop;
    reg [NSEG-1:0]      shift_axi_seg_err;
    reg [NSEG*MTYW-1:0] shift_axi_seg_mty;

    wire [KW-1:0]       shift_axis_tkeep;

    reg [DW-1:0]        save_axi_seg_tdata_reg;
    reg [NSEG-1:0]      save_axi_seg_ena_reg;
    reg [NSEG-1:0]      save_axi_seg_sop_reg;
    reg [NSEG-1:0]      save_axi_seg_eop_reg;
    reg [NSEG-1:0]      save_axi_seg_err_reg;
    reg [NSEG*MTYW-1:0] save_axi_seg_mty_reg;

    reg shift_axi_seg_extra_cycle_reg;

    reg transfer_in_save;
    reg flush_save;

    reg enable_read_reg, enable_read_next;

    wire [$clog2(FIFO_DEPTH):0] s_status_depth;

    assign s_fifo_almost_full = s_status_depth >= FIFO_DEPTH - 5*KW; // do not change this, used downstream as tready handshake

    // exactly one bit set
    function is_onehot;
        input [NSEG-1:0] x;
        begin
            is_onehot = (x != {NSEG{1'b0}}) && ((x & (x - 1'b1)) == {NSEG{1'b0}});
        end
    endfunction

    function [IDXW-1:0] onehot2idx;
        input [NSEG-1:0] x;
        integer i;
        begin
            onehot2idx = {IDXW{1'b0}};
            for (i = 0; i < NSEG; i = i + 1) begin
                if (x[i]) onehot2idx = i[IDXW-1:0];
            end
        end
    endfunction

    // shift stage: the shift amount is the alignment of the current (or extra-cycle) beat
    reg [IDXW-1:0] shift_align;

    always @(*) begin

        m_axis_tdata_int  = {DW{1'b0}};
        m_axis_tkeep_int  = {KW{1'b0}};
        m_axis_tvalid_int = 1'b0;
        m_axis_tlast_int  = 1'b0;
        m_axis_tuser_int  = 1'b0;

        enable_read_next = enable_read_reg;

        // track the alignment: a beat with a single SOP defines the new alignment
        if (s_axis_seg_tvalid && is_onehot(s_ena & s_sop)) begin
            current_align_next = onehot2idx(s_ena & s_sop);
        end else begin
            current_align_next = current_align_reg;
        end

        shift_align = shift_axi_seg_extra_cycle_reg ? current_align_reg_del : current_align_reg;

        if (shift_align == {IDXW{1'b0}}) begin
            // passthrough if no overlap
            shift_axi_seg_tdata = s_axis_seg_tdata_del;
            shift_axi_seg_ena   = s_ena_del;
            shift_axi_seg_sop   = s_sop_del;
            shift_axi_seg_eop   = s_eop_del;
            shift_axi_seg_err   = s_err_del;
            shift_axi_seg_mty   = s_mty_del;
        end else begin
            shift_axi_seg_tdata = {s_axis_seg_tdata_del, save_axi_seg_tdata_reg} >> (shift_align * SEGW);
            shift_axi_seg_ena   = {s_ena_del,            save_axi_seg_ena_reg  } >> shift_align;
            shift_axi_seg_sop   = {s_sop_del,            save_axi_seg_sop_reg  } >> shift_align;
            shift_axi_seg_eop   = {s_eop_del,            save_axi_seg_eop_reg  } >> shift_align;
            shift_axi_seg_err   = {s_err_del,            save_axi_seg_err_reg  } >> shift_align;
            shift_axi_seg_mty   = {s_mty_del,            save_axi_seg_mty_reg  } >> (shift_align * MTYW);
        end
        shift_axi_seg_tvalid = shift_axi_seg_extra_cycle_reg ? 1'b1 : s_axis_seg_tvalid_del;

        flush_save = 1'b0;
        transfer_in_save = 1'b0;

        if (s_axis_seg_tvalid_del || shift_axi_seg_extra_cycle_reg) begin
            transfer_in_save = 1'b1;

            if (current_align_reg != {IDXW{1'b0}}) begin
                enable_read_next = 1'b1;
            end

            if (|(shift_axi_seg_eop & shift_axi_seg_ena) & shift_axi_seg_tvalid) begin
                if (!(|(s_sop_del & s_ena_del) & s_axis_seg_tvalid_del)) begin
                    flush_save = 1'b1;
                    enable_read_next = 1'b0;
                end
            end
        end

        if (enable_read_reg || (current_align_reg == {IDXW{1'b0}})) begin
            m_axis_tdata_int  = shift_axi_seg_tdata;
            m_axis_tkeep_int  = shift_axis_tkeep;
            m_axis_tvalid_int = |(shift_axi_seg_ena) & shift_axi_seg_tvalid;
            m_axis_tlast_int  = |(shift_axi_seg_ena & shift_axi_seg_eop) & shift_axi_seg_tvalid;
            m_axis_tuser_int  = |(shift_axi_seg_ena & shift_axi_seg_err) & shift_axi_seg_tvalid;
        end

    end

    genvar k;
    generate
        for (k = 0; k < NSEG; k = k + 1) begin : g_keep
            mapmty2tkeep #(
                .SEGB(SEGB)
                ) mapmty2tkeep_instance (
                .s_mty       (shift_axi_seg_mty[MTYW*k +: MTYW]),
                .s_ena       (shift_axi_seg_ena[k] & shift_axi_seg_tvalid),
                .m_axis_tkeep(shift_axis_tkeep[SEGB*k +: SEGB])
            );
        end
    endgenerate

    always @(posedge s_clk) begin
        if (s_rst) begin
            enable_read_reg <= 1'b0;

            s_axis_seg_tdata_del  <= {DW{1'b0}};
            s_axis_seg_tvalid_del <= 1'b0;
            s_ena_del <= {NSEG{1'b0}};
            s_sop_del <= {NSEG{1'b0}};
            s_eop_del <= {NSEG{1'b0}};
            s_err_del <= {NSEG{1'b0}};
            s_mty_del <= {NSEG*MTYW{1'b0}};

            current_align_reg     <= {IDXW{1'b0}};
            current_align_reg_del <= {IDXW{1'b0}};

            save_axi_seg_tdata_reg <= {DW{1'b0}};
            save_axi_seg_ena_reg   <= {NSEG{1'b0}};
            save_axi_seg_sop_reg   <= {NSEG{1'b0}};
            save_axi_seg_eop_reg   <= {NSEG{1'b0}};
            save_axi_seg_err_reg   <= {NSEG{1'b0}};
            save_axi_seg_mty_reg   <= {NSEG*MTYW{1'b0}};

            shift_axi_seg_extra_cycle_reg <= 1'b0;

            m_axis_tdata_reg  <= {DW{1'b0}};
            m_axis_tkeep_reg  <= {KW{1'b0}};
            m_axis_tvalid_reg <= 1'b0;
            m_axis_tlast_reg  <= 1'b0;
            m_axis_tuser_reg  <= 1'b0;

        end else begin
            s_axis_seg_tdata_del  <= s_axis_seg_tdata;
            s_axis_seg_tvalid_del <= s_axis_seg_tvalid;
            s_ena_del <= s_ena;
            s_sop_del <= s_sop;
            s_eop_del <= s_eop;
            s_err_del <= s_err;
            s_mty_del <= s_mty;

            enable_read_reg <= enable_read_next;

            current_align_reg     <= current_align_next;
            current_align_reg_del <= current_align_reg;

            if (transfer_in_save) begin
                save_axi_seg_tdata_reg <= s_axis_seg_tdata_del;
                save_axi_seg_ena_reg   <= s_ena_del;
                save_axi_seg_sop_reg   <= s_sop_del;
                save_axi_seg_err_reg   <= s_err_del;
                save_axi_seg_mty_reg   <= s_mty_del;
            end

            if (flush_save) begin
                save_axi_seg_eop_reg          <= {NSEG{1'b0}};
                shift_axi_seg_extra_cycle_reg <= 1'b0;
            end else if (transfer_in_save) begin
                save_axi_seg_eop_reg          <= s_eop_del;
                shift_axi_seg_extra_cycle_reg <= current_align_reg != {IDXW{1'b0}} ? |(s_ena_del & s_eop_del) : 1'b0;
            end

            m_axis_tdata_reg  <= m_axis_tdata_int ;
            m_axis_tkeep_reg  <= m_axis_tkeep_int ;
            m_axis_tvalid_reg <= m_axis_tvalid_int;
            m_axis_tlast_reg  <= m_axis_tlast_int ;
            m_axis_tuser_reg  <= m_axis_tuser_int ;
        end
    end

    generate
        if (ASYNC_FIFO) begin
            axis_async_fifo #(
                .DEPTH(FIFO_DEPTH),
                .DATA_WIDTH(DW),
                .KEEP_ENABLE(1),
                .KEEP_WIDTH(KW),
                .RAM_PIPELINE(1),
                .USER_ENABLE(1),
                .USER_WIDTH(1),
                .DROP_WHEN_FULL(0),
                .FRAME_FIFO(0)
            ) axis_fifo_instance (
                .s_clk(s_clk),
                .s_rst(s_rst),
                .s_axis_tdata (m_axis_tdata_reg),
                .s_axis_tkeep (m_axis_tkeep_reg),
                .s_axis_tvalid(m_axis_tvalid_reg),
                .s_axis_tready(m_axis_fifo_tready),
                .s_axis_tlast (m_axis_tlast_reg),
                .s_axis_tid   (0),
                .s_axis_tdest (0),
                .s_axis_tuser (m_axis_tuser_reg),

                .m_clk(m_clk),
                .m_rst(m_rst),
                .m_axis_tdata (m_axis_tdata),
                .m_axis_tkeep (m_axis_tkeep),
                .m_axis_tvalid(m_axis_tvalid),
                .m_axis_tready(m_axis_tready),
                .m_axis_tlast (m_axis_tlast),
                .m_axis_tid   (),
                .m_axis_tdest (),
                .m_axis_tuser (m_axis_tuser),

                .s_status_depth(s_status_depth)
            );
        end else begin
            axis_fifo #(
                .DEPTH(FIFO_DEPTH),
                .DATA_WIDTH(DW),
                .KEEP_ENABLE(1),
                .KEEP_WIDTH(KW),
                .RAM_PIPELINE(1),
                .USER_ENABLE(1),
                .USER_WIDTH(1),
                .DROP_WHEN_FULL(0),
                .FRAME_FIFO(0)
            ) axis_fifo_instance (
                .clk(m_clk),
                .rst(m_rst),
                .s_axis_tdata (m_axis_tdata_reg),
                .s_axis_tkeep (m_axis_tkeep_reg),
                .s_axis_tvalid(m_axis_tvalid_reg),
                .s_axis_tready(m_axis_fifo_tready),
                .s_axis_tlast (m_axis_tlast_reg),
                .s_axis_tid   (0),
                .s_axis_tdest (0),
                .s_axis_tuser (m_axis_tuser_reg),

                .m_axis_tdata (m_axis_tdata),
                .m_axis_tkeep (m_axis_tkeep),
                .m_axis_tvalid(m_axis_tvalid),
                .m_axis_tready(m_axis_tready),
                .m_axis_tlast (m_axis_tlast),
                .m_axis_tid   (),
                .m_axis_tdest (),
                .m_axis_tuser (m_axis_tuser),

                .status_depth(s_status_depth)
            );
        end
    endgenerate

endmodule

`resetall
