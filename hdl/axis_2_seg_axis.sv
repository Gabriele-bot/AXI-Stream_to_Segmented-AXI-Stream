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
NSEG --> Number of Segments
SEGB --> Bytes per Segments 
e.g. Xilinx 400G DCMAC                NSEG=8, SEGB=16
e.g. Altera 400G F-Tile Hard Ethernet NSEG=16, SEGB=8

Full conversion AXI-Stream to AXI-Segment.
Expected input is an uniterrupted AXIS-Stream packet, valid stais asserted for the whol packet.
Min Frame length set to 60 Bytes.

Flow
AXI-Stream pipeline Registers
AXI-Stream to AXI-Segemnt input control 
AXI-Stream fifos (1 for each segment)
AXI-Segment reorder and backpressure managment

E.G.
+----------------------+
|        INPUT         | 
+----------------------+

AXI_Stream input
beat 0     1     2     3     4     5     
     +-----+-----+-----+-----+-----+-----+ 
     |  A0 |  B0 |  C0 |  C4 |  D0 |     | 
     +-----+-----+-----+-----+-----+-----+ 
     |  A1 |  B1 |  C1 |  C5 |  D1 |     | 
     +-----+-----+-----+-----+-----+-----+ 
     |  A2 |     |  C2 |  C6 |  D2 |     | 
     +-----+-----+-----+-----+-----+-----+ 
     |     |     |  C3 |  C7 |     |     | 
     +-----+-----+-----+-----+-----+-----+ 

+----------------------+
|     PRE_ALIGN        | 
+----------------------+
beat 0     1     2     3     4     5     
     +-----+-----+-----+-----+-----+-----+
     |  A0 |  B1 |  C3 |  C7 |     |     |
     +-----+-----+-----+-----+-----+-----+
     |  A1 |     |  C0 |  C4 |  D0 |     |
     +-----+-----+-----+-----+-----+-----+
     |  A2 |     |  C1 |  C5 |  D1 |     |
     +-----+-----+-----+-----+-----+-----+
     |     |  B0 |  C2 |  C6 |  D2 |     |
     +-----+-----+-----+-----+-----+-----+
 
+----------------------+
|    ALIGN & REORDER   | 
+----------------------+
AXI-Stream Segments reorder (post-alignment)
AXI-Stream Segements are sent togheter only if backpressure is expirienced
(Feed data faster than the sink can accept)
beat 0     1     2     3     4     5     
     +-----+-----+-----+-----+-----+-----+
     |     |  A0 |  B0 |  C2 |  C6 |  D2 |
     +-----+-----+-----+-----+-----+-----+
     |     |  A1 |  B1 |  C3 |  C7 |     |
     +-----+-----+-----+-----+-----+-----+
     |     |  A2 |  C0 |  C4 |  D0 |     |
     +-----+-----+-----+-----+-----+-----+
     |     |     |  C1 |  C5 |  D1 |     |
     +-----+-----+-----+-----+-----+-----+
 */
 
module axis_2_seg_axis #(
    parameter NSEG       = 8,
    parameter SEGB       = 16, // bytes per segment
    // input axis register
    parameter INPUT_REGS = 1
)(
    input  wire clk,
    input  wire rst,

    input  wire [NSEG*SEGB*8-1:0] s_axis_tdata,
    input  wire [NSEG*SEGB-1:0]   s_axis_tkeep,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,
    input  wire                   s_axis_tlast,
    input  wire                   s_axis_tuser,

    output wire [NSEG*SEGB*8-1:0]       m_axis_seg_tdata,
    output wire                         m_axis_seg_tvalid,
    input  wire                         m_axis_seg_tready,
    output wire [NSEG-1:0]              m_axis_seg_tuser_ena,
    output wire [NSEG-1:0]              m_axis_seg_tuser_sop,
    output wire [NSEG-1:0]              m_axis_seg_tuser_eop,
    output wire [NSEG-1:0]              m_axis_seg_tuser_err,
    output wire [NSEG*$clog2(SEGB)-1:0] m_axis_seg_tuser_mty
);

    localparam SEGW = SEGB*8;
    localparam DW   = NSEG*SEGW;
    localparam KW   = NSEG*SEGB;
    localparam IDXW = $clog2(NSEG);
    localparam MTYW = $clog2(SEGB);

    // ------------------------------------------------------------------
    // input register + segment/offset control
    // ------------------------------------------------------------------
    wire [DW-1:0]    m_axis_input_reg_tdata;
    wire [KW-1:0]    m_axis_input_reg_tkeep;
    wire             m_axis_input_reg_tvalid;
    wire             m_axis_input_reg_tready;
    wire             m_axis_input_reg_tlast;
    wire             m_axis_input_reg_tuser;

    wire [DW-1:0]         s_fifo_axis_seg_tdata;
    wire                  s_fifo_axis_seg_tvalid;
    wire                  s_fifo_axis_seg_tready;
    wire [NSEG-1:0]       s_fifo_axis_seg_tuser_ena;
    wire [NSEG-1:0]       s_fifo_axis_seg_tuser_sop;
    wire [NSEG-1:0]       s_fifo_axis_seg_tuser_eop;
    wire [NSEG-1:0]       s_fifo_axis_seg_tuser_err;
    wire [MTYW*NSEG-1:0]  s_fifo_axis_seg_tuser_mty;

    wire [NSEG-1:0]  fifo_tready;
    assign s_fifo_axis_seg_tready = &fifo_tready;

    generate
        if (INPUT_REGS > 0) begin

            axis_pipeline_register #(
                .DATA_WIDTH(DW),
                .KEEP_ENABLE(1),
                .KEEP_WIDTH(KW),
                .LAST_ENABLE(1),
                .USER_ENABLE(1),
                .USER_WIDTH(1),
                .REG_TYPE(2),
                .LENGTH(INPUT_REGS)
            ) axis_pipeline_register_instance (
                .clk(clk),
                .rst(rst),
                .s_axis_tdata (s_axis_tdata),
                .s_axis_tkeep (s_axis_tkeep),
                .s_axis_tvalid(s_axis_tvalid),
                .s_axis_tready(s_axis_tready),
                .s_axis_tlast (s_axis_tlast),
                .s_axis_tid   (0),
                .s_axis_tdest (0),
                .s_axis_tuser (s_axis_tuser),
                .m_axis_tdata (m_axis_input_reg_tdata),
                .m_axis_tkeep (m_axis_input_reg_tkeep),
                .m_axis_tvalid(m_axis_input_reg_tvalid),
                .m_axis_tready(m_axis_input_reg_tready),
                .m_axis_tlast (m_axis_input_reg_tlast),
                .m_axis_tuser (m_axis_input_reg_tuser)
            );
        end else begin

            assign m_axis_input_reg_tdata  = s_axis_tdata;
            assign m_axis_input_reg_tkeep  = s_axis_tkeep;
            assign m_axis_input_reg_tvalid = s_axis_tvalid;
            assign s_axis_tready           = m_axis_input_reg_tready;
            assign m_axis_input_reg_tlast  = s_axis_tlast;
            assign m_axis_input_reg_tuser  = s_axis_tuser;

        end
    endgenerate

    axis_2_seg_axis_input_ctrl #(
        .NSEG(NSEG),
        .SEGB(SEGB)
    ) axis_2_seg_axis_input_ctrl_instance (
        .clk(clk),
        .rst(rst),

        .s_axis_tdata (m_axis_input_reg_tdata),
        .s_axis_tkeep (m_axis_input_reg_tkeep),
        .s_axis_tvalid(m_axis_input_reg_tvalid),
        .s_axis_tready(m_axis_input_reg_tready),
        .s_axis_tlast (m_axis_input_reg_tlast),
        .s_axis_tuser (m_axis_input_reg_tuser),

        .m_axis_seg_tdata     (s_fifo_axis_seg_tdata),
        .m_axis_seg_tvalid    (s_fifo_axis_seg_tvalid),
        .m_axis_seg_tready    (s_fifo_axis_seg_tready),
        .m_axis_seg_tuser_ena (s_fifo_axis_seg_tuser_ena),
        .m_axis_seg_tuser_sop (s_fifo_axis_seg_tuser_sop),
        .m_axis_seg_tuser_eop (s_fifo_axis_seg_tuser_eop),
        .m_axis_seg_tuser_err (s_fifo_axis_seg_tuser_err),
        .m_axis_seg_tuser_mty (s_fifo_axis_seg_tuser_mty)
    );

    // ------------------------------------------------------------------
    // one FIFO per segment, only enabled segments are written
    // ------------------------------------------------------------------
    wire [NSEG-1:0]              fifo_tvalid_out;
    wire [NSEG-1:0]              fifo_tready_out;
    wire [NSEG-1:0][SEGW-1:0]    fifo_tdata_out;

    wire [NSEG-1:0]              ena_fifo_out, sop_fifo_out, eop_fifo_out, err_fifo_out;
    wire [NSEG-1:0][MTYW-1:0]    mty_fifo_out;

    generate
        for (genvar i = 0; i < NSEG; i = i + 1) begin : g_seg_fifo
            axis_fifo #(
                .DEPTH(64),
                .DATA_WIDTH(SEGW),
                .LAST_ENABLE(0),
                .ID_ENABLE(0),
                .DEST_ENABLE(0),
                .USER_ENABLE(1),
                .USER_WIDTH(4+MTYW),
                .RAM_PIPELINE(3),
                .FRAME_FIFO(0)
            ) segment_fifo (
                .clk(clk),
                .rst(rst),

                // AXI input
                .s_axis_tdata (s_fifo_axis_seg_tdata[SEGW*i +: SEGW]),
                .s_axis_tkeep (0),
                .s_axis_tvalid(s_fifo_axis_seg_tuser_ena[i] & s_fifo_axis_seg_tvalid & s_fifo_axis_seg_tready),
                .s_axis_tready(fifo_tready[i]),
                .s_axis_tlast (0),
                .s_axis_tid   (0),
                .s_axis_tdest (0),
                .s_axis_tuser ({s_fifo_axis_seg_tuser_ena[i], s_fifo_axis_seg_tuser_sop[i], s_fifo_axis_seg_tuser_eop[i], s_fifo_axis_seg_tuser_err[i], s_fifo_axis_seg_tuser_mty[MTYW*i +: MTYW]}),

                // AXI output
                .m_axis_tdata (fifo_tdata_out[i]),
                .m_axis_tvalid(fifo_tvalid_out[i]),
                .m_axis_tready(fifo_tready_out[i]),
                .m_axis_tuser ({ena_fifo_out[i], sop_fifo_out[i], eop_fifo_out[i], err_fifo_out[i], mty_fifo_out[i]})
            );
        end
    endgenerate

    // ------------------------------------------------------------------
    // output rotation: find where the next frame starts
    // ------------------------------------------------------------------
    reg [IDXW-1:0]            out_offset_reg = {IDXW{1'b0}}, out_offset_next;
    reg [NSEG-1:0][IDXW-1:0]  out_index_reg = {NSEG*IDXW{1'b0}};
    reg [NSEG-1:0][IDXW-1:0]  out_index_before_reg = {NSEG*IDXW{1'b0}}, out_index_next;

    reg [NSEG-1:0]      eop_vis, sop_vis;
    reg [2*NSEG-1:0]    temp_eop_ext, temp_sop_ext;
    reg [NSEG-1:0]      temp_eop, temp_sop;
    reg                 eop_found;
    reg [IDXW-1:0]      eop_hi, eop_next_seg;
    integer             k;

    always @(*) begin
        out_offset_next = out_offset_reg;
        out_index_next  = out_index_before_reg;

        eop_vis = ena_fifo_out & eop_fifo_out & fifo_tvalid_out & fifo_tready_out;
        sop_vis = ena_fifo_out & sop_fifo_out & fifo_tvalid_out & fifo_tready_out;

        // rotate right by out_offset_reg inside NSEG bits
        temp_eop_ext = {eop_vis, {NSEG{1'b0}}} >> out_offset_reg;
        temp_eop     = temp_eop_ext[2*NSEG-1:NSEG] | temp_eop_ext[NSEG-1:0];

        temp_sop_ext = {sop_vis, {NSEG{1'b0}}} >> out_offset_reg;
        temp_sop     = temp_sop_ext[2*NSEG-1:NSEG] | temp_sop_ext[NSEG-1:0];

        // highest eop in the rotated view
        eop_found = 1'b0;
        eop_hi    = {IDXW{1'b0}};
        for (k = 0; k < NSEG; k = k + 1) begin
            if (temp_eop[k]) begin
                eop_found = 1'b1;
                eop_hi    = k[IDXW-1:0];
            end
        end
        eop_next_seg = eop_hi + 1'b1; // wraps NSEG-1 -> 0

        if (&(ena_fifo_out & fifo_tvalid_out & fifo_tready_out)) begin
            // all fifos have data, no need for a new out offset
            out_offset_next = out_offset_reg;
        end else begin
            if (eop_found) begin
                if (temp_sop[eop_next_seg]) begin
                    // new subsequent frame starts right here, no need for a new offset
                    out_offset_next = out_offset_reg;
                end else begin
                    out_offset_next = out_offset_reg + eop_next_seg;
                end
            end else begin
                out_offset_next = out_offset_reg;
            end
            for (k = 0; k < NSEG; k = k + 1) begin
                out_index_next[k] = out_offset_next + k;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            out_offset_reg       <= {IDXW{1'b0}};
            out_index_before_reg <= {NSEG*IDXW{1'b0}};
        end else begin
            out_offset_reg       <= out_offset_next;
            out_index_before_reg <= out_index_next;
        end
    end

    // ------------------------------------------------------------------
    // skid buffer, no bubble cycles
    // ------------------------------------------------------------------
    reg                       fifo_out_axis_tready_reg = 1'b0;

    reg [NSEG-1:0][SEGW-1:0]  m_axis_seg_tdata_reg = {DW{1'b0}};
    reg                       m_axis_tvalid_reg = 1'b0, m_axis_tvalid_next;
    reg [NSEG-1:0]            m_ena_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]            m_sop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]            m_eop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]            m_err_reg = {NSEG{1'b0}};
    reg [NSEG-1:0][MTYW-1:0]  m_mty_reg = {NSEG*MTYW{1'b0}};

    reg [NSEG-1:0][SEGW-1:0]  temp_m_axis_seg_tdata_reg = {DW{1'b0}};
    reg                       temp_m_axis_tvalid_reg = 1'b0, temp_m_axis_tvalid_next;
    reg [NSEG-1:0]            temp_m_ena_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]            temp_m_sop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]            temp_m_eop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]            temp_m_err_reg = {NSEG{1'b0}};
    reg [NSEG-1:0][MTYW-1:0]  temp_m_mty_reg = {NSEG*MTYW{1'b0}};
    reg [NSEG-1:0][IDXW-1:0]  temp_out_index_reg = {NSEG*IDXW{1'b0}};

    // datapath control
    reg store_axis_input_to_output;
    reg store_axis_input_to_temp;
    reg store_axis_temp_to_output;

    assign fifo_tready_out = {NSEG{fifo_out_axis_tready_reg}};

    // enable ready input next cycle if output is ready or the temp reg will not be filled on the next cycle (output reg empty or no input)
    wire fifo_out_axis_tready_early = m_axis_seg_tready || (!temp_m_axis_tvalid_reg && (!m_axis_tvalid_reg || !(|fifo_tvalid_out)));

    always @* begin
        // transfer sink ready state to source
        m_axis_tvalid_next      = m_axis_tvalid_reg;
        temp_m_axis_tvalid_next = temp_m_axis_tvalid_reg;

        store_axis_input_to_output = 1'b0;
        store_axis_input_to_temp   = 1'b0;
        store_axis_temp_to_output  = 1'b0;

        if (fifo_out_axis_tready_reg) begin
            // input is ready
            if (m_axis_seg_tready || !m_axis_tvalid_reg) begin
                // output is ready or currently not valid, transfer data to output
                m_axis_tvalid_next = |fifo_tvalid_out;
                store_axis_input_to_output = 1'b1;
            end else begin
                // output is not ready, store input in temp
                temp_m_axis_tvalid_next = |fifo_tvalid_out;
                store_axis_input_to_temp = 1'b1;
            end
        end else if (m_axis_seg_tready) begin
            // input is not ready, but output is ready
            m_axis_tvalid_next      = temp_m_axis_tvalid_reg;
            temp_m_axis_tvalid_next = 1'b0;
            store_axis_temp_to_output = 1'b1;
        end
    end

    integer j;

    always @(posedge clk) begin
        fifo_out_axis_tready_reg <= fifo_out_axis_tready_early;
        m_axis_tvalid_reg        <= m_axis_tvalid_next;
        temp_m_axis_tvalid_reg   <= temp_m_axis_tvalid_next;

        // datapath
        if (store_axis_input_to_output) begin
            for (j = 0; j < NSEG; j = j + 1) begin
                m_axis_seg_tdata_reg[j] <= fifo_tvalid_out[j] ? fifo_tdata_out[j] : {SEGW{1'b0}};
                m_mty_reg[j]            <= fifo_tvalid_out[j] ? mty_fifo_out[j]   : {MTYW{1'b0}};
            end

            m_ena_reg <= ena_fifo_out & fifo_tvalid_out;
            m_sop_reg <= sop_fifo_out & fifo_tvalid_out;
            m_eop_reg <= eop_fifo_out & fifo_tvalid_out;
            m_err_reg <= err_fifo_out & fifo_tvalid_out;

            out_index_reg <= out_index_before_reg;
        end else if (store_axis_temp_to_output) begin
            m_axis_seg_tdata_reg <= temp_m_axis_seg_tdata_reg;
            m_ena_reg <= temp_m_ena_reg;
            m_sop_reg <= temp_m_sop_reg;
            m_eop_reg <= temp_m_eop_reg;
            m_err_reg <= temp_m_err_reg;
            m_mty_reg <= temp_m_mty_reg;

            out_index_reg <= temp_out_index_reg;
        end

        if (store_axis_input_to_temp) begin
            temp_m_axis_seg_tdata_reg <= fifo_tdata_out;

            for (j = 0; j < NSEG; j = j + 1) begin
                temp_m_mty_reg[j] <= fifo_tvalid_out[j] ? mty_fifo_out[j] : {MTYW{1'b0}};
            end

            temp_m_ena_reg <= ena_fifo_out & fifo_tvalid_out;
            temp_m_sop_reg <= sop_fifo_out & fifo_tvalid_out;
            temp_m_eop_reg <= eop_fifo_out & fifo_tvalid_out;
            temp_m_err_reg <= err_fifo_out & fifo_tvalid_out;

            temp_out_index_reg <= out_index_before_reg;
        end

        if (rst) begin
            fifo_out_axis_tready_reg <= 1'b0;
            m_axis_tvalid_reg        <= 1'b0;
            temp_m_axis_tvalid_reg   <= 1'b0;
        end
    end

    assign m_axis_seg_tvalid = m_axis_tvalid_reg;

    // reorder the segments accordingly
    generate
        for (genvar g = 0; g < NSEG; g = g + 1) begin : g_reorder
            assign m_axis_seg_tdata[g*SEGW +: SEGW]     = m_axis_seg_tdata_reg[out_index_reg[g]];
            assign m_axis_seg_tuser_mty[g*MTYW +: MTYW] = m_mty_reg[out_index_reg[g]];
            assign m_axis_seg_tuser_ena[g]              = m_ena_reg[out_index_reg[g]];
            assign m_axis_seg_tuser_sop[g]              = m_sop_reg[out_index_reg[g]];
            assign m_axis_seg_tuser_eop[g]              = m_eop_reg[out_index_reg[g]];
            assign m_axis_seg_tuser_err[g]              = m_err_reg[out_index_reg[g]];
        end
    endgenerate

endmodule

`resetall
