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

Simple conversion with no multiple packets sent per beat.
tvalid,tlast,tkeep and tuser are a 1-to-1 mapping to AXI-Segment userfields.

In the Xilinx DCMAC 400G the output throughput is limited 
e.g. 
For a 4kB RoCE packet we reached ~386 Gbps for the simple implementation 
and ~392 for the full implementation (payload throughput)

E.G.
+----------------------+
|        INPUT         | 
+----------------------+

AXI-Stream input
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
|    SIMPLE MAP        | 
+----------------------+
Segmented AXI-Stream output
Simple map of sideband signals to the segmented tuser fields
ena --> mapped based on tkeep value
sop --> start of packet, always segment 0
eop --> mapped based on tlast and tkeep value
err --> mapped based on tlast, tkeep value and tuser
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
 */

module axis_2_seg_axis_simple #(
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

    // index of the last segment that has at least one byte kept
    function [IDXW-1:0] keep2seg;
        input [KW-1:0] keep_in;
        integer i;
        begin
            keep2seg = {IDXW{1'b0}};
            for (i = 0; i < NSEG; i = i + 1) begin
                if (|keep_in[i*SEGB +: SEGB]) keep2seg = i[IDXW-1:0];
            end
        end
    endfunction

    function [NSEG-1:0] keep2ena;
        input [KW-1:0] keep_in;
        integer i;
        begin
            for (i = 0; i < NSEG; i = i + 1) begin
                keep2ena[i] = |keep_in[i*SEGB +: SEGB];
            end
        end
    endfunction

    wire [DW-1:0]    m_axis_input_reg_tdata;
    wire [KW-1:0]    m_axis_input_reg_tkeep;
    wire             m_axis_input_reg_tvalid;
    wire             m_axis_input_reg_tready;
    wire             m_axis_input_reg_tlast;
    wire             m_axis_input_reg_tuser;

    reg  [NSEG-1:0]       sop_next;
    reg  [NSEG-1:0]       eop_next;
    reg  [NSEG-1:0]       ena_next;
    wire [MTYW*NSEG-1:0]  mty_next;

    reg last_eop;

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

    always @(*) begin
        if (m_axis_input_reg_tvalid && m_axis_input_reg_tready && m_axis_input_reg_tlast) begin
            eop_next = {{(NSEG-1){1'b0}}, 1'b1} << keep2seg(m_axis_input_reg_tkeep);
        end else begin
            eop_next = {NSEG{1'b0}};
        end

        if (m_axis_input_reg_tvalid && m_axis_input_reg_tready && last_eop) begin
            sop_next = {{(NSEG-1){1'b0}}, 1'b1};
        end else begin
            sop_next = {NSEG{1'b0}};
        end

        if (m_axis_input_reg_tvalid && m_axis_input_reg_tready && !m_axis_input_reg_tlast) begin
            ena_next = {NSEG{1'b1}};
        end else if (m_axis_input_reg_tvalid && m_axis_input_reg_tready && m_axis_input_reg_tlast) begin
            ena_next = keep2ena(m_axis_input_reg_tkeep);
        end else begin
            ena_next = {NSEG{1'b0}};
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            last_eop <= 1'b1;
        end else if (m_axis_input_reg_tvalid && m_axis_input_reg_tready) begin
            last_eop <= m_axis_input_reg_tlast;
        end
    end

    generate
        for (genvar i = 0; i < NSEG; i = i + 1) begin : g_mty
            maptkeep2mty #(
                .SEGB(SEGB)
            ) maptkeep2mty_instance (
                .s_axis_tkeep(m_axis_input_reg_tkeep[i*SEGB +: SEGB] & {SEGB{m_axis_input_reg_tvalid}}),
                .m_mty(mty_next[MTYW*i +: MTYW])
            );
        end
    endgenerate

    // skid buffer, no bubble cycles

    // datapath registers
    reg                      m_axis_input_reg_tready_reg = 1'b0;

    reg [DW-1:0]             m_axis_seg_tdata_reg = {DW{1'b0}};
    reg                      m_axis_tvalid_reg = 1'b0, m_axis_tvalid_next;
    reg [NSEG-1:0]           m_ena_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           m_sop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           m_eop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           m_err_reg = {NSEG{1'b0}};
    reg [MTYW*NSEG-1:0]      m_mty_reg = {MTYW*NSEG{1'b0}};

    reg [DW-1:0]             temp_m_axis_seg_tdata_reg = {DW{1'b0}};
    reg                      temp_m_axis_tvalid_reg = 1'b0, temp_m_axis_tvalid_next;
    reg [NSEG-1:0]           temp_m_ena_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           temp_m_sop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           temp_m_eop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           temp_m_err_reg = {NSEG{1'b0}};
    reg [MTYW*NSEG-1:0]      temp_m_mty_reg = {MTYW*NSEG{1'b0}};

    // datapath control
    reg store_axis_input_to_output;
    reg store_axis_input_to_temp;
    reg store_axis_temp_to_output;

    assign m_axis_input_reg_tready = m_axis_input_reg_tready_reg;

    // enable ready input next cycle if output is ready or the temp reg will not be filled on the next cycle (output reg empty or no input)
    wire m_axis_input_reg_tready_early = m_axis_seg_tready || (!temp_m_axis_tvalid_reg && (!m_axis_tvalid_reg || !m_axis_input_reg_tvalid));

    always @* begin
        // transfer sink ready state to source
        m_axis_tvalid_next      = m_axis_tvalid_reg;
        temp_m_axis_tvalid_next = temp_m_axis_tvalid_reg;

        store_axis_input_to_output = 1'b0;
        store_axis_input_to_temp   = 1'b0;
        store_axis_temp_to_output  = 1'b0;

        if (m_axis_input_reg_tready_reg) begin
            // input is ready
            if (m_axis_seg_tready || !m_axis_tvalid_reg) begin
                // output is ready or currently not valid, transfer data to output
                m_axis_tvalid_next = m_axis_input_reg_tvalid;
                store_axis_input_to_output = 1'b1;
            end else begin
                // output is not ready, store input in temp
                temp_m_axis_tvalid_next = m_axis_input_reg_tvalid;
                store_axis_input_to_temp = 1'b1;
            end
        end else if (m_axis_seg_tready) begin
            // input is not ready, but output is ready
            m_axis_tvalid_next      = temp_m_axis_tvalid_reg;
            temp_m_axis_tvalid_next = 1'b0;
            store_axis_temp_to_output = 1'b1;
        end
    end

    always @(posedge clk) begin
        m_axis_input_reg_tready_reg <= m_axis_input_reg_tready_early;
        m_axis_tvalid_reg           <= m_axis_tvalid_next;
        temp_m_axis_tvalid_reg      <= temp_m_axis_tvalid_next;

        // datapath
        if (store_axis_input_to_output) begin
            m_axis_seg_tdata_reg <= m_axis_input_reg_tdata;
            m_ena_reg <= ena_next;
            m_sop_reg <= sop_next;
            m_eop_reg <= eop_next;
            m_err_reg <= eop_next & {NSEG{m_axis_input_reg_tuser}};
            m_mty_reg <= mty_next;
        end else if (store_axis_temp_to_output) begin
            m_axis_seg_tdata_reg <= temp_m_axis_seg_tdata_reg;
            m_ena_reg <= temp_m_ena_reg;
            m_sop_reg <= temp_m_sop_reg;
            m_eop_reg <= temp_m_eop_reg;
            m_err_reg <= temp_m_err_reg;
            m_mty_reg <= temp_m_mty_reg;
        end

        if (store_axis_input_to_temp) begin
            temp_m_axis_seg_tdata_reg <= m_axis_input_reg_tdata;
            temp_m_ena_reg <= ena_next;
            temp_m_sop_reg <= sop_next;
            temp_m_eop_reg <= eop_next;
            temp_m_err_reg <= eop_next & {NSEG{m_axis_input_reg_tuser}};
            temp_m_mty_reg <= mty_next;
        end

        if (rst) begin
            m_axis_input_reg_tready_reg <= 1'b0;
            m_axis_tvalid_reg           <= 1'b0;
            temp_m_axis_tvalid_reg      <= 1'b0;
        end
    end

    assign m_axis_seg_tvalid    = m_axis_tvalid_reg;
    assign m_axis_seg_tdata     = m_axis_seg_tdata_reg;
    assign m_axis_seg_tuser_mty = m_mty_reg;
    assign m_axis_seg_tuser_ena = m_ena_reg;
    assign m_axis_seg_tuser_sop = m_sop_reg;
    assign m_axis_seg_tuser_eop = m_eop_reg;
    assign m_axis_seg_tuser_err = m_err_reg;

endmodule

`resetall
