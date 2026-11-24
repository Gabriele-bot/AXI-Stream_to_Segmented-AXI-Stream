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

Expected input is an uniterrupted AXIS-Stream packet, valid stays asserted for the whole packet.
Min Frame length set to 60 Bytes (FCS insertion by MAC).

AXI-Stream to Segmented AXI-Stream control input stage
The module takes in the AXI-Stream and produces unaligned AXI segements
The segments are filled in in a round robin way.
E.G.
AXI-Stream input
 col  0     1     2     3     4     5     
    +-----+-----+-----+-----+-----+-----+ 
    |  A0 |  B0 |  C0 |  C4 |     |  D0 | 
    +-----+-----+-----+-----+-----+-----+ 
    |  A1 |  B1 |  C1 |  C5 |     |  D1 | 
    +-----+-----+-----+-----+-----+-----+ 
    |  A2 |     |  C2 |  C6 |     |  D2 | 
    +-----+-----+-----+-----+-----+-----+ 
    |     |     |  C3 |  C7 |     |     | 
    +-----+-----+-----+-----+-----+-----+ 

AXI-Stream Segments (pre-alignment), just move the segment around and feed them to the right segment fifo
col  0     1     2     3     4     5     
    +-----+-----+-----+-----+-----+-----+
    |  A0 |  B1 |  C3 |  C7 |     |     |
    +-----+-----+-----+-----+-----+-----+
    |  A1 |     |  C0 |  C4 |     |  D0 |
    +-----+-----+-----+-----+-----+-----+
    |  A2 |     |  C1 |  C5 |     |  D1 |
    +-----+-----+-----+-----+-----+-----+
    |     |  B0 |  C2 |  C6 |     |  D2 |
    +-----+-----+-----+-----+-----+-----+
 */
module axis_2_seg_axis_input_ctrl #(
    parameter NSEG = 8,
    parameter SEGB = 16 // bytes per segment
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

    reg  [IDXW-1:0]       offset_reg = {IDXW{1'b0}}, offset_next;
    reg  [NSEG-1:0]       sop_next;
    reg  [NSEG-1:0]       eop_next;
    reg  [NSEG-1:0]       ena_next;
    wire [MTYW*NSEG-1:0]  mty_next;

    reg [NSEG-1:0][IDXW-1:0] in_index_reg = {NSEG*IDXW{1'b0}};
    reg [NSEG-1:0][IDXW-1:0] in_index_next;

    reg last_eop; // previous accepted beat was a tlast

    integer k;

    always @(*) begin
        offset_next = offset_reg;

        if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
            eop_next = {{(NSEG-1){1'b0}}, 1'b1} << keep2seg(s_axis_tkeep);
        end else begin
            eop_next = {NSEG{1'b0}};
        end

        if (s_axis_tvalid && s_axis_tready && last_eop) begin
            sop_next = {{(NSEG-1){1'b0}}, 1'b1};
        end else begin
            sop_next = {NSEG{1'b0}};
        end

        if (s_axis_tvalid && s_axis_tready && !s_axis_tlast) begin
            ena_next = {NSEG{1'b1}};
        end else if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
            ena_next = keep2ena(s_axis_tkeep);
        end else begin
            ena_next = {NSEG{1'b0}};
        end

        // next frame starts right after the last used segment (IDXW-bit wrap-around)
        if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
            offset_next = offset_reg + keep2seg(s_axis_tkeep) + 1'b1;
        end

        for (k = 0; k < NSEG; k = k + 1) begin
            in_index_next[k] = k - offset_reg;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            offset_reg <= {IDXW{1'b0}};
            last_eop   <= 1'b1;
        end else begin
            offset_reg <= offset_next;
            if (s_axis_tvalid && s_axis_tready) begin
                last_eop <= s_axis_tlast;
            end
        end
    end

    generate
        for (genvar i = 0; i < NSEG; i = i + 1) begin : g_mty
            maptkeep2mty #(
                .SEGB(SEGB)
            ) maptkeep2mty_instance (
                .s_axis_tkeep(s_axis_tkeep[i*SEGB +: SEGB] & {SEGB{s_axis_tvalid}}),
                .m_mty(mty_next[MTYW*i +: MTYW])
            );
        end
    endgenerate

    // skid buffer, no bubble cycles

    // datapath registers
    reg                      s_axis_tready_reg = 1'b0;

    reg [NSEG-1:0][SEGW-1:0] m_axis_seg_tdata_reg = {DW{1'b0}};
    reg                      m_axis_tvalid_reg = 1'b0, m_axis_tvalid_next;
    reg [NSEG-1:0]           m_ena_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           m_sop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           m_eop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           m_err_reg = {NSEG{1'b0}};
    reg [NSEG-1:0][MTYW-1:0] m_mty_reg = {NSEG*MTYW{1'b0}};

    reg [NSEG-1:0][SEGW-1:0] temp_m_axis_seg_tdata_reg = {DW{1'b0}};
    reg                      temp_m_axis_tvalid_reg = 1'b0, temp_m_axis_tvalid_next;
    reg [NSEG-1:0]           temp_m_ena_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           temp_m_sop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           temp_m_eop_reg = {NSEG{1'b0}};
    reg [NSEG-1:0]           temp_m_err_reg = {NSEG{1'b0}};
    reg [NSEG-1:0][MTYW-1:0] temp_m_mty_reg = {NSEG*MTYW{1'b0}};
    reg [NSEG-1:0][IDXW-1:0] temp_in_index_reg = {NSEG*IDXW{1'b0}};

    // datapath control
    reg store_axis_input_to_output;
    reg store_axis_input_to_temp;
    reg store_axis_temp_to_output;

    assign s_axis_tready = s_axis_tready_reg;

    // enable ready input next cycle if output is ready or the temp reg will not be filled on the next cycle (output reg empty or no input)
    wire s_axis_tready_early = m_axis_seg_tready || (!temp_m_axis_tvalid_reg && (!m_axis_tvalid_reg || !s_axis_tvalid));

    always @* begin
        // transfer sink ready state to source
        m_axis_tvalid_next      = m_axis_tvalid_reg;
        temp_m_axis_tvalid_next = temp_m_axis_tvalid_reg;

        store_axis_input_to_output = 1'b0;
        store_axis_input_to_temp   = 1'b0;
        store_axis_temp_to_output  = 1'b0;

        if (s_axis_tready_reg) begin
            // input is ready
            if (m_axis_seg_tready || !m_axis_tvalid_reg) begin
                // output is ready or currently not valid, transfer data to output
                m_axis_tvalid_next = s_axis_tvalid;
                store_axis_input_to_output = 1'b1;
            end else begin
                // output is not ready, store input in temp
                temp_m_axis_tvalid_next = s_axis_tvalid;
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
        s_axis_tready_reg      <= s_axis_tready_early;
        m_axis_tvalid_reg      <= m_axis_tvalid_next;
        temp_m_axis_tvalid_reg <= temp_m_axis_tvalid_next;

        // datapath
        if (store_axis_input_to_output) begin
            m_axis_seg_tdata_reg <= s_axis_tdata;
            m_ena_reg <= ena_next;
            m_sop_reg <= sop_next;
            m_eop_reg <= eop_next;
            m_err_reg <= eop_next & {NSEG{s_axis_tuser}};
            m_mty_reg <= mty_next;
            in_index_reg <= in_index_next;
        end else if (store_axis_temp_to_output) begin
            m_axis_seg_tdata_reg <= temp_m_axis_seg_tdata_reg;
            m_ena_reg <= temp_m_ena_reg;
            m_sop_reg <= temp_m_sop_reg;
            m_eop_reg <= temp_m_eop_reg;
            m_err_reg <= temp_m_err_reg;
            m_mty_reg <= temp_m_mty_reg;
            in_index_reg <= temp_in_index_reg;
        end

        if (store_axis_input_to_temp) begin
            temp_m_axis_seg_tdata_reg <= s_axis_tdata;
            temp_m_ena_reg <= ena_next;
            temp_m_sop_reg <= sop_next;
            temp_m_eop_reg <= eop_next;
            temp_m_err_reg <= eop_next & {NSEG{s_axis_tuser}};
            temp_m_mty_reg <= mty_next;
            temp_in_index_reg <= in_index_next;
        end

        if (rst) begin
            s_axis_tready_reg      <= 1'b0;
            m_axis_tvalid_reg      <= 1'b0;
            temp_m_axis_tvalid_reg <= 1'b0;
        end
    end

    assign m_axis_seg_tvalid = m_axis_tvalid_reg;

    // reorder the segments accordingly
    generate
        for (genvar g = 0; g < NSEG; g = g + 1) begin : g_reorder
            assign m_axis_seg_tdata[g*SEGW +: SEGW]    = m_axis_seg_tdata_reg[in_index_reg[g]];
            assign m_axis_seg_tuser_mty[g*MTYW +: MTYW] = m_mty_reg[in_index_reg[g]];
            assign m_axis_seg_tuser_ena[g]             = m_ena_reg[in_index_reg[g]];
            assign m_axis_seg_tuser_sop[g]             = m_sop_reg[in_index_reg[g]];
            assign m_axis_seg_tuser_eop[g]             = m_eop_reg[in_index_reg[g]];
            assign m_axis_seg_tuser_err[g]             = m_err_reg[in_index_reg[g]];
        end
    endgenerate

endmodule

`resetall
