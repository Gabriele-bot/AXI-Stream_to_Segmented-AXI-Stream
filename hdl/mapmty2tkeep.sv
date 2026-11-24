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
 * mty (number of empty bytes) + ena -> tkeep of one segment (SEGB bits, low-aligned).
 * Purely combinational.
 */
module mapmty2tkeep #(
    parameter SEGB = 16
)(
    input  wire [$clog2(SEGB)-1:0] s_mty,
    input  wire                    s_ena,
    output wire [SEGB-1:0]         m_axis_tkeep
);

    assign m_axis_tkeep = s_ena ? ({SEGB{1'b1}} >> s_mty) : {SEGB{1'b0}};

endmodule

`resetall
