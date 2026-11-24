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
 * tkeep of one segment (SEGB bits, low-aligned) -> mty (number of empty bytes).
 * Purely combinational. Non low-aligned / zero keep gives 0.
 */
module maptkeep2mty #(
    parameter SEGB = 16
)(
    input  wire [SEGB-1:0]         s_axis_tkeep,
    output wire [$clog2(SEGB)-1:0] m_mty
);

    localparam MTYW = $clog2(SEGB);

    reg [MTYW-1:0] mty_reg;
    integer i;

    always @(*) begin
        mty_reg = {MTYW{1'b0}};
        // low-aligned keep: a value of the form 0..01..1
        if ((s_axis_tkeep != {SEGB{1'b0}}) && (((s_axis_tkeep + 1'b1) & s_axis_tkeep) == {SEGB{1'b0}})) begin
            for (i = 0; i < SEGB; i = i + 1) begin
                if (s_axis_tkeep[i]) mty_reg = (SEGB - 1 - i);
            end
        end
    end

    assign m_mty = mty_reg;

endmodule

`resetall
