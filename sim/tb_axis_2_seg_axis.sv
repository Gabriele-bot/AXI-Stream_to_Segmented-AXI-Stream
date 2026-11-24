// SPDX-License-Identifier: CERN-OHL-P-2.0
// Copyright 2026 Gabriele Bortolato
// Source Location: https://github.com/Gabriele-bot/AXI-Stream_to_Segmented-AXI-Stream
// This source describes Open Hardware and is licensed under the CERN-OHL-P v2.
// You may redistribute and modify this documentation and make products using it
// under the terms of the CERN-OHL-P v2 (https://ohwr.org/cern_ohl_p_v2.txt).
// This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
// INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE.
// Please see the CERN-OHL-P v2 for applicable conditions.
`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Testbench for axis_2_seg_axis_wrapper (AXI-Stream to Segmented AXI-Stream, e.g. TX direction for a DCMAC).
 *
 * Drives a plain AXI-Stream (packet mode: tvalid stays high from the first beat of a packet until
 * its tlast has been accepted), collects the Segmented AXI-Stream output (NSEG segments of SEGB bytes)
 * and verifies that every output packet matches the corresponding input packet in data, length,
 * error flag (tuser -> err on the eop segment) and arrival order. The segmented stream is rebuilt
 * with a behavioural reassembler that goes through the segments of every beat in index order, so
 * several packets in one beat (eop of one packet and sop of the next) can be handled.
 *
 * Generics (can be overridden in the cli with the option -g<PARAMETER>):
 *   NSEG             : number of segments  (e.g. Xilinx 128b: 2/4/8 = 100G/200G/400G ; Altera 64b: 4/8/16)
 *   SEGB             : bytes per segment   (e.g. Xilinx 128b: 16 ; Altera 64b: 8)
 *   NP               : number of packets sent in the tests that send a configurable amount of packets
 *   SIMPLE           : 0 = full implementation (sop can happen right after an eop), 1 = simple TX module (every packet starts at segment 0)
 *   INPUT_REGS       : input pipeline register stages of the TX module, just for timing closure
 *   IS_ASYNC         : 1 = async output fifo, the two clocks differ; 0 = no fifo, a single clock
 *   ASYNC_FIFO_DEPTH : depth of the output fifo, tested with value of 8 on a 400G DCMAC design
 *
 * Byte layout per segment (SEGB bytes):
 *   tdata[8*b +: 8]  = byte b of segment  (b=0 is first byte, b=SEGB-1 is last)
 *   MTY = number of empty bytes at the HIGH end of the last segment
 *         (MTY=0 -> all SEGB bytes valid, MTY=SEGB-1 -> only byte 0 valid)
 *
 * Packet encoding:
 *   Bytes [3:0] = 32-bit sequence number (big-endian)
 *   Remaining   = byte index within packet (mod 256)
 *
 * Generator:
 *   send_packet(seq, size, err)   one packet, tvalid kept high until its tlast is accepted
 *   idle_cycles(n)                n idle cycles (tvalid low) on s_clk
 *   Calling send_packet() calls one after the other gives back to back packets (no idle cycle).
 */
module tb_axis_2_seg_axis #(
    parameter NSEG             = 8,
    parameter SEGB             = 16,
    parameter NP               = 50,
    parameter SIMPLE           = 0,
    parameter INPUT_REGS       = 1,
    parameter IS_ASYNC         = 1,
    parameter ASYNC_FIFO_DEPTH = 8
);

    // ---------------------------------------------------------------------------
    // Parameters
    // ---------------------------------------------------------------------------
    localparam real S_CLK_PERIOD = 1000/411.0; // ns – ~411 MHz (AXI-Stream side)
    localparam real M_CLK_PERIOD = 1000/390.625; // ns – ~390.625 MHz (segmented / MAC side)
    localparam int  DRAIN_TIMEOUT_CYCLES = 100_000;

    localparam int  MTYW = $clog2(SEGB);       // bits of MTY per segment
    localparam int  DW   = NSEG*SEGB*8;        // data width of the AXI-Stream / segmented bus
    localparam int  KW   = NSEG*SEGB;          // tkeep width of the AXI-Stream

    // ---------------------------------------------------------------------------
    // DUT signals
    // ---------------------------------------------------------------------------
    logic           s_clk = 0, s_rst = 1;
    logic           m_clk_gen = 0;
    wire            m_clk = IS_ASYNC ? m_clk_gen : s_clk;
    logic           m_rst = 1;

    logic [DW-1:0]  s_axis_tdata  = '0;
    logic [KW-1:0]  s_axis_tkeep  = '0;
    logic           s_axis_tvalid = '0;
    wire            s_axis_tready;
    logic           s_axis_tlast  = '0;
    logic           s_axis_tuser  = '0;

    wire  [DW-1:0]        m_axis_seg_tdata;
    wire                  m_axis_seg_tvalid;
    logic                 m_axis_seg_tready = 1; // always ready by default; tests can override
    wire  [NSEG-1:0]      m_axis_seg_tuser_ena;
    wire  [NSEG-1:0]      m_axis_seg_tuser_sop;
    wire  [NSEG-1:0]      m_axis_seg_tuser_eop;
    wire  [NSEG-1:0]      m_axis_seg_tuser_err;
    wire  [NSEG*MTYW-1:0] m_axis_seg_tuser_mty;

    // ---------------------------------------------------------------------------
    // DUT
    // ---------------------------------------------------------------------------
    axis_2_seg_axis_wrapper #(
        .NSEG             (NSEG),
        .SEGB             (SEGB),
        .SIMPLE           (SIMPLE),
        .INPUT_REGS       (INPUT_REGS),
        .IS_ASYNC         (IS_ASYNC),
        .ASYNC_FIFO_DEPTH (ASYNC_FIFO_DEPTH)
    ) dut (
        .s_clk                (s_clk),
        .s_rst                (s_rst),
        .s_axis_tdata         (s_axis_tdata),
        .s_axis_tkeep         (s_axis_tkeep),
        .s_axis_tvalid        (s_axis_tvalid),
        .s_axis_tready        (s_axis_tready),
        .s_axis_tlast         (s_axis_tlast),
        .s_axis_tuser         (s_axis_tuser),
        .m_clk                (m_clk),
        .m_rst                (m_rst),
        .m_axis_seg_tdata     (m_axis_seg_tdata),
        .m_axis_seg_tvalid    (m_axis_seg_tvalid),
        .m_axis_seg_tready    (m_axis_seg_tready),
        .m_axis_seg_tuser_ena (m_axis_seg_tuser_ena),
        .m_axis_seg_tuser_sop (m_axis_seg_tuser_sop),
        .m_axis_seg_tuser_eop (m_axis_seg_tuser_eop),
        .m_axis_seg_tuser_err (m_axis_seg_tuser_err),
        .m_axis_seg_tuser_mty (m_axis_seg_tuser_mty)
    );

    // ---------------------------------------------------------------------------
    // Clocks
    // ---------------------------------------------------------------------------
    always #(S_CLK_PERIOD / 2.0) s_clk = ~s_clk;
    always #(M_CLK_PERIOD / 2.0) m_clk_gen = ~m_clk_gen;

    // ---------------------------------------------------------------------------
    // Scoreboard
    // ---------------------------------------------------------------------------
    typedef byte unsigned bu_t;
    typedef bu_t bu_q_t[$];

    bu_q_t sent_pkts[$];   // every packet, in transmit order
    bit    sent_err [$];   // expected err flag of every packet
    int    pkts_sent   = 0;
    int    pkts_recvd  = 0;
    int    errors      = 0;

    // eop -> sop adjacency coverage of the segmented output
    int    cnt_same_beat = 0;  // sop in the segment right after the eop, same beat
    int    cnt_cross     = 0;  // eop in the last segment, sop in segment 0 of the next beat
    int    cnt_realign   = 0;  // sop elsewhere (e.g. restarted at segment 0 after idle segments)

    // ---------------------------------------------------------------------------
    // Output checker (runs on m_clk): behavioural reassembler of the segmented stream
    // ---------------------------------------------------------------------------
    bu_q_t cur_pkt;            // bytes of the packet being reassembled
    bit    in_frame = 0;
    int    last_eop_seg = -1;  // segment of the last eop seen, -1 if none pending

    task automatic check_segment(int s);
        automatic int nb;
        automatic int mty;
        if (m_axis_seg_tuser_ena[s]) begin
            if (m_axis_seg_tuser_sop[s]) begin
                if (in_frame) begin
                    $error("[%0t ns] sop inside a frame (pkt#%0d, seg %0d)", $time, pkts_recvd, s);
                    errors++;
                end
                in_frame = 1;
                cur_pkt  = {};
                // adjacency coverage
                if (last_eop_seg >= 0) begin
                    if (s > 0 && last_eop_seg == s-1)           cnt_same_beat++;
                    else if (s == 0 && last_eop_seg == NSEG-1)  cnt_cross++;
                    else                                        cnt_realign++;
                end
                last_eop_seg = -1;
            end else if (!in_frame) begin
                $error("[%0t ns] data outside a frame (pkt#%0d, seg %0d)", $time, pkts_recvd, s);
                errors++;
            end

            mty = int'(m_axis_seg_tuser_mty[s*MTYW +: MTYW]);
            nb  = m_axis_seg_tuser_eop[s] ? (SEGB - mty) : SEGB;
            for (int b = 0; b < nb; b++)
                cur_pkt.push_back(m_axis_seg_tdata[s*SEGB*8 + 8*b +: 8]);

            if (m_axis_seg_tuser_err[s] && !m_axis_seg_tuser_eop[s]) begin
                $error("[%0t ns] err flag outside the eop segment (pkt#%0d, seg %0d)", $time, pkts_recvd, s);
                errors++;
            end

            if (m_axis_seg_tuser_eop[s]) begin
                if (pkts_recvd >= sent_pkts.size()) begin
                    $error("[%0t ns] received unexpected extra packet (size=%0d)", $time, cur_pkt.size());
                    errors++;
                end else begin
                    automatic bu_q_t exp = sent_pkts[pkts_recvd];
                    automatic int mismatches = 0;
                    if (cur_pkt.size() != exp.size()) begin
                        $error("[%0t ns] Pkt#%0d size mismatch: got %0d B, expected %0d B",
                            $time, pkts_recvd, cur_pkt.size(), exp.size());
                        errors++;
                    end else begin
                        for (int i = 0; i < int'(cur_pkt.size()); i++) begin
                            if (cur_pkt[i] !== exp[i]) begin
                                if (mismatches == 0)
                                    $error("[%0t ns] Pkt#%0d data error at byte[%0d]: got 0x%02x exp 0x%02x",
                                        $time, pkts_recvd, i, cur_pkt[i], exp[i]);
                                mismatches++;
                            end
                        end
                        if (mismatches > 1)
                            $error("[%0t ns] Pkt#%0d had %0d total byte mismatches", $time, pkts_recvd, mismatches);
                        if (mismatches > 0) errors++;
                    end
                    if (m_axis_seg_tuser_err[s] !== sent_err[pkts_recvd]) begin
                        $error("[%0t ns] Pkt#%0d err flag mismatch: got %0b expected %0b",
                            $time, pkts_recvd, m_axis_seg_tuser_err[s], sent_err[pkts_recvd]);
                        errors++;
                    end else if (mismatches == 0 && cur_pkt.size() == exp.size()) begin
                        $display("[%0t ns] Pkt#%0d OK (%0d bytes%s)", $time, pkts_recvd, cur_pkt.size(),
                            sent_err[pkts_recvd] ? ", err" : "");
                    end
                    pkts_recvd++;
                end
                in_frame     = 0;
                cur_pkt      = {};
                last_eop_seg = s;
            end
        end else begin
            if (m_axis_seg_tuser_sop[s] || m_axis_seg_tuser_eop[s] || m_axis_seg_tuser_err[s]) begin
                $error("[%0t ns] sop/eop/err set on a segment without ena (seg %0d)", $time, s);
                errors++;
            end
        end
    endtask

    always @(posedge m_clk) begin
        if (!m_rst && m_axis_seg_tvalid && m_axis_seg_tready) begin
            for (int s = 0; s < NSEG; s++) check_segment(s);
        end
    end

    // ---------------------------------------------------------------------------
    // Helper: build a packet byte array
    //   Bytes 0-3 : seq (big-endian)
    //   Bytes 4.. : byte-index mod 256
    // ---------------------------------------------------------------------------
    function automatic bu_q_t make_pkt(int seq, int size);
        bu_q_t p;
        p = {};
        p.push_back(bu_t'((seq >> 24) & 8'hFF));
        p.push_back(bu_t'((seq >> 16) & 8'hFF));
        p.push_back(bu_t'((seq >>  8) & 8'hFF));
        p.push_back(bu_t'( seq        & 8'hFF));
        for (int i = 4; i < size; i++)
            p.push_back(bu_t'(i & 8'hFF));
        return p;
    endfunction

    // ---------------------------------------------------------------------------
    // idle_cycles: drive n idle (tvalid=0) cycles on s_clk
    // ---------------------------------------------------------------------------
    task automatic idle_cycles(int n = 1);
        repeat (n) begin
            @(posedge s_clk); #1;
            s_axis_tvalid <= 0;
            s_axis_tlast  <= 0;
            s_axis_tuser  <= 0;
            s_axis_tkeep  <= '0;
        end
    endtask

    // ---------------------------------------------------------------------------
    // send_packet: drive one packet in packet mode.
    //   tvalid stays high from the first beat until the last beat has been accepted,
    //   there is no idle cycle inside the packet. Two calls one after the other give
    //   back to back packets (tvalid never drops between them).
    //   Beats are always full (KW bytes) except for the last one.
    //   err=1 raises tuser together with tlast.
    // ---------------------------------------------------------------------------
    task automatic send_packet(int seq, int size, bit err = 0);
        automatic bu_q_t pkt = make_pkt(seq, size);
        automatic int    off = 0;
        automatic int    n;
        sent_pkts.push_back(pkt);
        sent_err.push_back(err);
        pkts_sent++;

        while (off < size) begin
            n = ((size - off) > KW) ? KW : (size - off);
            // wait for the (#1 after the) clock edge; keep the previous beat's timing if back to back
            s_axis_tdata  <= '0;
            s_axis_tkeep  <= '0;
            for (int b = 0; b < n; b++) begin
                s_axis_tdata[8*b +: 8] <= pkt[off + b];
                s_axis_tkeep[b]        <= 1'b1;
            end
            s_axis_tlast  <= ((off + n) >= size);
            s_axis_tuser  <= ((off + n) >= size) ? err : 1'b0;
            s_axis_tvalid <= 1;
            // the beat is transferred on the first edge where tready is high
            do @(posedge s_clk); while (!s_axis_tready);
            #1;
            off += n;
            // deassert; an immediately following beat re-asserts it in the same time step,
            // so a back to back stream never shows a tvalid gap
            s_axis_tvalid <= 0;
            s_axis_tlast  <= 0;
            s_axis_tuser  <= 0;
        end
    endtask

    // ---------------------------------------------------------------------------
    // wait_drain: spin on m_clk until all sent packets are received or timeout.
    // ---------------------------------------------------------------------------
    task automatic wait_drain(int cycles = DRAIN_TIMEOUT_CYCLES);
        int cnt = 0;
        while (pkts_recvd < pkts_sent && cnt < cycles) begin
            @(posedge m_clk);
            cnt++;
        end
        if (cnt >= cycles) begin
            $error("Drain timeout: sent=%0d received=%0d", pkts_sent, pkts_recvd);
            errors++;
        end
    endtask

    // ---------------------------------------------------------------------------
    // Main test sequence
    // ---------------------------------------------------------------------------
    initial begin
        // --- Reset ---
        s_rst = 1; m_rst = 1;
        repeat (20) @(posedge s_clk);
        s_rst = 0;
        repeat (20) @(posedge m_clk);
        m_rst = 0;
        repeat (5)  @(posedge s_clk);
        #1;

        $display("Configuration: NSEG=%0d SEGB=%0d (beat = %0d B) NP=%0d SIMPLE=%0d INPUT_REGS=%0d IS_ASYNC=%0d ASYNC_FIFO_DEPTH=%0d",
            NSEG, SEGB, NSEG*SEGB, NP, SIMPLE, INPUT_REGS, IS_ASYNC, ASYNC_FIFO_DEPTH);

        // ===========================================================
        // Test 1: Sequential minimum-size packets (64 B)
        //         Idle cycles between each
        // ===========================================================
        $display("--- Test 1: Sequential 64-byte packets (idle between) ---");
        for (int i = 0; i < NP; i++) begin
            send_packet(i, 64);
            idle_cycles(2);
        end
        wait_drain();
        $display("Test 1 done. errors so far: %0d", errors);
        idle_cycles(20);

        // ===========================================================
        // Test 2: Multi-beat packets (256 B)
        // ===========================================================
        $display("--- Test 2: Multi-beat 256-byte packets (idle between) ---");
        for (int i = 0; i < NP; i++) begin
            send_packet(100 + i, 256);
            idle_cycles(2);
        end
        wait_drain();
        $display("Test 2 done. errors so far: %0d", errors);
        idle_cycles(20);

        // ===========================================================
        // Test 3: Non-aligned sizes (last segment not fully used)
        // ===========================================================
        $display("--- Test 3: Non-segment-aligned sizes ---");
        begin
            int sizes[22] = '{60, 61, 64, 65, 79, 128, 130, 255, 300, 430, 512,
            513, 600, 700, 800, 900, 1000, 1023, 1024,
            1025, 1500, 2000};
            for (int i = 0; i < 22; i++) begin
                send_packet(200 + i, sizes[i]);
                idle_cycles(2);
            end
        end
        wait_drain();
        $display("Test 3 done. errors so far: %0d", errors);
        idle_cycles(20);

        // ===========================================================
        // Test 4: Packets with the error flag (tuser) set on some of them
        // ===========================================================
        $display("--- Test 4: Packets with tuser (err) ---");
        for (int i = 0; i < NP; i++) begin
            send_packet(300 + i, 64 + ($urandom % 437), ($urandom % 3) == 0);
            idle_cycles($urandom % 3);
        end
        idle_cycles(2);
        wait_drain();
        $display("Test 4 done. errors so far: %0d", errors);
        idle_cycles(20);

        // ===========================================================
        // Test 5: Back to back packets (tvalid never drops between packets).
        //   4 fixed sizes followed by NP random ones (64-500 B).
        // ===========================================================
        $display("--- Test 5: Back-to-back packets ---");
        send_packet(500, 144);
        send_packet(501,  64);
        send_packet(502,  64);
        send_packet(503, 144);
        for (int i = 0; i < NP; i++)
            send_packet(504 + i, 64 + ($urandom % 437));
        idle_cycles(2);
        wait_drain();
        $display("Test 5 done. errors so far: %0d", errors);
        idle_cycles(20);

        // ===========================================================
        // Test 6: Back to back minimum-size packets (60 B = 64 B without FCS):
        //         maximum number of packets per beat
        // ===========================================================
        $display("--- Test 6: Back-to-back 60-byte packets ---");
        for (int i = 0; i < NP; i++)
            send_packet(700 + i, 60);
        idle_cycles(2);
        wait_drain();
        $display("Test 6 done. errors so far: %0d", errors);
        idle_cycles(20);

        // ===========================================================
        // Test 7: Back-pressure stress - random sizes, random gaps, random tready
        // ===========================================================
        $display("--- Test 7: Random packets with back-pressure ---");
        fork
            begin : tready_driver
                forever begin
                    @(posedge m_clk); #1;
                    m_axis_seg_tready <= ($urandom % 4 != 0); // ~75% ready
                end
            end
        join_none

        for (int i = 0; i < NP; i++) begin
            send_packet(1000 + i, 60 + ($urandom % 453), ($urandom % 8) == 0); // 60-512 bytes
            idle_cycles($urandom % 4); // 0-3 idle cycles between packets
        end
        idle_cycles(2);
        wait_drain(500_000);
        disable tready_driver;
        m_axis_seg_tready <= 1;
        $display("Test 7 done. errors so far: %0d", errors);
        idle_cycles(50);

        // ===========================================================
        // Test 8: Back-pressure stress - back to back
        // ===========================================================
        $display("--- Test 8: Back-to-back packets with back-pressure ---");
        fork
            begin : tready_driver_2
                forever begin
                    @(posedge m_clk); #1;
                    m_axis_seg_tready <= ($urandom % 4 != 0); // ~75% ready
                end
            end
        join_none

        for (int i = 0; i < NP; i++)
            send_packet(2000 + i, 60 + ($urandom % 6441), ($urandom % 8) == 0); // 60-500 bytes
        idle_cycles(2);
        wait_drain(500_000);
        disable tready_driver_2;
        m_axis_seg_tready <= 1;
        $display("Test 8 done. errors so far: %0d", errors);
        idle_cycles(50);

        // ===========================================================
        // Final report
        // ===========================================================
        $display("=========================================");
        $display("  NSEG=%0d SEGB=%0d NP=%0d SIMPLE=%0d IS_ASYNC=%0d", NSEG, SEGB, NP, SIMPLE, IS_ASYNC);
        $display("  Packets sent:     %0d", pkts_sent);
        $display("  Packets received: %0d", pkts_recvd);
        $display("  eop->sop: same beat=%0d, across beats=%0d, realigned=%0d", cnt_same_beat, cnt_cross, cnt_realign);
        $display("  Total errors:     %0d", errors);
        if (errors == 0 && pkts_sent == pkts_recvd)
            $display("  RESULT: PASS");
        else
            $display("  RESULT: FAIL");
        $display("=========================================");
        $finish;
    end

    // Safety watchdog
    initial begin
        #50_000_000;
        $error("Watchdog timeout at 50 ms simulation time");
        $finish;
    end

endmodule
