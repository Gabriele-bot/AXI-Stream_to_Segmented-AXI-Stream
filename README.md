# AXI-Stream <-> Segmented AXI-Stream

SystemVerilog modules that convert between a plain **AXI-Stream** and a **Segmented AXI-Stream**, the
interface used by hard 100G/200G/400G Ethernet MACs (e.g. Xilinx DCMAC, Altera F-Tile Hard Ethernet).

A segmented stream splits each beat into `NSEG` segments of `SEGB` bytes. Each segment has its own
sideband flags, so one beat can carry the end of one frame and the start of the next.

| Per-segment sideband | Meaning                                                              |
|----------------------|----------------------------------------------------------------------|
| `ena`                | segment carries data                                                 |
| `sop`                | segment carries the first bytes of a frame                           |
| `eop`                | segment carries the last bytes of a frame                            |
| `err`                | frame ending in this segment is bad                                  |
| `mty`                | number of unused bytes in the segment (0..SEGB-1), valid on `eop`    |

Example configurations:

| Device                 | SEGB | 100G     | 200G     | 400G      |
|------------------------|------|----------|----------|-----------|
| Xilinx DCMAC           | 16   | NSEG = 2 | NSEG = 4 | NSEG = 8  |
| Altera F-Tile Ethernet | 8    | NSEG = 4 | NSEG = 8 | NSEG = 16 |

## Repository layout

```
hdl/            SystemVerilog sources (the converters)
sim/            Self-checking testbenches
verilog-axis/   Submodule: Alex Forencich's verilog-axis (FIFOs, arbiter/mux used by the converters)
Makefile        Compile + test on Vivado xsim and Questa
```

Clone with `git clone --recursive`, or run `git submodule update --init` after cloning.

## Top modules

### `axis_2_seg_axis_wrapper.sv`: AXI-Stream -> Segmented AXI-Stream

Top-level for the TX direction. It selects the conversion core and optionally adds an output async FIFO
for the clock-domain crossing and for generating back pressure.

```
s_axis (s_clk) --> [ axis_2_seg_axis  |  axis_2_seg_axis_simple ] --> [ async FIFO (optional) ] --> m_axis_seg (m_clk)
```

The input must be a plain AXI-Stream in **packet mode**: `tvalid` stays asserted for the whole frame.
The minimum frame length is 60 bytes.

**Parameters**

| Parameter          | Default | Description                                                                              |
|--------------------|---------|------------------------------------------------------------------------------------------|
| `NSEG`             | 8       | Number of segments                                                                       |
| `SEGB`             | 16      | Bytes per segment                                                                        |
| `SIMPLE`           | 0       | `0`: full core (`axis_2_seg_axis`). `1`: simple core (`axis_2_seg_axis_simple`)          |
| `INPUT_REGS`       | 1       | Input pipeline register stages in the core                         |
| `IS_ASYNC`         | 1       | `1`: output async FIFO and CDC from `s_clk` to `m_clk`. `0`: pass-through, single clock  |
| `ASYNC_FIFO_DEPTH` | 8       | Async FIFO depth in entries                                                              |

**Ports**

- Slave AXI-Stream (`s_clk`): `s_axis_tdata/tkeep/tvalid/tready/tlast/tuser`, data width `NSEG*SEGB*8`.
- Master segmented stream (`m_clk`): `m_axis_seg_tdata/tvalid/tready`, plus the sideband flags
  `m_axis_seg_tuser_{ena,sop,eop,err,mty}`.

**Full vs simple core**

- **Full (`SIMPLE=0`)**: a new frame may start in the same beat in which the previous one ends. The
  pipeline is pre-align (rotates the data), then align and reorder. This gives the highest throughput.
- **Simple (`SIMPLE=1`)**: no more than one frame per beat. Every frame starts at segment 0 and the AXI-Stream
  `tvalid/tlast/tkeep/tuser` signals map 1-to-1 onto the segment flags. It is smaller and simpler but slightly
  slower. On a Xilinx DCMAC 400G with 4 kB RoCE packets, the simple core reached about 386 Gbps and the full
  core about 392 Gbps of payload throughput.

### `seg_axis_2_axis.sv`: Segmented AXI-Stream -> AXI-Stream

Top-level for the RX direction. The segmented input can carry several frames per beat, with `sop` in any
segment. The module turns it into one aligned AXI-Stream, where every frame starts at byte 0 and uses
`tkeep/tlast/tuser`.

```
                      +-----------------------------+
                  +-->| unaligned_seg_axis_2_axis 0 |--+
                  |   +-----------------------------+  |
+------------+    |   +-----------------------------+  |    +--------------+
| seg_axis_  |    +-->| unaligned_seg_axis_2_axis 1 |--+--->| axis_arb_mux |--> m_axis
| split      |----+   +-----------------------------+  |    | (round robin)|    (m_clk)
+------------+    |             ...                    |    +--------------+
 ^                |   +-----------------------------+  |
 | s_axis_seg     +-->| unaligned_seg_axis_2_axis   |--+
 | (s_clk)            |             N-1             |
                      +-----------------------------+
```

1. **`seg_axis_split`** distributes whole frames to `N` lanes in round-robin order (frame `k` goes to lane `k mod N`).
   After the split, each lane sees at most one frame per beat, with `sop` not necessarily in segment 0.
2. **`unaligned_seg_axis_2_axis`** (one per lane) tracks which segment holds the `sop`, shifts the data so the
   frame starts at byte 0, and writes it to a FIFO. The FIFO can be async, so it also handles the clock-domain crossing.
3. **`axis_arb_mux`** (from verilog-axis) merges the `N` lane streams into a single AXI-Stream, round robin per frame.

Back pressure toward the source (`s_axis_seg_tready`) is deasserted as soon as **any** lane FIFO is almost full.

**Parameters**

| Parameter    | Default                       | Description                                                          |
|--------------|-------------------------------|----------------------------------------------------------------------|
| `NSEG`       | 8                             | Number of segments                                                   |
| `SEGB`       | 16                            | Bytes per segment                                                    |
| `N`          | 3 if `NSEG*SEGB >= 128`, else 2 | Number of parallel lanes, sized for a 60-byte minimum frame (FCS stripped) |
| `FIFO_DEPTH` | 4096                          | Per-lane output FIFO depth                                           |
| `ASYNC_FIFO` | 1                             | `1`: async lane FIFOs, so `s_clk` and `m_clk` can differ             |

**Ports**ß

- Slave segmented stream (`s_clk`): `s_axis_seg_tdata/tvalid/tready` and `s_ena/s_sop/s_eop/s_err/s_mty`.
- Master AXI-Stream (`m_clk`): `m_axis_tdata/tkeep/tvalid/tready/tlast/tuser`.

## Simulation

`sim/` holds two self-checking testbenches: `tb_axis_2_seg_axis` and `tb_seg_axis_2_axis`. Each prints
`RESULT: PASS` or `RESULT: FAIL`. The Makefile compiles everything and runs a test on **Vivado xsim**
and **Questa**.

Configurations (the first two groups are the 16-byte segments of DCMAC and the 8-byte segments of F-Tile):

- `SEGB=8`:  `NSEG` = 4, 8, 16
- `SEGB=16`: `NSEG` = 2, 4, 8
- `tb_axis_2_seg_axis` is additionally run with `SIMPLE=1`.

```
make                  # every run on both simulators
make vivado           # Vivado only (make questa for Questa only)
make -j4              # in parallel
make questa_tb_axis_2_seg_axis_N8_S16_SIMPLE1   # single run: <sim>_<tb>_N<nseg>_S<segb>[_SIMPLE1]
make clean
```

If the tools are not in `PATH`: `make VIVADO_BIN=<dir> QUESTA_BIN=<dir>`. Logs go to `build/logs/`.

## License

The sources in `hdl/` and `sim/` are released under the
[CERN Open Hardware Licence Version 2 - Permissive](https://ohwr.org/cern_ohl_p_v2.txt)
(SPDX: `CERN-OHL-P-2.0`). Copyright 2026 Gabriele Bortolato. See [LICENSE](LICENSE). The `verilog-axis` submodule keeps its own license (MIT).
