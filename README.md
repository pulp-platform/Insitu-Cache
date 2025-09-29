# 🚀 Insitu-Cache

**Insitu-Cache** is a **non-blocking, high-performance cache architecture** for modern heterogeneous SoCs, developed at ETH Zurich & University of Bologna (as part of PULP platform).  
Its key insight is simple yet powerful: **Re-purpose wasted cache-line space for Write Buffer and MSHR fucntions**.
This “do more with what you already have” approach shrinks area, slashes latency, and keeps bandwidth-hungry accelerators happy.

<p align="center">
  <img src="doc/figures/MotivationOrig.jpg" alt="Motivation of Insitu-Cache"/>
</p>

---

## ✨  Highlights

| 💡 | What makes Insitu-Cache special? |
|---|----------------------------------|
| 🏗️ | **In-situ write buffers & MSHRs** – no extra SRAM macros required. |
| 🔄 | **Efficient Non-blocking** handle more than 10K cache misses. |
| 📐 | **Modular RTL**: drop-in compatible with ARM AXI or TCDM fabrics. |
| 📊 | **Area & energy saving**. |

---

## 🗂️  Repository Layout

<details>
<summary>Click to expand the directory tree</summary>

```text
Insitu-Cache/
├── include/
│   └── insitu_cache/
│       ├── assign.svh
│       └── hash.svh
├── src/
│   ├── cachepool/
│   │   └── cachepool_cache_ctrl.sv
│   ├── coalesce_unit/
│   │   ├── par_coalescer/
│   │   │   ├── non_coalescer.sv
│   │   │   ├── par_coalescer_equal_window.sv
│   │   │   ├── par_coalescer_extend_window.sv
│   │   │   ├── par_coalescer_top.sv
│   │   │   ├── req_coalescer_v2.sv
│   │   │   └── rsp_spliter_v2.sv
│   │   ├── seq_coalescer/
│   │   │   ├── seq_coalescer_multi_req_merger.sv
│   │   │   ├── seq_coalescer_req_merger.sv
│   │   │   └── seq_coalescer_top.sv
│   │   └── write_merger/
│   │       └── write_through_merger.sv
│   ├── dep/
│   │   └── dram_rtl_sim
│   ├── flamingo/
│   │   └── flamingo_spatz_cache_ctrl.sv
│   ├── insitu_cache/
│   │   ├── insitu_cache_core.sv
│   │   ├── insitu_cache_decoder.sv
│   │   ├── insitu_cache_encoder.sv
│   │   ├── insitu_cache_pkg.sv
│   │   ├── insitu_cache_tcdm_wrapper.sv
│   │   ├── insitu_cache_tcdm_wrapper_partitionable_flushable.sv
│   │   └── insitu_cache_top.sv
│   └── utilities/
│       ├── cache_to_axi.sv
│       ├── decouple_channels_adapter.sv
│       ├── decouple_queue_sync.sv
│       ├── dual_port_bank.sv
│       ├── dual_port_rf.sv
│       ├── id_buffer.sv
│       ├── pseudo_dual_port_bank.sv
│       ├── pseudo_dual_port_fifo.sv
│       └── pseudo_dual_port_way.sv
├── test/
│   └── tb_insitu_cache.sv
├── vsim/
│   └── start.tcl
├── Makefile
└── README.md
````

</details>

---

## 🏃‍♂️  Quick-Start

### 🔧 Prerequisites

- This design leverages [`bender`](https://github.com/pulp-platform/bender) for dependency management and automatic generation of compilation scripts.
  - `bender` version >= 0.27.2 is required
- Note: We currently do not offer an open-source simulation setup. Instead, we have utilized `Questasim` for RTL simulation.
- The testbech utilizes [DRAMSys5.0](https://github.com/tukl-msd/DRAMSys) for DRAM Models. For building DRAMSys:
  - `cmake` version >= 3.28.1 is required.
  - `gcc` version >= 11.2.0 is required
  - `g++` version >= 11.2.0 is required

### 🚀 Run Simulation
```bash
# Build Dramsys for insitu-cache testbench
git submodule update --init --recursive
make -C src/dep/dram_rtl_sim/ -j dramsys

# Compile RTL, elaborate, and run in ModelSim/Questa
make vsim
```

> **Tip**
> The supplied `start.tcl` script drops you straight into the GUI with waveforms pre-loaded.

---

## 🛠️  Building Blocks

| Module                                | Role                                                        |
| ------------------------------------- | ----------------------------------------------------------- |
| `insitu_cache_core.sv`                | Core pipeline: tag, data, write-combining, and MSHR FSMs    |
| `par_coalescer_*` / `seq_coalescer_*` | Parallel and sequential request coalescers                  |
| `cachepool_cache_ctrl.sv`             | Dynamic allocation of “pool” lines for write-buffer entries |
| `utilities/`                          | Handy adapters (AXI bridges, dual-port banks, FIFOs)        |

---

## 📜  License

All hardware sources and tool scripts are licensed under the Solderpad Hardware License 0.51 (see `LICENSE`), while figures under the `doc/figures` folder are licensed under the CC-BY-ND license (see `doc/figures/LICENSE`).
Feel free to use, modify, and star ⭐ the repo if you find Insitu-Cache helpful!


# Spatz Cache Wrapper

## Overview

This repository provides the Spatz Cache Wrapper, which is responsible for a specific subsystem in the larger Flamingo architecture. We currently provide the functionality encapsulated within the green block of the diagram below.

![Flamingo L1 Subsystem Diagram](doc/figures/Flamingo_Spatz_Cache.png)

## Initialization

To initialize the environment, source the appropriate shell script:

```bash
source sourceme.sh
```

## Running the Testbench

To run the testbench with the default simulation setup, use the following command:

```bash
make vsim
```

## To-Do List

- [x] First RTL Wrapper of (Coalescer + Insitu-Cache) for Flamingo
- [x] Wrapper TestBench and CI Setup
- [x] Insitu-Cache Hyper-SPM Function
- [x] Insitu-Cache Flush+Invalidation Function
- [x] Add TestBench with Hyper-SPM & Flush+Invalidation Check
- [x] Solve Problem When Configure All SPM
- [x] Test pseudo_dual_port_fifo
- [x] Solve All X in Simulation
- [x] Sanity Check: Parameter Assertion
- [x] Check All FlipFlop in The Design: They Should Be Async Reset
- [x] Check All FSM Has Defualt Return to IDLE
- [x] Connection test on spatz cluster
- [x] Add Bank Access Grant Single and Controller to Delay Cache Bank Access When Conflict with Other Bank Access
- [ ] Frontend Request Isolator for Cache (INIT/FLUSH/INVALID)
- [ ] Optimize Perfromace: Coaleacer Organization & Cache Slices & Tag hashing
- [ ] GF22 Area Estimation
