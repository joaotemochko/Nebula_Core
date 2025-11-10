# Nebula Core (RV64G+S)

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](...)
[![License](https://img.shields.io/badge/license-MIT-blue)](...)
[![Language](https://img.shields.io/badge/language-SystemVerilog-purple)](...)

The Nebula Core is an 8-stage, dual-issue, **In-Order** RISC-V core designed for high efficiency. It functions as the "LITTLE" core within a cluster, optimized to work as a scalable worker for the **STU (Speculative Threading Unit)**.

Nebula is designed to be fully Linux-capable, implementing the **RV64G+S** (IMAFD + Supervisor) instruction set, complete with an integrated **MMU (TLB/PTW)**.

---

## 🏛️ STU (Speculative Threading Unit) Integration

Nebula is not a standalone core; it is designed to be a "worker" managed by the `stu_top` module. It implements the full STU "contract" and is the ideal target for both Level 1 and Level 2 speculation.

### Level 1: Conservative Parallelism (Block Expansion)

Nebula is the primary target for the STU's Level 1 (safe) mode.
* The core monitors the `l1_dispatch_valid_in` signal.
* When activated, the `STAGE_FETCH` is bypassed, and the 2 "safe" instructions from `l1_dispatch_data_in` are injected directly into the `STAGE_DECODE` pipeline for execution. This allows the STU to use multiple idle Nebula cores to execute a single safe block in parallel.

### Level 2: Optimistic Speculation (Thread Worker)

Nebula also functions as an efficient, low-power worker for the STU's high-risk/high-reward Level 2 (optimistic) mode.
* **Context Management:** The core exposes a "backdoor" read/write port to its `regfile` and `fregfile` (via `core_copy_*` signals). This allows the `stu_context_manager` to copy the Master's context (GPRs/FPRs) into it before a fork.
* **Forking:** When `l2_spec_start_in` is asserted for this core's `HART_ID`, it creates a checkpoint (`shadow_regfile`, `shadow_pc`), flushes its pipeline, and begins executing at the new `l2_spec_pc_in`.
* **Tracking:** The Memory stage outputs the **Physical Address (PA)** (post-MMU) via `core_mem_pa_out` for the `stu_memory_tracker` to snoop.
* **Status Reporting:** The core reports `spec_exception_out` on a `STAGE_TRAP` (e.g., Page Fault) and `spec_task_done_out` (by detecting a loop-return PC) to the `stu_validator`.
* **Verdict:** The core obeys the `SQUASH` (restoring its checkpoint) and `COMMIT` (discarding its checkpoint) signals.

---

## 🔧 Pipeline & Features

* **ISA:** RV64G (IMAFD) + S (Supervisor Mode).
* **Pipeline:** 8-stage, In-Order, dual-issue (superscalar).
    1.  `STAGE_RESET`
    2.  `STAGE_FETCH` (Bypassed by L1)
    3.  `STAGE_DECODE`
    4.  `STAGE_ISSUE`
    5.  `STAGE_EXECUTE`
    6.  `STAGE_MEMORY` (Post-MMU)
    7.  `STAGE_WRITEBACK`
    8.  `STAGE_TRAP`
* **MMU:** Fully integrated Sv39 MMU with `tlb_associative` (TLB) and `ptw_sv39_full` (Page Table Walker) instances.
* **Forwarding:** Full data forwarding logic to minimize stalls.
* **Atomics:** Multi-cycle FSM for AMO operations (Extension 'A').

## 🚧 Status

**WIP (Work in Progress).** The core logic is defined. Next steps include instantiating and connecting multi-cycle MDU (Multiply/Divide) and FPU (Floating Point) units to complete the RV64G implementation.
