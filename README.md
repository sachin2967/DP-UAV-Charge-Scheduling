# DP-Based Charge Scheduling for Minimum Energy UAV Missions

## Overview

This repository contains the C++ reference implementation and experimental framework for the manuscript:

**"DP-based Charge Scheduling and Detouring for Minimum Energy RT Fixed-route UAV Missions"**  
Sreyashi Mukherjee, Sachin Yadav, Dipankar Mandal, Arnab Sarkar 

The project implements a Dynamic Programming (DP) framework to optimize UAV charging schedules and detour decisions under real-time mission constraints, with the objective of minimizing total energy consumption.

This repository is intended to support reproducibility of the experimental results reported in the manuscript.

---

## Repository Structure

```
.
├── algo/                     # Core DP solver logic
│   └── uav_solver_recursive.h
│
├── datagen/                  # Mission parameter generation framework
│
├── experiments/              # Scripts for reproducing experimental figures
│   ├── exp_fig3_ablation.cpp
│   ├── exp_fig4ac_tasks_quality.cpp
│   ├── exp_fig4b_payload.cpp
│   ├── exp_fig4d_tasks_runtime.cpp
│   └── exp_fig5_wind.cpp
│
├── results/                  # Generated CSV output data
│
├── running_example.cpp       # Minimal reproducible example from section 4.5 paper
├── paper_running_example.log # Reference output for verification
└── README.md
```

---

## Compilation Requirements

- C++ compiler supporting C++17 (GCC ≥ 7.0 or Clang recommended)
- Linux / macOS / Windows
- No external dependencies (Standard Library only)
- The experiments report process memory: on Windows this is the commit charge used for the
  paper; on Linux it is the virtual size from `/proc/self/statm` (an approximation), and on
  other platforms it is unavailable (`-1`).

Compile using:

```
-std=c++17 -O3
```

---

## Quick Start: Running Example

The file `running_example.cpp` reproduces the step-by-step scheduling example described in the manuscript.

### Compile (from root directory)

```bash
g++ -std=c++17 -O3 running_example.cpp -o run_example
```

### Run

Linux/macOS:
```bash
./run_example
```

Windows:
```bash
run_example.exe
```

### Verification

Compare the program output with `paper_running_example.log`. The headline result must match:

```
Total Energy Cost: 59752525.98 Joules
Total Energy Cost: 16597.92 Wh
```

Note: the current program prints an extended step-by-step DP trace and a different path
listing format than the archived log, and the first detour may name a different charger
when options tie on energy, so compare the total-energy lines rather than diffing the
whole file.

---

## Reproducing Experimental Results

All experimental scripts are located in the `experiments/` directory.

⚠ Always compile from the repository root directory to ensure header paths resolve correctly.

---

### Mapping Between Code and Figures

| Manuscript Figure | Script | Output |
|-------------------|--------|--------|
| Fig. 3 | `experiments/exp_fig3_ablation.cpp` | `results/data_fig3_*.csv` |
| Fig. 4(a,c) | `experiments/exp_fig4ac_tasks_quality.cpp` | `results/data_fig4ac_*.csv` |
| Fig. 4(b) | `experiments/exp_fig4b_payload.cpp` | `results/data_fig4b_*.csv` |
| Fig. 4(d) | `experiments/exp_fig4d_tasks_runtime.cpp` | `results/data_fig4d_*.csv` |
| Fig. 5 | `experiments/exp_fig5_wind.cpp` | `results/data_fig5_*.csv` |

---

### Example: Running Figure 4(a)

Compile:

```bash
g++ -std=c++17 -O3 experiments/exp_fig4ac_tasks_quality.cpp -o exp_tasks
```

Run:

```bash
./exp_tasks
```

CSV results will be generated in the `results/` directory.

Some experiments may require significant runtime depending on hardware.

---

## Core Solver

The Dynamic Programming solver is implemented in:

```
algo/uav_solver_recursive.h
```

The solver computes:

- Optimal charging schedule  
- Detour decisions  
- Total mission energy  
- Feasibility under battery constraints  

---

## Reproducibility Notes

- Use optimization flag `-O3` for consistent performance results.
- Minor runtime variations may occur across hardware platforms.
- Deterministic seeds are used where applicable.

---

## Contact

For implementation questions, please open an issue in this repository so the co-authors can follow the discussion.

## Reuse and attribution

This repository documents work with multiple co-authors. No software license is currently specified. Obtain permission from the authors before reusing or redistributing the implementation, manuscript material or experimental outputs.

