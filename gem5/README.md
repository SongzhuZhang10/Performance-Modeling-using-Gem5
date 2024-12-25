<p align="center">
  <h3 align="center"> Pythia: Performance Modeling using Gem5 for an RL-Based L2 Hardware Prefetcher
  </h3>
</p>

<p align="center">
    <a href="https://github.com/SongzhuZhang10/fun_with_gem5/blob/main/gem5/LICENSE">
        <img alt="GitHub" src="https://img.shields.io/badge/License-MIT-yellow.svg">
    </a>
</p>

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#what-is-pythia">What is Pythia?</a></li>
    <li><a href="#introduction">Introduction</a></li>
    <li><a href="#my-development-process">Development Process</a></li>
    <li><a href="#outcomes">Outcomes</a></li>
    <li><a href="#walkthrough-of-my-deliverables">Walkthrough of My Deliverables</a></li>
    <ul>
      <li><a href="#classic-memory-system">Classic Memory System</a></li>
      <li><a href="#ruby-memory-system"> Ruby Memory System</a></li>
    </ul>
    <li><a href="#python-configuration-files-for-cache-memory-system">Python Configuration Files for Cache Memory System</a></li>
      <ul>
        <li><a href="#simulation-using-classic-memory-system">Simulation using Classic Memory System</a></li>
        <li><a href="#simulation-using-ruby-memory-system">Simulation using Ruby Memory System</a></li>
      </ul>
    </li>
    <li><a href="#methodology">Methodology</a></li>
    <li><a href="#simulation-results-of-npb-benchmark-cg-program-class-a-on-gem5-with-classic-memory">Simulation Results using NPB Benchmark CG Program Class A</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

## What is Pythia?

> [Pythia](https://github.com/CMU-SAFARI/Pythia/tree/master?tab=readme-ov-file) is a hardware-realizable, light-weight data prefetcher that uses reinforcement learning to generate accurate, timely, and system-aware prefetch requests.
> Pythia formulates hardware prefetching as a reinforcement learning task. For every demand request, Pythia observes multiple different types of program context information to take a prefetch decision. For every prefetch decision, Pythia receives a numerical reward that evaluates prefetch quality under the current memory bandwidth utilization. Pythia uses this reward to reinforce the correlation between program context information and prefetch decision to generate highly accurate, timely, and system-aware prefetch requests in the future.

## Introduction
The primary outcome of this project is the performance model of Pythia.

The main objective of this side project is for me to gain hands-on experience in performance modeling for a multi-core computer system using C++ and the [Gem5](https://github.com/gem5/gem5) computer architecture simulator framework.

Originally, the performance model of Pythia was developed using a trace-driven simulator known as ChampSim. This project endeavors to reconstruct Pythia's performance model within an execution-driven simulator known as Gem5, transitioning from its initial simulation environment to a more complex and versatile one.

Much of the code was derived from my interpretation of the referenced paper. For prefetcher design details that are either omitted or ambiguous in the paper, I had to consult the original code used by the authors for architectural exploration. This process facilitated the identification of all necessary design details not explicitly mentioned in the paper.


## My Development Process
0. Acquired a foundational understanding of snooping and directory coherence protocols.

1. Got familiar with the Gem5 framework, especially its classic and ruby memory systems.

2. Comprehended the intricate hardware design and algorithmic abstraction of Pythia presented in the paper.

3. Translated the algorithmic representation of Pythia into a C++ performance model and integrated it into Gem5’s classic and Ruby memory systems. This integration leveraged Gem5’s infrastructure for accurate simulation.

4. Developed Python and shell scripts to configure a multi-core computer system comprising a private L1, private L2, and shared L3 classic memory system, and then ran simulations in Full System mode and System Emulation mode.

5. Conducted simulations under various configurations and benchmarks to assess Pythia's performance relative to other advanced prefetchers in Gem5’s prefetcher library.

## Outcomes

By the conclusion of this project, I have successfully:

* Developed the capability to create a performance model based on its algorithmic abstraction and/or hardware specifications.

* Infusing my own C++  and Python code to an industrial-grade simulation framework like Gem5.

## Walkthrough of My Deliverables
This section outlines the key components of the Pythia prefetcher project, structured within the Gem5 framework. The project is divided into C++ source files for the prefetcher's implementation and Python scripts for configuring the simulation environment. The small changes I made to the Gem5 are omitted here.

### Classic Memory System
The Pythia's implementation under Gem5's classic memory system resides in two files:

*  `scooby.hh`: The header file defining the structure and functionalities of the Pythia prefetcher.

*  `scooby.cc`: The source file containing the implementation details of the Pythia prefetcher.

*  `Prefetcher.py`: The configuration script of this prefetcher.

Location of these files: `src/mem/cache/prefetch/`.

### Ruby Memory System
The Pythia's implementation under Gem5's Ruby memory system resides in two files:

*  `PythiaPrefetcher.hh`: The header file defining the structure and functionalities of the Pythia prefetcher.

*  `PythiaPrefetcher.cc`: The source file containing the implementation details of the Pythia prefetcher.

*  `PythiaPrefetcher.py`: The configuration script of this prefetcher.

Location of these files: `src/mem/ruby/structures/`.

### Python Configuration Files for Cache Memory System
#### Simulation using Classic Memory System
To configure a multi-core computer system that comprises a private L1, private L2, and shared L3 classic memory system with Gem5, the following Python scripts are used:

*  `private_l1_private_l2_shared_l3_cache_hierarchy.py`: Configures the cache hierarchy to simulate an Intel Skylake-like multi-processor computer system under which the original design of Pythia was developed, tested, and assessed.

*  `abstract_three_level_cache_hierarchy.py`: Provides a template  for a three-level cache hierarchy within the Gem5 simulation.

Location of these files: `src/python/gem5/components/cachehierarchies/classic/`.

*  `se_top.py`: Configures the simulation environment for simulation in SE  (System Emulation) mode.

* `x86-npb-benchmarks-classic.py`: Sets up the environment for classic memory system simulation in FS  (Full System) mode, specifically targeting NPB  (NAS Parallel Benchmarks).

Location of these files: `configs/multicore_gem5/multicore_with_classic_caches/`.

* `./run.sh`: Script to simplify and automate the Gem5 build and simulation process.
* `./tests/test-progs/threads/src/threads.cpp`: Multi-threaded C++ test program that runs in SE mode for sanity checking and to provide rapid feedback during the debugging process.

#### Simulation using Ruby Memory System
To configure a multi-core computer system simulation using a private L1,  and shared L2 Ruby memory system with Gem5, the following Python scripts are used:

*  `l1_cache.py`,  `l2_cache.py`,  `simple_pt2pt.py`,  `cache_system_se.py`: Configures the MESI two-level cache hierarchy that is used for simulation in SE mode.

*  `mesi_se_top.py`: Configures the simulation environment for simulation in SE  (System Emulation) mode.

* `x86-npb-benchmarks-ruby.py`: Sets up the environment for Ruby memory system simulation in FS  (Full System) mode, specifically targeting NPB  (NAS Parallel Benchmarks).

Location of these files: `configs/multicore_gem5/mesi_two_level_ruby_caches/`.

### Methodology
In the initial phase of development, I constructed the performance model for the Pythia prefetcher based solely on its description within the referenced paper. However, the performance statistics generated by my model did not align with the outcomes described in the paper. This discrepancy led me to consider that the paper might have omitted some critical design details for the sake of simplicity and brevity.

Fortunately, the authors included a link to the original C++ code utilized in the development of the Pythia prefetcher with ChampSim. This codebase comprised extensive and convoluted C++ code, enabling a high degree of configurability. This configurability was crucial for navigating through numerous architectural design choices for the Pythia prefetcher, allowing for the exploration of various parameter variations to derive the final design of Pythia.

By examining the original C++ code, I was able to infer the missing elements of the Pythia's detailed design that were absent in the paper. This insight enabled me to successfully refine and complete my performance model.

### Simulation Results of NPB Benchmark CG Program Class A on Gem5 with Classic Memory

<div align="center">

**Table 1: Simulated system parameters**

| Component     | Specification |
|:-------------:|:-------------:|
| Core          | Timing Simple CPU |
| L1 I-Cache    | Private, 32KiB, 64B line, 8 way, LRU, 16 MSHRs, 4-cycle round-trip latency |
| L1 D-Cache    | Private, 32KiB, 64B line, 8 way, LRU, 16 MSHRs, 4-cycle round-trip latency, stride prefetcher |
| L2 Cache      | Private, 256KiB, 64B line, 8 way, LRU, 32 MSHRs, 14-cycle round-trip latency, variable prefetchers |
| L3 Cache (LLC)| 2MB/core, 64B line, 16 way, 64MSHRs per LLC bank, 34-cycle round-trip latency |
| Main Memory   | Dual channel DDR4 2400 |

</div>

SPP: Signature Path Prefetcher  
DCPT: Delta Correlating Prediction Tables Prefetcher  

<div align="center">

**Accuracy Results**

</div>

<div align="center">

| L2 Prefetcher | 1 Core    | 2 Cores   | 4 Cores   |
|:-------------:|:---------:|:---------:|:---------:|
| Pythia        | 0.77      | 0.78      | 0.72      |
| SPP           | 0.49      | 0.47      | 0.47      |
| DCPT          | 0.54      | 0.54      | 0.54      |

</div>

<div align="center">

**Coverage Results**

</div>

<div align="center">

| L2 Prefetcher | 1 Core    | 2 Cores   | 4 Cores   |
|:-------------:|:---------:|:---------:|:---------:|
| Pythia        | 0.64      | 0.66      | 0.58      |
| SPP           | 0.52      | 0.51      | 0.50      |
| DCPT          | 0.71      | 0.70      | 0.69      |

</div>

## Contact
Songzhu Zhang - zsz1002@outlook.com
