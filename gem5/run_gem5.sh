#!/bin/bash

build/X86_MESI_Two_Level_Pythia/gem5.debug \
    configs/multicore_gem5/multicore_with_classic_caches/x86-npb-benchmarks.py \
    --benchmark='npb-ep-a' \
    | tee -i fs_run.log