# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) Songzhu Zhang
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import Type

from m5.objects import (
    BasePrefetcher,
    StridePrefetcher,
    ScoobyPrefetcher,
    TaggedPrefetcher,
    DCPTPrefetcher,
    IndirectMemoryPrefetcher,
    SignaturePathPrefetcher,
    AMPMPrefetcher,
    BOPPrefetcher
)

from gem5.utils.override import *

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import (
    CPUTypes,
    get_cpu_type_from_str,
    get_cpu_types_str_set,
)
from gem5.resources.resource import Resource
from gem5.resources.resource import FileResource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires
from gem5.isas import ISA

import argparse

from m5.util import addToPath
import os, argparse, sys, time

from pprint import pprint

# Needed for debugging with PDB
import pdb


from gem5.components.cachehierarchies.classic.private_l1_private_l2_shared_l3_cache_hierarchy import (
    PrivateL1PrivateL2SharedL3CacheHierarchy,
)

requires(isa_required=ISA.X86)

valid_cpu = {
    "TimingSimpleCPU": CPUTypes.TIMING,
    "DerivO3CPU": CPUTypes.O3,
    "AtomicSimpleCPU": CPUTypes.ATOMIC,
    "MinorCPU": CPUTypes.MINOR
}

valid_prefetcher = {
    "Stride": StridePrefetcher,
    "Scooby": ScoobyPrefetcher,
    "Tagged": TaggedPrefetcher,
    "SPP": SignaturePathPrefetcher,
    "IM": IndirectMemoryPrefetcher,
}

parser = argparse.ArgumentParser(
    description="A gem5 script for running simple binaries in SE mode."
)

parser.add_argument(
    "-l2pf",
    "--l2pf",
    action="store",
    type=str,
    default="Stride",
    required=False,
    choices=valid_prefetcher.keys(),
    help="The L2 prefetcher to be used",
)

# CPU can be atomic, o3, timing, or minor.
parser.add_argument(
    "--cpu",
    action="store",
    type=str,
    #default="DerivO3CPU",
    default="TimingSimpleCPU",
    required=False,
    choices=valid_cpu.keys(),
    help="The CPU type used."
)

parser.add_argument(
    "-n",
    "--num-cores",
    type=int,
    default=1,
    required=False,
    help="The number of CPU cores to run.",
)

args = parser.parse_args()

cache_hierarchy = PrivateL1PrivateL2SharedL3CacheHierarchy(
    l1i_size="32KiB",
    l1i_assoc=8,
    l1i_mshrs=8,
    l1i_tgts_per_mshr=4,
    #l1i_PrefetcherCls=StridePrefetcher,

    l1d_size="32KiB",
    l1d_assoc=8,
    l1d_mshrs=16,
    l1d_tgts_per_mshr=4,
    l1d_PrefetcherCls=StridePrefetcher,

    l2_size="256KiB",
    l2_assoc=8,
    l2_mshrs=32,
    l2_tgts_per_mshr=8,
    l2_PrefetcherCls=valid_prefetcher[args.l2pf],

    l3_size="2MiB",
    l3_assoc=16,
    l3_mshrs=args.num_cores*64,
    l3_tgts_per_mshr=16,
    #l3_PrefetcherCls=SignaturePathPrefetcher
)

# Declare the rest of the components.
#memory = SingleChannelDDR3_1600("8GiB")
memory = DualChannelDDR4_2400("3GB")

processor = SimpleProcessor(cpu_type=valid_cpu[args.cpu], isa=ISA.X86, num_cores=args.num_cores)

# Add them to the motherboard.
motherboard = SimpleBoard(
    clk_freq="3GHz", processor=processor, memory=memory, cache_hierarchy=cache_hierarchy
)

# os.path.dirname returns the parent directory of the specified path.
# config_path = the absolute path of the directory containing the config file
config_path = os.path.dirname(os.path.abspath(__file__))
print("Directory path of the config file: ", config_path)

# config_root = the absolute path of the parent directory of `config_path`
config_root = os.path.dirname(config_path)
print("Config root: ", config_root)

# Set the workload.
binary_path = os.path.join(
    config_root,
    "../../",
    "tests/test-progs/threads/bin/x86/linux/threads"
)

# More `Abstract Resource` subclasses can be found in src/python/gem5/resources/resource.py
binary = FileResource(binary_path)

motherboard.set_se_binary_workload(binary)

# Setup the Simulator and run the simulation.
simulator = Simulator(board=motherboard)

print("Start running simulation ...")
globalStart = time.time()
simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)

print(
    "Ran a total of", simulator.get_current_tick() / 1e12, "simulated seconds"
)

print("Total wallclock time: %.2fs, %.2f min" % \
            (time.time()-globalStart, (time.time()-globalStart)/60))