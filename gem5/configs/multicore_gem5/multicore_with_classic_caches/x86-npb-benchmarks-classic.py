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

from typing import Type, Optional

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

import argparse
import time

import m5
from m5.objects import Root
from m5.stats.gem5stats import get_simstat
from m5.util import warn

from gem5.components.boards.x86_board import X86Board
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import (
    ExitEvent,
    Simulator,
)
from gem5.utils.requires import requires

requires(
    isa_required=ISA.X86,
    kvm_required=False,
)

valid_cpu = {
    "TimingSimpleCPU": CPUTypes.TIMING,
    "DerivO3CPU": CPUTypes.O3,
    "AtomicSimpleCPU": CPUTypes.ATOMIC,
    "MinorCPU": CPUTypes.MINOR,
}

valid_prefetcher = {
    "Stride": StridePrefetcher,
    "Scooby": ScoobyPrefetcher,
    "Tagged": TaggedPrefetcher,
    "SPP": SignaturePathPrefetcher,
    "IM": IndirectMemoryPrefetcher,
    "DCPT": DCPTPrefetcher,
}

# Following are the list of benchmark programs for npb.
"""
IS: Integer Sort, random memory access
EP: Embarrassingly Parallel
CG: Conjugate Gradient, irregular memory access and communication
MG: Multi-Grid on a sequence of meshes, long- and short-distance communication, memory intensive
FT: discrete 3D fast Fourier Transform, all-to-all communication
benchmark_choices = ["bt", "cg", "ep", "ft", "is", "lu", "mg", "sp"]

We are restricting classes of NPB to A, B and C as the other classes (D and
F) require main memory size of more than 3 GB. The X86Board is currently
limited to 3 GB of memory. This limitation is explained later.

The resource disk has binaries for class D. However, only `ep` benchmark
works with class D in the current configuration. More information on the
memory footprint for NPB is available at https://arxiv.org/abs/2010.13216

class_choices = ["a", "b", "c"]

The benchmarks that works with Scooby prefetcher: cg


The benchmarks that seem not working with Scooby prefetcher:
is: exception at `assert(flags.isSet(STATIC_DATA|DYNAMIC_DATA));` when multiple cores are used. Fine with single code.
ep: simulation taking forever

"""

parser = argparse.ArgumentParser(
    description="An example configuration script to run the npb benchmarks."
)

npb_suite = obtain_resource("npb-benchmark-suite")
# The only positional argument accepted is the benchmark name in this script.

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

parser.add_argument(
    "--workload",
    type=str,
    required=True,
    help="Input the benchmark program to execute.",
    choices=[workload.get_id() for workload in npb_suite],
)

# Short form: python script.py -n 2
# Long form: python script.py --num-cores 2
# Or use an equal sign: python script.py --num-cores=2
parser.add_argument(
    "-n",
    "--num-cores",
    type=int,
    default=2,
    required=False,
    help="The number of CPU cores to run.",
)

parser.add_argument(
    "--ticks",
    type=int,
    help="Optionally put the maximum number of ticks to execute during the "
    "ROI. It accepts an integer value.",
)

args = parser.parse_args()

print("Number of cores: {}".format(args.num_cores))

# The simulation may fail in the case of `mg` with class C as it uses 3.3 GB
# of memory (more information is available at https://arxiv.org/abs/2010.13216).
# We warn the user here.

if args.workload == "npb-mg-c":
    warn(
        "mg.C uses 3.3 GB of memory. Currently we are simulating 3 GB\
    of main memory in the system."
    )

# The simulation will fail in the case of `ft` with class C. We warn the user
# here.
elif args.workload == "npb-ft-c":
    warn(
        "There is not enough memory for ft.C. Currently we are\
    simulating 3 GB of main memory in the system."
    )

"""
Here we setup the processor. This is a special switchable processor in which
a starting core type and a switch core type must be specified.
Once a configuration is instantiated a user may call `processor.switch()` to
switch from the starting core types to the switch core types.
In this simulation we start with ATOMIC cores to simulate the OS boot, then
switch to the O3 cores for the command we wish to run after boot.

Note that you cannot use atomic CPUs to run the benchmarks because it will lead
to the prefetchers being diabled and thus no stats being collected.
You can either use TIMING or O3 CPU.
"""

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.ATOMIC,
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=args.num_cores,
)

from gem5.components.cachehierarchies.classic.private_l1_private_l2_shared_l3_cache_hierarchy import (
    PrivateL1PrivateL2SharedL3CacheHierarchy,
)

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

# The X86 board only supports 3 GB of main memory.
memory = DualChannelDDR4_2400(size = "3GB")

# Here we setup the board. The X86Board allows for Full-System X86 simulations
board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

"""
Here we set the FS workload, i.e., npb benchmark program.
After simulation has ended you may inspect
`m5out/system.pc.com_1.device` to the stdout, if any.
"""
board.set_workload(obtain_resource(args.workload))

# We need this for long running processes.
m5.disableAllListeners()

# The first exit_event ends with a `workbegin` cause. This means that the
# system started successfully and the execution on the program started.
def handle_workbegin():
    print("Done booting Linux")
    print("Resetting stats at the start of ROI!")

    m5.stats.reset()

    # We have completed up to this step using atomic cpu. Now we switch to timing
    # cpu for detailed simulation.

    # # Next, we need to check if the user passed a value for --ticks. If yes,
    # then we limit out execution to this number of ticks during the ROI.
    # Otherwise, we simulate until the ROI ends.
    processor.switch()
    if args.ticks:
        # schedule an exit event for this amount of ticks in the future.
        # The simulation will then continue.
        m5.scheduleTickExitFromCurrent(args.ticks)
    
    # Indicate that the simulation should continue running
    yield False


# The next exit_event is to simulate the ROI. It should be exited with a cause
# marked by `workend`.


# We exepect that ROI ends with `workend` or `simulate() limit reached`.
def handle_workend():
    print("Dump stats at the end of the ROI!")

    m5.stats.dump()
    #Indicate that the simulation should terminate
    yield True


simulator = Simulator(
    board=board,
    on_exit_event={
        ExitEvent.WORKBEGIN: handle_workbegin(),
        ExitEvent.WORKEND: handle_workend(),
    },
)

# We maintain the wall clock time.
globalStart = time.time()

print("Running the simulation")

# We start the simulation.
simulator.run()

# We need to note that the benchmark is not executed completely till this
# point, but, the ROI has. We collect the essential statistics here before
# resuming the simulation again.

# Simulation is over at this point. We acknowledge that all the simulation
# events were successful.
print("All simulation events were successful.")
# We print the final simulation statistics.

print("Done with the simulation")
print()
print("Performance statistics:")

# manually calculate ROI time if ticks arg is used in case the
# entire ROI wasn't simulated
if args.ticks:
    print(f"Simulated time in ROI (to tick): {args.ticks/ 1e12}s")
else:
    print(f"Simulated time in ROI: {simulator.get_roi_ticks()[0] / 1e12}s")

print(
    f"Ran a total of {simulator.get_current_tick() / 1e12} simulated seconds"
)
print(
    "Total wallclock time: %.2fs, %.2f min"
    % (time.time() - globalStart, (time.time() - globalStart) / 60)
)
