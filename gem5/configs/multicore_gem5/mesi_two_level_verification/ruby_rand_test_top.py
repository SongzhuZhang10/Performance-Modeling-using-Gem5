# Copyright (c) 2023 Songzhu Zhang
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
"""
******************************************************************************
*                                                                            *
* Ruby Random Test config for MESI Directory Chip Multi-Processor (CMP).     *
*                                                                            *
******************************************************************************

Author: Songzhu Zhang
Date: 2023/04/03

Description:
CMP refers to a type of multi-core processor design where multiple processing
units (cores) share a single physical chip, as opposed to multiple physical
chips connected via an interconnect. In this design, each core has access to
its own L1 cache, but multiple cores may share a higher-level cache or memory.

In a CMP system, when a processor wants to read or write to a memory location,
it first checks its own L1 cache. If the data is not in the cache, it sends a
request to the MESI directory to check if another processor has the data. If
another processor has the data, the MESI directory will ensure that the data
is transferred to the requesting processor's cache and all other copies of the
data are invalidated to maintain cache coherency.

Usage:
build/X86_mesi/gem5.opt configs/multicore_gem5/mesi_two_level_verification/ruby_rand_test_top.py [options]=[arguments]

Options:
-h, --help     Show help message and exit
-v, --verbose  Enable verbose mode

Examples:
build/X86_mesi/gem5.opt configs/multicore_gem5/mesi_two_level_verification/ruby_rand_test_top.py --max_mem_reqs=-1
build/X86_mesi/gem5.opt configs/multicore_gem5/mesi_two_level_verification/ruby_rand_test_top.py --max_mem_reqs=1200
build/X86_mesi/gem5.opt configs/multicore_gem5/mesi_two_level_verification/ruby_rand_test_top.py --max_mem_reqs=-1 --network_class=GarnetPt2Pt
build/X86_mesi/gem5.opt --debug-flags=RubySlicc configs/multicore_gem5/mesi_two_level_verification/ruby_rand_test_top.py
build/X86_mesi/gem5.opt --debug-flags=ProtocolTrace configs/multicore_gem5/mesi_two_level_verification/ruby_rand_test_top.py
"""

import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath
import os, argparse, sys, time
from m5.stats.gem5stats import get_simstat

from pprint import pprint
from test_cache_system import TestCacheSystem
#import pdb; pdb.set_trace()

if buildEnv["PROTOCOL"] != "MESI_Two_Level":
    fatal("This system assumes MESI_Two_Level!")

# Sets up a command-line argument parser using the argparse module
parser = argparse.ArgumentParser()

# Add a new command-line argument to specify the maximum number of memory
# requests (loads and stores) that the RubyTester should issue before
# terminating the simulation.
parser.add_argument(
    "--max_mem_reqs",
    metavar="N",
    default=10,
    help="Stop after N loads"
)

# The -f and --wakeup_freq are both options for the same command-line argument.
# You can use either of them to specify the wakeup_freq parameter.
parser.add_argument(
    "-f",
    "--wakeup_freq",
    metavar="N",
    default=10,
    help="Wakeup every N cycles"
)

parser.add_argument(
    "--sys_clock",
    action="store",
    type=str,
    default="4GHz",
    help="Top-level clock for blocks running at system speed"
)

parser.add_argument(
    "--network_class",
    action="store",
    type=str,
    default="GarnetMesh_XY",
    help="Garnet topology for on-chip interconnection network",
    choices=["GarnetPt2Pt", "SimplePt2Pt", "GarnetMesh"]
)

parser.add_argument(
    "--sys_voltage",
    action="store",
    type=str,
    default="1.0V",
    help="Top-level voltage for blocks running at system power supply",
)

parser.add_argument("-n", "--num_cpus", type=int, default=2)

parser.add_argument("--l1i_size", type=str, default="32kB")
parser.add_argument("--l1i_assoc", type=int, default=8)

parser.add_argument("--l1d_size", type=str, default="32kB")
parser.add_argument("--l1d_assoc", type=int, default=8)

parser.add_argument("--l2_size", type=str, default="256kB")
parser.add_argument("--l2_assoc", type=int, default=16)
parser.add_argument("--num_l2Caches", type=int, default=1)

"""
Parse the arguments and store the results in the `args` variable. This allows
the values of the command-line arguments to be easily accessed and used later
in the configuration script.
"""
args = parser.parse_args()

# Set the default cache size and associativity to be very small to encourage
# races between requests and writebacks.

args.l1d_size = "256B"
args.l1d_assoc = 2

args.l1i_size = "256B"
args.l1i_assoc = 2

args.l2_size = "512B"
args.l2_assoc = 2

print("max_mem_reqs: ", args.max_mem_reqs)
print("num_cpus: ", args.num_cpus)
print("num_l2Caches: ", args.num_l2Caches)
print("l1d_size: ", args.l1d_size)
print("l1d_assoc: ", args.l1d_assoc)
print("l1i_size: ", args.l1i_size)
print("l1i_assoc: ", args.l1i_assoc)
print("l2_size: ", args.l2_size)
print("l2_assoc: ", args.l2_assoc)
print("network_class: ", args.network_class)
system = System()

# Create a top-level voltage domain and clock domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = args.sys_clock
system.clk_domain.voltage_domain = system.voltage_domain

# Any memory requests made by the simulated system will be directed to the
# memory range specified by `args.mem_size`.
system.mem_mode = "timing"  # Use timing accesses
system.mem_ranges = [AddrRange("1GB")]  # Create an address range

# Create a DDR3 memory controller and connect it to the membus
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]

# Create the ruby random tester
system.tester = RubyTester(
    # NOTE: Because the flush type Ruby request type is not supported by the SLICC
    # implementation of the cache coherence protocol, one must set this to
    # `Flase`. Or, a panic message will be issued when running the simulation.
    check_flush=False,
    checks_to_complete=args.max_mem_reqs,
    wakeup_frequency=args.wakeup_freq,
    num_cpus=args.num_cpus
)

print(f"Cache line size of the system: {system.cache_line_size}")

system.caches = TestCacheSystem()
system.caches.setup(
    options=args,
    system=system,
    network_class=args.network_class
)

# -----------------------
#   Run Simulation
# -----------------------

# set up the root SimObject and start the simulation
root = Root(full_system=False, system=system)

# instantiate all of the objects we've created above
m5.instantiate()

globalStart = time.time()

print("Start running simulation ...")
exit_event = m5.simulate()

if exit_event.getCause() == "Ruby Tester completed" or \
    exit_event.getCause() == "user interrupt received":

    print("Exiting @ tick %i because %s" % (m5.curTick(), exit_event.getCause()))
    print("Dump stats at the end of the ROI...")

    m5.stats.dump()
else:
    print("Unexpected termination of simulation while ROI was being executed!")
    print(
        "Exiting @ tick {} because {}.".format(
            m5.curTick(),
            exit_event.getCause()
        )
    )
    exit(-1)

gem5stats = get_simstat(root)

# Simulation is over at this point. We acknowledge that all the simulation
# events were successful.
print("All simulation events were successful.")

print("Done with the simulation")
print()
print("Performance statistics:")

print("Ran a total of", m5.curTick()/1e12, "simulated seconds")
print("Total wallclock time: %.2fs, %.2f min" % \
            (time.time()-globalStart, (time.time()-globalStart)/60))